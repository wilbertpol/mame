// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/**********************************************************************

    TI Explorer System Interface Board (SIB).

**********************************************************************/

#include "emu.h"
#include "explorer_sib.h"
#include "speaker.h"


DEFINE_DEVICE_TYPE(SIB, explorer_sib_device, "explorer_sib", "TI Explorer System Interface Board (2243145-0001A)")


namespace {

static constexpr u16 VIDEO_RAM_SIZE = 0x8000; // Guestimate
static constexpr u16 VIDEO_RAM_MASK = VIDEO_RAM_SIZE - 1;
static constexpr u16 SCREEN_WIDTH = 1024;
static constexpr u16 SCREEN_HEIGHT = 808;

// The raster, taken from the values the system software loads into the CRT9007
// at cold boot. Table 4-13 lists them in its "Data (Hex)" column, TI's own
// copy of the list is SIB-CRT-Init-Sequence-List in ucode/lroy-qdev.lisp, and
// a boot logs exactly the same 22 bytes:
//
//   R00 = 2A   42 characters per horizontal period, 32 pixels each = 1344
//   R01 = 1F   32 characters per data row                          = 1024
//   R02 = 07    7 character times of horizontal delay              =  224
//   R07 = 64  101 visible data rows per frame ...
//   R08 = 67   ... of 8 scan lines each                            =  808
//   R08/R09   842 scan lines per frame
//
// A "character" here is one 32-bit word of the bit map, so the display is
// 1024 x 808 - which is what Figure 4-11 shows (line 0 at FSE80000, line 807
// at FSE99380) and what the system software believes: cold-load-stream.lisp
// initialises the cold-load stream with :WIDTH 1024. :HEIGHT 808.
//
// 1344 x 842 pixels sixty times a second needs 67.89888 MHz, and the crystal
// on the board is the 67.889 MHz oscillator labelled in Figure 1-11 of the
// Field Maintenance manual (paragraph 4.4.10.3 calls it "the 67.8989-megahertz
// pixel clock ... one bit every 14.72 nanoseconds"). Driven from the crystal
// the frame comes out at 59.99 Hz, i.e. the 16.67 ms of paragraph 4.4.10.7.
static constexpr XTAL PIXEL_CLOCK = 67.889_MHz_XTAL;
static constexpr u16 CHARACTER_WIDTH = 32;   // one bit-map word per character
static constexpr u16 HTOTAL = 42 * CHARACTER_WIDTH;  // R00
static constexpr u16 HBEND = 7 * CHARACTER_WIDTH;    // R02, horizontal delay
static constexpr u16 VTOTAL = 842;                   // R08/R09
// Only 34 of those 842 scan lines are left once the 808 visible ones are
// accounted for, and neither R04 (vertical sync width, 24 = 36 lines) nor R05
// (vertical delay, 25 = 37 lines) fits inside 34 - so how those 34 lines split
// into front and back porch is not something the documentation to hand pins
// down, and they all go ahead of the display here. Nothing depends on where
// they sit; the totals are what matter. (Horizontally the same arithmetic does
// close: 7 delay + 32 displayed + 3 = 42 characters.)
static constexpr u16 VBEND = VTOTAL - SCREEN_HEIGHT;

// Real hardware bit assignments (2243145-0001A SI General Description, page 4-15,
// Figure 4-4): bit 0 (Reset) is write-only and always reads 0 - a momentary strobe,
// never latched into the readable register. Bit 2 (SI board test LED) and bit 10
// (PSU overtemperature warning) are read-only, hardware-driven - software writes to
// them have no effect. Only bits 1/3 (NuBus master enable, NuBus test) and 8/9
// (monitor/chassis self-test LEDs) and the reserved-but-R/W bits 4-7 are storable.
static constexpr u32 CONFIGURATION_REGISTER_WRITABLE_MASK = 0x3fa;

u8 compute_parity(u8 data) { data ^= data >> 4; data ^= data >> 2; data ^= data >> 1; return data & 1; }

} // anonymous namespace


explorer_sib_device::explorer_sib_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock) :
	device_t(mconfig, SIB, tag, owner, clock),
	device_ti_nubus_card_interface(mconfig, *this),
	m_screen(*this, "screen"),
	m_crt9007(*this, "crt9007"),
	m_i8251(*this, "i8251"),
	m_keyboard(*this, "keyboard"),
	m_rtc(*this, "rtc"),
	m_pit(*this, "pit"),
	m_usart_clock(*this, "usart_clock"),
	m_sn76496(*this, "sn76496"),
	m_nvram(*this, "nvram"),
	m_video_ram(*this, "video_ram", VIDEO_RAM_SIZE * sizeof(u32), ENDIANNESS_BIG),
	m_nv_ram(*this,"nv_ram", 0x2000, ENDIANNESS_LITTLE)
{
}


void explorer_sib_device::device_start()
{
	nubus().install_map(*this, &explorer_sib_device::nubus_map);
	nubus().install_local_bus_map(*this, &explorer_sib_device::local_bus_map);
	m_nvram->set_base(m_nv_ram.begin(), m_nv_ram.bytes());

	m_i8251->write_cts(0);
	// DSR starts deasserted, matching TXD's own idle-high default (see
	// i8251_txd_w(), which keeps DSR wired to TXD from the first transition
	// onward).
	m_i8251->write_dsr(1);

	save_item(NAME(m_configuration_register));
	m_configuration_register = 0;
	save_item(NAME(m_event_vector));
	save_item(NAME(m_attribute_register));
	save_item(NAME(m_mask_register));
	save_item(NAME(m_operation_register));
	save_item(NAME(m_mouse_y_position));
	save_item(NAME(m_mouse_x_position));
	save_item(NAME(m_interrupt_diag_control));
	save_item(NAME(m_monitor_control));
	save_item(NAME(m_diagnostic_data));
	save_item(NAME(m_voice_data_register));
	save_item(NAME(m_printer_data));
	save_item(NAME(m_sound_control));
	save_item(NAME(m_speech_register));
	save_item(NAME(m_usart_rxrdy));
	save_item(NAME(m_usart_txrdy));
}


void explorer_sib_device::nubus_map(address_map &map)
{
	map.unmap_value_high();

	// 0xfa0030 - read
	// 0xe00058 - write 00 - interrupts?
	// 0xf00040 - read
	// 0xf2000c - write fe00
	// 0xf2000c - read
	// 0xf20014 - write 9f, bf, df, ff
	// 0xf80044 - write 00
	// 0xf80044 - read
	// 0xf9000c - write 30 / b0
	//
	// e00000 - graphics-and-bit-map-control-base
	// f00000 - event-generator-base
	// f10000 - printer-port-base
	// f20000 - mouse-registers-base
	// f80000 - real-time-clock-base
	// f90000 - timers-base
	// fa0000 - non-volatile-ram-base
	// fb0000 - rs232c-port-base
	// fc0000 - keyboard-base
	// fe0000 - configuration-rom-base
	//
	map(0x00000000, 0x00ffffff).lrw32(NAME([this] (offs_t offset) {
		if (!machine().side_effects_disabled())
		{
			printf("SIB unmapped read offset %08x / %08x\n", offset, offset << 2);
//			machine().debug_break();
		}
		return u32(0xffffffff);
	}), NAME([] (offs_t offset, u32 data) {
		printf("SIB unmapped write offset %08x / %08x, data %08x\n", offset, offset << 2, data);
//		machine().debug_break();
	}));

	graphics_bitmap_map(map);
	event_generator_map(map);
	printer_map(map);
	mouse_map(map);
	rtc_map(map);
	timers_map(map);
	nvram_map(map);
	rs232c_map(map);
	map(0x00fc0000, 0x00fc0007).lrw32(NAME([this] (offs_t offset) {
		if (offset == 1 && BIT(m_interrupt_diag_control, 2) && !BIT(m_interrupt_diag_control, 3))
			return diagnostic_loopback_value();
		return u32(m_i8251->read(offset ^ 1));
	}), NAME([this] (offs_t offset, u32 data) {
		m_i8251->write(offset ^ 1, u8(data));
	}));

	configuration_rom_map(map);
}


void explorer_sib_device::graphics_bitmap_map(address_map &map)
{
	// e00000 - graphics-and-bit-map-control-base
	//
	// e00000 - Graphics-Char-Per-Horiz-Period
	// e00004 - Graphics-Char-Per-Data-Row
	// e00008 - Graphics-Horiz-Delay
	// e0000c - Graphics-Horiz-Sync-Width
	// e00010 - Graphics-Vertical-Sync-Width
	// e00014 - Graphics-Vertical-Delay
	// e00018 - Graphics-Skew
	// e0001c - Graphics-Visible-Data-Rows-Per-Frame
	// e00020 - Graphics-Scan-Lines
	// e00024 - Graphics-Scan-Lines-Per-Frame-LS
	// e00028 - Graphics-Dma-Control
	// e0002c - Graphics-Operation-Control
	// e00030 - Graphics-Table-Start-Register-LS
	// e00034 - Graphics-Table-Start-Register-MS
	// e00038 - Graphics-Aux-Address-Register-1-LS
	// e0003c - Graphics-Aux-Address-Register-1-MS
	// e00040 - Graphics-Seq-Break-Register-1
	// e00044 - Graphics-Data-Row-Start
	// e00048 - Graphics-Data-Row-End
	// e0004c - Graphics-Aux-Address-Register-2-LS
	// e00050 - Graphics-Aux-Address-Register-2-MS
	// e00054 - Graphics-Start-Command
	// e00058 - Graphics-Reset-Command
	// e0005c - Graphics-Offset
	// e00060 - Graphics-Cursor-Row
	// e00064 - Graphics-Cursor-Column
	// e00068 - Graphics-Status-Register / Graphics-Interrupt-Enable
	// e0006c - Graphics-Light-Pen-Row
	// e00070 - Graphics-Light-Pen-Column
	// e0007c - Graphics-Char-Per-Horiz-Period
	// e00080 - Graphics-Attribute-Register
	// e00084 - Graphics-Mask-Register
	// e00088 - Graphics-Alu-Register
	// e00098 - Graphics-Video-Test-Register

	// All of these are registers of the CRT9007 video processor and controller
	// ("video processor controller 9007", Figure 1-11 of the Field Maintenance
	// manual), one per 32-bit slot in byte lane 0 - so the chip's register
	// number is bits 6-2 of the address. The chip has a 6-bit register space
	// and the board only supplies five bits of it, which is why Table 4-13
	// documents the same addresses twice: writes land in the write bank
	// (R00-R1F) and reads in the read bank 0x20 higher, so e00068 is R1A
	// "Interrupt enable" written and R3A "Status" read, and e00060/e00064 are
	// the cursor registers written (R18/R19) and read back (R38/R39).
	map(0x00e00000, 0x00e0007f).rw(FUNC(explorer_sib_device::crtc_r), FUNC(explorer_sib_device::crtc_w)).umask32(0x000000ff);
	// Readable as well as writable, like the adjacent mask and ALU registers.
	// Meroko's sib.c reads it back as sib_video_attr; installed write-only here,
	// reads fell through to the slot's unmapped handler instead.
	map(0x00e00080, 0x00e00083).lrw32(NAME([this] () {
		return m_attribute_register;
	}), NAME([this] (offs_t offset, u32 data, u32 mem_mask) {
		COMBINE_DATA(&m_attribute_register);
	}));
	map(0x00e00084, 0x00e00087).lrw32(NAME([this] () {
		return m_mask_register;
	}), NAME([this] (offs_t offset, u32 data, u32 mem_mask) {
		COMBINE_DATA(&m_mask_register);
	}));
	map(0x00e00088, 0x00e0008b).lrw32(NAME([this] () {
		return m_operation_register;
	}), NAME([this] (offs_t offset, u32 data, u32 mem_mask) {
		COMBINE_DATA(&m_operation_register);
	}));
	map(0x00e00098, 0x00e0009b).lw32(NAME([] (u32 data) {
		printf("Graphics-Video-Test-Register write %08x\n", data);
	}));
	// e9ffff?
	// e80000 - e993ff - displayed
	map(0x00e80000, 0x00e9ffff).rw(FUNC(explorer_sib_device::video_ram_r), FUNC(explorer_sib_device::video_ram_w));

	map(0x00ec0000, 0x00edffff).rw(FUNC(explorer_sib_device::video_ram_r), FUNC(explorer_sib_device::video_ram_rmw_w));
}


u8 explorer_sib_device::crtc_r(offs_t offset)
{
	// The map is 32 words wide, so offset is already the 0-0x1f register
	// number. Bit 5 is not in the address at all: the chip has a six-bit
	// register space and the board decodes five, selecting the read bank by
	// the direction of the cycle instead. That is why Table 4-13 lists some
	// addresses twice, and why the 0x20 has to be ORed in here - reads of
	// e00054/e00058 would otherwise hit the chip's read-side Start and Reset
	// commands at R15/R16.
	//
	// Paragraph 4.4.10.7's "bit 7 is set if an interrupt is pending
	// (hexadecimal C0); all bits are clear (hexadecimal 00) if no interrupt is
	// pending" reads like a two-valued register, but the C0 is just the chip
	// describing itself: a pending vertical-retrace interrupt sets the retrace
	// bit (0x40) alongside the interrupt-pending bit (0x80). It is only the 00
	// that is loose, since reading clears the pending bit alone and leaves 0x40
	// standing until the next vertical sync.
	//
	// Nothing minds, so the value is passed through as the device produces it.
	// The band reads this register from its retrace handler and nowhere else -
	// always while pending, never while idle - and TI's field spec for it is
	// %%SIB-TV-Status-Interrupt-Pending #o0701 in ucode/lroy-qdev.lisp, one bit
	// wide at bit 7. In practice it always reads C1: bit 0 is "frame timer
	// occurred", which crt9007_device sets whether or not the frame-timer
	// interrupt is enabled, and which the software masks off.
	return m_crt9007->read(0x20 | offset);
}


void explorer_sib_device::crtc_w(offs_t offset, u8 data)
{
	m_crt9007->write(offset, data);
}


void explorer_sib_device::local_bus_map(address_map &map)
{
	map(0x00e80000, 0x00e9ffff).rw(FUNC(explorer_sib_device::video_ram_r), FUNC(explorer_sib_device::video_ram_w));
	map(0x00ec0000, 0x00edffff).rw(FUNC(explorer_sib_device::video_ram_r), FUNC(explorer_sib_device::video_ram_rmw_w));
}


u32 explorer_sib_device::video_ram_r(offs_t offset)
{
	return m_video_ram[offset & VIDEO_RAM_MASK];
}


void explorer_sib_device::video_ram_w(offs_t offset, u32 data, u32 mem_mask)
{
	COMBINE_DATA(&m_video_ram[offset & VIDEO_RAM_MASK]);
}


void explorer_sib_device::video_ram_rmw_w(offs_t offset, u32 data, u32 mem_mask)
{
	u32 const d = m_video_ram[offset & VIDEO_RAM_MASK];
	u32 const s = data;
	u32 result;
	switch (m_operation_register & 0xf)
	{
	case 0x0: result = 0; break;              // CLEAR
	case 0x1: result = ~(d | s); break;       // D NOR S
	case 0x2: result = s & ~d; break;         // S AND D-
	case 0x3: result = ~d; break;             // D-
	case 0x4: result = d & ~s; break;         // S- AND D
	case 0x5: result = ~s; break;             // S-
	case 0x6: result = d ^ s; break;          // D XOR S
	case 0x7: result = ~(d & s); break;       // D NAND S
	case 0x8: result = d & s; break;          // D AND S
	case 0x9: result = ~(d ^ s); break;       // D XNOR S
	case 0xa: result = s; break;              // NOP (S)
	case 0xb: result = s | ~d; break;         // S OR D-
	case 0xc: result = d; break;              // D
	case 0xd: result = d | ~s; break;         // D OR S-
	case 0xe: result = d | s; break;          // D OR S
	default:  result = 0xffffffff; break;     // SET (0xf)
	}
	u32 const masked_result = (result & ~m_mask_register) | (d & m_mask_register);
	m_video_ram[offset & VIDEO_RAM_MASK] = (d & ~mem_mask) | (masked_result & mem_mask);
}


void explorer_sib_device::event_generator_map(address_map &map)
{
	//
	// f00000 - Event-Real-Time-Clock
	// f00004 - Event-Short-Interval-Timer
	// f00008 - Event-Long-Interval-Timer
	// f0000c - Event-RS232C-Port
	// f00010 - Event-Printer-Port
	// f00014 - Event-Graphics-Controller
	// f00018 - Event-Keyboard
	// f0001c - Event-Power-Supply
	// f00020 - Event-Keyboard-Special-Chord-Reset
	// f00024 - Event-Mouse-Motion
	// f00028 - Event-Mouse-Keyswitch
	// f0002c - Event-Voice-Data
	// f00030 - Event-Sound-Data
	// f00034 - fiber optic data link warning
	// f00038 - Event-Power-Failure
	// f0003c - Event-Power-Failure
	// f00040 - configuration register

	map(0x00f00000, 0x00f0003f).rw(FUNC(explorer_sib_device::event_vector_r), FUNC(explorer_sib_device::event_vector_w));

	map(0x00f00040, 0x00f00043).lrw32(NAME([this] {
		if (!machine().side_effects_disabled())
			printf("Configuration-Register read\n");
		return m_configuration_register;
	}), NAME([this] (u32 data) {
		printf("Configuration-Register write %08x\n", data);
		m_configuration_register = data & CONFIGURATION_REGISTER_WRITABLE_MASK;
	}));

	// TODO
}

u32 explorer_sib_device::event_vector_r(offs_t offset)
{
	return m_event_vector[offset];
}

void explorer_sib_device::event_vector_w(offs_t offset, u32 data, u32 mem_mask)
{
	COMBINE_DATA(&m_event_vector[offset]);
}

void explorer_sib_device::post_event(int cause)
{
	// Configuration register bit 1, "NuBus master enable" (Figure 4-4) - the doc
	// is explicit that the event generator must not act until this is set, since
	// it should only be enabled once the host has finished programming the event
	// addresses (section 4.4.4/4.4.5.2's own init-order requirement).
	if (!BIT(m_configuration_register, 1))
		return;

	nubus().space().write_byte(m_event_vector[cause], 0xff);
}

void explorer_sib_device::pit_out2_w(int state)
{
	// Mode 0 (interrupt on terminal count): the output is forced low the instant
	// the control word is written, then goes high (and stays high) once the count
	// reaches zero - only that rising edge is the real "interval elapsed" event.
	if (state)
		post_event(2); // "Interval timer (long)", Table 4-4
}

void explorer_sib_device::rtc_irq_w(int state)
{
	// explorer_rtc_device raises this only on the rising edge of an
	// otherwise-clear interrupt-status register (any of its eight sources -
	// Table 4-7), matching pit_out2_w() above - this device just didn't
	// have anything wired to the event generator before, so none of its own
	// interrupts (including D0 Compare) ever reached the CPU.
	if (state)
		post_event(0); // "Real-time clock", Table 4-4
}

void explorer_sib_device::crtc_int_w(int state)
{
	// The CRT9007's INT pin. Paragraph 4.4.10.7: the controller "can generate
	// an interrupt at the start of each vertical retrace... once every 16.67
	// milliseconds immediately after the CRT has completed a full video display
	// refresh", enabled by bit 6 of the byte written to e00068 (R1A), and
	// "once the interrupt occurs, an event is generated at the address loaded
	// into the event generator register file".
	//
	// Only the assertion posts an event. The line stays asserted until the
	// status register is read, so a retrace arriving while one is still pending
	// produces no fresh edge - which is the paragraph's "the interrupt must be
	// cleared before another interrupt is generated".
	if (state)
		post_event(5); // "Graphics controller", Table 4-4 (event vector f00014)
}

// Table 4-4 gives the keyboard USART one interrupt cause, "Ready to
// transmit/receive" - but the two readies have to be edge-detected separately
// rather than ORed into a single line. TxRDY idles asserted on an enabled,
// empty transmitter, so an ORed line sits permanently high and a later RxRDY
// can never produce an edge on it; the keystroke's interrupt is swallowed.
// (Observed exactly that way first: the byte reached the USART at t=45.076s and
// no event was posted.) Paragraph 4.4.5 has the generator posting one event per
// interrupt received, so each ready asserting is its own interrupt.
void explorer_sib_device::usart_rxrdy_w(int state)
{
	bool const asserted = bool(state);
	if (asserted && !m_usart_rxrdy)
		post_event(6); // "Keyboard USART", Table 4-4 (event vector f00018)
	m_usart_rxrdy = asserted;
}

void explorer_sib_device::usart_txrdy_w(int state)
{
	bool const asserted = bool(state);
	if (asserted && !m_usart_txrdy)
		post_event(6);
	m_usart_txrdy = asserted;
}


void explorer_sib_device::printer_map(address_map &map)
{
	// f10000 - printer-port-base
	//
	// f10000 - Printer-Data-Register (Register 0, Table 4-10)
	// f10004 - Printer-Control/Status-Register (Register 1, Table 4-11)
	//
	map(0x00f10000, 0x00f10003).lrw32(NAME([this] {
		return m_printer_data;
	}), NAME([this] (u32 data) {
		m_printer_data = data & 0xff;
	}));
	map(0x00f10004, 0x00f10007).lrw32(NAME([] {
		return u32(0x0c);
	}), NAME([] (u32 data) {
		(void)data;
	}));
}


void explorer_sib_device::mouse_map(address_map &map)
{
	// f20000 - mouse-registers-base
	//
	// f20000 - Mouse-Y-Position-Register
	// f20004 - Mouse-X-Position-Register
	// f20008 - Mouse-Motion-And-Keyswitch-Register
	// f2000c - Mouse-Control-Register
	// f20010 - Mouse-Diagnostic-Data-Register
	// f20014 - Mouse-Sound-Control-Register
	// f20018 - Mouse-Speech-Register
	// f2001c - Mouse-Voice-Register

	// f20005 - sn76496?
	// 9f - 10011111 - 001 - tone 1 attenuation - off
	// bf - 10111111 - 011 - tone 2 attenuation - off
	// df - 11011111 - 101 - tone 3 attenuation - off
	// ff - 11111111 - 111 - noise attenuation - off
	// TODO

	map(0x00f20000, 0x00f20003).lrw32(NAME([this] {
		return m_mouse_y_position;
	}), NAME([this] (u32 data) {
		m_mouse_y_position = data & 0xffff;
	}));
	map(0x00f20004, 0x00f20007).lrw32(NAME([this] {
		return m_mouse_x_position;
	}), NAME([this] (u32 data) {
		m_mouse_x_position = data & 0xffff;
	}));
	map(0x00f20008, 0x00f2000b).lr32(NAME([this] {
		if (diagnostic_loopback_active())
			return diagnostic_loopback_value();
		if (BIT(m_interrupt_diag_control, 1))
			return m_diagnostic_data & 0xff;
		return u32(0xffffffff);
	}));
	map(0x00f2000c, 0x00f2000f).lrw32(NAME([this] {
		return (m_interrupt_diag_control & 0xff) | ((m_monitor_control & 0xf) << 8);
	}), NAME([this] (u32 data) {
		m_interrupt_diag_control = data & 0xff;
		m_monitor_control = (data >> 8) & 0xf;
	}));
	map(0x00f20010, 0x00f20013).lrw32(NAME([this] {
		return (m_diagnostic_data & 0xff) | (u32(compute_parity(m_diagnostic_data & 0xff)) << 8);
	}), NAME([this] (u32 data) {
		m_diagnostic_data = data & 0x1ff;
		if (BIT(m_interrupt_diag_control, 0) && BIT(data, 8))
			m_voice_data_register = data & 0xff;
	}));
	map(0x00f20014, 0x00f20017).lrw32(NAME([this] {
		return (m_sound_control & 0xff) | (u32(compute_parity(m_sound_control & 0xff) ^ 1) << 8);
	}), NAME([this] (u32 data) {
		m_sound_control = data & 0xff;
		m_sn76496->write(u8(data));
	}));
	map(0x00f20018, 0x00f2001b).lrw32(NAME([this] {
		return (m_speech_register & 0xff) | (u32(compute_parity(m_speech_register & 0xff) ^ 1) << 8);
	}), NAME([this] (u32 data) {
		m_speech_register = data & 0xff;
	}));
	map(0x00f2001c, 0x00f2001f).lr32(NAME([this] {
		return m_voice_data_register & 0xff;
	}));
}


void explorer_sib_device::rtc_map(address_map &map)
{
	// f80000 - real-time-clock-base
	//
	// f80000 - Rtclock-100-Microseconds-Counter
	// f80004 - Rtclock-10-And-100-Millisecond-Counter
	// f80008 - Rtclock-Seconds-Counter
	// f8000c - Rtclock-Minutes-Counter
	// f80010 - Rtclock-Hours-Counter
	// f80014 - Rtclock-Day-Of-Week-Counter
	// f80018 - Rtclock-Day-Of-Month-Counter
	// f8001c - Rtclock-Month-Counter
	// f80020 - Rtclock-RAM-100-Microseconds-Counter
	// f80024 - Rtclock-RAM-10-And-100-Millisecond-Counter
	// f80028 - Rtclock-RAM-Seconds-Counter
	// f8002c - Rtclock-RAM-Minutes-Counter
	// f80030 - Rtclock-RAM-Hours-Counter
	// f80034 - Rtclock-RAM-Day-Of-Week-Counter
	// f80038 - Rtclock-RAM-Day-Of-Month-Counter
	// f8003c - Rtclock-RAM-Month-Counter
	// f80040 - Rtclock-Interrupt-Status-Register
	// f80044 - Rtclock-Interrupt-Control-Register
	// f80048 - Rtclock-Counters-Reset
	// f8004c - Rtclock-Ram-Reset
	// f80050 - Rtclock-Read-Status-Bit
	// f80054 - Rtclock-Go-Command
	// f80058 - Rtclock-Standby-Interrupt
	// f8005c - Rtclock-Test-Mode

	map(0x00f80000, 0x00f8005f).m(m_rtc, FUNC(explorer_rtc_device::map));
}


void explorer_sib_device::timers_map(address_map &map)
{
	// f90000 - timers-base
	//
	// f90000 - Timers-Read-Counter-0 / Timers-Load-Counter-0 (short-term interval timer)
	// f90004 - Timers-Read-Counter-1 / Timers-Load-Counter-1 (square-wave rate generator)
	// f90008 - Timers-Read-Counter-2 / Timers-Load-Counter-2 (long-term interval timer)
	// f9000c - Timers-Write-Mode-Control (Figure 4-6: SC1 SC0 RL1 RL0 M2 M1 M0 BCD)
	//
	// The SI General Description never names this chip - unlike the i8251,
	// MC68000, an NCR part, and the Z8530, which are all explicitly
	// identified elsewhere in the doc. But the documented control-byte
	// format, the Mode 0 "output forced low the instant the control word
	// is written" and "one full CLK edge pair before a load completes"
	// quirks (both specific real Intel 8253 behaviors, not just generic
	// PIT-shaped ones), and the exact power-up example (Figure 4-7:
	// 0x30/0xB0 to FSF9000C, matching the firmware's own boot sequence)
	// all line up precisely with a genuine 8253 - used here as a
	// well-evidenced best fit, not a confirmed part number.
	// pit8253_device::read/write expect the real 8253's own byte-adjacent register
	// spacing (offset 0-3, its A1/A0 address lines) - SIB's NuBus registers are
	// word-spaced (F90000/04/08/0C) like everything else on this board, so the
	// raw byte offset needs dividing by 4 first (same reason the i8251 wiring
	// above uses an explicit wrapper rather than installing it directly).
	// Installed as a dword-wide (.lrw32) handler, not byte-wide - matching the
	// i8251 wiring above. raven_cpu_device::write_unmapped_byte() issues a
	// masked DWORD write, not a true standalone byte access; a byte-wide
	// (.lrw8) install doesn't respect that mask and gets called once per byte
	// lane in the dword regardless, feeding three spurious 0x00 writes for
	// every real one - which corrupts pit8253_device's two-byte LSB-then-MSB
	// counter loads (confirmed live: counter 1 ended up loaded with 0,
	// wrapping to 65536, instead of the real 1000). Offset is already a
	// dword index here (0-3, matching the register order directly), unlike
	// pit8253_device's own read/write which expect the real chip's
	// byte-adjacent addressing - no manual shift needed either way now.
	// The band programs counters 0 and 2 for mode 0 (control bytes 30/B0), never
	// loads a count into either, and then repeatedly issues the counter-0 latch
	// command (control byte 00, Figure 4-6 RL=00) and reads the two latched
	// bytes - i.e. it reads counter 0 as though it were a free-running
	// microsecond clock. Per 4.4.8.1 an unloaded counter does not count, so
	// those reads correctly return a constant; Meroko instead free-runs the
	// counter down from 0xFFFF. Tested both ways: making counter 0 free-run
	// here produces a byte-identical boot and screen, so this difference is not
	// what the boot is waiting on.
	map(0x00f90000, 0x00f9000f).lrw32(NAME([this] (offs_t offset) {
		return u32(m_pit->read(offset));
	}), NAME([this] (offs_t offset, u32 data) {
		m_pit->write(offset, u8(data));
	}));
}


void explorer_sib_device::nvram_map(address_map &map)
{
	// fa0000 - non-volatile-ram-base
	map(0x00fa0000, 0x00fa1fff).lrw32(NAME([this] (offs_t offset) {
		if (!machine().side_effects_disabled())
			printf("NVRam read %08x\n", offset);
		return u32(m_nv_ram[offset]);
	}), NAME([this] (offs_t offset, u32 data) {
		m_nv_ram[offset] = data & 0xff;
		printf("NVRam write %08x, %08x\n", offset, data);
	}));
}


void explorer_sib_device::rs232c_map(address_map &map)
{
	// fb0000 - rs232c-port-base
	//
	// fb0000 - RS232C-Channel-B-Status / RS232C-Channel-B-Pointer
	// fb0008 - RS232C-Channel-A-Status / RS232C-Channel-A-Pointer
	// fb000c - RS232C-Channel-A-Receive-Buffer / RS232C-Channel-A-Transmit-Buffer
	// fb0010 - RS232C-Interrupt-Acknowledge-Address

	// TODO
}


void explorer_sib_device::configuration_rom_map(address_map &map)
{
	// fe0000 - configuration-rom-base
	map(0xff8000, 0xffffff).rom().region("sib_config", 0);
}


void explorer_sib_device::i8251_txd_w(int state)
{
	// The diagnostic loopback path and the real keyboard both drive the
	// i8251's RXD line; only one should be connected at a time.
	if (BIT(m_interrupt_diag_control, 3))
		m_i8251->write_rxd(state);
	else
		m_keyboard->rxd_w(state);

	// TXD is also hard-wired back into the i8251's own DSR input (matches
	// Meroko's sib.c: the keyboard USART's status-read comment documents DSR
	// as "always high... EXCEPT WITH BREAK", and its code reads DSR high
	// exactly while SEND-BREAK is commanded - which is exactly when TXD is
	// held low). Real hardware has no separate BRK output pin to route for
	// this, so TXD (which already reflects the break state) is the only
	// signal this loopback trace could plausibly be wired from.
	m_i8251->write_dsr(state);
}

void explorer_sib_device::keyboard_txd_w(int state)
{
	// Mirrors i8251_txd_w()'s own gating in the other direction: on real
	// hardware, diagnostic loopback mode physically disconnects the
	// fiber-optic keyboard link from the i8251's RXD, so the keyboard's own
	// TXD must not be allowed to drive it either while bit 3 is set - only
	// the SIB->keyboard direction was gated before, not this one.
	if (!BIT(m_interrupt_diag_control, 3))
		m_i8251->write_rxd(state);
}


u32 explorer_sib_device::screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	const u32 black = 0x000000;
	const u32 white = 0xffffff;

	// Video attributes register, Figure 4-12. Bit 0 blanks the display outright
	// ("1 = blank video, 0 = video enable"); bit 1 selects polarity - normal (0)
	// lights a pixel whose bit-mapped memory bit is 1, reverse (1) lights the
	// pixel whose bit is 0. The band runs the display in reverse video, so
	// ignoring this bit showed the whole screen inverted.
	if (BIT(m_attribute_register, 0))
	{
		bitmap.fill(black, cliprect);
		return 0;
	}
	const u32 invert = BIT(m_attribute_register, 1) ? 0xffffffff : 0;

	// The raster has a blanking interval in front of the display on both axes,
	// so bit-map line 0 pixel 0 sits at the top left of the visible area rather
	// than at (0, 0) of the screen.
	const rectangle &visarea = screen.visible_area();

	for (int y = cliprect.top(); y <= cliprect.bottom(); y++)
	{
		const u32 line_start = (y - visarea.top()) * (SCREEN_WIDTH / 32);

		for (int x = 0; x < (SCREEN_WIDTH / 32); x++)
		{
			const u32 d = m_video_ram[line_start + x] ^ invert;
			const int xs = visarea.left() + x * 32;

			for (int i = 0; i < 32; i++)
			{
				bitmap.pix(y, xs + i) = BIT(d, i) ? white : black;
			}
		}
	}
	return 0;
}


void explorer_sib_device::device_add_mconfig(machine_config &config)
{
	// 1024 x 808 out of a 1344 x 842 raster clocked from the board's 67.889 MHz
	// oscillator - see the derivation beside PIXEL_CLOCK above.
	SCREEN(config, m_screen);
	m_screen->set_raw(PIXEL_CLOCK, HTOTAL, HBEND, HBEND + SCREEN_WIDTH, VTOTAL, VBEND, VBEND + SCREEN_HEIGHT);
	m_screen->set_screen_update(FUNC(explorer_sib_device::screen_update));

	// The CRT9007 is clocked by the character clock, one thirty-second of the
	// pixel clock, since a character on this board is a 32-bit bit-map word.
	// Only the interrupt output is wired: the board takes the chip's sync and
	// blanking signals for the monitor and fibre-optic link, but its cursor,
	// light-pen and row-buffer DMA outputs go nowhere ("other CRT controller
	// functions suggested by the register names in Table 4-13 are not
	// functional due to hardware constraints").
	CRT9007(config, m_crt9007, PIXEL_CLOCK / CHARACTER_WIDTH);
	m_crt9007->set_screen(m_screen);
	m_crt9007->set_character_width(CHARACTER_WIDTH);
	m_crt9007->int_callback().set(FUNC(explorer_sib_device::crtc_int_w));

	I8251(config, m_i8251);
	m_i8251->txd_handler().set(FUNC(explorer_sib_device::i8251_txd_w));
	m_i8251->rxrdy_handler().set(FUNC(explorer_sib_device::usart_rxrdy_w));
	m_i8251->txrdy_handler().set(FUNC(explorer_sib_device::usart_txrdy_w));

	EXPLORER_KEYBOARD(config, m_keyboard);
	m_keyboard->txd_handler().set(FUNC(explorer_sib_device::keyboard_txd_w));

	EXPLORER_RTC(config, m_rtc);
	m_rtc->irq_handler().set(FUNC(explorer_sib_device::rtc_irq_w));

	// Counters 0/1 driven by "a 1-megahertz clock derived from the NuBus
	// clock" (section 4.4.8, 10MHz NuBus CLK- per section 4.4.1.2 - the
	// doc's own worked example, Figure 4-8, treats it as an exact 1MHz/
	// 1us period with no rounding). Counter 2 (the long-term interval
	// counter) has no fixed input clock at all - it's driven by counter
	// 1's own programmable square-wave output.
	PIT8253(config, m_pit);
	m_pit->set_clk<0>(1000000.0);
	m_pit->set_clk<1>(1000000.0);
	m_pit->out_handler<1>().set(m_pit, FUNC(pit8253_device::write_clk2));
	m_pit->out_handler<2>().set(FUNC(explorer_sib_device::pit_out2_w));

	CLOCK(config, m_usart_clock, 153600);
	m_usart_clock->signal_handler().set(m_i8251, FUNC(i8251_device::write_rxc));
	m_usart_clock->signal_handler().append(m_i8251, FUNC(i8251_device::write_txc));

	NVRAM(config, "nvram", nvram_device::DEFAULT_ALL_0);

	SPEAKER(config, "speaker").front_center();
	SN76496(config, "sn76496", 1'500'000).add_route(ALL_OUTPUTS, "speaker", 0.0); // Exact model and input frequency unknown, noise
}


ROM_START(sib)
	ROM_REGION32_BE(0x8000, "sib_config", ROMREGION_ERASE00)
	ROMX_LOAD("2236662_sib.bin", 0x003, 0x2000, CRC(3f1fc829) SHA1(f16d9d9b6d8e51282fd835e2cb716cb173b3eb39), ROM_SKIP(3))
ROM_END

const tiny_rom_entry *explorer_sib_device::device_rom_region() const
{
	return ROM_NAME(sib);
}
