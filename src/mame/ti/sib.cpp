// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/**********************************************************************

    TI Explorer System Interface Board (SIB).

**********************************************************************/

#include "emu.h"
#include "sib.h"
#include "speaker.h"


DEFINE_DEVICE_TYPE(SIB, sib_device, "sib", "TI Explorer System Interface Board")


namespace {

static constexpr u16 VIDEO_RAM_SIZE = 0x8000; // Guestimate
static constexpr u16 VIDEO_RAM_MASK = VIDEO_RAM_SIZE - 1;
static constexpr u16 SCREEN_WIDTH = 1024;

// Real hardware bit assignments (2243145-0001A SI General Description, page 4-15,
// Figure 4-4): bit 0 (Reset) is write-only and always reads 0 - a momentary strobe,
// never latched into the readable register. Bit 2 (SI board test LED) and bit 10
// (PSU overtemperature warning) are read-only, hardware-driven - software writes to
// them have no effect. Only bits 1/3 (NuBus master enable, NuBus test) and 8/9
// (monitor/chassis self-test LEDs) and the reserved-but-R/W bits 4-7 are storable.
static constexpr u32 CONFIGURATION_REGISTER_WRITABLE_MASK = 0x3fa;

u8 compute_parity(u8 data) { data ^= data >> 4; data ^= data >> 2; data ^= data >> 1; return data & 1; }

} // anonymous namespace


sib_device::sib_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock) :
	device_t(mconfig, SIB, tag, owner, clock),
	device_ti_nubus_card_interface(mconfig, *this),
	m_screen(*this, "screen"),
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


void sib_device::device_start()
{
	nubus().install_map(*this, &sib_device::nubus_map);
	nubus().install_local_bus_map(*this, &sib_device::local_bus_map);
	m_nvram->set_base(m_nv_ram.begin(), m_nv_ram.bytes());

	m_i8251->write_cts(0);
	m_i8251->write_dsr(1);

	save_item(NAME(m_configuration_register));
	m_configuration_register = 0;
	save_item(NAME(m_event_vector));
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
}


void sib_device::nubus_map(address_map &map)
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


void sib_device::graphics_bitmap_map(address_map &map)
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

	map(0x00e00000, 0x00e00003).lw32(NAME([] (u32 data) {
		printf("Graphics-Char-Per-Horiz-Period write %08x\n", data);
	}));
	map(0x00e00004, 0x00e00007).lw32(NAME([] (u32 data) {
		printf("Graphics-Char-Per-Data-Row write %08x\n", data);
	}));
	map(0x00e00008, 0x00e0000b).lw32(NAME([] (u32 data) {
		printf("Graphics-Horiz-Delay write %08x\n", data);
	}));
	map(0x00e0000c, 0x00e0000f).lw32(NAME([] (u32 data) {
		printf("Graphics-Horiz-Sync-Width write %08x\n", data);
	}));
	map(0x00e00010, 0x00e00013).lw32(NAME([] (u32 data) {
		printf("Graphics-Vertical-Sync-Width write %08x\n", data);
	}));
	map(0x00e00014, 0x00e00017).lw32(NAME([] (u32 data) {
		printf("Graphics-Vertical-Delay write %08x\n", data);
	}));
	map(0x00e00018, 0x00e0001b).lw32(NAME([] (u32 data) {
		printf("Graphics-Skew write %08x\n", data);
	}));
	map(0x00e0001c, 0x00e0001f).lw32(NAME([] (u32 data) {
		printf("Graphics-Visible-Data-Rows-Per-Frame write %08x\n", data);
	}));
	map(0x00e00020, 0x00e00023).lw32(NAME([] (u32 data) {
		printf("Graphics-Scan-Lines write %08x\n", data);
	}));
	map(0x00e00024, 0x00e00027).lw32(NAME([] (u32 data) {
		printf("Graphics-Scan-Lines-Per-Frame-LS write %08x\n", data);
	}));
	map(0x00e00028, 0x00e0002b).lw32(NAME([] (u32 data) {
		printf("Graphics-Dma-Control write %08x\n", data);
	}));
	map(0x00e0002c, 0x00e0002f).lw32(NAME([] (u32 data) {
		printf("Graphics-Operation-Control write %08x\n", data);
	}));
	map(0x00e00030, 0x00e00033).lw32(NAME([] (u32 data) {
		printf("Graphics-Table-Start-Register-LS write %08x\n", data);
	}));
	map(0x00e00034, 0x00e00037).lw32(NAME([] (u32 data) {
		printf("Graphics-Table-Start-Register-MS write %08x\n", data);
	}));
	map(0x00e00038, 0x00e0003b).lw32(NAME([] (u32 data) {
		printf("Graphics-Aux-Address-Register-1-LS write %08x\n", data);
	}));
	map(0x00e0003c, 0x00e0003f).lw32(NAME([] (u32 data) {
		printf("Graphics-Aux-Address-Register-1-MS write %08x\n", data);
	}));
	map(0x00e00040, 0x00e00043).lw32(NAME([] (u32 data) {
		printf("Graphics-Seq-Break-Register-1 write %08x\n", data);
	}));
	map(0x00e00044, 0x00e00047).lw32(NAME([] (u32 data) {
		printf("Graphics-Data-Row-Start write %08x\n", data);
	}));
	map(0x00e00048, 0x00e0004b).lw32(NAME([] (u32 data) {
		printf("Graphics-Data-Row-End write %08x\n", data);
	}));
	map(0x00e0004c, 0x00e0004f).lw32(NAME([] (u32 data) {
		printf("Graphics-Aux-Address-Register-2-LS write %08x\n", data);
	}));
	map(0x00e00050, 0x00e00053).lw32(NAME([] (u32 data) {
		printf("Graphics-Aux-Address-Register-2-MS write %08x\n", data);
	}));
	map(0x00e00054, 0x00e00057).lw32(NAME([] (u32 data) {
		printf("Graphics-Start-Command write %08x\n", data);
	}));
	map(0x00e00058, 0x00e0005b).lw32(NAME([] (u32 data) {
		printf("Graphics-Reset-Command write %08x\n", data);
	}));
	map(0x00e0005c, 0x00e0005f).lw32(NAME([] (u32 data) {
		printf("Graphics-Offset write %08x\n", data);
	}));
	map(0x00e00060, 0x00e00063).lw32(NAME([] (u32 data) {
		printf("Graphics-Cursor-Row write %08x\n", data);
	}));
	map(0x00e00064, 0x00e00067).lw32(NAME([] (u32 data) {
		printf("Graphics-Cursor-Column write %08x\n", data);
	}));
	map(0x00e00068, 0x00e0006b).lrw32(NAME([this] {
		if (!machine().side_effects_disabled())
			printf("Graphics-Status-Register read\n");
		return u32(0);
	}), NAME([] (u32 data) {
		printf("Graphics-Interrupt-Enable write %08x\n", data);
	}));
	map(0x00e0006c, 0x00e0006f).lw32(NAME([] (u32 data) {
		printf("Graphics-Light-Pen-Row write %08x\n", data);
	}));
	map(0x00e00070, 0x00e00073).lw32(NAME([] (u32 data) {
		printf("Graphics-Light-Pen-Column write %08x\n", data);
	}));
	map(0x00e0007c, 0x00e0007f).lw32(NAME([] (u32 data) {
		printf("Graphics-Char-Per-Horiz-Period write %08x\n", data);
	}));
	map(0x00e00080, 0x00e00083).lw32(NAME([] (u32 data) {
		printf("Graphics-Attribute-Register write %08x\n", data);
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
	map(0x00e80000, 0x00e9ffff).rw(FUNC(sib_device::video_ram_r), FUNC(sib_device::video_ram_w));

	map(0x00ec0000, 0x00edffff).rw(FUNC(sib_device::video_ram_r), FUNC(sib_device::video_ram_rmw_w));
}


void sib_device::local_bus_map(address_map &map)
{
	map(0x00e80000, 0x00e9ffff).rw(FUNC(sib_device::video_ram_r), FUNC(sib_device::video_ram_w));
	map(0x00ec0000, 0x00edffff).rw(FUNC(sib_device::video_ram_r), FUNC(sib_device::video_ram_rmw_w));
}


u32 sib_device::video_ram_r(offs_t offset)
{
	return m_video_ram[offset & VIDEO_RAM_MASK];
}


void sib_device::video_ram_w(offs_t offset, u32 data, u32 mem_mask)
{
	COMBINE_DATA(&m_video_ram[offset & VIDEO_RAM_MASK]);
}


void sib_device::video_ram_rmw_w(offs_t offset, u32 data, u32 mem_mask)
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


void sib_device::event_generator_map(address_map &map)
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

	map(0x00f00000, 0x00f0003f).rw(FUNC(sib_device::event_vector_r), FUNC(sib_device::event_vector_w));

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

u32 sib_device::event_vector_r(offs_t offset)
{
	return m_event_vector[offset];
}

void sib_device::event_vector_w(offs_t offset, u32 data, u32 mem_mask)
{
	COMBINE_DATA(&m_event_vector[offset]);
}

void sib_device::post_event(int cause)
{
	// Configuration register bit 1, "NuBus master enable" (Figure 4-4) - the doc
	// is explicit that the event generator must not act until this is set, since
	// it should only be enabled once the host has finished programming the event
	// addresses (section 4.4.4/4.4.5.2's own init-order requirement).
	if (!BIT(m_configuration_register, 1))
		return;

	nubus().space().write_byte(m_event_vector[cause], 0xff);
}

void sib_device::pit_out2_w(int state)
{
	// Mode 0 (interrupt on terminal count): the output is forced low the instant
	// the control word is written, then goes high (and stays high) once the count
	// reaches zero - only that rising edge is the real "interval elapsed" event.
	if (state)
		post_event(2); // "Interval timer (long)", Table 4-4
}


void sib_device::printer_map(address_map &map)
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


void sib_device::mouse_map(address_map &map)
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


void sib_device::rtc_map(address_map &map)
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


void sib_device::timers_map(address_map &map)
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
	map(0x00f90000, 0x00f9000f).lrw32(NAME([this] (offs_t offset) {
		return u32(m_pit->read(offset));
	}), NAME([this] (offs_t offset, u32 data) {
		m_pit->write(offset, u8(data));
	}));
}


void sib_device::nvram_map(address_map &map)
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


void sib_device::rs232c_map(address_map &map)
{
	// fb0000 - rs232c-port-base
	//
	// fb0000 - RS232C-Channel-B-Status / RS232C-Channel-B-Pointer
	// fb0008 - RS232C-Channel-A-Status / RS232C-Channel-A-Pointer
	// fb000c - RS232C-Channel-A-Receive-Buffer / RS232C-Channel-A-Transmit-Buffer
	// fb0010 - RS232C-Interrupt-Acknowledge-Address

	// TODO
}


void sib_device::configuration_rom_map(address_map &map)
{
	// fe0000 - configuration-rom-base
	map(0xff8000, 0xffffff).rom().region("sib_config", 0);
}


void sib_device::i8251_txd_w(int state)
{
	// The diagnostic loopback path and the real keyboard both drive the
	// i8251's RXD line; only one should be connected at a time.
	if (BIT(m_interrupt_diag_control, 3))
		m_i8251->write_rxd(state);
	else
		m_keyboard->rxd_w(state);
}


u32 sib_device::screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	const u32 black = 0x000000;
	const u32 white = 0xffffff;

	for (int y = cliprect.top(); y <= cliprect.bottom(); y++)
	{
		const u16 line_start = y * (SCREEN_WIDTH / 32);

		for (int x = 0; x < (SCREEN_WIDTH / 32); x++)
		{
			const u32 d = m_video_ram[line_start + x];
			const u32 xs = x * 32;

			for (int i = 0; i < 32; i++)
			{
				bitmap.pix(y, xs + i) = BIT(d, i) ? white : black;
			}
		}
	}
	return 0;
}


void sib_device::device_add_mconfig(machine_config &config)
{
	// System documents mention 1024x808 pixels
	SCREEN(config, m_screen);
	m_screen->set_refresh_hz(60);
	m_screen->set_size(1024, 1024); // TODO
	m_screen->set_visarea(0, 1024-1, 0, 1024-1); // TODO
	m_screen->set_screen_update(FUNC(sib_device::screen_update));

	I8251(config, m_i8251);
	m_i8251->txd_handler().set(FUNC(sib_device::i8251_txd_w));

	EXPLORER_KEYBOARD(config, m_keyboard);
	m_keyboard->txd_handler().set(m_i8251, FUNC(i8251_device::write_rxd));

	EXPLORER_RTC(config, m_rtc);

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
	m_pit->out_handler<2>().set(FUNC(sib_device::pit_out2_w));

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

const tiny_rom_entry *sib_device::device_rom_region() const
{
	return ROM_NAME(sib);
}
