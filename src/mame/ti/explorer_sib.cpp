// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/**********************************************************************

    TI Explorer System Interface Board (SIB).

Board references found:
- 2236590 (early board)
- 2236645
- 2243145


There is no schematic or detailed parts list known of the SIB board,
so some chips in the device map are guessed from software accessing
these chips:
- pit8253: There is only a "programmable interval time" mentioned in the
    documentation; the registers and bits mentioned are a 1-on-1 mapping
	with an 8253.
- sn76496: No direct mention of this chip, but two manuals describe it without
    naming it and both descriptions are a 1-on-1 mapping with an sn76496. The
    SI manual gives the programming side (4.4.11.6 and Figure 4-17, the sound
    control register). The generator itself is in the monitor rather than on
    this board, so it is the Explorer Display Unit General Description that
    describes the part, in paragraph 4.6: "three separate tone generators, each
    with a programmable frequency divider and a programmable output
    attenuator", a programmable 10-stage register per tone, a noise generator
    producing white or periodic noise, an internal summing junction, and the
    2.048-MHz clock used in device_add_mconfig().

**********************************************************************/

#include "emu.h"
#include "explorer_sib.h"
#include "speaker.h"

#define LOG_EVENT (1U << 1)
#define LOG_NVRAM (1U << 2)

//#define VERBOSE (0)
#define VERBOSE (LOG_EVENT | LOG_NVRAM)
#include "logmacro.h"


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
// The frame total is the one value that is not simply a register, because R08
// carries two unrelated fields at once (crt9007.cpp's SCAN_LINES_PER_FRAME is
// ((reg[8] << 3) & 0x0700) | reg[9]). R08 = 0x67 = 0110_0111 splits as
//
//   bits 4:0 = 00111  scan lines per data row, plus one        =   8
//   bits 7:5 =   011  the top three bits of the frame total    = 768
//
// and R09 = 0x4A = 74 supplies the low eight, so 768 + 74 = 842. Reading R08
// as "8 scan lines per row" alone leaves the 842 looking unsourced.
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
static constexpr u32 PIXEL_CLOCK = 67'889'000;
static constexpr u16 CHARACTER_WIDTH = 32;   // one bit-map word per character
static constexpr u16 HTOTAL = 42 * CHARACTER_WIDTH;  // R00
static constexpr u16 HBEND = 7 * CHARACTER_WIDTH;    // R02, horizontal delay
static constexpr u16 VTOTAL = 842;                   // R08/R09
// Only 34 of those 842 scan lines are left once the 808 visible ones are
// accounted for, and the vertical numbers TI programs do not fit in them - not
// merely "the split is undocumented", but the arithmetic does not close at all:
//
//   R05 = 25, vertical delay. crt9007.cpp counts this in scan lines ahead of
//   the display (VERTICAL_DELAY is reg[5] - 1 = 36, used as the top of the
//   display in m_vlt_bottom), so 36 + 808 = 844, two lines past the 842 the
//   frame is supposed to have.
//
//   R04 = 24, vertical sync width = 36 lines, which also exceeds 34 on its own.
//
// Horizontally the same sum does close - 7 delay + 32 displayed + 3 = 42
// characters - so this is specific to the vertical axis. Either the 9007's
// vertical delay is not in plain scan lines on this part, or the real board's
// vertical timing was slack enough that a two-line overrun did not matter.
// Until that is settled all 34 lines go ahead of the display here. Nothing in
// the driver depends on where they sit; the totals are what drive set_raw().
static constexpr u16 VBEND = VTOTAL - SCREEN_HEIGHT;

// Configuration register bit assignments (2243145-0001A SI General Description,
// page 4-15, Figure 4-4). Bits 4-7 are "reserved (read/write)" and bits 11-15 are
// not assigned and always read 0.
static constexpr u32 CONFIG_NUBUS_MASTER_ENABLE = 0x0002;
static constexpr u32 CONFIG_SI_BOARD_TEST_LED = 0x0004;   // the board's own red self-test fault LED
static constexpr u32 CONFIG_NUBUS_TEST = 0x0008;          // "not used on SI board"
static constexpr u32 CONFIG_RESERVED = 0x00f0;            // bits 4-7, read/write, no function
static constexpr u32 CONFIG_MONITOR_TEST_LED = 0x0100;    // the yellow monitor fault LED
static constexpr u32 CONFIG_CHASSIS_TEST_LED = 0x0200;    // front panel fault indicator, see below

// Two bits are deliberately not storable. Bit 0 is a momentary strobe -
// "Writing a 1 to this bit resets the entire board except for the NuBus
// interface" - and is never latched into the readable register; that board reset
// is not modelled. Bit 10, power supply overtemperature, is read-only and driven
// by the supply, and there is no supply here to overheat.
//
// Bit 2 is the one place Figure 4-4 has to be overruled: the figure labels it
// "(read only)", but paragraph 4.4.4 on the very next page says "The host
// processor must control configuration register bits 02, 08, and 09 because the SI
// board does not have processing capability for independent self-test and
// light-emitting diode (LED) test lamp control", and the firmware does exactly
// that - it writes bit 2 set when the slot 5 self-test starts and clear when the
// board passes. A read-only bit 2 would make the red LED unextinguishable.
static constexpr u32 CONFIGURATION_REGISTER_WRITABLE_MASK =
		CONFIG_NUBUS_MASTER_ENABLE | CONFIG_SI_BOARD_TEST_LED | CONFIG_NUBUS_TEST |
		CONFIG_RESERVED | CONFIG_MONITOR_TEST_LED | CONFIG_CHASSIS_TEST_LED;

// Paragraph 4.4.4, of bits 02, 08 and 09: "All of these LED indicators light at
// power-up, and the processor must extinguish each of these LEDs at the successful
// completion of the applicable self-test." Field Maintenance Table 1-1 shows the
// same thing from the outside - step 1 "All fault LEDs go on", step 10 "System
// interface red and yellow LEDs go off" as SLOT 5 PASSED appears.
//
// The chassis LED is left out of that: the same paragraph says "The chassis test
// LED is not implemented in the 7-slot chassis" and "current versions of the
// Explorer do not have a front panel fault indicator", so there is no lamp of it
// to light here - and, consistent with that, the firmware never writes bit 9 set.
static constexpr u32 CONFIGURATION_REGISTER_POWER_UP = CONFIG_SI_BOARD_TEST_LED | CONFIG_MONITOR_TEST_LED;

// Table 4-4, "Event Causes and Register File Storage Locations" (book 4-17): the
// sixteen conditions the event generator polls, in the order it indexes them off
// f00000 in steps of 4. Names follow TI's own Device and Cause columns.
//
// Index 13 is where this board differs from its predecessor, and the table's
// footnote is the reason TI's kernel/micro-time.lisp disagrees with this list:
// "The fiber-optic link warning is only available on part number 2236645-0001.
// On earlier SI boards, such as part number 2236590-0001, there are three
// power-failure warning interrupts and no fiber-optic link warning." The Lisp
// table has Event-Power-Failure at 13 and stops there, i.e. it describes the
// earlier board; 2236645-0001 is the one modelled here (see the board references
// at the top of this file), so 13 is the fiber-optic warning.
enum : int
{
	EVENT_REAL_TIME_CLOCK = 0,              // Time interrupt
	EVENT_INTERVAL_TIMER_SHORT = 1,         // Interval elapsed
	EVENT_INTERVAL_TIMER_LONG = 2,          // Interval elapsed
	EVENT_RS232C_PORT = 3,                  // Status interrupt
	EVENT_PRINTER_PORT = 4,                 // Printer acknowledge
	EVENT_GRAPHICS_CONTROLLER = 5,          // Command acknowledge
	EVENT_KEYBOARD_USART = 6,               // Ready to transmit/receive
	EVENT_POWER_SUPPLY_OVERTEMPERATURE = 7, // Overtemperature
	EVENT_KEYBOARD_CHORD_RESET = 8,         // Operator entry
	EVENT_MOUSE_MOTION = 9,                 // Mouse motion detected
	EVENT_MOUSE_KEYSWITCH = 10,             // Mouse keyswitch change
	EVENT_VOICE_DATA_PRESENT = 11,          // Voice data present
	EVENT_SOUND_PARITY_ERROR = 12,          // Sound parity error
	EVENT_FIBER_OPTIC_LINK_WARNING = 13,    // Fiber-optic link warning
	EVENT_POWER_FAILURE_WARNING = 14,       // Power failure warning
	EVENT_POWER_FAILURE_WARNING_2 = 15      // Power failure warning, a second entry
};

// The monitor's speaker amplifier, in both the places that set its level: the
// sound chip's route into the speaker, and the MOSENB mute in
// update_speaker_amplifier(). MAME multiplies the two, so they have to agree.
static constexpr float SN76496_GAIN = 1.0f;

u8 compute_parity(u8 data) { data ^= data >> 4; data ^= data >> 2; data ^= data >> 1; return data & 1; }

// A relative-axis input port accumulates modulo its own mask, so one step of
// the mouse is the wrapped difference between two successive values.
static constexpr int MOUSE_AXIS_BITS = 12;

int mouse_axis_delta(ioport_value oldval, ioport_value newval)
{
	int delta = int(newval) - int(oldval);
	if (delta >= (1 << (MOUSE_AXIS_BITS - 1)))
		delta -= 1 << MOUSE_AXIS_BITS;
	else if (delta <= -(1 << (MOUSE_AXIS_BITS - 1)))
		delta += 1 << MOUSE_AXIS_BITS;
	return delta;
}

} // anonymous namespace


// The mouse itself is at the far end of the fiber-optic link, in the monitor;
// what the SI board sees is quadrature motion data and three keyswitch lines
// (paragraph 4.4.11). Emulating the quadrature would only feed a motion
// detector whose whole job is to turn it back into counts, so the host mouse
// drives the position counters directly.
//
// Directions are the ones paragraph 4.4.11.2 gives: "Mouse motion in the X
// (horizontal) direction is positive to the right; motion in the Y (vertical)
// direction is positive downward. The convention of positive X to the right
// and positive Y downward corresponds to a mouse cursor with the origin at the
// top left corner of the video display." MAME's own relative axes use the same
// sign convention, so neither needs reversing.
static INPUT_PORTS_START(sib)
	// Figure 4-14 puts the three keyswitches in the motion/keyswitch data
	// register, left to right in descending bit order, and marks only the
	// quadrature lines as inverted - so a closed keyswitch reads as a 1.
	PORT_START("mouse_buttons")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_BUTTON1) PORT_NAME("Mouse Left Button")   PORT_CODE(MOUSECODE_BUTTON1) PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(explorer_sib_device::mouse_button_changed), 0)
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_BUTTON2) PORT_NAME("Mouse Middle Button") PORT_CODE(MOUSECODE_BUTTON3) PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(explorer_sib_device::mouse_button_changed), 0)
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_BUTTON3) PORT_NAME("Mouse Right Button")  PORT_CODE(MOUSECODE_BUTTON2) PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(explorer_sib_device::mouse_button_changed), 0)

	// Host motion is passed through unscaled. The band applies a gain of its
	// own on the way to the cursor - driving the counters by a known amount and
	// measuring the cursor in the resulting snapshot gives 0.40 screen pixels
	// per count horizontally and 0.36 vertically (an 80 and 72 pixel move for
	// 200 counts, measured at both 200 and 400) - so the cursor travels rather
	// less far than the host pointer, which is how it should feel.
	PORT_START("mouse_x")
	PORT_BIT(0xfff, 0x000, IPT_MOUSE_X) PORT_SENSITIVITY(100) PORT_KEYDELTA(0) PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(explorer_sib_device::mouse_x_changed), 0)

	PORT_START("mouse_y")
	PORT_BIT(0xfff, 0x000, IPT_MOUSE_Y) PORT_SENSITIVITY(100) PORT_KEYDELTA(0) PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(explorer_sib_device::mouse_y_changed), 0)
INPUT_PORTS_END

ioport_constructor explorer_sib_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(sib);
}


explorer_sib_device::explorer_sib_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock) :
	device_t(mconfig, SIB, tag, owner, clock),
	device_ti_nubus_card_interface(mconfig, *this),
	m_screen(*this, "screen"),
	m_crt9007(*this, "crt9007"),
	m_i8251(*this, "i8251"),
	m_keyboard(*this, "keyboard"),
	m_mm58167(*this, "mm58167"),
	m_pit(*this, "pit"),
	m_z85030ps(*this, "z85030ps"),
	m_rs232(*this, "rs232"),
	m_usart_clock(*this, "usart_clock"),
	m_sn76496(*this, "sn76496"),
	m_nvram(*this, "nvram"),
	m_centronics(*this, "centronics"),
	m_centronics_data_out(*this, "centronics_data_out"),
	m_mouse_buttons(*this, "mouse_buttons"),
	m_mouse_x_axis(*this, "mouse_x"),
	m_mouse_y_axis(*this, "mouse_y"),
	m_video_ram(*this, "video_ram", VIDEO_RAM_SIZE * sizeof(u32), ENDIANNESS_BIG),
	m_nv_ram(*this,"nv_ram", 0x2000, ENDIANNESS_LITTLE),
	m_fault_led(*this, "fault_led"),
	m_monitor_led(*this, "monitor_led")
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
	save_item(NAME(m_event_vector));
	save_item(NAME(m_attribute_register));
	save_item(NAME(m_mask_register));
	save_item(NAME(m_operation_register));
	save_item(NAME(m_mouse_y_position));
	save_item(NAME(m_mouse_x_position));
	save_item(NAME(m_mouse_keyswitches));
	save_item(NAME(m_keyboard_txd));
	save_item(NAME(m_mouse_motion_event_pending));
	save_item(NAME(m_mouse_keyswitch_event_pending));
	save_item(NAME(m_interrupt_diag_control));
	save_item(NAME(m_monitor_control));
	save_item(NAME(m_diagnostic_data));
	save_item(NAME(m_voice_data_register));
	save_item(NAME(m_voice_data_present));
	save_item(NAME(m_printer_data));
	save_item(NAME(m_printer_control));
	save_item(NAME(m_centronics_busy));
	save_item(NAME(m_centronics_perror));
	save_item(NAME(m_centronics_select));
	save_item(NAME(m_centronics_fault));
	save_item(NAME(m_centronics_ack));
	save_item(NAME(m_sound_control));
	save_item(NAME(m_speech_register));
	save_item(NAME(m_usart_rxrdy));
	save_item(NAME(m_usart_txrdy));
}


void explorer_sib_device::device_reset()
{
	// Both fault LEDs come up lit, and the event generator comes up disabled -
	// see CONFIGURATION_REGISTER_POWER_UP.
	m_configuration_register = CONFIGURATION_REGISTER_POWER_UP;
	update_leds();

	// Paragraph 4.4.11.4: both halves of f2000c clear "on power-up or SI board
	// reset" - the interrupt enables because "the programmer must set the
	// appropriate interrupt enables as part of the board initialization
	// procedures", and MOSENB so the speaker amplifier always comes back up
	// muted (see update_speaker_amplifier()).
	m_interrupt_diag_control = 0;
	m_monitor_control = 0;
	update_speaker_amplifier();

	// With the interrupt enables gone there is nothing left for the interrupt
	// handshake controller to be holding.
	m_mouse_motion_event_pending = false;
	m_mouse_keyswitch_event_pending = false;

	// The printer port's control register is an output register holding three
	// levels, so drive them out along with it rather than only remembering
	// them. Without this the peripheral never sees the first DATSTRB- edge:
	// MAME's centronics printer starts out believing the strobe is already
	// low, so the first "write 05" of 4.4.9.2's send sequence is not an edge
	// and the first character of a job is dropped.
	printer_control_w(0x07);
}


void explorer_sib_device::nubus_map(address_map &map)
{
	map.unmap_value_high();

	// e00000 - graphics-and-bit-map-control-base
	graphics_bitmap_map(map);

	// f00000 - event-generator-base
	event_generator_map(map);

	// f10000 - printer-port-base
	printer_map(map);

	// f20000 - mouse-registers-base
	mouse_map(map);

	// f80000 - real-time-clock-base
	rtc_map(map);

	// f90000 - timers-base
	timers_map(map);

	// fa0000 - non-volatile-ram-base
	nvram_map(map);

	// fb0000 - rs232c-port-base
	rs232c_map(map);

	// fc0000 - keyboard-base
	map(0x00fc0000, 0x00fc0007).lrw32(NAME([this] (offs_t offset) {
		if (offset == 1 && BIT(m_interrupt_diag_control, 2) && !BIT(m_interrupt_diag_control, 3))
			return diagnostic_loopback_value();
		return u32(m_i8251->read(offset ^ 1));
	}), NAME([this] (offs_t offset, u32 data) {
		m_i8251->write(offset ^ 1, u8(data));
	}));

	// fe0000 - configuration-rom-base
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
	// e00098 - Graphics-Video-Test-Register
	map(0x00e00098, 0x00e0009b).lr32(NAME([this] () { return video_test_register(); }));

	map(0x00e80000, 0x00e9ffff).rw(FUNC(explorer_sib_device::video_ram_r), FUNC(explorer_sib_device::video_ram_w));

	map(0x00ec0000, 0x00edffff).rw(FUNC(explorer_sib_device::video_ram_r), FUNC(explorer_sib_device::video_ram_rmw_w));
}


u8 explorer_sib_device::crtc_r(offs_t offset)
{
	// The map is 32 words wide, so offset is already the 0-0x1f register
	// number. Bit 5 is not in the address at all - the chip needs six register
	// select lines for its 64 registers and the board's four-byte spacing over
	// e00000-e0007f supplies five - so the read bank has to be ORed in here.
	// It is not a convenience: reads of e00054/e00058 would otherwise hit the
	// chip's read-side Start and Reset commands at R15/R16, and the read-only
	// light pen registers would be unreachable.
	//
	// What fixes the missing bit as "1 on a read" is TI's own offset table in
	// kernel/micro-time.lisp, which puts Graphics-Interrupt-Enable (write) and
	// Graphics-Status-Register (read) both at offset 104 = R1A, and puts the
	// read-only Graphics-Light-Pen-Row/Column at 108/112 = R1B/R1C where the
	// chip has no write registers at all. The same five address bits have to
	// reach R1A for a write and R3A/R3B/R3C for a read, so the bit cannot come
	// from the address. How the board actually generates it is not in the
	// documentation to hand; only the mapping matters here. That is also why
	// Table 4-13 lists some addresses twice.
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


// Paragraph 4.4.10.6 and Figure 4-13: "a 4-bit video test register records the
// video transitions in each scan line using 2 bits for the negative transitions
// and 2 bits for the positive transitions ... This register updates after each
// horizontal scan and remains valid until the end of the next line scanned".
// D0 is set for an odd number of negative (high to low) transitions in that
// line and D1 for an even number, D2 and D3 the same for the positive ones -
// i.e. each pair is a two-bit count of that line's transitions, D0/D2 the low
// bit and D1/D3 the high one.
//
// The register is computed from the bit map on read rather than accumulated by
// the renderer, because the line it must describe is the one the CRT has just
// finished scanning, which screen_update() has no relationship to.
//
// "The video test register always reflects an extra pair of transitions ...
// because the TSTOUT bit falls at the end of the Manchester encoded data on
// channel A", so one positive and one negative transition are always added.
// The manual's own worked examples are the test for this: an all-zero bit map
// with normal video reads 5 (one of each, from that extra pair alone), and in
// reverse video A (two of each - the line itself now starts and ends with a
// transition against the blanking level). "A working SI board generates the
// values 0, 5, A and F."
//
// TSTOUT itself is not modelled: the manual requires it to be cleared at
// f20010 before reading this register, but since the extra pair is there
// unconditionally there is nothing for the bit to change.
u32 explorer_sib_device::video_test_register()
{
	// The line just scanned, in bit-map coordinates.
	int const line = m_screen->vpos() - 1 - VBEND;

	u32 const invert = BIT(m_attribute_register, 1) ? 0xffffffff : 0;
	unsigned positive = 1, negative = 1;

	// Video sits at the blanking level either side of the displayed line, so a
	// line that does not start and end at zero has a transition at each edge.
	int previous = 0;

	if (!BIT(m_attribute_register, 0) && line >= 0 && line < SCREEN_HEIGHT)
	{
		u32 const line_start = line * (SCREEN_WIDTH / 32);

		for (int x = 0; x < (SCREEN_WIDTH / 32); x++)
		{
			// Same order the shift registers use and screen_update() draws:
			// the serial stream leaves the word least significant bit first.
			u32 const d = m_video_ram[(line_start + x) & VIDEO_RAM_MASK] ^ invert;

			for (int i = 0; i < 32; i++)
			{
				int const bit = BIT(d, i);

				if (bit != previous)
				{
					if (bit)
						positive++;
					else
						negative++;
				}

				previous = bit;
			}
		}
	}

	if (previous)
		negative++;

	return (negative & 3) | ((positive & 3) << 2);
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
	// f00000-f0003c - the 16-location event generator register file, one 32-bit
	//                 vector address per cause. Paragraph 4.4.5.1: the address is
	//                 where the FF byte gets posted when that cause fires, and it
	//                 may be written as four bytes, two halfwords or one word -
	//                 hence COMBINE_DATA in event_vector_w(). The cause that
	//                 selects each location is the EVENT_* enum above.
	// f00040         - configuration register

	map(0x00f00000, 0x00f0003f).rw(FUNC(explorer_sib_device::event_vector_r), FUNC(explorer_sib_device::event_vector_w));

	map(0x00f00040, 0x00f00043).lrw32(NAME([this] {
		if (!machine().side_effects_disabled())
			LOGMASKED(LOG_NVRAM, "Configuration-Register read\n");
		return m_configuration_register;
	}), NAME([this] (u32 data) {
		LOGMASKED(LOG_NVRAM, "Configuration-Register write %08x\n", data);
		m_configuration_register = data & CONFIGURATION_REGISTER_WRITABLE_MASK;
		update_leds();
	}));
}

u32 explorer_sib_device::event_vector_r(offs_t offset)
{
	return m_event_vector[offset];
}

void explorer_sib_device::event_vector_w(offs_t offset, u32 data, u32 mem_mask)
{
	COMBINE_DATA(&m_event_vector[offset]);
}

// The two fault LEDs the board carries, both at its lower front edge where the
// enclosure's viewing slots show them - Field Maintenance Figure 1-13 is where
// their colours come from. The chassis self-test bit drives no lamp in this
// enclosure, so it stays plain storage with no output behind it.
void explorer_sib_device::update_leds()
{
	m_fault_led = bool(m_configuration_register & CONFIG_SI_BOARD_TEST_LED);
	m_monitor_led = bool(m_configuration_register & CONFIG_MONITOR_TEST_LED);
}

void explorer_sib_device::post_event(int cause)
{
	// Configuration register bit 1, "NuBus master enable" (Figure 4-4) - the doc
	// is explicit that the event generator must not act until this is set, since
	// it should only be enabled once the host has finished programming the event
	// addresses (section 4.4.4/4.4.5.2's own init-order requirement).
	if (!(m_configuration_register & CONFIG_NUBUS_MASTER_ENABLE))
		return;

	nubus().space().write_byte(m_event_vector[cause], 0xff);
}

void explorer_sib_device::post_voice_sample(u8 data)
{
	// Paragraph 4.4.11.8: "If the data-present bit is set, there is no voice
	// register read operation in progress, and the voice interrupt is enabled;
	// then the voice interrupt controller triggers an event and loads the voice
	// register with voice data bits VO<07:00>."
	//
	// Read literally that makes all three conditions gate the load as well as the
	// event, but the board's own diagnostic proves otherwise: the "Voice loopback
	// circuitry" subtest of the extended self-test writes a simulated sample with
	// VOICSEL set and **VINTENB clear**, then reads it straight back and fails if
	// it does not get it. Gating the load on VINTENB turns that subtest into
	// ">> ERROR" (verified both ways). So the interrupt enable gates only the
	// event; the register loads regardless.
	//
	// The remaining term, "no voice register read operation in progress", is a bus
	// interlock against loading the register out from under a read that is part
	// way through. A read here is a single atomic access, so there is no such
	// window to guard.
	m_voice_data_register = data;
	m_voice_data_present = true;

	if (BIT(m_interrupt_diag_control, 4))
		post_event(EVENT_VOICE_DATA_PRESENT);
}

void explorer_sib_device::pit_out0_w(int state)
{
	// Counter 0 is the short-term interval timer (4.4.8), and Table 4-4 gives it
	// event cause 1, "Interval elapsed". Same shape as counter 2 below: only the
	// rising edge out of mode 0 is the event.
	//
	// The Lisp band never sees this one - it programs counter 0 for mode 0 and
	// then only ever issues the latch command and reads the counter back, never
	// loading a count, so the counter never reaches terminal count (see the
	// comment on timers_map()). The GDOS System Interface Board diagnostic's
	// test 31 does use it, and without this reports error SIB0315,
	// "Short-term interval timer did not generate an event within the time
	// specified by the test".
	if (state)
		post_event(EVENT_INTERVAL_TIMER_SHORT);
}

void explorer_sib_device::pit_out2_w(int state)
{
	// Mode 0 (interrupt on terminal count): the output is forced low the instant
	// the control word is written, then goes high (and stays high) once the count
	// reaches zero - only that rising edge is the real "interval elapsed" event.
	if (state)
		post_event(EVENT_INTERVAL_TIMER_LONG);
}

void explorer_sib_device::rtc_irq_w(int state)
{
	// The MM58167 asserts this for any of its eight interrupt sources (Table
	// 4-7) and clears it when the interrupt status register is read, matching
	// pit_out2_w() above. Nothing was wired to the event generator here at all
	// before, so none of the RTC's interrupts (including D0 Compare) ever
	// reached the CPU.
	if (state)
		post_event(EVENT_REAL_TIME_CLOCK);
}


void explorer_sib_device::scc_int_w(int state)
{
	// The Z8530's single INT- output, whichever channel and whichever of its
	// receive/transmit/external-status sources raised it. The board has no
	// interrupt line of its own either, so like every other condition on this
	// card it reaches the processor as an event-generator write.
	if (state)
		post_event(EVENT_RS232C_PORT);
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
		post_event(EVENT_GRAPHICS_CONTROLLER);
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
		post_event(EVENT_KEYBOARD_USART);
	m_usart_rxrdy = asserted;
}

void explorer_sib_device::usart_txrdy_w(int state)
{
	bool const asserted = bool(state);
	if (asserted && !m_usart_txrdy)
		post_event(EVENT_KEYBOARD_USART);
	m_usart_txrdy = asserted;
}


// The parallel printer port, paragraph 4.4.9. "For programming purposes, the
// parallel printer port can be considered to be two 8-bit read/write registers"
// (4.4.9.2): register 0 the data register at f10000, register 1 the control and
// status register at f10004, whose read and write halves are unrelated to each
// other. Register 0 is also the loopback data register of 4.4.9, which is why
// reading it gives back what was last written rather than the state of the
// cable.
//
// Table 4-11's two halves, both confirmed by TI's own field definitions in
// kernel/micro-time.lisp (Printer-Status-Fields / Printer-Control-Fields, ppss
// in octal - #o0301 is bit 3, one bit wide):
//
//   bit   read                     write
//   0     Busy                     AUTOF-  automatic feed (active low)
//   1     Paper out error          DATSTRB- data strobe (active low)
//   2     Select (online)          INIT-   initialize (active low)
//   3     Fault (active low)       interrupt enable
//
// So the three control bits are the pin levels themselves and go straight to
// the Centronics lines. 4.4.9.2's polled send is: check the status, write the
// byte, write 05 to drop DATSTRB-, write 07 to raise it again, then poll Busy.
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
		m_centronics_data_out->write(m_printer_data);
	}));
	map(0x00f10004, 0x00f10007).rw(FUNC(explorer_sib_device::printer_status_r), FUNC(explorer_sib_device::printer_control_w));
}


u32 explorer_sib_device::printer_status_r()
{
	return (m_centronics_busy   ? 0x01 : 0x00)
		 | (m_centronics_perror ? 0x02 : 0x00)
		 | (m_centronics_select ? 0x04 : 0x00)
		 | (m_centronics_fault  ? 0x08 : 0x00);
}


void explorer_sib_device::printer_control_w(u32 data)
{
	m_printer_control = data & 0xff;

	m_centronics->write_autofd(BIT(m_printer_control, 0));
	m_centronics->write_strobe(BIT(m_printer_control, 1));
	m_centronics->write_init(BIT(m_printer_control, 2));
}


void explorer_sib_device::centronics_busy_w(int state)
{
	m_centronics_busy = state ? 1 : 0;
}


void explorer_sib_device::centronics_perror_w(int state)
{
	m_centronics_perror = state ? 1 : 0;
}


void explorer_sib_device::centronics_select_w(int state)
{
	m_centronics_select = state ? 1 : 0;
}


void explorer_sib_device::centronics_fault_w(int state)
{
	m_centronics_fault = state ? 1 : 0;
}


// PACK-, Table 4-9: "acknowledge pulse returned by the printer to indicate that
// the last character has been received and that the next character can be sent.
// The pulse is also returned each time the printer goes from the offline to the
// online state." That is the condition the port's event exists to report - the
// interrupt-mode sequence of 4.4.9.2 arms Table 4-4 cause 4 (event vector
// f00010, TI's own %SIB-Parallel-Event-Address) with bit 3 of the control
// register and then just sends, leaving the handler to notice the acknowledge.
// Posted on the falling edge, which is where the pulse says the character was
// taken.
void explorer_sib_device::centronics_ack_w(int state)
{
	if (!state && m_centronics_ack && BIT(m_printer_control, 3))
		post_event(EVENT_PRINTER_PORT);
	m_centronics_ack = state ? 1 : 0;
}


// MOSENB, bit 8 of f2000c - i.e. bit 0 of the monitor control register at
// f2000d (Figure 4-16). None of the audio is on this board: paragraph 4.4.11.6
// puts "a sound generator, an audio amplifier, and a speaker" in the system
// monitor, reached over the fiber-optic link, and MOSENB is the enable for that
// speaker amplifier - so it gates the chip's output rather than the chip.
//
// Paragraph 4.4.11.4 spells out both the power-up state and the reason for it:
// "Monitor sound enable (MOSOENB) is disabled on power-up or SI board reset.
// The monitor sound enable bit must be set by a subsequent control word to
// enable the monitor speaker amplifier. This disabling and enabling procedure
// prevents possible annoying sound bursts between the time a board reset is
// released and the time the sound generator and speech synthesizer setups are
// completed." Which is exactly what MAME's sn76496_device does when left
// ungated: its four channels power up at attenuation 0 (maximum volume) with a
// 0x400 tone period, and drone until the SIB self-test finally writes
// 9f/bf/df/ff to turn them off.
//
// TI's own field specs agree on the bit: %%MONITOR-SPEAKER-ENABLE #o1001 in
// ucode/lroy-qdev.lisp, Mouse-Control-Sound-Enable #o1001 in
// kernel/micro-time.lisp - ppss, bit 8, one bit wide.
void explorer_sib_device::update_speaker_amplifier()
{
	m_sn76496->set_output_gain(ALL_OUTPUTS, BIT(m_monitor_control, 0) ? SN76496_GAIN : 0.0f);
}


// The motion/keyswitch data register, f20008 - read-only, IDATA 07-00 (Table
// 4-15), bit assignments in Figure 4-14.
u32 explorer_sib_device::motion_keyswitch_r()
{
	if (!machine().side_effects_disabled())
	{
		// Reading the register is what the keyswitch event asks the host to do
		// ("the event notifies the host processor to read the motion/keyswitch
		// register"), and it reports motion as well, so it acknowledges both.
		m_mouse_motion_event_pending = false;
		m_mouse_keyswitch_event_pending = false;
	}

	// Paragraph 4.4.11.9: in internal loopback the simulated channel B stream
	// has all its bits equal to TSTOUT, and "sampling any of the bits in the
	// motion/keyswitch data register should produce the complement of the
	// TSTOUT bit"; MOUSSEL instead "gates the simulated parallel data byte to
	// replace the mouse motion, keyswitch, and keyboard data normally received
	// from the fiber-optic interface".
	if (diagnostic_loopback_active())
		return diagnostic_loopback_value();
	if (BIT(m_interrupt_diag_control, 1))
		return m_diagnostic_data & 0xff;

	return m_mouse_keyswitches | (m_keyboard_txd ? 0x80 : 0x00);
}


// Paragraph 4.4.11.5: the interrupt handshake controller takes "interrupt
// initiating signals from the fiber-optic data link, mouse motion detector, and
// mouse keyswitch detector circuits" and gates them with the matching bit of
// the interrupt enable register (MINTENB 06, KINTENB 05 - Figure 4-16) before
// handing them to the event generator.
//
// One event is posted per condition until the host reads a register that
// reports it, which is the same shape as the CRT controller's own interrupt
// (see crtc_int_w()): a second motion event before the first has been picked up
// would tell the host nothing it will not already see in the counters. 4.4.11.3
// notes that software can also ignore the event scheme entirely and poll the
// register instead, which works either way.
void explorer_sib_device::post_mouse_motion_event()
{
	if (!BIT(m_interrupt_diag_control, 6) || m_mouse_motion_event_pending)
		return;

	m_mouse_motion_event_pending = true;
	post_event(EVENT_MOUSE_MOTION);
}

INPUT_CHANGED_MEMBER(explorer_sib_device::mouse_x_changed)
{
	m_mouse_x_position = (m_mouse_x_position + mouse_axis_delta(oldval, newval)) & 0xffff;
	post_mouse_motion_event();
}

INPUT_CHANGED_MEMBER(explorer_sib_device::mouse_y_changed)
{
	m_mouse_y_position = (m_mouse_y_position + mouse_axis_delta(oldval, newval)) & 0xffff;
	post_mouse_motion_event();
}

INPUT_CHANGED_MEMBER(explorer_sib_device::mouse_button_changed)
{
	// Paragraph 4.4.11.3: the keyswitch detector fires on any change of state,
	// "when the operator presses or releases any of the keyswitches".
	m_mouse_keyswitches = m_mouse_buttons->read() & 0x70;

	if (!BIT(m_interrupt_diag_control, 5) || m_mouse_keyswitch_event_pending)
		return;

	m_mouse_keyswitch_event_pending = true;
	post_event(EVENT_MOUSE_KEYSWITCH);
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

	// f20008 - motion/keyswitch data register, read-only (Figure 4-14):
	// KOUT 07 (serial keyboard data), LKEY 06, MKEY 05, RKEY 04, then the raw
	// quadrature mouse motion inverted - YB- 03, YA- 02, XB- 01, XA- 00.

	// f2000c is two registers in one longword (Figure 4-16). The low byte is the
	// Interrupt Enable and Diagnostic Control register: SERRENB 7, MINTENB 6,
	// KINTENB 5, VINTENB 4, diagnostic control 3-0 (Figure 4-20: ELOPBAK,
	// ILOPBAK, MOUSSEL, VOICSEL). The high byte is the Monitor Control register
	// at f2000d: MOSENB 8, HIGAIN 9 (3.2x microphone gain), ALOPBAK 10 (analog
	// loopback through the monitor's codec), PARCHK 11 (force bad sound parity);
	// 12-15 are no connection.

	// SERRENB and PARCHK are storage only, deliberately. Between them they
	// describe a complete diagnostic loop that no TI software ever runs.
	// 4.4.11.4 and 4.4.11.6: PARCHK "forces bad (even) parity on the sound
	// control output to the monitor", the monitor recomputes parity over the
	// byte, disagrees, and "returns a sound error bit (SONDERR) to the SI board";
	// if SERRENB is set that raises event cause 12 ("Sound data", f00030), which
	// the event generator then clears with its sound interrupt acknowledge - an
	// explicit ack, unlike the voice interrupt's clear-on-read.
	//
	// Nothing drives it, so there is nothing to model and nothing that could
	// notice if it were modelled wrongly. Verified by logging every write to this
	// register and to the sound control register, across both the extended
	// self-test and a band boot: all 40 sound bytes go out with PARCHK and
	// SERRENB clear. The two bits are only ever set as part of a walking-bit and
	// checkerboard sweep of this register - low byte 00 01 02 08 0f 33 55 aa cc
	// f0 ff, high byte 100 300 500 a00 c00 f00 - and every sweep ends by writing
	// zero before the first sound byte goes out, which makes it a register
	// storage test rather than a parity test. The band settles on 0100, MOSENB
	// alone. The slot 5 extended self-test has no sound subtest either.

	// f20014 - sound control register, an 8-bit value plus generated odd parity
	// (Figure 4-17) shipped to the sound generator in the monitor. The self-test
	// writes the four "channel off" bytes:
	// 9f - 10011111 - 001 - tone 1 attenuation - off
	// bf - 10111111 - 011 - tone 2 attenuation - off
	// df - 11011111 - 101 - tone 3 attenuation - off
	// ff - 11111111 - 111 - noise attenuation - off

	// The position registers are read/write: paragraph 4.4.11.2's counters are
	// 16-bit up/down counters and "the host processor can preload these
	// counters, making the mouse position relative to some fixed point".
	map(0x00f20000, 0x00f20003).lrw32(NAME([this] {
		if (!machine().side_effects_disabled())
			m_mouse_motion_event_pending = false;
		return m_mouse_y_position;
	}), NAME([this] (u32 data) {
		m_mouse_y_position = data & 0xffff;
	}));
	map(0x00f20004, 0x00f20007).lrw32(NAME([this] {
		if (!machine().side_effects_disabled())
			m_mouse_motion_event_pending = false;
		return m_mouse_x_position;
	}), NAME([this] (u32 data) {
		m_mouse_x_position = data & 0xffff;
	}));
	map(0x00f20008, 0x00f2000b).lr32(NAME([this] { return motion_keyswitch_r(); }));
	map(0x00f2000c, 0x00f2000f).lrw32(NAME([this] {
		return (m_interrupt_diag_control & 0xff) | ((m_monitor_control & 0xf) << 8);
	}), NAME([this] (u32 data) {
		m_interrupt_diag_control = data & 0xff;
		m_monitor_control = (data >> 8) & 0xf;
		update_speaker_amplifier();
	}));
	map(0x00f20010, 0x00f20013).lrw32(NAME([this] {
		return (m_diagnostic_data & 0xff) | (u32(compute_parity(m_diagnostic_data & 0xff)) << 8);
	}), NAME([this] (u32 data) {
		m_diagnostic_data = data & 0x1ff;
		// 4.4.11.9: "The voice select bit (VOICSEL) gates the simulated parallel
		// data byte to the voice register. The TSTOUT bit serves as the
		// voice-data-present bit. TSTOUT must be set to trigger a voice interrupt
		// and to load the voice register." This is the board's only source of
		// voice data here - the real one is a codec on the monitor interface
		// board at the far end of the fiber-optic link, and nothing models the
		// monitor.
		//
		// Not modelled, from the same paragraph: "After the video has been
		// established, setting VOICSEL to a logic 1 causes the video to blank."
		// Left alone deliberately. Nothing reads voice data, so the only effect
		// of implementing it would be to blank the display for any diagnostic
		// that sets VOICSEL - and the manual does not say what unblanks it again.
		if (BIT(m_interrupt_diag_control, 0) && BIT(data, 8))
			post_voice_sample(data & 0xff);
	}));
	map(0x00f20014, 0x00f20017).lrw32(NAME([this] {
		return (m_sound_control & 0xff) | (u32(compute_parity(m_sound_control & 0xff) ^ 1) << 8);
	}), NAME([this] (u32 data) {
		m_sound_control = data & 0xff;
		m_sn76496->write(u8(data));
	}));
	// Paragraph 4.4.11.7 and Figure 4-18: a 9-bit register holding a byte of
	// speech synthesis data plus a generated odd parity bit, shipped over the
	// fiber-optic link to a speech synthesizer in the monitor at a fixed 8 kHz
	// rate. Nothing comes back - "there is no path to read data back from the
	// speech synthesizer in the monitor" - so a read returns this register's own
	// contents through the same diagnostic three-state drivers the sound control
	// register uses, which is what the handler below does. That is the whole SI
	// board side of speech, and it is complete.
	//
	// There is nothing further to implement, despite 4.4.11.7 deferring the
	// programming format to the Explorer Display Unit General Description: that
	// manual has no speech section, and its only two mentions of speech are a
	// microphone and headset "that can be used for future speech operations"
	// (1.2) and a connector "for a handheld microphone for future speech
	// operations" (2.3.1). The capability was never shipped and nothing ever
	// drove it - "speech" appears nowhere in the System Software Design Notes,
	// and in TI's Lisp sources only as an unused field declaration in
	// kernel/micro-time.lisp.
	map(0x00f20018, 0x00f2001b).lrw32(NAME([this] {
		return (m_speech_register & 0xff) | (u32(compute_parity(m_speech_register & 0xff) ^ 1) << 8);
	}), NAME([this] (u32 data) {
		m_speech_register = data & 0xff;
	}));
	// Paragraph 4.4.11.8 and Figure 4-19: VO<07:00> is a digitized voice sample
	// travelling the other way, from the monitor to the SI board, and VO08 is the
	// data-present bit. That bit exists because the two ends disagree about rate:
	// the fiber-optic link can read samples at the 50.52 kHz horizontal scan rate
	// but the monitor's codec only produces them at a fixed 8 kHz, asynchronously
	// to the scan timing, so VO08 is what distinguishes a new sample from the
	// same one read again.
	//
	// Read-only, and the read is the acknowledge: "the voice interrupt is cleared
	// when the contents of the voice register is read onto the I bus. There is no
	// explicit interrupt acknowledge signal."
	map(0x00f2001c, 0x00f2001f).lr32(NAME([this] {
		const u32 data = (m_voice_data_register & 0xff) | (m_voice_data_present ? 0x100 : 0);
		if (!machine().side_effects_disabled())
			m_voice_data_present = false;
		return data;
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

	// The MM58167 core is byte-wide, but every register here is word-spaced on
	// NuBus (register N at f80000 + 4N, Table 4-5) and the exp1proc issues masked dword
	// writes - so wrap it rather than installing an 8-bit handler. The dword
	// offset IS the register number.
	map(0x00f80000, 0x00f8005f).lrw32(NAME([this] (offs_t offset) {
		return u32(m_mm58167->read(offset));
	}), NAME([this] (offs_t offset, u32 data) {
		m_mm58167->write(offset, u8(data));
	}));
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
	// i8251 wiring above. exp1proc_cpu_device::write_unmapped_byte() issues a
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
	map(0x00fa0000, 0x00fa1fff).lrw32(NAME([this] (offs_t offset) {
		if (!machine().side_effects_disabled())
			LOGMASKED(LOG_NVRAM, "NVRam read %08x\n", offset);
		return u32(m_nv_ram[offset]);
	}), NAME([this] (offs_t offset, u32 data) {
		m_nv_ram[offset] = data & 0xff;
		LOGMASKED(LOG_NVRAM, "NVRam write %08x, %08x\n", offset, data);
	}));
}


void explorer_sib_device::rs232c_map(address_map &map)
{
	// fb0000 - rs232c-port-base, Table 4-21:
	//
	// fb0000 - Channel B buffer status and external status (RR0)
	//          Channel B pointer register, control data (WR0)
	// fb0004 - Channel A buffer status and external status (RR0)
	//          Channel A pointer register, control data (WR0)
	// fb0008 - Channel B receive/transmit data - unused, channel B is not a
	//          communications channel
	// fb000c - Channel A receive data buffer (RR8)
	//          Channel A transmit data buffer (WR8)
	// fb0010 - Interrupt acknowledge address
	//
	// So IADDR03 selects the channel and IADDR02 selects data or control, which
	// is the Z8530's D/C- and A/B- inputs in the order MAME's dc_ab_*() decodes
	// them. The dword offset is the register index directly.
	//
	// TI's own kernel sources disagree here: kernel/micro-time.lisp puts
	// RS232C-Channel-A-Status at offset 8 rather than 4, citing a page number
	// from a different revision of this manual. Table 4-21 is followed instead,
	// because paragraph 4.4.15.2's IADDR03/IADDR02 description independently
	// produces exactly the addresses above - and because nothing in the Lisp
	// sources ever reads those constants, so they were never exercised.
	//
	// Byte-wide core on a bus that issues masked dword writes, so wrap it.
	map(0x00fb0000, 0x00fb000f).lrw32(NAME([this] (offs_t offset) {
		return u32(m_z85030ps->dc_ab_r(offset));
	}), NAME([this] (offs_t offset, u32 data) {
		m_z85030ps->dc_ab_w(offset, u8(data));
	}));

	// "A read or write operation to the Z8530 with address bit IADDR04 high
	// acknowledges an interrupt. This is ordinarily a read operation directed
	// to read register 2 of channel B, which includes interrupt status bits and
	// an unused interrupt vector." Table 4-21 calls the address write-only and
	// says the status is to be had by the ordinary multistep register reads, so
	// the returned vector is of no use to this board - what the cycle is for is
	// the side effect of setting the interrupt-under-service bit.
	map(0x00fb0010, 0x00fb0013).lrw32(NAME([this] () {
		return u32(m_z85030ps->m1_r());
	}), NAME([this] (u32 data) {
		m_z85030ps->m1_r();
	}));
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
	// KOUT, bit 07 of the motion/keyswitch data register: the same received
	// serial line, tapped before the deglitcher so that "the serial keyboard
	// data can also be sampled by the host processor for possible testing"
	// (paragraph 4.4.11.2). Software normally masks it off, exactly as it does
	// the raw motion bits beside it.
	m_keyboard_txd = state;

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

	// "The time base for the real-time clock is a 32 768-hertz crystal
	// oscillator" (4.4.7). The core divides clock() by 32.768 to get its
	// internal 1 kHz millisecond tick.
	MM58167(config, m_mm58167, 32.768_kHz_XTAL);
	m_mm58167->irq().set(FUNC(explorer_sib_device::rtc_irq_w));

	// Counters 0/1 driven by "a 1-megahertz clock derived from the NuBus
	// clock" (section 4.4.8, 10MHz NuBus CLK- per section 4.4.1.2 - the
	// doc's own worked example, Figure 4-8, treats it as an exact 1MHz/
	// 1us period with no rounding). Counter 2 (the long-term interval
	// counter) has no fixed input clock at all - it's driven by counter
	// 1's own programmable square-wave output.
	PIT8253(config, m_pit);
	m_pit->set_clk<0>(1000000.0);
	m_pit->set_clk<1>(1000000.0);
	m_pit->out_handler<0>().set(FUNC(explorer_sib_device::pit_out0_w));
	m_pit->out_handler<1>().set(m_pit, FUNC(pit8253_device::write_clk2));
	m_pit->out_handler<2>().set(FUNC(explorer_sib_device::pit_out2_w));

	// The RS-232C port of paragraph 4.4.15 - see rs232c_map(). 2.4576 MHz is
	// "the master clock frequency" the baud rate generator counts down
	// (time constant = 1228800 / baud - 2, giving 50 to 19 200 baud), and it
	// has to arrive as PCLK rather than on RTxC: Figure 4-33 wires RTxCA- and
	// TRxCA- to the RXCLK and TXCLK pins of P2, which are the receive and
	// transmit clocks a *synchronous* modem sends back, not an on-board
	// oscillator. Hence configure_channels() is left alone and those two
	// inputs come from the rs232 port instead.
	// z85030ps on sib image
	SCC8530(config, m_z85030ps, 2.4576_MHz_XTAL);
	m_z85030ps->out_int_callback().set(FUNC(explorer_sib_device::scc_int_w));

	// Figure 4-33, pin for pin. Channel A carries the data and the three
	// signals the Z8530 has channel A pins for; the remaining EIA signals are
	// hung off channel B, which is why the manual says channel B is wired for
	// "auxiliary control line input/output" only.
	m_z85030ps->out_txda_callback().set(m_rs232, FUNC(rs232_port_device::write_txd));   // pin 15 -> XMTD-
	m_z85030ps->out_rtsa_callback().set(m_rs232, FUNC(rs232_port_device::write_rts));   // pin 17 -> RTS
	m_z85030ps->out_dtra_callback().set(m_rs232, FUNC(rs232_port_device::write_dtr));   // pin 16 -> DTR
	// Three channel B outputs are left unwired. TRxCB drives BAUDOUT at P3 66,
	// "transmit clock to drive the transmitter section of a synchronous modem"
	// (Table 4-18) - that is DB25 pin 24, V.24 circuit 113, i.e. write_etc(),
	// but the SCC core does not expose the TRxC pin as an output. RTSB- drives
	// SRTS (secondary request to send, P3 70) and DTRB- drives AL (analog
	// loopback, "can be asserted by the CPU", P3 65); neither is an
	// rs232_port_device line. All three only matter to a synchronous modem.

	RS232_PORT(config, m_rs232, default_rs232_devices, nullptr);
	m_rs232->rxd_handler().set(m_z85030ps, FUNC(scc8530_device::rxa_w));    // RCVD- -> pin 13 RXDA
	m_rs232->cts_handler().set(m_z85030ps, FUNC(scc8530_device::ctsa_w));   // CTS   -> pin 18 CTSA-
	m_rs232->dcd_handler().set(m_z85030ps, FUNC(scc8530_device::dcda_w));   // DCD   -> pin 19 DCDA-
	m_rs232->ri_handler().set(m_z85030ps, FUNC(scc8530_device::ctsb_w));    // RI    -> pin 22 CTSB-
	m_rs232->dsr_handler().set(m_z85030ps, FUNC(scc8530_device::dcdb_w));   // DSR   -> pin 21 DCDB-
	m_rs232->si_handler().set(m_z85030ps, FUNC(scc8530_device::syncb_w));   // SI    -> pin 29 SYNCB-
	m_rs232->rxc_handler().set(m_z85030ps, FUNC(scc8530_device::rxca_w));   // RXCLK -> pin 12 RTxCA-
	m_rs232->txc_handler().set(m_z85030ps, FUNC(scc8530_device::txca_w));   // TXCLK -> pin 14 TRxCA-

	CLOCK(config, m_usart_clock, 153600);
	m_usart_clock->signal_handler().set(m_i8251, FUNC(i8251_device::write_rxc));
	m_usart_clock->signal_handler().append(m_i8251, FUNC(i8251_device::write_txc));

	NVRAM(config, "nvram", nvram_device::DEFAULT_ALL_0);

	// The parallel printer port of paragraph 4.4.9 - see printer_map(). Only
	// the four status inputs of Table 4-9 are taken; DAT<8:1>, DATSTRB-, AUTOF-
	// and INIT- are all outputs from this end.
	CENTRONICS(config, m_centronics, centronics_devices, "printer");
	m_centronics->busy_handler().set(FUNC(explorer_sib_device::centronics_busy_w));
	m_centronics->perror_handler().set(FUNC(explorer_sib_device::centronics_perror_w));
	m_centronics->select_handler().set(FUNC(explorer_sib_device::centronics_select_w));
	m_centronics->fault_handler().set(FUNC(explorer_sib_device::centronics_fault_w));
	m_centronics->ack_handler().set(FUNC(explorer_sib_device::centronics_ack_w));
	OUTPUT_LATCH(config, m_centronics_data_out);
	m_centronics->set_output_latch(*m_centronics_data_out);

	SPEAKER(config, "speaker").front_center();
	// "The sound generator operates on a clock frequency of 2.048 megahertz" -
	// Explorer Display Unit General Description, paragraph 4.6. The frequency was
	// a guess (1.5 MHz) until that paragraph was found, and it sets the pitch of
	// every tone the machine plays, so it is not a detail. The exact part is
	// still not named in any manual - see the file header.
	SN76496(config, "sn76496", 2'048'000).add_route(ALL_OUTPUTS, "speaker", SN76496_GAIN);
}


ROM_START(sib)
	ROM_REGION32_BE(0x8000, "sib_config", ROMREGION_ERASE00)
	ROMX_LOAD("2236662_sib.bin", 0x003, 0x2000, CRC(3f1fc829) SHA1(f16d9d9b6d8e51282fd835e2cb716cb173b3eb39), ROM_SKIP(3))
ROM_END

const tiny_rom_entry *explorer_sib_device::device_rom_region() const
{
	return ROM_NAME(sib);
}
