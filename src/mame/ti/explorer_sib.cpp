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

//#define VERBOSE (LOG_EVENT | LOG_NVRAM)
#include "logmacro.h"


DEFINE_DEVICE_TYPE(SIB, explorer_sib_device, "explorer_sib", "TI Explorer System Interface Board (2243145-0001A)")


namespace {

static constexpr u16 VIDEO_RAM_SIZE = 0x8000; // Guestimate
static constexpr u16 VIDEO_RAM_MASK = VIDEO_RAM_SIZE - 1;
static constexpr u16 SCREEN_WIDTH = 1024;
static constexpr u16 SCREEN_HEIGHT = 808;
static constexpr u32 PIXEL_CLOCK = 67'889'000;
static constexpr u16 CHARACTER_WIDTH = 32;
static constexpr u16 HTOTAL = 42 * CHARACTER_WIDTH;
static constexpr u16 HBEND = 7 * CHARACTER_WIDTH;
static constexpr u16 VTOTAL = 842;
static constexpr u16 VBEND = VTOTAL - SCREEN_HEIGHT;

static constexpr u32 CONFIG_NUBUS_MASTER_ENABLE = 0x0002;
static constexpr u32 CONFIG_SI_BOARD_TEST_LED = 0x0004;
static constexpr u32 CONFIG_NUBUS_TEST = 0x0008;
static constexpr u32 CONFIG_RESERVED = 0x00f0;
static constexpr u32 CONFIG_MONITOR_TEST_LED = 0x0100;
static constexpr u32 CONFIG_CHASSIS_TEST_LED = 0x0200;

static constexpr u32 CONFIGURATION_REGISTER_WRITABLE_MASK =
		CONFIG_NUBUS_MASTER_ENABLE | CONFIG_SI_BOARD_TEST_LED | CONFIG_NUBUS_TEST |
		CONFIG_RESERVED | CONFIG_MONITOR_TEST_LED | CONFIG_CHASSIS_TEST_LED;

static constexpr u32 CONFIGURATION_REGISTER_POWER_UP = CONFIG_SI_BOARD_TEST_LED | CONFIG_MONITOR_TEST_LED;

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

static constexpr float SN76496_GAIN = 1.0f;

static constexpr int MOUSE_AXIS_BITS = 12;

u8 calculate_parity(u8 data) {
	data ^= data >> 4;
	data ^= data >> 2;
	data ^= data >> 1;
	return data & 1;
}

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


static INPUT_PORTS_START(sib)
	PORT_START("mouse_buttons")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_BUTTON1) PORT_NAME("Mouse Left Button")   PORT_CODE(MOUSECODE_BUTTON1) PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(explorer_sib_device::mouse_button_changed), 0)
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_BUTTON2) PORT_NAME("Mouse Middle Button") PORT_CODE(MOUSECODE_BUTTON3) PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(explorer_sib_device::mouse_button_changed), 0)
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_BUTTON3) PORT_NAME("Mouse Right Button")  PORT_CODE(MOUSECODE_BUTTON2) PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(explorer_sib_device::mouse_button_changed), 0)

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
	m_configuration_register = CONFIGURATION_REGISTER_POWER_UP;
	update_leds();

	std::fill(std::begin(m_event_vector), std::end(m_event_vector), 0);
	m_attribute_register = 0x02;
	m_mask_register = 0;
	m_operation_register = 0;

	m_mouse_y_position = 0;
	m_mouse_x_position = 0;
	m_mouse_keyswitches = 0;
	m_keyboard_txd = 1;
	m_mouse_motion_event_pending = false;
	m_mouse_keyswitch_event_pending = false;

	m_interrupt_diag_control = 0;
	m_monitor_control = 0;
	update_speaker_amplifier();

	m_diagnostic_data = 0;
	m_voice_data_register = 0;
	m_voice_data_present = false;
	m_sound_control = 0;
	m_speech_register = 0;

	m_printer_data = 0;
	m_centronics_busy = 0;
	m_centronics_perror = 0;
	m_centronics_select = 1;
	m_centronics_fault = 1;
	m_centronics_ack = 1;

	m_usart_rxrdy = false;
	m_usart_txrdy = false;

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
	map(0x00e00000, 0x00e0007f).rw(FUNC(explorer_sib_device::crtc_r), FUNC(explorer_sib_device::crtc_w)).umask32(0x000000ff);
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
	map(0x00e00098, 0x00e0009b).lr32(NAME([this] () { return video_test_register(); }));

	map(0x00e80000, 0x00e9ffff).rw(FUNC(explorer_sib_device::video_ram_r), FUNC(explorer_sib_device::video_ram_w));

	map(0x00ec0000, 0x00edffff).rw(FUNC(explorer_sib_device::video_ram_r), FUNC(explorer_sib_device::video_ram_rmw_w));
}


u8 explorer_sib_device::crtc_r(offs_t offset)
{
	// It is unknown how the board comes up with the 0x20 offset on reads.
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


// A 4-bit video test register records the video transitions in each scan line
// using 2 bits for the negative transitions and 2 bits for the positive
// transitions ...
// This register updates after each horizontal scan and remains valid until the
// end of the next line.
//
// The register is computed from the bit map on read rather than accumulated by
// the renderer.
u32 explorer_sib_device::video_test_register()
{
	// The line just scanned, in bit-map coordinates.
	int const line = m_screen->vpos() - 1 - VBEND;

	u32 const invert = BIT(m_attribute_register, 1) ? 0xffffffff : 0;
	unsigned positive = 1, negative = 1;

	int previous = 0;

	if (!BIT(m_attribute_register, 0) && line >= 0 && line < SCREEN_HEIGHT)
	{
		u32 const line_start = line * (SCREEN_WIDTH / 32);

		for (int x = 0; x < (SCREEN_WIDTH / 32); x++)
		{
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


void explorer_sib_device::update_leds()
{
	m_fault_led = bool(m_configuration_register & CONFIG_SI_BOARD_TEST_LED);
	m_monitor_led = bool(m_configuration_register & CONFIG_MONITOR_TEST_LED);
}


void explorer_sib_device::post_event(int cause)
{
	if (!(m_configuration_register & CONFIG_NUBUS_MASTER_ENABLE))
		return;

	nubus().space().write_byte(m_event_vector[cause], 0xff);
}


void explorer_sib_device::post_voice_sample(u8 data)
{
	m_voice_data_register = data;
	m_voice_data_present = true;

	if (BIT(m_interrupt_diag_control, 4))
		post_event(EVENT_VOICE_DATA_PRESENT);
}

void explorer_sib_device::pit_out0_w(int state)
{
	if (state)
		post_event(EVENT_INTERVAL_TIMER_SHORT);
}


void explorer_sib_device::pit_out2_w(int state)
{
	if (state)
		post_event(EVENT_INTERVAL_TIMER_LONG);
}


void explorer_sib_device::rtc_irq_w(int state)
{
	if (state)
		post_event(EVENT_REAL_TIME_CLOCK);
}


void explorer_sib_device::scc_int_w(int state)
{
	if (state)
		post_event(EVENT_RS232C_PORT);
}


void explorer_sib_device::crtc_int_w(int state)
{
	if (state)
		post_event(EVENT_GRAPHICS_CONTROLLER);
}


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


void explorer_sib_device::printer_map(address_map &map)
{
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


void explorer_sib_device::centronics_ack_w(int state)
{
	if (!state && m_centronics_ack && BIT(m_printer_control, 3))
		post_event(EVENT_PRINTER_PORT);
	m_centronics_ack = state ? 1 : 0;
}


void explorer_sib_device::update_speaker_amplifier()
{
	m_sn76496->set_output_gain(ALL_OUTPUTS, BIT(m_monitor_control, 0) ? SN76496_GAIN : 0.0f);
}


u8 explorer_sib_device::sound_control_parity()
{
	return calculate_parity(m_sound_control & 0xff) ^ BIT(m_monitor_control, 3);
}


u32 explorer_sib_device::motion_keyswitch_r()
{
	if (!machine().side_effects_disabled())
	{
		m_mouse_motion_event_pending = false;
		m_mouse_keyswitch_event_pending = false;
	}

	if (diagnostic_loopback_active())
		return diagnostic_loopback_value();
	if (BIT(m_interrupt_diag_control, 1))
		return m_diagnostic_data & 0xff;

	return m_mouse_keyswitches | (m_keyboard_txd ? 0x80 : 0x00);
}


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
	m_mouse_keyswitches = m_mouse_buttons->read() & 0x70;

	if (!BIT(m_interrupt_diag_control, 5) || m_mouse_keyswitch_event_pending)
		return;

	m_mouse_keyswitch_event_pending = true;
	post_event(EVENT_MOUSE_KEYSWITCH);
}


void explorer_sib_device::mouse_map(address_map &map)
{
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
		return (m_diagnostic_data & 0xff) | (u32(calculate_parity(m_diagnostic_data & 0xff)) << 8);
	}), NAME([this] (u32 data) {
		m_diagnostic_data = data & 0x1ff;
		if (BIT(m_interrupt_diag_control, 0) && BIT(data, 8))
			post_voice_sample(data & 0xff);
	}));
	map(0x00f20014, 0x00f20017).lrw32(NAME([this] {
		return (m_sound_control & 0xff) | (u32(sound_control_parity()) << 8);
	}), NAME([this] (u32 data) {
		m_sound_control = data & 0xff;
		m_sn76496->write(u8(data));
		if (BIT(m_monitor_control, 3) && BIT(m_interrupt_diag_control, 7))
			post_event(EVENT_SOUND_PARITY_ERROR);
	}));
	map(0x00f20018, 0x00f2001b).lrw32(NAME([this] {
		return (m_speech_register & 0xff) | (u32(calculate_parity(m_speech_register & 0xff)) << 8);
	}), NAME([this] (u32 data) {
		m_speech_register = data & 0xff;
	}));
	map(0x00f2001c, 0x00f2001f).lr32(NAME([this] {
		const u32 data = (m_voice_data_register & 0xff) | (m_voice_data_present ? 0x100 : 0);
		if (!machine().side_effects_disabled())
			m_voice_data_present = false;
		return data;
	}));
}


void explorer_sib_device::rtc_map(address_map &map)
{
	map(0x00f80000, 0x00f8005f).lrw32(NAME([this] (offs_t offset) {
		return u32(m_mm58167->read(offset));
	}), NAME([this] (offs_t offset, u32 data) {
		m_mm58167->write(offset, u8(data));
	}));
}


void explorer_sib_device::timers_map(address_map &map)
{
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
	map(0x00fb0000, 0x00fb000f).lrw32(NAME([this] (offs_t offset) {
		return u32(m_z85030ps->dc_ab_r(offset));
	}), NAME([this] (offs_t offset, u32 data) {
		m_z85030ps->dc_ab_w(offset, u8(data));
	}));

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
	if (BIT(m_interrupt_diag_control, 3))
		m_i8251->write_rxd(state);
	else
		m_keyboard->rxd_w(state);

	m_i8251->write_dsr(state);
}


void explorer_sib_device::keyboard_txd_w(int state)
{
	m_keyboard_txd = state;

	if (!BIT(m_interrupt_diag_control, 3))
		m_i8251->write_rxd(state);
}


u32 explorer_sib_device::screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	const u32 black = 0x000000;
	const u32 white = 0xffffff;

	if (BIT(m_attribute_register, 0))
	{
		bitmap.fill(black, cliprect);
		return 0;
	}
	const u32 invert = BIT(m_attribute_register, 1) ? 0xffffffff : 0;

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
	SCREEN(config, m_screen);
	m_screen->set_raw(PIXEL_CLOCK, HTOTAL, HBEND, HBEND + SCREEN_WIDTH, VTOTAL, VBEND, VBEND + SCREEN_HEIGHT);
	m_screen->set_screen_update(FUNC(explorer_sib_device::screen_update));

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

	MM58167(config, m_mm58167, 32.768_kHz_XTAL);
	m_mm58167->irq().set(FUNC(explorer_sib_device::rtc_irq_w));

	PIT8253(config, m_pit);
	m_pit->set_clk<0>(1000000.0);
	m_pit->set_clk<1>(1000000.0);
	m_pit->out_handler<0>().set(FUNC(explorer_sib_device::pit_out0_w));
	m_pit->out_handler<1>().set(m_pit, FUNC(pit8253_device::write_clk2));
	m_pit->out_handler<2>().set(FUNC(explorer_sib_device::pit_out2_w));

	SCC8530(config, m_z85030ps, 2'457'600);
	m_z85030ps->out_int_callback().set(FUNC(explorer_sib_device::scc_int_w));
	m_z85030ps->out_txda_callback().set(m_rs232, FUNC(rs232_port_device::write_txd));   // pin 15 -> XMTD-
	m_z85030ps->out_rtsa_callback().set(m_rs232, FUNC(rs232_port_device::write_rts));   // pin 17 -> RTS
	m_z85030ps->out_dtra_callback().set(m_rs232, FUNC(rs232_port_device::write_dtr));   // pin 16 -> DTR

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

	CENTRONICS(config, m_centronics, centronics_devices, "printer");
	m_centronics->busy_handler().set(FUNC(explorer_sib_device::centronics_busy_w));
	m_centronics->perror_handler().set(FUNC(explorer_sib_device::centronics_perror_w));
	m_centronics->select_handler().set(FUNC(explorer_sib_device::centronics_select_w));
	m_centronics->fault_handler().set(FUNC(explorer_sib_device::centronics_fault_w));
	m_centronics->ack_handler().set(FUNC(explorer_sib_device::centronics_ack_w));
	OUTPUT_LATCH(config, m_centronics_data_out);
	m_centronics->set_output_latch(*m_centronics_data_out);

	SPEAKER(config, "speaker").front_center();
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
