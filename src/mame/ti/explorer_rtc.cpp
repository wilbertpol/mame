// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/**********************************************************************

    TI Explorer System Interface Board real-time clock.

    See explorer_rtc.h for background on the real hardware and the
    device_rtc_interface split (calendar rollover vs. sub-second digits).

**********************************************************************/

#include "emu.h"
#include "explorer_rtc.h"

DEFINE_DEVICE_TYPE(EXPLORER_RTC, explorer_rtc_device, "explorer_rtc", "TI Explorer Real-Time Clock")

explorer_rtc_device::explorer_rtc_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock) :
	device_t(mconfig, EXPLORER_RTC, tag, owner, clock),
	device_rtc_interface(mconfig, *this),
	m_irq_handler(*this)
{
}

void explorer_rtc_device::device_start()
{
	m_tick_timer = timer_alloc(FUNC(explorer_rtc_device::tick), this);
	m_tick_timer->adjust(attotime::from_msec(1), 0, attotime::from_msec(1));

	save_item(NAME(m_ram));
	save_item(NAME(m_second_boundary));
	save_item(NAME(m_interrupt_control));
	save_item(NAME(m_interrupt_status));
	save_item(NAME(m_10hz_divider));
}

void explorer_rtc_device::device_reset()
{
	m_second_boundary = machine().time();
	resync_tick_timer();
	m_interrupt_control = 0;
	m_interrupt_status = 0;
	std::fill(std::begin(m_ram), std::end(m_ram), 0);
}

// The 1ms tick's own phase is otherwise fixed from whenever device_start()
// first armed it, with no relation to m_second_boundary - so on a given real
// run, live_register(0) (the 100-microsecond digit) samples at essentially
// the same fixed, arbitrary point within each millisecond every time,
// almost never landing on exactly 0. Re-phasing the tick to fire exactly on
// m_second_boundary's own millisecond grid whenever the boundary moves fixes
// that: sampling then lines up with true millisecond edges, so frac_100us
// reads 0 right when register 1 also matches - see check_compare()'s D0
// Compare self-test, which needs both simultaneously.
void explorer_rtc_device::resync_tick_timer()
{
	double const elapsed = (machine().time() - m_second_boundary).as_double();
	double const into_ms = std::fmod(elapsed, 0.001);
	double const until_next = (into_ms > 0.0) ? (0.001 - into_ms) : 0.0;
	m_tick_timer->adjust(attotime::from_double(until_next), 0, attotime::from_msec(1));
}

// Register 0 (100-microseconds counter, Table 4-6): only the tens nibble
// (D7-D4) is used, a single BCD digit 0-9 counting hundreds of microseconds
// within the current millisecond. Register 1 (10/100-millisecond counter):
// both nibbles used, tens=hundreds-of-ms digit, units=tens-of-ms digit,
// together 0-990ms within the current second.
void explorer_rtc_device::update_subsecond_split(unsigned &ms_tens, unsigned &ms_units, unsigned &frac_100us) const
{
	double const ms = (machine().time() - m_second_boundary).as_double() * 1000.0;
	unsigned const whole_ms = unsigned(ms) % 1000;
	ms_tens = (whole_ms / 100) % 10;
	ms_units = (whole_ms / 10) % 10;
	frac_100us = unsigned((ms - std::floor(ms)) * 10.0) % 10;
}

// Re-derives m_second_boundary so a subsequent read reflects a just-written
// sub-second digit while preserving whatever the other digit(s) currently
// were (real hardware exposes register 0 and register 1 as separate NuBus
// writes, each only covering part of the total sub-second value).
void explorer_rtc_device::set_subsecond_part(unsigned reg_index, u8 bcd_value)
{
	unsigned ms_tens, ms_units, frac_100us;
	update_subsecond_split(ms_tens, ms_units, frac_100us);

	if (reg_index == 0)
		frac_100us = (bcd_value >> 4) & 0xf;
	else
	{
		ms_tens = (bcd_value >> 4) & 0xf;
		ms_units = bcd_value & 0xf;
	}

	double const total_ms = double(ms_tens * 100 + ms_units * 10) + frac_100us * 0.1;
	m_second_boundary = machine().time() - attotime::from_double(total_ms / 1000.0);
	resync_tick_timer();
}

// Maps Table 4-5's register indices 0-7 to their live BCD-packed byte value -
// used both for NuBus reads of the timing registers and for the D0 Compare
// check against the RAM registers (registers 8-15 use the same field order
// and format, Table 4-6).
u8 explorer_rtc_device::live_register(unsigned index)
{
	switch (index)
	{
	case 0: case 1:
		{
			unsigned ms_tens, ms_units, frac_100us;
			update_subsecond_split(ms_tens, ms_units, frac_100us);
			return index == 0 ? u8(frac_100us << 4) : u8((ms_tens << 4) | ms_units);
		}
	case 2: return convert_to_bcd(get_clock_register(RTC_SECOND));
	case 3: return convert_to_bcd(get_clock_register(RTC_MINUTE));
	case 4: return convert_to_bcd(get_clock_register(RTC_HOUR));
	case 5: return convert_to_bcd(get_clock_register(RTC_DAY_OF_WEEK));
	case 6: return convert_to_bcd(get_clock_register(RTC_DAY));
	case 7: return convert_to_bcd(get_clock_register(RTC_MONTH));
	}
	return 0;
}

// Sets an interrupt-status bit and, on the rising edge (no other source was
// already pending), asserts the device's own interrupt output line. Real
// hardware has a single interrupt output for all eight sources (Table 4-7),
// so the SIB only needs to know "some RTC event is now pending", not which
// one - matching how sib_device::post_event() is wired for the interval
// timer (pit_out2_w()).
void explorer_rtc_device::raise_interrupt_status(u32 bit)
{
	bool const was_clear = (m_interrupt_status == 0);
	m_interrupt_status |= bit;
	if (was_clear)
		m_irq_handler(1);
}

// D0 (Compare, Table 4-7): "an interrupt is generated when the current value
// in the timing registers is equal to the value programmed into the RAM
// registers."
void explorer_rtc_device::check_compare()
{
	if (!BIT(m_interrupt_control, 0))
		return;

	for (unsigned i = 0; i < 8; i++)
		if (live_register(i) != m_ram[i])
			return;

	raise_interrupt_status(0x01);
}

TIMER_CALLBACK_MEMBER(explorer_rtc_device::tick)
{
	// Ticks at 1ms (see device_start()) so check_compare() below reliably
	// samples the live registers finely enough to catch a match against the
	// RAM registers - the compare target's finest field (register 1) has
	// 10ms resolution, so a coarser tick (this used to be 100ms) has a real
	// chance of stepping right over a match window without ever sampling it
	// - confirmed live: the SIB's own "RTC Long" self-test kept re-arming
	// and retrying because of exactly this.
	//
	// D1 (10 Hz) only fires every 10th tick, to match its own documented
	// rate rather than this timer's finer internal rate.
	if (++m_10hz_divider >= 10)
	{
		m_10hz_divider = 0;
		if (BIT(m_interrupt_control, 1))
			raise_interrupt_status(0x02);
	}

	if (machine().time() - m_second_boundary >= attotime::from_seconds(1))
	{
		m_second_boundary += attotime::from_seconds(1);

		int const prev_minute = get_clock_register(RTC_MINUTE);
		int const prev_hour = get_clock_register(RTC_HOUR);
		int const prev_day = get_clock_register(RTC_DAY);
		int const prev_dow = get_clock_register(RTC_DAY_OF_WEEK);
		int const prev_month = get_clock_register(RTC_MONTH);

		advance_seconds();

		if (BIT(m_interrupt_control, 2))
			raise_interrupt_status(0x04); // 1 Hz
		if (BIT(m_interrupt_control, 3) && get_clock_register(RTC_MINUTE) != prev_minute)
			raise_interrupt_status(0x08); // 1 minute
		if (BIT(m_interrupt_control, 4) && get_clock_register(RTC_HOUR) != prev_hour)
			raise_interrupt_status(0x10); // 1 hour
		if (BIT(m_interrupt_control, 5) && get_clock_register(RTC_DAY) != prev_day)
			raise_interrupt_status(0x20); // 1 day
		if (BIT(m_interrupt_control, 6) && get_clock_register(RTC_DAY_OF_WEEK) != prev_dow && get_clock_register(RTC_DAY_OF_WEEK) == 1)
			raise_interrupt_status(0x40); // 1 week
		if (BIT(m_interrupt_control, 7) && get_clock_register(RTC_MONTH) != prev_month)
			raise_interrupt_status(0x80); // 1 month
	}

	check_compare();
}

u32 explorer_rtc_device::reg_r(offs_t offset)
{
	if (offset <= 7)
		return live_register(offset);
	if (offset <= 15)
		return m_ram[offset - 8];

	switch (offset)
	{
	case 16: // Interrupt status - "reading this register clears and resets the interrupt output"
		{
			u32 const result = m_interrupt_status;
			if (!machine().side_effects_disabled() && m_interrupt_status != 0)
			{
				m_interrupt_status = 0;
				m_irq_handler(0);
			}
			return result;
		}
	case 17: // Interrupt control (write-only per the doc, but harmless to read back)
		return m_interrupt_control;
	case 20: // Status bit - "ripple" flag; this emulation has no ripple-through
		// delay, so a read is always instantaneously valid.
		return 0;
	}
	return 0;
}

void explorer_rtc_device::reg_w(offs_t offset, u32 data)
{
	if (offset <= 1)
	{
		set_subsecond_part(offset, u8(data));
		return;
	}
	if (offset <= 7)
	{
		static constexpr int s_regs[6] = { RTC_SECOND, RTC_MINUTE, RTC_HOUR, RTC_DAY_OF_WEEK, RTC_DAY, RTC_MONTH };
		set_clock_register(s_regs[offset - 2], bcd_to_integer(u8(data)));
		clock_updated();
		return;
	}
	if (offset <= 15)
	{
		m_ram[offset - 8] = u8(data);
		return;
	}

	switch (offset)
	{
	case 17: // Interrupt control
		m_interrupt_control = data & 0xff;
		break;

	case 18: // Counters reset - "reset by writing all ones (hexadecimal FF)"
		if (data == 0xff)
		{
			for (int reg : { RTC_SECOND, RTC_MINUTE, RTC_HOUR, RTC_DAY_OF_WEEK, RTC_DAY, RTC_MONTH })
				set_clock_register(reg, 0);
			m_second_boundary = machine().time();
			resync_tick_timer();
			clock_updated();
		}
		break;

	case 19: // RAM reset - same "write all ones" convention
		if (data == 0xff)
			std::fill(std::begin(m_ram), std::end(m_ram), 0);
		break;

	case 21: // GO command - "resets the 1-ms, 10-ms, 100-ms, seconds, and
		// minutes counters... If the seconds counter is at a value greater
		// than 40 when the GO command is issued, the minute counter
		// increments; otherwise the counter does not increment." Data on
		// the bus is ignored.
		{
			int const seconds = get_clock_register(RTC_SECOND);
			set_clock_register(RTC_SECOND, 0);
			m_second_boundary = machine().time();
			resync_tick_timer();
			if (seconds > 40)
				advance_minutes();
			else
				clock_updated();
		}
		break;
	}
}

void explorer_rtc_device::map(address_map &map)
{
	map(0x00, 0x5f).rw(FUNC(explorer_rtc_device::reg_r), FUNC(explorer_rtc_device::reg_w));
}
