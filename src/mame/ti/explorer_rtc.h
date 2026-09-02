// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/**********************************************************************

    TI Explorer System Interface Board real-time clock.

    Real hardware (2243145-0001A SI General Description, section 4.4.7):
    an addressable real-time counter, 56 bits of RAM with a comparator,
    and an interrupt output, clocked from a 32768-hertz crystal. 24
    eight-bit registers (Table 4-5) live at NuBus FSF80000-FSF8005C: 8
    BCD timing registers (100us / 10&100ms / seconds / minutes / hours /
    day-of-week / day-of-month / month), 8 matching RAM "compare" target
    registers, an interrupt control/status register pair, counter/RAM
    reset registers, a status (ripple) bit, and a GO command.

    Calendar-correct minute/hour/day/month/leap-year rollover comes from
    MAME's own device_rtc_interface (dirtc.h), including seeding from the
    host's real date/time at machine start (running_machine::
    set_rtc_datetime(), automatic for every battery-backed device_rtc_
    interface). This device only owns the sub-second (100us/10ms/100ms)
    BCD digits itself, computed live from elapsed real time since the
    last whole-second boundary, since those sit below device_rtc_
    interface's one-second granularity.

**********************************************************************/

#ifndef MAME_TI_EXPLORER_RTC_H
#define MAME_TI_EXPLORER_RTC_H

#pragma once

#include "dirtc.h"


class explorer_rtc_device : public device_t, public device_rtc_interface
{
public:
	explorer_rtc_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	void map(address_map &map) ATTR_COLD;

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	virtual void rtc_clock_updated(int year, int month, int day, int day_of_week, int hour, int minute, int second) override {}
	virtual bool rtc_feature_leap_year() const override { return true; }

private:
	void update_subsecond_split(unsigned &ms_tens, unsigned &ms_units, unsigned &frac_100us) const;
	void set_subsecond_part(unsigned reg_index, u8 bcd_value);
	u8 live_register(unsigned index);
	void check_compare();

	u32 reg_r(offs_t offset);
	void reg_w(offs_t offset, u32 data);

	TIMER_CALLBACK_MEMBER(tick);

	u8 m_ram[8]{};
	attotime m_second_boundary;
	u32 m_interrupt_control = 0;
	u32 m_interrupt_status = 0;
	u8 m_10hz_divider = 0;
	emu_timer *m_tick_timer = nullptr;
};

DECLARE_DEVICE_TYPE(EXPLORER_RTC, explorer_rtc_device)

#endif // MAME_TI_EXPLORER_RTC_H
