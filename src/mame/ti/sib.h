// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/**********************************************************************

    TI Explorer System Interface Board (SIB).

**********************************************************************/

#ifndef MAME_TI_SIB_H
#define MAME_TI_SIB_H

#pragma once

#include "ti_nubus.h"
#include "explorer_kbd.h"
#include "explorer_rtc.h"
#include "screen.h"
#include "machine/clock.h"
#include "machine/i8251.h"
#include "machine/nvram.h"
#include "machine/pit8253.h"
#include "sound/sn76496.h"


class sib_device : public device_t, public device_ti_nubus_card_interface
{
public:
	sib_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;

private:
	void nubus_map(address_map &map) ATTR_COLD;
	void graphics_bitmap_map(address_map &map) ATTR_COLD;
	void local_bus_map(address_map &map) ATTR_COLD;
	u32 video_ram_r(offs_t offset);
	void video_ram_w(offs_t offset, u32 data, u32 mem_mask);
	void video_ram_rmw_w(offs_t offset, u32 data, u32 mem_mask);
	void event_generator_map(address_map &map) ATTR_COLD;
	void printer_map(address_map &map) ATTR_COLD;
	void mouse_map(address_map &map) ATTR_COLD;
	void rtc_map(address_map &map) ATTR_COLD;
	void timers_map(address_map &map) ATTR_COLD;
	void nvram_map(address_map &map) ATTR_COLD;
	void rs232c_map(address_map &map) ATTR_COLD;
	void configuration_rom_map(address_map &map) ATTR_COLD;
	uint32_t screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);
	void i8251_txd_w(int state);
	void keyboard_txd_w(int state);
	bool diagnostic_loopback_active() const { return (m_interrupt_diag_control & 0x0c) != 0; }
	u32 diagnostic_loopback_value() const { return BIT(m_diagnostic_data, 8) ? 0x00 : 0xff; }
	u32 event_vector_r(offs_t offset);
	void event_vector_w(offs_t offset, u32 data, u32 mem_mask);
	// Event Generator (section 4.4.5): on a monitored interrupt condition, becomes
	// NuBus master and writes a plain 0xFF byte to the preprogrammed address stored
	// in m_event_vector[cause] (Table 4-4 gives the cause->index mapping) - not a
	// real CPU interrupt line, software just polls that memory location. Only
	// wired to the long-term interval timer's own completion (cause 2) for now;
	// the other 15 causes aren't hooked up yet.
	void post_event(int cause);
	void pit_out2_w(int state);

	required_device<screen_device> m_screen;
	required_device<i8251_device> m_i8251;
	required_device<explorer_keyboard_device> m_keyboard;
	required_device<explorer_rtc_device> m_rtc;
	required_device<pit8253_device> m_pit;
	required_device<clock_device> m_usart_clock;
	required_device<sn76496_device> m_sn76496;
	required_device<nvram_device> m_nvram;
	memory_share_creator<u32> m_video_ram;
	memory_share_creator<u8> m_nv_ram;
	u32 m_configuration_register;
	u32 m_event_vector[16]{};
	u32 m_mask_register = 0;
	u32 m_operation_register = 0;
	u32 m_mouse_y_position = 0;
	u32 m_mouse_x_position = 0;
	u32 m_interrupt_diag_control = 0;
	u32 m_monitor_control = 0;
	u32 m_diagnostic_data = 0;
	u32 m_voice_data_register = 0;
	u32 m_printer_data = 0;
	u32 m_sound_control = 0;
	u32 m_speech_register = 0;
};

DECLARE_DEVICE_TYPE(SIB, sib_device)

#endif // MAME_TI_SIB_H
