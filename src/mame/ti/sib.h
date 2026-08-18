// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/**********************************************************************

    TI Explorer System Interface Board (SIB).

**********************************************************************/

#ifndef MAME_TI_SIB_H
#define MAME_TI_SIB_H

#pragma once

#include "ti_nubus.h"
#include "screen.h"
#include "machine/i8251.h"
#include "machine/nvram.h"
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
	void event_generator_map(address_map &map) ATTR_COLD;
	void printer_map(address_map &map) ATTR_COLD;
	void mouse_map(address_map &map) ATTR_COLD;
	void rtc_map(address_map &map) ATTR_COLD;
	void timers_map(address_map &map) ATTR_COLD;
	void nvram_map(address_map &map) ATTR_COLD;
	void rs232c_map(address_map &map) ATTR_COLD;
	void configuration_rom_map(address_map &map) ATTR_COLD;
	uint32_t screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);

	required_device<screen_device> m_screen;
    required_device<i8251_device> m_i8251;
    required_device<sn76496_device> m_sn76496;
    required_device<nvram_device> m_nvram;
    memory_share_creator<u32> m_video_ram;
    memory_share_creator<u8> m_nv_ram;
    u32 m_configuration_register;
};

DECLARE_DEVICE_TYPE(SIB, sib_device)

#endif // MAME_TI_SIB_H
