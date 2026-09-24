// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/**********************************************************************

    TI Explorer NuBus Ethernet controller board.

**********************************************************************/

#ifndef MAME_TI_EXPLORER_ENET_H
#define MAME_TI_EXPLORER_ENET_H

#pragma once

#include "ti_nubus.h"
#include "machine/i82586.h"


class explorer_enet_device : public device_t, public device_ti_nubus_card_interface
{
public:
	explorer_enet_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;

private:
	void nubus_map(address_map &map) ATTR_COLD;
	void lcc_map(address_map &map) ATTR_COLD;

	u32 buffer_ram_r(offs_t offset);
	void buffer_ram_w(offs_t offset, u32 data, u32 mem_mask = ~0);
	void channel_attention_w(offs_t offset, u32 data, u32 mem_mask = ~0);
	u32 event_address_r();
	void event_address_w(offs_t offset, u32 data, u32 mem_mask = ~0);
	u32 config_flag_r();
	void config_flag_w(offs_t offset, u32 data, u32 mem_mask = ~0);
	void lcc_irq_w(int state);
	void update_leds();

	required_device<i82586_device> m_i82586;

	required_shared_ptr<u16> m_buffer_ram;

	output_finder<> m_fault_led;

	u16 m_config_register = 0;
	u32 m_event_address = 0;
	bool m_lcc_irq = false;
};


DECLARE_DEVICE_TYPE(EXPLORER_ENET, explorer_enet_device)

#endif // MAME_TI_EXPLORER_ENET_H
