// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/**********************************************************************

    TI Explorer NuBus memory board.

**********************************************************************/

#ifndef MAME_TI_EXPLORER_MEM_H
#define MAME_TI_EXPLORER_MEM_H

#pragma once

#include "ti_nubus.h"


class explorer_mem_device_base : public device_t, public device_ti_nubus_card_interface
{
protected:
	explorer_mem_device_base(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, u32 clock, u32 ram_size);

	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

private:
	void nubus_map(address_map &map) ATTR_COLD;

	u8 config_register_r();
	void config_register_w(u8 data);
	u8 base_register_r();
	void base_register_w(u8 data);
	u8 failure_latch_r();
	u8 test_register_r();
	void test_register_w(u8 data);

	static u8 calculate_parity(u8 data);
	u8 get_parity(offs_t offset) const;
	void store_parity_bit(offs_t offset, u8 bit);
	u8 test_force_bit(offs_t offset) const;
	void update_failure_location(offs_t offset, bool failed);

	u8 ram_r(offs_t offset);
	void ram_w(offs_t offset, u8 data);
	u8 ram_test_r(offs_t offset);
	void ram_test_w(offs_t offset, u8 data);

	u32 const m_ram_size;
	std::unique_ptr<u8[]> m_ram;
	std::unique_ptr<u8[]> m_parity;
	memory_view m_ram_view;
	memory_view m_ram_view_local_bus;

	u8 m_config_register = 0;
	u8 m_base_register = 0;
	u8 m_failure_location = 0;
	u8 m_test_register = 0;
	u8 m_failure_latch = 0;
	u16 m_nubus_status = 0;
};


class explorer_mem8mb_device : public explorer_mem_device_base
{
public:
	explorer_mem8mb_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

protected:
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
};


class explorer_mem2mb_device : public explorer_mem_device_base
{
public:
	explorer_mem2mb_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

protected:
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
};


DECLARE_DEVICE_TYPE(EXPLORER_MEM8MB, explorer_mem8mb_device)
DECLARE_DEVICE_TYPE(EXPLORER_MEM2MB, explorer_mem2mb_device)

#endif // MAME_TI_EXPLORER_MEM_H
