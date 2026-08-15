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

private:
	void nubus_map(address_map &map) ATTR_COLD;

	u32 const m_ram_size;
	std::unique_ptr<u8[]> m_ram;
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
