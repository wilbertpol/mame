// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/**********************************************************************

    TI Explorer System Interface Board (SIB).

**********************************************************************/

#ifndef MAME_TI_SIB_H
#define MAME_TI_SIB_H

#pragma once

#include "ti_nubus.h"


class sib_device : public device_t, public device_ti_nubus_card_interface
{
public:
	sib_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;

private:
	void nubus_map(address_map &map) ATTR_COLD;
};

DECLARE_DEVICE_TYPE(SIB, sib_device)

#endif // MAME_TI_SIB_H
