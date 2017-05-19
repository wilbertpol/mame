// license:BSD-3-Clause
// copyright-holders:Wilbert Pol, Fabio Priuli
// thanks-to:Chris Covell
#ifndef MAME_BUS_SEGAAI_SOUNDBOX_H
#define MAME_BUS_SEGAAI_SOUNDBOX_H

#pragma once

#include "segaai_exp.h"
#include "machine/pit8253.h"
#include "machine/i8255.h"
#include "sound/ym2151.h"

// ======================> segaai_soundbox_device

class segaai_soundbox_device : public device_t,
						public device_segaai_exp_interface
{
public:
	segaai_soundbox_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	virtual machine_config_constructor device_mconfig_additions() const;
	virtual const tiny_rom_entry *device_rom_region() const;
	virtual void device_start();
	virtual void device_reset();

	virtual DECLARE_READ8_MEMBER(read_lo);
	virtual DECLARE_WRITE8_MEMBER(write_lo);
	virtual DECLARE_READ8_MEMBER(read_hi);
	virtual DECLARE_READ8_MEMBER(read_io);
	virtual DECLARE_WRITE8_MEMBER(write_io);

	DECLARE_READ8_MEMBER(tmp8255_porta_r);
	DECLARE_WRITE8_MEMBER(tmp8255_portb_w);
	DECLARE_WRITE8_MEMBER(tmp8255_portc_w);
	DECLARE_WRITE_LINE_MEMBER(ym2151_irq_w);

private:
	required_device<pit8253_device> m_tmp8253;
	required_device<i8255_device> m_tmp8255;
	required_device<ym2151_device> m_ym2151;
	required_region_ptr<u8> m_rom;
	u8 m_ram[0x20000];    // 128KB Expansion RAM
};

DECLARE_DEVICE_TYPE(SEGAAI_SOUNDBOX, segaai_soundbox_device);

#endif
