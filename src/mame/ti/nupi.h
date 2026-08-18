// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/**********************************************************************

    TI Explorer NuBus Peripheral Interface (NUPI) board.

    TI part number 2238040-0001. Bridges the Explorer's NuBus to a SCSI
    bus for hard disk and tape mass storage, per the "Explorer NuBus
    Peripheral Interface General Description" (2243146-0001B). The board
    is an intelligent, autonomous NuBus card: it has its own MC68000
    (10 MHz) running dumped firmware (2238056-5/2238057-5), an NCR5385
    SCSI protocol controller, and DMA/FIFO logic. The host CPU only ever
    talks to the documented NuBus-facing registers (Command Address
    Register, Configuration Register, Flag Register, ROM); the 68000
    firmware autonomously interprets the 8-word command block it is
    pointed at and drives the actual SCSI transaction.

    The manual only documents the NuBus-facing side. The 68000's own
    internal hardware addresses (where the NCR5385, DMA counters, and
    interval timer actually live) are not documented anywhere in the
    available manuals, so they were reverse-engineered from the firmware
    ROM disassembly. ROM base 0x000000, RAM base 0x180000 (4K), and the
    NCR5385 at 0x568000 (stride 2, low byte lane) are well corroborated
    (e.g. the confirmed SCSI interrupt handler reads AUX_STATUS then
    INT_STATUS at entry, exactly as expected). Several other address
    clusters found in the disassembly (host-write/DMA/misc glue logic)
    are handled by a logging catch-all pending further refinement against
    observed runtime behavior.

**********************************************************************/

#ifndef MAME_TI_NUPI_H
#define MAME_TI_NUPI_H

#pragma once

#include "ti_nubus.h"

#include "cpu/m68000/m68000.h"
#include "machine/ncr5385.h"
#include "machine/nscsi_bus.h"

#include <unordered_map>


class nupi_device : public device_t, public device_ti_nubus_card_interface
{
public:
	nupi_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;

private:
	required_device<m68000_device> m_mpu;
	required_device<ncr5385_device> m_scsi;
	required_device<nscsi_bus_device> m_scsibus;
	required_shared_ptr<u16> m_ram;
	required_memory_region m_firmware;

	void mpu_map(address_map &map) ATTR_COLD;
	void nubus_map(address_map &map) ATTR_COLD;

	// NuBus-facing (host) registers - Section 5.3 of the NUPI General Description
	u32 m_command_address; // Command Address Register (>Fs'E00004)
	u16 m_config_register; // Configuration Register (Figure 5-1)

	u32 command_address_r();
	void command_address_w(offs_t offset, u32 data, u32 mem_mask);
	u16 config_register_r();
	void config_register_w(u16 data);
	u8 flag_register_r();
	u8 rom_config_r(offs_t offset);

	// internal MPU-side hardware - see class comment above for confidence notes. Each
	// address_map window into misc_read/misc_write below is installed with a distinct
	// captured base so the sparse backing store stays keyed by true absolute address.
	u8 misc_read(u32 addr);
	void misc_write(u32 addr, u8 data);
	std::unordered_map<u32, u8> m_misc;

	void scsi_irq_w(int state);
	void scsi_dreq_w(int state);

	// Best-effort DMA address/count latches (0x800c00/0x802c00 in the reverse-engineered
	// map): the firmware reads both as longwords right after a NuBus-write interrupt, which
	// is consistent with an address+count pair, but this has not been validated dynamically.
	u32 m_dma_address;
	u32 m_dma_count;

	TIMER_CALLBACK_MEMBER(timer_tick);
	emu_timer *m_timer;
};

DECLARE_DEVICE_TYPE(NUPI, nupi_device)

#endif // MAME_TI_NUPI_H
