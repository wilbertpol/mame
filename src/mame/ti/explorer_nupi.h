// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/**********************************************************************

    TI Explorer NuBus Peripheral Interface (NUPI) board.

Bridges the Explorer's NuBus to a SCSI bus for hard disk and tape mass
storage. The board is an intelligent, autonomous NuBus card: it has its
own MC68000, an NCR5385 SCSI protocol controller, and DMA/FIFO logic.
The host CPU only ever talks to the mc68000 through shared RAM.

**********************************************************************/

#ifndef MAME_TI_EXPLORER_NUPI_H
#define MAME_TI_EXPLORER_NUPI_H

#pragma once

#include "ti_nubus.h"

#include "cpu/m68000/m68000.h"
#include "machine/ncr5385.h"
#include "machine/nscsi_bus.h"


class explorer_nupi_device : public device_t, public device_ti_nubus_card_interface
{
public:
	explorer_nupi_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;

private:
	// A go-strobe ($801aa) starts exactly one kind of transfer, selected by whether a
	// NuBus target ($801c0/$801d0 without an intervening $100001) and a direction
	// ($801a8) were programmed.
	enum : u8
	{
		DMA_IDLE,
		DMA_FIFO_TO_NUBUS,
		DMA_FIFO_DISCARD,  // no target programmed - the transfer runs but lands nowhere
		DMA_NUBUS_TO_SCSI,
		DMA_ONBOARD        // FIFO <-> the board's own RAM/ROM
	};

	void mpu_map(address_map &map) ATTR_COLD;
	void nubus_map(address_map &map) ATTR_COLD;
	u32 ram_window_r(offs_t offset);
	void ram_window_w(offs_t offset, u32 data, u32 mem_mask);
	u8 unknown_280001_r();
	u8 flag_register_r();
	void update_leds();
	u8 rom_r(offs_t offset);
	void scsi_irq_w(int state);
	void scsi_dreq_w(int state);
	void push_fifo_word_to_nubus(u16 word);
	u16 read_801c00_port(u16 half, u16 readback);
	TIMER_CALLBACK_MEMBER(dma_drain_timer_expired);
	void dma_drain_kick();
	void dma_transfer_complete();
	bool dma_draining() const { return m_dma_mode == DMA_FIFO_TO_NUBUS || m_dma_mode == DMA_FIFO_DISCARD; }
	void onboard_dma_run(bool host_group_read = false);
	u16 onboard_read16(u32 addr);
	void onboard_write16(u32 addr, u16 data);
	u16 page_register_r();
	void page_register_w(u16 data);
	u16 nubus_window_r(offs_t offset, u16 mem_mask);
	void nubus_window_w(offs_t offset, u16 data, u16 mem_mask);
	TIMER_CALLBACK_MEMBER(timer_tick);
	TIMER_CALLBACK_MEMBER(interval_timer_expired);

	required_device<m68000_device> m_mpu;
	required_device<ncr5385_device> m_scsi;
	required_device<nscsi_bus_device> m_scsibus;
	required_shared_ptr<u16> m_ram;
	required_memory_region m_firmware;
	required_memory_region m_firmware_nubus;
	output_finder<> m_fault_led;
	output_finder<> m_scsi_led;
	emu_timer *m_timer;
	emu_timer *m_interval_timer;
	emu_timer *m_dma_drain_timer;

	u8 m_unknown_280001;
	bool m_unknown_280001_bits12_toggle;
	u8 m_unknown_300000;
	u8 m_unknown_300001;
	u8 m_interval_timer_regs[0x40];
	u8 m_flag_register;
	u32 m_dma_address;
	u32 m_dma_count;
	u16 m_page_register_802c00_shadow;
	u8 m_scsi_fifo_pending_byte;
	bool m_scsi_fifo_have_pending_byte;
	u16 m_scsi_fifo_pending_word;
	bool m_scsi_fifo_have_pending_word;
	u8 m_dma_mode;
	u32 m_fifo_drain_pos;
	u32 m_dma_transfer_start_pos;
	bool m_dma_transfer_start_pending;
	bool m_dma_target_configured;
	u8 m_dma_out_byte_phase;
	bool m_dma_fire_irq;
	u8 m_dma_count_pending_byte;
	bool m_dma_count_have_pending_byte;
	bool m_dma_address_lo_fresh;
	bool m_dma_address_lo_negate_next;
	u16 m_dma_address_lo_raw;
	u16 m_dma_address_hi_raw;
	bool m_dma_address_loaded;
	u8 m_dma_direction;
	bool m_dma_irq5_enabled;
	u8 m_unknown_800c04_toggle;
	u16 m_unknown_508000;
	u16 m_unknown_508000_live;
	u16 m_unknown_518000;
	u16 m_unknown_518000_live;
	bool m_dma_in_flight;
	u16 m_unknown_dma_801c00;
	u16 m_unknown_dma_801c02;
	u8 m_fifo_801c00_count;
	u16 m_unknown_dma_803c00;
	u16 m_fifo[2048];
	u16 m_unknown_450000_holding;
	u16 m_fifo_in_pos;
	u8 m_unknown_450000_byte_phase;
	u16 m_fifo_out_pos;
	u16 m_page_register;
	u8 m_unknown_100001;
	u8 m_unknown_100005;
	u8 m_unknown_280000;
};

DECLARE_DEVICE_TYPE(NUPI, explorer_nupi_device)

#endif // MAME_TI_EXPLORER_NUPI_H
