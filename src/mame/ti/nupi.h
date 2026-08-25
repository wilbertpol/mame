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
	// NuBus-side view of the firmware ROM, unscrambled once at device_start()
	// from m_firmware (which mpu_map() also uses directly, in its original
	// interleave, for the 68000's own instruction fetch - see nubus_map()'s
	// comment on rom_r() for why the two views differ).
	required_memory_region m_firmware_nubus;

	void mpu_map(address_map &map) ATTR_COLD;
	void nubus_map(address_map &map) ATTR_COLD;

	// NuBus-facing (host) registers - Section 5.3 of the NUPI General Description
	u32 m_command_address; // Command Address Register (>Fs'E00004)
	// Configuration Register (Section 5.3.2/Figure 5-1, base address >Fs'E0000B -
	// see nubus_map()'s comment). Documented as 16 bits, but only bits 0-7 are
	// wired up here - see nubus_map().
	u8 m_config_register;

	u32 command_address_r();
	void command_address_w(offs_t offset, u32 data, u32 mem_mask);
	// $280001 (byte 1 of the 0x280000-0x28000f MPU-side status block). Purpose of the
	// byte as a whole is not confirmed - only bit 0 is understood so far: the NuBus
	// master-error-pending flag, ACTIVE LOW (1 = idle/no error, 0 = error pending),
	// confirmed by two independent sources - the self-test (0x298 dispatcher entry 3,
	// ROM 0x4a4) requires it to read 1, and the real IRQ4 handler (0x14f2, NUINT2-/
	// NuBus-error per the doc) tests the same bit first and bails to $264 when set (i.e.
	// only does real work when the error flag is active/0). Nothing in the firmware
	// ever writes this byte, so it's genuine read-only hardware state, not scratch RAM.
	//
	// Bits 1-2 confirmed (via direct register trace) to toggle as a pair on every read:
	// entry 3's second check (ROM 0x4c0-0x4c8, subroutine at 0x4ec, called twice back to
	// back) takes the value from its own most recent read, flips bits 1 and 2 in a
	// register copy (two bchg's), then compares that copy against a brand new read of
	// this byte - and only passes if the two match. That's only possible if the real
	// hardware's bits 1-2 flip themselves on every read the same way the self-test's own
	// bchg's predict, i.e. a real "read-strobe toggles a diagnostic bit pair" circuit,
	// the same kind of mechanism already confirmed for 0x800c02's toggle-on-read (see
	// m_dma_address_lo_negate_next). Bit 0 is not part of the toggle - stays fixed.
	u8 m_unknown_280001 = 0x01;
	bool m_unknown_280001_bits12_toggle = false;
	u8 unknown_280001_r();
	u8 config_register_r();
	void config_register_w(u8 data);
	// Flag Register (>Fs'D40002, Section 5.3.4/Figure 5-3): bits 0-2 are
	// active-low (self-test complete / self-test passed / SCSI passed);
	// bits 3-7 are reserved and always 0. Defaults to 0x07 (all three
	// conditions inactive) until the 801a2/801ac/801ae writes in mpu_map()
	// clear the corresponding bits as each condition becomes true.
	u8 m_flag_register;
	u8 flag_register_r();
	u8 rom_r(offs_t offset);
	// DMA-Test-Register (>Fs'E0000F) - see nubus_map()'s comment.
	u8 m_dma_test_register = 0;
	void dma_test_register_w(u8 data);

	void scsi_irq_w(int state);
	void scsi_dreq_w(int state);

	// Best-effort DMA address/count latches (0x800c00/0x802c00 in the reverse-engineered
	// map): the firmware reads both as longwords right after a NuBus-write interrupt, which
	// is consistent with an address+count pair, but this has not been validated dynamically.
	u32 m_dma_address;
	u32 m_dma_count;

	// Self-test source addresses 0x801c0/0x801d0/0x801b0 (the latter is the page
	// register, see below) are write-only - never read back anywhere in the ROM.
	// Each write directly stores rol(data, 2) into the corresponding half of
	// m_dma_address/m_dma_count below, exactly as if it were a real write to that
	// register - confirmed correct via direct debugger register capture. A later
	// real DMA write just overwrites it the normal way, so no separate "is this
	// live yet" state is needed.

	// 0x800c02 read toggle: confirmed via exhaustive read-history analysis (10 data
	// points - all 7 of the original write-then-read self-test comparisons, plus 3
	// further un-written reads later in the same overall self-test) that reads of the
	// low word alternate: the read immediately following a write always returns the
	// plain value, and each subsequent read (with no intervening write) alternates
	// between the plain value and its bitwise NOT, starting with plain. Only the low
	// word has ever been observed doing this - no corresponding evidence for the high
	// word or m_dma_count, so left unapplied there.
	bool m_dma_address_lo_fresh = true;
	bool m_dma_address_lo_negate_next = false;

	// 0x800c04: no register at all behind this address - confirmed nothing else in
	// the ROM ever touches it. Just replays the fixed two-value sequence this one
	// self-test expects to read - see mpu_map().
	u8 m_unknown_800c04_toggle = 0;

	// FIFO status/pointer registers - the board's own header comment notes "DMA/FIFO
	// logic", and entry 5's own self-test (ROM 0x682-0x6ba) walks exactly 0x800 (2048)
	// values while expecting this register to climb in lockstep with a free-running
	// counter - the board's FIFO RAM is a 2K part. A write to $3801e8 (ROM 0x698, once
	// per loop iteration, right before each re-read) is a real FIFO "advance" strobe
	// (see mpu_map()) - corroborated by entry 7's self-test (ROM 0x90a) and a real
	// DMA/FIFO interrupt handler (ROM 0xda8/0xdc4) also pulsing it. The strobe LATCHES
	// the current live counter into the externally-readable value, THEN advances the
	// live counter for next time - so a read always sees the value as of the most
	// recent strobe, not one step ahead of it. This is what makes entry 5's loop work:
	// the compare at ROM 0x69e reads the just-latched (pre-this-strobe) value, which
	// matches D6 (itself only incremented after a successful compare), while a plain
	// write (0x80180/0x80190) resets both the live counter and the latched value
	// together, preserving the earlier, already-confirmed write/immediate-read checks
	// in 0x5b2 (which never touch $3801e8, so nothing here ever drifts for them).
	u16 m_unknown_508000 = 0;
	u16 m_unknown_508000_live = 0;
	u16 m_unknown_518000 = 0;
	u16 m_unknown_518000_live = 0;

	// Likely a real register in the same DMA register file as m_dma_address (0x800c00),
	// m_dma_count (0x802c00), and m_unknown_dma_803c00 below: 0x801c00 sits exactly
	// halfway between the first two, each exactly 0x1000 apart at the same 0xc00 offset
	// within its own bank - a regular, evenly-spaced hardware register layout, not a
	// coincidence. Purpose still unknown - exercised here only by a self-test comparing
	// it directly (unrotated) against a raw ROM table word, which is written - every
	// iteration - to 0x5c8000/0x5c8002 respectively; that write address is well outside
	// any range this firmware otherwise touches, so unlike the pairs above there's no
	// plausible physical loopback to model here. Modeled as a plain read-only register
	// fed by a plain write-only one, same mechanism as 0x508000/0x518000, pending a
	// real identity.
	u16 m_unknown_dma_801c00 = 0;
	u16 m_unknown_dma_801c02 = 0;

	// Entry 7's own self-test (ROM 0x8fc-0x94e) writes 16 consecutive words to $440000
	// (see m_unknown_450000_holding above - a different port on the same underlying
	// register file), then reads them back two at a time via 0x801c02 then 0x801c00,
	// in write order, across 8 loop iterations (16 writes, 16 reads). Modeled as a
	// small FIFO: a write to $440000 pushes, and a read of EITHER 0x801c00 or 0x801c02
	// pops the next entry, in whichever order they're actually accessed - falling back
	// to the plain static m_unknown_dma_801c00/801c02 above (entry 5's own mechanism,
	// which never writes $440000 and so never populates this queue) once empty.
	u16 m_dma_test_fifo[16] = {};
	u8 m_dma_test_fifo_write_pos = 0;
	u8 m_dma_test_fifo_read_pos = 0;

	// The fourth slot in that same 0x1000-spaced register file (0x800c00/0x801c00/
	// 0x802c00/0x803c00). Unlike the other three, no hi/lo pair was found for this one
	// (0x803c02 is never referenced anywhere) - either a single 16-bit register, or a
	// 32-bit one whose low half nothing in this ROM happens to touch. Write-only, never
	// read back anywhere. Exercised by the same self-test as 0x801c00 above (11
	// successive raw ROM table words written here, each compared against 0x5c8000/
	// 0x5c8002's static leftover content), but also - separately - by what looks like
	// real functional code (ROM ~0x884: writes a specific constant 0x956a here as part
	// of setup that also touches 0x508000, 0x100001/0x100007, and ends in the same
	// interrupt-driven busy-wait used elsewhere for real hardware operations), which is
	// strong independent evidence this is genuine hardware, not self-test scratch space.
	u16 m_unknown_dma_803c00 = 0;

	// A real 2K FIFO (2048 = 0x800 entries, matching the board's FIFO RAM size).
	// $450000/$450001 are written a byte at a time (byte-swapped, so together they hold
	// a 16-bit value the same way a normal big-endian word write would) into a holding
	// register; each $3801e8 strobe (see mpu_map()) pushes that holding register's
	// current content into the FIFO and advances the write pointer, wrapping every 2048
	// entries. A read of $450000/$450001 as a word returns the entry about to be
	// overwritten next - i.e. the oldest currently-buffered value - and advances the
	// SAME pointer to the next-oldest entry every SECOND read (a read pair, matching
	// entry 5's own self-test doing two consecutive identical reads per loop pass, ROM
	// 0x6ce/0x6d2 - the first read of a pair doesn't advance, letting both see the same
	// value, and the pointer only steps forward once the pair completes). Also exposed,
	// unadvanced, via a write-here-read-there loopback at $440000 (ROM 0x6b0, same
	// mechanism as 0x803c00/0x5c8000 in entry 4). Confirmed by entry 5's own self-test
	// (ROM 0x6ce-0x6da): after the LFSR loop's own 2048 strobes plus one extra strobe
	// just before the read (ROM 0x6c8), the oldest slot is the loop's very first write
	// (its LFSR seed, 0xa569); each subsequent read-pair then exposes the next-oldest
	// entry, which - since the self-test's own D2 independently recomputes the exact
	// same seed/recurrence used to fill the FIFO - keeps matching all the way through.
	u16 m_unknown_450000_fifo[2048] = {};
	u8 m_unknown_450000_holding[2] = { 0, 0 };
	u16 m_unknown_450000_pos = 0;
	bool m_unknown_450000_read_toggle = false;

	// Page register (0x0801b0, word-sized): holds the upper 14 bits (address bits 31-18)
	// of a NuBus target address, confirmed via firmware disassembly at 0x2ee0 - the
	// command address is swapped/rotated to extract exactly bits 31-18 right-justified,
	// then written here, matching the doc's own description (Section 4.3.1): "the MPU
	// places the 14 most significant address bits from the command address in the page
	// register", used together with "the lower 17 bits of the MPU address bus" (i.e. the
	// low bits of whatever local address 0x880000-0x89ffff the 68000 itself accesses -
	// see nubus_window_r/w) to form the full 32-bit NuBus address. Bit 17 of the
	// reconstructed address is not accounted for by either the page register (bits 31-18)
	// or the window offset (bits 16-0) - unverified whether it's simply always 0 for this
	// mechanism or comes from elsewhere (the doc separately mentions "UDS-", a byte-lane
	// strobe, being concatenated in - possibly relevant here in a way not yet understood).
	u16 m_page_register = 0;
	u16 page_register_r();
	void page_register_w(u16 data);

	// The 68000's local "NuBus window" (0x880000-0x89ffff, 128KB - derived from firmware
	// disassembly at 0x2ee0-0x2efe: the local address it builds splits at bit 17 into a
	// fixed 0x44/0x880000 select field and the low 17 bits of the target NuBus address).
	// Accesses here are translated to real NuBus addresses using m_page_register for the
	// upper bits and the accessed offset for the lower bits, then forwarded to the real
	// nubus() address space - this is the actual command-block-fetch mechanism.
	// Word-wide, matching the MPU's own 16-bit data bus - a real move.l here
	// naturally becomes two word bus cycles; offset is word-indexed.
	u16 nubus_window_r(offs_t offset, u16 mem_mask);
	void nubus_window_w(offs_t offset, u16 data, u16 mem_mask);

	TIMER_CALLBACK_MEMBER(timer_tick);
	emu_timer *m_timer;

	// Real interval timer (doc Section 4.5.1.4 / IRQ level 3, TIMERINT-) - distinct
	// from m_timer above (which drives IRQ4, i.e. NUINT2-/NuBus-error per the doc,
	// not a general clock). Confirmed via ROM self-test (0x298 dispatcher entry 2,
	// ROM 0x466): writes a count (0x1e/30) to 0x100005, then a second write of 0
	// to the same address starts it - firing too fast (synchronously) makes the
	// self-test's own short first check see it already expired and treat that as
	// an error, so real elapsed time is required. The loaded count has no confirmed
	// relationship to the actual delay, so modeled as a fixed 30us one-shot instead -
	// lands comfortably between the self-test's first (~11 dbra iterations, ~11us)
	// and second (~68 total, ~68us) checks at the 10MHz MPU clock.
	TIMER_CALLBACK_MEMBER(interval_timer_expired);
	emu_timer *m_interval_timer;

	// DMA-complete self-test trigger (IRQ level 1, DMAINT-). Confirmed via ROM
	// self-test (0x298 dispatcher entry 7, ROM 0x88c-0x890): writes 0xfe then 3 to
	// 0x100001, then waits for an interrupt - the same "parameter write, then a
	// second write starts it" shape as the interval timer above (0x100005), but for
	// DMA completion instead of the timer. The real end-to-end DMA path
	// (m_dma_count reaching 0 via scsi_dreq_w) doesn't apply here since entry 7 never
	// sets up a real transfer, so modeled as its own one-shot, mirroring
	// m_interval_timer's fixed-delay approach pending better evidence for the real
	// timing.
	TIMER_CALLBACK_MEMBER(dma_test_timer_expired);
	emu_timer *m_dma_test_timer;

	// 0x100001's own dedicated storage, replacing the generic misc_read/misc_write
	// m_misc lookup now that this address has its own fully carved-out handler. The
	// timer starts on bit 0's rising edge (0xfe->0x03 sets bit 0; the earlier 0->0xfe
	// arm write only sets bit 1, so it correctly doesn't trigger) - see the write
	// handler in mpu_map() for the full evidence.
	u8 m_unknown_100001 = 0;

	// 0x100005's own dedicated storage, same reasoning as m_unknown_100001 above -
	// see the write handler in mpu_map() for the trigger condition (write of 0 while
	// previously nonzero starts m_interval_timer).
	u8 m_unknown_100005 = 0;

	// 0x280000's own dedicated storage - the IRQ5 handler's command dispatch byte
	// (ROM 0xb3c: move.b $280000.l,D0 then cmpi.b #$31/#$30/#$33,D0). Written by
	// command_address_w() as scaffolding (see there for the "not confirmed correct"
	// caveat), read by the 0x280000 handler in mpu_map() (which also clears IRQ5 as
	// a side effect of the read).
	u8 m_unknown_280000 = 0;
};

DECLARE_DEVICE_TYPE(NUPI, nupi_device)

#endif // MAME_TI_NUPI_H
