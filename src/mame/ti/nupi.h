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

	u32 command_address_r();
	void command_address_w(offs_t offset, u32 data, u32 mem_mask);
	// Generic view of the on-board 4KB RAM (m_ram, see mpu_map()) from the 32-bit
	// NuBus side (>Fs'E00000-E00FFF) - see nubus_map()'s comment for the evidence
	// this range is genuinely shared RAM, not standalone registers. m_ram is
	// u16-wide (matching the MPU's own 16-bit bus), so a plain .ram().share()
	// can't be used directly across the width mismatch (MAME rejects it at
	// validation time) - these do the same big-endian word-splitting
	// command_address_w() already did by hand, just generalized to the whole
	// range. offset is in dwords.
	u32 ram_window_r(offs_t offset);
	void ram_window_w(offs_t offset, u32 data, u32 mem_mask);
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
	// $300000/$300001 - local MPU-side status bytes, both read-only (no ROM code
	// ever writes either). Exclusively used by the real IRQ4 handler (ROM 0x14f2+,
	// NUINT2-/NuBus-error per the doc): $300000 is read once (0x1514) and tested
	// bit by bit to build IRQ4's dispatch - bit 1 forks between the dynamic-jump
	// path (0x154c) and the queue-2-drain path (0x155c), bits 2-3 feed a small D0
	// classification value, bit 0 (checked later, 0x15a8) selects the
	// redirect-return-address mechanism via $1800dc. $300001 is read twice by two
	// dedicated accessor subroutines: 0xa06 masks the low nibble (andi.b #$f) and
	// 0xb00 extracts the high nibble (andi.w #$f0, rol.w #2) into the same
	// accumulating status word later stored at ($50,A6)/($40,A6). Not yet known
	// what real hardware condition either byte reflects, except for one confirmed
	// data point: the real command-processing path (ROM 0x9b6-0x9b8, called via the
	// movep subroutine at 0x980) reads $300001's low nibble via the 0xa06 accessor
	// and immediately does `cmpi.b #$c, D0` - i.e. real hardware is expected to read
	// back 0x0c in the low nibble at that point, not 0. Stubbed at that fixed value
	// pending a real identity for the rest of the byte; $300000 remains stubbed at 0.
	// $300001's low nibble is not a constant: entry 8 (the only ROM code that reads it,
	// via the 0xa06 accessor) requires 0x0c at ROM 0x9b8 and 0x9f6 but 0x00 at 0x9d8
	// and 0xa02, so it tracks live channel state. What separates those four reads is
	// exactly which register was touched last, giving the rule modeled in mpu_map():
	// set to 0x0c by a $3801e0 arm (st) and by the $100007 DMA acknowledge - both of
	// which mean "no transfer of ours is outstanding" - and cleared to 0x00 by a
	// $3801e0 disarm (clr.w) and by any read of the $450000 FIFO port. Reading it as
	// "channel quiet" matches the real IRQ4 handler's own use of the same nibble (ROM
	// 0x152c-0x1544): bits 3 and 2 both set is its no-error case, and bit 3 set with
	// bit 2 clear is its hard-error case, which this rule never produces. The high
	// nibble stays 0 - $af2 folds it into the DMA setup word (ROM 0xb00) but nothing
	// checks the result, so there's no evidence for any value there.
	u8 m_unknown_300000 = 0;
	u8 m_unknown_300001 = 0x0c;
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
	// m_dma_count is now known to be the real interval-timer word count (doc 4.5.1.4),
	// fed from 0x100001 - see the 0x100001 write handler in mpu_map().
	u32 m_dma_address;
	u32 m_dma_count;

	// 0x802c00's self-test readback (entry 4: write a table word to 0x801b0/page
	// register, expect this to read back rol(word, 2) & 0xfffc) - decoupled from the
	// real m_dma_count above. 0x801b0 is a real, load-bearing register (NuBus paging,
	// hit constantly during real command processing - see page_register_w()), so tying
	// its writes to m_dma_count meant ordinary paging during real commands stomped the
	// real transfer count. Confirmed necessary by direct test: removing the write
	// entirely (without this shadow) broke entry 4's self-test.
	u16 m_page_register_802c00_shadow = 0;

	// scsi_dreq_w()'s own byte-pairing state for pushing real SCSI-side bytes into
	// m_unknown_450000_fifo (the real on-board FIFO RAM, doc Section 4.3.4/4.5.4 - see
	// that member above for the self-test evidence establishing it). Kept separate from
	// m_unknown_450000_holding/m_unknown_450000_byte_phase (the self-test-driven
	// $440000/$3801e8 write path into the same array) so this doesn't disturb that
	// already-verified wiring - both paths share the same underlying FIFO array/position
	// pointer, matching real hardware having one physical FIFO fed from two sources.
	u8 m_scsi_fifo_pending_byte = 0;
	bool m_scsi_fifo_have_pending_byte = false;

	// FIFO -> NuBus stage (doc 4.3.4: "the DMA bus transfers data from the SCSI bus to
	// the FIFO RAM and from the FIFO RAM to the NuBus"): NuBus is a 32-bit synchronous
	// bus, so every second FIFO halfword completes one real 32-bit NuBus write. Big-endian
	// assembly (first word = high 16 bits) to match every other multi-byte NuBus value
	// in this driver.
	u16 m_scsi_fifo_pending_word = 0;
	bool m_scsi_fifo_have_pending_word = false;

	// Real 0x801aa go-strobe (see mpu_map()): before this fires, scsi_dreq_w() only
	// fills m_unknown_450000_fifo - no NuBus writes, since $2FD2's real
	// m_dma_address/m_dma_count aren't set up yet at that point in the real command
	// sequence. Once the go-strobe arms m_dma_active, m_fifo_drain_pos starts at
	// whatever m_unknown_450000_fifo's write position is AT THAT MOMENT (see
	// mpu_map()) - not rewound backward by count, since go-strobe fires as soon as
	// $2FD2's setup finishes, well before real SCSI data bytes start arriving (SCSI
	// arbitration/selection/command-phase overhead), so there's normally nothing of
	// this transfer's own to rewind into yet. The FIFO then gets drained by
	// dma_drain_timer_expired()/dma_drain_kick() below - one real, timed step per
	// word, not an instant catch-up - decrementing m_dma_count and firing IRQ1 on
	// completion (subject to m_dma_fire_irq, see below). Whether a tick waits for
	// fresh bytes to exist at m_fifo_drain_pos before popping them depends on
	// m_dma_write_to_nubus (see dma_drain_timer_expired()'s own comment): a real
	// transfer does wait, for correctness; entry 7's self-test doesn't need to (no
	// real target, so the exact bytes transferred don't matter) and its own first
	// DMA-complete wait relies on this - it has no real FIFO data behind it at all
	// yet when its go-strobe fires. m_fifo_drain_pos tracks how far the drain has
	// gotten, independent of the write position (m_unknown_450000_pos) so this
	// doesn't disturb the self-test's own FIFO usage. Used uniformly for both a real
	// SCSI transfer and entry 7's DMA-complete self-test.
	bool m_dma_active = false;
	u32 m_fifo_drain_pos = 0;
	void push_fifo_word_to_nubus(u16 word);

	// Where a real transfer's own data actually begins in m_unknown_450000_fifo -
	// recorded by scsi_dreq_w() the moment the FIRST word of a new transfer lands,
	// not computed at go-strobe time. Necessary because go-strobe's own timing
	// relative to real data arrival varies: $2FD2's setup (descriptor/address/
	// go-strobe) can finish before SCSI arbitration/selection/command-phase overhead
	// lets real data start flowing, but confirmed live that it doesn't always - real
	// bytes can already be streaming in (even fully queued) well before go-strobe
	// fires. m_dma_transfer_start_pending marks "the next word scsi_dreq_w() buffers
	// starts a new transfer" - set true at device_reset() and whenever
	// push_fifo_word_to_nubus() finishes a transfer (m_dma_count reaches 0), consumed
	// (and m_dma_transfer_start_pos captured) on the next word write.
	u32 m_dma_transfer_start_pos = 0;
	bool m_dma_transfer_start_pending = true;
	TIMER_CALLBACK_MEMBER(dma_drain_timer_expired);
	void dma_drain_kick();

	// Whether this transfer's NuBus target address was freshly configured, i.e. real
	// hardware actually knows where to write - set by the 0x801c0/0x801d0 write
	// handlers (m_dma_address's own halves, always written by $2FD2 right before its
	// own go-strobe), cleared by any 0x100001 write (a new descriptor load starting,
	// invalidating whatever address was configured for a PREVIOUS transfer). Entry 7's
	// self-test never writes 0x801c0/0x801d0 at all, so this is false for its own
	// go-strobes - confirmed necessary, not just defensive: without it, its drain
	// performed real NuBus writes using whatever page_register happened to be left
	// over from an earlier, unrelated self-test (0x3fff), landing on unmapped space
	// and setting the main raven CPU's own m_nubus_error flag (see
	// cpu/raven/raven.cpp) on every word, corrupting its "bus error on last transfer"
	// status for later, completely unrelated bus activity. m_dma_write_to_nubus
	// snapshots this at go-strobe time (see mpu_map()) for push_fifo_word_to_nubus()
	// to consult for the whole transfer, since m_dma_target_configured itself may be
	// invalidated again before the drain finishes.
	bool m_dma_target_configured = false;
	bool m_dma_write_to_nubus = false;

	// Whether THIS transfer's own completion should assert IRQ1 - snapshotted at
	// go-strobe time (see mpu_map()). A real transfer (m_dma_write_to_nubus) always
	// fires its own - m_dma_test_fifo below is entry 7 self-test scratch space,
	// wholly unrelated to a real SCSI transfer; checking it unconditionally here
	// caused a real transfer's completion IRQ1 to be silently swallowed whenever
	// self-test happened to leave m_dma_test_fifo's read/write positions unequal
	// earlier in the session. For a transfer with no real target (self-test), this
	// DOES check whether m_dma_test_fifo already has queued, unread data: entry 7's
	// second DMA wait writes its 16 real test words to 0x440000 BEFORE its own
	// go-strobe - those land in m_dma_test_fifo, a separate, pre-existing 16-entry
	// array (fed exclusively by 0x440000, drained by the 0x801c00/0x801c02 read
	// handlers below), not m_unknown_450000_fifo. That mechanism already asserts
	// IRQ1 itself when its own last entry is popped - confirmed necessary to gate on
	// here, not fire unconditionally: without this, push_fifo_word_to_nubus()
	// (walking m_unknown_450000_fifo/m_fifo_drain_pos, which holds none of this
	// wait's real data at all) asserted a second, independent IRQ1 roughly 2.6ms
	// earlier than the real mechanism's own natural timing - two competing
	// completion signals for what the self-test expects to be one event. Entry 7's
	// FIRST DMA wait has no such competing source (the 0x440000 writes happen only
	// after it completes), so m_dma_test_fifo is still empty at that go-strobe and
	// this stays true, letting push_fifo_word_to_nubus() provide the only completion
	// signal that exists for it.
	bool m_dma_fire_irq = true;

	// Real per-transfer count (doc 4.5.1.4, interval timer): the ROM writes the real
	// transfer descriptor to 0x100001 as two sequential bytes (low byte, then the byte
	// originally at bits 8-15 after a ror.l #8) - see the 0x100001 write handler in
	// mpu_map(). Traced and confirmed: for a real 0x400 (1024) byte transfer, the ROM
	// writes 0xfe then 0x00, i.e. ((0x400>>2)-2) split across two bytes; inverting that
	// transform ((descriptor+2)*4) recovers the real byte count. Sets m_dma_count
	// directly once both bytes have arrived.
	u8 m_dma_count_pending_byte = 0;
	bool m_dma_count_have_pending_byte = false;

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

	// Raw (un-rotated) value of the last write to $801c0, kept because $280002 reads
	// its low byte straight back out - see the $280002 handler in mpu_map(). m_dma_address
	// itself can't serve: it holds the rotated form and the engine advances it as a
	// transfer runs, while entry 8 expects the value it wrote.
	u16 m_dma_address_lo_raw = 0;
	u16 m_dma_address_hi_raw = 0;

	// ---- On-board ("self-test") DMA transfers -------------------------------------
	//
	// Entry 8 runs three real DMA transfers between the FIFO and the NUPI's own
	// memory, and checks the moved data byte for byte. Everything below is what those
	// three phases pin down; none of it is documented, but each phase's expectations
	// only line up one way.
	//
	// Target address. The ROM builds it by ror.l #2 on the byte address and handing
	// the LOW word of that to $801c0 (ROM 0xace-0xad6, 0xb14) - i.e. $801c0 carries
	// address bits 17:2, and the transfer is confined to an 18-bit on-board space.
	// $801d0 gets a literal in these phases ($bc38/$3c38/$bc3f), so it carries no
	// address at all here - it's mode bits. Decode of the 18-bit address: bit 17 set
	// selects ROM, clear selects the 4K RAM. That's what the phases need - phase A/B
	// use 0x034e -> 0x0d38 -> RAM offset 0xd38 (= the stack scratch at 0x180d38 their
	// own checks read back), phase C uses 0xc080 -> 0x30200 -> ROM offset 0x200 (= the
	// 0x40200 mirror its own comparison loop walks).
	//
	// Width. $801d0 bit 15 set = 16-bit, clear = 32-bit. Phase A ($bc38) writes only
	// the halfword at each 4-byte step, leaving the other half of the longword as
	// $ac2 cleared it - which is exactly what its loop checks (ROM 0x9e2/0x9e6:
	// value, then zero). Phase B ($3c38) writes both halves, and its loop checks both
	// (0xa50/0xa56). Phase C ($bc3f) reads one halfword per 4-byte step, which is why
	// its comparison loop advances the ROM pointer by 4 while consuming one FIFO
	// halfword's worth of data.
	//
	// Direction comes from $801a8 (0 = FIFO to memory, 0xff = memory to FIFO), the
	// same byte the real $2FD2 setup drives from the command block's own bit 15
	// (ROM 0x2fd4-0x2fda).
	//
	// Halfword order within a 32-bit step: the first halfword out of the FIFO lands
	// at the HIGH byte offset (+2) and the second at +0. Phase B's loop is what pins
	// this down - it expects (0xfe, 0xff) for a FIFO holding 0x00ff, 0x00fe - and it
	// matches NuBus being little-endian while the 68000 reading it back is not.
	//
	// 16-bit mode splits the FIFO group: only one of the two halfwords goes to (or
	// comes from) memory. Going out, the other half is what the host reads through
	// the $450000 port, so a 16-bit FIFO-to-memory transfer only advances when the
	// host has taken its half - modeled as m_selftest_dma_credits. Phase A depends on
	// that pacing, not just on the data: its check at ROM 0x9d0 requires that only ONE
	// longword has been written by the time it runs, after its own single movep.l at
	// 0x9cc, and each loop iteration then checks the longword its previous movep.l
	// released. Coming in, there's no host half to pair with, so the same halfword
	// fills both - which is why phase C's readback loop reads each value twice
	// (0xa94-0xaaa reads $450000 four times per ROM word).
	bool m_dma_address_loaded = false;
	bool m_selftest_dma_active = false;
	bool m_selftest_dma_to_fifo = false;
	bool m_selftest_dma_16bit = false;
	u32 m_selftest_dma_addr = 0;
	u32 m_selftest_dma_left = 0;
	u32 m_selftest_dma_credits = 0;
	// Last value written to $801a8 - the transfer direction bit, see above.
	u8 m_dma_direction = 0;
	// An on-board transfer's completion raises IRQ5 as well as IRQ1, when $3801ea has
	// been st'd (0xff) to arm it - which $af2 does as part of every transfer setup
	// entry 8 performs (ROM 0xafa). Entry 8's shared post-transfer tail requires it:
	// $a16 clears D7 bit 12 at ROM 0xa22 and fails unless something set it, and the
	// only thing that ever does is the IRQ5 handler's own short path at ROM 0x23a -
	// the one it takes when D7 bit 11 is set, which is exactly what entry 8 arms
	// (ori.w #$900) immediately before unmasking interrupts at 0x9e0/0xa4c. $3801ea is
	// the natural place for that arm: $af2 addresses this whole flag-byte block
	// through A5 = $3801ea, and the same handler path acks by writing here (0x23e).
	// Deliberately limited to on-board transfers, so a real SCSI-to-NuBus transfer's
	// completion still raises IRQ1 alone.
	// Armed by an st (0xff) to $3801ea, cleared by any write to $801aa, and latched
	// into m_selftest_dma_irq5 by the go-strobe so only the transfer whose own setup
	// armed it raises the interrupt. The clear matters: the register test at ROM 0x510
	// fires a stray go-strobe of its own (smi $801aa at 0x586, with a leftover count
	// still loaded), and letting that transfer raise IRQ5 sent the firmware through the
	// full IRQ5 handler and back into the reset path at 0x1822 - a self-test restart
	// loop. It writes 0x00 to $801aa on its way in, so the clear disarms it there;
	// every real setup path arms $3801ea after its own $801aa write ($af2, ROM
	// 0xaf2 then 0xafa).
	bool m_dma_irq5_armed = false;
	bool m_selftest_dma_irq5 = false;
	void selftest_dma_run();
	u16 selftest_dma_read16(u32 addr);
	void selftest_dma_write16(u32 addr, u16 data);

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
	// FIFO address-counter pair (doc 4.5.4). Each side is an address counter plus its
	// own readback latch: a 0x3801e8 strobe reports the value from before that strobe
	// and advances the counter for the next one (confirmed by entry 5, whose first
	// pass requires the setup value to still be readable after its own first strobe,
	// ROM 0x692-0x69e). The _live half is the counter, the other half is what reads
	// return. See the 0x518000 handler in mpu_map() for how the pair reads back while
	// a transfer is in flight.
	u16 m_unknown_508000 = 0;
	u16 m_unknown_508000_live = 0;
	u16 m_unknown_518000 = 0;
	u16 m_unknown_518000_live = 0;
	// Set by the 0x801aa go-strobe, cleared by the 0x100007 DMA acknowledge. A strobe
	// trigger with an explicit acknowledge is the only "is a transfer running" signal
	// available here - there is nothing to poll - so this brackets the in-flight
	// window without depending on drain pacing (which is only a placeholder, see
	// dma_drain_kick()).
	bool m_dma_in_flight = false;

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
	//
	// Real hardware doc (NUPI manual Section 5.3.6) confirms this same FIFO RAM port
	// as genuine, narrow hardware ("a 2-byte block", diagnostic-window address
	// >FSB80000) - exactly the shape MOVEP exists to drain. The real command-processing
	// path (ROM ~0x980-0xa04) reads this port via movep.l, i.e. four individual
	// byte-wide accesses at $450000/$450002/$450004/$450006 (movep's byte-lane stride
	// is 2, so $450001/3/5/7 are never touched). That's the same "two reads make a
	// pair, pointer advances after the second" mechanism above, just observed at byte
	// instead of word granularity - each movep byte-lane is one more byte-wide access
	// into the same running phase, not a distinct register. m_unknown_450000_byte_phase
	// replaces the old bool toggle: phase 0/2 return the current word's high byte,
	// phase 1/3 the low byte (matching this space's big-endian word convention), and
	// the FIFO pointer only advances once phase wraps from 3 back to 0 - i.e. after
	// two full word's worth of byte accesses, same cadence as the original two-word-read
	// pair. A plain 16-bit read at $450000 (entry 5's own access pattern) still
	// decomposes into two byte sub-accesses the same way, so this is a strict
	// generalization, not a behavior change for the already-confirmed entry 5 path.
	u16 m_unknown_450000_fifo[2048] = {};
	u8 m_unknown_450000_holding[2] = { 0, 0 };
	u16 m_unknown_450000_pos = 0;
	u8 m_unknown_450000_byte_phase = 0;
	// The FIFO's own read cursor, split out from m_unknown_450000_pos (which stays the
	// write cursor). They used to be one variable, which only worked because every
	// path that had exercised the FIFO so far wrote and read in lockstep. Entry 8 does
	// not: it fills the FIFO with 256 words up front and only then starts reading, so
	// the two genuinely have to move independently. A $80180 write reloads BOTH (see
	// mpu_map()) - that's what keeps every self-test's fill and its own readback
	// aligned, since each one reloads $80180 immediately before the phase that reads
	// back what it just wrote.
	u16 m_fifo_out_pos = 0;

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

	// DMA completion trigger (IRQ level 1, DMAINT-). Confirmed via ROM self-test
	// (0x298 dispatcher entry 7, ROM 0x88c-0x890): writes 0xfe then 3 to 0x100001,
	// then waits for an interrupt - the same "parameter write, then a second write
	// starts it" shape as the interval timer above (0x100005), but for DMA completion
	// instead of the timer. Both entry 7's self-test and a real SCSI transfer share
	// the real end-to-end path now (m_dma_count reaching 0 via
	// dma_drain_timer_expired()/push_fifo_word_to_nubus() above) - see m_dma_active.
	emu_timer *m_dma_drain_timer;

	// 0x100001's own dedicated storage, replacing the generic misc_read/misc_write
	// m_misc lookup now that this address has its own fully carved-out handler. Plain
	// data storage only - see m_unknown_801aa below for the actual "go" trigger.
	u8 m_unknown_100001 = 0;

	// 0x801aa (see the write handler in mpu_map()): re-examination of the ROM around
	// entry 7's own 0x100001 self-test (0x88c-0x918) showed the busy-wait that
	// follows is gated by D7 bit 8 (bsr $802), armed via a SEPARATE st $801aa write
	// right after the 0x100001 load - not by 0x100001 itself. The real
	// (non-self-test) DMA-transfer setup path confirms the same shape: it loads
	// 0x100001 (ROM 0x2ff6/0x2ffe), then loads m_dma_address/0x80180 from the
	// transfer descriptor, and only THEN does st $801aa (ROM 0x3020) - previously
	// unexplained, now read as the real arm/go strobe. The DMA-complete (bit8/IRQ1)
	// interrupt handler itself clears this same byte (sf $801aa, ROM 0xbbc) on
	// completion, which a pure software bookkeeping flag wouldn't need. Triggers on
	// data == 0xff specifically (the exact value st writes, never anything else) and
	// is gated on m_unknown_100001 being nonzero, so the many other, unrelated
	// st/sf $801aa call sites elsewhere in the ROM (generic self-test busy flags with
	// nothing loaded in 0x100001 at the time) don't spuriously arm this timer. No
	// separate storage/edge-detection needed for this byte itself.

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
