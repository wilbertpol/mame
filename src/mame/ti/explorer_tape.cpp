// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/**********************************************************************

    TI Explorer NUPI cartridge tape drive.

Everything this device does comes from nscsi_tape_device. The one thing it
adds is a transfer rate, for the same reason explorer_msu_device has one.

**********************************************************************/

#include "emu.h"
#include "explorer_tape.h"


DEFINE_DEVICE_TYPE(NUPI_TAPE, explorer_tape_device, "explorer_tape", "TI Explorer cartridge tape drive")


explorer_tape_device::explorer_tape_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	nscsi_tape_device(mconfig, NUPI_TAPE, tag, owner, clock, "MAME", "SCSI tape drive", "1.0")
{
}


// nscsi_full_device's default is attotime::zero, "immediate": every byte of a
// data phase is handed over inside one step() recursion, so the whole transfer
// happens between two instructions of the NUPI's 68000. The NUPI cannot work
// that way. Its FIFO-to-NuBus drain is driven by a real timer (see
// dma_drain_timer_expired() in explorer_nupi.cpp), so an instantaneous data
// phase leaves the drain nothing to carry, the firmware finds no data moved,
// and it completes the command with controller error >84,
// NUPI-COMPLETE-WITHOUT-DATA-TRANSFER - which the boot PROM prints as
// "DEVICE ERROR: 60840000" (>6.. is complete+error, and the low byte of the
// status high halfword is the controller error; see Table A-1 in Introduction
// to the Explorer System, and DECODE-NUPI-STATUS in the system sources'
// disk-io/disk-io.lisp).
//
// The rate itself is not critical and is not the real drive's: a real QIC
// cartridge streams an order of magnitude slower than this. What matters is
// that it is finite and that it stays under the ceiling the NUPI's FIFO/DMA
// path can sustain, so it is simply the same 1.25MB/s explorer_msu_device
// uses - see the note there, which measured where that ceiling is.
attotime explorer_tape_device::scsi_data_byte_period()
{
	return attotime::from_ticks(1, 1'250'000);
}
