// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/**********************************************************************

    TI Explorer CPU board.

    Board part number 2243895-0001, board type "CPU", vendor "TIAU".
    Carries the raven microcoded processor, its 16K x 56-bit writable
    control store, the boot PROMs holding the static microcode, and the
    board's configuration ROM.

    It is a NuBus card like every other board in the machine, and the
    driver puts it in slot 6 ("Slot 6 is reserved for the CPU board" -
    see explorer.cpp). What makes it unusual is that the NuBus and the
    local bus *are* this board's own address spaces: the raven's AS_DATA
    is the NuBus and AS_LOCAL_BUS is the local bus, which is why the
    backplane (ti_nubus_device) is pointed at this board's CPU rather
    than the other way round. Both spaces are left entirely to the cards
    on them - the processor itself handles a cycle no card answers, since
    neither bus has a signal for that (see raven.cpp's data_map()).

**********************************************************************/

#ifndef MAME_TI_EXPLORER_CPU_H
#define MAME_TI_EXPLORER_CPU_H

#pragma once

#include "ti_nubus.h"

#include "cpu/raven/raven.h"


class explorer_cpu_device : public device_t, public device_ti_nubus_card_interface
{
public:
	explorer_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;

	// device_ti_nubus_card_interface implementation
	virtual void assert_bus_error() override;

private:
	required_device<raven_cpu_device> m_cpu;
	// The seven boot PROMs as dumped (one bit-slice per file, interleaved), and
	// the control store the raven actually executes from - see device_start()
	// for the descramble between them.
	required_memory_region m_microcode_proms;
	required_memory_region m_control_store;

	// This board's own NuBus slot window (>Fs'xx000000), installed the same way
	// every other card installs its own.
	void nubus_map(address_map &map) ATTR_COLD;
};

DECLARE_DEVICE_TYPE(EXPLORER_CPU, explorer_cpu_device)

#endif // MAME_TI_EXPLORER_CPU_H
