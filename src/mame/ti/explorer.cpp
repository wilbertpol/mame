// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/***************************************************************************

  Texas Instruments Explorer

The Explorer uses a NuBus backplane combined with a local bus.
The boards that could be connected to the local bus are:
- CPU board
- System interface controller boad (monitor, keyboard, mouse, printer, rs232c)
- Memory board(s)

Slot 6 is reserved for the CPU board.
Slot 5 is reserved for the system interface controller board.
Slots 3 and 4 are reserved for memory boards.
Slots 3-6 are connected through a local bus but are also accessible through
the nubus like all the other slots.

TODO:
- The Ethernet board talks to no network - see explorer_enet.cpp.
- Mass Storage Controller, interfacing SMD or SCSI (no schematics or dumps).
- Odyssey Coprocessor (no schematics or dumps).

***************************************************************************/

#include "emu.h"
#include "cpu/tiexp/exp1proc.h"
#include "ti_nubus.h"
#include "explorer_cpu.h"
#include "explorer_enet.h"
#include "explorer_nupi.h"
#include "explorer_mem.h"
#include "explorer_sib.h"


namespace {

void tiexplorer_nubus_local_bus_cards(device_slot_interface &device)
{
	device.option_add("mem8mb", EXPLORER_MEM8MB);
	device.option_add("mem2mb", EXPLORER_MEM2MB);
	device.option_add("sib", SIB);
}

void tiexplorer_nubus_cards(device_slot_interface &device)
{
	device.option_add("enet", EXPLORER_ENET);
	device.option_add("nupi", NUPI);
}

// Slot 6 only ever holds the CPU board - the machine has no processor without
// it, and the NuBus/local bus it drives are that board's own address spaces.
void tiexplorer_cpu_cards(device_slot_interface &device)
{
	device.option_add("cpu", EXPLORER_CPU);
}

class tiexplorer_state : public driver_device
{
public:
	tiexplorer_state(const machine_config &mconfig, device_type type, const char *tag) :
		driver_device(mconfig, type, tag),
		m_maincpu(*this, "nb6:cpu:maincpu"),
		m_nubus(*this, "nubus")
	{ }

	void tiexplorer(machine_config &config);

private:
	// The CPU lives on the board in slot 6 (see explorer_cpu.h), so the driver
	// reaches it through the slot. This is here only to hand the backplane the
	// two address spaces that board provides - see tiexplorer() below.
	required_device<exp1proc_cpu_device> m_maincpu;
	required_device<ti_nubus_device> m_nubus;
};


static INPUT_PORTS_START(tiexplorer)
INPUT_PORTS_END


void tiexplorer_state::tiexplorer(machine_config &config)
{
	TI_NUBUS(config, m_nubus);

	// Reserved / local bus card slots
	TI_NUBUS_SLOT(config, "nb6", "nubus", 6, tiexplorer_cpu_cards, "cpu");
	TI_NUBUS_SLOT(config, "nb5", "nubus", 5, tiexplorer_nubus_local_bus_cards, "sib");
	TI_NUBUS_SLOT(config, "nb4", "nubus", 4, tiexplorer_nubus_local_bus_cards, "mem8mb");
	TI_NUBUS_SLOT(config, "nb3", "nubus", 3, tiexplorer_nubus_local_bus_cards, nullptr);

	// Other cards
	TI_NUBUS_SLOT(config, "nb2", "nubus", 2, tiexplorer_nubus_cards, "nupi");
	TI_NUBUS_SLOT(config, "nb1", "nubus", 1, tiexplorer_nubus_cards, nullptr);

	// The Ethernet board passes its power-up self-test and all nine of its
	// extended subtests, and the system boots with it fitted. See
	// explorer_enet.cpp for what it does and does not do.
	TI_NUBUS_SLOT(config, "nb0", "nubus", 0, tiexplorer_nubus_cards, "enet");

	// The backplane has no address spaces of its own: the NuBus and the local
	// bus are the CPU board's AS_DATA and AS_LOCAL_BUS, so point it at the
	// processor in slot 6. Every card then installs its own slot window into
	// those spaces from its device_start().
	m_nubus->set_space(m_maincpu, AS_DATA);
	m_nubus->set_local_bus_space(m_maincpu, exp1proc_cpu_device::AS_LOCAL_BUS);
}


ROM_START(tiexplorer)
	// Everything this machine needs is on its boards - see explorer_cpu.cpp for
	// the CPU board's microcode PROMs and configuration ROM.
ROM_END


} // anonymous namespace

COMP(1985, tiexplorer, 0, 0, tiexplorer, tiexplorer, tiexplorer_state, empty_init, "Texas Instruments", "Explorer", MACHINE_NOT_WORKING)
