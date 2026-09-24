// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/***************************************************************************

  Texas Instruments Explorer

The Explorer uses a NuBus backplane with a higher speed local bus for slots 3-6.
The boards which could be connected to the local bus are:
- CPU board
- System interface controller board (monitor, keyboard, mouse, printer, rs232c)
- Memory board(s)

Slot 6 is reserved for the CPU board.
Slot 5 is reserved for the system interface controller board.
Slots 3 and 4 are reserved for memory boards.

TODO:
- Tape support
- The Ethernet board talks to no network - see explorer_enet.cpp.
- Mass Storage Controller, interfacing SMD or SCSI (no schematics or dumps).
- Odyssey Coprocessor (no schematics or dumps).
- ...

***************************************************************************/

#include "emu.h"
#include "ti_nubus.h"
#include "explorer_cpu.h"
#include "explorer_enet.h"
#include "explorer_mem.h"
#include "explorer_nupi.h"
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

void tiexplorer_cpu_cards(device_slot_interface &device)
{
	device.option_add("cpu", EXPLORER_CPU);

}


class tiexplorer_state : public driver_device
{
public:
	tiexplorer_state(const machine_config &mconfig, device_type type, const char *tag) :
		driver_device(mconfig, type, tag),
		m_nubus(*this, "nubus")
	{ }

	void tiexplorer(machine_config &config);

private:
	required_device<ti_nubus_device> m_nubus;
};


static INPUT_PORTS_START(tiexplorer)
INPUT_PORTS_END


void tiexplorer_state::tiexplorer(machine_config &config)
{
	TI_NUBUS(config, m_nubus);

	// Required / local bus card slots
	TI_NUBUS_SLOT(config, "nb6", m_nubus, 6, tiexplorer_cpu_cards, "cpu");
	TI_NUBUS_SLOT(config, "nb5", m_nubus, 5, tiexplorer_nubus_local_bus_cards, "sib");
	TI_NUBUS_SLOT(config, "nb4", m_nubus, 4, tiexplorer_nubus_local_bus_cards, "mem8mb");
	TI_NUBUS_SLOT(config, "nb3", m_nubus, 3, tiexplorer_nubus_local_bus_cards, nullptr);

	// Other cards
	TI_NUBUS_SLOT(config, "nb2", m_nubus, 2, tiexplorer_nubus_cards, "nupi");
	TI_NUBUS_SLOT(config, "nb1", m_nubus, 1, tiexplorer_nubus_cards, nullptr);
	TI_NUBUS_SLOT(config, "nb0", m_nubus, 0, tiexplorer_nubus_cards, "enet");
}


ROM_START(tiexplorer)
	// Everything this machine needs is on its boards
ROM_END


} // anonymous namespace

COMP(1985, tiexplorer, 0, 0, tiexplorer, tiexplorer, tiexplorer_state, empty_init, "Texas Instruments", "Explorer", MACHINE_NOT_WORKING)
