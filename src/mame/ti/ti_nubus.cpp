// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/**********************************************************************

    TI Explorer NuBus slot and card interface.

**********************************************************************/

#include "emu.h"
#include "ti_nubus.h"


DEFINE_DEVICE_TYPE(TI_NUBUS_SLOT, ti_nubus_slot_device, "ti_nubus_slot", "TI Explorer NuBus slot")

ti_nubus_slot_device::ti_nubus_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock) :
	device_t(mconfig, TI_NUBUS_SLOT, tag, owner, clock),
	device_single_card_slot_interface(mconfig, *this),
	m_nubus(*this, finder_base::DUMMY_TAG),
	m_nubus_slottag(nullptr),
	m_slot(0)
{
}

void ti_nubus_slot_device::device_resolve_objects()
{
	device_ti_nubus_card_interface *const dev = get_card_device();

	if (dev)
	{
		dev->set_ti_nubus(m_nubus.target(), m_nubus_slottag, m_slot);
		m_nubus->add_ti_nubus_card(*dev);
	}
}

void ti_nubus_slot_device::device_start()
{
}


DEFINE_DEVICE_TYPE(TI_NUBUS, ti_nubus_device, "ti_nubus", "TI Explorer NuBus")

ti_nubus_device::ti_nubus_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock) :
	device_t(mconfig, TI_NUBUS, tag, owner, clock),
	m_space(nullptr),
	m_local_bus_space(nullptr),
	m_bus_master_card(nullptr)
{
}

void ti_nubus_device::device_start()
{
	// Some card has to drive the buses, and on this machine that is always the
	// CPU board in slot 6 - without it there is no processor either. The
	// backplane is configured before any slot, so it starts first and can say
	// so plainly here, rather than letting the first card to install a map
	// crash on a null space.
	if (!m_space || !m_local_bus_space)
		fatalerror("No card has claimed the TI Explorer NuBus - slot 6 needs the CPU board\n");
}

void ti_nubus_device::add_ti_nubus_card(device_ti_nubus_card_interface &card)
{
	m_device_list.emplace_back(card);
}

void ti_nubus_device::assert_bus_error()
{
	// Straight to the board that owns the line. This used to reach into
	// m_space->device() and downcast it to exp1proc_cpu_device, which baked two
	// assumptions into the backplane: that the bus master is whatever device
	// happens to own AS_DATA, and that it is an exp1proc. Both belong to the CPU
	// board (see explorer_cpu.cpp), which nominates itself as bus master.
	//
	// Nothing to do if no card claimed the line: a machine with no CPU board
	// has no bus cycles to fail in the first place.
	if (m_bus_master_card)
		m_bus_master_card->assert_bus_error();
}


device_ti_nubus_card_interface::device_ti_nubus_card_interface(const machine_config &mconfig, device_t &device) :
	device_interface(device, "ti_nubus"),
	m_nubus(nullptr),
	m_nubus_slottag(nullptr),
	m_slot(0)
{
}

device_ti_nubus_card_interface::~device_ti_nubus_card_interface()
{
}

void device_ti_nubus_card_interface::interface_pre_start()
{
	if (!m_nubus)
		fatalerror("Can't find TI NuBus device\n");

	if (m_slot < 0 || m_slot > 0xf)
		fatalerror("Slot %x out of range for TI Explorer NuBus\n", m_slot);
}
