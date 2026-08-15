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
	m_space(*this, finder_base::DUMMY_TAG, -1)
{
}

void ti_nubus_device::device_start()
{
}

void ti_nubus_device::add_ti_nubus_card(device_ti_nubus_card_interface &card)
{
	m_device_list.emplace_back(card);
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
