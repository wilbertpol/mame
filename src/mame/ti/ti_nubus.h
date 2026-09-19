// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/**********************************************************************

    TI Explorer NuBus slot and card interface.

**********************************************************************/

#ifndef MAME_TI_TI_NUBUS_H
#define MAME_TI_TI_NUBUS_H

#pragma once

#include <vector>
#include <functional>


class ti_nubus_device;

class device_ti_nubus_card_interface : public device_interface
{
	friend class ti_nubus_device;
public:
	virtual ~device_ti_nubus_card_interface();

	u32 get_slotspace() const { return 0xf000'0000 | (u32(m_slot) << 24); }

	// Only slots 3 through 6 are wired to the local bus that ties the CPU,
	// memory and SIB boards together (see explorer.cpp's header); a card in any
	// lower slot reaches the processor over the NuBus alone. A board that can
	// sit on either - the memory board can legitimately be put in slot 2 - must
	// consult this rather than assume, and install_local_bus_map() below
	// enforces it for the cards that use the helper.
	static constexpr int FIRST_LOCAL_BUS_SLOT = 3;
	bool on_local_bus() const { return m_slot >= FIRST_LOCAL_BUS_SLOT; }

	void set_ti_nubus(ti_nubus_device *nubus, const char *slottag, int slot)
	{
		m_nubus = nubus;
		m_nubus_slottag = slottag;
		m_slot = slot;
	}

	// Raised when a bus cycle finds nothing at its target address. The
	// bus-error line is not the backplane's - it lives on the board that
	// provides the bus's address spaces, i.e. the CPU board, which is the only
	// card that implements this and registers itself as the target via
	// ti_nubus_device::set_bus_master_card().
	virtual void assert_bus_error() { }

protected:
	device_ti_nubus_card_interface(const machine_config &mconfig, device_t &device);
	virtual void interface_pre_start() override;

	int slotno() const { assert(m_nubus); return m_slot; }
	ti_nubus_device &nubus() { assert(m_nubus); return *m_nubus; }

private:
	ti_nubus_device *m_nubus;
	const char *m_nubus_slottag;
	int m_slot;
};


class ti_nubus_slot_device : public device_t, public device_single_card_slot_interface<device_ti_nubus_card_interface>
{
public:
	template <typename T, typename U>
	ti_nubus_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, T &&nbtag, int slot, U &&opts, const char *dflt)
		: ti_nubus_slot_device(mconfig, tag, owner, (u32)0)
	{
		set_options(std::forward<U>(opts), dflt, false);
		set_ti_nubus_slot(std::forward<T>(nbtag), tag, slot);
	}

	ti_nubus_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	template <typename T>
	void set_ti_nubus_slot(T &&tag, const char *slottag, int slot)
	{
		m_nubus.set_tag(std::forward<T>(tag));
		m_nubus_slottag = slottag;
		m_slot = slot;
	}

protected:
	// device_t implementation
	virtual void device_resolve_objects() override ATTR_COLD;
	virtual void device_start() override ATTR_COLD;

	required_device<ti_nubus_device> m_nubus;
	const char *m_nubus_slottag;
	int m_slot;
};

DECLARE_DEVICE_TYPE(TI_NUBUS_SLOT, ti_nubus_slot_device)


class ti_nubus_device : public device_t
{
public:
	ti_nubus_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	void add_ti_nubus_card(device_ti_nubus_card_interface &card);

	// Nominate the card that drives the bus. The backplane has no address
	// spaces of its own - the NuBus and the local bus are the bus master's,
	// which on this machine means the CPU board's processor - and the
	// bus-error line is that board's too, see
	// device_ti_nubus_card_interface::assert_bus_error().
	//
	// The card calls this from its own device_resolve_objects(), which is late
	// enough for the address spaces to exist and early enough that every card's
	// device_start() can install into them. Doing it this way rather than
	// through an address-space finder is what keeps the driver out of it: a
	// finder would need its tag set at machine-configuration time, so the
	// driver would have to know which slot holds the bus master and what its
	// processor is called.
	void set_bus_master_card(device_ti_nubus_card_interface &card, address_space &space, address_space &local_bus_space)
	{
		m_bus_master_card = &card;
		m_space = &space;
		m_local_bus_space = &local_bus_space;
	}

	template <typename T>
	void install_map(T &device, void (T::*map)(address_map &map))
	{
		const offs_t start = device.get_slotspace();
		const offs_t end = start + 0x00ff'ffff;

		m_space->install_device(start, end, device, map);
	}

	template <typename T>
	void install_local_bus_map(T &device, void (T::*map)(address_map &map))
	{
		// Silently nothing to do for a card that is not on the local bus - that
		// is a legal machine, not a mistake, and the card simply is not wired to
		// this bus. See device_ti_nubus_card_interface::on_local_bus().
		if (!device.on_local_bus())
			return;

		const offs_t start = device.get_slotspace();
		const offs_t end = start + 0x00ff'ffff;

		m_local_bus_space->install_device(start, end, device, map);
	}

	address_space &space() { return *m_space; }

	address_space &local_bus_space() { return *m_local_bus_space; }

	void assert_bus_error();

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;

	address_space *m_space;
	address_space *m_local_bus_space;

	std::vector<std::reference_wrapper<device_ti_nubus_card_interface>> m_device_list;
	device_ti_nubus_card_interface *m_bus_master_card;
};

DECLARE_DEVICE_TYPE(TI_NUBUS, ti_nubus_device)

#endif // MAME_TI_TI_NUBUS_H
