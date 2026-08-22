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

	void set_ti_nubus(ti_nubus_device *nubus, const char *slottag, int slot)
	{
		m_nubus = nubus;
		m_nubus_slottag = slottag;
		m_slot = slot;
	}

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

	template <typename T> void set_space(T &&tag, int spacenum) { m_space.set_tag(std::forward<T>(tag), spacenum); }
	template <typename T> void set_local_bus_space(T &&tag, int spacenum) { m_local_bus_space.set_tag(std::forward<T>(tag), spacenum); }

	void add_ti_nubus_card(device_ti_nubus_card_interface &card);

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

	required_address_space m_space;
	required_address_space m_local_bus_space;

	std::vector<std::reference_wrapper<device_ti_nubus_card_interface>> m_device_list;
};

DECLARE_DEVICE_TYPE(TI_NUBUS, ti_nubus_device)

#endif // MAME_TI_TI_NUBUS_H
