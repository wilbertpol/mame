// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/**********************************************************************

    TI Explorer NuBus slot and card interface.

    The Explorer processor's own AS_DATA address space *is* the NuBus
    address space (there is no separate host-side window to bridge, unlike
    the Macintosh II family's nubus_device). A slot occupies a 16 MB window
    at >FSxxxxxx, where S is the (hardwired, per physical slot) 4-bit slot
    ID, matching the "Explorer Processor Auxiliary Board" and NUPI General
    Description manuals. Unlike Apple's NuBus, slot IDs are not restricted
    to 9-E; the Explorer's own CPU/SIB/memory boards are documented using
    slots 3-6.

    Also unlike the Mac's per-slot IRQ lines, a NuBus card on the Explorer
    interrupts the CPU board by becoming a NuBus master and writing to one
    of the CPU board's own event-posting addresses (>FSE0000-3C, in the
    CPU's own slot space) - so this bus has no IRQ line plumbing of its
    own; a card that needs to interrupt just writes through nubus().space().

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

	// 16 MB slot window base: >FSxxxxxx with S = slot number (0-15)
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

	void add_ti_nubus_card(device_ti_nubus_card_interface &card);

	// Installs a card's own address_map over its full 16 MiB slot space, >Fs00'0000-Fs'FFFFFF.
	template <typename T>
	void install_map(T &device, void (T::*map)(address_map &map))
	{
		const offs_t start = device.get_slotspace();
		const offs_t end = start + 0x00ff'ffff;

		m_space->install_device(start, end, device, map);
	}

	// Direct access to the shared bus address space, for cards that need to act as a NuBus
	// master (DMA into system RAM, or posting an interrupt event to another board).
	address_space &space() { return *m_space; }

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;

	required_address_space m_space;

	std::vector<std::reference_wrapper<device_ti_nubus_card_interface>> m_device_list;
};

DECLARE_DEVICE_TYPE(TI_NUBUS, ti_nubus_device)

#endif // MAME_TI_TI_NUBUS_H
