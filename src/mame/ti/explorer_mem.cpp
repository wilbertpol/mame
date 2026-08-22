// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/**********************************************************************

    TI Explorer NuBus memory board.

**********************************************************************/

#include "emu.h"
#include "explorer_mem.h"


DEFINE_DEVICE_TYPE(EXPLORER_MEM8MB, explorer_mem8mb_device, "explorer_mem8mb", "TI Explorer 8MB Memory Board (2243910-0003)")
DEFINE_DEVICE_TYPE(EXPLORER_MEM2MB, explorer_mem2mb_device, "explorer_mem2mb", "TI Explorer 2MB Memory Board (2243910-0001)")


explorer_mem_device_base::explorer_mem_device_base(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, u32 clock, u32 ram_size) :
	device_t(mconfig, type, tag, owner, clock),
	device_ti_nubus_card_interface(mconfig, *this),
	m_ram_size(ram_size),
	m_ram_view(*this, "ram_view"),
	m_ram_view_local_bus(*this, "ram_view_local_bus")
{
}


void explorer_mem_device_base::device_start()
{
	m_ram = std::make_unique<u8[]>(m_ram_size);
	save_pointer(NAME(m_ram), m_ram_size);

	u32 const parity_size = (m_ram_size + 7) / 8;
	m_parity = std::make_unique<u8[]>(parity_size);
	save_pointer(NAME(m_parity), parity_size);

	offs_t const base = get_slotspace();
	nubus().space().install_view(base, base + m_ram_size - 1, m_ram_view);
	m_ram_view[0].install_readwrite_handler(base, base + m_ram_size - 1, read8sm_delegate(*this, FUNC(explorer_mem_device_base::ram_r)), write8sm_delegate(*this, FUNC(explorer_mem_device_base::ram_w)));
	m_ram_view[1].install_readwrite_handler(base, base + m_ram_size - 1, read8sm_delegate(*this, FUNC(explorer_mem_device_base::ram_test_r)), write8sm_delegate(*this, FUNC(explorer_mem_device_base::ram_test_w)));

	nubus().install_map(*this, &explorer_mem_device_base::nubus_map);

	nubus().local_bus_space().install_view(base, base + m_ram_size - 1, m_ram_view_local_bus);
	m_ram_view_local_bus[0].install_readwrite_handler(base, base + m_ram_size - 1, read8sm_delegate(*this, FUNC(explorer_mem_device_base::ram_r)), write8sm_delegate(*this, FUNC(explorer_mem_device_base::ram_w)));
	m_ram_view_local_bus[1].install_readwrite_handler(base, base + m_ram_size - 1, read8sm_delegate(*this, FUNC(explorer_mem_device_base::ram_test_r)), write8sm_delegate(*this, FUNC(explorer_mem_device_base::ram_test_w)));

	save_item(NAME(m_config_register));
	save_item(NAME(m_base_register));
	save_item(NAME(m_failure_location));
	save_item(NAME(m_test_register));
	save_item(NAME(m_nubus_status));
}


void explorer_mem_device_base::device_reset()
{
	m_config_register = 0;
	m_base_register = 0;
	m_failure_location = 0;
	m_test_register = 0;
	m_failure_latch = 0;
	m_nubus_status = 0;
	m_ram_view.select(0);
	m_ram_view_local_bus.select(0);
}


void explorer_mem_device_base::nubus_map(address_map &map)
{
	map.unmap_value_high();

	map(0xffc000, 0xffc000).rw(FUNC(explorer_mem_device_base::config_register_r), FUNC(explorer_mem_device_base::config_register_w));
	map(0xffc008, 0xffc008).rw(FUNC(explorer_mem_device_base::base_register_r), FUNC(explorer_mem_device_base::base_register_w));
	map(0xffc010, 0xffc010).r(FUNC(explorer_mem_device_base::failure_latch_r));
	map(0xffc011, 0xffc011).rw(FUNC(explorer_mem_device_base::test_register_r), FUNC(explorer_mem_device_base::test_register_w));
	map(0xffc014, 0xffc015).lr16(NAME([this] () {
		return u16(m_nubus_status);
	}));

	map(0xffe000, 0xffefff).rom().region("memory_config", 0x1000);
	map(0xfff000, 0xffffff).rom().region("memory_config", 0x0000);
}


// 76543-1- - unused
// -----2-- - test LED
// -------0 - board reset
u8 explorer_mem_device_base::config_register_r()
{
	return m_config_register;
}

void explorer_mem_device_base::config_register_w(u8 data)
{
	m_config_register = data & 0x05;
	if (BIT(m_config_register, 0))
	{
		// Board reset
		m_test_register = 0;
		m_base_register = 0xf4; // 0xf3 when the card is in slot 3.
		m_ram_view.select(0);
		m_ram_view_local_bus.select(0);
	}
	// TODO output led status
}

u8 explorer_mem_device_base::base_register_r()
{
	return m_base_register;
}

void explorer_mem_device_base::base_register_w(u8 data)
{
	m_base_register = data;
}

u8 explorer_mem_device_base::failure_latch_r()
{
	return m_failure_location;
}

u8 explorer_mem_device_base::test_register_r()
{
	return m_test_register;
}

void explorer_mem_device_base::test_register_w(u8 data)
{
	m_test_register = data;
	m_ram_view.select(BIT(m_test_register, 4));
	m_ram_view_local_bus.select(BIT(m_test_register, 4));
}


u8 explorer_mem_device_base::calculate_parity(u8 data)
{
	data ^= data >> 4;
	data ^= data >> 2;
	data ^= data >> 1;
	return data & 1;
}

u8 explorer_mem_device_base::get_parity(offs_t offset) const
{
	return BIT(m_parity[offset >> 3], offset & 7);
}

void explorer_mem_device_base::store_parity_bit(offs_t offset, u8 bit)
{
	if (bit)
		m_parity[offset >> 3] |= (1 << (offset & 7));
	else
		m_parity[offset >> 3] &= ~(1 << (offset & 7));
}

u8 explorer_mem_device_base::test_force_bit(offs_t offset) const
{
	return BIT(m_test_register, offset & 3) ^ 1;
}

void explorer_mem_device_base::update_failure_location(offs_t offset, bool failed)
{
	m_failure_location = ((offset & 0x03) << 5) | (failed ? 0x10 : 0x0f);

	if (failed)
		nubus().assert_bus_error();
}

u8 explorer_mem_device_base::ram_r(offs_t offset)
{
	u8 const data = m_ram[offset];
	update_failure_location(offset, get_parity(offset) != calculate_parity(data));
	return data;
}

void explorer_mem_device_base::ram_w(offs_t offset, u8 data)
{
	m_ram[offset] = data;
	store_parity_bit(offset, calculate_parity(data));
}

u8 explorer_mem_device_base::ram_test_r(offs_t offset)
{
	u8 const data = m_ram[offset];
	update_failure_location(offset, test_force_bit(offset) != calculate_parity(data));
	return data;
}

void explorer_mem_device_base::ram_test_w(offs_t offset, u8 data)
{
	m_ram[offset] = data;
	store_parity_bit(offset, test_force_bit(offset));
}


//**************************************************************************
//  8MB board (256K-bit DRAM chips) - confirmed part number 2243910-0003
//**************************************************************************

explorer_mem8mb_device::explorer_mem8mb_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock) :
	explorer_mem_device_base(mconfig, EXPLORER_MEM8MB, tag, owner, clock, 0x800000)
{
}

ROM_START(explorer_mem8mb)
	ROM_REGION32_BE(0x2000, "memory_config", ROMREGION_ERASE00)
	// board part number "2243910-0003" (board type "MEM", vendor "TIAU"
	ROMX_LOAD("2243924-2_27s291_8mb.bin", 0x003, 0x800, CRC(6f699641) SHA1(f65ba4a2672bc5c90040da26237f1d14baac3370), ROM_SKIP(3))
ROM_END

const tiny_rom_entry *explorer_mem8mb_device::device_rom_region() const
{
	return ROM_NAME(explorer_mem8mb);
}


//**************************************************************************
//  2MB board (64K-bit DRAM chips) - part number/ROM not confirmed, see header
//**************************************************************************

explorer_mem2mb_device::explorer_mem2mb_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock) :
	explorer_mem_device_base(mconfig, EXPLORER_MEM2MB, tag, owner, clock, 0x200000)
{
}

ROM_START(explorer_mem2mb)
	ROM_REGION32_BE(0x2000, "memory_config", ROMREGION_ERASE00)
	ROMX_LOAD("2243910-1_27s291_2mb.bin", 0x003, 0x800, NO_DUMP, ROM_SKIP(3))
ROM_END

const tiny_rom_entry *explorer_mem2mb_device::device_rom_region() const
{
	return ROM_NAME(explorer_mem2mb);
}
