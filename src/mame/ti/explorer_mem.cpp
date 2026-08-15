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
	m_ram_size(ram_size)
{
}


void explorer_mem_device_base::device_start()
{
	m_ram = std::make_unique<u8[]>(m_ram_size);
	save_pointer(NAME(m_ram), m_ram_size);

	offs_t const base = get_slotspace();
	nubus().space().install_ram(base, base + m_ram_size - 1, m_ram.get());

	nubus().install_map(*this, &explorer_mem_device_base::nubus_map);
}


void explorer_mem_device_base::nubus_map(address_map &map)
{
	map.unmap_value_high();

	map(0xffe000, 0xffefff).rom().region("memory_config", 0x1000);
	map(0xfff000, 0xffffff).rom().region("memory_config", 0x0000);
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
