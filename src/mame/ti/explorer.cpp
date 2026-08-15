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

***************************************************************************/

#include "emu.h"
#include "cpu/raven/raven.h"
#include "ti_nubus.h"
#include "nupi.h"
#include "emupal.h"
#include "screen.h"


namespace {

void tiexplorer_nubus_cards(device_slot_interface &device)
{
	device.option_add("nupi", NUPI);
}

class tiexplorer_state : public driver_device
{
public:
	tiexplorer_state(const machine_config &mconfig, device_type type, const char *tag) :
        driver_device(mconfig, type, tag),
        m_maincpu(*this, "maincpu"),
        m_nubus(*this, "nubus")
    { }

    void tiexplorer(machine_config &config);

protected:
	void machine_start() override ATTR_COLD;

private:
	required_device<raven_cpu_device> m_maincpu;
	required_device<ti_nubus_device> m_nubus;

	u32 screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)\
	{
		return 0;
	}
	void mem_map(address_map &map);
};


static INPUT_PORTS_START(tiexplorer)
INPUT_PORTS_END


void tiexplorer_state::mem_map(address_map &map)
{
	map.unmap_value_high();

	// Catch-all, lowest priority (declared first - later, more specific entries below and
	// NuBus cards' dynamically-installed slot windows override it): any address with
	// nothing real behind it reports a NuBus error, matching what a real bus timeout
	// would produce. Self-test relies on this to detect and skip absent NuBus cards.
	map(0x00000000, 0xffffffff).rw(m_maincpu, FUNC(raven_cpu_device::nubus_unmapped_r), FUNC(raven_cpu_device::nubus_unmapped_w));

	// NuBus cards (e.g. NUPI in slot 2) install their own 16 MB slot windows directly into
	// this space via m_nubus->install_map(), see tiexplorer().

	// Slot 3 - RAM
	map(0xf3000000, 0xf31fffff).ram(); // 2MB
	map(0xf3ffe000, 0xf3ffefff).rom().region("memory_config", 0x1000);
	map(0xf3fff000, 0xf3ffffff).rom().region("memory_config", 0x0000);

	// Slot 6 - CPU
	map(0xf6c00000, 0xf6c00003).r(m_maincpu, FUNC(raven_cpu_device::nubus_flag_r));
	map(0xf6e00000, 0xf6e0003f).w(m_maincpu, FUNC(raven_cpu_device::irq_w));
	map(0xf6fffc00, 0xf6ffffff).rom().region("cpu_config", 0);
}


void tiexplorer_state::tiexplorer(machine_config &config)
{
	RAVEN(config, m_maincpu, 28_MHz_XTAL);
	m_maincpu->set_addrmap(AS_DATA, &tiexplorer_state::mem_map);

	TI_NUBUS(config, m_nubus);
	m_nubus->set_space(m_maincpu, AS_DATA);

	// Slots 3-6 are reserved for memory/SIB/CPU boards (wired directly above rather than
	// through the slot mechanism); slot 2 hosts the NUPI mass storage interface.
	TI_NUBUS_SLOT(config, "nb2", "nubus", 2, tiexplorer_nubus_cards, "nupi");

	// TODO
	screen_device &screen(SCREEN(config, "screen"));
	screen.set_raw(6.144_MHz_XTAL, 515, 0, 160 /*480*/, 199, 0, 152);
	screen.set_screen_update(FUNC(tiexplorer_state::screen_update));
	screen.set_palette("palette");

	PALETTE(config, "palette", palette_device::MONOCHROME_HIGHLIGHT);
}


ROM_START(tiexplorer)

	ROM_REGION64_BE(0x4000, "maincpu", ROMREGION_ERASE00)

	ROM_REGION(0x4000, "microcode_proms", ROMREGION_ERASE00)
	ROMX_LOAD("2236480-03_microcode.bin", 0x0000, 0x0800, CRC(e54001de) SHA1(cad8e4e0071cf4d010c3aa06d09547f935ab2eca), ROM_SKIP(7))
	ROMX_LOAD("2236481-03_microcode.bin", 0x0001, 0x0800, CRC(1bbba705) SHA1(a3a2b8b5a54a235a40ec9651db36b46e14d141eb), ROM_SKIP(7))
	ROMX_LOAD("2236482-03_microcode.bin", 0x0002, 0x0800, CRC(c2fba197) SHA1(2c34e1c77a2db848883bc50384910a31a98ef0b4), ROM_SKIP(7))
	ROMX_LOAD("2236483-03_microcode.bin", 0x0003, 0x0800, CRC(eaee4d54) SHA1(b10a0a92bf47c635f6e5e9cc8bbb3326aa6e1187), ROM_SKIP(7))
	ROMX_LOAD("2236484-03_microcode.bin", 0x0004, 0x0800, CRC(9494631b) SHA1(493eb9bd4a8b077d614e1379c3e76d7078c958f2), ROM_SKIP(7))
	ROMX_LOAD("2236485-03_microcode.bin", 0x0005, 0x0800, CRC(aebd8fd0) SHA1(8507d581cf81c45089a16257711d824c44ae50fa), ROM_SKIP(7))
	ROMX_LOAD("2236486-03_microcode.bin", 0x0006, 0x0800, CRC(8a953a12) SHA1(f10ce4f53a65da5d133489d4f43b1c7f4ec5726d), ROM_SKIP(7))

	ROM_REGION32_BE(0x400, "cpu_config", ROMREGION_ERASE00)
	ROMX_LOAD("cpu_config.bin", 0x000, 0x100, NO_DUMP, ROM_SKIP(3)) // not present in any known dump; content unknown

	ROM_REGION32_BE(0x2000, "memory_config", ROMREGION_ERASE00)
	ROMX_LOAD("2243924-2_27s291_8mb.bin", 0x000, 0x800, CRC(6f699641) SHA1(f65ba4a2672bc5c90040da26237f1d14baac3370), ROM_SKIP(3))

ROM_END


void tiexplorer_state::machine_start()
{
	u8 *source = memregion("microcode_proms")->base();
	u8 *dest = memregion("maincpu")->base();

	// Reverse order of roms
	for (int i = 0; i < 0x4000; i += 8)
	{
		for (int j = 0; j < 7; j++)
		{
			dest[0x4000 - 8 - i + j] = source[i + j];
		}
	}

	// Memory config
//	u8 *mem_cfg_src = memregion("memory_config_rom")->base();
//	u8 *mem_cfg_dest = memregion("memory_config")->base();
//	for (int i = 0; i < 0x800; i++)
//	{
//		mem_cfg_dest[(i << 2) + 3] = mem_cfg_src[i ^ 0x400];
////		mem_cfg_dest[(i << 2) + 1] = mem_cfg_src[i ^ 0x400];
////		mem_cfg_dest[(i << 2) + 2] = mem_cfg_src[i ^ 0x400];
////		mem_cfg_dest[(i << 2) + 3] = mem_cfg_src[i ^ 0x400];
//	}
}

} // anonymous namespace

COMP(1985, tiexplorer, 0, 0, tiexplorer, tiexplorer, tiexplorer_state, empty_init, "Texas Instruments", "Explorer", MACHINE_NOT_WORKING | MACHINE_NO_SOUND)
