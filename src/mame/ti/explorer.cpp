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
- Add distinction between nubus and local bus.

***************************************************************************/

#include "emu.h"
#include "cpu/raven/raven.h"
#include "ti_nubus.h"
#include "nupi.h"
#include "explorer_mem.h"
#include "sib.h"


namespace {

void tiexplorer_nubus_cards(device_slot_interface &device)
{
	device.option_add("nupi", NUPI);
	device.option_add("mem8mb", EXPLORER_MEM8MB);
	device.option_add("mem2mb", EXPLORER_MEM2MB);
	device.option_add("sib", SIB);
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

	map(0x00000000, 0xffffffff).rw(m_maincpu, FUNC(raven_cpu_device::nubus_unmapped_r), FUNC(raven_cpu_device::nubus_unmapped_w));

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

	TI_NUBUS_SLOT(config, "nb2", "nubus", 2, tiexplorer_nubus_cards, "nupi");
//	TI_NUBUS_SLOT(config, "nb3", "nubus", 3, tiexplorer_nubus_cards, "mem8mb");
	TI_NUBUS_SLOT(config, "nb4", "nubus", 4, tiexplorer_nubus_cards, "mem8mb");
	TI_NUBUS_SLOT(config, "nb5", "nubus", 5, tiexplorer_nubus_cards, "sib");

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

	// board part number "2243895-0001", board type "CPU", vendor "TIAU"
	ROM_REGION32_BE(0x400, "cpu_config", ROMREGION_ERASE00)
	ROMX_LOAD("cpu_config.bin", 0x003, 0x100, CRC(f3513e0c) SHA1(10217eee94f961ddce3e7c9d2190a3ef9208efcc), ROM_SKIP(3))

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
}

} // anonymous namespace

COMP(1985, tiexplorer, 0, 0, tiexplorer, tiexplorer, tiexplorer_state, empty_init, "Texas Instruments", "Explorer", MACHINE_NOT_WORKING | MACHINE_NO_SOUND)
