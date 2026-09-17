// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/**********************************************************************

    TI Explorer CPU board.

Board references found:
- 2243144
- 2243881
- 2243895

TODO:
- Leds: self-test fault led (red), 8 self-test leds (yellow).

**********************************************************************/

#include "emu.h"
#include "explorer_cpu.h"


DEFINE_DEVICE_TYPE(EXPLORER_CPU, explorer_cpu_device, "explorer_cpu", "TI Explorer CPU Board (2243895-0001)")


explorer_cpu_device::explorer_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock) :
	device_t(mconfig, EXPLORER_CPU, tag, owner, clock),
	device_ti_nubus_card_interface(mconfig, *this),
	m_cpu(*this, "maincpu"),
	m_microcode_proms(*this, "microcode_proms"),
	m_control_store(*this, "maincpu")
{
}


void explorer_cpu_device::device_start()
{
	nubus().install_map(*this, &explorer_cpu_device::nubus_map);

	nubus().set_bus_error_card(*this);

	u8 const *const source = m_microcode_proms->base();
	u8 *const dest = m_control_store->base();

	// The address signals to the PROMs are inverted.
	for (int i = 0; i < 0x4000; i += 8)
	{
		for (int j = 0; j < 7; j++)
			dest[0x4000 - 8 - i + j] = source[i + j];
	}
}


void explorer_cpu_device::assert_bus_error()
{
	m_cpu->assert_bus_error();
}


void explorer_cpu_device::nubus_map(address_map &map)
{
	map(0xc00000, 0xc00003).r(m_cpu, FUNC(raven_cpu_device::nubus_flag_r));
	map(0xd00000, 0xd00003).rw(m_cpu, FUNC(raven_cpu_device::config_register_r), FUNC(raven_cpu_device::config_register_w));
	map(0xe00000, 0xe0003f).w(m_cpu, FUNC(raven_cpu_device::irq_w));
	map(0xfffc00, 0xffffff).rom().region("cpu_config", 0);
}


ROM_START(explorer_cpu)
	ROM_REGION64_BE(0x4000, "maincpu", ROMREGION_ERASE00)

	ROM_REGION(0x4000, "microcode_proms", ROMREGION_ERASE00)
	ROMX_LOAD("2236480-03_microcode.bin", 0x0000, 0x0800, CRC(e54001de) SHA1(cad8e4e0071cf4d010c3aa06d09547f935ab2eca), ROM_SKIP(7))
	ROMX_LOAD("2236481-03_microcode.bin", 0x0001, 0x0800, CRC(1bbba705) SHA1(a3a2b8b5a54a235a40ec9651db36b46e14d141eb), ROM_SKIP(7))
	ROMX_LOAD("2236482-03_microcode.bin", 0x0002, 0x0800, CRC(c2fba197) SHA1(2c34e1c77a2db848883bc50384910a31a98ef0b4), ROM_SKIP(7))
	ROMX_LOAD("2236483-03_microcode.bin", 0x0003, 0x0800, CRC(eaee4d54) SHA1(b10a0a92bf47c635f6e5e9cc8bbb3326aa6e1187), ROM_SKIP(7))
	ROMX_LOAD("2236484-03_microcode.bin", 0x0004, 0x0800, CRC(9494631b) SHA1(493eb9bd4a8b077d614e1379c3e76d7078c958f2), ROM_SKIP(7))
	ROMX_LOAD("2236485-03_microcode.bin", 0x0005, 0x0800, CRC(aebd8fd0) SHA1(8507d581cf81c45089a16257711d824c44ae50fa), ROM_SKIP(7))
	ROMX_LOAD("2236486-03_microcode.bin", 0x0006, 0x0800, CRC(8a953a12) SHA1(f10ce4f53a65da5d133489d4f43b1c7f4ec5726d), ROM_SKIP(7))

	ROM_REGION32_LE(0x400, "cpu_config", ROMREGION_ERASE00)
	ROMX_LOAD("cpu_config.bin", 0x000, 0x100, CRC(4f4b10c1) SHA1(7e33f843af8c3152475847c3cabadb835097f189), ROM_SKIP(3))
ROM_END


const tiny_rom_entry *explorer_cpu_device::device_rom_region() const
{
	return ROM_NAME(explorer_cpu);
}


void explorer_cpu_device::device_add_mconfig(machine_config &config)
{
	RAVEN(config, m_cpu, 28_MHz_XTAL);
}
