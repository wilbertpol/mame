// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/**********************************************************************

    TI Explorer System Interface Board (SIB).

**********************************************************************/

#include "emu.h"
#include "sib.h"


DEFINE_DEVICE_TYPE(SIB, sib_device, "sib", "TI Explorer System Interface Board")


sib_device::sib_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock) :
	device_t(mconfig, SIB, tag, owner, clock),
	device_ti_nubus_card_interface(mconfig, *this)
{
}


void sib_device::device_start()
{
	nubus().install_map(*this, &sib_device::nubus_map);
}


void sib_device::nubus_map(address_map &map)
{
	map.unmap_value_high();

	map(0xff8000, 0xffffff).rom().region("sib_config", 0);
}


ROM_START(sib)
	ROM_REGION32_BE(0x8000, "sib_config", ROMREGION_ERASE00)
	ROMX_LOAD("2236662_sib.bin", 0x003, 0x2000, CRC(3f1fc829) SHA1(f16d9d9b6d8e51282fd835e2cb716cb173b3eb39), ROM_SKIP(3))
ROM_END

const tiny_rom_entry *sib_device::device_rom_region() const
{
	return ROM_NAME(sib);
}
