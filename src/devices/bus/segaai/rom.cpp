/***********************************************************************************************************

 Sega AI card emulation

 ***********************************************************************************************************/


#include "emu.h"
#include "rom.h"


DEFINE_DEVICE_TYPE(SEGAAI_ROM_128, segaai_rom_128_device, "segaai_rom_128", "Sega AI Card - 128KB")
DEFINE_DEVICE_TYPE(SEGAAI_ROM_256, segaai_rom_256_device, "segaai_rom_256", "Sega AI Card - 256KB")


segaai_rom_128_device::segaai_rom_128_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, type, tag, owner, clock)
	, device_segaai_card_interface( mconfig, *this )
{
}

segaai_rom_128_device::segaai_rom_128_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: segaai_rom_128_device(mconfig, SEGAAI_ROM_128, tag, owner, clock)
{
}

void segaai_rom_128_device::device_start()
{
}

void segaai_rom_128_device::device_reset()
{
}

READ8_MEMBER(segaai_rom_128_device::read_cart)
{
	return m_rom[offset];
}

WRITE8_MEMBER(segaai_rom_128_device::write_cart)
{
}



segaai_rom_256_device::segaai_rom_256_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: segaai_rom_128_device(mconfig, SEGAAI_ROM_256, tag, owner, clock)
{
	for (int i = 0; i < ARRAY_LENGTH(m_bank_reg); i++)
	{
		m_bank_reg[i] = i;
	}
}


void segaai_rom_256_device::device_start()
{
	save_item(NAME(m_bank_reg));
}


void segaai_rom_256_device::device_reset()
{
	for (int i = 0; i < ARRAY_LENGTH(m_bank_reg); i++)
	{
		m_bank_reg[i] = i;
	}
}


READ8_MEMBER(segaai_rom_256_device::read_cart)
{
	offset &= 0xffff;
 
	int bank = offset / 0x4000;

	switch (bank)
	{
		case 0:
			return m_rom[(offset & 0x3fff)];

		case 1:
			return m_rom[m_bank_reg[2] * 0x4000 + (offset & 0x3fff)];

		case 2:
			return m_rom[m_bank_reg[3] * 0x4000 + (offset & 0x3fff)];

	}

	return 0xff;
}


WRITE8_MEMBER(segaai_rom_256_device::write_cart)
{
	switch (offset & 0xffff)
	{
		case 0xFFFC:
			break;
		case 0xFFFD:
			break;
		case 0xFFFE:
			m_bank_reg[2] = data & 0x0f;
			break;
		case 0xFFFF:
			m_bank_reg[3] = data & 0x0f;
			break;
	}
}

