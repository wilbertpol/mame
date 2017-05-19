// license:BSD-3-Clause
// copyright-holders:Wilbert Pol, Fabio Priuli
// thanks-to:Chris Covell
/***********************************************************************************************************

 Sega AI Soundbox expansion emulation


 Sega AI Computer Sound Box, Model "AI-2002"  quick PCB overview by Chris Covell

ICs on board:

IC 2       TMP82C53F-2    (91/09)  Toshiba (Peripheral Timer?)
IC 3       HN27512G-25    (87/12)  Hitachi 64K EPROM
IC 6       YM2151         (91/10)  Yamaha FM chip
IC 7       TMP82C55AF-10  (88/15)  Toshiba (Peripheral Interface?)
IC 8       YM3012         (91/10)  Yamaha Stereo DAC
IC 9       HA17358                 Hitachi Dual Op-Amp
IC 10      LC7537N                 Sanyo (Volume Control IC)
IC 11      C324C          (90/42)  NEC Quad Op-Amp
IC 12      LA4520                  (Sanyo Power Audio Amp?)
IC 16-19   MB81464-12     (91/12)  Fujitsu 32K DRAMs


Misc Flat DIPs

IC ??      LS125A        Hitachi
IC ??      HC04          TI
IC ??      74HC157A x2   Toshiba
IC ??      HC138         TI
IC ??      HC139         TI

 ***********************************************************************************************************/


#include "emu.h"
#include "soundbox.h"
#include "speaker.h"


DEFINE_DEVICE_TYPE(SEGAAI_SOUNDBOX, segaai_soundbox_device, "segaai_soundbox", "Sega AI Expansion - Soundbox")


segaai_soundbox_device::segaai_soundbox_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, SEGAAI_SOUNDBOX, tag, owner, clock)
	, device_segaai_exp_interface( mconfig, *this )
	, m_tmp8253(*this, "tmp8253")
	, m_tmp8255(*this, "tmp8255")
	, m_ym2151(*this, "ym2151")
	, m_rom(*this, "soundbox")
{
}


static MACHINE_CONFIG_FRAGMENT( soundbox )
	MCFG_DEVICE_ADD("tmp8253", PIT8253, 0)
//	MCFG_PIT8253_CLK0() // unknown frequency, 5MHz?
//	MCFG_PIT8253_OUT0_HANDLER()
//	MCFG_PIT8253_CLK1() // unknown frequency, 5MHz?
//	MCFG_PIT8253_OUT1_HANDLER()
//	MCFG_PIT8253_CLK2() // unknown frequency, 5MHz?
//	MCFG_PIT8253_OUT2_HANDLER()

	MCFG_DEVICE_ADD("tmp8255", I8255, 0)
	MCFG_I8255_IN_PORTA_CB(READ8(segaai_soundbox_device, tmp8255_porta_r))
	MCFG_I8255_OUT_PORTB_CB(WRITE8(segaai_soundbox_device, tmp8255_portb_w))
	MCFG_I8255_OUT_PORTC_CB(WRITE8(segaai_soundbox_device, tmp8255_portc_w))

	MCFG_SPEAKER_STANDARD_STEREO("lspeaker", "rspeaker")
	MCFG_YM2151_ADD("ym2151", XTAL_21_4772MHz/6)   // ~3.58MHz
	MCFG_YM2151_IRQ_HANDLER(WRITELINE(segaai_soundbox_device, ym2151_irq_w))
	MCFG_SOUND_ROUTE(0, "lspeaker", 1.00)
	MCFG_SOUND_ROUTE(1, "rspeaker", 1.00)
MACHINE_CONFIG_END


machine_config_constructor segaai_soundbox_device::device_mconfig_additions() const
{
	return MACHINE_CONFIG_NAME( soundbox );
}


ROM_START( soundbox )
	ROM_REGION(0x10000, "soundbox", 0)
	ROM_LOAD("ai-snd-2002-cecb.bin", 0x0000, 0x10000, CRC(ef2dabc0) SHA1(b60cd9f6f46b6c77dba8610df6fd83368569e713))
ROM_END


const tiny_rom_entry *segaai_soundbox_device::device_rom_region() const
{
	return ROM_NAME( soundbox );
}


void segaai_soundbox_device::device_start()
{
	save_item(NAME(m_ram));
}


void segaai_soundbox_device::device_reset()
{
}


READ8_MEMBER(segaai_soundbox_device::read_lo)
{
	return m_ram[offset & 0x1ffff];
}


WRITE8_MEMBER(segaai_soundbox_device::write_lo)
{
	m_ram[offset & 0x1ffff] = data;
}


READ8_MEMBER(segaai_soundbox_device::read_hi)
{
	return m_rom[offset & 0xffff];
}


READ8_MEMBER(segaai_soundbox_device::read_io)
{
	switch (offset & 0x0c)
	{
		case 0x00:
			return m_ym2151->read(space, offset & 0x01);

		case 0x04:
			return m_tmp8253->read(space, offset & 0x03);

		case 0x08:
			return m_tmp8255->read(space, offset & 0x03);
	}
	return 0xff;
}


WRITE8_MEMBER(segaai_soundbox_device::write_io)
{
	switch (offset & 0x0c)
	{
		case 0x00:
			m_ym2151->write(space, offset & 0x01, data);
			break;

		case 0x04:
			m_tmp8253->write(space, offset & 0x03, data);
			break;

		case 0x08:
			m_tmp8255->write(space, offset & 0x03, data);
			break;
	}
}


READ8_MEMBER(segaai_soundbox_device::tmp8255_porta_r)
{
	// Read pressed keys on music keyboard row (see routine @0x82399)
	return 0xff;
}


WRITE8_MEMBER(segaai_soundbox_device::tmp8255_portb_w)
{
}


WRITE8_MEMBER(segaai_soundbox_device::tmp8255_portc_w)
{
	// Selects music keyboard row to scan (see routine @0x82399)
}


WRITE_LINE_MEMBER(segaai_soundbox_device::ym2151_irq_w)
{
	logerror("Soundbox: IRQ from ym2151 is '%s'", state ? "ASSERT" : "CLEAR");
}

