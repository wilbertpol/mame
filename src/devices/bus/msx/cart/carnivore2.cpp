// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/***********************************************************************************

Emulation for RBSC Carnivore2

The Carnivore2 provides:
- YM-2413, AY-3-8910/YM2149, SCC/SCC+, and keyboard clicker in FPGA
- FlashROM storage
- CF Card interface
- Stereo jack for output of sound
- a MegaRAM implementation

Parts:
- U1 - EP2C5Q208C8 Altera CYCLONE II
- U2 - M29W640GB FlashROM - 8 MB
- U3 - CY62167DV30 RAM - 2MB RAM
- U4 - EPCS4SI8 Altera configuration EEPROM - 4MB
- U5 - YAC516-E DAC 16-bit Stereo
- U6 - M93C46MN1 EEPROM - 128 bytes
- U7 - SG-310 - 50MHz

Known limitations:
- User setting of master slot is not supported.


TODO:
- Sound mixing
- 64KB Banking: dawnpatr first reboots the machine before it fully starts; is that correct?
- 32KB support not implemented, no test cases.
- 4KB support not implemented, no test cases.
- MMM control writing not implemented, no test cases.

***********************************************************************************/

#include "emu.h"
#include "carnivore2.h"

#include "bus/ata/ataintf.h"
#include "bus/ata/hdd.h"
#include "bus/generic/slot.h"
#include "machine/eepromser.h"
#include "machine/intelfsh.h"
#include "sound/ay8910.h"
#include "sound/dac.h"
#include "sound/k051649.h"
#include "sound/ymopl.h"

#include "speaker.h"

// Debugging
#define LOG_SETUP_MASK (LOG_GENERAL << 1)
#define LOG_SETUP(...) LOGMASKED(LOG_SETUP_MASK, __VA_ARGS__)
#define LOG_REGS_MASK (LOG_SETUP_MASK << 1)
#define LOG_REGS(...) LOGMASKED(LOG_REGS_MASK, __VA_ARGS__)

//#define VERBOSE 0
#define LOG_OUTPUT_FUNC osd_printf_info
#define VERBOSE (LOG_GENERAL | LOG_SETUP_MASK | LOG_REGS_MASK)
#include "logmacro.h"


namespace {

enum
{
	REG_CARDMDR = 0x00,
		// 7------- Don't show registers (0 - show registers, 1 - don't show registers)
		// -65----- Page for control registers (00 - page 0, 01 - page 1, 10 - page 2, 11 - page 3)
		// ---4---- SCC enable (0 - disable, 1 - enable)
		// ----3--- Delayed configuration enable (0 - immediately, 1 - delay)
		// -----2-- Delayed configuration settings (0 - execution from 0000, 1 - reading from 4000)
		// ------1- BIOS data read from FlashROM or RAM (0 - FlashROM, 1 - RAM)
		// -------0 Configuration registers visible (0 - registers visible, 1 - not visible) (What is the difference with bit 7?)
	REG_ADDRM0 = 0x01,
	REG_ADDRM1 = 0x02,
	REG_ADDRM2 = 0x03,
	REG_DATM0 = 0x04,
	REG_ADDRFR = 0x05,
	REG_R1MASK = 0x06,
	REG_R1ADDR = 0x07,
	REG_R1REG = 0x08,
	REG_R1MULT = 0x09,
		// 7------- Bank register (0 - disabled, 1 - enabled)
		// -6------ Mirroring (0 - disabled, 1 - enabled)
		// --5----- Media type (0 - FlashROM, 1 - RAM)
		// ---4---- Writing (0 - disabled, 1 - enabled)
		// ----3--- Bank enabled (0 - enabled, 1 - disabled)
		// -----210 Bank size (111 - 64KB, 110 - 32KB, 101 - 16KB, 100 - 8KB, 011 - 4KB)
	REG_B1MASKR = 0x0a,
	REG_B1ADRD = 0x0b,
	REG_R2MASK = 0x0c,
	REG_R2ADDR = 0x0d,
	REG_R2REG = 0x0e,
	REG_R2MULT = 0x0f,
	REG_B2MASKR = 0x10,
	REG_B2ADRD = 0x11,
	REG_R3MASK = 0x12,
	REG_R3ADDR = 0x13,
	REG_R3REG = 0x14,
	REG_R3MULT = 0x15,
	REG_B3MASKR = 0x16,
	REG_B3ADRD = 0x17,
	REG_R4MASK = 0x18,
	REG_R4ADDR = 0x19,
	REG_R4REG = 0x1a,
	REG_R4MULT = 0x1b,
	REG_B4MASKR = 0x1c,
	REG_B4ADRD = 0x1d,
	REG_MCONF = 0x1e,
		// 7------- 1 - slot is expanded, 0 slot is not expanded
		// -6------ 1 - MMM mapper i/o ports FC, FD, FE, FF reading is enabled
		// --5----- 1 - control YM2413 (i/o ports 7C, 7D)
		// ---4---- 1 - control i/o port 3C
		// ----3--- 1 - control subslot FM-PAC bios
		// -----2-- 1 - control subslot MMM mapper with 1MB of SRAM is enabled
		// ------1- 1 - control subslot CF card interface
		// -------0 1 - control subslot MSCC
	REG_MDRCPY = 0x1f,
	REG_CONFFL = 0x20,
	REG_NSREG = 0x21,
	REG_SNDLVL = 0x22,
		// 76------ FM-PAC select
		//          00 - FM_PAC Stereo
		//          10 - FM-PAC Mono
		// --543--- FM-PAC Audio level
		// -----210 SCC/SCC+ Audio level
	REG_CFGEEPR = 0x23,
		// ----3--- Eeprom Chip Select signal
		// -----2-- Eeprom CLK signal
		// ------1- Eeprom Data Input signal
		// -------0 Eeprom Data Output signal
	REG_PSGCTRL = 0x24,
		// 7------- Enable/Disable PSG
		// -6------ Enable/Disable PPI clicker
		// --543--- PSG audio level
		// -----210 PPI clicker audio level
	REG_SLM_CFG = 0x28,
		// 76------ FM-PAC subslot number
		// --54---- RAM subslot number
		// ----32-- IDE subslot number
		// ------10 FlashROM/SCC subslot number
	REG_SCART_CFG = 0x29,
		// 7------- Scart slot enable (1 - enabled, 0 - disabled)
		// -6------ 1 - scart slot location assigned by user, 0 - scart slot assigned as subslot of master slot
		// --5----- 1 - scart slot expanded, 0 - scart slot not expanded
		// ---4---- 1 - master slot assigned by user, 0 - master slot located at physical slot
		// ----3--- unused
		// -----2-- 1 - allow slot select register for emulated slot, 0 - use real slot select register
	REG_SCART_SLT = 0x2a,
		// 76------ 00 - mini ROM up to 32KB without mapper
		//          01 - K4 mapper
		//          10 - K5 mapper without SCC
		//          11 - K5 mapper with SCC
		// --54---- master slot number
		// ----32-- expanded scart slot number
		// ------10 scart slot number
	REG_SCART_STBL = 0x2b,
	REG_FPGA_VER0 = 0x2c,
	REG_FPGA_VER1 = 0x2d,
	REG_FPGA_VER2 = 0x2e,
	REG_MROM_OFFS = 0x2f,
	REG_PSGALT = 0x30,
		// ------1- Reserved
		// -------0 PSG Ports: 0 - A0-A1, 1 - 10-11
	REG_PFXN = 0x35,
};


class msx_cart_carnivore2_device : public device_t, public msx_cart_interface
{
public:
	msx_cart_carnivore2_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
		: device_t(mconfig, MSX_CART_CARNIVORE2, tag, owner, clock)
		, msx_cart_interface(mconfig, *this)
		, m_speaker(*this, "jack")
		, m_ay8910(*this, "ay8910")
		, m_dac(*this, "dac")
		, m_k051649(*this, "k051649")
		, m_ym2413(*this, "ym2413")
		, m_flash(*this, "flash")
		, m_eeprom(*this, "eeprom")
		, m_ata(*this, "ata")
		, m_view{ {*this, "vp0"}, {*this, "vp1"}, {*this, "vp2"}, {*this, "vp3"} }
		, m_fmpac_view(*this, "fmpac_view")
		, m_ide_view(*this, "ide_view")
		, m_scc_view{ {*this, "scc_view_8000"}, {*this, "scc_view_a000"}}
		, m_rambank64(*this, "rambank64%u", 0U)
		, m_rambank32(*this, "rambank32%u", 0U)
		, m_rambank16(*this, "rambank16%u", 0U)
		, m_rambank8(*this, "rambank8%u", 0U)
		, m_rambank4(*this, "rambank4%u", 0U)
		, m_ide_rombank(*this, "ide_rombank")
		, m_fmpac_rombank(*this, "fmpac_rombank")
		, m_rombank_configured(false)
		, m_regs{ 0 }
		, m_regs_delay{ 0 }
		, m_portF0(0)
		, m_pf0_rv(0)
		, m_a8_save(0)
		, m_crslt(0)
		, m_crslt_set(false)
		//, m_slt_save{ 0 }
		, m_scrt_reg{ 0 }
		, m_scrt_base{ 0 }
		, m_exp_slt_reg(0)
		, m_fmpac_sram_active(false)
		, m_fmpac_opll_active(false)
		, m_fmpac_sram_unlock{ 0 }
		, m_fmpac_control(0)
		, m_port3C(0)
		, m_scc_bank2(2)
		, m_scc_bank3(3)
		, m_scc_modea(0)
		, m_scc_modeb(0)
		, m_scc_control(0)
		, m_creg(0)
		, m_idedat(0)
		, m_rbank{ { 0 } }
		, m_ram64_mask(0)
		, m_ram32_mask(0)
		, m_ram16_mask(0)
		, m_ram8_mask(0)
		, m_ram4_mask(0)
		, m_flash64_mask(0)
		, m_flash32_mask(0)
		, m_flash16_mask(0)
		, m_flash8_mask(0)
		, m_flash4_mask(0)
	{ }

protected:
	virtual void device_start() override;
	virtual void device_reset() override;
	virtual void device_add_mconfig(machine_config &config) override;
	virtual const tiny_rom_entry *device_rom_region() const override;

private:
	static constexpr u8 NUM_REGISTERS = 0x40;
	static constexpr u32 RAM_SIZE = 0x200000;
	static constexpr u32 FLASH_SIZE = 0x800000;
	static constexpr u8 VIEW_FLASHROM_SCC = 0;
	static constexpr u8 VIEW_IDE = 1;
	static constexpr u8 VIEW_RAM = 2;
	static constexpr u8 VIEW_FMPAC = 3;
	static constexpr u8 VIEW_SCART = 4;
	static constexpr u8 NUM_VIEWS = 5;
	static constexpr u8 NUM_SUB_SLOTS = 4;
	static constexpr u8 TOTAL_VIEWS = NUM_VIEWS * NUM_SUB_SLOTS;
	static const bool m_first_delay[NUM_REGISTERS];

	required_device<speaker_device> m_speaker;
	required_device<ay8910_device> m_ay8910;
	required_device<dac_1bit_device> m_dac;
	required_device<k051649_device> m_k051649;
	required_device<ym2413_device> m_ym2413;
	required_device<st_m29w640gb_device> m_flash;
	required_device<eeprom_serial_93c46_8bit_device> m_eeprom;
	required_device<ata_interface_device> m_ata;
	memory_view m_view[4];
	memory_view m_fmpac_view;
	memory_view m_ide_view;
	memory_view m_scc_view[2];
	memory_bank_array_creator<4> m_rambank64;
	memory_bank_array_creator<4> m_rambank32;
	memory_bank_array_creator<4> m_rambank16;
	memory_bank_array_creator<4> m_rambank8;
	memory_bank_array_creator<4> m_rambank4;
	memory_bank_creator m_ide_rombank;
	memory_bank_creator m_fmpac_rombank;
	memory_passthrough_handler m_tap_pfx;
	memory_passthrough_handler m_tap_crslt[4];
	memory_passthrough_handler m_psgalt_tap;
	memory_passthrough_handler m_delayed_banking_tap;
	bool m_rombank_configured;
	u8 m_regs[NUM_REGISTERS];
	u8 m_regs_delay[NUM_REGISTERS];
	u8 m_portF0;
	u8 m_pf0_rv;
	u8 m_a8_save;
	u8 m_crslt;
	bool m_crslt_set;
	//u8 m_slt_save[4];
	u8 m_scrt_reg[4];
	u32 m_scrt_base[4];
	u8 m_exp_slt_reg;
	// fmpac
	bool m_fmpac_sram_active;
	bool m_fmpac_opll_active;
	u8 m_fmpac_sram_unlock[2];
	u8 m_fmpac_control;
	// ram
	u8 m_port3C;
	u8 m_scc_bank2;
	u8 m_scc_bank3;
	u8 m_scc_modea;
	u8 m_scc_modeb;
	u8 m_scc_control;
	// ide
	u8 m_creg;
	u16 m_idedat;
	struct rbank
	{
		u32 bank_size;
		u8 bank_mask;
		u32 flash_bank_base;
	} m_rbank[4];
	std::unique_ptr<u8[]> m_ram;
	u16 m_ram64_mask;
	u16 m_ram32_mask;
	u16 m_ram16_mask;
	u16 m_ram8_mask;
	u16 m_ram4_mask;
	u16 m_flash64_mask;
	u16 m_flash32_mask;
	u16 m_flash16_mask;
	u16 m_flash8_mask;
	u16 m_flash4_mask;

	void control_w(offs_t offset, u8 data);
	u8 control_r(offs_t offset);
	void portF0_w(u8 data);
	void setup_banking();
	void setup_flashrom_banking();
	void setup_ide();
	void setup_ram();
	void setup_fmpac();
	void setup_scart();
	void setup_banking_64(int banking_id);
	void setup_banking_32(int banking_id);
	void setup_banking_16(int banking_id);
	void setup_banking_8(int banking_id);
	void setup_banking_4(int banking_id);
	void apply_delayed_registers();
	void setup_pfx_tap();
	void setup_scc_taps();
	void setup_crslt_taps();
	void remove_crslt_taps();
	void setup_psgalt_tap();
	void setup_delayed_banking_tap();
	void set_scrt_base(int index);
	void exp_slt_reg_w(u8 data);
	u8 exp_slt_reg_r();
	void select_views();
	void fmpac_write_ym2413(offs_t offset, u8 data);
	void fmpac_sram_unlock(offs_t offset, u8 data);
	u8 fmpac_control_r();
	void fmpac_control_w(u8 data);
	u8 fmpac_bank_r();
	void fmpac_bank_w(u8 data);
};


const bool msx_cart_carnivore2_device::m_first_delay[0x40] =
{
	false, false, false, false, false, true,  true,  true,
	true,  true,  true,  true,  true,  true,  true,  true,
	true,  true,  true,  true,  true,  true,  true,  true,
	true,  true,  true,  true,  true,  true,  true,  false,
	false, true,  false, false, false, false, false, false,
	true,  true,  true,  true,  false, false, false, false,
	false, false, false, false, false, false, false, false,
	false, false, false, false, false, false, false, false,
};


ROM_START(carnivore2)
	ROM_REGION(0x800000, "flash", ROMREGION_ERASEFF)
/*
	ROM_DEFAULT_BIOS("v2.53")
	ROM_SYSTEM_BIOS(0, "v2.53", "v2.53")
	ROMX_LOAD("v2.53.u2", 0, 0x800000, CRC(bd7cacfa) SHA1(e826ba73c8e471f1ade80e1451a60b47f5fe458c), ROM_BIOS(0))

	ROM_SYSTEM_BIOS(1, "v2.52", "v2.52")
	ROMX_LOAD("v2.52.u2", 0, 0x800000, CRC(2034309a) SHA1(28b617abe15bfa07c0e33ef7382c62eca862830f), ROM_BIOS(1))

	ROM_SYSTEM_BIOS(2, "v2.50", "v2.50")
	ROMX_LOAD("v2.50.u2", 0, 0x800000, CRC(f21f5d3c) SHA1(3e46f003b5eeb514a40b1c02884f2b653e34b937), ROM_BIOS(2))

	ROM_SYSTEM_BIOS(3, "v2.40", "v2.40")
	ROMX_LOAD("v2.40.u2", 0, 0x800000, CRC(ca60f089) SHA1(1b315d4168ee3a6452589aaa50463193e438760a), ROM_BIOS(3))

	ROM_SYSTEM_BIOS(4, "v2.30", "v2.30")
	ROMX_LOAD("v2.30.u2", 0, 0x800000, CRC(0e481e50) SHA1(84eb112e729a3ef974e03a7688dd740e1f816e2a), ROM_BIOS(4))
*/

	// Firmware for the Altera Cyclone II
	ROM_REGION(524474, "cyclone", ROMREGION_ERASE00)
/*
	ROM_LOAD("v2.30.pof.u1", 0, 524474, CRC(fcb8a5f4) SHA1(9214c0dea93d2c762d7aa0c92f58f233ae844c66))
	ROM_LOAD("v2.40.pof.u1", 0, 524474, CRC(bbb7d6b0) SHA1(8270fe962ff08ac688e58c8a3d2f4242927d3c5b))
	ROM_LOAD("v2.50.pof.u1", 0, 524474, CRC(2388a8e7) SHA1(92823455e270ecf00f47bfa195e1485eccf74d7c))
*/
ROM_END


const tiny_rom_entry *msx_cart_carnivore2_device::device_rom_region() const
{
	return ROM_NAME(carnivore2);
}


void carnivore2_devices(device_slot_interface &device)
{
    device.option_add("cfcard", ATA_CF);
}


void msx_cart_carnivore2_device::device_add_mconfig(machine_config &config)
{
	ST_M29W640GB(config, m_flash);

	EEPROM_93C46_8BIT(config, m_eeprom);

	// This is actually a separate output jack
	SPEAKER(config, m_speaker).front_center();

	AY8910(config, m_ay8910, DERIVED_CLOCK(1, 2));
	m_ay8910->set_flags(AY8910_SINGLE_OUTPUT);
	m_ay8910->add_route(ALL_OUTPUTS, m_speaker, 0.6);

	K051649(config, m_k051649, DERIVED_CLOCK(1, 1));
	m_k051649->add_route(ALL_OUTPUTS, m_speaker, 0.45);

	YM2413(config, m_ym2413, DERIVED_CLOCK(1, 1));
	m_ym2413->add_route(ALL_OUTPUTS, m_speaker, 0.8);

	// For key click
	DAC_1BIT(config, m_dac, 0);
	m_dac->add_route(ALL_OUTPUTS, m_speaker, 0.1);

	ATA_INTERFACE(config, m_ata).options(carnivore2_devices, "cfcard", nullptr, true);
}


void msx_cart_carnivore2_device::device_start()
{
	m_ram = std::make_unique<u8[]>(RAM_SIZE);

	m_regs[REG_FPGA_VER0] = 0x32;
	m_regs[REG_FPGA_VER1] = 0x35;
	m_regs[REG_FPGA_VER2] = 0x30;

	save_item(NAME(m_rombank_configured));
	save_item(NAME(m_regs));
	save_item(NAME(m_regs_delay));
	save_item(NAME(m_portF0));
	save_item(NAME(m_pf0_rv));
	save_item(NAME(m_a8_save));
	save_item(NAME(m_crslt));
	save_item(NAME(m_crslt_set));
	//save_item(NAME(m_slt_save));
	save_item(NAME(m_scrt_reg));
	save_item(NAME(m_scrt_base));
	save_item(NAME(m_fmpac_sram_active));
	save_item(NAME(m_fmpac_opll_active));
	save_item(NAME(m_fmpac_sram_unlock));
	save_item(NAME(m_fmpac_control));
	save_item(NAME(m_port3C));
	save_item(NAME(m_scc_bank2));
	save_item(NAME(m_scc_bank3));
	save_item(NAME(m_scc_modea));
	save_item(NAME(m_scc_modeb));
	save_item(NAME(m_creg));
	save_item(NAME(m_idedat));
	save_item(STRUCT_MEMBER(m_rbank, bank_size));
	save_item(STRUCT_MEMBER(m_rbank, bank_mask));
	save_item(STRUCT_MEMBER(m_rbank, flash_bank_base));
	save_pointer(NAME(m_ram), RAM_SIZE);

	m_ram64_mask = device_generic_cart_interface::map_non_power_of_two(
			unsigned(RAM_SIZE / 0x10000),
			[this] (unsigned entry, unsigned page)
			{
				for (int i = 0; i < 4; i++)
					m_rambank64[i]->configure_entry(entry, m_ram.get() + 0x10000 * page);
			}
	);

	m_ram32_mask = device_generic_cart_interface::map_non_power_of_two(
			unsigned(RAM_SIZE / 0x8000),
			[this] (unsigned entry, unsigned page)
			{
				for (int i = 0; i < 4; i++)
					m_rambank32[i]->configure_entry(entry, m_ram.get() + 0x8000 * page);
			}
	);

	m_ram16_mask = device_generic_cart_interface::map_non_power_of_two(
			unsigned(RAM_SIZE / 0x4000),
			[this] (unsigned entry, unsigned page)
			{
				for (int i = 0; i < 4; i++)
					m_rambank16[i]->configure_entry(entry, m_ram.get() + 0x4000 * page);
			}
	);

	m_ram8_mask = device_generic_cart_interface::map_non_power_of_two(
			unsigned(RAM_SIZE / 0x2000),
			[this] (unsigned entry, unsigned page)
			{
				for (int i = 0; i < 4; i++)
					m_rambank8[i]->configure_entry(entry, m_ram.get() + 0x2000 * page);
			}
	);

	m_ram4_mask = device_generic_cart_interface::map_non_power_of_two(
			unsigned(RAM_SIZE / 0x1000),
			[this] (unsigned entry, unsigned page)
			{
				for (int i = 0; i < 4; i++)
					m_rambank4[i]->configure_entry(entry, m_ram.get() + 0x1000 * page);
			}
	);

	m_flash64_mask = (FLASH_SIZE / 0x10000) - 1;
	m_flash32_mask = (FLASH_SIZE / 0x8000) - 1;
	m_flash16_mask = (FLASH_SIZE / 0x4000) - 1;
	m_flash8_mask = (FLASH_SIZE / 0x2000) - 1;
	m_flash4_mask = (FLASH_SIZE / 0x1000) - 1;

	m_regs[REG_PFXN] = 0x00;

	// Setup views
	for (int pg = 0; pg < 4; pg++)
	{
		const u16 page_start = 0x4000 * pg;
		const u16 page_end = page_start + 0x3fff;
		page(pg)->install_view(page_start, page_end, m_view[pg]);
		for (int v = 0; v < NUM_VIEWS; v++)
			m_view[pg][v];
	}
	setup_ide();
	setup_ram();
	setup_fmpac();
	setup_scart();


	// listening for writes to slot expand register on all slots!
	// This is only needed for the manual master slot functionality
	//memory_space().install_write_tap(0xffff, 0xffff, "ffff", [this] (offs_t, u8 &data, u8) { m_slt_save[(m_a8_save >> 6)] = data; });

	// Listening for writes to I/O port A8
	io_space().install_write_tap(0xa8, 0xa8, "a8", [this] (offs_t, u8 &data, u8) { m_a8_save = data; });

	// Listening for writes to I/O port AA-AB for key clicks
	io_space().install_write_tap(0xaa, 0xab, "key_click", [this] (offs_t offset, u8 &data, u8)
		{
			if (!BIT(offset, 0))
			{
				m_dac->write(BIT(data, 7));
			}
			else
			{
				if (!BIT(data, 7) && (data & 0x0e) == 0x0e)
				{
					m_dac->write(BIT(data, 0));
				}
			}
		}
	);
}


void msx_cart_carnivore2_device::device_reset()
{
	if (!m_rombank_configured)
	{
		m_ide_rombank->configure_entries(0, 8, m_flash->base() + 0x10000, 0x4000);
		m_fmpac_rombank->configure_entries(0, 4, m_flash->base() + 0x30000, 0x4000);
		m_rombank_configured = true;
	}

	m_regs[REG_CARDMDR] = 0x20;
	m_regs[REG_ADDRM0] = 0x00;
	m_regs[REG_ADDRM1] = 0x00;
	m_regs[REG_ADDRM2] = 0x00;
	m_regs[REG_ADDRFR] = m_regs_delay[REG_ADDRFR] = 0x00;
	m_regs[REG_R1MASK] = m_regs_delay[REG_R1MASK] = 0xf8;
	m_regs[REG_R1ADDR] = m_regs_delay[REG_R1ADDR] = 0x50;
	m_regs[REG_R1REG] = m_regs_delay[REG_R1REG] = 0x00;
	m_regs[REG_R1MULT] = m_regs_delay[REG_R1MULT] = 0x85;
	m_regs[REG_B1MASKR] = m_regs_delay[REG_B1MASKR] = 0x03;
	m_regs[REG_B1ADRD] = m_regs_delay[REG_B1ADRD] = 0x40;
	m_regs[REG_R2MULT] = m_regs_delay[REG_R2MULT] = 0x00;
	m_regs[REG_R3MULT] = m_regs_delay[REG_R3MULT] = 0x00;
	m_regs[REG_R4MULT] = m_regs_delay[REG_R4MULT] = 0x00;
	m_regs[REG_MCONF] = m_regs_delay[REG_MCONF] = 0xff;
	m_regs[REG_CONFFL] = 0x02;
	m_regs[REG_NSREG] = m_regs[REG_NSREG] = 0x00;
	m_regs[REG_SNDLVL] = 0x1b;
	m_regs[REG_PSGCTRL] = 0x1b;
	m_regs[REG_SLM_CFG] = m_regs_delay[REG_SLM_CFG] = 0xe4;
	m_regs[REG_SCART_CFG] = m_regs_delay[REG_SCART_CFG] = 0x00;
	m_regs[REG_SCART_SLT] = m_regs_delay[REG_SCART_SLT] = 0x00;
	m_regs[REG_SCART_STBL] = m_regs_delay[REG_SCART_STBL] = 0x00;
	m_portF0 = 0x00;
	m_pf0_rv = 0x00;
	m_a8_save = 0x00;
	m_crslt = 0x00;
	m_crslt_set = false;
	//m_slt_save[0] = 0x00;
	//m_slt_save[1] = 0x55;
	//m_slt_save[2] = 0x00;
	//m_slt_save[3] = 0x00;
	for (int i = 0; i < 4; i++)
	{
		m_scrt_reg[i] = i;
		set_scrt_base(i);
	}
	m_exp_slt_reg = 0x00;
	m_fmpac_sram_active = false;
	m_fmpac_opll_active = false;
	m_fmpac_sram_unlock[0] = 0;
	m_fmpac_sram_unlock[1] = 0;
	m_fmpac_control = 0;
	m_port3C = 0x00;
	m_scc_bank2 = 2;
	m_scc_bank3 = 3;
	m_scc_modea = 0;
	m_scc_modeb = 0;
	m_scc_control = 0x00;
	m_creg = 0x00;

	m_fmpac_view.select(0);
	m_fmpac_rombank->set_entry(0);

	for (int pg = 0; pg < 4; pg++)
		m_rbank[pg].flash_bank_base = m_regs[REG_R1REG + 6 * pg];

	setup_banking();

	setup_delayed_banking_tap();
	setup_pfx_tap();
	setup_crslt_taps();
	setup_psgalt_tap();
}


void msx_cart_carnivore2_device::setup_pfx_tap()
{
	const u8 port = 0xf0 + (m_regs[REG_PFXN] & 0x03);

	m_tap_pfx.remove();
	m_tap_pfx = io_space().install_readwrite_tap(port, port, "pfx",
		[this] (offs_t offset, u8 &data, u8)
		{
			if (!machine().side_effects_disabled())
			{
				if (m_portF0 == 0x01)
					data = 0x32;
				if (m_portF0 == 0x02)
					data = 0x30 + m_crslt;
			}
		},
		[this] (offs_t, u8 &data, u8)
		{
			portF0_w(data);
		}
	);
}


void msx_cart_carnivore2_device::setup_psgalt_tap()
{
	const u8 port = BIT(m_regs[REG_PSGALT], 0) ? 0x10 : 0xa0;

	m_psgalt_tap.remove();
	m_psgalt_tap = io_space().install_write_tap(port, port + 1, "psg_a",
		[this] (offs_t offset, u8 &data, u8)
		{
			m_ay8910->address_data_w(offset, data);
		}
	);
}


void msx_cart_carnivore2_device::setup_delayed_banking_tap()
{
	m_delayed_banking_tap.remove();

	if (BIT(m_regs[REG_CARDMDR], 3))
	{
		if (BIT(m_regs[REG_CARDMDR], 2))
		{
			// Apply delayed register changes when reading from 4000
			m_delayed_banking_tap = memory_space().install_read_tap(0x4000, 0x4000, "delay", [this] (offs_t, u8&, u8)
				{
					if (!machine().side_effects_disabled())
					{
						apply_delayed_registers();
						m_delayed_banking_tap.remove();
						// TODO Trigger re-read of 4000
					}
				}
			);
		}
		else
		{
			// Apply delayed register changes when executing from 0000 from wherever.
			m_delayed_banking_tap = memory_space().install_read_tap(0x0000, 0x0000, "delay", [this] (offs_t, u8&, u8)
				{
					// TODO: This should also check for M1/execute
					// TODO Trigger re-read of 0000?
					if (!machine().side_effects_disabled())
					{
						apply_delayed_registers();
						m_delayed_banking_tap.remove();
					}
				}
			);
		}
	}
}


void msx_cart_carnivore2_device::setup_crslt_taps()
{
	// In hardware this is actually continuously stored when the cartridge is accessed, but the cartridge
	// can only be in one physical slot and this will not change. So we store the physical slot number once
	// and remove our taps.

	printf("setup_crslt_taps\n");
	for (int pg = 0; pg < 4; pg++)
	{
		const int shift = pg * 2;
		m_tap_crslt[pg] = page(pg)->install_readwrite_tap(pg * 0x4000, pg * 0x4000 + 0x3fff, "crslt1",
			[this, shift] (offs_t, u8&, u8)
			{
				if (!m_crslt_set && !machine().side_effects_disabled())
				{
					printf("crslt read tap slot %d\n", shift / 2);
					m_crslt = (m_a8_save >> shift) & 0x03;
					remove_crslt_taps();
					machine().debug_break();
				}
			},
			[this, shift] (offs_t, u8&, u8)
			{
				printf("crslt write tap slot %d\n", shift / 2);
				if (!m_crslt_set)
				{
					m_crslt = (m_a8_save >> shift) & 0x03;
					remove_crslt_taps();
					machine().debug_break();
				}
			}
		);
	}
}


void msx_cart_carnivore2_device::remove_crslt_taps()
{
//	m_crslt_set = true;
	for (int pg = 0; pg < 4; pg++)
		m_tap_crslt[pg].remove();
}


void msx_cart_carnivore2_device::portF0_w(u8 data)
{
	switch (data)
	{
	case 0x30: // '0'
	case 0x31: // '1'
	case 0x32: // '2'
	case 0x33: // '3'
		m_regs[REG_CARDMDR] = (m_regs[REG_CARDMDR] & 0x9f) | ((data & 0x03) << 5);
		setup_banking();
		break;
	case 0x41: // 'A'
		m_regs[REG_MCONF] = (m_regs[REG_MCONF] & 0x70) | 0x01;
		setup_banking();
		break;
	case 0x43: // 'C'
		m_pf0_rv = 0x01;
		break;
	case 0x48: // 'H'
		m_regs[REG_CARDMDR] |= 0x80;
		setup_banking();
		break;
	case 0x4d: // 'M'
		m_regs[REG_MCONF] = (m_regs[REG_MCONF] & 0x70) | 0x8f;
		setup_banking();
		break;
	case 0x52: // 'R'
		m_regs[REG_CARDMDR] &= 0x7f;
		setup_banking();
		break;
	case 0x53: // 'S'
		// TODO: Respond with physical slot carnivore is in: 0x30, 0x31, 0x32, 0x33
		m_pf0_rv = 0x02;
		break;
	default:
		m_pf0_rv = 0x00;
		break;
	}
}


void msx_cart_carnivore2_device::setup_scc_taps()
{
	m_view[1][VIEW_FLASHROM_SCC].install_write_tap(0x7ffe, 0x7fff, "scc_modea", [this] (offs_t, u8 &data, u8)
		{
			// TODO Only apply most of these condition when NSC_SSCP = 1
			if (((m_regs[REG_SCART_SLT] && 0xc0 == 0xc0) || BIT(m_regs[REG_CARDMDR], 4)) && !BIT(m_scc_modeb, 5) && !BIT(m_scc_modeb, 4))
				m_scc_modea = data;
		}
	);
	m_view[2][VIEW_FLASHROM_SCC].install_write_tap(0x9000, 0x97ff, "scc_bank2", [this] (offs_t, u8 &data, u8)
		{
			// TODO Only apply most of these condition when NSC_SSCP = 1
			if (((m_regs[REG_SCART_SLT] && 0xc0 == 0xc0) || BIT(m_regs[REG_CARDMDR], 4)) && !BIT(m_scc_modeb, 4))
				m_scc_bank2 = data;
		}
	);
	m_view[2][VIEW_FLASHROM_SCC].install_write_tap(0xb000, 0xb7ff, "scc_bank3", [this] (offs_t, u8 &data, u8)
		{
			// TODO Only apply most of these condition when NSC_SSCP = 1
			if (((m_regs[REG_SCART_SLT] && 0xc0 == 0xc0) || BIT(m_regs[REG_CARDMDR], 4)) && !BIT(m_scc_modea, 6) && !BIT(m_scc_modea, 4 && !BIT(m_scc_modeb, 4)))
				m_scc_bank3 = data;
		}
	);
	m_view[2][VIEW_FLASHROM_SCC].install_write_tap(0xbffe, 0xbfff, "scc_modeb", [this] (offs_t, u8 &data, u8)
		{
			// TODO Only apply most of these condition when NSC_SSCP = 1
			if (((m_regs[REG_SCART_SLT] && 0xc0 == 0xc0) || BIT(m_regs[REG_CARDMDR], 4)) && !BIT(m_scc_modea, 6) && !BIT(m_scc_modea, 4))
				m_scc_modeb = data;
		}
	);
	m_view[2][VIEW_FLASHROM_SCC].install_readwrite_tap(0x9800, 0x9fff, "scca_rw",
		[this] (offs_t offset, u8 &data, u8)
		{
			if (((m_regs[REG_SCART_SLT] && 0xc0 == 0xc0) || BIT(m_regs[REG_CARDMDR], 4)) && !BIT(m_scc_modeb, 5) && ((m_scc_bank2 & 0x3f) == 0x3f))
			{
				offset &= 0xff;
				if (offset < 0x80)
					data = m_k051649->k051649_waveform_r(offset);
				else if (offset >= 0xe0)
					data = m_k051649->k051649_test_r(memory_space());
			}			
		},
		[this] (offs_t offset, u8 &data, u8)
		{
			if (((m_regs[REG_SCART_SLT] && 0xc0 == 0xc0) || BIT(m_regs[REG_CARDMDR], 4)) && !BIT(m_scc_modeb, 5) && ((m_scc_bank2 & 0x3f) == 0x3f))
			{
				offset &= 0xff;
				if (offset < 0x80)
					m_k051649->k051649_waveform_w(offset, data);
				else if (offset >= 0xe0)
					m_k051649->k051649_test_w(data);
				else
				{
					offset &= ~0x10;
					if (offset < 0x8a)
						m_k051649->k051649_frequency_w(offset - 0x80, data);
					else if (offset < 0x8f)
						m_k051649->k051649_volume_w(offset - 0x8a, data);
					else if (offset == 0x8f)
						m_k051649->k051649_keyonoff_w(data);
				}
			}			
		}
	);
	m_view[2][VIEW_FLASHROM_SCC].install_readwrite_tap(0xb800, 0xbffd, "sccb_rw",
		[this] (offs_t offset, u8 &data, u8)
		{
			if (((m_regs[REG_SCART_SLT] && 0xc0 == 0xc0) || BIT(m_regs[REG_CARDMDR], 4)) && BIT(m_scc_modeb, 5) && BIT(m_scc_bank3, 7))
			{
				offset &= 0xff;
				if (offset < 0xa0)
					data = m_k051649->k051649_waveform_r(offset);
				else if (offset >= 0xc0 && offset < 0xe0)
					data = m_k051649->k051649_test_r(memory_space());
			}			
		},
		[this] (offs_t offset, u8 &data, u8)
		{
			if (((m_regs[REG_SCART_SLT] && 0xc0 == 0xc0) || BIT(m_regs[REG_CARDMDR], 4)) && BIT(m_scc_modeb, 5) && BIT(m_scc_bank3, 7))
			{
				offset &= 0xff;
				if (offset < 0xa0)
					m_k051649->k051649_waveform_w(offset, data);
				else if (offset >= 0xc0 && offset < 0xe0)
					m_k051649->k051649_test_w(data);
				else
				{
					offset &= ~0x10;
					if (offset < 0xaa)
						m_k051649->k051649_frequency_w(offset - 0xa0, data);
					else if (offset < 0xaf)
						m_k051649->k051649_volume_w(offset - 0xaa, data);
					else if (offset == 0xaf)
						m_k051649->k051649_keyonoff_w(data);
				}
			}			
		}
	);
}


void msx_cart_carnivore2_device::setup_banking()
{
	// Clear current banking.
	for (int pg = 0; pg < 4; pg++)
	{
		m_view[pg][VIEW_FLASHROM_SCC].unmap_readwrite(0x4000 * pg, 0x4000 * pg + 0x3fff);
	}

	setup_flashrom_banking();
	setup_scart();

	for (int i = 0; i < 4; i++)
		set_scrt_base(i);

	if (BIT(m_regs[REG_MCONF], 7))
	{
		// Install handlers for expanded slot register
		for (int v = 0; v < NUM_VIEWS; v++)
		{
			m_view[3][v].install_write_handler(0xffff, 0xffff, emu::rw_delegate(*this, FUNC(msx_cart_carnivore2_device::exp_slt_reg_w)));
			m_view[3][v].install_read_handler(0xffff, 0xffff, emu::rw_delegate(*this, FUNC(msx_cart_carnivore2_device::exp_slt_reg_r)));
		}
	}

	select_views();

	const u8 regs_page = (m_regs[REG_CARDMDR] >> 5) & 0x03;
	const u16 page_start = 0x4000 * regs_page;
	m_view[regs_page][VIEW_FLASHROM_SCC].install_write_handler(page_start + 0x0f80, page_start + 0x0fbf, emu::rw_delegate(*this, FUNC(msx_cart_carnivore2_device::control_w)));
	if (!BIT(m_regs[REG_CARDMDR], 7))
		m_view[regs_page][VIEW_FLASHROM_SCC].install_read_handler(page_start + 0x0f80, page_start + 0x0fbf, emu::rw_delegate(*this, FUNC(msx_cart_carnivore2_device::control_r)));
}


void msx_cart_carnivore2_device::setup_flashrom_banking()
{
	for (int bank_reg = 0; bank_reg < 4; bank_reg++)
	{
		if (!BIT(m_regs[REG_R1MULT + 6 * bank_reg], 3))
		{
			switch(m_regs[REG_R1MULT + 6 * bank_reg] & 0x07)
			{
			case 0x07:
				setup_banking_64(bank_reg);
				break;
			case 0x06:
				setup_banking_32(bank_reg);
				break;
			case 0x05:
				setup_banking_16(bank_reg);
				break;
			case 0x04:
				setup_banking_8(bank_reg);
				break;
			case 0x03:
				setup_banking_4(bank_reg);
				break;
			default:
				logerror("Unknown banking size %02x selected\n", m_regs[REG_R1MULT + 6 * bank_reg] & 0x07);
				break;
			}
		}
	}
	setup_scc_taps();
}


void msx_cart_carnivore2_device::setup_ide()
{
	m_view[1][VIEW_IDE].install_read_bank(0x4000, 0x7fff, m_ide_rombank);
	m_view[1][VIEW_IDE].install_write_handler(0x4104, 0x4104, write8smo_delegate(*this, [this] (u8 data)
		{
			m_creg = data;
			m_ide_rombank->set_entry(bitswap(m_creg, 5, 6, 7));
			m_ide_view.select(BIT(m_creg, 0));
		},
		"creg"
	));
	m_view[1][VIEW_IDE].install_view(0x7c00, 0x7eff, m_ide_view);
	m_ide_view[0];
	m_ide_view[1].install_read_handler(0x7c00, 0x7dff, read8sm_delegate(*this, [this] (offs_t offset)
		{
			if (!machine().side_effects_disabled())
			{
				if (!BIT(offset, 0))
				{
					m_ata->write_dmack(1);
					m_idedat = m_ata->read_dma();
					m_ata->write_dmack(0);
					return m_idedat & 0xff;
				}
			}
			return m_idedat >> 8;
		},
		"ide_data_r"
	));
	m_ide_view[1].install_write_handler(0x7c00, 0x7dff, write8sm_delegate(*this, [this] (offs_t offset, u8 data)
		{
			if (BIT(offset, 0))
			{
				m_idedat = (m_idedat & 0x00ff) | (data << 8);
				m_ata->write_dmack(1);
				m_ata->write_dma(m_idedat);
				m_ata->write_dmack(0);
			}
			else
			{
				m_idedat = (m_idedat & 0xff00) | data;
			}
		},
		"ide_data_w"
	));
	m_ide_view[1].install_write_handler(0x7e00, 0x7e0f, write8sm_delegate(*this, [this] (offs_t offset, u8 data)
		{
			if (BIT(offset, 3))
				m_ata->cs1_w(offset & 0x07, data, 0xff);
			else
				m_ata->cs0_w(offset & 0x07, data, 0xff);
		},
		"ide_w"
	));
	m_ide_view[1].install_read_handler(0x7e00, 0x7e0f, read8sm_delegate(*this, [this] (offs_t offset)
		{
			if (BIT(offset, 3))
				return m_ata->cs1_r(offset & 0x07, 0xff);
			else
				return m_ata->cs0_r(offset & 0x07, 0xff);
		},
		"ide_r"
	));
}


void msx_cart_carnivore2_device::setup_ram()
{
	for (int pg = 0; pg < 4; pg++)
		m_view[pg][VIEW_RAM].install_readwrite_bank(0x4000 * pg, 0x4000 * pg + 0x3fff, m_rambank16[pg]);

	// TODO install taps in device_start
	// Not been able to test this yet, no software found that writes to this register
	io_space().install_write_tap(0x3c, 0x3c, "mmm_control_w", [this] (offs_t, u8 &data, u8)
		{
			//printf("************** port3C write %02x\n", data);
			if (BIT(m_regs[REG_MCONF], 4))
			{
				m_port3C = data;
				// bit 5 must be 0 to allow writing to FC-FF
				BIT(m_port3C, 0); // TODO: Controls writing to 0000-3fff
				BIT(m_port3C, 1); // TODO: Controls writing to 4000-7fff
				BIT(m_port3C, 2); // TODO: Controls writing to 8000-bfff
				BIT(m_port3C, 3); // TODO: Controls writing to c000-ffff
			}
		}
	);
	io_space().install_read_tap(0x3c, 0x3c, "mmm_control_r", [this] (offs_t offset, u8 &data, u8)
		{
			if (BIT(m_port3C, 7))
				data &= m_port3C;
		}
	);

	io_space().install_write_tap(0xfc, 0xff, "mm", [this] (offs_t offset, u8 &data, u8)
		{
			// Only the upper 1MB is used for the memory mapper
			m_rambank16[offset & 0x03]->set_entry((data | 0x40) & m_ram16_mask);
		}
	);
	// Read back of mapper registers, if enabled
	io_space().install_read_tap(0xfc, 0xff, "mm", [this] (offs_t offset, u8 &data, u8)
		{
			if (BIT(m_regs[REG_MCONF], 6))
				data &= (m_rambank16[offset & 0x03]->entry() & 0x3f) | 0xc0;
		}
	);
}


void msx_cart_carnivore2_device::setup_fmpac()
{
	m_view[1][VIEW_FMPAC].install_view(0x4000, 0x7fff, m_fmpac_view);
	m_fmpac_view[0].install_read_bank(0x4000, 0x7fff, m_fmpac_rombank);
	m_fmpac_view[0].install_write_handler(0x5ffe, 0x5fff, emu::rw_delegate(*this, FUNC(msx_cart_carnivore2_device::fmpac_sram_unlock)));
	m_fmpac_view[0].install_write_handler(0x7ff4, 0x7ff5, emu::rw_delegate(*this, FUNC(msx_cart_carnivore2_device::fmpac_write_ym2413)));
	m_fmpac_view[0].install_read_handler(0x7ff6, 0x7ff6, emu::rw_delegate(*this, FUNC(msx_cart_carnivore2_device::fmpac_control_r)));
	m_fmpac_view[0].install_write_handler(0x7ff6, 0x7ff6, emu::rw_delegate(*this, FUNC(msx_cart_carnivore2_device::fmpac_control_w)));
	m_fmpac_view[0].install_read_handler(0x7ff7, 0x7ff7, emu::rw_delegate(*this, FUNC(msx_cart_carnivore2_device::fmpac_bank_r)));
	m_fmpac_view[0].install_write_handler(0x7ff7, 0x7ff7, emu::rw_delegate(*this, FUNC(msx_cart_carnivore2_device::fmpac_bank_w)));

	m_fmpac_view[1].install_ram(0x4000, 0x5fff, &m_ram[0xfe000]);
	m_fmpac_view[1].install_write_handler(0x5ffe, 0x5fff, emu::rw_delegate(*this, FUNC(msx_cart_carnivore2_device::fmpac_sram_unlock)));
	m_fmpac_view[1].install_write_handler(0x7ff4, 0x7ff5, emu::rw_delegate(*this, FUNC(msx_cart_carnivore2_device::fmpac_write_ym2413)));
	m_fmpac_view[1].install_read_handler(0x7ff6, 0x7ff6, emu::rw_delegate(*this, FUNC(msx_cart_carnivore2_device::fmpac_control_r)));
	m_fmpac_view[1].install_write_handler(0x7ff6, 0x7ff6, emu::rw_delegate(*this, FUNC(msx_cart_carnivore2_device::fmpac_control_w)));
	m_fmpac_view[1].install_read_handler(0x7ff7, 0x7ff7, emu::rw_delegate(*this, FUNC(msx_cart_carnivore2_device::fmpac_bank_r)));
	m_fmpac_view[1].install_write_handler(0x7ff7, 0x7ff7, emu::rw_delegate(*this, FUNC(msx_cart_carnivore2_device::fmpac_bank_w)));

	io_space().install_write_tap(0x7c, 0x7d, "ym2413", [this] (offs_t offset, u8 &data, u8)
		{
			if (BIT(m_regs[REG_MCONF], 5) && m_fmpac_opll_active)
				fmpac_write_ym2413(offset & 0x01, data);
		}
	);
}


void msx_cart_carnivore2_device::setup_scart()
{
	for (int pg = 0; pg < 4; pg++)
		m_view[pg][VIEW_SCART].unmap_readwrite(0x4000 * pg, 0x4000 * pg + 0x3fff);

	for (int i = 0; i < 4; i++)
	{
		const int bank = i;
		const u16 data_start = 0x4000 + (i * 0x2000);
		m_view[1 + (i / 2)][VIEW_SCART].install_read_handler(data_start, data_start + 0x1fff,
			read8sm_delegate(*this, [this, bank] (offs_t offset)
			{
				return this->m_flash->read(m_scrt_base[bank] + offset);
			},
			"scread"
		));
		if (m_regs[REG_SCART_SLT] & 0xc0)
		{
			const u16 bank_start = (BIT(m_regs[REG_SCART_SLT], 7) ? 0x5000 : 0x4000) + (i * 0x2000);
			m_view[1 + (i / 2)][VIEW_SCART].install_write_handler(bank_start, bank_start + 0x7ff,
				write8smo_delegate(*this, [this, bank] (u8 data)
				{
					m_scrt_reg[bank] = data & 0x3f;
					set_scrt_base(bank);
				},
				"scwrite"
			));
		}
	}
}


void msx_cart_carnivore2_device::set_scrt_base(int index)
{
	m_scrt_base[index] = ((m_regs[REG_SCART_STBL] * 0x10000) + ((m_scrt_reg[index] + m_regs[REG_MROM_OFFS]) * 0x2000)) & (FLASH_SIZE - 1);
}


void msx_cart_carnivore2_device::setup_banking_64(int banking_id)
{
	m_rbank[banking_id].bank_size = 0x10000;
	const u16 data_start = 0;
	const u16 data_end = 0xffff;

	const u8 banking_page = m_regs[REG_R1ADDR + 6 * banking_id] >> 6;
	const u16 banking_start = m_regs[REG_R1ADDR + 6 * banking_id] << 8;
	const u16 banking_end = banking_start + ~(m_regs[REG_R1MASK + 6 * banking_id] << 8);

	if (BIT(m_regs[REG_R1MULT + 6 * banking_id], 7))
		LOG_SETUP("%d: 64KB Installing %s at %04x-%04x%s, banking at %04x-%04x\n",
			banking_id,
			BIT(m_regs[REG_R1MULT + 6 * banking_id], 5) ? "RAM" : "FlashROM",
			data_start,
			data_end,
			BIT(m_regs[REG_R1MULT + 6 * banking_id], 4) ? ", write enabled" : "",
			banking_start,
			banking_end
		);
	else
		LOG_SETUP("%d: 64KB Installing %s at %04x-%04x%s\n",
			banking_id,
			BIT(m_regs[REG_R1MULT + 6 * banking_id], 5) ? "RAM" : "FlashROM",
			data_start,
			data_end,
			BIT(m_regs[REG_R1MULT + 6 * banking_id], 4) ? ", write enabled" : ""
		);

	if (BIT(m_regs[REG_R1MULT + 6 * banking_id], 5))
	{
		LOG_SETUP("64KB RAM banking not supported yet\n");
	}
	else
	{
		// FlashROM
		m_rbank[banking_id].bank_mask = m_regs[REG_B1MASKR + 6 * banking_id] & m_flash64_mask;
		m_rbank[banking_id].flash_bank_base = (m_regs[REG_ADDRFR] * 0x10000) + m_rbank[banking_id].bank_size * (m_regs[REG_R1REG + 6 * banking_id] & m_rbank[banking_id].bank_mask);
		for (int i = 0; i < 4; i++)
		{
			const u16 page_start = i * 0x4000;
			const u16 page_end = page_start + 0x3fff;
			m_view[i][VIEW_FLASHROM_SCC].install_read_handler(page_start, page_end, read8sm_delegate(
				*this,
				[this, banking_id, page_start] (offs_t offset)
				{
					return this->m_flash->read(m_rbank[banking_id].flash_bank_base + page_start + offset);
				},
				"read"
			));
		}
		if (BIT(m_regs[REG_R1MULT + 6 * banking_id], 4))
		{
			LOG_SETUP(", 64KB FlashROM writing not supported yet\n");
		}
		if (BIT(m_regs[REG_R1MULT + 6 * banking_id], 7))
		{
			m_view[banking_page][VIEW_FLASHROM_SCC].install_write_handler(banking_start, banking_end, write8sm_delegate(
				*this,
				[this, banking_id] (offs_t, u8 data)
				{
					m_regs[REG_R1REG + 6 * banking_id] = data;
					m_rbank[banking_id].flash_bank_base = (m_regs[REG_ADDRFR] * 0x10000) + m_rbank[banking_id].bank_size * (data & m_rbank[banking_id].bank_mask);
				},
				"bank"
			));
		}
		else
		{
			LOG_SETUP("%d: 64KB FlashROM non-banking not supported yet\n", banking_id);
		}
	}
}


void msx_cart_carnivore2_device::setup_banking_32(int banking_id)
{
	m_rbank[banking_id].bank_size = 0x8000;
	LOG_SETUP("%d: 32KB banking not supported yet\n", banking_id);
}


void msx_cart_carnivore2_device::setup_banking_16(int banking_id)
{
	m_rbank[banking_id].bank_size = 0x4000;
	const u8 data_page = m_regs[REG_B1ADRD + 6 * banking_id] >> 6;
	const u16 data_start = (m_regs[REG_B1ADRD + 6 * banking_id] & 0xc0) << 8;
	const u16 data_end = data_start + 0x3fff;

	const u8 banking_page = m_regs[REG_R1ADDR + 6 * banking_id] >> 6;
	const u16 banking_start = m_regs[REG_R1ADDR + 6 * banking_id] << 8;
	const u16 banking_end = banking_start + ~(m_regs[REG_R1MASK + 6 * banking_id] << 8);

	const bool scc_enabled = BIT(m_regs[REG_CARDMDR], 4) && BIT(m_regs[REG_R1MULT + 6 * banking_id], 7);

	if (BIT(m_regs[REG_R1MULT + 6 * banking_id], 7))
		LOG_SETUP("%d: 16KB Installing %s at %04x-%04x%s%s, banking at %04x-%04x\n",
			banking_id,
			BIT(m_regs[REG_R1MULT + 6 * banking_id], 5) ? "RAM" : "FlashROM",
			data_start,
			data_end,
			BIT(m_regs[REG_R1MULT + 6 * banking_id], 4) ? ", write enabled" : "",
			scc_enabled ? ", enable SCC" : "",
			banking_start,
			banking_end
		);
	else
		LOG_SETUP("%d: 16KB Installing %s at %04x-%04x%s%s\n",
			banking_id,
			BIT(m_regs[REG_R1MULT + 6 * banking_id], 5) ? "RAM" : "FlashROM",
			data_start,
			data_end,
			BIT(m_regs[REG_R1MULT + 6 * banking_id], 4) ? ", write enabled" : "",
			scc_enabled ? ", enable SCC" : ""
		);

	if (scc_enabled)
		LOG_SETUP("16KB mode SCC not supported yet\n");

	if (BIT(m_regs[REG_R1MULT + 6 * banking_id], 5))
	{
		// RAM
		m_rbank[banking_id].bank_mask = m_regs[REG_B1MASKR + 6 * banking_id] & m_ram16_mask;
		m_rambank16[banking_id]->set_entry(((m_regs[REG_ADDRFR] * 4) + m_regs[REG_R1REG + 6 * banking_id]) & m_rbank[banking_id].bank_mask);
		m_view[data_page][VIEW_FLASHROM_SCC].install_read_bank(data_start, data_end, m_rambank16[banking_id]);
		if (BIT(m_regs[REG_R1MULT + 6 * banking_id], 4))
		{
			m_view[data_page][VIEW_FLASHROM_SCC].install_write_bank(data_start, data_end, m_rambank16[banking_id]);
		}
		if (BIT(m_regs[REG_R1MULT + 6 * banking_id], 7))
		{
			m_view[banking_page][VIEW_FLASHROM_SCC].install_write_handler(banking_start, banking_end, write8sm_delegate(
				*this,
				[this, banking_id] (offs_t, u8 data)
				{
					m_regs[REG_R1REG + 6 * banking_id] = data;
					m_rambank16[banking_id]->set_entry(((m_regs[REG_ADDRFR] * 4) + data) & m_rbank[banking_id].bank_mask);
				},
				"bank"
			));
		}
	}
	else
	{
		// FlashROM
		m_rbank[banking_id].bank_mask = m_regs[REG_B1MASKR + 6 * banking_id] & m_flash16_mask;
		m_rbank[banking_id].flash_bank_base = (m_regs[REG_ADDRFR] * 0x10000) + m_rbank[banking_id].bank_size * (m_regs[REG_R1REG + 6 * banking_id] & m_rbank[banking_id].bank_mask);
		m_view[data_page][VIEW_FLASHROM_SCC].install_read_handler(data_start, data_end, read8sm_delegate(*this, [this, banking_id] (offs_t offset) { return this->m_flash->read(m_rbank[banking_id].flash_bank_base + offset); }, "read"));
		if (BIT(m_regs[REG_R1MULT + 6 * banking_id], 4))
		{
			m_view[data_page][VIEW_FLASHROM_SCC].install_write_handler(data_start, data_end, write8sm_delegate(*this, [this, banking_id] (offs_t offset, u8 data) { return this->m_flash->write(m_rbank[banking_id].flash_bank_base + offset, data); }, "write"));
		}
		if (BIT(m_regs[REG_R1MULT + 6 * banking_id], 7))
		{
			m_view[banking_page][VIEW_FLASHROM_SCC].install_write_handler(banking_start, banking_end, write8sm_delegate(
				*this,
				[this, banking_id] (offs_t, u8 data)
				{
					m_regs[REG_R1REG + 6 * banking_id] = data;
					m_rbank[banking_id].flash_bank_base = (m_regs[REG_ADDRFR] * 0x10000) + m_rbank[banking_id].bank_size * (data & m_rbank[banking_id].bank_mask);
				},
				"bank"
			));
		}
	}
}

void msx_cart_carnivore2_device::setup_banking_8(int banking_id)
{
	m_rbank[banking_id].bank_size = 0x2000;
	const u8 data_page = m_regs[REG_B1ADRD + 6 * banking_id] >> 6;
	const u16 data_start = (m_regs[REG_B1ADRD + 6 * banking_id] & 0xe0) << 8;
	const u16 data_end = data_start + 0x1fff;

	const u8 banking_page = m_regs[REG_R1ADDR + 6 * banking_id] >> 6;
	const u16 banking_start = m_regs[REG_R1ADDR + 6 * banking_id] << 8;
	const u16 banking_end = banking_start + ~(m_regs[REG_R1MASK + 6 * banking_id] << 8);

	const bool scc_enabled = BIT(m_regs[REG_CARDMDR], 4) && BIT(m_regs[REG_R1MULT + 6 * banking_id], 7);	
	const bool install_scc = (data_page == 2 && scc_enabled);

	if (BIT(m_regs[REG_R1MULT + 6 * banking_id], 7))
		LOG_SETUP("%d: 8KB Installing %s at %04x-%04x%s%s, banking at %04x-%04x\n",
			banking_id,
			BIT(m_regs[REG_R1MULT + 6 * banking_id], 5) ? "RAM" : "FlashROM",
			data_start,
			data_end,
			BIT(m_regs[REG_R1MULT + 6 * banking_id], 4) ? ", write enabled" : "",
			install_scc ? ", enable SCC" : "",
			banking_start,
			banking_end
		);
	else
		LOG_SETUP("%d: 8KB Installing %s at %04x-%04x%s%s\n",
			banking_id,
			BIT(m_regs[REG_R1MULT + 6 * banking_id], 5) ? "RAM" : "FlashROM",
			data_start,
			data_end,
			BIT(m_regs[REG_R1MULT + 6 * banking_id], 4) ? ", write enabled" : "",
			install_scc ? ", enable SCC" : ""
		);

	if (BIT(m_regs[REG_R1MULT + 6 * banking_id], 5))
	{
		// RAM
		m_rbank[banking_id].bank_mask = m_regs[REG_B1MASKR + 6 * banking_id] & m_ram8_mask;
		m_rambank8[banking_id]->set_entry(((m_regs[REG_ADDRFR] * 8) + m_regs[REG_R1REG + 6 * banking_id]) & m_rbank[banking_id].bank_mask);
		m_view[data_page][VIEW_FLASHROM_SCC].install_read_bank(data_start, data_end, m_rambank8[banking_id]);
		if (BIT(m_regs[REG_R1MULT + 6 * banking_id], 4))
		{
			m_view[data_page][VIEW_FLASHROM_SCC].install_write_bank(data_start, data_end, m_rambank8[banking_id]);
		}
		if (BIT(m_regs[REG_R1MULT + 6 * banking_id], 7))
		{
			const int mode_bit = (((banking_start - 0x4000) >> 13) & 0x03);
			const u16 base = banking_start - data_start;
			if (BIT(m_regs[REG_R1MULT + 6 * banking_id], 4) && scc_enabled)
				if ((data_start & 0xe000) == (banking_start & 0xe000))
				{
					m_view[banking_page][VIEW_FLASHROM_SCC].install_write_handler(banking_start, banking_end, write8sm_delegate(
						*this,
						[this, banking_id, mode_bit, base] (offs_t offset, u8 data)
						{
							if (!(BIT(m_scc_modeb, 4) || BIT(m_scc_modeb, mode_bit)))
							{
								m_regs[REG_R1REG + 6 * banking_id] = data;
								m_rambank8[banking_id]->set_entry(((m_regs[REG_ADDRFR] * 8) + data) & m_rbank[banking_id].bank_mask);
							}
							else
							{
								// Banking register is disabled, writes should go to RAM
								reinterpret_cast<u8 *>(m_rambank8[banking_id]->base())[base + offset] = data;
							}
						},
						"bank"
					));
				}
				else
				{
					LOG_SETUP("RAM and banking are not in the same area, this is not supported.\n");
					m_view[banking_page][VIEW_FLASHROM_SCC].install_write_handler(banking_start, banking_end, write8sm_delegate(
						*this,
						[this, banking_id, mode_bit] (offs_t offset, u8 data)
						{
							if (!(BIT(m_scc_modeb, 4) || BIT(m_scc_modeb, mode_bit)))
							{
								m_regs[REG_R1REG + 6 * banking_id] = data;
								m_rambank8[banking_id]->set_entry(((m_regs[REG_ADDRFR] * 8) + data) & m_rbank[banking_id].bank_mask);
							}
						},
						"bank"
					));
				}
			else
				m_view[banking_page][VIEW_FLASHROM_SCC].install_write_handler(banking_start, banking_end, write8sm_delegate(
					*this,
					[this, banking_id, mode_bit] (offs_t offset, u8 data)
					{
						if (!(BIT(m_scc_modeb, 4) || BIT(m_scc_modeb, mode_bit)))
						{
							m_regs[REG_R1REG + 6 * banking_id] = data;
							m_rambank8[banking_id]->set_entry(((m_regs[REG_ADDRFR] * 8) + data) & m_rbank[banking_id].bank_mask);
						}
					},
					"bank"
				));
		}
	}
	else
	{
		// FlashROM
		m_rbank[banking_id].bank_mask = m_regs[REG_B1MASKR + 6 * banking_id] & m_flash8_mask;
		m_rbank[banking_id].flash_bank_base = (m_regs[REG_ADDRFR] * 0x10000) + m_rbank[banking_id].bank_size * (m_regs[REG_R1REG + 6 * banking_id] & m_rbank[banking_id].bank_mask);
		m_view[data_page][VIEW_FLASHROM_SCC].install_read_handler(data_start, data_end, read8sm_delegate(*this, [this, banking_id] (offs_t offset) { return this->m_flash->read(m_rbank[banking_id].flash_bank_base + offset); }, "read"));
		if (BIT(m_regs[REG_R1MULT + 6 * banking_id], 4))
		{
			m_view[data_page][VIEW_FLASHROM_SCC].install_write_handler(data_start, data_end, write8sm_delegate(
				*this,
				[this, banking_id] (offs_t offset, u8 data)
				{
					this->m_flash->write(m_rbank[banking_id].flash_bank_base + offset, data);
				},
				"write"
			));
		}
		if (BIT(m_regs[REG_R1MULT + 6 * banking_id], 7))
		{
			m_view[banking_page][VIEW_FLASHROM_SCC].install_write_handler(banking_start, banking_end, write8sm_delegate(
				*this,
				[this, banking_id] (offs_t, u8 data)
				{
					m_regs[REG_R1REG + 6 * banking_id] = data;
					m_rbank[banking_id].flash_bank_base = (m_regs[REG_ADDRFR] * 0x10000) + m_rbank[banking_id].bank_size * (data & m_rbank[banking_id].bank_mask);
				},
				"bank"
			));
		}
	}
}


void msx_cart_carnivore2_device::setup_banking_4(int banking_id)
{
	m_rbank[banking_id].bank_size = 0x1000;
	LOG_SETUP("%d: 4KB banking not supported yet\n", banking_id);
}


void msx_cart_carnivore2_device::control_w(offs_t offset, u8 data)
{
	static const char *reg_names[0x40] = {
		"CARDMDR", "ADDRM0", "ADDRM1", "ADDRM2", "DATM0", "ADDRFR", "R1MASK", "R1ADDR",
		"R1REG", "R1MULT", "B1MASKR", "B1ADRD", "R2MASK", "R2ADDR", "R2REG", "R2MULT",
		"B2MASKR", "B2ADRD", "R3MASK", "R3ADDR", "R3REG", "R3MULT", "B3MASKR", "B3ADRD",
		"R4MASK", "R4ADDR", "R4REG", "R4MULT", "B4MASKR", "B4ADRD", "MCONF", "MDRCPY",
		"CONFFL", "NSREG", "SNDLVL", "CFGEEPR", "PSGCTRL", "25", "26", "27",
		"SLM_CFG", "SCART_CFG", "SCART_SLT", "SCART_STBL", "FPGA_VER0", "FPGA_VER1", "FPGA_VER2", "MROM_OFFS",
		"PSGALT", "31", "32", "33", "34", "PFXN", "36", "37",
		"38", "39", "3a", "3b", "3c", "3d", "3e", "3f"
	};
	if (offset != REG_CFGEEPR)
		LOG_REGS("config_w %s, %02x\n", reg_names[offset], data);

	switch (offset)
	{
	case REG_ADDRM2:
	case REG_ADDRFR:
		data &= 0x7f;
		break;

	case REG_MCONF:
		if (!(BIT(data, 7) || (data & 0x0f) != 0x0f))
			return;
		break;

	case REG_MDRCPY:
		offset = REG_CARDMDR;
		break;

	case REG_CONFFL:
		data &= 0x07;
		break;

	case REG_DATM0:
		m_flash->write((m_regs[REG_ADDRM2] << 16) | (m_regs[REG_ADDRM1] << 8) | m_regs[REG_ADDRM0], data);
		return;

	case REG_CFGEEPR:
		m_eeprom->di_write(BIT(data, 1));
		m_eeprom->cs_write(BIT(data, 3));
		m_eeprom->clk_write(BIT(data, 2));
		return;

	case REG_MROM_OFFS:
		data &= 0x07;
		break;

	case REG_PSGALT:
		data &= 0x03;
		m_regs[offset] = data;
		setup_psgalt_tap();
		break;

	case REG_PFXN:
		data &= 0x03;
		m_regs[offset] = data;
		setup_pfx_tap();
		break;
	}

	if (m_first_delay[offset])
		m_regs_delay[offset] = data;
	else
		m_regs[offset] = data;

	if (!BIT(m_regs[REG_CARDMDR], 3))
		m_regs[offset] = data;

	if (offset == REG_CARDMDR || offset == REG_ADDRFR || (!BIT(m_regs[REG_CARDMDR], 3) && (offset >= REG_R1MASK && offset <= REG_B4ADRD)))
		setup_banking();

	if (offset == REG_CARDMDR)
		setup_delayed_banking_tap();
}


void msx_cart_carnivore2_device::apply_delayed_registers()
{
	bool registers_changed = false;
	for (int i = 0; i < NUM_REGISTERS; i++)
	{
		if (m_first_delay[i] && m_regs[i] != m_regs_delay[i])
		{
			m_regs[i] = m_regs_delay[i];
			registers_changed = true;
		}
	}
	if (registers_changed)
	{
		LOG_SETUP("apply delayed registers\n");
		setup_banking();
	}
}


u8 msx_cart_carnivore2_device::control_r(offs_t offset)
{
	switch (offset)
	{
	case REG_DATM0:
		return m_flash->read((m_regs[REG_ADDRM2] << 16) | (m_regs[REG_ADDRM1] << 8) | m_regs[REG_ADDRM0]);
	case REG_MDRCPY:
		return m_regs[REG_CARDMDR];
	case REG_CFGEEPR:
		return (m_regs[REG_CFGEEPR] & 0xfe) | m_eeprom->do_read();
	default:
		return (m_first_delay[offset]) ? m_regs_delay[offset] : m_regs[offset];
	}
}


void msx_cart_carnivore2_device::exp_slt_reg_w(u8 data)
{
	m_exp_slt_reg = data;
	select_views();
}


u8 msx_cart_carnivore2_device::exp_slt_reg_r()
{
	return ~m_exp_slt_reg;
}


void msx_cart_carnivore2_device::select_views()
{
	if (BIT(m_regs[REG_MCONF], 7))
	{
		// Expanded
		const u8 scart_subslot = (m_regs[REG_SCART_SLT] >> 2) & 0x03;

		for (int pg = 0; pg < 4; pg++)
		{
			const u8 subslot = (m_exp_slt_reg >> (pg * 2)) & 0x03;

			if (BIT(m_regs[REG_SCART_CFG], 7) && !BIT(m_regs[REG_SCART_CFG], 6) && subslot == scart_subslot)
				m_view[pg].select(VIEW_SCART);
			else if (subslot == (m_regs[REG_SLM_CFG] & 0x03))
				m_view[pg].select(VIEW_FLASHROM_SCC);
			else if (subslot == ((m_regs[REG_SLM_CFG] >> 2) & 0x03))
				m_view[pg].select(VIEW_IDE);
			else if (subslot == ((m_regs[REG_SLM_CFG] >> 4) & 0x03))
				m_view[pg].select(VIEW_RAM);
			else if (subslot == ((m_regs[REG_SLM_CFG] >> 6) & 0x03))
				m_view[pg].select(VIEW_FMPAC);
		}
	}
	else
	{
		// Not expanded
		u8 view = VIEW_FLASHROM_SCC;
		if (BIT(m_regs[REG_MCONF], 0))
			view = VIEW_FLASHROM_SCC;
		else if (BIT(m_regs[REG_MCONF], 1))
			view = VIEW_IDE;
		else if (BIT(m_regs[REG_MCONF], 2))
			view = VIEW_RAM;
		else if (BIT(m_regs[REG_MCONF], 3))
			view = VIEW_FMPAC;

		for (int pg = 0; pg < 4; pg++)
			m_view[pg].select(view);
	}
}


void msx_cart_carnivore2_device::fmpac_sram_unlock(offs_t offset, u8 data)
{
	m_fmpac_sram_unlock[offset] = data;
	m_fmpac_sram_active = m_fmpac_sram_unlock[0] == 0x4d && m_fmpac_sram_unlock[1] == 0x69;
	m_fmpac_view.select(m_fmpac_sram_active ? 1 : 0);
}


u8 msx_cart_carnivore2_device::fmpac_control_r()
{
	return m_fmpac_control;
}


void msx_cart_carnivore2_device::fmpac_control_w(u8 data)
{
	m_fmpac_control = data & 0x11;
	m_fmpac_opll_active = BIT(data, 0);
}


u8 msx_cart_carnivore2_device::fmpac_bank_r()
{
	return m_fmpac_rombank->entry();
}


void msx_cart_carnivore2_device::fmpac_bank_w(u8 data)
{
	m_fmpac_rombank->set_entry(data);
}


void msx_cart_carnivore2_device::fmpac_write_ym2413(offs_t offset, u8 data)
{
	if (m_fmpac_opll_active)
		m_ym2413->write(offset & 1, data);
}

} // anonymous namespace

DEFINE_DEVICE_TYPE_PRIVATE(MSX_CART_CARNIVORE2, msx_cart_interface, msx_cart_carnivore2_device, "msx_cart_carnivore2", "RBSC Carnivore2")
