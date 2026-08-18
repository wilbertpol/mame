// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/**********************************************************************

    TI Explorer NuBus Peripheral Interface (NUPI) board.

**********************************************************************/

#include "emu.h"
#include "nupi.h"

#include "bus/nscsi/hd.h"

#define LOG_MISC (1U << 1)

#define VERBOSE (0)
#include "logmacro.h"


DEFINE_DEVICE_TYPE(NUPI, nupi_device, "nupi", "TI NuBus Peripheral Interface")

namespace {

void nupi_scsi_devices(device_slot_interface &device)
{
	device.option_add("harddisk", NSCSI_HARDDISK);
}

} // anonymous namespace


nupi_device::nupi_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock) :
	device_t(mconfig, NUPI, tag, owner, clock),
	device_ti_nubus_card_interface(mconfig, *this),
	m_mpu(*this, "mpu"),
	m_scsi(*this, "scsi"),
	m_scsibus(*this, "scsibus"),
	m_ram(*this, "ram"),
	m_firmware(*this, "firmware"),
	m_command_address(0),
	m_config_register(0),
	m_dma_address(0),
	m_dma_count(0),
	m_timer(nullptr)
{
}


void nupi_device::device_start()
{
	nubus().install_map(*this, &nupi_device::nubus_map);

	m_timer = timer_alloc(FUNC(nupi_device::timer_tick), this);
	m_timer->adjust(attotime::from_hz(60), 0, attotime::from_hz(60));

	save_item(NAME(m_command_address));
	save_item(NAME(m_config_register));
	save_item(NAME(m_dma_address));
	save_item(NAME(m_dma_count));
}

void nupi_device::device_reset()
{
	m_command_address = 0;
	m_config_register = 0;
	m_dma_address = 0;
	m_dma_count = 0;
	m_misc.clear();
}

TIMER_CALLBACK_MEMBER(nupi_device::timer_tick)
{
	m_mpu->set_input_line(M68K_IRQ_4, ASSERT_LINE);
	m_mpu->set_input_line(M68K_IRQ_4, CLEAR_LINE);
}


//**************************************************************************
//  NuBus-facing (host) registers - Section 5.3 of the NUPI General Description
//**************************************************************************

void nupi_device::nubus_map(address_map &map)
{
	map.unmap_value_high();

	map(0x00aa8000, 0x00aa801f).m(m_scsi, FUNC(ncr5385_device::map)).umask16(0x00ff);

	// Command Address Register (>Fs'E00004) - Section 5.3.5
	map(0x00e00004, 0x00e00007).rw(FUNC(nupi_device::command_address_r), FUNC(nupi_device::command_address_w));
	map(0x00e00008, 0x00e00009).rw(FUNC(nupi_device::config_register_r), FUNC(nupi_device::config_register_w));

	map(0x00d40002, 0x00d40002).r(FUNC(nupi_device::flag_register_r));

	map(0x00ffc000, 0x00ffffff).rom().region("firmware", 0);
	map(0x00ffff00, 0x00ffffff).mirror(0).r(FUNC(nupi_device::rom_config_r));
}

u32 nupi_device::command_address_r()
{
	return m_command_address;
}

void nupi_device::command_address_w(offs_t offset, u32 data, u32 mem_mask)
{
	logerror("%s: command_address_w data=%08x mask=%08x\n", machine().describe_context(), data, mem_mask);
	COMBINE_DATA(&m_command_address);

	// Writing the MSB completes the address and starts command processing (5.3.5). Real
	// hardware's NuBus slave logic latches the address into the MPU's own RAM at fixed offset
	// 0x0004 and posts a level 5 interrupt with cause code 0x31 at the shared interrupt-cause
	// register (0x280000) - confirmed against the ROM disassembly: level5int_vector reads the
	// cause byte, and on 0x31 pushes the value it finds at 0x180004 onto its internal queue of
	// pending commands.
	if (mem_mask & 0xff000000)
	{
		m_ram[0x0004 / 2] = m_command_address >> 16;
		m_ram[0x0006 / 2] = m_command_address & 0xffff;
		misc_write(0x280000, 0x31);

		m_mpu->set_input_line(M68K_IRQ_5, ASSERT_LINE);
		m_mpu->set_input_line(M68K_IRQ_5, CLEAR_LINE);
	}
}

u16 nupi_device::config_register_r()
{
	return m_config_register;
}

void nupi_device::config_register_w(u16 data)
{
	m_config_register = data;

	// Reset bit (bit 0): resets both the NUPI and the SCSI bus.
	if (BIT(data, 0))
	{
		m_mpu->reset();
		m_scsibus->reset();
	}
}

u8 nupi_device::flag_register_r()
{
	logerror("%s: flag_register_r\n", machine().describe_context());
	// Self-test-complete/passed/SCSI-passed are active low; report "all good" until the
	// firmware's actual self-test state is wired up.
	return 0x00;
}

u8 nupi_device::rom_config_r(offs_t offset)
{
	if ((offset & 3) != 3)
		return 0x00;
	return m_firmware->base()[0x3f00 + offset];
}


//**************************************************************************
//  MPU-side (internal) hardware
//**************************************************************************

void nupi_device::mpu_map(address_map &map)
{
	map.unmap_value_high();

	map(0x000000, 0x003fff).rom().region("firmware", 0);
	map(0x180000, 0x180fff).ram().share("ram");

	map(0x568000, 0x56801f).m(m_scsi, FUNC(ncr5385_device::map)).umask16(0x00ff);

	map(0x080100, 0x0801ff).lrw8(
			NAME([this](offs_t offset) { return misc_read(0x080100 + offset); }),
			NAME([this](offs_t offset, u8 data) { misc_write(0x080100 + offset, data); }));
	map(0x100000, 0x10000f).lrw8(
			NAME([this](offs_t offset) { return misc_read(0x100000 + offset); }),
			NAME([this](offs_t offset, u8 data) { misc_write(0x100000 + offset, data); }));
	map(0x280000, 0x28000f).lrw8(
			NAME([this](offs_t offset) { return misc_read(0x280000 + offset); }),
			NAME([this](offs_t offset, u8 data) { misc_write(0x280000 + offset, data); }));
	map(0x300000, 0x30000f).lrw8(
			NAME([this](offs_t offset) { return misc_read(0x300000 + offset); }),
			NAME([this](offs_t offset, u8 data) { misc_write(0x300000 + offset, data); }));
	map(0x380000, 0x3801ff).lrw8(
			NAME([this](offs_t offset) { return misc_read(0x380000 + offset); }),
			NAME([this](offs_t offset, u8 data) { misc_write(0x380000 + offset, data); }));
	map(0x508000, 0x50800f).lrw8(
			NAME([this](offs_t offset) { return misc_read(0x508000 + offset); }),
			NAME([this](offs_t offset, u8 data) { misc_write(0x508000 + offset, data); }));
	map(0x518000, 0x51800f).lrw8(
			NAME([this](offs_t offset) { return misc_read(0x518000 + offset); }),
			NAME([this](offs_t offset, u8 data) { misc_write(0x518000 + offset, data); }));

	// Best-effort DMA NuBus address/count latches - see m_dma_address/m_dma_count comment.
	// The 68000's AS_PROGRAM is a 16-bit bus, so each 32-bit value is split into two words.
	map(0x800c00, 0x800c01).lrw16(
			NAME([this]() { return u16(m_dma_address >> 16); }),
			NAME([this](u16 data) { m_dma_address = (m_dma_address & 0x0000ffff) | (u32(data) << 16); }));
	map(0x800c02, 0x800c03).lrw16(
			NAME([this]() { return u16(m_dma_address); }),
			NAME([this](u16 data) { m_dma_address = (m_dma_address & 0xffff0000) | data; }));
	map(0x802c00, 0x802c01).lrw16(
			NAME([this]() { return u16(m_dma_count >> 16); }),
			NAME([this](u16 data) { m_dma_count = (m_dma_count & 0x0000ffff) | (u32(data) << 16); }));
	map(0x802c02, 0x802c03).lrw16(
			NAME([this]() { return u16(m_dma_count); }),
			NAME([this](u16 data) { m_dma_count = (m_dma_count & 0xffff0000) | data; }));
}

u8 nupi_device::misc_read(u32 addr)
{
	LOGMASKED(LOG_MISC, "%s: misc_read %06x\n", machine().describe_context(), addr);
	auto const it = m_misc.find(addr);
	return (it != m_misc.end()) ? it->second : 0x00;
}

void nupi_device::misc_write(u32 addr, u8 data)
{
	LOGMASKED(LOG_MISC, "%s: misc_write %06x = %02x\n", machine().describe_context(), addr, data);
	m_misc[addr] = data;
}


//**************************************************************************
//  SCSI
//**************************************************************************

void nupi_device::scsi_irq_w(int state)
{
	m_mpu->set_input_line(M68K_IRQ_2, state ? ASSERT_LINE : CLEAR_LINE);
}

void nupi_device::scsi_dreq_w(int state)
{
	if (!state || !m_dma_count)
		return;

	u32 const ctrl = m_scsibus->ctrl_r();
	bool const in = (ctrl & nscsi_device_interface::S_INP);

	if (in)
		nubus().space().write_byte(m_dma_address, m_scsi->dma_r());
	else
		m_scsi->dma_w(nubus().space().read_byte(m_dma_address));

	m_dma_address++;
	m_dma_count--;
}


//**************************************************************************
//  Machine configuration
//**************************************************************************

void nupi_device::device_add_mconfig(machine_config &config)
{
	M68000(config, m_mpu, 10_MHz_XTAL); // "controlled by an MC68000 ... running at a frequency of 10 megahertz" (1.2)
	m_mpu->set_addrmap(AS_PROGRAM, &nupi_device::mpu_map);

	NSCSI_BUS(config, m_scsibus);
	NSCSI_CONNECTOR(config, "scsibus:0", nupi_scsi_devices, "harddisk", false);
	NSCSI_CONNECTOR(config, "scsibus:1", nupi_scsi_devices, nullptr, false);
	NSCSI_CONNECTOR(config, "scsibus:2", nupi_scsi_devices, nullptr, false);
	NSCSI_CONNECTOR(config, "scsibus:3", nupi_scsi_devices, nullptr, false);
	NSCSI_CONNECTOR(config, "scsibus:4", nupi_scsi_devices, nullptr, false);
	NSCSI_CONNECTOR(config, "scsibus:5", nupi_scsi_devices, nullptr, false);
	NSCSI_CONNECTOR(config, "scsibus:6", nupi_scsi_devices, nullptr, false);

	NCR5385(config, m_scsi, 10_MHz_XTAL); // clock not documented
	m_scsibus->set_external_device(7, m_scsi); // conventional initiator ID; not documented
	m_scsi->set_own_id(7);
	m_scsi->irq().set(FUNC(nupi_device::scsi_irq_w));
	m_scsi->dreq().set(FUNC(nupi_device::scsi_dreq_w));
}


//**************************************************************************
//  ROM
//**************************************************************************

ROM_START(nupi)
	ROM_REGION16_BE(0x4000, "firmware", 0)
	ROMX_LOAD("2238056-5_nupi.bin", 0x0000, 0x2000, CRC(4caf00b9) SHA1(0b96ba609e764e7052d1f26e7e242f6e89d73c9c), ROM_SKIP(1))
	ROMX_LOAD("2238057-5_nupi.bin", 0x0001, 0x2000, CRC(bb14cf27) SHA1(3b140274764ebc1ad1efbc4ed184a3d70eb84b0c), ROM_SKIP(1))
ROM_END

const tiny_rom_entry *nupi_device::device_rom_region() const
{
	return ROM_NAME(nupi);
}
