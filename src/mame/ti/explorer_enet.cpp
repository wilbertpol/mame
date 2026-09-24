// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/**********************************************************************

    TI Explorer NuBus Ethernet controller board.

Board references found:
- 2236400 (the part number in the board's own configuration ROM)
- 2236430 (the EPROM)

TODO:
- The four green leds - HOLD (82586 active), RTS (transmit active), CRS (carrier
  sensed) and CDT (collision detected).
- There is no network behind this. The 82586 is wired to nothing, so nothing
  arrives and nothing transmitted goes anywhere; only the board's own loopback
  paths work.

  **********************************************************************/

#include "emu.h"
#include "explorer_enet.h"

#define LOG_REG   (1U << 1)
#define LOG_CA    (1U << 2)
#define LOG_EVENT (1U << 3)

#define VERBOSE (0)
//#define VERBOSE (LOG_REG | LOG_CA | LOG_EVENT)
#include "logmacro.h"


DEFINE_DEVICE_TYPE(EXPLORER_ENET, explorer_enet_device, "explorer_enet", "TI Explorer NuBus Ethernet Controller Board (2236400-0001)")


namespace {

static constexpr u16 CONFIG_W_RESET = 0x0001;           // RST - not latched on the board
static constexpr u16 CONFIG_W_ENABLE_EVENTS = 0x0002;   // RUS code - enable bus master (events)
static constexpr u16 CONFIG_W_FAULT_LED = 0x0004;       // CUS code - fault LED, 1 = on
static constexpr u16 CONFIG_W_SERIAL_LOOPBACK = 0x0100; // SER LPBK

static constexpr u16 CONFIG_R_MASK = CONFIG_W_ENABLE_EVENTS | CONFIG_W_FAULT_LED | CONFIG_W_SERIAL_LOOPBACK;

static constexpr u32 FLAG_BUS_ERROR = 0x0001'0000;   // NuBus error while posting an event
static constexpr u32 FLAG_HOLD_LED = 0x0002'0000;    // memory handshake
static constexpr u32 FLAG_RTS_LED = 0x0004'0000;     // request to send
static constexpr u32 FLAG_CRS_LED = 0x0008'0000;     // carrier sensed
static constexpr u32 FLAG_CDT_LED = 0x0010'0000;     // collision detected
static constexpr u32 FLAG_MEMORY_SIZE = 0x0020'0000; // 1 = 32KB, 0 = 8KB

} // anonymous namespace


explorer_enet_device::explorer_enet_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock) :
	device_t(mconfig, EXPLORER_ENET, tag, owner, clock),
	device_ti_nubus_card_interface(mconfig, *this),
	m_i82586(*this, "i82586"),
	m_buffer_ram(*this, "buffer_ram"),
	m_fault_led(*this, "fault_led")
{
}


void explorer_enet_device::device_add_mconfig(machine_config &config)
{
	I82586(config, m_i82586, 8_MHz_XTAL);
	m_i82586->out_irq_cb().set(FUNC(explorer_enet_device::lcc_irq_w));
	m_i82586->set_addrmap(0, &explorer_enet_device::lcc_map);
}


void explorer_enet_device::lcc_map(address_map &map)
{
	map(0x000000, 0x007fff).mirror(0xff8000).ram().share("buffer_ram");
}


void explorer_enet_device::device_start()
{
	nubus().install_map(*this, &explorer_enet_device::nubus_map);

	save_item(NAME(m_config_register));
	save_item(NAME(m_event_address));
	save_item(NAME(m_lcc_irq));
}


void explorer_enet_device::device_reset()
{
	m_config_register = CONFIG_W_FAULT_LED;
	update_leds();

	m_event_address = 0;
	m_lcc_irq = false;
}


//**************************************************************************
//  NuBus slave interface
//**************************************************************************

void explorer_enet_device::nubus_map(address_map &map)
{
	map(0x000000, 0x007fff).rw(FUNC(explorer_enet_device::buffer_ram_r), FUNC(explorer_enet_device::buffer_ram_w));

	map(0x008000, 0x009fff).w(FUNC(explorer_enet_device::channel_attention_w));

	map(0x00a000, 0x00bfff).rw(FUNC(explorer_enet_device::event_address_r), FUNC(explorer_enet_device::event_address_w));

	map(0x00c000, 0x00dfff).rw(FUNC(explorer_enet_device::config_flag_r), FUNC(explorer_enet_device::config_flag_w));

	map(0x00e000, 0x00ffff).rom().region("enet_config", 0x0000);
	map(0xffe000, 0xffffff).rom().region("enet_config", 0x2000);
}


u32 explorer_enet_device::buffer_ram_r(offs_t offset)
{
	return m_buffer_ram[offset * 2] | (u32(m_buffer_ram[offset * 2 + 1]) << 16);
}


void explorer_enet_device::buffer_ram_w(offs_t offset, u32 data, u32 mem_mask)
{
	u16 const lo_mask = u16(mem_mask);
	u16 const hi_mask = u16(mem_mask >> 16);

	m_buffer_ram[offset * 2] = (m_buffer_ram[offset * 2] & ~lo_mask) | (u16(data) & lo_mask);
	m_buffer_ram[offset * 2 + 1] = (m_buffer_ram[offset * 2 + 1] & ~hi_mask) | (u16(data >> 16) & hi_mask);
}


void explorer_enet_device::channel_attention_w(offs_t offset, u32 data, u32 mem_mask)
{
	LOGMASKED(LOG_CA, "%s: channel attention\n", machine().describe_context());

	m_i82586->ca(1);
	m_i82586->ca(0);
}


void explorer_enet_device::lcc_irq_w(int state)
{
	if (!state)
	{
		m_lcc_irq = false;
		return;
	}

	m_lcc_irq = true;

	if (!(m_config_register & CONFIG_W_ENABLE_EVENTS))
		return;

	LOGMASKED(LOG_EVENT, "%s: posting event to %08x\n", machine().describe_context(), m_event_address);

	nubus().space().write_byte(m_event_address, 0xff);
}


u32 explorer_enet_device::event_address_r()
{
	return m_event_address;
}


void explorer_enet_device::event_address_w(offs_t offset, u32 data, u32 mem_mask)
{
	COMBINE_DATA(&m_event_address);
	LOGMASKED(LOG_REG, "%s: event address = %08x\n", machine().describe_context(), m_event_address);
}


u32 explorer_enet_device::config_flag_r()
{
	u32 data = m_config_register & CONFIG_R_MASK;

	data |= FLAG_MEMORY_SIZE;

	return data;
}


void explorer_enet_device::config_flag_w(offs_t offset, u32 data, u32 mem_mask)
{
	if (!ACCESSING_BITS_0_15)
		return;

	u16 const value = u16(data);

	LOGMASKED(LOG_REG, "%s: configuration register = %04x\n", machine().describe_context(), value);

	if (value & CONFIG_W_RESET)
	{
		m_i82586->reset_w(1);
		m_i82586->reset_w(0);
	}

	m_config_register = value & ~CONFIG_W_RESET;
	update_leds();
}


void explorer_enet_device::update_leds()
{
	m_fault_led = bool(m_config_register & CONFIG_W_FAULT_LED);
}


//**************************************************************************
//  ROM
//**************************************************************************

ROM_START(explorer_enet)
	ROM_REGION32_BE(0x4000, "enet_config", ROMREGION_ERASE00)
	ROMX_LOAD("2236430_nec_ethernet.bin", 0x0003, 0x1000, CRC(25bec2a9) SHA1(bf904ad5fdc9decf914223c5cd6d51a22d35556b), ROM_SKIP(3))
ROM_END


const tiny_rom_entry *explorer_enet_device::device_rom_region() const
{
	return ROM_NAME(explorer_enet);
}
