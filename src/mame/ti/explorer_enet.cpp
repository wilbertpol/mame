// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/**********************************************************************

    TI Explorer NuBus Ethernet controller board.

Board references found:
- 2236400 (the part number in the board's own configuration ROM)
- 2236430 (the EPROM)

TODO:
- Leds:
  - self-test fault led (red)
  - HOLD, 82586 active (green)
  - RTS, transmit active (green)
  - CRS, carrier sended (green)
  - CDT, collision detected (green)


Documented by 2243161-0001A, "Explorer NuBus Ethernet Controller General
Description", January 1987. Paragraph references below are to that manual
unless another one is named.

Paragraph 4.2.1: "there is no general-purpose microprocessor on the board". It
is a NuBus slave, and a bus master only to post an event. Its real work is done
by a LAN coprocessor - the manual only ever calls that the "LCC", but paragraph
5.1 documents its command set, its System Configuration Pointer and its Command
List verbatim as an Intel 82586's, and the board's own extended self-test names
one of its subtests "82586 int lpbk". The LCC and the NuBus see the same static
buffer RAM, the LCC through a 16-bit port of its own (paragraph 5.2.3) and the
NuBus through the window this card installs.

The self-test code the extended tests run is not on the board's coprocessor
either - it is Explorer code in the board's own ROM, which the processor finds
through the "diagnostic offset" pointer in the configuration ROM (Table 5-3)
and executes itself. That is why the subtest names are plain ASCII in the
dumped EPROM.

The board passes its power-up self-test and all nine of its extended subtests:
E'net memory, Initialization, SCB commands, Diagnose, IA setup, Configure,
82586 int lpbk, Serial int lpbk and Network presence. The last three are all
loopbacks that never leave the board, which is why they pass with no serial
interface and no transceiver emulated.

TODO:
- There is no network behind this. The 82586 is wired to nothing, so nothing
  arrives and nothing transmitted goes anywhere; only the board's own loopback
  paths work. Giving it a real network means emulating the serial interface -
  paragraph 4.2.1's Seeq 8023 - and attaching i82586.cpp to MAME's networking.

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

// Configuration register, paragraph 5.2.2.4. Figure 5-21 is the write format
// and Figure 5-22 the read format; the two differ, so they are kept apart here
// rather than folded into one set of names.
static constexpr u16 CONFIG_W_RESET = 0x0001;        // RST - not latched on the board
static constexpr u16 CONFIG_W_ENABLE_EVENTS = 0x0002; // RUS code - enable bus master (events)
static constexpr u16 CONFIG_W_FAULT_LED = 0x0004;    // CUS code - fault LED, 1 = on
static constexpr u16 CONFIG_W_SERIAL_LOOPBACK = 0x0100; // SER LPBK

// The bits the read format actually reports back. Bit 0 reads as "not used" -
// the reset is a pulse, not a latch - so it is not in this mask, and writing a
// one into it must not leave a one behind to be read out later.
static constexpr u16 CONFIG_R_MASK = CONFIG_W_ENABLE_EVENTS | CONFIG_W_FAULT_LED | CONFIG_W_SERIAL_LOOPBACK;

// Flag register, paragraph 5.2.2.5 and Figure 5-23. It shares the configuration
// register's word - configuration in bytes 0 and 1, flags in bytes 2 and 3 -
// which is why both live in one handler pair below.
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
	m_buffer_ram(*this, "buffer_ram")
{
}


void explorer_enet_device::device_add_mconfig(machine_config &config)
{
	// Paragraph 4.2.2.5's "the 8-megahertz clock that drives the" coprocessor,
	// and 4.2.2.6's "an 8-megahertz clock, the period is 125 nanoseconds". The
	// manual never says where those 8 MHz come from; the only crystal it names
	// is the 20 MHz one the serial interface halves for its transmit clock.
	I82586(config, m_i82586, 8_MHz_XTAL);
	m_i82586->out_irq_cb().set(FUNC(explorer_enet_device::lcc_irq_w));
	m_i82586->set_addrmap(0, &explorer_enet_device::lcc_map);
}


// Paragraph 4.2.1: "The LCC on the NuBus Ethernet controller board can only
// access buffer memory", and 5.2.3: of the 24 address lines its pointer formats
// allow, "only address bits 0 through 15 are wired on the NuBus Ethernet
// controller board". With 32 kilobytes of RAM behind those sixteen lines, A15
// is not decoded either, so the one window mirrors through the whole space.
//
// That mirror is what puts the coprocessor's one fixed address where the board
// wants it: it fetches the System Configuration Pointer from >FFFFF6, which
// folds onto buffer RAM offset >7FF6 - paragraph 5.2.2.8's "top 10 bytes of the
// buffer RAM".
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
	// Figure 5-21's legend for the fault LED bit: "LED on at power-up". The
	// board lights it while its self-test runs and the system turns it off once
	// the board reports good, so powering up with it already lit is the
	// documented state, not a placeholder.
	m_config_register = CONFIG_W_FAULT_LED;
	m_event_address = 0;
	m_lcc_irq = false;
}


//**************************************************************************
//  NuBus slave interface
//**************************************************************************

void explorer_enet_device::nubus_map(address_map &map)
{
	// Table 5-4, "NuBus Memory Map for Ethernet Controller". Addresses here are
	// NuBus byte addresses within the card's own >FS000000 slot space, and only
	// the windows the table draws are decoded - everything else, including the
	// two large "DO NOT ADDRESS THIS AREA" gaps, falls through to the
	// processor's bus-cycle timeout (see raven.cpp's data_map). The manual's
	// CAUTION in 5.2.2.7 says diagnostics deliberately probe unimplemented
	// memory to check the NuBus response, so those gaps have to keep erroring.
	//
	// One deliberate simplification. The decoding diagrams in 5.2.2.4 through
	// 5.2.2.7 show the register windows decoded from address bits 15 through 12
	// alone, with bits 23 through 16 don't-care, which would repeat all three
	// register blocks once per 64K through the whole slot space. Table 5-4 draws
	// them only in the low 64K and marks everything from >FS010000 up as not to
	// be addressed, and that is what is implemented: the mirrors the manual
	// spells out in words (5.2.2.6's "FSXXAXXX through FSXXBXXX" and the like)
	// are present within the low 64K, and nothing answers above it.

	// Static buffer RAM, paragraph 5.2.2.8. The top ten bytes are the LCC's
	// System Configuration Pointer, which is plain RAM like the rest of it - the
	// only fixed address the LCC has, and the root of every data structure it
	// uses.
	map(0x000000, 0x007fff).rw(FUNC(explorer_enet_device::buffer_ram_r), FUNC(explorer_enet_device::buffer_ram_w));

	// Channel attention, paragraph 5.2.2.7: an arbitrary write to a reserved
	// address, telling the LCC to start executing the command list. The content
	// of the write is not examined. Reads are not decoded at all - Table 5-4
	// labels the block "channel attention (write only)" - so they time out.
	map(0x008000, 0x009fff).w(FUNC(explorer_enet_device::channel_attention_w));

	// Event address register, paragraph 5.2.2.6. Holds the NuBus destination
	// address the board writes to when the LCC interrupts; the Explorer
	// processor loads it during initialization. Byte, halfword and word
	// accessible.
	map(0x00a000, 0x00bfff).rw(FUNC(explorer_enet_device::event_address_r), FUNC(explorer_enet_device::event_address_w));

	// Configuration register (bytes 0 and 1) and flag register (bytes 2 and 3),
	// paragraphs 5.2.2.4 and 5.2.2.5.
	map(0x00c000, 0x00dfff).rw(FUNC(explorer_enet_device::config_flag_r), FUNC(explorer_enet_device::config_flag_w));

	// The two ROM pages, paragraphs 5.2.2.2 and 5.2.2.3 - see the ROM region
	// below for the one-byte-per-word layout. The lower page is the device
	// driver routine area and the upper page carries the board's self-test code
	// followed by the configuration ROM proper.
	map(0x00e000, 0x00ffff).rom().region("enet_config", 0x0000);
	map(0xffe000, 0xffffff).rom().region("enet_config", 0x2000);
}


// The buffer RAM is one memory with two ports of different widths: 32 bits wide
// and byte-selectable to the NuBus, 16 bits wide to the coprocessor. It is the
// coprocessor's side that owns the share - that is the side an address_map can
// declare - so these translate the NuBus's word onto its two halves. Both sides
// are little-endian, so the lower NuBus half is the lower coprocessor word.
//
// These have to be word-wide handlers doing their own lane selection. A
// byte-wide install looks more natural for a byte-selectable memory and is
// wrong on this bus: the processor's byte and half-word writes both arrive as
// masked word writes, MAME then calls a byte handler once for every lane in the
// word rather than once for the lanes in the mask, and the lanes outside the
// mask are written as spurious zeroes. The same rule applies to every register
// on this bus - see the wrappers in explorer_sib.cpp.
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

	// A pulse, both edges. The 82586 data sheet's pin description says CA "must
	// be HIGH for at least one system clock period" and "is latched internally
	// on HIGH to LOW edge", so it is the trailing edge the real part acts on.
	// i82586.cpp acts on the leading one instead, so driving only ca(1) happens
	// to work today; pulsing both edges is what the part actually asks for and
	// costs nothing either way.
	m_i82586->ca(1);
	m_i82586->ca(0);
}


// Paragraph 5.2.2.6: an interrupt out of the coprocessor makes the board
// arbitrate for the NuBus and write to the address in the event address
// register, "the lower byte switches to a hard-wired all-1s (FF) event code" -
// the same event protocol the SIB uses, see explorer_sib.cpp's post_event().
void explorer_enet_device::lcc_irq_w(int state)
{
	// Paragraph 5.2.2.6 is worded as an occurrence, not a level - "the board
	// issues an event whenever the LCC issues an interrupt" - so every rising
	// edge of the coprocessor's INT posts one event.
	//
	// i82586.cpp signals a fresh interrupt on an already-asserted line by
	// re-pulsing the output ("ensure an edge is generated if interrupt already
	// asserted"), which is exactly the occurrence this board is watching for.
	// The real INT is a level, held until the host acknowledges in the SCB, so
	// on hardware the second interrupt of an unacknowledged pair raises no edge
	// at all - but the board's own diagnostic never acknowledges anything and
	// still expects an event per command it flags for interrupt, so the level
	// reading cannot be what the hardware does. Take the re-pulse at face value.
	if (!state)
	{
		m_lcc_irq = false;
		return;
	}

	m_lcc_irq = true;

	// "Two conditions must occur before the NuBus Ethernet controller can
	// generate an event. The LCC must be programmed to generate an interrupt,
	// and the configuration register event enable must be set."
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
	// Paragraph 5.2.2.5: "Byte and halfword read operations do not apply to the
	// flag register. Any flag register read operation converts to a word read
	// operation" - which is what a 32-bit handler over a 32-bit space already
	// is, so the two registers can be assembled unconditionally.
	u32 data = m_config_register & CONFIG_R_MASK;

	// Figure 5-23's memory-size bit. The four LED bits below it are live board
	// status, not latches, and every one of them is an LCC or transceiver
	// condition, so with neither present they all read 0. So does the bus-error
	// bit, which can only be set by an event posting that has never happened.
	data |= FLAG_MEMORY_SIZE;

	return data;
}


void explorer_enet_device::config_flag_w(offs_t offset, u32 data, u32 mem_mask)
{
	// The flag register half is read-only and a word write across both is
	// legal: paragraph 5.2.2.5, "A write operation has no effect on the flag
	// register, and no error is indicated."
	if (!ACCESSING_BITS_0_15)
		return;

	u16 const value = u16(data);

	LOGMASKED(LOG_REG, "%s: configuration register = %04x\n", machine().describe_context(), value);

	// RST is a pulse, not a latch (Figure 5-21, "not latched on the board"), so
	// act on it and keep it out of the stored value.
	//
	// What it resets is the coprocessor, not the board's own registers - in
	// particular the event address register survives it. TI's own device driver
	// settles that: it loads the event address register, *then* pulses this bit,
	// then issues channel attention, and never reloads the register. Calling
	// device_reset() here instead loses the address and every event the
	// coprocessor raises afterwards is posted to >00000000.
	if (value & CONFIG_W_RESET)
	{
		m_i82586->reset_w(1);
		m_i82586->reset_w(0);
	}

	m_config_register = value & ~CONFIG_W_RESET;
}


//**************************************************************************
//  ROM
//**************************************************************************

ROM_START(explorer_enet)
	// One 4-kilobyte EPROM, "divided into two pages of 2048 bytes each"
	// (paragraph 5.2.1), each page appearing at one byte per NuBus word in the
	// byte 0 - least significant - lane, which is what the ROM_SKIP(3) produces.
	// So the file's first 2048 bytes fill region 0x0000-0x1fff (the device
	// driver page, >FS00E000) and its second 2048 fill region 0x2000-0x3fff (the
	// self-test page, >FSFFE000, whose last 48 bytes are the configuration ROM
	// that Table 5-3 describes).
	ROM_REGION32_BE(0x4000, "enet_config", ROMREGION_ERASE00)
	ROMX_LOAD("2236430_nec_ethernet.bin", 0x0003, 0x1000, CRC(25bec2a9) SHA1(bf904ad5fdc9decf914223c5cd6d51a22d35556b), ROM_SKIP(3))
ROM_END


const tiny_rom_entry *explorer_enet_device::device_rom_region() const
{
	return ROM_NAME(explorer_enet);
}
