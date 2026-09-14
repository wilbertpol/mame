// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/******************************************************************************

    TI Explorer I Raven cpu core emulation.

******************************************************************************/

#include "emu.h"
#include "raven.h"
#include "raven_dasm.h"


namespace {

// Machine control register bit positions, high to low. Only the bits the
// emulation actually consults are named here, so the numbering has gaps.
static constexpr u8 MCR_SELF_TEST_FLAG_BIT = 27;
static constexpr u8 MCR_MACROINSTRUCTION_CHAINING_ENABLE_BIT = 26;
// The two MISCOP-decode group enables gating the IBUF instruction-decode
// dispatch (see execute_dispatch()); 2243144-0001A paragraph 4.5.9 notes only
// that "MISCOP detection can be disabled under the control of two bits in the
// MCR" without naming them - these positions are Meroko's MCR_Misc_Op_Group_0
// and _1.
static constexpr u8 MCR_MISC_OP_GROUP_1_BIT = 25;
static constexpr u8 MCR_MISC_OP_GROUP_0_BIT = 24;
static constexpr u8 MCR_LOOP_ON_SELF_TEST_BIT = 23;
static constexpr u8 MCR_NEED_FETCH_BIT = 22;
static constexpr u8 MCR_LOCAL_RESET_BIT = 20;
static constexpr u8 MCR_INT_ENABLE_BIT = 15;
// Real MCR bit 11 (0x800), per Meroko's own raven_cpu.c (MCR_PROM_Disable).
// Switches instruction fetch for addresses 0-0x7FF away from the boot ROM overlay
// to the writable control-store RAM (see m_inst_view/program_map()), letting
// freshly-downloaded microcode (e.g. a microload read from disk) actually execute.
static constexpr u8 MCR_PROM_DISABLE_BIT = 11;
// 2243144-0001A Table 4-16, MCR M(09): "Forced access request". Pairs with the
// level-2 map control's own M(10) "Forced access bit" - see vm_resolve_address().
static constexpr u8 MCR_FORCED_ACCESS_REQUEST_BIT = 9;
// Suppresses the bus cycle entirely when clear - see memory_cycle_enabled().
static constexpr u8 MCR_MEMORY_CYCLE_ENABLE_BIT = 8;
static constexpr u8 MCR_SUB_SYSTEM_FLAG_BIT = 7;
static constexpr u8 MCR_TEST_FAIL_FLAG_BIT = 6;

// Real memory read latency, in instructions, before requested data becomes visible
// in MD. Was 6 - confirmed wrong: the microcode's own read-then-use pattern
// (VMA-START-UNMAPPED-READ, then two instructions later SETM MD) only leaves 2
// instructions before use, matching Meroko's real hardware model ("2 INSTRUCTIONS
// PASS BEFORE COMPLETION", raven_cpu.c's lcbus_io_request()). Confirmed working
// live: MAME successfully reads a real microload from disk with this value.
static constexpr u8 MEMORY_CYCLE_BUSY_CYCLES = 2; // was 6


static const u32 shift_mask_left[32] =
{
	0x00000001, 0x00000003, 0x00000007, 0x0000000f,
	0x0000001f, 0x0000003f, 0x0000007f, 0x000000ff,
	0x000001ff, 0x000003ff, 0x000007ff, 0x00000fff,
	0x00001fff, 0x00003fff, 0x00007fff, 0x0000ffff,
	0x0001ffff, 0x0003ffff, 0x0007ffff, 0x000fffff,
	0x001fffff, 0x003fffff, 0x007fffff, 0x00ffffff,
	0x01ffffff, 0x03ffffff, 0x07ffffff, 0x0fffffff,
	0x1fffffff, 0x3fffffff, 0x7fffffff, 0xffffffff
};

static const u32 shift_mask_right[32] =
{
	0xffffffff, 0xfffffffe, 0xfffffffc, 0xfffffff8,
	0xfffffff0, 0xffffffe0, 0xffffffc0, 0xffffff80,
	0xffffff00, 0xfffffe00, 0xfffffc00, 0xfffff800,
	0xfffff000, 0xffffe000, 0xffffc000, 0xffff8000,
	0xffff0000, 0xfffe0000, 0xfffc0000, 0xfff80000,
	0xfff00000, 0xffe00000, 0xffc00000, 0xff800000,
	0xff000000, 0xfe000000, 0xfc000000, 0xf8000000,
	0xf0000000, 0xe0000000, 0xc0000000, 0x80000000
};


} // anonymous namespace


DEFINE_DEVICE_TYPE(RAVEN, raven_cpu_device, "raven", "TI Raven")


enum
{
	EXPLORER_IBUF,
	EXPLORER_IR,
	EXPLORER_LC,
	EXPLORER_MCR,
	EXPLORER_MD,
	EXPLORER_PDL_INDEX,
	EXPLORER_PDL_POINTER,
	EXPLORER_Q,
	EXPLORER_SP,
	EXPLORER_VMA
};


raven_cpu_device::raven_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: cpu_device(mconfig, RAVEN, tag, owner, clock)
	, m_program_config("program", ENDIANNESS_BIG, 64/*56*/, ADDRESS_BITS, -3, address_map_constructor(FUNC(raven_cpu_device::program_map), this))
	, m_data_config("data", ENDIANNESS_LITTLE, 32, EXTERNAL_ADDRESS_BITS, 0, address_map_constructor(FUNC(raven_cpu_device::data_map), this))
	, m_local_bus_config("local_bus", ENDIANNESS_LITTLE, 32, EXTERNAL_ADDRESS_BITS, 0, address_map_constructor(FUNC(raven_cpu_device::local_bus_map), this))
	, m_inst_view(*this, "inst_view")
	, m_control_store(*this, "control_store")
{
}


raven_cpu_device::space_config_vector raven_cpu_device::memory_space_config() const
{
	return space_config_vector {
		std::make_pair(AS_PROGRAM,    &m_program_config),
		std::make_pair(AS_DATA,       &m_data_config),
		std::make_pair(AS_LOCAL_BUS,  &m_local_bus_config)
	};
}



std::unique_ptr<util::disasm_interface> raven_cpu_device::create_disassembler()
{
	return std::make_unique<raven_disassembler>();
}


void raven_cpu_device::device_start()
{
	space(AS_PROGRAM).specific(m_program);
	space(AS_DATA).specific(m_data);
	space(AS_LOCAL_BUS).specific(m_local_bus);

	save_item(NAME(m_pc));
	save_item(NAME(m_prev_pc));
	save_item(NAME(m_next_pc));
	save_item(NAME(m_n));
	save_item(NAME(m_ir));
	save_item(NAME(m_a));
	save_item(NAME(m_a_mem));
	save_item(NAME(m_m));
	save_item(NAME(m_m_mem));
	save_item(NAME(m_t_memory));
	save_item(NAME(m_o_bus));
	save_item(NAME(m_q));
	save_item(NAME(m_md));
	save_item(NAME(m_sp));
	save_item(NAME(m_stack));
	save_item(NAME(m_mcr));
	save_item(NAME(m_config_register));
	save_item(NAME(m_imod_lo));
	save_item(NAME(m_imod_hi));
	save_item(NAME(m_vma));
	save_item(NAME(m_pdl));
	save_item(NAME(m_pdl_pointer));
	save_item(NAME(m_pdl_index));
	save_item(NAME(m_lc));
	save_item(NAME(m_ibuf));
	save_item(NAME(m_vma_lvl1_map));
	save_item(NAME(m_vma_lvl2_control));
	save_item(NAME(m_vma_lvl2_map));
	save_item(NAME(m_dispatch));
	save_item(NAME(m_dispatch_constant));
	save_item(NAME(m_cached_gc_volatility));
	save_item(NAME(m_cached_lvl1));
	save_item(NAME(m_page_fault));
	save_item(NAME(m_read_data));
	save_item(NAME(m_memory_busy_counter));
	save_item(NAME(m_read_pending));
	save_item(NAME(m_pj14_fetch_vma));
	save_item(NAME(m_pj14_fetch_addr));
	save_item(NAME(m_pj14_fetch_pending));
	save_item(NAME(m_pj14_fetch_go));
	save_item(NAME(m_pending_interrupts));
	save_item(NAME(m_bus_error));
	save_item(NAME(m_local_bus_miss));

	state_add(STATE_GENPCBASE, "CURPC", m_pc).noshow();
	state_add(STATE_GENPC, "PC", m_pc);
	state_add(EXPLORER_SP, "SP", m_sp);
	state_add(EXPLORER_IR, "IR", m_ir);
	state_add(EXPLORER_MCR, "MCR", m_mcr);
	state_add(EXPLORER_LC, "LC", m_lc);
	state_add(EXPLORER_Q, "Q", m_q);
	state_add(EXPLORER_MD, "MD", m_md);
	state_add(EXPLORER_VMA, "VMA", m_vma);
	state_add(EXPLORER_IBUF, "IBUF", m_ibuf);
	state_add(EXPLORER_PDL_INDEX, "PDL-Index", m_pdl_index);
	state_add(EXPLORER_PDL_POINTER, "PDL-Pointer", m_pdl_pointer);

	set_icountptr(m_icount);
}



void raven_cpu_device::device_reset()
{
	m_pc = 0;
	m_prev_pc = 0;
	m_next_pc = 0;
	m_n = true; // skip/nop the first instruction
	m_sp = 0;
	m_ir = 0;
	const u8 nubus_id = 6;
	m_mcr = (u32((~nubus_id) & 0x0f) << 28) | (1 << MCR_LOOP_ON_SELF_TEST_BIT);
	m_imod_lo = 0;
	m_imod_hi = 0;
	m_pdl_pointer = 0;
	m_page_fault = false;
	m_memory_busy_counter = 0;
	m_read_pending = false;
	m_pending_interrupts = 0;
	m_bus_error = false;
	m_local_bus_miss = false;
	m_inst_view.select(0);
}


void raven_cpu_device::program_map(address_map &map)
{
	map(0, 0x3fff).ram().share(m_control_store);
	map(0, 0x7ff).view(m_inst_view);
	m_inst_view[0](0, 0x7ff).rom();
	m_inst_view[1];
}


// AS_DATA is the NuBus and AS_LOCAL_BUS is the local bus. Neither bus has any
// way of reporting that nothing answered a cycle - there is no "unmapped"
// signal on the backplane - so the processor detects it by timing the cycle out
// and these catch-alls are that timeout. They are the space configuration's own
// internal maps (see memory_space_config()), which means any board this CPU is
// placed on gets the behavior for free and cannot forget to wire it up; the
// cards on the bus then install their own slot windows over the top at runtime.
void raven_cpu_device::data_map(address_map &map)
{
	map.unmap_value_high();

	map(0x00000000, 0xffffffff).rw(FUNC(raven_cpu_device::nubus_unmapped_r), FUNC(raven_cpu_device::nubus_unmapped_w));
}


void raven_cpu_device::local_bus_map(address_map &map)
{
	map.unmap_value_high();

	map(0x00000000, 0xffffffff).rw(FUNC(raven_cpu_device::local_bus_miss_r), FUNC(raven_cpu_device::local_bus_miss_w));
}


u32 raven_cpu_device::nubus_flag_r()
{
	u32 data = 0;
	if (!BIT(m_mcr, MCR_SUB_SYSTEM_FLAG_BIT))
		data |= 0x04;
	if (!BIT(m_mcr, MCR_TEST_FAIL_FLAG_BIT))
		data |= 0x02;
	if (!BIT(m_mcr, MCR_SELF_TEST_FLAG_BIT))
		data |= 0x01;
	return data;
}


u32 raven_cpu_device::config_register_r()
{
	return m_config_register;
}


void raven_cpu_device::config_register_w(offs_t offset, u32 data, u32 mem_mask)
{
	m_config_register = data & 0xff;
}


// MCR M(08), "Memory cycle enable". With the bit clear the processor issues no
// bus cycle at all: nothing is driven onto either bus, MD is left alone, and
// memory never reports busy. Every cycle starter below is gated on it, which is
// where Meroko puts the same test - the early return at the top of its single
// lcbus_io_request().
//
// The boot PROM's map self-test depends on precisely this. $01F5 starts an
// unmapped read, and $01F6 - the very next instruction - is
//
//     (M-0c) SETM MICROSTACK-POINTER IF-MEMORY-BUSY AND-CALL-ILLOP
//
// so it traps unless that cycle has already finished one instruction later,
// which no real cycle can do against the two-instruction read latency. It passes
// because memory cycles are still *disabled* there and the read never happens:
// $0201's (M-04,MCR) DPB (BYTE-FIELD 1 8) M-03 A-004 is what first sets this
// bit, and only then, at $0203, does the PROM start a cycle it expects to
// complete. Before this was modelled, that single instruction was special-cased
// by matching the address the test happens to compute (0x3db00000 - a value from
// the test pattern in M-06, not a device address at all).
bool raven_cpu_device::memory_cycle_enabled()
{
	if (BIT(m_mcr, MCR_MEMORY_CYCLE_ENABLE_BIT))
		return true;

	m_memory_busy_counter = 0;
	m_read_pending = false;
	return false;
}


void raven_cpu_device::read()
{
	m_bus_error = false;
	u32 address = vm_resolve_address<MEM_READ>();

	// A page fault leaves any cycle already in progress alone, so the enable is
	// only consulted once the access is actually going to be attempted.
	if (!m_page_fault && memory_cycle_enabled())
	{
		m_read_data = m_data.read_dword(address);
		m_memory_busy_counter = MEMORY_CYCLE_BUSY_CYCLES;
		m_read_pending = true;
	}
}


void raven_cpu_device::write()
{
	m_bus_error = false;
	u32 address = vm_resolve_address<MEM_WRITE>();

	if (!m_page_fault && memory_cycle_enabled())
	{
		m_data.write_dword(address, m_md);
		m_memory_busy_counter = MEMORY_CYCLE_BUSY_CYCLES;
		m_read_pending = false;
	}
}


// The width of an unmapped - that is, NuBus - access is not in the
// microinstruction. It is carried the way the NuBus itself carries it, in the
// two low address bits together with TM1, which is what picks between the two
// flavours of unmapped destination: the plain one (ravfmt.lisp's
// %MBD-VMA-Start-Write-Unmapped and friends) drives TM1 high and reaches this
// function, and the "-NU" one drives it low and reaches the byte functions
// below. With TM1 high the NuBus transfer table reads
//
//   A1 A0 = 00   word
//   A1 A0 = 01   half-word 0, bytes 0 and 1
//   A1 A0 = 11   half-word 1, bytes 2 and 3
//   A1 A0 = 10   block transfer
//
// so the same destination that writes a full word writes a half-word when the
// microcode sets A0. Either half is already in its own lane of MD, and a read
// leaves it in its own lane too, so the only thing the width decides is which
// byte lanes take part in the bus cycle.
//
// Ignoring it and always transferring the full word costs the other half of
// every half-word written. It is not a rare access: TI's own code uses it for
// every 16-bit field in a data structure a device shares with the processor,
// and writing the second field of such a pair then erases the first. That is
// what made the Ethernet board's "82586 int lpbk" subtest fail - see
// explorer_enet.cpp - where it wiped out the coprocessor's receive frame area
// pointer, the last two bytes of a destination address and a transmit buffer
// descriptor's count.
//
// Block transfer is not implemented; nothing in this machine has asked for one.
u32 raven_cpu_device::unmapped_mem_mask() const
{
	switch (m_vma & 3)
	{
	case 1: return 0x0000ffff;
	case 3: return 0xffff0000;
	default: return 0xffffffff;
	}
}


void raven_cpu_device::read_unmapped()
{
	m_bus_error = false;
	// VMA is the physical address here and no translation happens, so there is
	// nothing that could fault.
	m_page_fault = false;
	if (!memory_cycle_enabled())
		return;

	u32 const mask = unmapped_mem_mask();

	m_local_bus_miss = false;
	m_read_data = m_local_bus.read_dword(m_vma & ~3, mask);
	if (m_local_bus_miss)
	{
		m_read_data = m_data.read_dword(m_vma & ~3, mask);
		m_memory_busy_counter = MEMORY_CYCLE_BUSY_CYCLES;
	}
	else
	{
		m_memory_busy_counter = MEMORY_CYCLE_BUSY_CYCLES;
	}
	m_read_pending = true;
}


void raven_cpu_device::write_unmapped()
{
	m_bus_error = false;
	m_page_fault = false;
	if (!memory_cycle_enabled())
		return;

	u32 const mask = unmapped_mem_mask();

	m_local_bus_miss = false;
	m_local_bus.write_dword(m_vma & ~3, m_md, mask);
	if (m_local_bus_miss)
	{
		m_data.write_dword(m_vma & ~3, m_md, mask);
		m_memory_busy_counter = MEMORY_CYCLE_BUSY_CYCLES;
	}
	else
	{
		m_memory_busy_counter = MEMORY_CYCLE_BUSY_CYCLES;
	}
	m_read_pending = false;
}


void raven_cpu_device::read_unmapped_byte()
{
	m_bus_error = false;
	m_page_fault = false;
	if (!memory_cycle_enabled())
		return;

	u32 const shift = 8 * (m_vma & 3);
	u32 const mask = 0xff << shift;
	m_local_bus_miss = false;
	u32 raw = m_local_bus.read_dword(m_vma & ~3, mask);
	if (m_local_bus_miss)
	{
		raw = m_data.read_dword(m_vma & ~3, mask);
		m_memory_busy_counter = MEMORY_CYCLE_BUSY_CYCLES;
	}
	else
	{
		m_memory_busy_counter = MEMORY_CYCLE_BUSY_CYCLES;
	}
	u8 const byte_value = u8(raw >> shift);
	m_read_data = u32(byte_value) << (8 * (m_vma & 3));
	m_read_pending = true;
}

void raven_cpu_device::write_unmapped_byte()
{
	m_bus_error = false;
	m_page_fault = false;
	if (!memory_cycle_enabled())
		return;

	u32 const shift = 8 * (m_vma & 3);
	u32 const mask = 0xff << shift;
	u8 const byte_value = u8(m_md >> (8 * (m_vma & 3)));
	m_local_bus_miss = false;
	m_local_bus.write_dword(m_vma & ~3, u32(byte_value) << shift, mask);
	if (m_local_bus_miss)
	{
		m_data.write_dword(m_vma & ~3, u32(byte_value) << shift, mask);
		m_memory_busy_counter = MEMORY_CYCLE_BUSY_CYCLES;
	}
	else
	{
		m_memory_busy_counter = MEMORY_CYCLE_BUSY_CYCLES;
	}
	m_read_pending = false;
}


u32 raven_cpu_device::nubus_unmapped_r(offs_t offset, u32 mem_mask)
{
	m_bus_error = true;
	return 0xffffffff;
}


void raven_cpu_device::nubus_unmapped_w(offs_t offset, u32 data, u32 mem_mask)
{
	m_bus_error = true;
}


u32 raven_cpu_device::local_bus_miss_r(offs_t offset, u32 mem_mask)
{
	m_local_bus_miss = true;
	return 0xffffffff;
}


void raven_cpu_device::local_bus_miss_w(offs_t offset, u32 data, u32 mem_mask)
{
	m_local_bus_miss = true;
}


void raven_cpu_device::irq_w(offs_t offset, u32 data)
{
	int irq_level = offset & 0x0f;
	if (data)
	{
		m_pending_interrupts |= (1 << irq_level);
	}
	else
	{
		m_pending_interrupts &= ~(1 << irq_level);
	}

	u8 highest_pi = 0;
	bool pi_rq = false;
	for (int level = 15; level >= 2; level--)
	{
		if (BIT(m_pending_interrupts, level))
		{
			highest_pi = level;
			pi_rq = true;
		}
	}

	m_mcr &= ~(15 << 16);
	if (pi_rq)
	{
		m_mcr |= (highest_pi << 16);
	}
}


bool raven_cpu_device::active_int() const
{
	return ((m_pending_interrupts & 0xfffc) != 0) && BIT(m_mcr, MCR_INT_ENABLE_BIT);
}


template <int Action>
u32 raven_cpu_device::vm_resolve_address()
{
	u32 address = m_vma;
	u32 vpage_block = (m_vma >> 13) & 0xfff;
	u32 vpage_offset = (m_vma >> 8) & 0x1f;
	u32 page_offset = m_vma & 0xff;
	u32 lvl1_map_data = m_vma_lvl1_map[vpage_block];
	u32 lvl2_index = ((lvl1_map_data & 0x7f) << 5) | vpage_offset;
	u32 lvl2_control = m_vma_lvl2_control[lvl2_index];

	// Cache this page's GC volatility (level-2 control bits 12:11) for the next
	// GC-volatility dispatch - see execute_dispatch(). Same point Meroko updates
	// its cached_gcv, inside the address translation itself.
	m_cached_gc_volatility = (lvl2_control >> 11) & 0x03;

	m_page_fault = false;

	bool m1_valid = BIT(lvl1_map_data, 11);
	bool m2_forceable = BIT(lvl2_control, 10);
	bool m2_access = BIT(lvl2_control, 9);
	bool m2_writeable = BIT(lvl2_control, 8);

	if (!m1_valid || !m2_access)
	{
		m_page_fault = true;
	}

	address = (((m_vma_lvl2_map[lvl2_index] & 0x3fffff) << 8) | page_offset) << 2;

	if (Action == MEM_WRITE)
	{
		// The level-2 control's M(10) "Forced access bit" (Table 4-16) is
		// *permissive*, not restrictive: together with the MCR's own M(09)
		// "Forced access request" it is a second way to let a write through a
		// page that is not otherwise writeable. It is not a reason to fault a
		// page that is.
		//
		// This was inverted, faulting whenever the bit was set. The band hung
		// forever because of it: the page at VMA cbfdfc00 is valid, accessible
		// and writeable with only the forced-access bit set, so every write
		// faulted, and the microcode's write-retry loop at $32F2-$32FA re-issued
		// VMA-START-WRITE about 33000 times a second with the location counter
		// frozen. The loop's own dispatch at $32F6 selects dispatch[$18C], whose
		// entry is the one "nothing to fix here" entry among its neighbours -
		// the microcode had correctly concluded the write should simply succeed.
		if (!(m2_writeable || (m2_forceable && BIT(m_mcr, MCR_FORCED_ACCESS_REQUEST_BIT))))
		{
			m_page_fault = true;
		}

		// Level-1 cycle-status write-back, Table 4-16: the top of the LVL1 map
		// data read is not stored map contents at all but status from the cycle
		// just performed - M(15) "Unmapped cycle", M(14) "Not(forced cycle)",
		// M(13) "Privilege fault - write", M(12) "Privilege fault - access". The
		// microcode reads them back through the MEMORY-MAP-LEVEL-1 M source to
		// find out what its own access did, so they have to be deposited here.
		// (A write leaves M(12) alone; only a read sets or clears it.)
		lvl1_map_data &= 0x1fff;
		if (!(m2_forceable && BIT(m_mcr, MCR_FORCED_ACCESS_REQUEST_BIT)))
			lvl1_map_data |= 0x4000;
		if (m_page_fault)
			lvl1_map_data |= 0x2000;
		m_vma_lvl1_map[vpage_block] = lvl1_map_data;
		m_cached_lvl1 = lvl1_map_data;
	}
	else
	{
		lvl1_map_data &= 0x0fff;
		lvl1_map_data |= 0x4000;
		if (m_page_fault)
			lvl1_map_data |= 0x1000;
		m_vma_lvl1_map[vpage_block] = lvl1_map_data;
		m_cached_lvl1 = lvl1_map_data;
	}

	return address;
}


// The level-1 map output latch - see the MEMORY-MAP-LEVEL-1 M source in
// get_m_source(). The map is addressed by MD whenever MD is loaded, so refresh
// the latch from the MD-indexed entry there; vm_resolve_address() refreshes it
// from the VMA-indexed entry it just translated.
void raven_cpu_device::update_cached_lvl1_from_md()
{
	m_cached_lvl1 = m_vma_lvl1_map[(m_md >> 13) & 0xfff];
}


u16 raven_cpu_device::map2_addr()
{
	const u32 map1_addr = (m_md >> 13) & 0xfff;
	const u16 map1_data = m_vma_lvl1_map[map1_addr];
	const u32 map2_block = map1_data & 0x7f;
	const u32 map2_page = (m_md >> 8) & 0x1f;
	return (map2_block << 5) | map2_page; // load block bits and virtual page block offset
}


u32 raven_cpu_device::get_m_source()
{
	if (BIT(m_ir, 48))
	{
		switch ((m_ir >> 42) & 0x3f)
		{
		case 0x00: // VMA
			return m_vma;
		case 0x01: // Q
			return m_q;
		case 0x02: // IBUF argument offset field zero extended. Table 4-16 defines
		           // this as "IBUF(05:00) of current macroinstruction" - the same
		           // wording as the IBUF register and IBUF branch offset sources
		           // below, so LC(0) selects which 16-bit half of IBUF is current
		           // in exactly the same way. This used to take the low half
		           // unconditionally, which is right only for odd LC: on even LC
		           // it fed the *previous* macroinstruction's argument field into
		           // every MIB-ARGUMENT-OFFSET-FIELD read (PDL indexing at $01A8,
		           // the argument-count tests at $2416/$26B6, ...).
			return BIT(m_lc, 0) ? (m_ibuf & 0x3f) : ((m_ibuf >> 16) & 0x3f);
		case 0x03: // micro-stack pointer
			return m_sp;
		case 0x04: // MCR
			return m_mcr;
		case 0x05: // LC
			return m_lc;
		case 0x06: // memory map level 2 address
			return m_vma_lvl2_map[map2_addr()];
		case 0x07: // dispatch constant
			return m_dispatch_constant;
		case 0x08: // memory map level 1
			// Figure 4-8 (Map Logic Block Diagram) feeds the map's VIRTUAL ADDRESS
			// input from a VMA/MD multiplexer and takes READ DATA out to the MF
			// bus, so what this source returns is whatever the map last put out -
			// addressed by VMA when a cycle translated one, and by MD when MD was
			// last loaded. m_cached_lvl1 is that output; see update_cached_lvl1().
			//
			// This used to index by MD unconditionally. Found live at $2BD0,
			//   2bcf: JUMP #x2BE7 IF-BIT-SET <GC valid, M(09)> MEMORY-MAP-LEVEL-1
			//   2bd0: (M-1c) LDB <M(08:07), GC region volatility> MEMORY-MAP-LEVEL-1
			//   2bd1: (MD) SETA A-2b0
			// - the microcode reads the GC volatility of the page it has just
			// accessed and only *then* loads MD with that page's address (saved
			// out of VMA at $2BCE) for the map writes that follow. Indexing by MD
			// read a stale, unrelated page: MD was C806A245 where VMA was
			// 184A73FA, giving GC volatility 11 instead of 00. That inverted A-2af
			// bits 06:05 at $2BD4, stopped the search loop at $282B-$2834 one entry
			// early, and the Lisp world went on to read an uninitialised word and
			// take a TRANS-TRAP into the debugger.
			//
			// Indexing by VMA instead is *not* enough - tried, and it breaks the
			// boot far earlier (CMDLOG 332 -> 35): once MD has been loaded without
			// an intervening cycle the map output has to follow MD. Table 4-16's
			// own M(15:12) for this source are cycle status, which only mean
			// anything for the cycle just performed, so a latch of the map output
			// is the right shape. Same model as Meroko's `cached_lv1`.
			return m_cached_lvl1 & 0xffff;
		case 0x09: // memory map level 2 - control
			return m_vma_lvl2_control[map2_addr()];
		case 0x0a: // IBUF register
			return BIT(m_lc, 0) ? (m_ibuf & 0xffff) : ((m_ibuf >> 16) & 0xffff);
		case 0x0b: // IBUF branch offset
			return BIT(m_lc, 0) ? (m_ibuf & 0x1ff) : ((m_ibuf >> 16) & 0x1ff);
		case 0x10: // micro-stack data
			return m_stack[m_sp] & 0xfffff;
		case 0x11: // micro-stack data pop
			{
				u32 result = m_stack[m_sp] & 0xfffff;
				m_sp = (m_sp - 1) & 0x3f;
				return result;
			}
		case 0x12: // MD
			return m_md;
		case 0x20: // PDL buffer pointer data
			return m_pdl[m_pdl_pointer & 0x3ff];
		case 0x21: // PDL buffer index data
			return m_pdl[m_pdl_index & 0x3ff];
		case 0x24: // PDL buffer pointer, pop
			{
				u32 result = m_pdl[m_pdl_pointer];
				m_pdl_pointer = (m_pdl_pointer - 1) & 0x3ff;
				return result;
			}
		case 0x25: // PDL buffer index pop
			{
				u32 result = m_pdl[m_pdl_index];
				m_pdl_index = (m_pdl_index - 1) & 0x3ff;
				return result;
			}
		case 0x28: // PDL pointer(09:00)
			return m_pdl_pointer;
		case 0x29: // PDL index(09:00)
			return m_pdl_index;
		case 0x2c: // PDL pointer, pop
			{
				u32 result = m_pdl_pointer;
				m_pdl_pointer = (m_pdl_pointer - 1) & 0x3ff;
				return result;
			}
		case 0x2d: // PDL index, decrement
			{
				u32 result = m_pdl_index;
				m_pdl_index = (m_pdl_index - 1) & 0x3ff;
				return result;
			}
		default:   // reserved
			fatalerror("%x: get_m_source: functional m source %02x not implemented", m_prev_pc, (m_ir >> 42) & 0x3f);
			break;
		}
		return 0;
	}
	else
	{
		return m_m_mem[(m_ir >> 42) & 0x3f];
	}
}


// Condition 01100 in 2243144-0001A Table 4-19 is "Typed-data overflow", and the
// next entry in the same table identifies ALU(24) as the "boxed sign bit" - so
// the flag is *signed* overflow of the 25-bit boxed value in ALU(24:00), not a
// carry out of some narrower unsigned field. Both helpers below therefore use
// the textbook signed-overflow test taken at bit 24: for an add, both operands'
// signs differ from the result's; for a subtract, the operands' signs differ and
// the result's sign differs from the minuend's. (This used to be a carry out of
// bit 23 of a 24-bit field, which is a different quantity entirely and made
// TYPED-DATA SUB at microcode PC $0305 report an overflow the real machine does
// not - see ti_explorer.md.) Matches Meroko's ALU_Fixnum_Oflow.
void raven_cpu_device::add32(u32 a, u32 m, u32 carry_in, u32 &res, u32 &carry_out, u32 &fixnum_overflow)
{
	const u64 result = u64(a) + u64(m) + carry_in;
	res = u32(result);
	carry_out = BIT(result, 32);
	fixnum_overflow = BIT((m ^ res) & (a ^ res), 24);
}



void raven_cpu_device::sub32(u32 a, u32 m, u32 carry_in, u32 &res, u32 &carry_out, u32 &fixnum_overflow)
{
	const u64 result = u64(m) - u64(a) - (carry_in ? 0 : 1);
	res = u32(result);
	carry_out = BIT(result, 32);
	fixnum_overflow = BIT((m ^ a) & (m ^ res), 24);
}


void raven_cpu_device::alu_operation(u32 &result, u32 &carry_out, u32 &fixnum_overflow)
{
	switch ((m_ir >> 3) & 0x1f)
	{
	case 0x00: // SETZ
		result = 0;
		break;
	case 0x01: // AND
		result = m_m & m_a;
		if (BIT(result, 31))
		{
			carry_out = 1;
		}
		break;
	case 0x02: // ANDCA
		result = m_m & ~m_a;
		if (BIT(result, 31))
		{
			carry_out = 1;
		}
		break;
	case 0x03: // SETM
		result = m_m;
		if (BIT(result, 31))
		{
			carry_out = 1;
		}
		break;
	case 0x04: // ANDCM
		result = ~m_m & m_a;
		if (BIT(result, 31))
		{
			carry_out = 1;
		}
		break;
	case 0x05: // SETA
		result = m_a;
		break;
	case 0x06: // XOR
		result = m_m ^ m_a;
		if (BIT(result, 31))
		{
			carry_out = 1;
		}
		break;
	case 0x07: // IOR
		result = m_m | m_a;
		if (BIT(result, 31))
		{
			carry_out = 1;
		}
		break;
	case 0x08: // ANDCB
		result = ~m_m & ~m_a;
		if (BIT(result, 31))
		{
			carry_out = 1;
		}
		break;
	case 0x09: // EQV
		m_a = m_m;
		result = m_m;
		if (BIT(result, 31))
		{
			carry_out = 1;
		}
		break;
	case 0x0a: // SETCA
		result = ~m_a;
		if (BIT(result, 31))
		{
			carry_out = 1;
		}
		break;
	case 0x0b: // ORCA
		result = m_m | ~m_a;
		if (BIT(result, 31))
		{
			carry_out = 1;
		}
		break;
	case 0x0c: // SETCM
		result = ~m_m;
		if (BIT(result, 31))
		{
			carry_out = 1;
		}
		break;
	case 0x0d: // ORCM
		result = ~m_m | m_a;
		if (BIT(result, 31))
		{
			carry_out = 1;
		}
		break;
	case 0x0f: // SETO
		result = 0xffffffff;
		if (BIT(result, 31))
		{
			carry_out = 1;
		}
		break;
	case 0x10: // MUL
		if (BIT(m_q, 0))
		{
			result = m_a + m_m;
		}
		else
		{
			result = m_m;
		}
		if (BIT(result, 31))
		{
			carry_out = 1;
		}
		break;
	case 0x12: // DIV
		if (BIT(m_q, 0))
		{
			result = m_m - m_a;
		}
		else
		{
			result = m_m + m_a;
		}
		if (BIT(result, 31))
		{
			carry_out = 1;
		}
		break;
	case 0x13: // DIV-First
		result = m_m - m_a;
		if (BIT(result, 31))
		{
			carry_out = 1;
		}
		break;
	case 0x14: // DIV-Corr
		if (BIT(m_q, 0))
		{
			result = m_m;
		}
		else
		{
			result = m_m + m_a;
		}
		if (BIT(result, 31))
		{
			carry_out = 1;
		}
		break;
	case 0x19: // ADD
		add32(m_a, m_m, BIT(m_ir, 2), result, carry_out, fixnum_overflow);
		break;
	case 0x1c: // M
		add32(0, m_m, BIT(m_ir, 2), result, carry_out, fixnum_overflow);
		break;
	case 0x1e: // M-A-1
		sub32(m_a, m_m, BIT(m_ir, 2), result, carry_out, fixnum_overflow);
		break;
	case 0x1f: // M+M
		add32(m_m, m_m, BIT(m_ir, 2), result, carry_out, fixnum_overflow);
		break;
	default:
		fatalerror("%04x: ALU Operation %02x not implemented\n", m_prev_pc, (m_ir >> 3) & 0x1f);
		break;
	}
}


void raven_cpu_device::set_o_bus(u32 alu_out, u32 carry_out)
{
	u32 o_bus_input = alu_out;

	if (BIT(m_ir, 8))
	{
		o_bus_input = (m_a & 0xfe000000) | (alu_out & 0x01ffffff);
		switch ((m_ir >> 16) & 0x07)
		{
		case 0x03: // ALU
			m_o_bus = o_bus_input;
			break;

		default:
			fatalerror("%04x, %08x%08x: set_o_bus tagged %02x not implemented\n", m_prev_pc, (m_ir >> 32), u32(m_ir), (m_ir >> 16) & 0x07);
		}
	}
	else
	{
		switch ((m_ir >> 16) & 0x07)
		{
		case 0x00: // A bus
			m_o_bus = m_a;
			break;
		case 0x01: // R bus
            m_o_bus = shifter(true, false, 32);
			break;
		case 0x03: // ALU output
			m_o_bus = o_bus_input;
			break;
		case 0x04: // ALU output left shift
			m_o_bus = o_bus_input << 1;
			if (BIT(m_q, 31))
			{
				m_o_bus |= 1;
			}
			break;
		case 0x05: // ALU output shift right
			m_o_bus = o_bus_input >> 1;
			if (carry_out)
			{
				m_o_bus |= 0x80000000;
			}
			break;
		case 0x06: // ALU pointer field sign extended
			if (BIT(o_bus_input, 24))
			{
				m_o_bus = o_bus_input | 0xfe000000;
			}
			else
			{
				m_o_bus = o_bus_input & 0x00ffffff;
			}
			break;
		case 0x07: // ALU mirror
			m_o_bus = alu_out;
			m_o_bus = ((m_o_bus >>  1) & 0x55555555) | ((m_o_bus <<  1) & 0xaaaaaaaa);
			m_o_bus = ((m_o_bus >>  2) & 0x33333333) | ((m_o_bus <<  2) & 0xcccccccc);
			m_o_bus = ((m_o_bus >>  4) & 0x0f0f0f0f) | ((m_o_bus <<  4) & 0xf0f0f0f0);
			m_o_bus = ((m_o_bus >>  8) & 0x00ff00ff) | ((m_o_bus <<  8) & 0xff00ff00);
			m_o_bus = ((m_o_bus >> 16) & 0x0000ffff) | ((m_o_bus << 16) & 0xffff0000);
			break;
		case 0x02: // A bus
		default:
			fatalerror("%04x: set_o_bus not tagged %02x not implemented\n", m_prev_pc, (m_ir >> 16) & 0x07);
		}
	}
}


void raven_cpu_device::store_o_bus()
{
	if (BIT(m_ir, 31))
	{
		m_a_mem[(m_ir >> 19) & 0x3ff] = m_o_bus;
	}
	else
	{
		m_a_mem[(m_ir >> 19) & 0x3f] = m_o_bus;
		m_m_mem[(m_ir >> 19) & 0x3f] = m_o_bus;

		switch ((m_ir >> 25) & 0x3f)
		{
		case 0x00: // nop
			break;
		case 0x01: // LC
			m_lc = m_o_bus & 0x03ffffff;
			m_mcr |= (1 << MCR_NEED_FETCH_BIT);
			break;
		case 0x02: // MCR
			m_mcr = (m_mcr & (0xf08f0000 | (1 << MCR_NEED_FETCH_BIT))) | (m_o_bus & (0x0f70ffff & ~(1 << MCR_NEED_FETCH_BIT)));
			// The boot-PROM overlay follows the PROM-disable bit's current *level*, not
			// its 0->1 edge. After the loaded microcode is live the microcode clears this
			// bit again to run PROM-resident code (the $001E-$0023 entry sequence), and
			// must see the PROM there; latching the overlay on the rising edge left
			// address 0-0x7ff permanently mapped to the writable control store, so
			// $001E executed the wrong microinstruction and fell into the PROM
			// self-test loop - the "Loading Configuration Partition" hang. Meroko
			// re-evaluates "loc_ctr_cnt > 2048 || MCregister & MCR_PROM_Disable" on
			// every fetch; its 0->1 test in the MBD-MCR case is only a logmsg().
			m_inst_view.select(BIT(m_mcr, MCR_PROM_DISABLE_BIT) ? 1 : 0);
			if (BIT(m_mcr, 21))
			{
				fatalerror("store_mf: NUBUS RESET\n");
			}
			break;
		case 0x03: // stack pointer
			m_sp = m_o_bus & 0x3f;
			break;
		case 0x04: // micro-stack data
			m_stack[m_sp] = m_o_bus;
			break;
		case 0x05: // micro-stack data push
			push(m_o_bus);
			break;
		case 0x06: // IMOD-lo
			m_imod_lo = m_o_bus;
			break;
		case 0x07: // IMOD-hi
			m_imod_hi = m_o_bus;
			break;
		case 0x08: // IBUF
			m_ibuf = m_o_bus;
			break;
		case 0x0f: // TEST-SYNC
			m_md = 0;
			m_bus_error = false;
/*
			m_local_bus_error = 0;
*/
			break;
		case 0x10: // VMA
			m_vma = m_o_bus;
			break;
		case 0x11: // VMA write map level 1
			m_vma = m_o_bus;
			m_vma_lvl1_map[(m_md >> 13) & 0xfff] = m_vma & 0x0fff;
			m_cached_lvl1 = m_vma; // the map put out what was just written to it
			break;
		case 0x12: // VMA write map level 2 control
			// Table 4-17: "the map is addressed from MD and LVL1 and written from
			// VMA(12:00)" - thirteen bits, not sixteen. What the write does not
			// reach is Table 4-16's M(15:13), "Last TM0" / "Last TM1" / "Last
			// locked", which are hardware status for the cycle just performed in
			// the same way the LVL1 read's M(15:12) are (see the cycle-status
			// write-back in vm_resolve_address()). Nothing produces them yet, so
			// today the mask only stops software depositing stray VMA bits into a
			// field that is not its to write.
			m_vma = m_o_bus;
			m_vma_lvl2_control[map2_addr()] = m_vma & 0x1fff;
			break;
		case 0x13: // VMA write map level 2
			m_vma = m_o_bus;
			m_vma_lvl2_map[map2_addr()] = m_vma & 0x3fffff;
			break;
		case 0x14: // VMA start read
			m_vma = m_o_bus;
			read();
			break;
		case 0x15: // VMA start write
			m_vma = m_o_bus;
			write();
			break;
		case 0x16: // VMA start unmapped read
			m_vma = m_o_bus;
			read_unmapped();
			break;
		case 0x17: // VMA start unmapped write
			m_vma = m_o_bus;
			write_unmapped();
			break;
		case 0x18: // MD
			m_md = m_o_bus;
			update_cached_lvl1_from_md();
			break;
		case 0x19: // MD write map level 1
			m_md = m_o_bus;
			m_vma_lvl1_map[(m_md >> 13) & 0xfff] = m_vma & 0x0fff;
			m_cached_lvl1 = m_vma;
			break;
		case 0x1a: // MD write map level 2 control
			m_md = m_o_bus;
			update_cached_lvl1_from_md();
			m_vma_lvl2_control[map2_addr()] = m_vma & 0x1fff; // VMA(12:00) - see 0x12
			break;
		case 0x1b: // MD write map level 2
			m_md = m_o_bus;
			update_cached_lvl1_from_md();
			m_vma_lvl2_map[map2_addr()] = m_vma & 0x3fffff;
			break;
		case 0x1c: // MD start read
			m_md = m_o_bus;
			update_cached_lvl1_from_md();
			read();
			break;
		case 0x1d: // MD start write
			m_md = m_o_bus;
			update_cached_lvl1_from_md();
			write();
			break;
		case 0x1e: // MD start unmapped read
			m_md = m_o_bus;
			update_cached_lvl1_from_md();
			read_unmapped();
			break;
		case 0x1f: // MD start unmapped write
			m_md = m_o_bus;
			update_cached_lvl1_from_md();
			write_unmapped();
			break;
		case 0x20: // PDL buffer pointer data
			m_pdl[m_pdl_pointer & 0x3ff] = m_o_bus;
			break;
		case 0x21: // PDL buffer index data
			m_pdl[m_pdl_index & 0x3ff] = m_o_bus;
			break;
		case 0x24: // PDL buffer pointer push
			m_pdl_pointer = (m_pdl_pointer + 1) & 0x3ff;
			m_pdl[m_pdl_pointer] = m_o_bus;
			break;
		case 0x25: // PDF buffer index push
			m_pdl_index = (m_pdl_index + 1) & 0x3ff;
			m_pdl[m_pdl_index] = m_o_bus;
			break;
		case 0x28: // PDL buffer pointer
			m_pdl_pointer = m_o_bus & 0x3ff;
			break;
		case 0x29: // PDL buffer index
			m_pdl_index = m_o_bus & 0x3ff;
			break;
		case 0x36: // VMA start unmapped read
			m_vma = m_o_bus;
			read_unmapped_byte();
			break;
		case 0x37: // VMA start unmapped write
			m_vma = m_o_bus;
			write_unmapped_byte();
			break;
		case 0x3e: // MD start unmapped read
			m_md = m_o_bus;
			read_unmapped_byte();
			break;
		case 0x3f: // MD start unmapped write
			m_md = m_o_bus;
			write_unmapped_byte();
			break;
		default:
			fatalerror("%04x: store_o_bus MF %02x not implemented\n", m_prev_pc, (m_ir >> 25) & 0x3f);
		}
	}
}


u32 raven_cpu_device::shifter(bool rotate_r, bool rotate_mask, int rot_count)
{
	u32 r = m_m;

	if (rotate_r)
	{
		if (BIT(m_ir, 16))
		{
			r = (r >> rot_count) | (r << (32 - rot_count));
		}
		else
		{
			r = (r << rot_count) | (r >> (32 - rot_count));
		}
	}

	// Rotate mask
	const u8 mask_index_right = rotate_mask ? ((BIT(m_ir, 16) ? (32 - rot_count) : rot_count) & 0x1f) : 0;
	const u8 mask_index_left = (mask_index_right + ((m_ir >> 5) & 0x1f) - 1) & 0x1f;
	u32 mask = shift_mask_left[mask_index_left] & shift_mask_right[mask_index_right];

	// Merge A with R (when mask bit is set)
	m_o_bus = 0;
	for (u32 x = 0x01; x != 0; x <<= 1)
	{
		m_o_bus |= (mask & x) ? (r & x) : (m_a & x);
	}

	return r;
}


bool raven_cpu_device::is_condition(u32 alu_out, u32 carry_out, u32 fixnum_overflow)
{
	u8 condition = (m_ir >> 10) & 0x0f;
	bool result;
	if (!BIT(m_ir, 14))
	{
		switch (condition)
		{
		case 0x00: // LSB of shifter output
			{
				u32 rot_count = m_ir & 0x1f;
				u32 r = rot_count == 0 ? m_m : (BIT(m_ir, 16) ? ((m_m >> rot_count) | (m_m << (32 - rot_count))) : ((m_m << rot_count) | (m_m >> (32 - rot_count))));
				result = BIT(r, 0);
			}
			break;
		case 0x01: // M source less than A source (ALU negative)
			result = (0x80000000 ^ m_m) < (0x80000000 ^ m_a);
			break;
		case 0x02: // less or equal (M source <= A source)
			result = BIT(alu_out, 31);
			break;
		case 0x03: // not (this instruction's own ALU result equals all-ones)
			result = alu_out != 0xffffffff;
			break;
		case 0x04: // page fault
			result = m_page_fault;
			break;
		case 0x05: // page fault or interrupt
			result = m_page_fault || active_int();
			break;
		case 0x06: // page fault or interrupt or sequence break
			result = m_page_fault || active_int() || BIT(m_mcr, 14);
			break;
		case 0x07: // unconditionally true
			result = true;
			break;
		case 0x08: // not (A-TYPE equals M-TYPE)
			result = ((m_m & (0x1f << 25)) != (m_a & (0x1f << 25)));
			break;
		case 0x09: // not (memory busy)
			result = m_memory_busy_counter == 0;
			break;
		case 0x0a: // Q(0)
			result = BIT(m_q, 0);
			break;
		case 0x0b: // bus error on last transfer attempt
			result = m_bus_error;
			break;
		case 0x0c: // not (typed-data overflow)
			result = !fixnum_overflow;
			break;
		case 0x0d: // boxed sign bit (ALU(24))
			result = BIT(alu_out, 24);
			break;
		case 0x0e: // not (interrupt active)
			result = !active_int();
			break;
		default: // 0x0f: reserved
			result = true;
			break;
		}
	}
	else
	{
		// Classifier RAM (T-memory) read: the condition-select field selects one of 16 tag
		// registers, and the bit tested is the type field of the current M source.
		u32 tpos = (m_m >> 25) & 0x1f;
		result = BIT(m_t_memory[condition], tpos);
	}
	return BIT(m_ir, 15) ? !result : result;
}


void raven_cpu_device::push(u32 pc)
{
	m_sp = (m_sp + 1) & 0x3f;
	m_stack[m_sp] = pc & 0xfffff;
}


void raven_cpu_device::pop(bool after_next)
{
	m_next_pc = m_stack[m_sp] & 0xfffff;
	m_sp = (m_sp - 1) & 0x3f;
	handle_popj14(after_next);
}


// The macroinstruction-chaining POPJ's prefetch does not take effect in the
// cycle that starts it: the bus request, and with it the VMA overwrite, land two
// microinstructions later - so the delay-slot instruction of a POPJ-XCT-next
// still sees the VMA the *previous* memory cycle left behind. See
// handle_popj14() for the evidence.
void raven_cpu_device::service_pj14_fetch()
{
	if (!m_pj14_fetch_pending)
		return;

	if (!m_pj14_fetch_go)
	{
		m_pj14_fetch_go = true;
		return;
	}

	m_vma = m_pj14_fetch_vma;
	m_read_data = m_data.read_dword(m_pj14_fetch_addr);
	m_memory_busy_counter = MEMORY_CYCLE_BUSY_CYCLES;
	m_read_pending = true;
	m_pj14_fetch_pending = false;
	m_pj14_fetch_go = false;
}


void raven_cpu_device::handle_popj14(bool after_next)
{
	if (!BIT(m_next_pc, 14))
		return;

	bool const chain_enable = BIT(m_mcr, MCR_MACROINSTRUCTION_CHAINING_ENABLE_BIT);
	bool const need_fetch = BIT(m_mcr, MCR_NEED_FETCH_BIT);

	if (need_fetch)
	{
		// The address is resolved now - the map side effects and any page fault
		// belong to this cycle - but the memory cycle itself is queued, and for
		// the XCT-next forms (RPN=100 Return-XCT-Next and ABJ POPJ-XCT-next) so
		// is the VMA overwrite. The manual documents the dispatch's ISTREAM bit
		// and Table 4-23's transfer types but says nothing about when the
		// chaining POPJ loads VMA; Meroko models it explicitly, saving VMA
		// across the resolve in handle_popj_14_nxt() and restoring it, then
		// overwriting it from the main loop's pj14_fetch_go interlock.
		//
		// Found live at microcode PC $0195,
		//   (C-PDL-POINTER-PUSH) DPB (BYTE-FIELD 25 0) VMA A-1eb
		// the delay slot of $0194's DISPATCH ... AND-POPJ-XCT-NEXT. It builds a
		// locative out of VMA. Overwriting VMA a cycle early made that a
		// locative to the macrocode word being fetched ($160C2BD4) instead of to
		// the operand cell the previous cycle read ($1606A842); the microcode
		// then dereferenced it, read a word of compiled code as if it were a
		// forwarding pointer, chased it into unallocated storage and the Lisp
		// world took ">>Trap #o26136 (TRANS-TRAP) ... #<SYS:DTP-TRAP 0> was read
		// from location #o16050030" during NET::HOST :SET-HOST-DEFAULTS.
		//
		// A page fault is the exception: the VMA overwrite happens immediately,
		// because the fault handler reads VMA to find the faulting address.
		u32 const saved_vma = m_vma;
		m_vma = (m_lc >> 1) & 0x1ffffff;
		u32 const address = vm_resolve_address<MEM_READ>();
		if (!m_page_fault)
		{
			m_pj14_fetch_vma = m_vma;
			m_pj14_fetch_addr = address;
			m_pj14_fetch_pending = true;
			m_pj14_fetch_go = false;
			if (after_next)
				m_vma = saved_vma;
		}
	}

	m_lc++;

	if (!chain_enable)
		m_next_pc |= 2;

	if (!need_fetch && chain_enable)
	{
		m_next_pc |= BIT(m_mcr, MCR_LOCAL_RESET_BIT) ? 2 : 3; // LISP : EXPT
	}

	if (!need_fetch || chain_enable)
	{
		if (BIT(m_lc, 0))
			m_mcr &= ~(1 << MCR_NEED_FETCH_BIT);
		else
			m_mcr |= (1 << MCR_NEED_FETCH_BIT);
	}

	m_next_pc &= 0x7bfff;
}


void raven_cpu_device::perform_abj()
{
	switch ((m_ir >> 51) & 0x07)
	{
	case 0x00: // nop
		break;
	case 0x01: // skip
		m_n = true;
		break;
	case 0x02: // call illop
		push(m_pc);
		m_next_pc = 0x0008;
		m_n = true;
		break;
	case 0x03: // call trap
		push(m_pc);
		m_next_pc = 0x000a;
		m_n = true;
		break;
	case 0x04: // call buserr
		push(m_pc);
		m_next_pc = 0x000c;
		m_n = true;
		break;
	case 0x05: // call ununsed
		push(m_pc);
		m_next_pc = 0x000e;
		m_n = true;
		break;
	case 0x06: // pop
		pop(false);
		m_n = true;
		break;
	case 0x07: // popj after next
		pop(true);
		break;
	default:
		fatalerror("%04x: perform_abj %02x not implemented\n", m_prev_pc, (m_ir >> 51) & 0x07);
	}
}


void raven_cpu_device::execute_alu()
{
	u32 alu_out = 0;
	u32 carry_out = 0;
	u32 fixnum_overflow = 0;
	alu_operation(alu_out, carry_out, fixnum_overflow);

	if (BIT(m_ir, 9))
	{
		// Write classifier RAM (T-memory): the condition-select field IR(13:10) picks which
		// of the 16 tag registers is written, and the ALU result's type field picks the bit.
		u32 mask = 1 << ((alu_out >> 25) & 0x1f);
		u8 index = (m_ir >> 10) & 0x0f;

		if (BIT(alu_out, 30))
		{
			m_t_memory[index] |= mask;
		}
		else
		{
			m_t_memory[index] &= ~mask;
		}
	}

	if (is_condition(alu_out, carry_out, fixnum_overflow))
	{
		perform_abj();
	}

	set_o_bus(alu_out, carry_out);

	switch (m_ir & 0x03)
	{
	case 0x00: // Q nop
		break;
	case 0x01: // Q <<
		m_q <<= 1;
		if (!BIT(alu_out, 31))
		{
			m_q |= 1;
		}
		break;
	case 0x02: // Q >>
		m_q >>= 1;
		if (BIT(alu_out, 0))
		{
			m_q |= 0x8000'0000;
		}
		break;
	case 0x03: // Q load
		m_q = alu_out;
		break;
	}

	store_o_bus();
}


void raven_cpu_device::execute_byte()
{
	u64 alu_out = m_m - m_a - 1;
	// The condition and sense field is common to the ALU, byte and jump formats
	// (2243144-0001A paragraph 4.5.5), so "Typed-data overflow" is testable here
	// too - it used to be hardcoded inactive. Same forced M-A-1 subtract as the
	// jump instruction, so the same signed-overflow test at bit 24 applies.
	u32 const byte_fixnum_overflow = BIT((m_m ^ m_a) & (m_m ^ u32(alu_out)), 24);

	shifter(BIT(m_ir, 17), BIT(m_ir, 18), m_ir & 0x1f);
/*
	u32 r = m_m;
	const u32 rot_count = m_ir & 0x1f;

	// Rotate R
	if (BIT(m_ir, 17))
	{
		if (BIT(m_ir, 16))
		{
			r = (r >> rot_count) | (r << (32 - rot_count));
		}
		else
		{
			r = (r << rot_count) | (r >> (32 - rot_count));
		}
	}

	// Rotate mask
	const u8 mask_index_right = BIT(m_ir, 18) ? ((BIT(m_ir, 16) ? (32 - rot_count) : rot_count) & 0x1f) : 0;
	const u8 mask_index_left = (mask_index_right + ((m_ir >> 5) & 0x1f) - 1) & 0x1f;
	u32 mask = shift_mask_left[mask_index_left] & shift_mask_right[mask_index_right];

	// Merge A with R (when mask bit is set)
	m_o_bus = 0;
	for (u32 x = 0x01; x != 0; x <<= 1)
	{
		m_o_bus |= (mask & x) ? (r & x) : (m_a & x);
	}
*/

	store_o_bus();

	if (is_condition(alu_out, BIT(alu_out, 32), byte_fixnum_overflow))
	{
		perform_abj();
	}
}


void raven_cpu_device::execute_jump()
{
	if (BIT(m_ir, 8))
	{
		if (BIT(m_ir, 31))
		{
			m_a_mem[(m_ir >> 19) & 0x3ff] = m_program.read_qword(m_pc) >> 32;
		}
		else
		{
			m_m_mem[(m_ir >> 19) & 0x3f] = u32(m_program.read_qword(m_pc));
		}
	}
	if (BIT(m_ir, 9))
	{
		m_control_store[m_pc & 0x3fff] = (u64(m_a) << 32) | m_m;
	}

	// The jump instruction forces the ALU operation to a subtract mode so that
	// the ALU related test condition flags are meaningful.
	u64 alu_out = m_m - m_a - 1;
	m_o_bus = alu_out & 0xffffffff;

	if (BIT(m_ir, 17))
		fatalerror("%04x: jump MSEL (IR(17)) set - unexpected, ir=%014x\n", m_prev_pc, m_ir);

	// See execute_byte(): the condition field is shared, so the jump's own forced
	// M-A-1 subtract has to supply a real typed-data overflow flag as well.
	u32 const jump_fixnum_overflow = BIT((m_m ^ m_a) & (m_m ^ u32(alu_out)), 24);
	bool const condition = is_condition(alu_out, BIT(alu_out, 32), jump_fixnum_overflow);

	if (condition)
	{
		u16 new_pc = (m_ir >> 18) & 0x3fff;
		m_n = BIT(m_ir, 5);

		switch ((m_ir >> 6) & 0x03)
		{
		case 0x00: // branch
			m_next_pc = new_pc;
			break;
		case 0x01: // call
			// Figure 4-16 names the jump format's three transfer bits RETURN, PUSH
			// and NOP, and the abbreviated jump field's POPJ-XCT-next is a return
			// as well - so RPN=010 (Call-XCT-next) with ABJ=111 asks the uPCS for a
			// push and a pop in the same cycle. Both act after the delay slot, and
			// with one stack pointer they cancel: the transfer happens and the
			// depth is unchanged, i.e. it degenerates into Branch-XCT-next.
			//
			// 4.5.1.2 does not define this - it says the abbreviated jump
			// operations have "no effect in jump or dispatch microinstructions"
			// apart from POPJ/POPJ-XCT-next, and then that POPJ-XCT-next "should
			// not be set when the destination of a microinstruction is the uPCS",
			// which is exactly what the P bit is. The band's microcode does it
			// anyway (5 instructions in the live control store), so the encoding
			// has to be given the meaning the hardware gave it. Meroko's
			// raven_cpu.c suppresses the push here too, under its own "HACK HERE"
			// comment, and this is the only evidence there is.
			//
			// Found live at $1B27, in the scan loop at $1B0A: with the push, the
			// call to $1B21 returned into $1B29, whose tail
			// (JUMP-XCT-NEXT #x1B1D + MICROSTACK-DATA-POP) pops again - so the
			// loop leaked one microstack entry per iteration. Four iterations in
			// and the uPCS was empty, the next POPJ read 0 and trapped to $0000,
			// which calls the band's error handler at $0039 and halts at $0051.
			//
			// Only RPN=010 is treated this way, matching Meroko. RPN=011 (Call,
			// delay slot inhibited) with the same ABJ also exists in the control
			// store, twice, but nothing has exercised it yet and its two halves
			// disagree about the delay slot as well, so it is left pushing.
			if (m_n || ((m_ir >> 51) & 0x07) < 0x06)
				push(m_n ? m_pc : (m_pc + 1));
			m_next_pc = new_pc;
			break;
		case 0x02: // return
			// RPN=100 Return-XCT-Next defers the chaining prefetch's VMA
			// overwrite past the delay slot; RPN=101 Return does not.
			pop(!m_n);
			break;
		case 0x03: // RPN = 11x: same as branch (R and P both set degenerates to a plain branch)
			m_next_pc = new_pc;
			break;
		default:
			fatalerror("%04x: jump type %02x not implemented\n", m_prev_pc, (m_ir >> 6) & 0x03);
		}
	}
	else
	{
		// The abbreviated jump field is the jump instruction's *else* arm: the
		// RPN transfer above happens when the tested condition is true, and the
		// ABJ only when it is false - never both, since either way it is the one
		// uPCS operation the instruction performs. 2243144-0001A (Processor
		// General Description) Table 4-22 restricts IR(53:51) to 000, 110 or 111
		// in the jump format, and paragraph 4.5.1.2 says the ABJ operations
		// "allow a change of control in ALU and byte microinstructions only,
		// having no effect in jump or dispatch microinstructions" apart from
		// POPJ/POPJ-XCT-next, with "POPJ ... interpreted as POPJ-XCT-Next in
		// jump and dispatch microinstructions" - so 110 behaves as 111 here,
		// i.e. pop without inhibiting the delay slot, and the call/skip codes
		// are ignored rather than run through perform_abj().
		//
		// Found live: microcode PC $26A9 is
		//   POPJ IF-GREATER A-016 M-1c AND-POPJ-XCT-NEXT   (RPN=101, ABJ=111)
		// - "return now, skipping the next instruction, if greater; otherwise
		// execute the next instruction and then return". With the ABJ attached
		// to the true arm this fell through to $26AB instead of returning, and
		// the Lisp world span forever in the $2690-$26AB scan right after the
		// first four demand-paging reads.
		switch ((m_ir >> 51) & 0x07)
		{
		case 0x06: // POPJ - reads as POPJ-XCT-next in a jump microinstruction
		case 0x07: // POPJ-XCT-next
			pop(true);
			break;
		default:
			break;
		}
	}
}


void raven_cpu_device::execute_dispatch()
{
	u32 dispatch_source = 0;

	switch ((m_ir >> 12) & 0x03)
	{
	case 0x00: // R
		{
			u32 mask = (1 << ((m_ir >> 5) & 0x07)) - 1;
			u32 rot_count = m_ir & 0x1f;

			if (m_ir & 0xc00)
			{
				mask &= 0xfffffffe;
			}

			if (BIT(m_ir, 16))
			{
				// right rotate
				dispatch_source = ((m_m >> rot_count) | (m_m << (32 - rot_count))) & mask;
			}
			else
			{
				// left rotate
				dispatch_source = ((m_m << rot_count) | (m_m >> (32 - rot_count))) & mask;
			}
		}
		break;
	case 0x01: // MF bus - MF(29:25), shifted into dispatch address positions (5:1); position 0 comes
	           // from the dispatch address field itself (IR(20)), or is overridden below.
		dispatch_source = ((m_m >> 25) & 0x1f) << 1;
		break;
	default: // IR(13) set: IBUF - the instruction-decode dispatch (2243144-0001A
	         // Table 4-24, IR(13:12) = 1x, "IBUF(09:00) or IBUF(15:06) - auto
	         // selected by the macroinstruction opcode if MISCOP decoding is
	         // enabled"). Paragraph 4.5.9: only IBUF's seven low-order bits are
	         // ORed with the dispatch address field, the next three MSBs replace
	         // the IR field's (hence mir_mask below), the next MSB is the IR bit
	         // ORed with the MISCOP decode status, and the MSB comes from IR -
	         // which is also why "if the MSB of the dispatch address source
	         // select field (IR(13)) is 1, then the most significant address bit
	         // into the dispatch memory is forced to 1". The MISCOP decode test
	         // itself (which macroinstruction opcodes count, and the MCR group
	         // enables that gate it) is not spelled out in the doc; the form here
	         // is Meroko's, raven_cpu.c's own MIR/MIR2 dispatch source.
		{
			u32 const ibuf = BIT(m_lc, 0) ? (m_ibuf & 0xffff) : ((m_ibuf >> 16) & 0xffff);
			if (BIT(m_mcr, MCR_MISC_OP_GROUP_0_BIT)
				&& ((BIT(m_mcr, MCR_MISC_OP_GROUP_1_BIT) ^ 1) & BIT(ibuf, 13)) == 0
				&& ((ibuf >> 9) & 0x0f) == 0x0d)
			{
				dispatch_source = 0x800 | ((BIT(ibuf, 13) ^ 1) << 9) | (ibuf & 0x1ff);
			}
			else
			{
				dispatch_source = 0xc00 | ((ibuf >> 6) & 0x3ff);
			}
		}
		break;
	}

	// GC volatility enable, IR(10) (2243144-0001A Table 4-24): "When IR(10) is
	// set, the LSB of the dispatch address is set to 1 if the GC volatility bit
	// is 1", and per paragraph 4.5.9 it is ORed together with the old-space bit
	// and IR(20) when IR(11) is set too. The doc does not say how the fault bit
	// itself is derived; this comparison of the referencing page's cached
	// volatility against the referenced region's level-1 volatility field (bits
	// 9:7, stored inverted) is Meroko's gc_volatilty_flag.
	u32 gc_volatility_flag = 0;
	if (BIT(m_ir, 10))
	{
		u8 const map_1_volatility = (m_vma_lvl1_map[(m_md >> 13) & 0xfff] >> 7) & 0x07;
		gc_volatility_flag = (m_cached_gc_volatility + 4 > u32(map_1_volatility ^ 7)) ? 0 : 1;
	}

	m_dispatch_constant = (m_ir >> 32) & 0x3ff;

	// Map-Oldspace (IR(11)): when set, dispatch address bit 0 carries the GC
	// "oldspace" answer for the object MD points at - level-1 map entry bit 10 -
	// rather than coming from the rotated source, whose bit 0 the mask above
	// already cleared for exactly this purpose. Without it the dispatch always
	// selected the not-in-oldspace arm; that only starts to matter once the
	// loaded Lisp world runs its own GC-aware code, where it stalled the boot
	// right after the band load. Matches Meroko's oldspace_flag in raven_cpu.c.
	u32 oldspace_flag = 0;
	if (BIT(m_ir, 11))
		oldspace_flag = BIT(m_vma_lvl1_map[(m_md >> 13) & 0xfff], 10) ? 1 : 0;

	// Dispatch address field IR(31:20), inclusively ORed with the selected source's LSBs.
	// On an IBUF (instruction-decode) dispatch the three bits below the two MSBs
	// come from IBUF instead of the IR field, so they are masked out of the IR's
	// contribution first - 2243144-0001A paragraph 4.5.9, "the three next MSBs of
	// IBUF replace the bits from the IR field".
	u32 const mir_mask = BIT(m_ir, 13) ? 0xc7f : 0xfff;
	u32 const disp_address = ((mir_mask & ((m_ir >> 20) & 0xfff)) | dispatch_source | oldspace_flag | gc_volatility_flag) & 0xfff;

	switch ((m_ir >> 8) & 0x03)
	{
	case 0x00: // plain dispatch - multiway transfer of control via the dispatch memory.
	           // Each dispatch memory entry holds a 14-bit target micro-PC and 3 transfer-type
	           // bits (R:P:N) with identical semantics to the jump instruction's R/P/N bits.
		{
			u32 const disp_word = m_dispatch[disp_address];
			u16 const new_pc = disp_word & 0x3fff;
			u8 const jump_op = (disp_word >> 14) & 0x07;

			// IR(15), "Enable instruction stream hardware" (2243144-0001A
			// Table 4-24): a plain dispatch with this bit set also advances the
			// macroinstruction stream - prefetch the next 32-bit word into MD
			// when the low half of LC is exhausted, then step LC and recompute
			// the need-fetch flag. Same sequence handle_popj14() already runs
			// for the macroinstruction-chaining POPJ, and the same as Meroko's
			// MInst_Enable_IStream block in raven_cpu.c's dispatch case.
			//
			// Found live at microcode PC $153D, the macroinstruction decode
			// path: $153C loads LOCATION-COUNTER, $153D is this ISTREAM
			// dispatch, $153E tests for the resulting page fault, and $1546
			// then does (IBUF) SETM MD before $154A dispatches on the opcode.
			// Without the prefetch, MD (and so IBUF, and so the decode
			// dispatch) still held whatever the previous instruction left, and
			// the Lisp world ran off into the microcode's halt loop at $0051.
			if (BIT(m_ir, 15))
			{
				if (BIT(m_mcr, MCR_NEED_FETCH_BIT))
				{
					m_vma = (m_lc >> 1) & 0x1ffffff;
					u32 const address = vm_resolve_address<MEM_READ>();
					if (!m_page_fault)
					{
						m_read_data = m_data.read_dword(address);
						m_memory_busy_counter = MEMORY_CYCLE_BUSY_CYCLES;
						m_read_pending = true;
					}
				}

				m_lc++;

				if (BIT(m_lc, 0))
					m_mcr &= ~(1 << MCR_NEED_FETCH_BIT);
				else
					m_mcr |= (1 << MCR_NEED_FETCH_BIT);
			}

			m_n = BIT(jump_op, 0);

			switch ((jump_op >> 1) & 0x03)
			{
			case 0x00: // branch
				m_next_pc = new_pc;
				break;
			case 0x01: // call
				// IR(17), Stack-own-address (2243144-0001A Table 4-24):
				// "alters the return address pushed on the uPCS by the call
				// transfer type. If the N bit is set, the address of this
				// instruction should be stacked rather than the next
				// instruction." That is how a faulting dispatch arranges to be
				// *re-executed* once the trap handler returns, rather than
				// resumed at its successor. m_pc is already this instruction's
				// address + 1 here, so its own address is m_pc - 1.
				//
				// Found live at microcode PC $027B, the macroinstruction branch
				// dispatch (which carries Stack-Own-Addr): MAME stacked $027C,
				// so when the page-fault handler at $32D8 returned the microcode
				// resumed one instruction past the dispatch, never retried it,
				// and ran on into a trap to $000A.
				push(m_n ? ((BIT(m_ir, 17) ? (m_pc - 1) : m_pc)) : (m_pc + 1));
				m_next_pc = new_pc;
				break;
			case 0x02: // return
				// As in execute_jump(): N picks Return vs Return-XCT-Next.
				pop(!m_n);
				break;
			case 0x03: // R and P both set: dispatch is ignored, next instruction's
			           // execution still depends on N (already applied above).
			           // 2243144-0001A paragraph 4.5.9: "With both R and P set to
			           // one, the dispatch operation is ignored and the execution
			           // of the next instruction is based on the state of the N
			           // bit." That leaves the uPCS free, so this is the one
			           // dispatch-word transfer type under which the abbreviated
			           // jump field can act - Table 4-24 restricts IR(53:51) to
			           // 000/110/111 here, and paragraph 4.5.1.2's "POPJ is
			           // interpreted as POPJ-XCT-Next in jump and dispatch
			           // microinstructions" makes both non-zero codes a pop that
			           // leaves the delay slot running. Same restriction Meroko
			           // expresses with its live_abj flag.
			           //
			           // Found live at microcode PC $027B,
			           //   DISPATCH <A-$001,MD> addr $680 ... AND-POPJ-XCT-NEXT
			           // in the macroinstruction branch path: without the pop the
			           // microcode fell through to $027D instead of returning,
			           // and ended up taking a trap to $000A.
				switch ((m_ir >> 51) & 0x07)
				{
				case 0x06: // POPJ - reads as POPJ-XCT-next in a dispatch microinstruction
				case 0x07: // POPJ-XCT-next
					pop(true);
					break;
				default:
					break;
				}
				break;
			}
		}
		break;
	case 0x01: // read
		m_q = m_dispatch[disp_address];
		break;
	case 0x02: // write
		m_dispatch[disp_address] = m_a & 0x1ffff;
		break;
	default:
		fatalerror("%04x: dispatch mode %02x not implemented, source = %08x\n", m_prev_pc, (m_ir >> 8) & 0x03, dispatch_source);
	}
}


void raven_cpu_device::execute_run()
{
	do {
		// A queued macroinstruction-chaining prefetch takes effect here, at the
		// top of the clock and before this cycle's microinstruction runs - the
		// same position as Meroko's pj14_fetch_go interlock.
		service_pj14_fetch();

		if (m_memory_busy_counter)
		{
			m_memory_busy_counter--;
			if (!m_memory_busy_counter && m_read_pending)
			{
				m_md = m_read_data;
				m_read_pending = false;
			}
		}

		m_ir |= m_imod_lo;
		m_imod_lo = 0;
		m_ir |= u64(m_imod_hi) << 32;
		m_imod_hi = 0;

		// CPU is stalled when targetting VMA or MD while a memory cycle is in progress.
		if (!m_n && m_memory_busy_counter)
		{
			bool const dest_hazard = !BIT(m_ir, 31) && ((m_ir >> 25) & 0x3f) >= 0x10 && ((m_ir >> 25) & 0x3f) <= 0x1f;
			if (dest_hazard)
			{
				m_icount -= m_memory_busy_counter;
				if (m_read_pending)
				{
					m_md = m_read_data;
					m_read_pending = false;
				}
				m_memory_busy_counter = 0;
			}
		}

		if (!m_n)
		{
			debugger_instruction_hook(m_pc);
		}

		m_prev_pc = m_pc;
		m_pc = m_next_pc;
		u64 next_op = m_program.read_qword(m_next_pc++);

		if (!m_n)
		{
			m_a = m_a_mem[(m_ir >> 32) & 0x3ff];
			m_m = get_m_source();

			switch (m_ir & (u64(3) << 54))
			{
			case u64(0) << 54: execute_alu(); break;
			case u64(1) << 54: execute_byte(); break;
			case u64(2) << 54: execute_jump(); break;
			case u64(3) << 54: execute_dispatch(); break;
			}
		}
		else
		{
			m_n = false;
		}

		m_ir = next_op;

		m_icount--;
	} while (m_icount > 0);
}


void raven_cpu_device::state_string_export(const device_state_entry &entry, std::string &str) const
{
	switch (entry.index())
	{
	case STATE_GENPC:
		str = string_format("%04x" , m_pc);
		break;
	}
}


void raven_cpu_device::execute_set_input(int inputnum, int state)
{
}
