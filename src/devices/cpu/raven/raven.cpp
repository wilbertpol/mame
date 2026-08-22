// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/******************************************************************************

    TI Explorer I Raven cpu core emulation.

******************************************************************************/

#include "emu.h"
#include "raven.h"
#include "raven_dasm.h"


namespace {

static constexpr u8 MCR_SELF_TEST_FLAG_BIT = 27;
static constexpr u8 MCR_MACROINSTRUCTION_CHAINING_ENABLE_BIT = 26;
static constexpr u8 MCR_LOOP_ON_SELF_TEST_BIT = 23;
static constexpr u8 MCR_NEED_FETCH_BIT = 22;
static constexpr u8 MCR_LOCAL_RESET_BIT = 20;
static constexpr u8 MCR_INT_ENABLE_BIT = 15;
static constexpr u8 MCR_MEMORY_CYCLE_ENABLE_BIT = 8;
static constexpr u8 MCR_SUB_SYSTEM_FLAG_BIT = 7;
static constexpr u8 MCR_TEST_FAIL_FLAG_BIT = 6;

static constexpr u8 MEMORY_CYCLE_BUSY_CYCLES = 6;


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
	, m_data_config("data", ENDIANNESS_LITTLE, 32, EXTERNAL_ADDRESS_BITS)
	, m_local_bus_config("local_bus", ENDIANNESS_LITTLE, 32, EXTERNAL_ADDRESS_BITS)
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
	save_item(NAME(m_page_fault));
	save_item(NAME(m_read_data));
	save_item(NAME(m_memory_busy_counter));
	save_item(NAME(m_read_pending));
	save_item(NAME(m_pending_interrupts));
	save_item(NAME(m_nubus_error));
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
	m_nubus_error = false;
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


void raven_cpu_device::read()
{
	m_nubus_error = false;
	u32 address = vm_resolve_address<MEM_READ>();

	if (!m_page_fault)
	{
		m_read_data = m_data.read_dword(address);
		m_memory_busy_counter = MEMORY_CYCLE_BUSY_CYCLES;
		m_read_pending = true;
		if (m_vma == 0xf6c00000)
		{
			m_memory_busy_counter = 0;
			m_read_pending = false;
			m_md = m_read_data;
		}
	}
}


void raven_cpu_device::write()
{
	m_nubus_error = false;
	u32 address = vm_resolve_address<MEM_WRITE>();

	if (!m_page_fault)
	{
		m_data.write_dword(address, m_md);
		m_memory_busy_counter = MEMORY_CYCLE_BUSY_CYCLES;
		m_read_pending = false;
		if (m_vma == 0xf6c00000)
		{
			m_memory_busy_counter = 0;
		}
	}
}


void raven_cpu_device::read_unmapped()
{
	m_nubus_error = false;
	m_page_fault = false;
	m_local_bus_miss = false;
	m_read_data = m_local_bus.read_dword(m_vma);
	if (m_local_bus_miss)
	{
		m_read_data = m_data.read_dword(m_vma);
		m_memory_busy_counter = MEMORY_CYCLE_BUSY_CYCLES;
	}
	else
	{
		m_memory_busy_counter = MEMORY_CYCLE_BUSY_CYCLES;
	}
	m_read_pending = true;
	// TODO Get rid of this hack
	if (/*m_vma == 0xf6c00000 ||*/ m_vma == 0x3db00000)
	{
		m_memory_busy_counter = 0;
		m_read_pending = false;
		m_md = m_read_data;
	}
}


void raven_cpu_device::write_unmapped()
{
	m_nubus_error = false;
	m_page_fault = false;
	m_local_bus_miss = false;
	m_local_bus.write_dword(m_vma, m_md);
	if (m_local_bus_miss)
	{
		m_data.write_dword(m_vma, m_md);
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
	m_nubus_error = false;
	m_page_fault = false;
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
	m_nubus_error = false;
	m_page_fault = false;
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
	m_nubus_error = true;
	return 0xffffffff;
}


void raven_cpu_device::nubus_unmapped_w(offs_t offset, u32 data, u32 mem_mask)
{
	m_nubus_error = true;
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
		if (!m2_writeable || (m2_forceable))
		{
			m_page_fault = true;
		}
		if (!m_page_fault)
		{
//			fatalerror("%04x: vm_resolve_address write not implemented", m_prev_pc);
		}
	}
	else
	{
		if (!m_page_fault)
		{
//			fatalerror("%04x: vm_resolve_address read not implemented", m_prev_pc);
		}
	}
	return address;
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
		// TODO
		switch ((m_ir >> 42) & 0x3f)
		{
		case 0x00: // VMA
			return m_vma;
		case 0x01: // Q
			return m_q;
		case 0x02: // IBUF argument offset field zero extended
			return m_ibuf & 0x3f;
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
			return m_vma_lvl1_map[(m_md >> 13) & 0xfff];
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


void raven_cpu_device::add32(u32 a, u32 m, u32 carry_in, u32 &res, u32 &carry_out, u32 &fixnum_overflow)
{
	const u64 result = u64(a) + u64(m) + carry_in;
	res = u32(result);
	carry_out = BIT(result, 32);
	fixnum_overflow = BIT((a & 0xffffff) + (m & 0xffffff) + carry_in, 24);
}



void raven_cpu_device::sub32(u32 a, u32 m, u32 carry_in, u32 &res, u32 &carry_out, u32 &fixnum_overflow)
{
	const u64 result = u64(m) - u64(a) - (carry_in ? 0 : 1);
	res = u32(result);
	carry_out = BIT(result, 32);
	fixnum_overflow = BIT((m & 0xffffff) - (a & 0xffffff) - (carry_in ? 0 : 1), 24);
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
			fatalerror("%04x: set_o_bus tagged %02x not implemented\n", m_prev_pc, (m_ir >> 16) & 0x07);
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
			m_nubus_error = false;
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
			break;
		case 0x12: // VMA write map level 2 control
			m_vma = m_o_bus;
			m_vma_lvl2_control[map2_addr()] = m_vma & 0xffff;
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
			break;
		case 0x19: // MD write map level 1
			m_md = m_o_bus;
			m_vma_lvl1_map[(m_md >> 13) & 0xfff] = m_vma & 0x0fff;
			break;
		case 0x1a: // MD write map level 2 control
			m_md = m_o_bus;
			m_vma_lvl2_control[map2_addr()] = m_vma & 0xffff;
			break;
		case 0x1b: // MD write map level 2
			m_md = m_o_bus;
			m_vma_lvl2_map[map2_addr()] = m_vma & 0x3fffff;
			break;
		case 0x1c: // MD start read
			m_md = m_o_bus;
			read();
			break;
		case 0x1d: // MD start write
			m_md = m_o_bus;
			write();
			break;
		case 0x1e: // MD start unmapped read
			m_md = m_o_bus;
			read_unmapped();
			break;
		case 0x1f: // MD start unmapped write
			m_md = m_o_bus;
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
			result = m_nubus_error;
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
	// TODO what is set in the higher bits on the stack??
	m_sp = (m_sp + 1) & 0x3f;
	m_stack[m_sp] = pc;
}


void raven_cpu_device::pop()
{
	// TODO What to do with the other bits from the mpcs?
	m_next_pc = m_stack[m_sp] & 0xfffff;
	m_sp = (m_sp - 1) & 0x3f;
	handle_popj14();
}


void raven_cpu_device::handle_popj14()
{
	if (!BIT(m_next_pc, 14))
		return;

	bool const chain_enable = BIT(m_mcr, MCR_MACROINSTRUCTION_CHAINING_ENABLE_BIT);
	bool const need_fetch = BIT(m_mcr, MCR_NEED_FETCH_BIT);

	if (need_fetch)
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
		pop();
		m_n = true;
		break;
	case 0x07: // popj after next
		pop();
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

	if (is_condition(alu_out, BIT(alu_out, 32), 0))
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

	bool const condition = is_condition(alu_out, BIT(alu_out, 32), 0);

	if (condition)
	{
		if ((m_ir >> 51) & 0x07)
		{
			perform_abj();
		}
		else
		{
			u16 new_pc = (m_ir >> 18) & 0x3fff;
			m_n = BIT(m_ir, 5);

			switch ((m_ir >> 6) & 0x03)
			{
			case 0x00: // branch
				m_next_pc = new_pc;
				break;
			case 0x01: // call
				push(m_n ? m_pc : (m_pc + 1));
				m_next_pc = new_pc;
				break;
			case 0x02: // return
				pop();
				break;
			case 0x03: // RPN = 11x: same as branch (R and P both set degenerates to a plain branch)
				m_next_pc = new_pc;
				break;
			default:
				fatalerror("%04x: jump type %02x not implemented\n", m_prev_pc, (m_ir >> 6) & 0x03);
			}
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
	default:
		fatalerror("%04x: dispatch source address %02x not implemented\n", m_prev_pc, (m_ir >> 12) & 0x03);
	}

//	u8 gc_volatility_flag = 0;

	if (BIT(m_ir, 10))
	{
//		u8 map_1_volatility = (m_vma_lvl1_map[(m_md >> 13) & 0xfff] >> 7) & 0x07;

		fatalerror("%04x: dispatch gc volatility not implemented\n", m_prev_pc);
//		gc_volatility_flag = (m_cached_gc_volatility + 4 > (map_1_volatility ^ 7)) ? 0 : 1;
	}

	m_dispatch_constant = (m_ir >> 32) & 0x3ff;

	switch ((m_ir >> 8) & 0x03)
	{
	case 0x01: // read
		m_q = m_dispatch[(m_ir >> 20) & 0xfff];
		break;
	case 0x02: // write
		m_dispatch[(m_ir >> 20) & 0xfff] = m_a & 0x1ffff;
		break;
	default:
		fatalerror("%04x: dispatch mode %02x not implemented, source = %08x\n", m_prev_pc, (m_ir >> 8) & 0x03, dispatch_source);
	}
}


void raven_cpu_device::execute_run()
{
	do {
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
