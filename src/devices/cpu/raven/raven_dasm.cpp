// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/******************************************************************************

    TI Explorer I Raven cpu disassembler.

TODO:
- Review DISPATCH disassembly

******************************************************************************/

#include "emu.h"
#include "raven_dasm.h"


namespace {

static constexpr u8 A_USED = (1 << 0);
static constexpr u8 M_USED = (1 << 1);
static constexpr u8 B_USED = (1 << 2);

struct desc_info {
	const char *name;
	const u8 source_used;
};

static const char *const o_bus_destination[64] =
{
	"", // NOP
	"LOCATION-COUNTER",
	"MCR",
	"MICROSTACK-POINTER",
	"MICROSTACK-DATA",
	"MICROSTACK-DATA-PUSH",
	"IMOD-LOW",
	"IMOD-HIGH",
	"IBUF",
	"UNUSED-11",
	"UNUSED-12",
	"UNUSED-13",
	"UNUSED-14",
	"UNUSED-15",
	"UNUSED-16",
	"TEST-SYNCH",
	"VMA",
	"VMA-WRITE-MAP-LEVEL-1",
	"VMA-WRITE-MAP-LEVEL-2-CONTROL",
	"VMA-WRITE-MAP-LEVEL-2-ADDRESS",
	"VMA-START-READ",
	"VMA-START-WRITE",
	"VMA-START-UNMAPPED-READ",
	"VMA-START-UNMAPPER-WRITE",
	"MD",
	"MD-WRITE-MAP-LEVEL-1",
	"MD-WRITE-MAP-LEVEL-2-CONTROL",
	"MD-WRITE-MAP-LEVEL-2-ADDRESS",
	"MD-START-READ",
	"MD-START-WRITE",
	"MD-START-UNMAPPED-READ",
	"MD-START-UNMAPPED-WRITE",
	"C-PDL-POINTER",
	"C-PDL-INDEX",
	"UNUSED-42",
	"UNUSED-43",
	"C-PDL-POINTER-PUSH",
	"C-PDL-POINTER-INC",
	"UNUSED-46",
	"UNUSED-47",
	"PDL-POINTER",
	"PDL-INDEX",
	"UNUSED-52",
	"UNUSED-53",
	"UNUSED-54",
	"UNUSED-55",
	"UNUSED-56",
	"UNUSED-57",
	"UNUSED-60",
	"UNUSED-61",
	"UNUSED-62",
	"UNUSED-63",
	"UNUSED-64",
	"UNUSED-65",
	"VMA-START-UNMAPPED-READ-NUBUS", // Same as VMA-Start-Unmapped-Read; sets TM0 on NuBus access
	"VMA-START-UNMAPPED-WRITE-NUBUS", // Same as VMA-Start-Unmapped-Write; sets TM0 on NuBus access
	"UNUSED-70",
	"UNUSED-71",
	"UNUSED-72",
	"UNUSED-73",
	"UNUSED-74",
	"UNUSED-75",
	"MD-START-UNMAPPED-READ-NUBUS", // Same as MD-Start-Unmapped-Read; sets TM0 on NuBus access
	"MD-START-UNMAPPED-WRITE-NUBUS" // Same as MD-Start-Unmapped-Write; sets TM0 on NuBus access
};


static const char *const m_mf_source[64] =
{
	"VMA",
	"Q",
	"MIB-ARGUMENT-OFFSET-FIELD",
	"MICROSTACK-POINTER",
	"MCR",
	"LOCATION-COUNTER",
	"MEMORY-MAP-LEVEL-2-ADDRESS",
	"READ-I-ARG",
	"MEMORY-MAP-LEVEL-1",
	"MEMORY-MAP-LEVEL-2-CONTROL",
	"MIB",
	"MIB-BRANCH-OFFSET-FIELD",
	"UNUSED-14",
	"UNUSED-15",
	"UNUSED-16",
	"UNUSED-17",
	"MICROSTACK-DATA",
	"MICROSTACK-DATA-POP",
	"MD",
	"UNUSED-23",
	"UNUSED-24",
	"UNUSED-25",
	"UNUSED-26",
	"UNUSED-27",
	"UNUSED-30",
	"UNUSED-31",
	"UNUSED-32",
	"UNUSED-33",
	"UNUSED-34",
	"UNUSED-35",
	"UNUSED-36",
	"UNUSED-37",
	"C-PDL-BUFFER-POINTER",
	"C-PDL-BUFFER-INDEX",
	"UNUSED-42",
	"UNUSED-43",
	"C-PDL-BUFFER-POINTER-POP",
	"C-PDL-BUFFER-INDEX-DECREMENT",
	"UNUSED-46",
	"UNUSED-47",
	"PDL-POINTER",
	"PDL-INDEX",
	"UNUSED-52",
	"UNUSED-53",
	"PDL-POINTER-POP",
	"PDL-INDEX-DECREMENT",
	"UNUSED-56",
	"UNUSED-57",
	"UNUSED-60",
	"UNUSED-61",
	"UNUSED-62",
	"UNUSED-63",
	"UNUSED-64",
	"UNUSED-65",
	"UNUSED-66",
	"UNUSED-67",
	"UNUSED-70",
	"UNUSED-71",
	"UNUSED-72",
	"UNUSED-73",
	"UNUSED-74",
	"UNUSED-75",
	"UNUSED-76",
	"UNUSED-77"
};


static const desc_info alu_operation[32+9] =
{
	{ "SETZ", 0 },
	{ "AND", A_USED | M_USED },
	{ "ANDCA", A_USED | M_USED },
	{ "SETM", M_USED },
	{ "ANDCM", A_USED | M_USED },
	{ "SETA", A_USED },
	{ "XOR", A_USED | M_USED },
	{ "IOR", A_USED | M_USED },
	{ "ANDCB", A_USED | M_USED },
	{ "EQV", A_USED | M_USED },
	{ "SETCA", A_USED },
	{ "ORCA", A_USED | M_USED },
	{ "SETCM", M_USED },
	{ "ORCM", A_USED | M_USED },
	{ "ORCB", A_USED | M_USED },
	{ "SETO", 0 },

	{ "OPC-MUL", A_USED | M_USED },
	{ "MULTIPLY-STEP-LAST", A_USED | M_USED },
	{ "OPC-DIV", A_USED | M_USED },
	{ "OPC-DIV-FIRST", A_USED | M_USED },
	{ "DIVIDE-REMAINDER-CORRECTION-STEP", A_USED | M_USED },
	{ "ALU-25", A_USED | M_USED },
	{ "ALU-26", A_USED | M_USED },
	{ "ALU-27", A_USED | M_USED },
	{ "ALU-30", A_USED | M_USED },
	{ "ADD", A_USED | M_USED },
	{ "ALU-32", A_USED | M_USED },
	{ "ALU-33", A_USED | M_USED },
	{ "M", M_USED },
	{ "ALU-35", A_USED | M_USED },
	{ "M-A-1", A_USED | M_USED },
	{ "M+M", M_USED },

	{ "MULTIPLY-STEP", A_USED | M_USED },
	{ "DIVIDE-STEP", A_USED | M_USED },
	{ "DIVIDE-FIST-STEP", A_USED | M_USED },
	{ "M+A+1", A_USED | M_USED },
	{ "M+1", M_USED },
	{ "SUB", A_USED | M_USED },
	{ "M+M+1", M_USED }
};


static const char *const o_bus_control[8] =
{
	"OBus-A-Bus ",
	"OBus-R-Bus ",
	"OBus-A-Bus2 ",
	"", // OBus-Normal
	"OBus-LeftShift-1 ",
	"OBus-RightShift-1 ",
	"OBus-Pointer-Extend ",
	"OBus-Mirror "
};


static const desc_info condition[64] =
{
	{ "BIT-SET", M_USED | B_USED },                             // 00: LSB of shifter output R(00) (normally used only for M sources)
	{ "LESS", A_USED | M_USED },                                // 01: M source less than A source (ALU negative)
	{ "LESS-OR-EQUAL", A_USED | M_USED },                       // 02: M source less than or equal to A source
	{ "NOT-EQUAL", A_USED | M_USED },                           // 03: Not (M source equals A source)
	{ "PAGE-FAULT", 0 },                                        // 04
	{ "PAGE-FAULT-OR-INTERRUPT_PENDING", 0 },                   // 05
	{ "PAGE-FAULT-OR-INTERRUPT_PENDING-OR-SEQUENCE-BREAK", 0 }, // 06
	{ "ALWAYS", 0 },                                            // 07
	{ "TAG-NOT-EQUAL", A_USED | M_USED },                       // 08: Not (A-TYPE equals M-TYPE)
	{ "NOT-MEMORY-BUSY", 0 },                                   // 09
	{ "Q0", 0 },                                                // 0a: Q(0)
	{ "NUBUS-ERROR", 0 },                                       // 0b
	{ "NOT-FIXNUM-OVERFLOW", 0 },                               // 0c
	{ "NEGATIVE", 0 },                                          // 0d
	{ "NO-INTERRUPT-PENDING", 0 },                              // 0e
	{ "RESERVED-15", 0 },                                       // 0f
	{ "RESERVED-16", 0 },                                       // 10
	{ "RESERVED-17", 0 },                                       // 11
	{ "RESERVED-18", 0 },                                       // 12
	{ "RESERVED-19", 0 },                                       // 13
	{ "RESERVED-20", 0 },                                       // 14
	{ "RESERVED-21", 0 },                                       // 15
	{ "RESERVED-22", 0 },                                       // 16
	{ "RESERVED-23", 0 },                                       // 17
	{ "RESERVED-24", 0 },                                       // 18
	{ "RESERVED-25", 0 },                                       // 19
	{ "RESERVED-26", 0 },                                       // 1a
	{ "RESERVED-27", 0 },                                       // 1b
	{ "RESERVED-28", 0 },                                       // 1c
	{ "RESERVED-29", 0 },                                       // 1d
	{ "RESERVED-30", 0 },                                       // 1e
	{ "IN-CLASS", M_USED },                                     // 1f

	{ "BIT-CLEAR", M_USED | B_USED },
	{ "GREATER-OR-EQUAL", A_USED | M_USED },
	{ "GREATER", A_USED | M_USED },
	{ "EQUAL", A_USED | M_USED },
	{ "NOT-PAGE-FAULT", 0 },
	{ "NOT-PAGE-FAULT-OR-INTERRUPT_PENDING", 0 },
	{ "NOT-PAGE-FAULT-OR-INTERRUPT_PENDING-OR-SEQUENCE-BREAK", 0 },
	{ "NEVER", 0 },
	{ "TAG-EQUAL", A_USED | M_USED },
	{ "MEMORY-BUSY", 0 },
	{ "NOT-Q0", 0 },
	{ "NOT-NUBUS-ERROR", 0 },
	{ "FIXNUM-OVERFLOW", 0 },
	{ "POSITIVE", 0 },
	{ "INTERRUPT-PENDING", 0 },
	{ "NOT-RESERVED-15", 0 },
	{ "NOT-RESERVED-16", 0 },
	{ "NOT-RESERVED-17", 0 },
	{ "NOT-RESERVED-18", 0 },
	{ "NOT-RESERVED-19", 0 },
	{ "NOT-RESERVED-20", 0 },
	{ "NOT-RESERVED-21", 0 },
	{ "NOT-RESERVED-22", 0 },
	{ "NOT-RESERVED-23", 0 },
	{ "NOT-RESERVED-24", 0 },
	{ "NOT-RESERVED-25", 0 },
	{ "NOT-RESERVED-26", 0 },
	{ "NOT-RESERVED-27", 0 },
	{ "NOT-RESERVED-28", 0 },
	{ "NOT-RESERVED-29", 0 },
	{ "NOT-RESERVED-30", 0 },
	{ "NOT-IN-CLASS", M_USED },
};


static const char *const abj[8] =
{
	"AND-NOP",
	"AND-SKIP",
	"AND-CALL-ILLOP",
	"AND-CALL-TRAP",
	"AND-CALL-BUSERR",
	"AND-UNUSED",
	"AND-POPJ",
	"AND-POPJ-XCT-NEXT"
};


static const char *const q_control[4] =
{
	"", // Q-NOP
	"SHIFT-Q-LEFT",
	"SHIFT-Q-RIGHT",
	"LOAD-Q"
};


static const char *const byte_operation[4] =
{
	"NOP",
	"LDB",
	"Selective-Deposit",
	"DPB"
};


static const char *const jump_operation[8] =
{
	"JUMP-XCT-NEXT",
	"JUMP",
	"CALL-XCT-NEXT",
	"CALL",
	"POPJ-XCT-NEXT",
	"POPJ",
	"JUMP-XCT-NEXT", // same as 000
	"JUMP"           // same as 001
};


// Dispatch address source field, IR(13:12) (Table 4-24). 1x is IBUF-derived in both cases.
static const char *const dispatch_src[4] =
{
	"R-Bus",
	"MF-Bus",
	"IBUF",
	"IBUF"
};


static const char *const dispatch_op[4] =
{
	"DISPATCH",
	"I-READ",
	"I-WRITE",
	"DISP-ERR"
};

} // anonymous namespace


u32 raven_disassembler::opcode_alignment() const
{
	return 1;
}


void raven_disassembler::destination(std::ostream &stream, u64 op)
{
	if (BIT(op, 31))
	{
		util::stream_format(stream, "(A-%03x) ", (op >> 19) & 0x3ff);
	}
	else
	{
		if (((op >> 25) & 0x3f))
		{
			if (((op >> 19) & 0x3f))
				util::stream_format(stream, "(M-%02x,%s) ", (op >> 19) & 0x3f, o_bus_destination[((op >> 25) & 0x3f)]);
			else
				util::stream_format(stream, "(%s) ", o_bus_destination[((op >> 25) & 0x3f)]);
		}
		else
		{
			util::stream_format(stream, "(M-%02x) ", (op >> 19) & 0x3f);
		}
	}
}


void raven_disassembler::sources(std::ostream &stream, u64 op)
{
	if (BIT(op, 48))
		util::stream_format(stream, "%s", m_mf_source[(op >> 42) & 0x3f]);
	else
		util::stream_format(stream, "M-%02x", (op >> 42) & 0x3f);
	util::stream_format(stream, " A-%03x", (op >> 32) & 0x3ff);
}


void raven_disassembler::conditional(std::ostream &stream, u64 op)
{
	const u8 cond = (op >> 10) & 0x3f;
	if (cond != 0x07)
	{
		util::stream_format(stream, " IF-%s", condition[cond].name);
		if (condition[cond].source_used & B_USED)
			util::stream_format(stream, " (BYTE-FIELD 1 %d)", (op & 0x1f));
		if (condition[cond].source_used & A_USED)
			util::stream_format(stream, " A-%03x", (op >> 32) & 0x3ff);
		if (condition[cond].source_used & M_USED)
		{
			if (BIT(op, 48))
				util::stream_format(stream, " %s", m_mf_source[(op >> 42) & 0x3f]);
			else
				util::stream_format(stream, " M-%02x", (op >> 42) & 0x3f);
		}
	}
}


void raven_disassembler::disassemble_alu(std::ostream &stream, u64 op)
{
	// Destination
	destination(stream, op);

	// Typed-data
	if (BIT(op, 8))
	{
		util::stream_format(stream, "TYPED-DATA ");
	}

	// Operation
	u8 alu_op = (op >> 3) & 0x1f;
	u8 q_c = op & 0x03;
	u8 o_c = (op >> 16) & 0x7;
	bool carry_in = BIT(op, 2);
	switch (alu_op)
	{
	case 0x10: // opc-mul
		if (o_c == 0x05 && q_c == 0x02)
		{
			alu_op = 32;
			q_c = 0;
			o_c = 3;
		}
		break;
	case 0x12: // opc-div
		if (o_c == 0x04 && q_c == 0x01)
		{
			alu_op = 33;
			q_c = 0;
			o_c = 3;
		}
		break;
	case 0x13: // opc-div-first
		if (o_c == 0x04 && q_c == 0x01)
		{
			alu_op = 34;
			q_c = 0;
			o_c = 3;
		}
		break;
	case 0x19: // add
		if (carry_in)
		{
			alu_op = 35;
			carry_in = false; // To suppress separate output of CARRY-IN
		}
		break;
	case 0x1c: // m+
		if (carry_in)
		{
			alu_op = 36;
			carry_in = false; // To suppress separate output of CARRY-IN
		}
		break;
	case 0x1e: // m-a-1
		if (carry_in)
		{
			alu_op = 37;
			carry_in = false; // To suppress separate output of CARRY-IN
		}
		break;
	case 0x1f: // m+m
		if (carry_in)
		{
			alu_op = 38;
			carry_in = false; // To suppress separate output of CARRY-IN
		}
		break;
	}
	util::stream_format(stream, "%s", alu_operation[alu_op].name);

	// Q control
	if (q_c)
		util::stream_format(stream, " %s", q_control[q_c]);

	if (alu_operation[alu_op].source_used & M_USED)
	{
		if (BIT(op, 48))
			util::stream_format(stream, " %s", m_mf_source[(op >> 42) & 0x3f]);
		else
			util::stream_format(stream, " M-%02x", (op >> 42) & 0x3f);
	}
	if (alu_operation[alu_op].source_used & A_USED)
		util::stream_format(stream, " A-%03x", (op >> 32) & 0x3ff);

	if (carry_in)
		util::stream_format(stream, " CARRY-IN");

	// O-control
	if (o_c != 3)
		util::stream_format(stream, " %s", o_bus_control[o_c]);

	// T-Write?
	if (BIT(op, 9))
		util::stream_format(stream, " TM-Wt-$%02x ", (op >> 10) & 0x0f);

	conditional(stream, op);
}


void raven_disassembler::disassemble_byte(std::ostream &stream, u64 op)
{
	// Destination
	destination(stream, op);

	// Operation
	if (op & 0x10000)
		util::stream_format(stream, "RIGHT ");

	util::stream_format(stream, "%s (BYTE-FIELD %d %d) ", byte_operation[(op >> 17) & 0x3], (op >> 5) & 0x1f, (u16) (op & 0x1f));

	// Sources
	sources(stream, op);

	conditional(stream, op);
}


void raven_disassembler::disassemble_jump(std::ostream &stream, u64 op)
{
	// Operation and new PC
	if (((op >> 5) & 0x07) != 0x04 && ((op >> 5) & 0x07) != 0x05)
		util::stream_format(stream, "%s #x%04X", jump_operation[(op >> 5) & 0x7], (op >> 18) & 0x3fff);
	else
		util::stream_format(stream, "%s", jump_operation[(op >> 5) & 0x7]);

	// WCS Read or write
	if (op & 0x200)
		util::stream_format(stream, " WCS-WRITE");
	if (op & 0x100)
		util::stream_format(stream, " WCS-READ");

	conditional(stream, op);
}


void raven_disassembler::disassemble_dispatch(std::ostream &stream, u64 op)
{
	util::stream_format(stream, "DISPATCH ");

	// Sources
	util::stream_format(stream, "<A-$%03x,", (op >> 32) & 0x3ff);
	if (BIT(op, 48))
	{
		util::stream_format(stream, "%s> ", m_mf_source[(op >> 42) & 0x3f]);
	}
	else
	{
		util::stream_format(stream, "M-$%02x> ", (op >> 42) & 0x3f);
	}

	util::stream_format(stream, "addr $%03x ", (op >> 20) & 0xfff);

	util::stream_format(stream, "%s", BIT(op, 17) ? "Stack-Own-Addr " : "");

	util::stream_format(stream, "%s", BIT(op, 15) ? "ISTREAM " : "");

	util::stream_format(stream, "%s", BIT(op, 11) ? "Old-Space " : "");

	util::stream_format(stream, "%s", BIT(op, 10) ? "GC-Volatility " : "");

	util::stream_format(stream, "%s ", dispatch_src[(op >> 12) & 0x03]);

	util::stream_format(stream, "%s ", dispatch_op[(op >> 8) & 0x03]);

	util::stream_format(stream, "(L=%d C=%d)", (op >> 5) & 0x07, op & 0x1f);
}


offs_t raven_disassembler::disassemble(std::ostream &stream, offs_t pc, const data_buffer &opcodes, const data_buffer &params)
{
	u32 dasmflags = 0;
	u64 op = opcodes.r64(pc);

	stream << '(';
	switch (op & (u64(3) << 54))
	{
	case u64(0) << 54: // ALU
		disassemble_alu(stream, op);
		break;

	case u64(1) << 54: // BYTE
		disassemble_byte(stream, op);
		break;

	case u64(2) << 54: // JUMP
		disassemble_jump(stream, op);
		break;

	case u64(3) << 54: // DISPATCH, unsure about the disassembly for this instruction
		disassemble_dispatch(stream, op);
		break;
	}
	// abj
	if (op & u64(0x38000000000000))
		util::stream_format(stream, " %s", abj[(op >> 51) & 0x07]);

	stream << ')';

	return 1 | dasmflags | SUPPORTED;
}
