// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/******************************************************************************

    TI Explorer I Raven cpu disassembler.

This code is mostly based on the disassembly code in the Meroko emulator.
TODO:
- improve DISPATCH disassembly
- With information from the nevermore disassembler rework some instruction
  disassemblies to look more like the original input assembly.

******************************************************************************/

#include "emu.h"
#include "raven_dasm.h"


const char *const raven_disassembler::o_bus_destination[64] =
{
	"", // NOP
	",LC",
	",MCR",
	",uStack-Pointer",
	",uStack-Data",
	",uStack-Data-Push",
	",IMOD-low",
	",IMOD-high",
	",IBUF",
	",unused-11",
	",unused-12",
	",unused-13",
	",unused-14",
	",unused-15",
	",unused-16",
	",TEST-SYNC",
	",VMA",
	",VMA-Write-Map-LV1",
	",VMA-Write-Map-LV2-C",
	",VMA-Write-Map-LV2-A",
	",VMA-Start-Read",
	",VMA-Start-Write",
	",VMA-Start-Unmapped-Read",
	",VMA-Start-Unmapped-Write",
	",MD",
	",MD-Write-Map-LV1",
	",MD-Write-Map-LV2-C",
	",MD-Write-Map-LV2-A",
	",MD-Start-Read",
	",MD-Start-Write",
	",MD-Start-Unmapped-Read",
	",MD-Start-Unmapped-Write",
	",C-PDL-Pointer",
	",C-PDL-Index",
	",unused-42",
	",unused-43",
	",C-PDL-Pointer-Push",
	",C-PDL-Pointer-Inc",
	",unused-46",
	",unused-47",
	",PDL-Pointer",
	",PDL-Index",
	",unused-52",
	",unused-53",
	",unused-54",
	",unused-55",
	",unused-56",
	",unused-57",
	",unused-60",
	",unused-61",
	",unused-62",
	",unused-63",
	",unused-64",
	",unused-65",
	",VMA-Start-Unmapped-Read-NuBus", // Same as VMA-Start-Unmapped-Read; sets TM0 on NuBus access
	",VMA-Start-Unmapped-Write-NuBus", // Same as VMA-Start-Unmapped-Write; sets TM0 on NuBus access
	",unused-70",
	",unused-71",
	",unused-72",
	",unused-73",
	",unused-74",
	",unused-75",
	",MD-Start-Unmapped-Read-NuBus", // Same as MD-Start-Unmapped-Read; sets TM0 on NuBus access
	",MD-Start-Unmapped-Write-NuBus" // Same as MD-Start-Unmapped-Write; sets TM0 on NuBus access
};


const char *const raven_disassembler::m_mf_source[64] =
{
	"VMA",
	"Q",
	"IBUF-Arg",
	"Micro-Stack-Ptr",
	"MCR",
	"Location-Counter",
	"Map-lv2-addr",
	"Dispatch-Const",
	"Map-level-1",
	"Map-Lv2-Cntl",
	"IBUF",
	"IBUF-Branch-Ofs",
	"Unused-14",
	"Unused-15",
	"Unused-16",
	"Unused-17",
	"uStack-Data",
	"uStack-Pop",
	"MD",
	"Unused-23",
	"Unused-24",
	"Unused-25",
	"Unused-26",
	"Unused-27",
	"Unused-30",
	"Unused-31",
	"Unused-32",
	"Unused-33",
	"Unused-34",
	"Unused-35",
	"Unused-36",
	"Unused-37",
	"C-PDL-Pointer",
	"C-PDL-Index",
	"Unused-42",
	"Unused-43",
	"C-PDL-Pointer-Pop",
	"C-PDL-Index-Dec",
	"Unused-46",
	"Unused-47",
	"PDL-Pointer",
	"PDL-Index",
	"Unused-52",
	"Unused-53",
	"PDL-Pointer-Pop",
	"PDL-Index-Dec",
	"Unused-56",
	"Unused-57",
	"Unused-60",
	"Unused-61",
	"Unused-62",
	"Unused-63",
	"Unused-64",
	"Unused-65",
	"Unused-66",
	"Unused-67",
	"Unused-70",
	"Unused-71",
	"Unused-72",
	"Unused-73",
	"Unused-74",
	"Unused-75",
	"Unused-76",
	"Unused-77"
};


const raven_disassembler::desc_info raven_disassembler::alu_operation[32] =
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
	{ "MUL", 0 },
	{ "MUL-Last", 0 },
	{ "DIV", 0 },
	{ "DIV-First", 0 },
	{ "DIV-Corr", 0 },
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
	{ "M+M", M_USED }
};


const char *const raven_disassembler::o_bus_control[8] =
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


const raven_disassembler::desc_info raven_disassembler::condition[16] =
{
	{ "R(0)", M_USED },              // 00: LSB of shifter output R(00) (normally used only for M sources)
	{ "LESS", A_USED | M_USED },     // 01: M source less than A source (ALU negative)
	{ "LESS-OR-EQ", A_USED | M_USED },// 02: M source less than or equal to A source
	{ "NOT-EQ", A_USED | M_USED },   // 03: Not (M source equals A source)
	{ "PAGE-FAULT", 0 },             // 04
	{ "PAGE-FAULT-OR-INT", 0 },      // 05
	{ "PAGE-FAULT-OR-INT-OR-SQB", 0 },// 06
	{ "ALWAYS", 0 },                 // 07
	{ "TYPE-NOT-EQ", A_USED | M_USED },// 08: Not (A-TYPE equals M-TYPE)
	{ "NOT-MEM-BUSY", 0 },           // 09
	{ "Q-ZERO", 0 },                 // 0a: Q(0)
	{ "NUBUS-ERROR", 0 },            // 0b
	{ "NOT-FIXNUM-OVERFLOW", 0 },    // 0c
	{ "BOXED-SIGN-BIT", 0 },         // 0d
	{ "NO-INTERRUPT", 0 },           // 0e
	{ "RESERVED-15", 0 },            // 0f
};


const char *const raven_disassembler::abj[8] =
{
	"A-NOP",
	"A-SKIP",
	"A-CALL-ILLOP",
	"A-CALL-TRAP",
	"A-CALL-BUSERR",
	"A-UNUSED",
	"A-RETURN",
	"A-RETURN-XCT-NEXT"
};


const char *const raven_disassembler::q_control[4] =
{
	"", // Q-NOP
	"Q-Shift-Left",
	"Q-Shift-Right",
	"Q-Load"
};


const char *const raven_disassembler::byte_operation[4] =
{
	"NOP",
	"Load-Byte",
	"Selective-Deposit",
	"Deposit-Byte"
};


const char *const raven_disassembler::jump_operation[8] =
{
	"Jump-XCT-Next",
	"JUMP",
	"Call-XCT-Next",
	"CALL",
	"Return-XCT-Next",
	"RETURN",
	"Jump-XCT-Next", // same as 000
	"JUMP"           // same as 001
};


// Dispatch address source field, IR(13:12) (Table 4-24). 1x is IBUF-derived in both cases.
const char *const raven_disassembler::dispatch_src[4] =
{
	"R-Bus",
	"MF-Bus",
	"IBUF",
	"IBUF"
};


const char *const raven_disassembler::dispatch_op[4] =
{
	"DISPATCH",
	"I-READ",
	"I-WRITE",
	"DISP-ERR"
};


u32 raven_disassembler::opcode_alignment() const
{
	return 1;
}


void raven_disassembler::destination(std::ostream &stream, u64 op)
{
	if (BIT(op, 31))
	{
		// A memory
		util::stream_format(stream, "(A-%03x) ", (op >> 19) & 0x3ff);
	}
	else
	{
		// A&M or O
		util::stream_format(stream, "(M-%02x%s) ", (op >> 19) & 0x3f, o_bus_destination[((op >> 25) & 0x3f)]);
	}
}


void raven_disassembler::sources(std::ostream &stream, u64 op)
{
	util::stream_format(stream, "<A-%03x,", (op >> 32) & 0x3ff);
	if (BIT(op, 48))
	{
		util::stream_format(stream, "%s> ", m_mf_source[(op >> 42) & 0x3f]);
	}
	else
	{
		util::stream_format(stream, "M-%02x> ", (op >> 42) & 0x3f);
	}
}


void raven_disassembler::conditional(std::ostream &stream, u64 op)
{
	const u8 cond = (op >> 10) & 0x0f;
	if (BIT(op, 14))
	{
		util::stream_format(stream, "IF %sTM-$%x<", (BIT(op, 15) ? "NOT " : ""), cond);
		if (BIT(op, 48))
		{
			util::stream_format(stream, "%s> ", m_mf_source[(op >> 42) & 0x3f]);
		}
		else
		{
			util::stream_format(stream, "M-%02x> ", (op >> 42) & 0x3f);
		}
	}
	else if (cond != 0x07)
	{
		util::stream_format(stream, "IF %s%s ", (BIT(op, 15) ? "NOT " : ""), condition[cond].name);
		if (condition[cond].source_used & A_USED)
		{
			util::stream_format(stream, "A-%03x", (op >> 32) & 0x3ff);
		}
		if (condition[cond].source_used & M_USED)
		{
			if (condition[cond].source_used & A_USED)
			{
				util::stream_format(stream, ",");
			}
			if (BIT(op, 48))
			{
				util::stream_format(stream, "%s", m_mf_source[(op >> 42) & 0x3f]);
			}
			else
			{
				util::stream_format(stream, "M-%02x", (op >> 42) & 0x3f);
			}
		}
	}
	else
	{
		if (BIT(op,15))
		{
			util::stream_format(stream, "NEVER ");
		}
	}
}


void raven_disassembler::disassemble_alu(std::ostream &stream, u64 op)
{
	// nevermore outputs
	// destination
	// typed-data
	// operation
	// carry-in
	// condition
	// q-control
	// output-bus
	// m-source
	// a-source
	// abj

	// Destination
	destination(stream, op);

	// Typed-data
	if (BIT(op, 8))
	{
		util::stream_format(stream, "TYPED-DATA ");
	}

	// Operation
	const u8 alu_op = (op >> 3) & 0x1f;
	util::stream_format(stream, "%s", alu_operation[alu_op].name);
	if (alu_operation[alu_op].source_used & A_USED)
	{
		util::stream_format(stream, " A-%03x", (op >> 32) & 0x3ff);
	}
	if (alu_operation[alu_op].source_used & M_USED)
	{
		if (alu_operation[alu_op].source_used & A_USED)
		{
			util::stream_format(stream, ",");
		}
		else
		{
			util::stream_format(stream, " ");
		}
		if (BIT(op, 48))
		{
			util::stream_format(stream, "%s", m_mf_source[(op >> 42) & 0x3f]);
		}
		else
		{
			util::stream_format(stream, "M-%02x", (op >> 42) & 0x3f);
		}
	}
	util::stream_format(stream, " ");

	// Carry-in?
	if (BIT(op, 2))
	{
		util::stream_format(stream, "CARRY-IN ");
	}

	// O-control
	util::stream_format(stream, "%s", o_bus_control[(op >> 16) & 0x7]);

	// T-Write?
	if (BIT(op, 9))
	{
		util::stream_format(stream, "TM-Wt-$%02x ", (op >> 10) & 0x0f);
	}

	conditional(stream, op);

	// ABJ
	if (op & u64(0x38000000000000))
	{
		util::stream_format(stream, "%s ", abj[(op >> 51) & 0x07]);
	}

	// Q control
	util::stream_format(stream, "%s", q_control[op & 0x03]);
}


void raven_disassembler::disassemble_byte(std::ostream &stream, u64 op)
{
	// Destination
	destination(stream, op);

	// Operation
	if (op & 0x10000)
	{
		util::stream_format(stream, "RIGHT ");
	}

	util::stream_format(stream, "%s (L=%d C=%d) ", byte_operation[(op >> 17) & 0x3], (op >> 5) & 0x1f, (u16) (op & 0x1f));

	// Sources
	sources(stream, op);

	conditional(stream, op);

	// ABJ
	if (op & u64(0x38000000000000))
	{
		util::stream_format(stream, "%s ", abj[(op >> 51) & 0x07]);
	}
}


void raven_disassembler::disassemble_jump(std::ostream &stream, u64 op)
{
	// Operation and new PC
	if (((op >> 5) & 0x07) != 0x04 && ((op >> 5) & 0x07) != 0x05)
	{
		util::stream_format(stream, "%s $%04x ", jump_operation[(op >> 5) & 0x7], (op >> 18) & 0x3fff);
	}
	else
	{
		util::stream_format(stream, "%s ", jump_operation[(op >> 5) & 0x7]);
	}

	// WCS Read or write
	if (op & 0x200)
	{
		util::stream_format(stream, "WCS-Write ");
	}
	if (op & 0x100)
	{
		util::stream_format(stream, "WCS-Read ");
	}

	conditional(stream, op);

	// ABJ
	if (op & u64(0x38000000000000))
	{
		util::stream_format(stream, "%s ", abj[(op >> 51) & 0x07]);
	}
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

	return 1 | dasmflags | SUPPORTED;
}
