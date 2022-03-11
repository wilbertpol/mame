// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/*****************************************************************************
 *
 *   z80lle.cpp
 *   A low-level Zilog Z80 emulator based on:
 *   - MAME's HLE Z80 cpu core
 *   - Programming the Z80 by Rodnay Zaks
 *   - Goran Devic's Z80 blog posts at baltazarstudios.com
 *
 *
 *   Any read or write performed by the Z80 can be delayed by asserting the WAIT
 *   signal. The instructions in this cpu core have been split up in parts to
 *   allow drivers to set and clear the WAIT signal as needed.
 * 
 *   When servicing interrupts in interrupt mode 2, the entire system must be
 *   designed to not respond to memory reads for the first instruction read from
 *   the interrupt "vector". In this core this is currently ignored, memory and
 *   wait cycles still occur as usual and the next instruction byte is simply
 *   taken from the stored m_irq_vector.
 * 
 *   TODO:
 *   - Split the RFSH and MREQ cycles of REFRESH
 *   - Test interrupts in IM0
 *   - Test interrupts in IM2
 *   - RETI: When should the daisy chain be notified?
 *   - check flags on INPUT_A
 *   - These instructions are largely untested:
 *     - 76 / dd/fd 76 - HALT (leaving halt state is also untested)
 *     - f9 / dd/fd f9 - LD SP,HL
 *     - ed 40 - IN B,(C)
 *     - ed 47 - LD I,A
 *     - ed 48 - IN C,(C)
 *     - ed 4d - RETI
 *     - ed 4f - LD R,A
 *     - ed 50 - IN D,(C)
 *     - ed 57 - LD A,I
 *     - ed 58 - IN E,(C)
 *     - ed 60 - IN H,(C)
 *     - ed 68 - IN L,(C)
 *     - ed 70 - IN F,(C)
 *     - ed 78 - IN A,(C)
 *     - ed a2 - INI
 *     - ed aa - IND
 *     - ed b2 - INIR
 *     - ed ba - INDR
 *
 *****************************************************************************/

#include "emu.h"
#include "debugger.h"
#include "z80lle.h"
#include "z80dasm.h"

#define VERBOSE             0

#define LOG(x)  do { if (VERBOSE) logerror x; } while (0)


bool z80lle_device::tables_initialised = false;

u8 z80lle_device::SZ[256];       // zero and sign flags
u8 z80lle_device::SZ_BIT[256];   // zero, sign and parity/overflow (=zero) flags for BIT opcode
u8 z80lle_device::SZP[256];      // zero, sign and parity flags
u8 z80lle_device::SZHV_inc[256]; // zero, sign, half carry and overflow flags INC r8
u8 z80lle_device::SZHV_dec[256]; // zero, sign, half carry and overflow flags DEC r8

u8 z80lle_device::SZHVC_add[2*256*256];
u8 z80lle_device::SZHVC_sub[2*256*256];


const u8 z80lle_device::jr_conditions[8][2] = {
	{ 0,  0  },  // always
	{ 0,  0  },  // always
	{ 0,  0  },  // always
	{ 0,  0  },  // always
	{ ZF, 0  },  // NZ
	{ ZF, ZF },  // Z
	{ CF, 0  },  // NC
	{ CF, CF }   // C
};


const u8 z80lle_device::jp_conditions[8][2] = {
	{ ZF, 0  },  // NZ
	{ ZF, ZF },  // Z
	{ CF, 0  },  // NC
	{ CF, CF },  // C
	{ PF, 0  },  // PO
	{ PF, PF },  // PE
	{ SF, 0  },  // P
	{ SF, SF }   // M
};


const u16 z80lle_device::insts[5 * 256 + 6][10] = {
	///////////////////////////////////////////////////////
	// Regular instructions                              //
	///////////////////////////////////////////////////////

	// 00, 4 cycles, NOP
	{ END },

	// 01, 10 cycles, LD BC,nn
	//  5 T1 AB:1235 DB:--
	//  6 T2 AB:1235 DB:XX MREQ RD
	//  7 T3 AB:1235 DB:XX MREQ RD
	//  8 T1 AB:1236 DB:--
	//  9 T2 AB:1236 DB:YY MREQ RD
	// 10 T3 AB:1236 DB:YY MREQ RD
	{ READ_S_PC, DB_R16L_READ_S_PC, DB_R16H | END },

	// 02, 7 cycles, LD (BC),A
	//  5 T1 AB:5678 DB:--
	//  6 T2 AB:5678 DB:AA MREQ
	//  7 T3 AB:5678 DB:AA MREQ WR
	{ WRITE_BCDE_A | END },

	// 03, 6 cycles, INC BC
	//  5 T5 AB:1234 DB:--
	//  6 T6 AB:1234 DB:--
	{ INC_R16 | END },

	/* 04, 4 cycles, INC B */ { INC_R8 | END },
	/* 05, 4 cycles, DEC B */ { DEC_R8 | END },

	// 06, 7 cycles, LD B,n
	//  5 T1 AB:1235 DB:--
	//  6 T2 AB:1235 DB:nn MREQ RD
	//  7 T3 AB:1235 DB:nn MREQ RD
	{ READ_S_PC, DB_REGD | END },

	// 07, 4 cycles, RLCA
	{ RLCA | END },
	// 08, 4 cycles, EX AF,AF'
	{ EX_AF_AF | END },

	// 09, 11 cycles, ADD HL,BC
	//  5 T1 AB:1234 DB:--
	//  6 T2 AB:1234 DB:--
	//  7 T3 AB:1234 DB:--
	//  8 T4 AB:1234 DB:--
	//  9 T1 AB:1234 DB:--
	// 10 T2 AB:1234 DB:--
	// 11 T3 AB:1234 DB:--
	{ ADD16 | END },

	// 0a, 7 cycles, LD A,(BC)
	//  5 T1 AB:5678 DB:--
	//  6 T2 AB:5678 DB:XX MREQ RD
	//  7 T3 AB:5678 DN:XX MREQ RD
	{ READ_S_BCDE_WZ_INC, DB_A | END },

	// 0b, 6 cycles, DEC BC
	//  5 T5 AB:1234 DB:--
	//  6 T6 AB:1234 DB:--
	{ DEC_R16 | END },

	// 0c, 4 cycles, INC C
	{ INC_R8 | END },
	// 0d, 4 cycles, DEC C
	{ DEC_R8 | END },

	// 0e, 7 cycles, LD C,n, see 06 for timing
	{ READ_S_PC, DB_REGD | END },

	// 0f, 4 cycles, RRCA
	{ RRCA | END },

	// 10, 8/13 cycles, DJNZ n
	//  5 T5 AB:1234 DB:--
	//  6 T1 AB:1235 DB:--
	//  7 T2 AB:1235 DB:nn MREQ RD
	//  8 T3 AB:1235 DB:nn MREQ RD
	//  9 T1 AB:1235 DB:-- *9-13 when jump taken
	// 10 T2 AB:1235 DB:--
	// 11 T3 AB:1235 DB:--
	// 12 T4 AB:1235 DB:--
	// 13 T5 AB:1235 DB:--
	{ X_READ_S_PC, DJNZ | END },

	// 11, 10 cycles, LD DE,nn, see 01 for timing
	{ READ_S_PC, DB_R16L_READ_S_PC, DB_R16H | END },
	// 12, 7 cycles, LD (DE),A, see 02 for timing
	{ WRITE_BCDE_A | END },
	// 13, 6 cycles, INC DE, see 03 for timing
	{ INC_R16 | END },
	// 14, 4 cycles, INC D
	{ INC_R8 | END },
	// 15, 4 cycles, DEC D
	{ DEC_R8 | END },
	// 16, 7 cycles, LD D,n, see 06 for timing
	{ READ_S_PC, DB_REGD | END },
	// 17, 4 cycles, RLA
	{ RLA | END },

	// 18, 12 cycles, JR n
	//  5 T1 AB:1235 DB:--
	//  6 T2 AB:1235 DB:nn MREQ RD
	//  7 T3 AB:1235 DB:nn MREQ RD
	//  8 T1 AB:1235 DB:--
	//  9 T2 AB:1235 DB:--
	// 10 T3 AB:1235 DB:--
	// 11 T4 AB:1235 DB:--
	// 12 T5 AB:1235 DB:--
	{ READ_S_PC, JR_COND | END },

	// 19, 11 cycles, ADD HL,DE
	{ ADD16 | END },
	// 1a, 7 cycles, LD A,(DE), see 0a for timing
	{ READ_S_BCDE_WZ_INC, DB_A | END },
	// 1b, 6 cycles, DEC DE, see 0b for timing
	{ DEC_R16 | END },
	// 1c, 4 cycles, INC E
	{ INC_R8 | END },
	// 1d, 4 cycles, DEC E
	{ DEC_R8 | END },
	// 1e, 7 cycles, LD E,n, see 06 for timing
	{ READ_S_PC, DB_REGD | END },
	// 1f, 4 cycles, RRA
	{ RRA | END },

	// 20, 7/12 cycles, JR NZ,n
	//  5 T1 AB:1235 DB:--
	//  6 T2 AB:1235 DB:nn MREQ RD
	//  7 T3 AB:1235 DB:nn MREQ RD
	//  8 T1 AB:1235 DB:-- *8-12 when jump taken
	//  9 T2 AB:1235 DB:--
	// 10 T3 AB:1235 DB:--
	// 11 T4 AB:1235 DB:--
	// 12 T5 AB:1235 DB:--
	{ READ_S_PC, JR_COND | END },
	// 21, 10 cycles, LD HL,nn, see 01 for timing
	{ READ_S_PC, DB_R16L_READ_S_PC, DB_R16H | END },

	// 22, 16 cycles, LD (nn),HL
	//  5 T1 AB:1235 DB:--
	//  6 T2 AB:1235 DB:78 MREQ RD
	//  7 T3 AB:1235 DB:78 MREQ RD
	//  8 T1 AB:1236 DB:--
	//  9 T2 AB:1236 DB:56 MREQ RD
	// 10 T3 AB:1236 DB:56 MREQ RD
	// 11 T1 AB:5678 DB:--
	// 12 T2 AB:5678 DB:ll MREQ
	// 13 T3 AB:5678 DB:ll MREQ WR
	// 14 T1 AB:5679 DB:--
	// 15 T2 AB:5679 DB:hh MREQ
	// 16 T3 AB:5679 DB:hh MREQ WR
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W_L_DB_WRITE_S_WZ_INC, H_DB_WRITE_S_WZ | END },

	// 23, 6 cycles, INC HL
	{ INC_R16 | END },
	// 24, 4 cycles, INC H
	{ INC_R8 | END },
	// 25, 4 cycles, DEC H
	{ DEC_R8 | END },
	// 26, 7 cycles, LD H,n, see 06 for timing
	{ READ_S_PC, DB_REGD | END },
	// 27, 4 cycles, DAA
	{ DAA | END },

	// 28, 7/12 cycles, JR Z,n, see 20 for timing
	{ READ_S_PC, JR_COND | END },
	// 29, 11 cycles, ADD HL,HL
	{ ADD16 | END },
	// 2a, 16 cycles, LD HL,(nn)
	//  5 T1 AB:1235 DB:--
	//  6 T2 AB:1235 DB:78 MREQ RD
	//  7 T3 AB:1235 DB:78 MREQ RD
	//  8 T1 AB:1236 DB:--
	//  9 T2 AB:1236 DB:56 MREQ RD
	// 10 T3 AB:1236 DB:56 MREQ RD
	// 11 T1 AB:5678 DB:--
	// 12 T2 AB:5678 DB:ll MREQ RD
	// 13 T3 AB:5678 DB:ll MREQ RD
	// 14 T1 AB:5679 DB:--
	// 15 T2 AB:5679 DB:hh MREQ RD
	// 16 T3 AB:5679 DB:hh MREQ RD
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W_READ_S_WZ_INC, DB_R16L_READ_S_WZ, DB_R16H | END },
	// 2b, 6 cycles, DEC HL
	{ DEC_R16 | END },
	// 2c, 4 cycles, INC L
	{ INC_R8 | END },
	// 2d, 4 cycles, DEC L
	{ DEC_R8 | END },
	// 2e, 7 cycles, LD L,n, see 06 for timing
	{ READ_S_PC, DB_REGD | END },
	// 2f, 4 cycles, CPL
	{ CPL | END },

	// 30, 7/12 cycles, JR NC,n, see 20 for timing
	{ READ_S_PC, JR_COND | END },
	// 31, 10 cycles, LD SP,nn, see 01 for timing
	{ READ_S_PC, DB_R16L_READ_S_PC, DB_R16H | END },
	// 32, 13 cycles, LD (nn),A
	//  5 T1 AB:1235 DB:--
	//  6 T2 AB:1235 DB:78 MREQ RD
	//  7 T3 AB:1235 DB:78 MREQ RD
	//  8 T1 AB:1236 DB:--
	//  9 T2 AB:1236 DB:56 MREQ RD
	// 10 T3 AB:1236 DB:56 MREQ RD
	// 11 T1 AB:5678 DB:--
	// 12 T2 AB:5678 DB:aa MREQ
	// 13 T3 AB:5678 DB:aa MREQ WR
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W_A_DB_WRITE_S_WZ_INC | END },
	// 33, 6 cycles, INC SP
	{ INC_R16 | END },
	// 34, 11 cycles, INC (HL)
	//  5 T1 AB:hhll DB:--
	//  6 T2 AB:hhll DB:xx MREQ RD
	//  7 T3 AB:hhll DB:xx MREQ RD
	//  8 T4 AB:hhll DB:--
	//  9 T1 AB:hhll DB:--
	// 10 T2 AB:hhll DB:yy MREQ
	// 11 T3 AB:hhll DB:yy MREQ WR
	{ READ_S_HL, INC_DB_WRITE_S | END },
	// 35, 11 cycles, DEC (HL)
	//  5 T1 AB:hhll DB:--
	//  6 T2 AB:hhll DB:xx MREQ RD
	//  7 T3 AB:hhll DB:xx MREQ RD
	//  8 T4 AB:hhll DB:--
	//  9 T1 AB:hhll DB:--
	// 10 T2 AB:hhll DB:yy MREQ
	// 11 T3 AB:hhll DB:yy MREQ WR
	{ READ_S_HL, DEC_DB_WRITE_S | END },
	// 36, 10 cycles, LD (HL),n
	//  5 T1 AB:1235 DB:--
	//  6 T2 AB:1235 DB:nn MREQ RD
	//  7 T3 AB:1235 DB:nn MREQ RD
	//  8 T1 AB:hhll DB:--
	//  9 T2 AB:hhll DB:nn MREQ
	// 10 T3 AB:hhll DB:nn MREQ WR
	{ READ_S_PC, WRITE_S_HL | END },
	// 37, 4 cycles, SCF
	{ SCF | END },

	// 38, 7/12 cycles, JR C,n, see 20 for timing
	{ READ_S_PC, JR_COND | END },
	// 39, 11 cycles, ADD HL,SP
	{ ADD16 | END },
	// 3a, 13 cycles, LD A,(nn)
	//  5 T1 AB:1235 DB:--
	//  6 T2 AB:1235 DB:78 MREQ RD
	//  7 T3 AB:1235 DB:78 MREQ RD
	//  8 T1 AB:1236 DB:--
	//  9 T2 AB:1236 DB:56 MREQ RD
	// 10 T3 AB:1236 DB:56 MREQ RD
	// 11 T1 AB:5678 DB:--
	// 12 T2 AB:5678 DB:xx MREQ RD
	// 13 T3 AB:5678 DB:xx MREQ RD
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W_READ_S_WZ_INC, DB_A | END },
	// 3b, 6 cycles, DEC SP
	{ DEC_R16 | END },
	// 3c, 4 cycles, INC A
	{ INC_R8 | END },
	// 3d, 4 cycles, DEC A
	{ DEC_R8 | END },
	// 3e, 7 cycles, LD A,n, see 06 for timing
	{ READ_S_PC, DB_REGD | END },
	// 3f, 4 cycles, CCF
	{ CCF | END },

	// 40, 4 cycles, LD B,B
	{ REGS_TMP_REG | END },
	// 41, 4 cycles, LD B,C
	{ REGS_TMP_REG | END },
	// 42, 4 cycles, LD B,D
	{ REGS_TMP_REG | END },
	// 43, 4 cycles, LD B,E
	{ REGS_TMP_REG | END },
	// 44, 4 cycles, LD B,H
	{ REGS_TMP_REG | END },
	// 45, 4 cycles, LD B,L
	{ REGS_TMP_REG | END },
	// 46, 7 cycles, LD B,(HL)
	// 5 T1 AB:hhll DB:--
	// 6 T2 AB:hhll DB:xx MREQ RD
	// 7 T3 AB:hhll DB:xx MREQ RD
	{ READ_S_HL, DB_REGD | END },
	// 47, 4 cycles, LD B,A
	{ REGS_TMP_REG | END },
	// 48, 4 cycles, LD C,B
	{ REGS_TMP_REG | END },
	// 49, 4 cycles, LD C,C
	{ REGS_TMP_REG | END },
	// 4a, 4 cycles, LD C,D
	{ REGS_TMP_REG | END },
	// 4b, 4 cycles, LD C,E
	{ REGS_TMP_REG | END },
	// 4c, 4 cycles, LD C,H
	{ REGS_TMP_REG | END },
	// 4d, 4 cycles, LD C,L
	{ REGS_TMP_REG | END },
	// 4e, 7 cycles, LD C,(HL)
	{ READ_S_HL, DB_REGD | END },
	// 4f, 4 cycles, LD C,A
	{ REGS_TMP_REG | END },

	// 50, 4 cycles, LD D,B
	{ REGS_TMP_REG | END },
	// 51, 4 cycles, LD D,C
	{ REGS_TMP_REG | END },
	// 52, 4 cycles, LD D,D
	{ REGS_TMP_REG | END },
	// 53, 4 cycles, LD D,E
	{ REGS_TMP_REG | END },
	// 54, 4 cycles, LD D,H
	{ REGS_TMP_REG | END },
	// 55, 4 cycles, LD D,L
	{ REGS_TMP_REG | END },
	// 56, 7 cycles, LD D,(HL)
	{ READ_S_HL, DB_REGD | END },
	// 57, 4 cycles, LD D,A
	{ REGS_TMP_REG | END },
	// 58, 4 cycles, LD E,B
	{ REGS_TMP_REG | END },
	// 59, 4 cycles, LD E,C
	{ REGS_TMP_REG | END },
	// 5a, 4 cycles, LD E,D
	{ REGS_TMP_REG | END },
	// 5b, 4 cycles, LD E,E
	{ REGS_TMP_REG | END },
	// 5c, 4 cycles, LD E,H
	{ REGS_TMP_REG | END },
	// 5d, 4 cycles, LD E,L
	{ REGS_TMP_REG | END },
	// 5e, 7 cycles, LD E,(HL)
	{ READ_S_HL, DB_REGD | END },
	// 5f, 4 cycles, LD E,A
	{ REGS_TMP_REG | END },

	// 60, 4 cycles, LD H,B
	{ REGS_TMP_REG | END },
	// 61, 4 cycles, LD H,C
	{ REGS_TMP_REG | END },
	// 62, 4 cycles, LD H,D
	{ REGS_TMP_REG | END },
	// 63, 4 cycles, LD H,E
	{ REGS_TMP_REG | END },
	// 64, 4 cycles, LD H,H
	{ REGS_TMP_REG | END },
	// 65, 4 cycles, LD H,L
	{ REGS_TMP_REG | END },
	// 66, 7 cycles, LD H,(HL)
	{ READ_S_HL, DB_REGD | END },
	// 67, 4 cycles, LD H,A
	{ REGS_TMP_REG | END },
	// 68, 4 cycles, LD L,B
	{ REGS_TMP_REG | END },
	// 69, 4 cycles, LD L,C
	{ REGS_TMP_REG | END },
	// 6a, 4 cycles, LD L,D
	{ REGS_TMP_REG | END },
	// 6b, 4 cycles, LD L,E
	{ REGS_TMP_REG | END },
	// 6c, 4 cycles, LD L,H
	{ REGS_TMP_REG | END },
	// 6d, 4 cycles, LD L,L
	{ REGS_TMP_REG | END },
	// 6e, 7 cycles, LD L,(HL)
	{ READ_S_HL, DB_REGD | END },
	// 6f, 4 cycles, LD L,A
	{ REGS_TMP_REG | END },

	// 70, 7 cycles, LD (HL),B
	// 5 T1 AB:hhll DB:--
	// 6 T2 AB:hhll DB:bb MREQ
	// 7 T3 AB:hhll DB:bb MREQ WR
	{ REGS0_WRITE_S_HL | END },
	// 71, 7 cycles, LD (HL),C
	{ REGS0_WRITE_S_HL | END },
	// 72, 7 cycles, LD (HL),D
	{ REGS0_WRITE_S_HL | END },
	// 73, 7 cycles, LD (HL),E
	{ REGS0_WRITE_S_HL | END },
	// 74, 7 cycles, LD (HL),H
	{ REGS0_WRITE_S_HL | END },
	// 75, 7 cycles, LD (HL),L
	{ REGS0_WRITE_S_HL | END },
	// 76, 4 cycles, HALT
	{ HALT | END },
	// 77, 7 cycles, LD (HL),A
	{ REGS0_WRITE_S_HL | END },
	// 78, 4 cycles, LD A,B
	{ REGS_TMP_REG | END },
	// 79, 4 cycles, LD A,C
	{ REGS_TMP_REG | END },
	// 7a, 4 cycles, LD A,D
	{ REGS_TMP_REG | END },
	// 7b, 4 cycles, LD A,E
	{ REGS_TMP_REG | END },
	// 7c, 4 cycles, LD A,H
	{ REGS_TMP_REG | END },
	// 7d, 4 cycles, LD A,L
	{ REGS_TMP_REG | END },
	// 7e, 7 cycles, LD A,(HL)
	{ READ_S_HL, DB_REGD | END },
	// 7f, 4 cycles, LD A,A 
	{ REGS_TMP_REG | END },

	// 80, 4 cycles, ADD B
	{ ADD_R8 | END },
	// 81, 4 cycles, ADD C
	{ ADD_R8 | END },
	// 82, 4 ccyles, ADD D
	{ ADD_R8 | END },
	// 83, 4 cycles, ADD E
	{ ADD_R8 | END },
	// 84, 4 cycles, ADD H
	{ ADD_R8 | END },
	// 85, 4 cycles, ADD L
	{ ADD_R8 | END },
	// 86, 7 cycles, ADD (HL)
	{ READ_S_HL, ADD_DB | END },
	// 87, 4 cycles, ADC A
	{ ADD_R8 | END },
	// 88, 4 cycles, ADC B
	{ ADC_R8 | END },
	// 89, 4 cycles, ADC C
	{ ADC_R8 | END },
	// 8a, 4 cycles, ADC D
	{ ADC_R8 | END },
	// 8b, 4 cycles, ADC E
	{ ADC_R8 | END },
	// 8c, 4 cycles, ADC H
	{ ADC_R8 | END },
	// 8d, 4 cycles, ADC L
	{ ADC_R8 | END },
	// 8e, 7 cycles, ADC (HL)
	{ READ_S_HL, ADC_DB | END },
	// 8f, 4 cycles, ADC A
	{ ADC_R8 | END },

	// 90, 4 cycles, SUB B
	{ SUB_R8 | END },
	// 91, 4 cycles, SUB C
	{ SUB_R8 | END },
	// 92, 4 cycles, SUB D
	{ SUB_R8 | END },
	// 93, 4 cycles, SUB E
	{ SUB_R8 | END },
	// 94, 4 cycles, SUB H
	{ SUB_R8 | END },
	// 95, 4 cycles, SUB L
	{ SUB_R8 | END },
	// 96, 7 cycles, SUB (HL)
	{ READ_S_HL, SUB_DB | END },
	// 97, 4 cycles, SUB A
	{ SUB_R8 | END },
	// 98, 4 cycles, SBC B
	{ SBC_R8 | END },
	// 99, 4 cycles, SBC C
	{ SBC_R8 | END },
	// 9a, 4 cycles, SBC D
	{ SBC_R8 | END },
	// 9b, 4 cycles, SBC E
	{ SBC_R8 | END },
	// 9c, 4 cycles, SBC H
	{ SBC_R8 | END },
	// 9d, 4 cycles, SBC L
	{ SBC_R8 | END },
	// 9e, 7 cycles, SBC (HL)
	{ READ_S_HL, SBC_DB | END },
	// 9f, 4 cycles, SBC A
	{ SBC_R8 | END },

	// a0, 4 cycles, AND B
	{ AND_R8 | END },
	// a1, 4 cycles, AND C
	{ AND_R8 | END },
	// a2, 4 cycles, AND D
	{ AND_R8 | END },
	// a3, 4 cycles, AND E
	{ AND_R8 | END },
	// a4, 4 cycles, AND H
	{ AND_R8 | END },
	// a5, 4 cycles, AND L
	{ AND_R8 | END },
	// a6, 7 cycles, AND (HL)
	{ READ_S_HL, AND_DB | END },
	// a7, 4 cycles, AND A
	{ AND_R8 | END },
	// a8, 4 cycles, XOR B
	{ XOR_R8 | END },
	// a9, 4 cycles, XOR C
	{ XOR_R8 | END },
	// aa, 4 cycles, XOR D
	{ XOR_R8 | END },
	// ab, 4 cycles, XOR E
	{ XOR_R8 | END },
	// ac, 4 cycles, XOR H
	{ XOR_R8 | END },
	// ad, 4 cycles, XOR L
	{ XOR_R8 | END },
	// ae, 7 cycles, XOR (HL)
	{ READ_S_HL, XOR_DB | END },
	// af, 4 cycles, XOR A
	{ XOR_R8 | END },

	// b0, 4 cycles, OR B
	{ OR_R8 | END },
	// b1, 4 cycles, OR C
	{ OR_R8 | END },
	// b2, 4 cycles, OR D
	{ OR_R8 | END },
	// b3, 4 cycles, OR E
	{ OR_R8 | END },
	// b4, 4 cycles, OR H
	{ OR_R8 | END },
	// b5, 4 cycles, OR L
	{ OR_R8 | END },
	// b6, 7 cycles, OR (HL)
	{ READ_S_HL, OR_DB | END },
	// b7, 4 cycles, OR A
	{ OR_R8 | END },
	// b8, 4 cycles, CP B
	{ CP_R8 | END },
	// b9, 4 cycles, CP C
	{ CP_R8 | END },
	// ba, 4 cycles, CP D
	{ CP_R8 | END },
	// bb, 4 cycles, CP E
	{ CP_R8 | END },
	// bc, 4 cycles, CP H
	{ CP_R8 | END },
	// bd, 4 cycles, CP L
	{ CP_R8 | END },
	// be, 7 cycles, CP (HL)
	{ READ_S_HL, CP_DB | END },
	// bf, 4 cycles, CP A
	{ CP_R8 | END },

	// c0, 5/11 cycles, RET NZ
	// cycles 6-11 only taken when condition is true
	//  5 T5 AB:1234 DB:--
	//  6 T1 AB:5678 DB:--
	//  7 T2 AB:5678 DB:xx MREQ RD
	//  8 T3 AB:5678 DB:xx MREQ RD
	//  9 T1 AB:5679 DB:--
	// 10 T2 AB:5679 DB:yy MREQ RD
	// 11 T3 AB:5679 DB:yy MREQ RD
	{ RET_COND, DB_Z_READ_S_SP_INC, DB_W_WZ_PC | END },
	// c1, 10 cycles, POP BC
	//  5 T1 AB:5678 DB:--
	//  6 T2 AB:5678 DB:xx MREQ RD
	//  7 T3 AB:5678 DB:xx MREQ RD
	//  8 T1 AB:5679 DB:--
	//  9 T2 AB:5679 DB:yy MREQ RD
	// 10 T3 AB:5679 DB:yy MREQ RD
	{ READ_S_SP_INC, DB_R16L_READ_S_SP_INC, DB_R16H | END },
	// c2, 10 cycles, JP NZ,nn
	//  5 T1 AB:1235 DB:--
	//  6 T2 AB:1235 DB:xx MREQ RD
	//  7 T3 AB:1235 DB:xx MREQ RD
	//  8 T1 AB:1236 DB:--
	//  9 T2 AB:1236 DB:yy MREQ RD
	// 10 T3 AB:1236 DB:yy MREQ RD
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W_JP_COND | END },
	// c3, 10 cycles, JMP nn
	//  5 T1 AB:1235 DB:--
	//  6 T2 AB:1235 DB:xx MREQ RD
	//  7 T3 AB:1235 DB:xx MREQ RD
	//  8 T1 AB:1236 DB:--
	//  9 T2 AB:1236 DB:yy MREQ RD
	// 10 T3 AB:1236 DB:yy MREQ RD
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W_WZ_PC | END },
	// c4, 10/17 cycles, CALL NZ,nn
	// cycles 12-17 only taken when condition is true
	//  5 T1 AB:1235 DB:--
	//  6 T2 AB:1235 DB:xx MREQ RD
	//  7 T3 AB:1235 DB:xx MREQ RD
	//  8 T1 AB:1236 DB:--
	//  9 T2 AB:1236 DB:yy MREQ RD
	// 10 T3 AB:1236 DB:yy MREQ RD
	// 11 T4 AB:1236 DB:--
	// 12 T1 AB:5678 DB:--
	// 13 T2 AB:5678 DB:yy MREQ
	// 14 T3 AB:5678 DB:yy MREQ WR
	// 15 T1 AB:5677 DB:--
	// 16 T2 AB:5677 DB:xx MREQ
	// 17 T3 AB:5677 DB:xx MREQ WR
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W_CALL_COND, PCL_WRITE_S_SP_DEC, WZ_PC | END },
	// c5, 11 cycles, PUSH BC
	//  5 T5 AB:1234 DB:--
	//  6 T1 AB:5677 DB:--
	//  7 T2 AB:5677 DB:cc MREQ
	//  8 T3 AB:5677 DB:cc MREQ WR
	//  9 T1 AB:5676 DB:--
	// 10 T2 AB:5676 DB:bb MREQ
	// 11 T3 AB:5676 DB:bb MREQ WR
	{ R16H_WRITE_S_SP_DEC, R16L_WRITE_S_SP_DEC | END },
	// c6, 7 cycles, ADD A,n
	//  5 T1 AB:1235 DB:--
	//  6 T2 AB:1235 DB:nn MREQ RD
	//  7 T3 AB:1235 DB:nn MREQ RD
	{ READ_S_PC, ADD_DB | END },
	// c7, 11 cycles, RST 0H
	//  5 T5 AB:1234 DB:--
	//  6 T1 AB:5677 DB:--
	//  7 T2 AB:5677 DB:cc MREQ
	//  8 T3 AB:5677 DB:cc MREQ WR
	//  9 T1 AB:5676 DB--
	// 10 T2 AB:5676 DB:pp MREQ
	// 11 T3 AB:5676 DB:pp MREQ WR
	{ PCH_WRITE_S_SP_DEC, PCL_WRITE_S_SP_DEC, RST | END },
	// c8, 5/11 cycles, RET Z, see c0 for timing
	{ RET_COND, DB_Z_READ_S_SP_INC, DB_W_WZ_PC | END },
	// c9, 10 cycles, RET
	//  5 T1 AB:5678 DB:--
	//  6 T2 AB:5678 DB:xx MREQ RD
	//  7 T3 AB:5678 DB:xx MREQ RD
	//  8 T1 AB:5679 DB:--
	//  9 T2 AB:5679 DB:yy MREQ RD
	// 10 T3 AB:5679 DB:yy MREQ RD
	{ READ_S_SP_INC, DB_Z_READ_S_SP_INC, DB_W_WZ_PC | END },
	// ca, 10 cycles, JP Z,nn, see c2 for timing
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W_JP_COND | END },
	// cb, +4 cycles, CB prefix
	{ 0 },
	// cc, 10/17 cycles, CALL Z,nn, see c4 for timing
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W_CALL_COND, PCL_WRITE_S_SP_DEC, WZ_PC | END },
	// cd, 17 cycles, CALL nn
	//  5 T1 AB:1235 DB:--
	//  6 T2 AB:1235 DB:yy MREQ RD
	//  7 T3 AB:1235 DB:yy MREQ RD
	//  8 T1 AB:1236 DB:--
	//  9 T2 AB:1236 DB:xx MREQ RD
	// 10 T3 AB:1236 DB:xx MREQ RD
	// 11 T4 AB:1236 DB:--
	// 12 T1 AB:5677 DB:--
	// 13 T2 AB:5677 DB:cc MREQ
	// 14 T3 AB:5677 DB:cc MREQ WR
	// 15 T1 AB:5676 DB:--
	// 16 T2 AB:5676 DB:pp MREQ
	// 17 T3 AB:5676 DB:pp MREQ WR
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W, PCH_WRITE_S_SP_DEC, PCL_WRITE_S_SP_DEC, WZ_PC | END },
	// ce, 7 cycles, ADC A,n, see c6 for timing
	{ READ_S_PC, ADC_DB | END },
	// cf, 11 cycles, RST 8H, see c7 for timing
	{ PCH_WRITE_S_SP_DEC, PCL_WRITE_S_SP_DEC, RST | END },

	// d0, 5/11 cycles, RET NC, see c0 for timing
	{ RET_COND, DB_Z_READ_S_SP_INC, DB_W_WZ_PC | END },
	// d1, 10 cycles, POP DE, see c1 for timing
	{  READ_S_SP_INC, DB_R16L_READ_S_SP_INC, DB_R16H | END },
	// d2, 10 cycles, JP NC,nn, see c2 for timing
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W_JP_COND | END },
	// d3, 11 cycles, OUT (n), A
	//  5 T1 AB:1235 DB:--
	//  6 T2 AB:1235 DB:nn MREQ RD
	//  7 T3 AB:1235 DB:nn MREQ RD
	//  8 T1 AB:1235 DB:--
	//  9 T2 AB:aann DB:aa         WR IORQ
	// 10 T3 AB:aann DB:aa         WR IORQ
	// 11 T4 AB:aann DB:aa         WR IORQ
	{ READ_S_PC, DB_Z, A_W, A_DB, OUTPUT_S_WZ_INC, DB_W | END },
	// d4, 10/17 cycles, CALL NC,nn, see c4 for timing
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W_CALL_COND, PCL_WRITE_S_SP_DEC, WZ_PC | END },
	// d5, 11 cycles, PUSH DE, see c5 for timing
	{ R16H_WRITE_S_SP_DEC, R16L_WRITE_S_SP_DEC | END },
	// d6, 7 cycles, SUB n, see c6 for timing
	{ READ_S_PC, SUB_DB | END },
	// d7, 11 cycles, RST 10H, see c7 for timing
	{ PCH_WRITE_S_SP_DEC, PCL_WRITE_S_SP_DEC, RST | END },
	// d8, 5/11 cycles, RET C, see c0 for timing
	{ RET_COND, DB_Z_READ_S_SP_INC, DB_W_WZ_PC | END },
	/* d9, 4 cycles, EXX */ { EXX | END },
	// da, 10 cycles, JP C,nn, see c2 for timing
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W_JP_COND | END },
	// db, 11 cycles, IN A,(n)
	//  5 T1 AB:1235 DB:--
	//  6 T2 AB:1235 DB:nn MREQ RD
	//  7 T3 AB:1235 DB:nn MREG RD
	//  8 T1 AB:1235 DB:--
	//  9 T2 AB:aann DB:xx      RD IORQ
	// 10 T3 AB:aann DB:xx      RD IORQ
	// 11 T4 AB:aann DB:xx      RD IORQ
	{ READ_S_PC, DB_Z, A_W, INPUT_S_WZ_INC, INPUT_A | END },
	// dc, 10/17 cycles, CALL C,nn, see c4 for timing
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W_CALL_COND, PCL_WRITE_S_SP_DEC, WZ_PC | END },
	// dd, +4 cycles, DD prefix
	{ 0 },
	// de, 7 cycles, SBC n, see c6 for timing
	{ READ_S_PC, SBC_DB | END },
	// df, 11 cycles, RST 18H, see c7 for timing
	{ PCH_WRITE_S_SP_DEC, PCL_WRITE_S_SP_DEC, RST | END },

	// e0, 5/11 cycles, RET PO, see c0 for timing
	{ RET_COND, DB_Z_READ_S_SP_INC, DB_W_WZ_PC | END },
	// e1, 10 cycles, POP HL, see c1 for timng
	{ READ_S_SP_INC, DB_R16L_READ_S_SP_INC, DB_R16H | END },
	// e2, 10 cycles, JP PO,nn, see c2 for timing
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W_JP_COND | END },
	// e3, 19 cycles, EX (SP),HL
	//  5 T1 AB:5678 DB:--
	//  6 T2 AB:5678 DB:xx MREQ RD
	//  7 T3 AB:5678 DB:xx MREQ RD
	//  8 T1 AB:5679 DB:--
	//  9 T2 AB:5679 DB:yy MREQ RD
	// 10 T3 AB:5679 DB:yy MREQ RD
	// 11 T4 AB:5679 DB:--
	// 12 T1 AB:5679 DB:--
	// 13 T2 AB:5679 DB:ll MREQ
	// 14 T3 AB:5679 DB:ll MREQ WR
	// 15 T1 AB:5678 DB:--
	// 16 T2 AB:5678 DB:hh MREQ
	// 17 T3 AB:5678 DB:hh MREQ WR
	// 18 T4 AB:5678 DB:--
	// 19 T5 AB:5678 DB:--
	{ READ_S_SP_INC, DB_Z_READ_S_SP, EX_SP_WRITE_S, R16L_WRITE_S_SP_DEC, X2, WZ_HL | END },
	// e4, 10/17 cycles, CALL PO,nn, see c4 for timing
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W_CALL_COND, PCL_WRITE_S_SP_DEC, WZ_PC | END },
	// e5, 11 cycles, PUSH HL, see c5 for timing
	{ R16H_WRITE_S_SP_DEC, R16L_WRITE_S_SP_DEC | END },
	// e6, 7 cycles, AND n, see c6 for timing
	{ READ_S_PC, AND_DB | END },
	// e7, 11 cycles, RST 20H, see c7 for timing
	{ PCH_WRITE_S_SP_DEC, PCL_WRITE_S_SP_DEC, RST | END },
	// e8, 5/11 cycles, RET PE, see c0 for timing
	{ RET_COND, DB_Z_READ_S_SP_INC, DB_W_WZ_PC | END },
	// e9, 4 cycles, JP (HL)
	{ HL_PC | END },
	// ea, 10 cycles, JP PE,nn, see c2 for timing
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W_JP_COND | END },
	// eb, 4 cycles, EX DE,HL
	{ EX_DE_HL | END },
	// ec, 10/17 cycles, CALL PE,nn, see c4 for timing
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W_CALL_COND, PCL_WRITE_S_SP_DEC, WZ_PC | END },
	// ed, +4 cycles, ED prefix
	{ 0 },
	// ee, 7 cycles, XOR n, see c6 for timing
	{ READ_S_PC, XOR_DB | END },
	// ef, 11 cycles, RST 28H, see c7 for timing
	{ PCH_WRITE_S_SP_DEC, PCL_WRITE_S_SP_DEC, RST | END },

	// f0, 5/11 cycles, RET P, see c0 for timing
	{ RET_COND, DB_Z_READ_S_SP_INC, DB_W_WZ_PC | END },
	// f1, 10 cycles, POP AF, see c1 for timing
	{  READ_S_SP_INC, DB_R16L_READ_S_SP_INC, DB_R16H | END },
	// f2, 10 cycles, JP P,nn, see c2 for timing
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W_JP_COND | END },
	// f3, 4 cycles, DI
	{ DI | END },
	// f4, 10/17 cycles, CALL P,nn, see c4 for timing
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W_CALL_COND, PCL_WRITE_S_SP_DEC, WZ_PC | END },
	// f5, 11 cycles, PUSH AF, see c5 for timing
	{ R16H_WRITE_S_SP_DEC, R16L_WRITE_S_SP_DEC | END },
	// f6, 7 cycles, OR n, see c6 for timing
	{ READ_S_PC, OR_DB | END },
	// f7, 11 cycles, RST 30H, see c7 for timing
	{ PCH_WRITE_S_SP_DEC, PCL_WRITE_S_SP_DEC, RST | END },
	// f8, 5/11 cycles, RET M, see c0 for timing
	{ RET_COND, DB_Z_READ_S_SP_INC, DB_W_WZ_PC | END },
	// f9, 6 cycles, LD SP,HL
	{ LD_SP_HL | END },
	// fa, 10 cycles, JP M,nn, see c2 for timing
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W_JP_COND | END },
	// fb, 4 cycles, EI
	{ EI | END },
	// fc, 10/17 cycles, CALL M,nn, see c4 for timing
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W_CALL_COND, PCL_WRITE_S_SP_DEC, WZ_PC | END }, 
	// fd, +4 cycles, FD prefix
	{ 0 },
	// fe, 7 cycles, CP n, see c6 for timing
	{ READ_S_PC, CP_DB | END },
	// ff, 11 cycles, RST 38H, see c7 for timing
	{ PCH_WRITE_S_SP_DEC, PCL_WRITE_S_SP_DEC, RST | END },

	///////////////////////////////////////////////////////
	// CB prefixed instructions                          //
	///////////////////////////////////////////////////////

	// cb 00, 8 cycles, RLC B
	{ RLC_R8 | END },
	// cb 01, 8 cycles, RLC C
	{ RLC_R8 | END },
	// cb 02, 8 cycles, RLC D
	{ RLC_R8 | END },
	// cb 03, 8 cycles, RLC E
	{ RLC_R8 | END },
	// cb 04, 8 cycles, RLC H
	{ RLC_R8 | END },
	// cb 05, 8 cycles, RLC L
	{ RLC_R8 | END },
	// cb 06, 15 cycles, RLC (HL)
	//  9 T1 AB:hhll DB:--
	// 10 T2 AB:hhll DB:xx MREQ RD
	// 11 T3 AB:hhll DB:xx MREQ RD
	// 12 T4 AB:hhll DB:--
	// 13 T1 AB:hhll DB:--
	// 14 T2 AB:hhll DB:yy MREQ
	// 15 T3 AB:hhll DB:yy MREQ WR
	{ READ_S_HL, RLC_WRITE_S | END },
	// cb 07, 8 cycles, RLC A
	{ RLC_R8 | END },
	// cb 08, 8 cycles, RRC B
	{ RRC_R8 | END },
	// cb 09, 8 cycles, RRC C
	{ RRC_R8 | END },
	// cb 0a, 8 cycles, RRC D
	{ RRC_R8 | END },
	// cb 0b, 8 cycles, RRC E
	{ RRC_R8 | END },
	// cb 0c, 8 cycles, RRC H
	{ RRC_R8 | END },
	// cb 0d, 8 cycles, RRC L
	{ RRC_R8 | END },
	// cb 0e, 15 cycles, RRC (HL), see cb 06 for timing
	{ READ_S_HL, RRC_WRITE_S | END },
	// cb 0f, 8 cycles, RRC A
	{ RRC_R8 | END },

	// cb 10, 8 cycles, RL B
	{ RL_R8 | END },
	// cb 11, 8 cycles, RL C
	{ RL_R8 | END },
	// cb 12, 8 cycles, RL D
	{ RL_R8 | END },
	// cb 13, 8 cycles, RL E
	{ RL_R8 | END },
	// cb 14, 8 cycles, RL H
	{ RL_R8 | END },
	// cb 15, 8 cycles, RL L
	{ RL_R8 | END },
	// cb 16, 15 cycles, RL (HL), see cb 06 for timing
	{ READ_S_HL, RL_WRITE_S | END },
	// cb 17, 8 cycles, RL A
	{ RL_R8 | END },
	// cb 18, 8 cycles, RR B
	{ RR_R8 | END },
	// cb 19, 8 cycles, RR C
	{ RR_R8 | END },
	// cb 1a, 8 cycles, RR D
	{ RR_R8 | END },
	// cb 1b, 8 cycles, RR E
	{ RR_R8 | END },
	// cb 1c, 8 cycles, RR H
	{ RR_R8 | END },
	// cb 1d, 8 cycles, RR L
	{ RR_R8 | END },
	// cb 1e, 15 cycles, RR (HL), see cb 06 for timing
	{ READ_S_HL, RR_WRITE_S | END },
	// cb 1f, 8 cycles, RR A
	{ RR_R8 | END },

	// cb 20, 8 cycles, SLA B
	{ SLA_R8 | END },
	// cb 21, 8 cycles, SLA C
	{ SLA_R8 | END },
	// cb 22, 8 cycles, SLA D
	{ SLA_R8 | END },
	// cb 23, 8 cycles, SLA E
	{ SLA_R8 | END },
	// cb 24, 8 cycles, SLA H
	{ SLA_R8 | END },
	// cb 25, 8 cycles, SLA L
	{ SLA_R8 | END },
	// cb 26, 15 cycles, SLA (HL), see cb 06 for timing
	{ READ_S_HL, SLA_WRITE_S | END },
	// cb 27, 8 cycles, SLA A
	{ SLA_R8 | END },
	// cb 28, 8 cycles, SRA B
	{ SRA_R8 | END },
	// cb 29, 8 cycles, SRA C
	{ SRA_R8 | END },
	// cb 2a, 8 cycles, SRA D
	{ SRA_R8 | END },
	// cb 2b, 8 cycles, SRA E
	{ SRA_R8 | END },
	// cb 2c, 8 cycles, SRA H
	{ SRA_R8 | END },
	// cb 2d, 8 cycles, SRA L
	{ SRA_R8 | END },
	// cb 2e, 15 cycles, SRA (HL), see cb 06 for timing
	{ READ_S_HL, SRA_WRITE_S | END },
	// cb 2f, 8 cycles, SRA A
	{ SRA_R8 | END },

	// cb 30, 8 cycles, SLL B
	{ SLL_R8 | END },
	// cb 31, 8 cycles, SLL C
	{ SLL_R8 | END },
	// cb 32, 8 cycles, SLL D
	{ SLL_R8 | END },
	// cb 33, 8 cycles, SLL E
	{ SLL_R8 | END },
	// cb 34, 8 cycles, SLL H
	{ SLL_R8 | END },
	// cb 35, 8 cycles, SLL L
	{ SLL_R8 | END },
	// cb 36, 15 cycles, SLL (HL), see cb 06 for timing
	{ READ_S_HL, SLL_WRITE_S | END },
	// cb 37, 8 cycles, SLL A
	{ SLL_R8 | END },
	// cb 38, 8 cycles, SRL B
	{ SRL_R8 | END },
	// cb 39, 8 cycles, SRL C
	{ SRL_R8 | END },
	// cb 3a, 8 cycles, SRL D
	{ SRL_R8 | END },
	// cb 3b, 8 cycles, SRL E
	{ SRL_R8 | END },
	// cb 3c, 8 cycles, SRL H
	{ SRL_R8 | END },
	// cb 3d, 8 cycles, SRL L
	{ SRL_R8 | END },
	// cb 3e, 15 cycles, SRL (HL), see cb 06 for timing
	{ READ_S_HL, SRL_WRITE_S | END },
	// cb 3f, 8 cycles, SRL A
	{ SRL_R8 | END },

	// cb 40, 8 cycles, BIT 0,B
	{ BIT_R8 | END },
	// cb 41, 8 cycles, BIT 0,C
	{ BIT_R8 | END },
	// cb 42, 8 cycles, BIT 0,D
	{ BIT_R8 | END },
	// cb 43, 8 cycles, BIT 0,E
	{ BIT_R8 | END },
	// cb 44, 8 cycles, BIT 0,H
	{ BIT_R8 | END },
	// cb 45, 8 cycles, BIT 0,L
	{ BIT_R8 | END },
	// cb 46, 12 cycles, BIT 0,(HL)
	//  9 T1 AB:hhll DB:--
	// 10 T2 AB:hhll DB:xx MREQ RD
	// 11 T3 AB:hhll DB:xx MREQ RD
	// 12 T4 AB:hhll DB:--
	{ READ_S_HL, BIT_DB | END },
	// cb 47, 8 cycles, BIT 0,A
	{ BIT_R8 | END },
	// cb 48, 8 cycles, BIT 1,B
	{ BIT_R8 | END },
	// cb 49, 8 cycles, BIT 1,C
	{ BIT_R8 | END },
	// cb 4a, 8 cycles, BIT 1,D
	{ BIT_R8 | END },
	// cb 4b, 8 cycles, BIT 1,E
	{ BIT_R8 | END },
	// cb 4c, 8 cycles, BIT 1,H
	{ BIT_R8 | END },
	// cb 4d, 8 cycles, BIT 1,L
	{ BIT_R8 | END },
	// cb 4e, 12 cycles, BIT 1,(HL), see cb 46 for timing
	{ READ_S_HL, BIT_DB | END },
	// cb 4f, 8 cycles, BIT 1,A
	{ BIT_R8 | END },

	// cb 50, 8 cycles, BIT 2,B
	{ BIT_R8 | END },
	// cb 51, 8 cycles, BIT 2,C
	{ BIT_R8 | END },
	// cb 52, 8 cycles, BIT 2,D
	{ BIT_R8 | END },
	// cb 53, 8 cycles, BIT 2,E
	{ BIT_R8 | END },
	// cb 54, 8 cycles, BIT 2,H
	{ BIT_R8 | END },
	// cb 55, 8 cycles, BIT 2,L
	{ BIT_R8 | END },
	// cb 56, 12 cycles, BIT 2,(HL), see cb 46 for timing
	{ READ_S_HL, BIT_DB | END },
	// cb 57, 8 cycles, BIT 2,A
	{ BIT_R8 | END },
	// cb 58, 8 cycles, BIT 3,B
	{ BIT_R8 | END },
	// cb 59, 8 cycles, BIT 3,C
	{ BIT_R8 | END },
	// cb 5a, 8 cycles, BIT 3,D
	{ BIT_R8 | END },
	// cb 5b, 8 cycles, BIT 3,E
	{ BIT_R8 | END },
	// cb 5c, 8 cycles, BIT 3,H
	{ BIT_R8 | END },
	// cb 5d, 8 cycles, BIT 3,L
	{ BIT_R8 | END },
	// cb 5e, 12 cycles, BIT 3,(HL), see cb 46 for timing
	{ READ_S_HL, BIT_DB | END },
	// cb 5f, 8 cycles, BIT 3,A
	{ BIT_R8 | END },

	// cb 60, 8 cycles, BIT 4,B
	{ BIT_R8 | END },
	// cb 61, 8 cycles, BIT 4,C
	{ BIT_R8 | END },
	// cb 62, 8 cycles, BIT 4,D
	{ BIT_R8 | END },
	// cb 63, 8 cycles, BIT 4,E
	{ BIT_R8 | END },
	// cb 64, 8 cycles, BIT 4,H
	{ BIT_R8 | END },
	// cb 65, 8 cycles, BIT 4,L
	{ BIT_R8 | END },
	// cb 66, 12 cycles, BIT 4,(HL), see cb 46 for timing
	{ READ_S_HL, BIT_DB | END },
	// cb 67, 8 cycles, BIT 4,A
	{ BIT_R8 | END },
	// cb 68, 8 cycles, BIT 5,B
	{ BIT_R8 | END },
	// cb 69, 8 cycles, BIT 5,C
	{ BIT_R8 | END },
	// cb 6a, 8 cycles, BIT 5,D
	{ BIT_R8 | END },
	// cb 6b, 8 cycles, BIT 5,E
	{ BIT_R8 | END },
	// cb 6c, 8 cycles, BIT 5,H
	{ BIT_R8 | END },
	// cb 6d, 8 cycles, BIT 5,L
	{ BIT_R8 | END },
	// cb 6e, 12 cycles, BIT 5,(HL), see cb 46 for timing
	{ READ_S_HL, BIT_DB | END },
	// cb 6f, 8 cycles, BIT 5,A
	{ BIT_R8 | END },

	// cb 70, 8 cycles, BIT 6,B
	{ BIT_R8 | END },
	// cb 71, 8 cycles, BIT 6,C
	{ BIT_R8 | END },
	// cb 72, 8 cycles, BIT 6,D
	{ BIT_R8 | END },
	// cb 73, 8 cycles, BIT 6,E
	{ BIT_R8 | END },
	// cb 74, 8 cycles, BIT 6,H
	{ BIT_R8 | END },
	// cb 75, 8 cycles, BIT 6,L
	{ BIT_R8 | END },
	// cb 76, 12 cycles, BIT 6,(HL), see cb 46 for timing
	{ READ_S_HL, BIT_DB | END },
	// cb 77, 8 cycles, BIT 6,A
	{ BIT_R8 | END },
	// cb 78, 8 cycles, BIT 7,B
	{ BIT_R8 | END },
	// cb 79, 8 cycles, BIT 7,C
	{ BIT_R8 | END },
	// cb 7a, 8 cycles, BIT 7,D
	{ BIT_R8 | END },
	// cb 7b, 8 cycles, BIT 7,E
	{ BIT_R8 | END },
	// cb 7c, 8 cycles, BIT 7,H
	{ BIT_R8 | END },
	// cb 7d, 8 cycles, BIT 7,L
	{ BIT_R8 | END },
	// cb 7e, 12 cycles, BIT 7,(HL), see cb 46 for timing
	{ READ_S_HL, BIT_DB | END },
	// cb 7f, 8 cycles, BIT 7,A
	{ BIT_R8 | END },

	// cb 80, 8 cycles, RES 0,B
	{ RES_R8 | END },
	// cb 81, 8 cycles, RES 0,C
	{ RES_R8 | END },
	// cb 82, 8 cycles, RES 0,D
	{ RES_R8 | END },
	// cb 83, 8 cycles, RES 0,E
	{ RES_R8 | END },
	// cb 84, 8 cycles, RES 0,H
	{ RES_R8 | END },
	// cb 85, 8 cycles, RES 0,L
	{ RES_R8 | END },
	// cb 86, 15 cycles, RES 0,(HL)
	//  9 T1 AB:hhll DB:--
	// 10 T2 AB:hhll DB:xx MREQ RD
	// 11 T3 AB:hhll DB:xx MREQ RD
	// 12 T4 AB:hhll DB:--
	// 13 T1 AB:hhll DB:--
	// 14 T2 AB:hhll DB:yy MREG
	// 15 T3 AB:hhll DB:yy MREQ WR
	{ READ_S_HL, RES_WRITE_S | END },
	// cb 87, 8 cycles, RES 0,A
	{ RES_R8 | END },
	// cb 88, 8 cycles, RES 1,B
	{ RES_R8 | END },
	// cb 89, 8 cycles, RES 1,C
	{ RES_R8 | END },
	// cb 8a, 8 cycles, RES 1,D
	{ RES_R8 | END },
	// cb 8b, 8 cycles, RES 1,E
	{ RES_R8 | END },
	// cb 8c, 8 cycles, RES 1,H
	{ RES_R8 | END },
	// cb 8d, 8 cycles, RES 1,L
	{ RES_R8 | END },
	// cb 8e, 15 cycles, RES 1,(HL), see cb 86 for timing
	{ READ_S_HL, RES_WRITE_S | END },
	// cb 8f, 8 cycles, RES 1,A
	{ RES_R8 | END },

	// cb 90, 8 cycles, RES 2,B
	{ RES_R8 | END },
	// cb 91, 8 cycles, RES 2,C
	{ RES_R8 | END },
	// cb 92, 8 cycles, RES 2,D
	{ RES_R8 | END },
	// cb 93, 8 cycles, RES 2,E
	{ RES_R8 | END },
	// cb 94, 8 cycles, RES 2,H
	{ RES_R8 | END },
	// cb 95, 8 cycles, RES 2,L
	{ RES_R8 | END },
	// cb 96, 15 cycles, RES 2,(HL), see cb 86 for timing
	{ READ_S_HL, RES_WRITE_S | END },
	// cb 97, 8 cycles, RES 2,A
	{ RES_R8 | END },
	// cb 98, 8 cycles, RES 3,B
	{ RES_R8 | END },
	// cb 99, 8 cycles, RES 3,C
	{ RES_R8 | END },
	// cb 9a, 8 cycles, RES 3,D
	{ RES_R8 | END },
	// cb 9b, 8 cycles, RES 3,E
	{ RES_R8 | END },
	// cb 9c, 8 cycles, RES 3,H
	{ RES_R8 | END },
	// cb 9d, 8 cycles, RES 3,L
	{ RES_R8 | END },
	// cb 9e, 15 cycles, RES 3,(HL), see cb 86 for timing
	{ READ_S_HL, RES_WRITE_S | END },
	// cb 9f, 8 cycles, RES 3,A
	{ RES_R8 | END },

	// cb a0, 8 cycles, RES 4,B
	{ RES_R8 | END },
	// cb a1, 8 cycles, RES 4,C
	{ RES_R8 | END },
	// cb a2, 8 cycles, RES 4,D
	{ RES_R8 | END },
	// cb a3, 8 cycles, RES 4,E
	{ RES_R8 | END },
	// cb a4, 8 cycles, RES 4,H
	{ RES_R8 | END },
	// cb a5, 8 cycles, RES 4,L
	{ RES_R8 | END },
	// cb a6, 15 cycles, RES 4,(HL), see cb 86 for timing
	{ READ_S_HL, RES_WRITE_S | END },
	// cb a7, 8 cycles, RES 4,A
	{ RES_R8 | END },
	// cb a8, 8 cycles, RES 5,B
	{ RES_R8 | END },
	// cb a9, 8 cycles, RES 5,C
	{ RES_R8 | END },
	// cb aa, 8 cycles, RES 5,D
	{ RES_R8 | END },
	// cb ab, 8 cycles, RES 5,E
	{ RES_R8 | END },
	// cb ac, 8 cycles, RES 5,H
	{ RES_R8 | END },
	// cb ad, 8 cycles, RES 5,L
	{ RES_R8 | END },
	// cb ae, 15 cycles, RES 5,(HL), for cb 86 for timing
	{ READ_S_HL, RES_WRITE_S | END },
	// cb af, 8 cycles, RES 5,A
	{ RES_R8 | END },

	// cb b0, 8 cycles, RES 6,B
	{ RES_R8 | END },
	// cb b1, 8 cycles, RES 6,C
	{ RES_R8 | END },
	// cb b2, 8 cycles, RES 6,D
	{ RES_R8 | END },
	// cb b3, 8 cycles, RES 6,E
	{ RES_R8 | END },
	// cb b4, 8 cycles, RES 6,H
	{ RES_R8 | END },
	// cb b5, 8 cycles, RES 6,L
	{ RES_R8 | END },
	// cb b6, 15 cycles, RES 6,(HL), see cb 86 for timing
	{ READ_S_HL, RES_WRITE_S | END },
	// cb b7, 8 cycles, RES 6,A
	{ RES_R8 | END },
	// cb b8, 8 cycles, RES 7,B
	{ RES_R8 | END },
	// cb b9, 8 cycles, RES 7,C
	{ RES_R8 | END },
	// cb ba, 8 cycles, RES 7,D
	{ RES_R8 | END },
	// cb bb, 8 cycles, RES 7,E
	{ RES_R8 | END },
	// cb bc, 8 cycles, RES 7,H
	{ RES_R8 | END },
	// cb bd, 8 cycles, RES 7,L
	{ RES_R8 | END },
	// cb be, 15 cycles, RES 7,(HL), see cb 86 for timing
	{ READ_S_HL, RES_WRITE_S | END },
	// cb bf, 8 cycles, RES 7,A
	{ RES_R8 | END },

	// cb c0, 8 cycles, SET 0,B
	{ SET_R8 | END },
	// cb c1, 8 cycles, SET 0,C
	{ SET_R8 | END },
	// cb c2, 8 cycles, SET 0,D
	{ SET_R8 | END },
	// cb c3, 8 cycles, SET 0,E
	{ SET_R8 | END },
	// cb c4, 8 cycles, SET 0,H
	{ SET_R8 | END },
	// cb c5, 8 cycles, SET 0,L
	{ SET_R8 | END },
	// cb c6, 15 cycles, SET 0,(HL), see cb 86 for timing
	{ READ_S_HL, SET_WRITE_S | END },
	// cb c7, 8 cycles, SET 0,A
	{ SET_R8 | END },
	// cb c8, 8 cycles, SET 1,B
	{ SET_R8 | END },
	// cb c9, 8 cycles, SET 1,C
	{ SET_R8 | END },
	// cb ca, 8 cycles, SET 1,D
	{ SET_R8 | END },
	// cb cb, 8 cycles, SET 1,E
	{ SET_R8 | END },
	// cb cc, 8 cycles, SET 1,H
	{ SET_R8 | END },
	// cb cd, 8 cycles, SET 1,L
	{ SET_R8 | END },
	// cb ce, 15 cycles, SET 1,(HL), see cb 86 for timing
	{ READ_S_HL, SET_WRITE_S | END },
	// cb cf, 8 cycles, SET 1,A
	{ SET_R8 | END },

	// cb d0, 8 cycles, SET 2,B
	{ SET_R8 | END },
	// cb d1, 8 cycles, SET 2,C
	{ SET_R8 | END },
	// cb d2, 8 cycles, SET 2,D
	{ SET_R8 | END },
	// cb d3, 8 cycles, SET 2,E
	{ SET_R8 | END },
	// cb d4, 8 cycles, SET 2,H
	{ SET_R8 | END },
	// cb d5, 8 cycles, SET 2,L
	{ SET_R8 | END },
	// cb d6, 15 cycles, SET 2,(HL), see cb 86 for timing
	{ READ_S_HL, SET_WRITE_S | END },
	// cb d7, 8 cycles, SET 2,A
	{ SET_R8 | END },
	// cb d8, 8 cycles, SET 3,B
	{ SET_R8 | END },
	// cb d9, 8 cycles, SET 3,C
	{ SET_R8 | END },
	// cb da, 8 cycles, SET 3,D
	{ SET_R8 | END },
	// cb db, 8 cycles, SET 3,E
	{ SET_R8 | END },
	// cb dc, 8 cycles, SET 3,H
	{ SET_R8 | END },
	// cb dd, 8 cycles, SET 3,L
	{ SET_R8 | END },
	// cb de, 15 cycles, SET 3,(HL), see cb 86 for timing
	{ READ_S_HL, SET_WRITE_S | END },
	// cb df, 8 cycles, SET 3,A
	{ SET_R8 | END },

	// cb e0, 8 cycles, SET 4,B
	{ SET_R8 | END },
	// cb e1, 8 cycles, SET 4,C
	{ SET_R8 | END },
	// cb e2, 8 cycles, SET 4,D
	{ SET_R8 | END },
	// cb e3, 8 cycles, SET 4,E
	{ SET_R8 | END },
	// cb e4, 8 cycles, SET 4,H
	{ SET_R8 | END },
	// cb e5, 8 cycles, SET 4,L
	{ SET_R8 | END },
	// cb e6, 15 cycles, SET 4,(HL), see cb 86 for timing
	{ READ_S_HL, SET_WRITE_S | END },
	// cb e7, 8 cycles, SET 4,A
	{ SET_R8 | END },
	// cb e8, 8 cycles, SET 5,B
	{ SET_R8 | END },
	// cb e9, 8 cycles, SET 5,C
	{ SET_R8 | END },
	// cb ea, 8 cycles, SET 5,D
	{ SET_R8 | END },
	// cb eb, 8 cycles, SET 5,E
	{ SET_R8 | END },
	// cb ec, 8 cycles, SET 5,H
	{ SET_R8 | END },
	// cb ed, 8 cycles, SET 5,L
	{ SET_R8 | END },
	// cb ee, 15 cycles, SET 5,(HL), see cb 86 for timing
	{ READ_S_HL, SET_WRITE_S | END },
	// cb ef, 8 cycles, SET 5,A
	{ SET_R8 | END },

	// cb f0, 8 cycles, SET 6,B
	{ SET_R8 | END },
	// cb f1, 8 cycles, SET 6,C
	{ SET_R8 | END },
	// cb f2, 8 cycles, SET 6,D
	{ SET_R8 | END },
	// cb f3, 8 cycles, SET 6,E
	{ SET_R8 | END },
	// cb f4, 8 cycles, SET 6,H
	{ SET_R8 | END },
	// cb f5, 8 cycles, SET 6,L
	{ SET_R8 | END },
	// cb f6, 15 cycles, SET 6,(HL), see cb 86 for timing
	{ READ_S_HL, SET_WRITE_S | END },
	// cb f7, 8 cycles, SET 6,A
	{ SET_R8 | END },
	// cb f8, 8 cycles, SET 7,B
	{ SET_R8 | END },
	// cb f9, 8 cycles, SET 7,C
	{ SET_R8 | END },
	// cb fa, 8 cycles, SET 7,D
	{ SET_R8 | END },
	// cb fb, 8 cycles, SET 7,E
	{ SET_R8 | END },
	// cb fc, 8 cycles, SET 7,H
	{ SET_R8 | END },
	// cb fd, 8 cycles, SET 7,L
	{ SET_R8 | END },
	// cb fe, 15 cycles, SET 7,(HL), see cb 86 for timing
	{ READ_S_HL, SET_WRITE_S | END },
	// cb ff, 8 cycles, SET 7,A
	{ SET_R8 | END },

	///////////////////////////////////////////////////////
	// ED-prefixed instructions                          //
	///////////////////////////////////////////////////////

	// ed 00-07, 8 cycles
	{ END }, { END }, { END }, { END }, { END }, { END }, { END }, { END },
	// ed 08-0f, 8 cycles
	{ END }, { END }, { END }, { END }, { END }, { END }, { END }, { END },

	// ed 10-17, 8 cycles
	{ END }, { END }, { END }, { END }, { END }, { END }, { END }, { END },
	// ed 18-1f, 8 cycles
	{ END }, { END }, { END }, { END }, { END }, { END }, { END }, { END },

	// ed 20-27, 8 cycles
	{ END }, { END }, { END }, { END }, { END }, { END }, { END }, { END },
	// ed 28-2f, 8 cycles
	{ END }, { END }, { END }, { END }, { END }, { END }, { END }, { END },

	// ed 30-37, 8, cycles
	{ END }, { END }, { END }, { END }, { END }, { END }, { END }, { END },
	// ed 38-3f, 8 cycles
	{ END }, { END }, { END }, { END }, { END }, { END }, { END }, { END },

	// ed 40, 12 cycles, IN B,(C)
	//  9 T1 AB:bbcc DB:--
	// 10 T2 AB:bbcc DB:xx RD IORQ
	// 11 T3 AB:bbcc DB:xx RD IORQ
	// 12 T4 AB:bbcc DB:xx RD IORQ
	{ INPUT_S_BC, INPUT_REGD | END },
	// ed 41, 12 cycles, OUT (C),B
	//  9 T1 AB:bbcc DB:--
	// 10 T2 AB:bbcc DB:xx WR IORQ
	// 11 T3 AB:bbcc DB:xx WR IORQ
	// 12 T4 AB:bbcc DB:xx WR IORQ
	{ REGD_DB, OUTPUT_S_BC | END },
	// ed 42, 15 cycles, SBC HL,BC
	//  9 T1 AB:1235 DB:--
	// 10 T2 AB:1235 DB:--
	// 11 T3 AB:1235 DB:--
	// 12 T4 AB:1235 DB:--
	// 13 T1 AB:1235 DB:--
	// 14 T2 AB:1235 DB:--
	// 15 T3 AB:1235 DB:--
	{ SBC16 | END },
	// ed 43, 20 cycles, LD (nn),BC
	//  9 T1 AB:1236 DB:--
	// 10 T2 AB:1236 DB:78 MREQ RD
	// 11 T3 AB:1236 DB:78 MREQ RD
	// 12 T1 AB:1237 DB:--
	// 13 T2 AB:1237 DB:56 MREQ RD
	// 14 T3 AB:1237 DB:56 MREQ RD
	// 15 T1 AB:5678 DB:--
	// 16 T2 AB:5678 DB:cc MREQ
	// 17 T3 AB:5678 DB:cc MREQ WR
	// 18 T1 AB:5679 DB:--
	// 19 T2 AB:5679 DB:bb MREQ
	// 20 T3 AB:5679 DB:bb MREQ WR
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W, R16L_WRITE_S_WZ_INC, R16H_WRITE_S_WZ | END },
	// ed 44, 8 cycles, NEG
	{ NEG | END },
	// ed 45, 14 cycles, RETN
	//  9 T1 AB:5678 DB:--
	// 10 T2 AB:5678 DB:yy MREQ RD
	// 11 T3 AB:5678 DB:yy MREQ RD
	// 12 T1 AB:5679 DB:--
	// 13 T2 AB:5679 DB:xx MREQ RD
	// 14 T3 AB:5679 DB:xx MREQ RD
	{ RETN, READ_S_SP_INC, DB_Z_READ_S_SP_INC, DB_W_WZ_PC | END },
	// ed 46, 8 cycles, IM 0
	{ IM | END },
	// ed 47, 9 cycles, LD I,A
	// 9 AB:1235 DB:--
	{ LD_I_A | END },
	// ed 48, 12 cycles, IN C,(C), see ed 40 for timing
	{ INPUT_S_BC, INPUT_REGD | END },
	// ed 49, 12 cycles, OUT (C),C, see ed 41 for timing
	{ REGD_DB, OUTPUT_S_BC | END },
	// ed 4a, 15 cycles, ADC HL,BC, see ed 42 for timing
	{ ADC16 | END },
	// ed 4b, 20 cycles, LD BC,(nn)
	//  9 T1 AB:1236 DB:--
	// 10 T2 AB:1236 DB:78 MREQ RD
	// 11 T3 AB:1236 DB:78 MREQ RD
	// 12 T1 AB:1237 DB:--
	// 13 T2 AB:1237 DB:56 MREQ RD
	// 14 T3 AB:1237 DB:56 MREQ RD
	// 15 T1 AB:5678 DB:--
	// 16 T2 AB:5678 DB:yy MREQ RD
	// 17 T3 AB:5678 DB:yy MREQ RD
	// 18 T1 AB:5679 DB:--
	// 19 T2 AB:5679 DB:xx MREQ RD
	// 20 T3 AB:5679 DB:xx MREQ RD
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W_READ_S_WZ_INC, DB_R16L_READ_S_WZ, DB_R16H | END },
	// ed 4c, 8 cycles, NEG
	{ NEG | END },
	// ed 4d, 14 cycles, RETI, sed ed 45 for timing
	{ RETI, READ_S_SP_INC, DB_Z_READ_S_SP_INC, DB_W_WZ_PC | END },
	// ed 4e, 8 cycles, IM 0
	{ IM | END },
	// ed 4f, 9 cycles, LD R,A
	{ LD_R_A | END },

	// ed 50, 12 cycles, IN D,(C), see ed 40 for timing
	{ INPUT_S_BC, INPUT_REGD | END },
	// ed 51, 12 cycles, OUT (C),D, see ed 41 for timing
	{ REGD_DB, OUTPUT_S_BC | END },
	// ed 52, 15 cycles SBC HL,DE, see ed 42 for timing
	{ SBC16 | END },
	// ed 53, 20 cycles, LD (nn),DE, see ed 43 for timing
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W, R16L_WRITE_S_WZ_INC, R16H_WRITE_S_WZ | END },
	// ed 54, 8 cycles, NEG
	{ NEG | END },
	// ed 55, 14 cycles, RETN, see ed 45 for timing
	{ RETN, READ_S_SP_INC, DB_Z_READ_S_SP_INC, DB_W_WZ_PC | END },
	// ed 56, 8 cycles, IM 1
	{ IM | END },
	// ed 57, 9 cycles, LD A,I
	{ LD_A_I | END },
	// ed 58, 12 cycles, IN E,(C), see ed 40 for timing
	{ INPUT_S_BC, INPUT_REGD | END },
	// ed 59, 12 cycles, OUT (C),E, see ed 41 for timing
	{ REGD_DB, OUTPUT_S_BC | END },
	// ed 5a, 15 cycles, ADC HL,DE, see ed 42 for timing
	{ ADC16 | END },
	// ed 5b, 20 cycles, LD DE,(nn), see ed 4b for timing
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W_READ_S_WZ_INC, DB_R16L_READ_S_WZ, DB_R16H | END },
	// ed 5c, 8 cycles, NEG
	{ NEG | END },
	// ed 5d, 14 cycles, RETN, see ed 42 for timing
	{ RETN, READ_S_SP_INC, DB_Z_READ_S_SP_INC, DB_W_WZ_PC | END },
	// ed 5e, 8 cycles, IM 2
	{ IM | END },
	// ed 5f, 9 cycles, LD A,R
	{ LD_A_R | END },

	// ed 60, 12 cycles, IN H,(C), see ed 40 for timing
	{ INPUT_S_BC, INPUT_REGD | END },
	// ed 61, 12 cycles, OUT (C),H, see ed 41 for timing
	{ REGD_DB, OUTPUT_S_BC | END },
	// ed 62, 15 cycles, SBC HL,HL, see ed 42 for timing
	{ SBC16 | END },
	// ed 63, 20 cycles, LD (nn),HL, see ed 43 for timing
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W, R16L_WRITE_S_WZ_INC, R16H_WRITE_S_WZ | END },
	// ed 64, 8 cycles, NEG
	{ NEG | END },
	// ed 65, 14 cycles, RETN, see ed 45 for timing
	{ RETN, READ_S_SP_INC, DB_Z_READ_S_SP_INC, DB_W_WZ_PC | END },
	// ed 66, 8 cycles, IM 0
	{ IM | END },
	// ed 67, 18 cycles, RRD
	//  9 T1 AB:hhll DB:--
	// 10 T2 AB:hhll DB:xx MREQ RD
	// 11 T3 AB:hhll DB:xx MREQ RD
	// 12 T1 AB:hhll DB:--
	// 13 T2 AB:hhll DB:--
	// 14 T3 AB:hhll DB:--
	// 15 T4 AB:hhll DB:--
	// 16 T1 AB:hhll DB:--
	// 17 T2 AB:hhll DB:yy MREQ
	// 18 T3 AB:hhll DB:yy MREQ WR
	{ HL_WZ, READ_S_WZ_INC, RRD, WRITE_S | END },
	// ed 68, 12 cycles, IN L,(C), see ed 40 for timing
	{ INPUT_S_BC, INPUT_REGD | END },
	// ed 69, 12 cycles, OUT (C),L, see ed 41 for timing
	{ REGD_DB, OUTPUT_S_BC | END },
	// ed 6a, 15 cycles, ADC HL,HL, see ed 42 for timing
	{ ADC16 | END },
	// ed 6b, 20 cycles, LD HL,(nn), see ed 4b for timing
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W_READ_S_WZ_INC, DB_R16L_READ_S_WZ, DB_R16H | END },
	// ed 6c, 8 cycles, NEG
	{ NEG | END },
	// ed 6d, 14 cycles, RETN, see ed 45 for timing
	{ RETN, READ_S_SP_INC, DB_Z_READ_S_SP_INC, DB_W_WZ_PC | END },
	// ed 6e, 8 cycles, IM 0
	{ IM | END },
	// ed 6f, 18 cycles, RLD, see ed 67 for timing
	{ HL_WZ, READ_S_WZ_INC, RLD, WRITE_S | END },

	// ed 70, 12 cycles, IN F,(C), see ed 40 for timing
	{ INPUT_S_BC, INPUT_REGD | END },
	// ed 71, 12 cycles, OUT (C),0, see ed 41 for timing
	{ ZERO_DB, OUTPUT_S_BC | END },
	// ed 72, 15 cycles, SBC HL,SP, see ed 42 for timing
	{ SBC16 | END },
	// ed 73, 20 cycles, LD (nn),SP, see ed 43 for timing
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W, R16L_WRITE_S_WZ_INC, R16H_WRITE_S_WZ | END },
	// ed 74, 8 cycles, NEG
	{ NEG | END },
	// ed 75, 14 cycles, RETN, see ed 45 for timing
	{ RETN, READ_S_SP_INC, DB_Z_READ_S_SP_INC, DB_W_WZ_PC | END },
	// ed 76, 8 cycles, IM 1
	{ IM | END },
	// ed 77, 8 cycles, illegal
	{ END },
	// ed 78, 12 cycles, IN A,(C), see ed 40 for timing
	{ INPUT_S_BC, INPUT_REGD | END },
	// ed 79, 12 cycles, OUT (C),A, see ed 41 for timing
	{ REGD_DB, OUTPUT_S_BC | END },
	// ed 7a, 15 cycles, ADC HL,SP, see ed 42 for timing
	{ ADC16 | END },
	// ed 7b, 20 cycles, LD SP,(nn), see ed 4b for timing
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W_READ_S_WZ_INC, DB_R16L_READ_S_WZ, DB_R16H | END },
	// ed 7c, 8 cycles, NEG
	{ NEG | END },
	// ed 7d, 14 cycles, RETN, see ed 45 for timing
	{ RETN, READ_S_SP_INC, DB_Z_READ_S_SP_INC, DB_W_WZ_PC | END },
	// ed 7e, 8 cycles, IM 2
	{ IM | END },
	// ed 7f, 8 cycles, illegal
	{ END },

	// ed 80-87
	{ END }, { END }, { END }, { END }, { END }, { END }, { END }, { END },
	// ed 88-8f
	{ END }, { END }, { END }, { END }, { END }, { END }, { END }, { END },

	// ed 90-97
	{ END }, { END }, { END }, { END }, { END }, { END }, { END }, { END },
	// ed 98-9f
	{ END }, { END }, { END }, { END }, { END }, { END }, { END }, { END },

	// ed a0, 16 cycles, LDI
	//  9 T1 AB:hhll DB:--
	// 10 T2 AB:hhll DB:xx MREQ RD
	// 11 T3 AB:hhll DB:xx MREQ RD
	// 12 T1 AB:ddee DB:--
	// 13 T2 AB:ddee DB:xx MREQ
	// 14 T3 AB:ddee DB:xx MREQ WR
	// 15 T4 AB:ddee DB:--
	// 16 T5 AB:ddee DB:--
	{ READ_S_HL, WRITE_S_DE, LDI | END },
	// ed a1, 16 cycles, CPI
	//  9 T1 AB:hhll DB:--
	// 10 T2 AB:hhll DB:xx MREQ RD
	// 11 T3 AB:hhll DB:xx MREQ RD
	// 12 T1 AB:hhll DB:--
	// 13 T2 AB:hhll DB:--
	// 14 T3 AB:hhll DB:--
	// 15 T4 AB:hhll DB:--
	// 16 T5 AB:hhll DB:--
	{ READ_S_HL, CPI | END },
	// ed a2, 16 cycles, INI
	//  9 T5 AB:1235 DB:--
	// 10 T1 AB:bbcc DB:--
	// 11 T2 AB:bbcc DB:xx RD IORQ
	// 12 T3 AB:bbcc DB:xx RD IORQ
	// 13 T4 AB:bbcc DB:xx RD IORQ
	// 14 T1 AB:hhll DB:--
	// 15 T2 AB:hhll DB:xx MREQ
	// 16 T3 AB:hhll DB:xx MREQ WR
	{ X, INPUT_S_BC, WRITE_S_HL, INI | END },
	// ed a3, 16 cycles, OUTI
	//  9 T5 AB:1235 DB:--
	// 10 T1 AB:hhll DB:--
	// 11 T2 AB:hhll DB:xx MREQ RD
	// 12 T3 AB:hhll DB:xx MREQ RD
	// 13 T1 AB:bbcc DB:--
	// 14 T2 AB:bbcc DB:xx WR IORQ
	// 15 T3 AB:bbcc DB:xx WR IORQ
	// 16 T4 AB:bbcc DB:xx WR IORQ
	{ X, READ_S_HL, OUTI, OUTPUT_S_BC | END },
	// ed a4-a7
	{ END }, { END }, { END }, { END },
	// ed a8, 16 cycles, LDD
	//  9 T1 AB:hhll DB:--
	// 10 T2 AB:hhll DB:xx MREQ RD
	// 11 T3 AB:hhll DB:xx MREQ RD
	// 12 T1 AB:ddee DB:--
	// 13 T2 AB:ddee DB:xx MREQ
	// 14 T3 AB:ddee DB:xx MREQ WR
	// 15 T4 AB:ddee DB:--
	// 16 T5 AB:ddee DB:--
	{ READ_S_HL, WRITE_S_DE, LDD | END },
	// ed a9, 16 cycles, CPD
	//  9 T1 AB:hhll DB:--
	// 10 T2 AB:hhll DB:xx MREQ RD
	// 11 T3 AB:hhll DB:xx MREQ RD
	// 12 T1 AB:hhll DB:--
	// 13 T2 AB:hhll DB:--
	// 14 T3 AB:hhll DB:--
	// 15 T4 AB:hhll DB:--
	// 16 T5 AB:hhll DB:--
	{ READ_S_HL, CPD | END },
	// ed aa, 16 cycles, IND
	//  9 T5 AB:1235 DB:--
	// 10 T1 AB:bbcc DB:--
	// 11 T2 AB:bbcc DB:xx RD IORQ
	// 12 T3 AB:bbcc DB:xx RD IORQ
	// 13 T4 AB:bbcc DB:xx RD IORQ
	// 14 T1 AB:hhll DB:--
	// 15 T2 AB:hhll DB:xx MREQ
	// 16 T3 AB:hhll DB:xx MREQ WR
	{ X, INPUT_S_BC, WRITE_S_HL, IND | END },
	// ed ab, 16 cycles, OUTD
	//  9 T5 AB:1235 DB:--
	// 10 T1 AB:hhll DB:--
	// 11 T2 AB:hhll DB:xx MREQ RD
	// 12 T3 AB:hhll DB:xx MREQ RD
	// 13 T1 AB:bbcc DB:--
	// 14 T2 AB:bbcc DB:xx WR IORQ
	// 15 T3 AB:bbcc DB:xx WR IORQ
	// 16 T4 AB:bbcc DB:xx WR IORQ
	{ X, READ_S_HL, OUTPUT_S_BC | END },
	// ed ac-af
	{ END }, { END }, { END }, { END },

	// ed b0, 16/21 cycles, LDIR
	// cycles 17-21 when BC != 0
	//  9 T1 AB:hhll DB:--
	// 10 T2 AB:hhll DB:xx MREQ RD
	// 11 T3 AB:hhll DB:xx MREQ RD
	// 12 T1 AB:ddee DB:--
	// 13 T2 AB:ddee DB:xx MREQ
	// 14 T3 AB:ddee DB:xx MREQ WR
	// 15 T4 AB:ddee DB:--
	// 16 T5 AB:ddee DB:--
	// 17 T1 AB:ddee DB:--
	// 18 T2 AB:ddee DB:--
	// 19 T3 AB:ddee DB:--
	// 20 T4 AB:ddee DB:--
	// 21 T5 AB:ddee DB:--
	{ READ_S_HL, WRITE_S_DE, LDI, REPEAT | END },
	// ed b1, 16/21 cycles, CPIR
	// cycles 17-21 when BC != 0
	//  9 T1 AB:hhll DB:--
	// 10 T2 AB:hhll DB:xx MREQ RD
	// 11 T3 AB:hhll DB:xx MREQ RD
	// 12 T1 AB:hhll DB:--
	// 13 T2 AB:hhll DB:--
	// 14 T3 AB:hhll DB:--
	// 15 T4 AB:hhll DB:--
	// 16 T5 AB:hhll DB:--
	// 17 T1 AB:hhll DB:--
	// 18 T2 AB:hhll DB:--
	// 19 T3 AB:hhll DB:--
	// 20 T4 AB:hhll DB:--
	// 21 T5 AB:hhll DB:--
	{ READ_S_HL, CPI, REPEATCP | END },
	// ed b2, 16/21 cycles, INIR
	// cycles 17-21 when BC != 0
	//  9 T5 AB:1235 DB:--
	// 10 T1 AB:bbcc DB:--
	// 11 T2 AB:bbcc DB:xx RD IORQ
	// 12 T3 AB:bbcc DB:xx RD IORQ
	// 13 T4 AB:bbcc DB:xx RD IORQ
	// 14 T1 AB:hhll DB:--
	// 15 T2 AB:hhll DB:xx MREQ
	// 16 T3 AB:hhll DB:xx MREQ WR
	// 17 T1 AB:hhll DB:--
	// 18 T2 AB:hhll DB:--
	// 19 T3 AB:hhll DB:--
	// 20 T4 AB:hhll DB:--
	// 21 T5 AB:hhll DB:--
	{ X, INPUT_S_BC, WRITE_S_HL, INI, REPEATIO | END },
	// ed b3, 16/21 cycles, OTIR
	// cycles 17-21 when BC != 0
	//  9 T5 AB:1235 DB:--
	// 10 T1 AB:hhll DB:--
	// 11 T2 AB:hhll DB:xx MREQ RD
	// 12 T3 AB:hhll DB:xx MREQ RD
	// 13 T1 AB:bbcc DB:--
	// 14 T2 AB:bbcc DB:xx WR IORQ
	// 15 T3 AB:bbcc DB:xx WR IORQ
	// 16 T4 AB:bbcc DB:xx WR IORQ
	// 17 T1 AB:bbcc DB:--
	// 18 T2 AB:bbcc DB:--
	// 19 T3 AB:bbcc DB:--
	// 20 T4 AB:bbcc DB:--
	// 21 T5 AB:bbcc DB:--
	{ X, READ_S_HL, OUTI, OUTPUT_S_BC, REPEATIO | END },
	// ed b4-b7
	{ END }, { END }, { END }, { END },
	// ed b8, 16/21 cycles, LDDR
	// cycles 17-21 when BC != 0
	//  9 T1 AB:hhll DB:--
	// 10 T2 AB:hhll DB:xx MREQ RD
	// 11 T3 AB:hhll DB:xx MREQ RD
	// 12 T1 AB:ddee DB:--
	// 13 T2 AB:ddee DB:xx MREQ
	// 14 T3 AB:ddee DB:xx MREQ WR
	// 15 T4 AB:ddee DB:--
	// 16 T5 AB:ddee DB:--
	// 17 T1 AB:ddee DB:--
	// 18 T2 AB:ddee DB:--
	// 19 T3 AB:ddee DB:--
	// 20 T4 AB:ddee DB:--
	// 21 T5 AB:ddee DB:--
	{ READ_S_HL, WRITE_S_DE, LDD, REPEAT | END },
	// ed b9, 16/21 cycles, CPDR
	// cycles 17-21 when BC != 0
	//  9 T1 AB:hhll DB:--
	// 10 T2 AB:hhll DB:xx MREQ RD
	// 11 T3 AB:hhll DB:xx MREQ RD
	// 12 T1 AB:hhll DB:--
	// 13 T2 AB:hhll DB:--
	// 14 T3 AB:hhll DB:--
	// 15 T4 AB:hhll DB:--
	// 16 T5 AB:hhll DB:--
	// 17 T1 AB:hhll DB:--
	// 18 T2 AB:hhll DB:--
	// 19 T3 AB:hhll DB:--
	// 20 T4 AB:hhll DB:--
	// 21 T5 AB:hhll DB:--
	{ READ_S_HL, CPD, REPEATCP | END },
	// ed ba, 16/21 cycles, INDR
	// cycles 17-21 when BC != 0
	//  9 T5 AB:1235 DB:--
	// 10 T1 AB:bbcc DB:--
	// 11 T2 AB:bbcc DB:xx RD IORQ
	// 12 T3 AB:bbcc DB:xx RD IORQ
	// 13 T4 AB:bbcc DB:xx RD IORQ
	// 14 T1 AB:hhll DB:--
	// 15 T2 AB:hhll DB:xx MREQ
	// 16 T3 AB:hhll DB:xx MREQ WR
	// 17 T1 AB:hhll DB:--
	// 18 T2 AB:hhll DB:--
	// 19 T3 AB:hhll DB:--
	// 20 T4 AB:hhll DB:--
	// 21 T5 AB:hhll DB:--
	{ X, INPUT_S_BC, WRITE_S_HL, IND, REPEATIO | END },
	// ed bb, 16/21 cycles, OTDR
	// cycles 17-21 when BC != 0
	//  9 T5 AB:1235 DB:--
	// 10 T1 AB:hhll DB:--
	// 11 T2 AB:hhll DB:xx MREQ RD
	// 12 T3 AB:hhll DB:xx MREQ RD
	// 13 T1 AB:bbcc DB:--
	// 14 T2 AB:bbcc DB:xx WR IORQ
	// 15 T3 AB:bbcc DB:xx WR IORQ
	// 16 T4 AB:bbcc DB:xx WR IORQ
	// 17 T1 AB:bbcc DB:--
	// 18 T2 AB:bbcc DB:--
	// 19 T3 AB:bbcc DB:--
	// 20 T4 AB:bbcc DB:--
	// 21 T5 AB:bbcc DB:--
	{ X, READ_S_HL, OUTPUT_S_BC, REPEATIO | END },
	// ed bc-bf
	{ END }, { END }, { END }, { END },

	// ed c0-c7
	{ END }, { END }, { END }, { END }, { END }, { END }, { END }, { END },
	// ed c8-cf
	{ END }, { END }, { END }, { END }, { END }, { END }, { END }, { END },

	// ed d0-d7
	{ END }, { END }, { END }, { END }, { END }, { END }, { END }, { END },
	// ed d8-df
	{ END }, { END }, { END }, { END }, { END }, { END }, { END }, { END },

	// ed e0-e7
	{ END }, { END }, { END }, { END }, { END }, { END }, { END }, { END },
	// ed e8-ef
	{ END }, { END }, { END }, { END }, { END }, { END }, { END }, { END },

	// ed f0-f7
	{ END }, { END }, { END }, { END }, { END }, { END }, { END }, { END },
	// ed f8-ff
	{ END }, { END }, { END }, { END }, { END }, { END }, { END }, { END },

	///////////////////////////////////////////////////////
	// DD/FD prefixed instructions                       //
	// Almost equal to regular instructions              //
	///////////////////////////////////////////////////////

	// dd/fd 00, 8 cycles, NOP
	{ END },
	// dd/fd 01, 14 cycles, LD BC,nn
	{ READ_S_PC, DB_R16L_READ_S_PC, DB_R16H | END },
	// dd/fd 02, 11 cycles, LD (BC),A
	{ WRITE_BCDE_A | END },
	// dd/fd 03, 10 cycles, INC BC
	{ INC_R16 | END },
	// dd/fd 04, 8 cycles, INC B
	{ INC_R8 | END },
	// dd/fd 05, 8 cycles, DEC B
	{ DEC_R8 | END },
	// dd/fd 06, 11 cycles, LD B,n
	{ READ_S_PC, DB_REGD | END },
	// dd/fd 07, 8 cycles, RLCA
	{ RLCA | END },
	// dd/fd 08, 8 cycles, EX AF,AF'
	{ EX_AF_AF | END },
	// dd/fd 09, 15 cycles, ADD IX/IY,BC
	{ ADD16 | END },
	// dd/fd 0a, 11 cycles, LD A,(BC)
	{ READ_S_BCDE_WZ_INC, DB_A | END },
	// dd/fd 0b, 10 cycles, DEC BC
	{ DEC_R16 | END }, 
	// dd/fd 0c, 8 cycles, INC C
	{ INC_R8 | END },
	// dd/fd 0d, 8 cycles, DEC C
	{ DEC_R8 | END },
	// dd/fd 0e, 11 cycles, LD C,n
	{ READ_S_PC, DB_REGD | END },
	// dd/fd 0f, 8 cycles, RRCA
	{ RRCA | END },

	// dd/fd 10, 12/17 cycles, DJNZ n
	{ X_READ_S_PC, DJNZ | END },
	// dd/fd 11, 14 cycles, LD DE,nn
	{ READ_S_PC, DB_R16L_READ_S_PC, DB_R16H | END },
	// dd/fd 12, 7 cycles, LD (DE),A
	{ WRITE_BCDE_A | END },
	// dd/fd 13, 10 cycles, INC DE
	{ INC_R16 | END },
	// dd/fd 14, 8 cycles, INC D
	{ INC_R8 | END },
	// dd/fd 15, 8 cycles, DEC D
	{ DEC_R8 | END },
	// dd/fd 16, 11 cycles, LD D,n
	{ READ_S_PC, DB_REGD | END },
	// dd/fd 17, 8 cycles, RLA
	{ RLA | END },
	// dd/fd 18, 16 cycles, JR n
	{ READ_S_PC, JR_COND | END },
	// dd/fd 19, 11 cycles, ADD IX/IY,DE
	{ ADD16 | END },
	// dd/fd 1a, 11 cycles, LD A,(DE)
	{ READ_S_BCDE_WZ_INC, DB_A | END },
	// dd/fd 1b, 10 cycles, DEC DE
	{ DEC_R16 | END },
	// dd/fd 1c, 8 cycles, INC E
	{ INC_R8 | END },
	// dd/fd 1d, 8 cycles, DEC E
	{ DEC_R8 | END },
	// dd/fd 1e, 11 cycles, LD E,n
	{ READ_S_PC, DB_REGD | END },
	// dd/fd 1f, 8 cycles, RRA
	{ RRA | END },

	// dd/fd 20, 11/16 cycles, JR NZ,n
	{ READ_S_PC, JR_COND | END },
	// dd/fd 21, 14 cycles, LD IX/IY,nn
	{ READ_S_PC, DB_R16L_READ_S_PC, DB_R16H | END },
	// dd/fd 22, 20 cycles, LD (nn),IX/IY
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W_L_DB_WRITE_S_WZ_INC, H_DB_WRITE_S_WZ | END },
	// dd/fd 23, 10 cycles, INC IX/IY
	{ INC_R16 | END },
	// dd/fd 24, 8 cycles, INC IXh/IYh
	{ INC_R8 | END },
	// dd/fd 25, 8 cycles, DEC IXh/IYh
	{ DEC_R8 | END },
	// dd/fd 26, 11 cycles, LD IXh/IYh,n
	{ READ_S_PC, DB_REGD | END },
	// dd/fd 27, 8 cycles, DAA
	{ DAA | END },
	// dd/fd 28, 11/16 cycles, JR Z,n
	{ READ_S_PC, JR_COND | END },
	// dd/fd 29, 15 cycles, ADD IX/IY,IX/IY
	{ ADD16 | END },
	// dd/fd 2a, 20 cycles, LD IX/IY,(nn)
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W_READ_S_WZ_INC, DB_R16L_READ_S_WZ, DB_R16H | END },
	// dd/fd 2b, 10 cycles, DEC IX/IY
	{ DEC_R16 | END },
	// dd/fd 2c, 8 cycles, INC IXl/IYl
	{ INC_R8 | END },
	// dd/fd 2d, 8 cycles, DEC IXl/IYl
	{ DEC_R8 | END },
	// dd/fd 2e, 11 cycles, LD IXl/IYl,n
	{ READ_S_PC, DB_REGD | END },
	// dd/fd 2f, 8 cycles, CPL
	{ CPL | END },

	// dd/fd 30, 11/16 cycles, JR NC,n
	{ READ_S_PC, JR_COND | END },
	// dd/fd 31, 14 cycles, LD SP,nn
	{ READ_S_PC, DB_R16L_READ_S_PC, DB_R16H | END },
	// dd/fd 32, 17 cycles, LD (nn),A
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W_A_DB_WRITE_S_WZ_INC | END },
	// dd/fd 33, 10 cycles, INC SP
	{ INC_R16 | END },
	// dd/fd 34, 23 cycles, INC (IX/IY+dd)
	//  9 T1 AB:1236 DB:--
	// 10 T2 AB:1236 DB:dd MREQ RD
	// 11 T3 AB:1236 DB:dd MREQ RD
	// 12 T1 AB:1236 DB:--
	// 13 T2 AB:1236 DB:--
	// 14 T3 AB:1236 DB:--
	// 15 T4 AB:1236 DB:--
	// 16 T5 AB:1236 DB:--
	// 17 T1 AB:5678 DB:--
	// 18 T2 AB:5678 DB:xx MREQ RD
	// 19 T3 AB:5678 DB:xx MREQ RD
	// 20 T4 AB:5678 DB:--
	// 21 T1 AB:5678 DB:--
	// 22 T2 AB:5678 DB:xx MREQ
	// 23 T3 AB:5678 DB:xx MREQ WR
	{ READ_S_PC, DISP_WZ5_READ_S_WZ, INC_DB_WRITE_S | END },
	// dd/fd 35, 23 cycles, DEC (IX/IY+dd)
	//  9 T1 AB:1236 DB:--
	// 10 T2 AB:1236 DB:dd MREQ RD
	// 11 T3 AB:1236 DB:dd MREQ RD
	// 12 T1 AB:1236 DB:--
	// 13 T2 AB:1236 DB:--
	// 14 T3 AB:1236 DB:--
	// 15 T4 AB:1236 DB:--
	// 16 T5 AB:1236 DB:--
	// 17 T1 AB:5678 DB:--
	// 18 T2 AB:5678 DB:xx MREQ RD
	// 19 T3 AB:5678 DB:xx MREQ RD
	// 20 T4 AB:5678 DB:--
	// 21 T1 AB:5678 DB:--
	// 22 T2 AB:5678 DB:xx MREQ
	// 23 T3 AB:5678 DB:xx MREQ WR
	{ READ_S_PC, DISP_WZ5_READ_S_WZ, DEC_DB_WRITE_S | END },
	// dd/fd 36, 19 cycles, LD (IX/IY+dd),n
	//  9 T1 AB:1236 DB:--
	// 10 T2 AB:1236 DB:dd MREQ RD
	// 11 T3 AB:1236 DB:dd MREQ RD
	// 12 T1 AB:1237 DB:--
	// 13 T2 AB:1237 DB:nn MREQ RD
	// 14 T3 AB:1237 DB:nn MREQ RD
	// 15 T4 AB:1237 DB:--
	// 16 T5 AB:1237 DB:--
	// 17 T1 AB:5678 DB:--
	// 18 T2 AB:5678 DB:nn MREQ
	// 19 T3 AB:5678 DB:nn MREQ WR
	{ READ_S_PC, DISP_WZ2, READ_S_PC, WRITE_S_WZ | END },
	// dd/fd 37, 8 cycles, SCF
	{ SCF | END },
	// dd/fd 38, 11/16 cycles, JR C,n
	{ READ_S_PC, JR_COND | END },
	// dd/fd 39, 15 cycles, ADD IX/IY,SP
	{ ADD16 | END },
	// dd/fd 3a, 17 cycles, LD A,(nn)
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W_READ_S_WZ_INC, DB_A | END },
	// dd/fd 3b, 10 cycles, DEC SP
	{ DEC_R16 | END },
	// dd/fd 3c, 8 cycles, INC A
	{ INC_R8 | END },
	// dd/fd 3d, 8 cycles, DEC A
	{ DEC_R8 | END },
	// dd/fd 3e, 11 cycles, LD A,n
	{ READ_S_PC, DB_REGD | END },
	// dd/fd 3f, 8 cycles, CCF
	{ CCF | END },

	// dd/fd 40, 8 cycles, LD B,B
	{ REGS_TMP_REG | END },
	// dd/fd 41, 8 cycles, LD B,C
	{ REGS_TMP_REG | END },
	// dd/fd 42, 8 cycles, LD B,D
	{ REGS_TMP_REG | END },
	// dd/fd 43, 8 cycles, LD B,E
	{ REGS_TMP_REG | END },
	// dd/fd 44, 8 cycles, LD B,IXh/IYh
	{ REGS_TMP_REG | END },
	// dd/fd 45, 8 cycles, LD B,IXl/IYl
	{ REGS_TMP_REG | END },
	// dd/fd 46, 19 cycles, LD B,(IX/IY+dd)
	//  9 T1 AB:1236 DB:--
	// 10 T2 AB:1236 DB:dd MREQ RD
	// 11 T3 AB:1236 DB:dd MREQ RD
	// 12 T1 AB:1236 DB:--
	// 13 T2 AB:1236 DB:--
	// 14 T3 AB:1236 DB:--
	// 15 T4 AB:1236 DB:--
	// 16 T5 AB:1236 DB:--
	// 17 T1 AB:5678 DB:--
	// 18 T2 AB:5678 DB:ee MREQ RD
	// 19 T3 AB:5678 DB:ee MREQ RD
	{ READ_S_PC, DISP_WZ5_READ_S_WZ, DB_REGD | END },
	// dd/fd 47, 8 cycles, LD B,A
	{ REGS_TMP_REG | END },
	// dd/fd 48, 8 cycles, LD C,B
	{ REGS_TMP_REG | END },
	// dd/fd 49, 8 cycles, LD C,C
	{ REGS_TMP_REG | END },
	// dd/fd 4a, 8 cycles, LD C,D
	{ REGS_TMP_REG | END },
	// dd/fd 4b, 8 cycles, LD C,E
	{ REGS_TMP_REG | END },
	// dd/fd 4c, 8 cycles, LD C,IXh/IYh
	{ REGS_TMP_REG | END },
	// dd/fd 4d, 8 cycles, LD C,IXl/IYl
	{ REGS_TMP_REG | END },
	// dd/fd 4e, 19 cycles, LD C,(IX/IY+dd)
	{ READ_S_PC, DISP_WZ5_READ_S_WZ, DB_REGD | END },
	// dd/fd 4f, 8 cycles, LD C,A
	{ REGS_TMP_REG | END },

	// dd/fd 50, 8 cycles, LD D,B
	{ REGS_TMP_REG | END },
	// dd/fd 51, 8 cycles, LD D,C
	{ REGS_TMP_REG | END },
	// dd/fd 52, 8 cycles, LD D,D
	{ REGS_TMP_REG | END },
	// dd/fd 53, 8 cycles, LD D,E
	{ REGS_TMP_REG | END },
	// dd/fd 54, 8 cycles, LD D,IXh/IYh
	{ REGS_TMP_REG | END },
	// dd/fd 55, 8 cycles, LD D,IXl/IYl
	{ REGS_TMP_REG | END },
	// dd/fd 56, 19 cycles, LD D,(IX/IY+dd)
	{ READ_S_PC, DISP_WZ5_READ_S_WZ, DB_REGD | END },
	// dd/fd 57, 8 cycles, LD D,A
	{ REGS_TMP_REG | END },
	// dd/fd 58, 8 cycles, LD E,B
	{ REGS_TMP_REG | END },
	// dd/fd 59, 8 cycles, LD E,C
	{ REGS_TMP_REG | END },
	// dd/fd 5a, 8 cycles, LD E,D
	{ REGS_TMP_REG | END },
	// dd/fd 5b, 8 cycles, LD E,E
	{ REGS_TMP_REG | END },
	// dd/fd 5c, 8 cycles, LD E,IXh/IYh
	{ REGS_TMP_REG | END },
	// dd/fd 5d, 8 cycles, LD E,IXl/IYl
	{ REGS_TMP_REG | END },
	// dd/fd 5e, 19 cycles, LD E,(IX/IY+dd)
	{ READ_S_PC, DISP_WZ5_READ_S_WZ, DB_REGD | END },
	// dd/fd 5f, 8 cycles, LD E,A
	{ REGS_TMP_REG | END },

	// dd/fd 60, 8 cycles, LD IXh/IYh,B
	{ REGS_TMP_REG | END },
	// dd/fd 61, 8 cycles, LD IXh/IYh,C
	{ REGS_TMP_REG | END },
	// dd/fd 62, 8 cycles, LD IXh/IYh,D
	{ REGS_TMP_REG | END },
	// dd/fd 63, 8 cycles, LD IXh/IYh,E
	{ REGS_TMP_REG | END },
	// dd/fd 64, 8 cycles, LD IXh/IYh,IXh/IYh
	{ REGS_TMP_REG | END },
	// dd/fd 65, 8 cycles, LD IXh/IYh,IXl/IYl
	{ REGS_TMP_REG | END },
	// dd/fd 66, 19 cycles, LD H,(IX/IY+dd)
	{ READ_S_PC, DISP_WZ5_READ_S_WZ, DB_REGD0 | END },
	// dd/fd 67, 8 cycles, LD IXh/IYh,A
	{ REGS_TMP_REG | END },
	// dd/fd 68, 8 cycles, LD IXl/IYl,B
	{ REGS_TMP_REG | END },
	// dd/fd 69, 8 cycles, LD IXl/IYl,C
	{ REGS_TMP_REG | END },
	// dd/fd 6a, 8 cycles, LD IXl/IYl,D
	{ REGS_TMP_REG | END },
	// dd/fd 6b, 8 cycles, LD IXl/IYl,E
	{ REGS_TMP_REG | END },
	// dd/fd 6c, 8 cycles, LD IXl/IYl,IXh/IYh
	{ REGS_TMP_REG | END },
	// dd/fd 6d, 8 cycles, LD IXl/IYl,IXl/IYl
	{ REGS_TMP_REG | END },
	// dd/fd 6e, 19 cycles, LD L,(IX/IY+dd)
	{ READ_S_PC, DISP_WZ5_READ_S_WZ, DB_REGD0 | END },
	// dd/fd 6f, 8 cycles, LD IXl/IYl,A
	{ REGS_TMP_REG | END },

	// dd/fd 70, 19 cycles, LD (IX/IY+dd),B
	//  9 T1 AB:1236 DB:--
	// 10 T2 AB:1236 DB:dd MREQ RD
	// 11 T3 AB:1236 DB:dd MREQ RD
	// 12 T1 AB:1236 DB:--
	// 13 T2 AB:1236 DB:--
	// 14 T3 AB:1236 DB:--
	// 15 T4 AB:1236 DB:--
	// 16 T5 AB:1236 DB:--
	// 17 T1 AB:5678 DB:--
	// 18 T2 AB:5678 DB:ee MREQ 
	// 19 T3 AB:5678 DB:ee MREQ WR
	{ READ_S_PC, DISP_WZ5, REGS0_DB, WRITE_S_WZ | END },
	// dd/fd 71, 19 cycles, LD (IX/IY+dd),C
	{ READ_S_PC, DISP_WZ5, REGS0_DB, WRITE_S_WZ | END },
	// dd/fd 72, 19 cycles, LD (IX/IY+dd),D
	{ READ_S_PC, DISP_WZ5, REGS0_DB, WRITE_S_WZ | END },
	// dd/fd 73, 19 cycles, LD (IX/IY+dd),E
	{ READ_S_PC, DISP_WZ5, REGS0_DB, WRITE_S_WZ | END },
	// dd/fd 74, 19 cycles, LD (IX/IY+dd),H
	{ READ_S_PC, DISP_WZ5, REGS0_DB, WRITE_S_WZ | END },
	// dd/fd 75, 19 cycles, LD (IX/IY+dd),L
	{ READ_S_PC, DISP_WZ5, REGS0_DB, WRITE_S_WZ | END },
	// dd/fd 76, 8 cycles, HALT
	{ HALT | END },
	// dd/fd 77, 19 cycles, LD (IX/IY+dd),A
	{ READ_S_PC, DISP_WZ5, REGS0_DB, WRITE_S_WZ | END },
	// dd/fd 78, 8 cycles, LD A,B
	{ REGS_TMP_REG | END },
	// dd/fd 79, 8 cycles, LD A,C
	{ REGS_TMP_REG | END },
	// dd/fd 7a, 8 cycles, LD A,D
	{ REGS_TMP_REG | END },
	// dd/fd 7b, 8 cycles, LD A,E
	{ REGS_TMP_REG | END },
	// dd/fd 7c, 8 cycles, LD A,IXh/IYh
	{ REGS_TMP_REG | END },
	// dd/fd 7d, 8 cycles, LD A,IXl/IYl
	{ REGS_TMP_REG | END },
	// dd/fd 7e, 19 cycles, LD A,(IX/IY+dd)
	{ READ_S_PC, DISP_WZ5_READ_S_WZ, DB_REGD | END },
	// dd/fd 7f, 8 cycles, LD A,A
	{ REGS_TMP_REG | END },

	// dd/fd 80, 8 cycles, ADD B
	{ ADD_R8 | END },
	// dd/fd 81, 8 cycles, ADD C
	{ ADD_R8 | END },
	// dd/fd 82, 8 cycles, ADD D
	{ ADD_R8 | END },
	// dd/fd 83, 8 cycles, ADD E
	{ ADD_R8 | END },
	// dd/fd 84, 8 cycles, ADD IXh/IYh
	{ ADD_R8 | END },
	// dd/fd 85, 8 cycles, ADD IXl/IYl
	{ ADD_R8 | END },
	// dd/fd 86, 19 cycles, ADD (IX/IY+dd)
	//  9 T1 AB:1236 DB:--
	// 10 T2 AB:1236 DB:dd MREQ RD
	// 11 T3 AB:1236 DB:dd MREQ RD
	// 12 T1 AB:1236 DB:--
	// 13 T2 AB:1236 DB:--
	// 14 T3 AB:1236 DB:--
	// 15 T4 AB:1236 DB:--
	// 16 T5 AB:1236 DB:--
	// 17 T1 AB:5678 DB:--
	// 18 T2 AB:5678 DB:ee MREQ RD
	// 19 T3 AB:5678 DB:ee MREQ RD
	{ READ_S_PC, DISP_WZ5_READ_S_WZ, ADD_DB | END },
	// dd/fd 87, 8 cycles, ADD A
	{ ADD_R8 | END },
	// dd/fd 88, 8 cycles, ADC B
	{ ADC_R8 | END },
	// dd/fd 89, 8 cycles, ADC C
	{ ADC_R8 | END },
	// dd/fd 8a, 8 cycles, ADC D
	{ ADC_R8 | END },
	// dd/fd 8b, 8 cycles, ADC E
	{ ADC_R8 | END },
	// dd/fd 8c, 8 cycles, ADC IXh/IYh
	{ ADC_R8 | END },
	// dd/fd 8d, 8 cycles, ADC IXl/IYl
	{ ADC_R8 | END },
	// dd/fd 8e, 19 cycles, ADC (IX/IY+dd)
	{ READ_S_PC, DISP_WZ5_READ_S_WZ, ADC_DB | END },
	// dd/fd 8f, 8 cycles, ADC A
	{ ADC_R8 | END },

	// dd/fd 90, 8 cycles, SUB B
	{ SUB_R8 | END },
	// dd/fd 91, 8 cycles, SUB C
	{ SUB_R8 | END },
	// dd/fd 92, 8 cycles, SUB D
	{ SUB_R8 | END },
	// dd/fd 93, 8 cycles, SUB E
	{ SUB_R8 | END },
	// dd/fd 94, 8 cycles, SUB IXh/IYh
	{ SUB_R8 | END },
	// dd/fd 95, 8 cycles, SUB IXl/IYl
	{ SUB_R8 | END },
	// dd/fd 96, 19 cycles, SUB (IX/IY+dd)
	{ READ_S_PC, DISP_WZ5_READ_S_WZ, SUB_DB | END },
	// dd/fd 97, 8 cycles, SUB A
	{ SUB_R8 | END },
	// dd/fd 98, 8 cycles, SBC B
	{ SBC_R8 | END },
	// dd/fd 99, 8 cycles, SBC C
	{ SBC_R8 | END },
	// dd/fd 9a, 8 cycles, SBC D
	{ SBC_R8 | END },
	// dd/fd 9b, 8 cycles, SBC E
	{ SBC_R8 | END },
	// dd/fd 9c, 8 cycles, SBC IXh/IYh
	{ SBC_R8 | END },
	// dd/fd 9d, 8 cycles, SBC IXl/IYl
	{ SBC_R8 | END },
	// dd/fd 9e, 19 cycles, SBC (IX/IY+dd)
	{ READ_S_PC, DISP_WZ5_READ_S_WZ, SBC_DB | END },
	// dd/fd 9f, 8 cycles, SBC A
	{ SBC_R8 | END },

	// dd/fd a0, 8 cycles, AND B
	{ AND_R8 | END },
	// dd/fd a1, 8 cycles, AND C
	{ AND_R8 | END },
	// dd/fd a2, 8 cycles, AND D
	{ AND_R8 | END },
	// dd/fd a3, 8 cycles, AND E
	{ AND_R8 | END },
	// dd/fd a4, 8 cycles, AND IXh/IYh
	{ AND_R8 | END },
	// dd/fd a5, 8 cycles, AND IXl/IYl
	{ AND_R8 | END },
	// dd/fd a6, 19 cycles, AND (IX/IY+dd)
	{ READ_S_PC, DISP_WZ5_READ_S_WZ, AND_DB | END },
	// dd/fd a7, 8 cycles, AND A
	{ AND_R8 | END },
	// dd/fd a8, 8 cycles, XOR B
	{ XOR_R8 | END },
	// dd/fd a9, 8 cycles, XOR C
	{ XOR_R8 | END },
	// dd/fd aa, 8 cycles, XOR D
	{ XOR_R8 | END },
	// dd/fd ab, 8 cycles, XOR E
	{ XOR_R8 | END },
	// dd/fd ac, 8 cycles, XOR IXh/IYh
	{ XOR_R8 | END },
	// dd/fd ad, 8 cycles, XOR IXl/IYl
	{ XOR_R8 | END },
	// dd/fd ae, 19 cycles, XOR (IX/IY+dd)
	{ READ_S_PC, DISP_WZ5_READ_S_WZ, XOR_DB | END },
	// dd/fd af, 8 cycles, XOR A
	{ XOR_R8 | END },

	// dd/fd b0, 8 cycles, OR B
	{ OR_R8 | END },
	// dd/fd b1, 8 cycles, OR C
	{ OR_R8 | END },
	// dd/fd b2, 8 cycles, OR D
	{ OR_R8 | END },
	// dd/fd b3, 8 cycles, OR E
	{ OR_R8 | END },
	// dd/fd b4, 8 cycles, OR IXh/IYh
	{ OR_R8 | END },
	// dd/fd b5, 8 cycles, OR IXl/IYl
	{ OR_R8 | END },
	// dd/fd b6, 19 cycles, OR (IX/IY+dd)
	{ READ_S_PC, DISP_WZ5_READ_S_WZ, OR_DB | END },
	// dd/fd b7, 8 cycles, OR A
	{ OR_R8 | END },
	// dd/fd b8, 8 cycles, CP B
	{ CP_R8 | END },
	// dd/fd b9, 8 cycles, CP C
	{ CP_R8 | END },
	// dd/fd ba, 8 cycles, CP D
	{ CP_R8 | END },
	// dd/fd bb, 8 cycles, CP E
	{ CP_R8 | END },
	// dd/fd bc, 8 cycles, CP IXh/IYh
	{ CP_R8 | END },
	// dd/fd bd, 8 cycles, CP IXl/IYl
	{ CP_R8 | END },
	// dd/fd be, 19 cycles, CP (IX/IY+dd)
	{ READ_S_PC, DISP_WZ5_READ_S_WZ, CP_DB | END },
	// dd/fd bf, 8 cycles, CP A
	{ CP_R8 | END },

	// dd/fd c0, 9/15 cycles, RET NZ
	{ RET_COND, DB_Z_READ_S_SP_INC, DB_W_WZ_PC | END },
	// dd/fd c1, 14 cycles, POP BC
	{  READ_S_SP_INC, DB_R16L_READ_S_SP_INC, DB_R16H | END },
	// dd/fd c2, 14 cycles, JP NZ,nn
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W_JP_COND | END },
	// dd/fd c3, 14 cycles, JMP nn
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W_WZ_PC | END },
	// dd/fd c4, 14/21 cycles, CALL NZ,nn
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W_CALL_COND, PCL_WRITE_S_SP_DEC, WZ_PC | END },
	// dd/fd c5, 15 cycles, PUSH BC
	{ R16H_WRITE_S_SP_DEC, R16L_WRITE_S_SP_DEC | END },
	// dd/fd c6, 11 cycles, ADD A,n
	{ READ_S_PC, ADD_DB | END },
	// dd/fd c7, 15 cycles, RST 0H
	{ PCH_WRITE_S_SP_DEC, PCL_WRITE_S_SP_DEC, RST | END },
	// dd/fd c8, 9/15 cycles, RET Z
	{ RET_COND, DB_Z_READ_S_SP_INC, DB_W_WZ_PC | END },
	// dd/fd c9, 14 cycles, RET
	{ READ_S_SP_INC, DB_Z_READ_S_SP_INC, DB_W_WZ_PC | END },
	// dd/fd ca, 14 cycles, JP Z,nn
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W_JP_COND | END },
	// dd/fd cb, +4 cycles, DD/FD + CB prefix
	{ 0 },
	// dd/fd cc, 14/21 cycles, CALL Z,nn
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W_CALL_COND, PCL_WRITE_S_SP_DEC, WZ_PC | END },
	// dd/fd cd, 21 cycles, CALL nn
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W, PCH_WRITE_S_SP_DEC, PCL_WRITE_S_SP_DEC, WZ_PC | END },
	// dd/fd ce, 11 cycles, ADC A,n
	{ READ_S_PC, ADC_DB | END },
	// dd/fd cf, 15 cycles, RST 8H
	{ PCH_WRITE_S_SP_DEC, PCL_WRITE_S_SP_DEC, RST | END },

	// dd/fd d0, 9/15 cycles, RET NC
	{ RET_COND, DB_Z_READ_S_SP_INC, DB_W_WZ_PC | END },
	// dd/fd d1, 14 cycles, POP DE
	{ READ_S_SP_INC, DB_R16L_READ_S_SP_INC, DB_R16H | END },
	// dd/fd d2, 14 cycles, JP NC,nn
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W_JP_COND | END },
	// dd/fd d3, 15 cycles, OUT (n), A
	{ READ_S_PC, DB_Z, A_W, A_DB, OUTPUT_S_WZ_INC, DB_W | END },
	// dd/fd d4, 14/21 cycles, CALL NC,nn
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W_CALL_COND, PCL_WRITE_S_SP_DEC, WZ_PC | END },
	// dd/fd d5, 15 cycles, PUSH DE
	{ R16H_WRITE_S_SP_DEC, R16L_WRITE_S_SP_DEC | END },
	// dd/fd d6, 11 cycles, SUB n
	{ READ_S_PC, SUB_DB | END },
	// dd/fd d7, 15 cycles, RST 10H
	{ PCH_WRITE_S_SP_DEC, PCL_WRITE_S_SP_DEC, RST | END },
	// dd/fd d8, 9/15 cycles, RET C
	{ RET_COND, DB_Z_READ_S_SP_INC, DB_W_WZ_PC | END },
	// dd/fd d9, 8 cycles, EXX
	{ EXX | END },
	// dd/fd da, 14 cycles, JP C,nn
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W_JP_COND | END },
	// dd/fd db, 15 cycles, IN A,(n)
	{ READ_S_PC, DB_Z, A_W, INPUT_S_WZ_INC, INPUT_A | END },
	// dd/fd dc, 14/21 cycles, CALL C,nn
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W_CALL_COND, PCL_WRITE_S_SP_DEC, WZ_PC | END },
	// dd/fd dd, +4 cycles, DD prefix
	{ 0 },
	// dd/fd de, 11 cycles, SBC n
	{ READ_S_PC, SBC_DB | END },
	// dd/fd df, 15 cycles, RST 18H
	{ PCH_WRITE_S_SP_DEC, PCL_WRITE_S_SP_DEC, RST | END },

	// dd/fd e0, 9/15 cycles, RET PO
	{ RET_COND, DB_Z_READ_S_SP_INC, DB_W_WZ_PC | END },
	// dd/fd e1, 14 cycles, POP IX/IY
	{ READ_S_SP_INC, DB_R16L_READ_S_SP_INC, DB_R16H | END },
	// dd/fd e2, 14 cycles, JP PO,nn
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W_JP_COND | END },
	// dd/fd e3, 23 cycles, EX (SP),IX/IY
	{ READ_S_SP_INC, DB_Z_READ_S_SP, EX_SP_WRITE_S, R16L_WRITE_S_SP_DEC, X2, WZ_HL | END },
	// dd/fd e4, 14/21 cycles, CALL PO,nn
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W_CALL_COND, PCL_WRITE_S_SP_DEC, WZ_PC | END },
	// dd/fd e5, 15 cycles, PUSH IX/IY
	{ R16H_WRITE_S_SP_DEC, R16L_WRITE_S_SP_DEC | END },
	// dd/fd e6, 11 cycles, AND n
	{ READ_S_PC, AND_DB | END },
	// dd/fd e7, 15 cycles, RST 20H
	{ PCH_WRITE_S_SP_DEC, PCL_WRITE_S_SP_DEC, RST | END },
	// dd/fd e8, 9/15 cycles, RET PE
	{ RET_COND, DB_Z_READ_S_SP_INC, DB_W_WZ_PC | END },
	// dd/fd e9, 8 cycles, JP (HL)
	{ HL_PC | END },
	// dd/fd ea, 14 cycles, JP PE,nn
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W_JP_COND | END },
	// dd/fd eb, 8 cycles, EX DE,HL
	{ EX_DE_HL | END },
	// dd/fd ec, 14/21 cycles, CALL PE,nn
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W_CALL_COND, PCL_WRITE_S_SP_DEC, WZ_PC | END },
	// dd/fd ed, +4 cycles, ED prefix
	{ 0 },
	// dd/fd ee, 11 cycles, XOR n
	{ READ_S_PC, XOR_DB | END },
	// dd/fd ef, 15 cycles, RST 28H
	{ PCH_WRITE_S_SP_DEC, PCL_WRITE_S_SP_DEC, RST | END },

	// dd/fd f0, 9/15 cycles, RET P
	{ RET_COND, DB_Z_READ_S_SP_INC, DB_W_WZ_PC | END },
	// dd/fd f1, 14 cycles, POP AF
	{ READ_S_SP_INC, DB_R16L_READ_S_SP_INC, DB_R16H | END },
	// dd/fd f2, 14 cycles, JP P,nn
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W_JP_COND | END },
	// dd/fd f3, 8 cycles, DI
	{ DI | END },
	// dd/fd f4, 14/21 cycles, CALL P,nn
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W_CALL_COND, PCL_WRITE_S_SP_DEC, WZ_PC | END },
	// dd/fd f5, 15 cycles, PUSH AF
	{ R16H_WRITE_S_SP_DEC, R16L_WRITE_S_SP_DEC | END },
	// dd/fd f6, 11 cycles, OR n
	{ READ_S_PC, OR_DB | END },
	// dd/fd f7, 15 cycles, RST 30H
	{ PCH_WRITE_S_SP_DEC, PCL_WRITE_S_SP_DEC, RST | END },
	// dd/fd f8, 9/15 cycles, RET M
	{ RET_COND, DB_Z_READ_S_SP_INC, DB_W_WZ_PC | END },
	// dd/fd f9, 10 cycles, LD SP,IX/IY
	{ LD_SP_HL | END },
	// dd/fd fa, 14 cycles, JP M,nn
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W_JP_COND | END },
	// dd/fd fb, 8 cycles, EI
	{ EI | END },
	// dd/fd fc, 14/21 cycles, CALL M,nn
	{ READ_S_PC, DB_Z_READ_S_PC, DB_W_CALL_COND, PCL_WRITE_S_SP_DEC, WZ_PC | END },
	// dd/fd fd, +4 cycles, FD prefix
	{ 0 },
	// dd/fd fe, 11 cycles, CP n
	{ READ_S_PC, CP_DB | END },
	// dd/fd ff, 15 cycles, RST 38H
	{ PCH_WRITE_S_SP_DEC, PCL_WRITE_S_SP_DEC, RST | END },

	///////////////////////////////////////////////////////
	// DD/FD + CB prefixed instructions                  //
	///////////////////////////////////////////////////////

	// dd/fd cb dd 00, 23 cycles, RLC (IX/IY+dd),B
	// 17 T1 AB:5678 DB:--
	// 18 T2 AB:5678 DB:xx MREQ RD
	// 19 T3 AB:5678 DB:xx MREQ RD
	// 20 T4 AB:5678 DB:--
	// 21 T1 AB:5678 DB:--
	// 22 T2 AB:5678 DB:yy MREQ
	// 23 T3 AB:5678 DB:yy MREQ WR
	{ READ_S_WZ, RLC_WRITE_S | END },
	// dd/fd cb dd 01, 23 cycles, RLC (IX/IY+dd),C
	{ READ_S_WZ, RLC_WRITE_S | END },
	// dd/fd cb dd 02, 23 cycles, RLC (IX/IY+dd),D
	{ READ_S_WZ, RLC_WRITE_S | END },
	// dd/fd cb dd 03, 23 cycles, RLC (IX/IY+dd),E
	{ READ_S_WZ, RLC_WRITE_S | END },
	// dd/fd cb dd 04, 23 cycles, RLC (IX/IY+dd),H
	{ READ_S_WZ, RLC_WRITE_S | END },
	// dd/fd cb dd 05, 23 cycles, RLC (IX/IY+dd),L
	{ READ_S_WZ, RLC_WRITE_S | END },
	// dd/fd cb dd 06, 23 cycles, RLC (IX/IY+dd)
	{ READ_S_WZ, RLC_WRITE_S | END },
	// dd/fd cb dd 07, 23 cycles, RLC (IX/IY+dd),A
	{ READ_S_WZ, RLC_WRITE_S | END },
	// dd/fd cb dd 08, 23 cycles, RRC (IX/IY+dd),B
	{ READ_S_WZ, RRC_WRITE_S | END },
	// dd/fd cb dd 09, 23 cycles, RRC (IX/IY+dd),C
	{ READ_S_WZ, RRC_WRITE_S | END },
	// dd/fd cb dd 0a, 23 cycles, RRC (IX/IY+dd),D
	{ READ_S_WZ, RRC_WRITE_S | END },
	// dd/fd cb dd 0b, 23 cycles, RRC (IX/IY+dd),E
	{ READ_S_WZ, RRC_WRITE_S | END },
	// dd/fd cb dd 0c, 23 cycles, RRC (IX/IY+dd),H
	{ READ_S_WZ, RRC_WRITE_S | END },
	// dd/fd cb dd 0d, 23 cycles, RRC (IX/IY+dd),L
	{ READ_S_WZ, RRC_WRITE_S | END },
	// dd/fd cb dd 0e, 23 cycles, RRC (IX/IY+dd)
	{ READ_S_WZ, RRC_WRITE_S | END },
	// dd/fd cb dd 0f, 23 cycles, RRC (IX/IY+dd),A
	{ READ_S_WZ, RRC_WRITE_S | END },

	// dd/fd cb dd 10, 23 cycles, RL (IX/IY+dd),B
	{ READ_S_WZ, RL_WRITE_S | END },
	// dd/fd cb dd 11, 23 cycles, RL (IX/IY+dd),C
	{ READ_S_WZ, RL_WRITE_S | END },
	// dd/fd cb dd 12, 23 cycles, RL (IX/IY+dd),D
	{ READ_S_WZ, RL_WRITE_S | END },
	// dd/fd cb dd 13, 23 cycles, RL (IX/IY+dd),E
	{ READ_S_WZ, RL_WRITE_S | END },
	// dd/fd cb dd 14, 23 cycles, RL (IX/IY+dd),H
	{ READ_S_WZ, RL_WRITE_S | END },
	// dd/fd cb dd 15, 23 cycles, RL (IX/IY+dd),L
	{ READ_S_WZ, RL_WRITE_S | END },
	// dd/fd cb dd 16, 23 cycles, RL (IX/IY+dd)
	{ READ_S_WZ, RL_WRITE_S | END },
	// dd/fd cb dd 17, 23 cycles, RL (IX/IY+dd),A
	{ READ_S_WZ, RL_WRITE_S | END },
	// dd/fd cb dd 18, 23 cycles, RR (IX/IY+dd),B
	{ READ_S_WZ, RR_WRITE_S | END },
	// dd/fd cb dd 19, 23 cycles, RR (IX/IY+dd),C
	{ READ_S_WZ, RR_WRITE_S | END },
	// dd/fd cb dd 1a, 23 cycles, RR (IX/IY+dd),D
	{ READ_S_WZ, RR_WRITE_S | END },
	// dd/fd cb dd 1b, 23 cycles, RR (IX/IY+dd),E
	{ READ_S_WZ, RR_WRITE_S | END },
	// dd/fd cb dd 1c, 23 cycles, RR (IX/IY+dd),H
	{ READ_S_WZ, RR_WRITE_S | END },
	// dd/fd cb dd 1d, 23 cycles, RR (IX/IY+dd),L
	{ READ_S_WZ, RR_WRITE_S | END },
	// dd/fd cb dd 1e, 23 cycles, RR (IX/IY+dd)
	{ READ_S_WZ, RR_WRITE_S | END },
	// dd/fd cb dd 1f, 23 cycles, RR (IX/IY+dd),A
	{ READ_S_WZ, RR_WRITE_S | END },

	// dd/fd cb dd 20, 23 cycles, SLA (IX/IY+dd),B
	{ READ_S_WZ, SLA_WRITE_S | END },
	// dd/fd cb dd 21, 23 cycles, SLA (IX/IY+dd),C
	{ READ_S_WZ, SLA_WRITE_S | END },
	// dd/fd cb dd 22, 23 cycles, SLA (IX/IY+dd),D
	{ READ_S_WZ, SLA_WRITE_S | END },
	// dd/fd cb dd 23, 23 cycles, SLA (IX/IY+dd),E
	{ READ_S_WZ, SLA_WRITE_S | END },
	// dd/fd cb dd 24, 23 cycles, SLA (IX/IY+dd),H
	{ READ_S_WZ, SLA_WRITE_S | END },
	// dd/fd cb dd 25, 23 cycles, SLA (IX/IY+dd),L
	{ READ_S_WZ, SLA_WRITE_S | END },
	// dd/fd cb dd 26, 23 cycles, SLA (IX/IY+dd)
	{ READ_S_WZ, SLA_WRITE_S | END },
	// dd/fd cb dd 27, 23 cycles, SLA (IX/IY+dd),A
	{ READ_S_WZ, SLA_WRITE_S | END },
	// dd/fd cb dd 28, 23 cycles, SRA (IX/IY+dd),B
	{ READ_S_WZ, SRA_WRITE_S | END },
	// dd/fd cb dd 29, 23 cycles, SRA (IX/IY+dd),C
	{ READ_S_WZ, SRA_WRITE_S | END },
	// dd/fd cb dd 2a, 23 cycles, SRA (IX/IY+dd),D
	{ READ_S_WZ, SRA_WRITE_S | END },
	// dd/fd cb dd 2b, 23 cycles, SRA (IX/IY+dd),E
	{ READ_S_WZ, SRA_WRITE_S | END },
	// dd/fd cb dd 2c, 23 cycles, SRA (IX/IY+dd),H
	{ READ_S_WZ, SRA_WRITE_S | END },
	// dd/fd cb dd 2d, 23 cycles, SRA (IX/IY+dd),L
	{ READ_S_WZ, SRA_WRITE_S | END },
	// dd/fd cb dd 2e, 23 cycles, SRA (IX/IY+dd)
	{ READ_S_WZ, SRA_WRITE_S | END },
	// dd/fd cb dd 2f, 23 cycles, SRA (IX/IY+dd),A
	{ READ_S_WZ, SRA_WRITE_S | END },

	// dd/fd cb dd 30, 23 cycles, SLL (IX/IY+dd),B
	{ READ_S_WZ, SLL_WRITE_S | END },
	// dd/fd cb dd 31, 23 cycles, SLL (IX/IY+dd),C
	{ READ_S_WZ, SLL_WRITE_S | END },
	// dd/fd cb dd 32, 23 cycles, SLL (IX/IY+dd),D
	{ READ_S_WZ, SLL_WRITE_S | END },
	// dd/fd cb dd 33, 23 cycles, SLL (IX/IY+dd),E
	{ READ_S_WZ, SLL_WRITE_S | END },
	// dd/fd cb dd 34, 23 cycles, SLL (IX/IY+dd),H
	{ READ_S_WZ, SLL_WRITE_S | END },
	// dd/fd cb dd 35, 23 cycles, SLL (IX/IY+dd),L
	{ READ_S_WZ, SLL_WRITE_S | END },
	// dd/fd cb dd 36, 23 cycles, SLL (IX/IY+dd)
	{ READ_S_WZ, SLL_WRITE_S | END },
	// dd/fd cb dd 37, 23 cycles, SLL (IX/IY+dd),A
	{ READ_S_WZ, SLL_WRITE_S | END },
	// dd/fd cb dd 38, 23 cycles, SRL (IX/IY+dd),B
	{ READ_S_WZ, SRL_WRITE_S | END },
	// dd/fd cb dd 39, 23 cycles, SRL (IX/IY+dd),C
	{ READ_S_WZ, SRL_WRITE_S | END },
	// dd/fd cb dd 3a, 23 cycles, SRL (IX/IY+dd),D
	{ READ_S_WZ, SRL_WRITE_S | END },
	// dd/fd cb dd 3b, 23 cycles, SRL (IX/IY+dd),E
	{ READ_S_WZ, SRL_WRITE_S | END },
	// dd/fd cb dd 3c, 23 cycles, SRL (IX/IY+dd),H
	{ READ_S_WZ, SRL_WRITE_S | END },
	// dd/fd cb dd 3d, 23 cycles, SRL (IX/IY+dd),L
	{ READ_S_WZ, SRL_WRITE_S | END },
	// dd/fd cb dd 3e, 23 cycles, SRL (IX/IY+dd)
	{ READ_S_WZ, SRL_WRITE_S | END },
	// dd/fd cb dd 3f, 23 cycles, SRL (IX/IY+dd),A
	{ READ_S_WZ, SRL_WRITE_S | END },

	// dd/fd cb dd 40, 20 cycles, BIT 0,(IX/IY+dd)*
	// 17 T1 AB:5678 DB:--
	// 18 T2 AB:5678 DB:xx MREQ RD
	// 19 T3 AB:5678 DB:xx MREQ RD
	// 20 T4 AB:5678 DB:--
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 41, 20 cycles, BIT 0,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 42, 20 cycles, BIT 0,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 43, 20 cycles, BIT 0,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 44, 20 cycles, BIT 0,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 45, 20 cycles, BIT 0,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 46, 20 cycles, BIT 0,(IX/IY+dd)
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 47, 20 cycles, BIT 0,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 48, 20 cycles, BIT 1,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 49, 20 cycles, BIT 1,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 4a, 20 cycles, BIT 1,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 4b, 20 cycles, BIT 1,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 4c, 20 cycles, BIT 1,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 4d, 20 cycles, BIT 1,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 4e, 20 cycles, BIT 1,(IX/IY+dd)
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 4f, 20 cycles, BIT 1,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },

	// dd/fd cb dd 50, 20 cycles, BIT 2,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 51, 20 cycles, BIT 2,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 52, 20 cycles, BIT 2,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 53, 20 cycles, BIT 2,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 54, 20 cycles, BIT 2,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 55, 20 cycles, BIT 2,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 56, 20 cycles, BIT 2,(IX/IY+dd)
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 57, 20 cycles, BIT 2,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 58, 20 cycles, BIT 3,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 59, 20 cycles, BIT 3,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 5a, 20 cycles, BIT 3,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 5b, 20 cycles, BIT 3,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 5c, 20 cycles, BIT 3,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 5d, 20 cycles, BIT 3,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 5e, 20 cycles, BIT 3,(IX/IY+dd)
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 5f, 20 cycles, BIT 3,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },

	// dd/fd cb dd 60, 20 cycles, BIT 4,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 61, 20 cycles, BIT 4,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 62, 20 cycles, BIT 4,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 63, 20 cycles, BIT 4,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 64, 20 cycles, BIT 4,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 65, 20 cycles, BIT 4,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 66, 20 cycles, BIT 4,(IX/IY+dd)
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 67, 20 cycles, BIT 4,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 68, 20 cycles, BIT 5,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 69, 20 cycles, BIT 5,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 6a, 20 cycles, BIT 5,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 6b, 20 cycles, BIT 5,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 6c, 20 cycles, BIT 5,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 6d, 20 cycles, BIT 5,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 6e, 20 cycles, BIT 5,(IX/IY+dd)
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 6f, 20 cycles, BIT 5,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },

	// dd/fd cb dd 70, 20 cycles, BIT 6,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 71, 20 cycles, BIT 6,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 72, 20 cycles, BIT 6,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 73, 20 cycles, BIT 6,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 74, 20 cycles, BIT 6,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 75, 20 cycles, BIT 6,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 76, 20 cycles, BIT 6,(IX/IY+dd)
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 77, 20 cycles, BIT 6,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 78, 20 cycles, BIT 7,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 79, 20 cycles, BIT 7,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 7a, 20 cycles, BIT 7,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 7b, 20 cycles, BIT 7,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 7c, 20 cycles, BIT 7,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 7d, 20 cycles, BIT 7,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 7e, 20 cycles, BIT 7,(IX/IY+dd)
	{ READ_S_WZ, BIT_DB | END },
	// dd/fd cb dd 7f, 20 cycles, BIT 7,(IX/IY+dd)*
	{ READ_S_WZ, BIT_DB | END },

	// dd/fd cb dd 80, 23 cycles, RES 0,(IX/IY+dd),B
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd 81, 23 cycles, RES 0,(IX/IY+dd),C
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd 82, 23 cycles, RES 0,(IX/IY+dd),D
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd 83, 23 cycles, RES 0,(IX/IY+dd),E
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd 84, 23 cycles, RES 0,(IX/IY+dd),H
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd 85, 23 cycles, RES 0,(IX/IY+dd),L
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd 86, 23 cycles, RES 0,(IX/IY+dd)
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd 87, 23 cycles, RES 0,(IX/IY+dd),A
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd 88, 23 cycles, RES 1,(IX/IY+dd),B
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd 89, 23 cycles, RES 1,(IX/IY+dd),C
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd 8a, 23 cycles, RES 1,(IX/IY+dd),D
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd 8b, 23 cycles, RES 1,(IX/IY+dd),E
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd 8c, 23 cycles, RES 1,(IX/IY+dd),H
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd 8d, 23 cycles, RES 1,(IX/IY+dd),L
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd 8e, 23 cycles, RES 1,(IX/IY+dd)
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd 8f, 23 cycles, RES 1,(IX/IY+dd),A
	{ READ_S_WZ, RES_WRITE_S | END },

	// dd/fd cb dd 90, 23 cycles, RES 2,(IX/IY+dd),B
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd 91, 23 cycles, RES 2,(IX/IY+dd),C
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd 92, 23 cycles, RES 2,(IX/IY+dd),D
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd 93, 23 cycles, RES 2,(IX/IY+dd),E
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd 94, 23 cycles, RES 2,(IX/IY+dd),H
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd 95, 23 cycles, RES 2,(IX/IY+dd),L
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd 96, 23 cycles, RES 2,(IX/IY+dd)
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd 97, 23 cycles, RES 2,(IX/IY+dd),A
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd 98, 23 cycles, RES 3,(IX/IY+dd),B
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd 99, 23 cycles, RES 3,(IX/IY+dd),C
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd 9a, 23 cycles, RES 3,(IX/IY+dd),D
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd 9b, 23 cycles, RES 3,(IX/IY+dd),E
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd 9c, 23 cycles, RES 3,(IX/IY+dd),H
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd 9d, 23 cycles, RES 3,(IX/IY+dd),L
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd 9e, 23 cycles, RES 3,(IX/IY+dd)
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd 9f, 23 cycles, RES 3,(IX/IY+dd),A
	{ READ_S_WZ, RES_WRITE_S | END },

	// dd/fd cb dd a0, 23 cycles, RES 4,(IX/IY+dd),B
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd a1, 23 cycles, RES 4,(IX/IY+dd),C
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd a2, 23 cycles, RES 4,(IX/IY+dd),D
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd a3, 23 cycles, RES 4,(IX/IY+dd),E
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd a4, 23 cycles, RES 4,(IX/IY+dd),H
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd a5, 23 cycles, RES 4,(IX/IY+dd),L
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd a6, 23 cycles, RES 4,(IX/IY+dd)
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd a7, 23 cycles, RES 4,(IX/IY+dd),A
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd a8, 23 cycles, RES 5,(IX/IY+dd),B
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd a9, 23 cycles, RES 5,(IX/IY+dd),C
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd aa, 23 cycles, RES 5,(IX/IY+dd),D
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd ab, 23 cycles, RES 5,(IX/IY+dd),E
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd ac, 23 cycles, RES 5,(IX/IY+dd),H
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd ad, 23 cycles, RES 5,(IX/IY+dd),L
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd ae, 23 cycles, RES 5,(IX/IY+dd)
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd af, 23 cycles, RES 5,(IX/IY+dd),A
	{ READ_S_WZ, RES_WRITE_S | END },

	// dd/fd cb dd b0, 23 cycles, RES 6,(IX/IY+dd),B
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd b1, 23 cycles, RES 6,(IX/IY+dd),C
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd b2, 23 cycles, RES 6,(IX/IY+dd),D
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd b3, 23 cycles, RES 6,(IX/IY+dd),E
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd b4, 23 cycles, RES 6,(IX/IY+dd),H
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd b5, 23 cycles, RES 6,(IX/IY+dd),L
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd b6, 23 cycles, RES 6,(IX/IY+dd)
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd b7, 23 cycles, RES 6,(IX/IY+dd),A
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd b8, 23 cycles, RES 7,(IX/IY+dd),B
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd b9, 23 cycles, RES 7,(IX/IY+dd),C
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd ba, 23 cycles, RES 7,(IX/IY+dd),D
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd bb, 23 cycles, RES 7,(IX/IY+dd),E
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd bc, 23 cycles, RES 7,(IX/IY+dd),H
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd bd, 23 cycles, RES 7,(IX/IY+dd),L
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd be, 23 cycles, RES 7,(IX/IY+dd)
	{ READ_S_WZ, RES_WRITE_S | END },
	// dd/fd cb dd bf, 23 cycles, RES 7,(IX/IY+dd),A
	{ READ_S_WZ, RES_WRITE_S | END },

	// dd/fd cb dd c0, 23 cycles, SET 0,(IX/IY+dd),B
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd c1, 23 cycles, SET 0,(IX/IY+dd),C
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd c2, 23 cycles, SET 0,(IX/IY+dd),D
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd c3, 23 cycles, SET 0,(IX/IY+dd),E
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd c4, 23 cycles, SET 0,(IX/IY+dd),H
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd c5, 23 cycles, SET 0,(IX/IY+dd),L
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd c6, 23 cycles, SET 0,(IX/IY+dd)
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd c7, 23 cycles, SET 0,(IX/IY+dd),A
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd c8, 23 cycles, SET 1,(IX/IY+dd),B
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd c9, 23 cycles, SET 1,(IX/IY+dd),C
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd ca, 23 cycles, SET 1,(IX/IY+dd),D
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd cb, 23 cycles, SET 1,(IX/IY+dd),E
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd cc, 23 cycles, SET 1,(IX/IY+dd),H
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd cd, 23 cycles, SET 1,(IX/IY+dd),L
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd ce, 23 cycles, SET 1,(IX/IY+dd)
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd cf, 23 cycles, SET 1,(IX/IY+dd),A
	{ READ_S_WZ, SET_WRITE_S | END },

	// dd/fd cb dd d0, 23 cycles, SET 2,(IX/IY+dd),B
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd d1, 23 cycles, SET 2,(IX/IY+dd),C
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd d2, 23 cycles, SET 2,(IX/IY+dd),D
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd d3, 23 cycles, SET 2,(IX/IY+dd),E
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd d4, 23 cycles, SET 2,(IX/IY+dd),H
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd d5, 23 cycles, SET 2,(IX/IY+dd),L
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd d6, 23 cycles, SET 2,(IX/IY+dd)
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd d7, 23 cycles, SET 2,(IX/IY+dd),A
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd d8, 23 cycles, SET 3,(IX/IY+dd),B
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd d9, 23 cycles, SET 3,(IX/IY+dd),C
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd da, 23 cycles, SET 3,(IX/IY+dd),D
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd db, 23 cycles, SET 3,(IX/IY+dd),E
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd dc, 23 cycles, SET 3,(IX/IY+dd),H
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd dd, 23 cycles, SET 3,(IX/IY+dd),L
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd de, 23 cycles, SET 3,(IX/IY+dd)
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd df, 23 cycles, SET 3,(IX/IY+dd),A
	{ READ_S_WZ, SET_WRITE_S | END },

	// dd/fd cb dd e0, 23 cycles, SET 4,(IX/IY+dd),B
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd e1, 23 cycles, SET 4,(IX/IY+dd),C
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd e2, 23 cycles, SET 4,(IX/IY+dd),D
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd e3, 23 cycles, SET 4,(IX/IY+dd),E
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd e4, 23 cycles, SET 4,(IX/IY+dd),H
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd e5, 23 cycles, SET 4,(IX/IY+dd),L
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd e6, 23 cycles, SET 4,(IX/IY+dd)
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd e7, 23 cycles, SET 4,(IX/IY+dd),A
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd e8, 23 cycles, SET 5,(IX/IY+dd),B
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd e9, 23 cycles, SET 5,(IX/IY+dd),C
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd ea, 23 cycles, SET 5,(IX/IY+dd),D
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd eb, 23 cycles, SET 5,(IX/IY+dd),E
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd ec, 23 cycles, SET 5,(IX/IY+dd),H
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd ed, 23 cycles, SET 5,(IX/IY+dd),L
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd ee, 23 cycles, SET 5,(IX/IY+dd)
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd ef, 23 cycles, SET 5,(IX/IY+dd),A
	{ READ_S_WZ, SET_WRITE_S | END },

	// dd/fd cb dd f0, 23 cycles, SET 6,(IX/IY+dd),B
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd f1, 23 cycles, SET 6,(IX/IY+dd),C
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd f2, 23 cycles, SET 6,(IX/IY+dd),D
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd f3, 23 cycles, SET 6,(IX/IY+dd),E
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd f4, 23 cycles, SET 6,(IX/IY+dd),H
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd f5, 23 cycles, SET 6,(IX/IY+dd),L
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd f6, 23 cycles, SET 6,(IX/IY+dd)
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd f7, 23 cycles, SET 6,(IX/IY+dd),A
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd f8, 23 cycles, SET 7,(IX/IY+dd),B
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd f9, 23 cycles, SET 7,(IX/IY+dd),C
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd fa, 23 cycles, SET 7,(IX/IY+dd),D
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd fb, 23 cycles, SET 7,(IX/IY+dd),E
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd fc, 23 cycles, SET 7,(IX/IY+dd),H
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd fd, 23 cycles, SET 7,(IX/IY+dd),L
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd fe, 23 cycles, SET 7,(IX/IY+dd)
	{ READ_S_WZ, SET_WRITE_S | END },
	// dd/fd cb dd ff, 23 cycles, SET 7,(IX/IY+dd),A
	{ READ_S_WZ, SET_WRITE_S | END },

	///////////////////////////////////////////////////////
	// Special sequences                                 //
	///////////////////////////////////////////////////////

	// CB/ED/DD/FD prefixed instructions have 2 M1 cycles taking an initial total of 8 cycles
	// M1, 4 cycles
	// 1 T1 AB:1234 DB:-- M1
	// 2 T2 AB:1234 DB:YY M1      MREQ RD
	// 3 T3 AB:1234 DB:--    RFSH
	// 4 T4 AB:1234 DB:--    RFSH MREQ
	{ READ_OP1_S, REFRESH_DECODE },
	// DD/FD CB, 8 cycles, read displacement and next opcode
	//  9 T1 AB:1236 DB:--
	// 10 T2 AB:1236 DB:dd
	// 11 T3 AB:1236 DB:dd
	// 12 T1 AB:1237 DB:--
	// 13 T2 AB:1237 DB:op
	// 14 T3 AB:1237 DB:op
	// 15 T4 AB:1237 DB:--
	// 16 T5 AB:1237 DB:--
	{ READ_S_PC, DISP_WZ2_READ_OP2_S, DECODE_X2 },
	// Take IRQ Mode 0, 6 cycles
	//  1  T1 AB:ppcc DB:-- M1                  Begin IRQACK cycle to get vector information from interrupting device
	//  2  T2 AB:ppcc DB:-- M1
	//  3  T3 AB:ppcc DB:-- M1
	//  4  T4 AB:ppcc DB:-- M1           IORQ   Wait states can be inserted here
	//  5  T5 AB:rfsh DB:--    RFSH
	//  6  T6 AB:rfsh DB:--    RFSH MREQ
	// Execution of further instruction reading and processing continues here
	// Exanple with a CALL mmnn instruction:
	//  1  T1 AB:ppcc DB:-- M1
	//  2  T2 AB:ppcc DB:-- M1
	//  3  T3 AB:ppcc DB:-- M1
	//  4  T4 AB:ppcc DB:-- M1           IORQ
	//  5  T5 AB:rfsh DB:--    RFSH
	//  6  T6 AB:rfsh DB:--    RFSH MREQ
	//  8  T8 AB:ppcc DB:--                    This is actually processing of the CD instruction already, the reads actually occur from the vector
	//  9  T9 AB:ppcc DB:nn    MREQ RD
	// 10 T10 AB:ppcc DB:nn    MREQ RD
	// 11 T11 AB:ppcc DB:--
	// 12 T12 AB:ppcc DB:mm    MREQ RD
	// 13 T13 AB:ppcc DB:mm    MREQ RD
	{ READ_OP_IRQ, REFRESH_DECODE },
	// Take IRQ Mode 1, 6 cycles, Taking IRQ
	// Only the first 6 cycles are IRQ specific the rest of the cycles is execution of an RST $38 instruction
	//  1 T1 AB:ppcc DB:-- M1
	//  2 T2 AB:ppcc DB:-- M1
	//  3 T3 AB:ppcc DB:-- M1
	//  4 T4 AB:ppcc DB:-- M1           IORQ
	//  5 T5 AB:rfsh DB:--    RFSH
	//  6 T6 AB:rfsh DB:--    RFSH MREQ
	//  7 T5* AB:rfsh DB:--                 Execution of RST instruction continues here
	//  8 T1 AB:sspp DB:--
	//  9 T2 AB:sspp DB:cc    MREQ
	// 10 T3 AB:sspp DB:cc    MREQ WR
	// 11 T1 AB:sspp DB:--
	// 12 T2 AB:sspp DB:pp    MREQ
	// 13 T3 AB:sspp DB:pp    MREQ WR
	{ READ_OP_IRQ, REFRESH_DECODE },
	// Take IRQ Mode 2, 19 cycles
	//  1  T1 AB:ppcc DB:-- M1
	//  2  T2 AB:ppcc DB:-- M1
	//  3  T3 AB:ppcc DB:-- M1
	//  4  T4 AB:ppcc DB:-- M1           IORQ (vector is read here)
	//  5  T5 AB:rfsh DB:--    RFSH
	//  6  T6 AB:rfsh DB:--    RFSH MREQ
	//  7  T7 AB:rfsh DB:--
	//  8  T8 AB:sspp DB:--
	//  9  T9 AB:sspp DB:cc    MREQ
	// 10 T10 AB:sspp DB:cc    MREQ WR
	// 11 T11 AB:sspp DB:--
	// 12 T12 AB:sspp DB:pp    MREQ
	// 13 T13 AB:sspp DB:pp    MREQ WR
	// 14 T14 AB:iivv DB:--
	// 15 T15 AB:iivv DB:yy    MREQ RD
	// 16 T16 AB:iivv DB:yy    MREQ RD
	// 17 T17 AB:iivv DB:--
	// 18 T18 AB:iivv DB:xx    MREQ RD
	// 19 T19 AB:iivv DB:xx    MREQ RD
	{ READ_OP_IRQ, REFRESH, PCH_WRITE_S_SP_DEC, PCL_WRITE_S_SP_DEC, READ_S_VEC, DB_Z_READ_S_VEC, DB_W_WZ_PC | END },
	// Take NMI, 11 cycles, opcode is read but ignored, also asserts M1
	//  1 T1 AB:ppcc DB:-- M1
	//  2 T2 AB:ppcc DB:xx M1 MREQ RD
	//  3 T3 AB:ppcc DB:-- RFSH
	//  4 T4 AB:ppcc DB:-- RFSH MREQ
	//  5 T5 AB:ppcc DB:--
	//  6 T1 AB:sspp DB:--
	//  7 T2 AB:sspp DB:cc MREQ
	//  8 T3 AB:sspp DB:cc MREQ WR
	//  9 T1 AB:sspp DB:--
	// 10 T2 AB:sspp DB:pp MREQ
	// 11 T3 AB:sspp DB:pp MREQ WR
	{ READ_OP_S, REFRESH, PCH_WRITE_S_SP_DEC, PCL_WRITE_S_SP_DEC, NMI | END },

};


inline u16 z80lle_device::adc16(u16 arg1, u16 arg2)
{
	u32 res = arg1 + arg2 + (m_af.b.l & CF);
	m_wz.w.l = arg1 + 1;
	m_af.b.l = (((arg1 ^ res ^ arg2) >> 8) & HF) |
		((res >> 16) & CF) |
		((res >> 8) & (SF | YF | XF)) |
		((res & 0xffff) ? 0 : ZF) |
		(((arg2 ^ arg1 ^ 0x8000) & (arg2 ^ res) & 0x8000) >> 13);
	return res;
}


inline u16 z80lle_device::add16(u16 arg1, u16 arg2)
{
	u32 res = arg1 + arg2;
	m_wz.w.l = res + 1;
	m_af.b.l = (m_af.b.l & (SF | ZF | VF)) |
		(((arg1 ^ res ^ arg2) >> 8) & HF) |
		((res >> 16) & CF) | ((res >> 8) & (YF | XF));
	return (u16)res;
}


inline u16 z80lle_device::sbc16(u16 arg1, u16 arg2)
{
	u32 res = arg1 - arg2 - (m_af.b.l & CF);
	m_wz.w.l = arg1 + 1;
	m_af.b.l = (((arg1 ^ res ^ arg2) >> 8) & HF) | NF |
		((res >> 16) & CF) |
		((res >> 8) & (SF | YF | XF)) |
		((res & 0xffff) ? 0 : ZF) |
		(((arg2 ^ arg1) & (arg1 ^ res) & 0x8000) >> 13);
	return res;
}


inline void z80lle_device::leave_halt()
{
	if (m_halt)
	{
		m_halt = 0;
		m_halt_cb(m_halt);
		m_pc.w.l++;
	}
}


inline void z80lle_device::adc16()
{
	switch (m_ir & 0x30)
	{
	case 0x00:
		m_hl_index[m_hl_offset].w.l = adc16(m_hl_index[m_hl_offset].w.l, m_bc.w.l);
		break;
	case 0x10:
		m_hl_index[m_hl_offset].w.l = adc16(m_hl_index[m_hl_offset].w.l, m_de.w.l);
		break;
	case 0x20:
		m_hl_index[m_hl_offset].w.l = adc16(m_hl_index[m_hl_offset].w.l, m_hl_index[m_hl_offset].w.l);
		break;
	case 0x30:
		m_hl_index[m_hl_offset].w.l = adc16(m_hl_index[m_hl_offset].w.l, m_sp.w.l);
		break;
	}
	m_icount -= 7;
}


inline void z80lle_device::add16()
{
	switch (m_ir & 0x30)
	{
	case 0x00:
		m_hl_index[m_hl_offset].w.l = add16(m_hl_index[m_hl_offset].w.l, m_bc.w.l);
		break;
	case 0x10:
		m_hl_index[m_hl_offset].w.l = add16(m_hl_index[m_hl_offset].w.l, m_de.w.l);
		break;
	case 0x20:
		m_hl_index[m_hl_offset].w.l = add16(m_hl_index[m_hl_offset].w.l, m_hl_index[m_hl_offset].w.l);
		break;
	case 0x30:
		m_hl_index[m_hl_offset].w.l = add16(m_hl_index[m_hl_offset].w.l, m_sp.w.l);
		break;
	}
	m_icount -= 7;
}


inline void z80lle_device::sbc16()
{
	switch (m_ir & 0x30)
	{
	case 0x00:
		m_hl_index[m_hl_offset].w.l = sbc16(m_hl_index[m_hl_offset].w.l, m_bc.w.l);
		break;
	case 0x10:
		m_hl_index[m_hl_offset].w.l = sbc16(m_hl_index[m_hl_offset].w.l, m_de.w.l);
		break;
	case 0x20:
		m_hl_index[m_hl_offset].w.l = sbc16(m_hl_index[m_hl_offset].w.l, m_hl_index[m_hl_offset].w.l);
		break;
	case 0x30:
		m_hl_index[m_hl_offset].w.l = sbc16(m_hl_index[m_hl_offset].w.l, m_sp.w.l);
		break;
	}
	m_icount -= 7;
}


inline void z80lle_device::alu_adc(u8 arg)
{
	u8 res = m_af.b.h + arg + (m_af.b.l & CF);
	m_af.b.l = SZHVC_add[((m_af.b.l & CF) << 16) | (m_af.b.h << 8) | res];
	m_af.b.h = res;
}


inline void z80lle_device::alu_add(u8 arg)
{
	u8 res = m_af.b.h + arg;
	m_af.b.l = SZHVC_add[(m_af.b.h << 8) | res];
	m_af.b.h = res;
}


inline void z80lle_device::alu_and(u8 arg)
{
	u8 res = m_af.b.h & arg;
	m_af.b.l = SZP[res] | HF;
	m_af.b.h = res;
}


inline void z80lle_device::alu_bit(u8 arg)
{
	if ((m_ir & 0x07) == 0x06)
	{
		m_af.b.l = (m_af.b.l & CF) | HF | (SZ_BIT[arg & (1 << ((m_ir >> 3) & 0x07))] & ~(YF | XF)) | (m_wz.b.h & (YF | XF));
	}
	else
	{
		m_af.b.l = (m_af.b.l & CF) | HF | (SZ_BIT[arg & (1 << ((m_ir >> 3) & 0x07))] & ~(YF | XF)) | (arg & (YF | XF));
	}
}


inline void z80lle_device::alu_cp(u8 arg)
{
	// Flag handling is slightly different from SUB
	u8 res = m_af.b.h - arg;
	m_af.b.l = (SZHVC_sub[(m_af.b.h << 8) | res] & ~(YF | XF)) | (arg & (YF | XF));
}


inline u8 z80lle_device::alu_dec(u8 arg)
{
	u8 res = arg - 1;
	m_af.b.l = (m_af.b.l & CF) | SZHV_dec[res];
	return res;
}


inline u8 z80lle_device::alu_inc(u8 arg)
{
	u8 res = arg + 1;
	m_af.b.l = (m_af.b.l & CF) | SZHV_inc[res];
	return res;
}


inline void z80lle_device::alu_or(u8 arg)
{
	u8 res = m_af.b.h | arg;
	m_af.b.l = SZP[res];
	m_af.b.h = res;
}


inline u8 z80lle_device::alu_res(u8 arg)
{
	return arg & ~(1 << ((m_ir >> 3) & 0x07));
}


inline u8 z80lle_device::alu_rl(u8 arg)
{
	u8 res = (arg << 1) | (m_af.b.l & CF);
	m_af.b.l = SZP[res] | ((arg & 0x80) ? CF : 0);
	return res;
}


inline u8 z80lle_device::alu_rlc(u8 arg)
{
	u8 res = (arg << 1) | (arg >> 7);
	m_af.b.l = SZP[res] | ((arg & 0x80) ? CF : 0);
	return res;
}


inline u8 z80lle_device::alu_rr(u8 arg)
{
	u8 res = (arg >> 1) | (m_af.b.l << 7);
	m_af.b.l = SZP[res] | ((arg & 0x01) ? CF : 0);
	return res;
}


inline u8 z80lle_device::alu_rrc(u8 arg)
{
	u8 res = (arg >> 1) | (arg << 7);
	m_af.b.l = SZP[res] | ((arg & 0x01) ? CF : 0);
	return res;
}


inline void z80lle_device::alu_sbc(u8 arg)
{
	u8 res = m_af.b.h - arg - (m_af.b.l & CF);
	m_af.b.l = SZHVC_sub[((m_af.b.l & CF) << 16) | (m_af.b.h << 8) | res];
	m_af.b.h = res;
}


inline u8 z80lle_device::alu_set(u8 arg)
{
	return arg | (1 << ((m_ir >> 3) & 0x07));
}


inline u8 z80lle_device::alu_sla(u8 arg)
{
	u8 res = arg << 1;
	m_af.b.l = SZP[res] | ((arg & 0x80) ? CF : 0);
	return res;
}


inline u8 z80lle_device::alu_sll(u8 arg)
{
	u8 res = (arg << 1) | 0x01;
	m_af.b.l = SZP[res] | ((arg & 0x80) ? CF : 0);
	return res;
}


inline u8 z80lle_device::alu_sra(u8 arg)
{
	u8 res = (arg >> 1) | (arg & 0x80);
	m_af.b.l = SZP[res] | ((arg & 0x01) ? CF : 0);
	return res;
}


inline u8 z80lle_device::alu_srl(u8 arg)
{
	u8 res = arg >> 1;
	m_af.b.l = SZP[res] | ((arg & 0x01) ? CF : 0);
	return res;
}


inline void z80lle_device::alu_sub(u8 arg)
{
	u8 res = m_af.b.h - arg;
	m_af.b.l = SZHVC_sub[(m_af.b.h << 8) | res];
	m_af.b.h = res;
}


inline void z80lle_device::alu_xor(u8 arg)
{
	u8 res = m_af.b.h ^ arg;
	m_af.b.l = SZP[res];
	m_af.b.h = res;
}


inline void z80lle_device::db_r16h()
{
	switch (m_ir & 0x30)
	{
	case 0x00:
		m_bc.b.h = m_data_bus;
		break;
	case 0x10:
		m_de.b.h = m_data_bus;
		break;
	case 0x20:
		m_hl_index[m_hl_offset].b.h = m_data_bus;
		break;
	case 0x30:
		if (m_ir & 0x80)
			m_af.b.h = m_data_bus;
		else
			m_sp.b.h = m_data_bus;
		break;
	}
}


inline void z80lle_device::db_r16l()
{
	switch (m_ir & 0x30)
	{
	case 0x00:
		m_bc.b.l = m_data_bus;
		break;
	case 0x10:
		m_de.b.l = m_data_bus;
		break;
	case 0x20:
		m_hl_index[m_hl_offset].b.l = m_data_bus;
		break;
	case 0x30:
		if (m_ir & 0x80)
			m_af.b.l = m_data_bus;
		else
			m_sp.b.l = m_data_bus;
		break;
	}
}


inline void z80lle_device::db_regd()
{
	switch (m_ir & 0x38)
	{
	case 0x00:
		m_bc.b.h = m_data_bus;
		break;
	case 0x08:
		m_bc.b.l = m_data_bus;
		break;
	case 0x10:
		m_de.b.h = m_data_bus;
		break;
	case 0x18:
		m_de.b.l = m_data_bus;
		break;
	case 0x20:
		m_hl_index[m_hl_offset].b.h = m_data_bus;
		break;
	case 0x28:
		m_hl_index[m_hl_offset].b.l = m_data_bus;
		break;
	case 0x30:
		fatalerror("DB_REGD: illegal register reference 0x30\n");
		break;
	case 0x38:
		m_af.b.h = m_data_bus;
		break;
	}
}


inline void z80lle_device::db_regd0()
{
	switch (m_ir & 0x38)
	{
	case 0x00:
		m_bc.b.h = m_data_bus;
		break;
	case 0x08:
		m_bc.b.l = m_data_bus;
		break;
	case 0x10:
		m_de.b.h = m_data_bus;
		break;
	case 0x18:
		m_de.b.l = m_data_bus;
		break;
	case 0x20:
		m_hl_index[HL_OFFSET].b.h = m_data_bus;
		break;
	case 0x28:
		m_hl_index[HL_OFFSET].b.l = m_data_bus;
		break;
	case 0x30:
		fatalerror("DB_REGD0: illegal register reference 0x30\n");
		break;
	case 0x38:
		m_af.b.h = m_data_bus;
		break;
	}
}


inline void z80lle_device::db_regd_input()
{
	switch (m_ir & 0x38)
	{
	case 0x00:
		m_bc.b.h = m_data_bus;
		break;
	case 0x08:
		m_bc.b.l = m_data_bus;
		break;
	case 0x10:
		m_de.b.h = m_data_bus;
		break;
	case 0x18:
		m_de.b.l = m_data_bus;
		break;
	case 0x20:
		m_hl_index[m_hl_offset].b.h = m_data_bus;
		break;
	case 0x28:
		m_hl_index[m_hl_offset].b.l = m_data_bus;
		break;
	case 0x30:
		// the byte read is not stored in a register, only the flags are updated.
		break;
	case 0x38:
		m_af.b.h = m_data_bus;
		break;
	}
	m_af.b.l = (m_af.b.l & CF) | SZP[m_data_bus];
}


inline void z80lle_device::input_s(u16 address)
{
	m_address_bus = address;
	m_address_bus_cb(m_address_bus);
	m_icount -= 1;
	set_iorq();
	set_rd();
	m_check_wait = true;
	m_icount -= 3;
}


inline void z80lle_device::output_s(u16 address)
{
	m_address_bus = address;
	m_address_bus_cb(m_address_bus);
	m_icount -= 1;
	set_iorq();
	set_wr();
	m_check_wait = true;
	m_icount -= 3;
}


inline void z80lle_device::r16h_db()
{
	switch (m_ir & 0x30)
	{
	case 0x00:
		m_data_bus = m_bc.b.h;
		break;
	case 0x10:
		m_data_bus = m_de.b.h;
		break;
	case 0x20:
		m_data_bus = m_hl_index[m_hl_offset].b.h;
		break;
	case 0x30:
		if (m_ir & 0x80)
			m_data_bus = m_af.b.h;
		else
			m_data_bus = m_sp.b.h;
		break;
	}
}


inline void z80lle_device::r16l_db()
{
	switch (m_ir & 0x30)
	{
	case 0x00:
		m_data_bus = m_bc.b.l;
		break;
	case 0x10:
		m_data_bus = m_de.b.l;
		break;
	case 0x20:
		m_data_bus = m_hl_index[m_hl_offset].b.l;
		break;
	case 0x30:
		if (m_ir & 0x80)
			m_data_bus = m_af.b.l;
		else
			m_data_bus = m_sp.b.l;
		break;
	}
}


inline void z80lle_device::read_op_s()
{
	m_address_bus = m_pc.w.l;
	m_address_bus_cb(m_address_bus);
	set_m1();
	m_icount -= 1;
	set_mreq();
	set_rd();
	m_icount -= 1;
	m_icount -= m_m1_wait_states;
	m_opcode_read = true;
	m_check_wait = true;
}


inline void z80lle_device::read_s(u16 address)
{
	m_address_bus = address;
	m_address_bus_cb(m_address_bus);
	m_icount -= 1;
	set_mreq();
	set_rd();
	m_icount -= 2;
	m_check_wait = true;
}


inline u8 z80lle_device::regd()
{
	switch (m_ir & 0x38)
	{
	case 0x00:
		return m_bc.b.h;
	case 0x08:
		return m_bc.b.l;
	case 0x10:
		return m_de.b.h;
	case 0x18:
		return m_de.b.l;
	case 0x20:
		return m_hl_index[m_hl_offset].b.h;
	case 0x28:
		return m_hl_index[m_hl_offset].b.l;
	case 0x30:
		fatalerror("REGD: illegal register reference 0x30\n");
		break;
	case 0x38:
		return m_af.b.h;
	}
	return 0;
}


inline void z80lle_device::regd(u8 data)
{
	switch (m_ir & 0x38)
	{
	case 0x00:
		m_bc.b.h = data;
		break;
	case 0x08:
		m_bc.b.l = data;
		break;
	case 0x10:
		m_de.b.h = data;
		break;
	case 0x18:
		m_de.b.l = data;
		break;
	case 0x20:
		m_hl_index[m_hl_offset].b.h = data;
		break;
	case 0x28:
		m_hl_index[m_hl_offset].b.l = data;
		break;
	case 0x30:
		fatalerror("REGD: illegal register reference 0x30\n");
		break;
	case 0x38:
		m_af.b.h = data;
		break;
	}
}


inline u8 z80lle_device::regs()
{
	switch (m_ir & 0x07)
	{
	case 0x00:
		return m_bc.b.h;
	case 0x01:
		return m_bc.b.l;
	case 0x02:
		return m_de.b.h;
	case 0x03:
		return m_de.b.l;
	case 0x04:
		return m_hl_index[m_hl_offset].b.h;
	case 0x05:
		return m_hl_index[m_hl_offset].b.l;
	case 0x06:
		fatalerror("REGS: illegal register reference 0x06\n");
		break;
	case 0x07:
		return m_af.b.h;
	}
	return 0;
}


inline void z80lle_device::regs(u8 data)
{
	switch (m_ir & 0x07)
	{
	case 0x00:
		m_bc.b.h = data;
		break;
	case 0x01:
		m_bc.b.l = data;
		break;
	case 0x02:
		m_de.b.h = data;
		break;
	case 0x03:
		m_de.b.l = data;
		break;
	case 0x04:
		m_hl_index[m_hl_offset].b.h = data;
		break;
	case 0x05:
		m_hl_index[m_hl_offset].b.l = data;
		break;
	case 0x06:
		fatalerror("REGS: illegal register reference 0x06\n");
		break;
	case 0x07:
		m_af.b.h = data;
		break;
	}
}


// store in base registers only, not index register
inline void z80lle_device::regs0(u8 data)
{
	switch (m_ir & 0x07)
	{
	case 0x00:
		m_bc.b.h = data;
		break;
	case 0x01:
		m_bc.b.l = data;
		break;
	case 0x02:
		m_de.b.h = data;
		break;
	case 0x03:
		m_de.b.l = data;
		break;
	case 0x04:
		m_hl_index[HL_OFFSET].b.h = data;
		break;
	case 0x05:
		m_hl_index[HL_OFFSET].b.l = data;
		break;
	case 0x06:
		// no side effect
		break;
	case 0x07:
		m_af.b.h = data;
		break;
	}
}


inline void z80lle_device::regs0_db()
{
	switch (m_ir & 0x07)
	{
	case 0x00:
		m_data_bus = m_bc.b.h;
		break;
	case 0x01:
		m_data_bus = m_bc.b.l;
		break;
	case 0x02:
		m_data_bus = m_de.b.h;
		break;
	case 0x03:
		m_data_bus = m_de.b.l;
		break;
	case 0x04:
		m_data_bus = m_hl_index[HL_OFFSET].b.h;
		break;
	case 0x05:
		m_data_bus = m_hl_index[HL_OFFSET].b.l;
		break;
	case 0x06:
		fatalerror("REGS0_DB: illegal register reference 0x06\n");
		break;
	case 0x07:
		m_data_bus = m_af.b.h;
		break;
	}
}


inline void z80lle_device::write_s()
{
	set_mreq();
	m_icount -= 1;
	set_wr();
	m_icount -= 1;
	m_check_wait = true;
}


inline void z80lle_device::write_s(u16 address)
{
	m_address_bus = address;
	m_address_bus_cb(m_address_bus);
	m_icount -= 1;
	write_s();
}


inline void z80lle_device::decode()
{
	m_instruction = m_instruction_offset | m_ir;
	m_instruction_step = 0;
	if (m_instruction_offset != CB_OFFSET && m_instruction_offset != FDCB_OFFSET)
	{
		switch (m_ir) {
		case 0xcb:
			if (m_hl_offset == HL_OFFSET)
			{
				m_instruction_offset = CB_OFFSET;
				m_instruction = M1;
			}
			else
			{
				m_instruction_offset = FDCB_OFFSET;
				m_instruction = DD_FD_CB;
			}
			break;
		case 0xdd:
			m_hl_offset = IX_OFFSET;
			m_instruction_offset = FD_OFFSET;
			m_instruction = M1;
			break;
		case 0xed:
			m_hl_offset = HL_OFFSET;
			m_instruction_offset = ED_OFFSET;
			m_instruction = M1;
			break;
		case 0xfd:
			m_hl_offset = IY_OFFSET;
			m_instruction_offset = FD_OFFSET;
			m_instruction = M1;
			break;
		}
	}
}


inline void z80lle_device::check_interrupts()
{
	// check for interrupts before each instruction
	if (m_nmi_pending)
	{
		leave_halt();
		m_iff1 = 0;
		m_instruction = TAKE_NMI;
		m_nmi_pending = false;
	}
	else if (m_irq_state != CLEAR_LINE && m_iff1 && !m_after_ei)
	{
		// check if processor was halted
		leave_halt();

		// clear both interrupt flip flops
		m_iff1 = m_iff2 = 0;

		if (m_im == 2)
		{
			m_instruction = TAKE_IRQ_2;
		}
		else if (m_im == 1)
		{
			// Interrupt mode 1. RST 38h
			m_instruction = TAKE_IRQ_1;
		}
		else
		{
			m_instruction = TAKE_IRQ_0;
		}
	}

	m_after_ei = false;
	m_after_ldair = false;
}

//////////////////////////////////////////////////////////////////////////////
// Processor initialization
//////////////////////////////////////////////////////////////////////////////
void z80lle_device::setup_flag_tables()
{
	if (!tables_initialised)
	{
		u8 *padd = &SZHVC_add[  0*256];
		u8 *padc = &SZHVC_add[256*256];
		u8 *psub = &SZHVC_sub[  0*256];
		u8 *psbc = &SZHVC_sub[256*256];
		for (int oldval = 0; oldval < 256; oldval++)
		{
			for (int newval = 0; newval < 256; newval++)
			{
				// add or adc w/o carry set
				int val = newval - oldval;
				*padd = (newval) ? ((newval & 0x80) ? SF : 0) : ZF;
				*padd |= (newval & (YF | XF));  // undocumented flag bits 5+3
				if ((newval & 0x0f) < (oldval & 0x0f))
					*padd |= HF;
				if (newval < oldval)
					*padd |= CF;
				if ((val^oldval^0x80) & (val^newval) & 0x80)
					*padd |= VF;
				padd++;

				// adc with carry set
				val = newval - oldval - 1;
				*padc = (newval) ? ((newval & 0x80) ? SF : 0) : ZF;
				*padc |= (newval & (YF | XF));  // undocumented flag bits 5+3
				if ((newval & 0x0f) <= (oldval & 0x0f))
					*padc |= HF;
				if (newval <= oldval )
					*padc |= CF;
				if ((val^oldval^0x80) & (val^newval) & 0x80)
					*padc |= VF;
				padc++;

				// cp, sub or sbc w/o carry set
				val = oldval - newval;
				*psub = NF | ((newval) ? ((newval & 0x80) ? SF : 0) : ZF);
				*psub |= (newval & (YF | XF));  // undocumented flag bits 5+3
				if ((newval & 0x0f) > (oldval & 0x0f))
					*psub |= HF;
				if (newval > oldval)
					*psub |= CF;
				if ((val^oldval) & (oldval^newval) & 0x80)
					*psub |= VF;
				psub++;

				// sbc with carry set
				val = oldval - newval - 1;
				*psbc = NF | ((newval) ? ((newval & 0x80) ? SF : 0) : ZF);
				*psbc |= (newval & (YF | XF));  // undocumented flag bits 5+3
				if ((newval & 0x0f) >= (oldval & 0x0f))
					*psbc |= HF;
				if (newval >= oldval)
					*psbc |= CF;
				if ((val^oldval) & (oldval^newval) & 0x80)
					*psbc |= VF;
				psbc++;
			}
		}

		for (int i = 0; i < 256; i++)
		{
			int p = 0;
			if (i & 0x01) ++p;
			if (i & 0x02) ++p;
			if (i & 0x04) ++p;
			if (i & 0x08) ++p;
			if (i & 0x10) ++p;
			if (i & 0x20) ++p;
			if (i & 0x40) ++p;
			if (i & 0x80) ++p;
			SZ[i] = i ? i & SF : ZF;
			SZ[i] |= (i & (YF | XF));       // undocumented flag bits 5+3
			SZ_BIT[i] = i ? i & SF : ZF | PF;
			SZ_BIT[i] |= (i & (YF | XF));   // undocumented flag bits 5+3
			SZP[i] = SZ[i] | ((p & 1) ? 0 : PF);
			SZHV_inc[i] = SZ[i];
			if (i == 0x80)
				SZHV_inc[i] |= VF;
			if ((i & 0x0f) == 0x00)
				SZHV_inc[i] |= HF;
			SZHV_dec[i] = SZ[i] | NF;
			if (i == 0x7f)
				SZHV_dec[i] |= VF;
			if ((i & 0x0f) == 0x0f)
				SZHV_dec[i] |= HF;
		}

		tables_initialised = true;
	}
}


void z80lle_device::device_start()
{
	setup_flag_tables();

	save_item(NAME(m_prvpc.w.l));
	save_item(NAME(m_pc.w.l));
	save_item(NAME(m_sp.w.l));
	save_item(NAME(m_af.w.l));
	save_item(NAME(m_bc.w.l));
	save_item(NAME(m_de.w.l));
	save_item(NAME(m_hl_index[HL_OFFSET].w.l));
	save_item(NAME(m_hl_index[IX_OFFSET].w.l));
	save_item(NAME(m_hl_index[IY_OFFSET].w.l));
	save_item(NAME(m_wz.w.l));
	save_item(NAME(m_af2.w.l));
	save_item(NAME(m_bc2.w.l));
	save_item(NAME(m_de2.w.l));
	save_item(NAME(m_hl2.w.l));
	save_item(NAME(m_r));
	save_item(NAME(m_r2));
	save_item(NAME(m_iff1));
	save_item(NAME(m_iff2));
	save_item(NAME(m_halt));
	save_item(NAME(m_im));
	save_item(NAME(m_i));
	save_item(NAME(m_nmi_state));
	save_item(NAME(m_nmi_pending));
	save_item(NAME(m_irq_state));
	save_item(NAME(m_wait_state));
	save_item(NAME(m_busrq_state));
	save_item(NAME(m_after_ei));
	save_item(NAME(m_after_ldair));
	save_item(NAME(m_hl_offset));
	save_item(NAME(m_address_bus));
	save_item(NAME(m_data_bus));
	save_item(NAME(m_instruction_step));
	save_item(NAME(m_instruction_offset));
	save_item(NAME(m_instruction));
	save_item(NAME(m_ir));
	save_item(NAME(m_mreq));
	save_item(NAME(m_iorq));
	save_item(NAME(m_rd));
	save_item(NAME(m_wr));
	save_item(NAME(m_m1));
	save_item(NAME(m_opcode_read));

	// Reset registers to their initial values
	m_prvpc.d = 0;
	m_pc.d = 0;
	m_sp.d = 0xffff;
	m_af.d = 0xfffd;
	m_bc.d = 0xffff;
	m_de.d = 0xffff;
	m_hl_index[HL_OFFSET].d = 0xffff;
	m_hl_index[IX_OFFSET].d = 0xffff;
	m_hl_index[IY_OFFSET].d = 0xffff;
	m_wz.d = 0xffff;
	m_af2.d = 0xffff;
	m_bc2.d = 0xffff;
	m_de2.d = 0xffff;
	m_hl2.d = 0xffff;
	m_r = 0;
	m_r2 = 0;
	m_iff1 = 0;
	m_iff2 = 0;
	m_halt = 0;
	m_im = 0;
	m_i = 0;
	m_nmi_state = 0;
	m_nmi_pending = 0;
	m_irq_state = 0;
	m_wait_state = 1;   // active low
	m_busrq_state = 0;
	m_after_ei = false;
	m_after_ldair = false;
	m_mreq = false;
	m_iorq = false;
	m_rd = false;
	m_wr = false;
	m_m1 = false;
	m_opcode_read = false;
	m_reset = true;

	space(AS_PROGRAM).cache(m_args);
	space(has_space(AS_OPCODES) ? AS_OPCODES : AS_PROGRAM).cache(m_opcodes);
	space(AS_PROGRAM).specific(m_data);
	space(AS_IO).specific(m_io);

	m_hl_index[IX_OFFSET].w.l = m_hl_index[IY_OFFSET].w.l = 0xffff; // IX and IY are FFFF after a reset!
	m_af.b.l = ZF;           // Zero flag is set

	// set up the state table
	state_add(STATE_GENPC,     "PC",        m_pc.w.l).callimport();
	state_add(STATE_GENPCBASE, "CURPC",     m_prvpc.w.l).callimport().noshow();
	state_add(Z80LLE_SP,       "SP",        m_sp.w.l);
	state_add(STATE_GENFLAGS,  "GENFLAGS",  m_af.b.l).noshow().formatstr("%8s");
	state_add(Z80LLE_A,        "A",         m_af.b.h).noshow();
	state_add(Z80LLE_B,        "B",         m_bc.b.h).noshow();
	state_add(Z80LLE_C,        "C",         m_bc.b.l).noshow();
	state_add(Z80LLE_D,        "D",         m_de.b.h).noshow();
	state_add(Z80LLE_E,        "E",         m_de.b.l).noshow();
	state_add(Z80LLE_H,        "H",         m_hl_index[HL_OFFSET].b.h).noshow();
	state_add(Z80LLE_L,        "L",         m_hl_index[HL_OFFSET].b.l).noshow();
	state_add(Z80LLE_AF,       "AF",        m_af.w.l);
	state_add(Z80LLE_BC,       "BC",        m_bc.w.l);
	state_add(Z80LLE_DE,       "DE",        m_de.w.l);
	state_add(Z80LLE_HL,       "HL",        m_hl_index[HL_OFFSET].w.l);
	state_add(Z80LLE_IX,       "IX",        m_hl_index[IX_OFFSET].w.l);
	state_add(Z80LLE_IY,       "IY",        m_hl_index[IY_OFFSET].w.l);
	state_add(Z80LLE_AF2,      "AF2",       m_af2.w.l);
	state_add(Z80LLE_BC2,      "BC2",       m_bc2.w.l);
	state_add(Z80LLE_DE2,      "DE2",       m_de2.w.l);
	state_add(Z80LLE_HL2,      "HL2",       m_hl2.w.l);
	state_add(Z80LLE_WZ,       "WZ",        m_wz.w.l);
	state_add(Z80LLE_R,        "R",         m_rtemp).callimport().callexport();
	state_add(Z80LLE_I,        "I",         m_i);
	state_add(Z80LLE_IM,       "IM",        m_im).mask(0x3);
	state_add(Z80LLE_IFF1,     "IFF1",      m_iff1).mask(0x1);
	state_add(Z80LLE_IFF2,     "IFF2",      m_iff2).mask(0x1);
	state_add(Z80LLE_HALT,     "HALT",      m_halt).mask(0x1);

	// set our instruction counter
	set_icountptr(m_icount);

	m_irqack_cb.resolve_safe();
	m_refresh_cb.resolve_safe();
	m_halt_cb.resolve_safe();
	m_mreq_cb.resolve_safe();
	m_iorq_cb.resolve_safe();
	m_rd_cb.resolve_safe();
	m_wr_cb.resolve_safe();
	m_m1_cb.resolve_safe();
	m_address_bus_cb.resolve_safe();
}


/////////////////////////////////////////////////////////////////////////////
// Do a reset
/////////////////////////////////////////////////////////////////////////////
void z80lle_device::device_reset()
{
	m_pc.d = 0x0000;
	m_i = 0;
	m_r = 0;
	m_r2 = 0;
	m_nmi_pending = false;
	m_after_ei = false;
	m_after_ldair = false;
	m_iff1 = 0;
	m_iff2 = 0;
	m_reset = true;

	m_wz.d = m_pc.d;

	m_instruction = M1;
	m_instruction_step = 0;
	m_instruction_offset = 0;

	m_hl_offset = HL_OFFSET;
	m_check_wait = false;
	m_opcode_read = false;
	m_in_irq_acknowledge = false;

	clear_mreq();
	clear_iorq();
	clear_rd();
	clear_wr();
	clear_m1();
}


/////////////////////////////////////////////////////////////////////////////
// Execute 'cycles' T-states.
/////////////////////////////////////////////////////////////////////////////
void z80lle_device::execute_run()
{
	if (m_reset)
	{
		// 2-3 cycle delay after a reset
		m_icount -= 3;
		m_reset = false;
	}
	do
	{
		if (m_check_wait) {
			if (!m_wait_state) {
				// Wait for an external source to release the WAIT input
				m_icount = 0;
				return;
			}
			m_check_wait = false;
		}

		// Check if we should perform a memory or io read or write
		// TODO Move this inside the m_check_wait check?
		if (m_mreq) {
			if (m_rd) {
				if (m_m1 || m_opcode_read) {
					if (m_in_irq_acknowledge)
					{
						m_data_bus = m_irq_vector >> 24;
						m_irq_vector <<= 8;
					}
					else
						m_data_bus = m_opcodes.read_byte(m_address_bus);
					m_ir = m_data_bus;
					if (m_m1) {
						clear_m1();
					}
					m_opcode_read = false;
				} else {
					if (m_in_irq_acknowledge)
					{
						m_data_bus = m_irq_vector >> 24;
						m_irq_vector <<= 8;
					}
					else
						m_data_bus = m_data.read_byte(m_address_bus);
				}
				clear_mreq();
				clear_rd();
			}
			if (m_wr) {
				m_data.write_byte(m_address_bus, m_data_bus);
				clear_mreq();
				clear_wr();
			}
		}
		if (m_iorq) {
			if (m_m1) {
				m_irqack_cb(true);

				// fetch the IRQ vector
				device_z80daisy_interface *intf = daisy_get_irq_device();
				m_irq_vector = (intf != nullptr) ? intf->z80daisy_irq_ack() : standard_irq_callback_member(*this, 0);

				switch (m_im)
				{
				case 0:
					if (m_irq_vector != 0)
					{
						while (!(m_irq_vector & 0xff000000))
						{
							m_irq_vector <<= 8;
						}
					}
					m_ir = m_irq_vector >> 24;
					m_in_irq_acknowledge = true;
					break;
				case 1:
					// Interrupt mode 1, RST 38H (0xff)
					m_ir = 0xff;
					break;
				case 2:
					m_irq_vector = (m_i << 8) | (m_irq_vector & 0xff);
					break;
				}
				clear_iorq();
				clear_m1();
			}
			if (m_rd) {
				m_data_bus = m_io.read_byte(m_address_bus);
				clear_iorq();
				clear_rd();
			}
			if (m_wr) {
				m_io.write_byte(m_address_bus, m_data_bus);
				clear_iorq();
				clear_wr();
			}
		}

		if (m_instruction == M1 && m_instruction_step == 0 && m_instruction_offset == 0) {
			check_interrupts();

			m_prvpc.d = m_pc.d;
			debugger_instruction_hook(m_pc.d);
		}

		// Execute steps for instruction
		u16 step = insts[m_instruction][m_instruction_step++];
		switch (step & ~END)
		{
		case UNKNOWN:
			if (step != END)
				fatalerror("Unsupported instruction %d,%02x encountered at address %04x", m_instruction_offset / 256,
						   m_ir, m_prvpc.d);
			break;
		// register A to data bus, takes 0 cycles
		case A_DB:
			m_data_bus = m_af.b.h;
			break;
		// register A to W, takes 0 cycles
		case A_W:
			m_wz.b.h = m_af.b.h;
			break;
		// Perform ADC operation on A and data bus, takes 0 cycles
		case ADC_DB:
			alu_adc(m_data_bus);
			break;
		// Perform ADC operation on A and 8 bit register, takes 0 cycles
		case ADC_R8:
			alu_adc(regs());
			break;
		// 16bit addition with carry, takes 7 cycles
		case ADC16:
			adc16();
			break;
		// Perform ADD operation on A and data bus, takes 0 cycles
		case ADD_DB:
			alu_add(m_data_bus);
			break;
		// Perform ADD operation on A and 8 bit register, takes 0 cycles
		case ADD_R8:
			alu_add(regs());
			break;
		// 16bit addition, takes 7 cycles
		case ADD16:
			add16();
			break;
		// Perform AND operation on A and data bus, takes 0 cycles
		case AND_DB:
			alu_and(m_data_bus);
			break;
		// Perform AND operation on A and 8 bit register, takes 0 cycles
		case AND_R8:
			alu_and(regs());
			break;
		// Perform BIT operation on data bus, takes 1 cycle
		case BIT_DB:
			alu_bit(m_data_bus);
			m_icount -= 1;
			break;
		// Perform BIT operation on 8 bit register, takes 0 cycles
		case BIT_R8:
			alu_bit(regs());
			break;
		// Clear carry flag, takes 0 cycles
		case CCF:
			m_af.b.l = ((m_af.b.l & (SF | ZF | YF | XF | PF | CF)) | ((m_af.b.l & CF) << 4) | (m_af.b.h & (YF | XF))) ^ CF;
			break;
		// Perform CP operation on A and data bus, takes 0 cycles
		case CP_DB:
			alu_cp(m_data_bus);
			break;
		// Perform CP operation on A and 8 bit register, takes 0 cycles
		case CP_R8:
			alu_cp(regs());
			break;
		// Set flags and update pointers and counter, takes 5 cycles
		case CPD:
			{
				u8 res = m_af.b.h - m_data_bus;
				m_wz.w.l--;
				m_hl_index[m_hl_offset].w.l--;
				m_bc.w.l--;
				m_af.b.l = (m_af.b.l & CF) | (SZ[res] & ~(YF | XF)) | ((m_af.b.h ^ m_data_bus ^ res) & HF) | NF;
				if (m_af.b.l & HF)
					res -= 1;
				if (res & 0x02)
					m_af.b.l |= YF; /* bit 1 -> flag 5 */
				if (res & 0x08)
					m_af.b.l |= XF; /* bit 3 -> flag 3 */
				if (m_bc.w.l)
					m_af.b.l |= VF;
			}
			break;
		// Set flags and update pointers and counter, takes 5 cycles
		case CPI:
			{
				u8 res = m_af.b.h - m_data_bus;
				m_wz.w.l++;
				m_hl_index[m_hl_offset].w.l++;
				m_bc.w.l--;
				m_af.b.l = (m_af.b.l & CF) | (SZ[res] & ~(YF | XF)) | ((m_af.b.h ^ m_data_bus ^ res) & HF) | NF;
				if (m_af.b.l & HF)
					res -= 1;
				if (res & 0x02)
					m_af.b.l |= YF; /* bit 1 -> flag 5 */
				if (res & 0x08)
					m_af.b.l |= XF; /* bit 3 -> flag 3 */
				if (m_bc.w.l)
					m_af.b.l |= VF;
			}
			break;
		// Complement A, takes 0 cycles
		case CPL:
			m_af.b.h ^= 0xff;
			m_af.b.l = (m_af.b.l & (SF | ZF | PF | CF)) | HF | NF | (m_af.b.h & (YF | XF));
			break;
		// Decimal adjust, takes 0 cycles
		case DAA:
			{
				u8 res = m_af.b.h;
				if (m_af.b.l & NF)
				{
					if ((m_af.b.l & HF) | ((m_af.b.h & 0xf) > 9))
						res -= 6;
					if ((m_af.b.l & CF) | (m_af.b.h > 0x99))
						res -= 0x60;
				}
				else
				{
					if ((m_af.b.l & HF) | ((m_af.b.h & 0xf) > 9))
						res += 6;
					if ((m_af.b.l & CF) | (m_af.b.h > 0x99))
						res += 0x60;
				}
				m_af.b.l = (m_af.b.l & (CF | NF)) | (m_af.b.h > 0x99) | ((m_af.b.h ^ res) & HF) | SZP[res];
				m_af.b.h = res;
			}
			break;
		// Store data bus in A, takes 0 cycles
		case DB_A:
			m_af.b.h = m_data_bus;
			break;
		// Store data bus in high 8 bits of 16 bit register, takes 0 cycles
		case DB_R16H:
			db_r16h();
			break;
		// Store data bus in low 8 bits of 16 bit register, takes 0 cycles
		case DB_R16L:
			db_r16l();
			break;
		// Store data bus in low 8 bits of 16 bit register, put PC on address bus, increment PC, assert MREQ and RD signals for read cycle, takes 3 cycles
		case DB_R16L_READ_S_PC:
			db_r16l();
			read_s(m_pc.w.l);
			m_pc.w.l++;
			break;
		// Store data bus in low 8 bits of 16 bit register, put SP on address bus, increment SP, assert MREQ and RD signals for read cycle, takes 3 cycles
		case DB_R16L_READ_S_SP_INC:
			db_r16l();
			read_s(m_sp.w.l);
			m_sp.w.l += 1;
			break;
		// Store data bus in low 8 bits of 16 bit register, put WZ on address bus, assert MREQ and RD signals for read cycle, takes 3 cycles
		case DB_R16L_READ_S_WZ:
			db_r16l();
			read_s(m_wz.w.l);
			break;
		// Store data bus in 8bit destination register (bits ..xxx...), takes 0 cycles
		case DB_REGD:
			db_regd();
			break;
		// Store data bus from input operation in 8bit register (bits ..xxx...), takes 0 cycles
		case DB_REGD_INPUT:
			db_regd_input();
			break;
		// Store data bus in 8bit register (bits ..xxx...), not to index registers, takes 0 cycles
		case DB_REGD0:
			db_regd0();
			break;
		// Store data bus in W, takes 0 cycles
		case DB_W:
			m_wz.b.h = m_data_bus;
			break;
		// Store data bus in W, check condition for CALL, put PC 8 high bits on data bus, decrement SP, put SP on address bus, assert MREQ and WR signals for write, takes 4 cycle when condition is true
		case DB_W_CALL_COND:
			m_wz.b.h = m_data_bus;
			if ((m_af.b.l & jp_conditions[((m_ir >> 3) & 0x07)][0]) == jp_conditions[((m_ir >> 3) & 0x07)][1])
			{
				m_icount -= 1;
				m_data_bus = m_pc.b.h;
				m_sp.w.l -= 1;
				write_s(m_sp.w.l);
			}
			else
			{
				end_instruction();
			}
			break;
		// Store data bus in W, check condition for JP and perform jump, takes 0 cycles
		case DB_W_JP_COND:
			m_wz.b.h = m_data_bus;
			if ((m_af.b.l & jp_conditions[((m_ir >> 3) & 0x07)][0]) == jp_conditions[((m_ir >> 3) & 0x07)][1])
			{
				m_pc.w.l = m_wz.w.l;
			}
			break;
		// Store data bus in W, register A to data bus, put WZ on address bus, increment WZ, assert MREQ and WR signals for write, store A in W, takes 3 cycles
		case DB_W_A_DB_WRITE_S_WZ_INC:
			m_wz.b.h = m_data_bus;
			m_data_bus = m_af.b.h;
			write_s(m_wz.w.l);
			m_wz.w.l++;
			m_wz.b.h = m_af.b.h;
			break;
		// Store data bus in W, register L to data bus, put WZ on address bus, increment WZ, assert MREQ and WR signals for write, takes 3 cycles
		case DB_W_L_DB_WRITE_S_WZ_INC:
			m_wz.b.h = m_data_bus;
			m_data_bus = m_hl_index[m_hl_offset].b.l;
			write_s(m_wz.w.l);
			m_wz.w.l++;
			break;
		// Store data bus in W, put WZ on address bus, increment WZ, assert MREQ and RD signals for read cycle, takes 3 cycles
		case DB_W_READ_S_WZ_INC:
			m_wz.b.h = m_data_bus;
			read_s(m_wz.w.l);
			m_wz.w.l++;
			break;
		// Store data bus in W, store contents of WZ in PC, takes 0 cycles
		case DB_W_WZ_PC:
			m_wz.b.h = m_data_bus;
			m_pc.w.l = m_wz.w.l;
			break;
		// Store data bus in Z, takes 0 cycles
		case DB_Z:
			m_wz.b.l = m_data_bus;
			break;
		// Store data bus in Z, put PC on address bus, increment PC, assert MREQ and RD signals for read cycle, takes 3 cycles
		case DB_Z_READ_S_PC:
			m_wz.b.l = m_data_bus;
			read_s(m_pc.w.l);
			m_pc.w.l++;
			break;
		// Store data bus in Z, put SP on address bus, assert MREQ and RD signals for read cycle, takes 3 cycles
		case DB_Z_READ_S_SP:
			m_wz.b.l = m_data_bus;
			read_s(m_sp.w.l);
			m_sp.w.l += 1;
			break;
		// Store data bus in Z, put SP on address bus, increment SP, assert MREQ and RD signals for read cycle, takes 3 cycles
		case DB_Z_READ_S_SP_INC:
			m_wz.b.l = m_data_bus;
			read_s(m_sp.w.l);
			m_sp.w.l += 1;
			break;
		// Store data bus in Z, put vector address on address bus, assert MREQ and RD signals for read cycle, takes 3 cycles
		case DB_Z_READ_S_VEC:
			m_wz.b.l = m_data_bus;
			read_s(m_irq_vector);
			break;
		// Perform DEC operation on data bus, assert MREQ and WR signals for write, takes 4 cycles
		case DEC_DB_WRITE_S:
			m_data_bus = alu_dec(m_data_bus);
			m_icount -= 2;
			write_s();
			break;
		// Decrement an 8 bit register, takes 0 cycles
		case DEC_R8:
			regd(alu_dec(regd()));
			break;
		// Decrement a 16 bit register, takes 2 cycles
		case DEC_R16:
			switch (m_ir & 0x30)
			{
			case 0x00:
				m_bc.w.l--;
				break;
			case 0x10:
				m_de.w.l--;
				break;
			case 0x20:
				m_hl_index[m_hl_offset].w.l--;
				break;
			case 0x30:
				m_sp.w.l--;
				break;
			}
			m_icount -= 2;
			break;
		// Decode instruction, takes 2 cycles
		case DECODE_X2:
			decode();
			m_icount -= 2;
			break;
		// Reset interrupt flip flops, takes 0 cycles
		case DI:
			m_iff1 = m_iff2 = 0;
			break;
		// Calculate IX/IY displacement into WZ, takes 2 cycles (in DD CB xx II instructions)
		case DISP_WZ2:
			m_wz.w.l = m_hl_index[m_hl_offset].w.l + (s8) m_data_bus;
			m_icount -= 2;
			break;
		// Calculate IX/IY displacement into WZ, assert MREQ and signals for opcode read as part of DD/FD CB dd xx instructions, takes 3 cycles
		case DISP_WZ2_READ_OP2_S:
			m_wz.w.l = m_hl_index[m_hl_offset].w.l + (s8) m_data_bus;
			// This is a regular read without M1 but the result ends up in the instruction register (for DDCB / FDCB instructions)
			m_opcode_read = true;
			read_s(m_pc.w.l);
			m_pc.w.l++;
			break;
		// Calculate IX/IY displacement into WZ, takes 5 cycles (in DD xx instructions)
		case DISP_WZ5:
			m_wz.w.l = m_hl_index[m_hl_offset].w.l + (s8) m_data_bus;
			m_icount -= 5;
			break;
		// Calculate IX/IY displacement into WZ, put WZ on address bus, assert MREQ and RD signals for read cycle, takes 8 cycles
		case DISP_WZ5_READ_S_WZ:
			m_wz.w.l = m_hl_index[m_hl_offset].w.l + (s8) m_data_bus;
			m_icount -= 5;
			read_s(m_wz.w.l);
			break;
		// Decrement B and jump when not zero, takes 5 cycles when branch taken
		case DJNZ:
			m_bc.b.h--;
			if (m_bc.b.h)
			{
				m_wz.w.l = m_pc.w.l + (s8) m_data_bus;
				m_pc.w.l = m_wz.w.l;
				m_icount -= 5;
			}
			break;
		// Set interrupt flip flops, takes 0 cycles
		case EI:
			m_iff1 = m_iff2 = 1;
			m_after_ei = true;
			break;
		// Swap AF and AF', takes 0 cycles
		case EX_AF_AF:
			{
				PAIR tmp = m_af;
				m_af = m_af2;
				m_af2 = tmp;
			}
			break;
		// Swap DE and HL, takes 0 cycles
		case EX_DE_HL:
			{
				u16 tmp = m_de.w.l;
				m_de.w.l = m_hl_index[m_hl_offset].w.l;
				m_hl_index[m_hl_offset].w.l = tmp;
			}
			break;
		// Store data bus in W, skip 2 cycls, put high 8 bits of 16 bit register on data bus, assert MREQ and WR signals for write, takes 4 cycles
		case EX_SP_WRITE_S:
			m_wz.b.h = m_data_bus;
			m_icount -= 2;
			r16h_db();
			write_s();
			break;
		// Swap BC, DE, HL and BC2, DE2, HL2, takes 0 cycles
		case EXX:
			{
				PAIR tmp;
				tmp = m_bc;
				m_bc = m_bc2;
				m_bc2 = tmp;
				tmp = m_de;
				m_de = m_de2;
				m_de2 = tmp;
				tmp = m_hl_index[HL_OFFSET];
				m_hl_index[HL_OFFSET] = m_hl2;
				m_hl2 = tmp;
			}
			break;
		// Register H to data bus, put WZ on address bus, assert MREQ and WR signals for write, takes 3 cycles
		case H_DB_WRITE_S_WZ:
			m_data_bus = m_hl_index[m_hl_offset].b.h;
			write_s(m_wz.w.l);
			break;
		// HALT, takes 0 cycles
		case HALT:
			m_pc.w.l--;
			if (!m_halt)
			{
				m_halt = 1;
				m_halt_cb(1);
			}
			break;
		// Store HL in PC, takes 0 cycles
		case HL_PC:
			m_pc.w.l = m_hl_index[m_hl_offset].w.l;
			break;
		// Store HL in WZ, takes 0 cycles
		case HL_WZ:
			m_wz.w.l = m_hl_index[m_hl_offset].w.l;
			break;
		// Set interrupt mode, takes 0 cycles
		case IM:
			m_im = (m_ir >> 3) & 0x03;
			if (m_im)
				m_im--;
			break;
		// Perform INC operation on data bus, assert MREQ and WR signals for write, takes 4 cycles
		case INC_DB_WRITE_S:
			m_data_bus = alu_inc(m_data_bus);
			m_icount -= 2;
			write_s();
			break;
		// Increment an 8 bit register, takes 0 cycles
		case INC_R8:
			regd(alu_inc(regd()));
			break;
		// Increment a 16 bit register, takes 2 cycles
		case INC_R16:
			switch (m_ir & 0x30)
			{
			case 0x00:
				m_bc.w.l++;
				break;
			case 0x10:
				m_de.w.l++;
				break;
			case 0x20:
				m_hl_index[m_hl_offset].w.l++;
				break;
			case 0x30:
				m_sp.w.l++;
				break;
			}
			m_icount -= 2;
			break;
		// Set flags and update pointers and counter, takes 0 cycles
		case IND:
			{
				m_wz.w.l = m_bc.w.l - 1;
				m_bc.b.h--;
				m_hl_index[m_hl_offset].w.l++;
				m_af.b.l = SZ[m_bc.b.h];
				u16 t = ((m_bc.b.l - 1) & 0xff) + m_data_bus;
				if (m_data_bus & SF)
					m_af.b.l |= NF;
				if (t & 0x100)
					m_af.b.l |= HF | CF;
				m_af.b.l |= SZP[(t & 0x07) ^ m_bc.b.h] & PF;
			}
			break;
		// Set flags and update pointers and counter, takes 0 cycles
		case INI:
			{
				m_wz.w.l = m_bc.w.l + 1;
				m_bc.b.h--;
				m_hl_index[m_hl_offset].w.l--;
				m_af.b.l = SZ[m_bc.b.h];
				u16 t = ((m_bc.b.l + 1) & 0xff) + m_data_bus;
				if (m_data_bus & SF)
					m_af.b.l |= NF;
				if (t & 0x100)
					m_af.b.l |= HF | CF;
				m_af.b.l |= SZP[(t & 0x07) ^ m_bc.b.h] & PF;
			}
			break;
		// Read data bus from input, store in A, takes 0 cycles
		case INPUT_A:
			// TODO: Flags?
			m_af.b.h = m_data_bus;
			break;
		// Read data bus from input, store in 8 bit register, takes 0 cycles
		case INPUT_REGD:
			db_regd_input();
			break;
		// Put BC on address bus, assert IORQ and RD signals for input cycle, takes 4 cycles
		case INPUT_S_BC:
			input_s(m_bc.w.l);
			break;
		// Put WZ on address bus, assert IORQ and RD signals for inoput cycle, takes 4 cycles
		case INPUT_S_WZ_INC:
			input_s(m_wz.w.l);
			m_wz.w.l++;
			break;
		// Check condition for JP and perform jump, takes 0 cycles
		case JP_COND:
			if ((m_af.b.l & jp_conditions[((m_ir >> 3) & 0x07)][0]) == jp_conditions[((m_ir >> 3) & 0x07)][1])
			{
				m_pc.w.l = m_wz.w.l;
			}
			break;
		// Check condition (Z, NZ, etc) for JR and perform jump, 5 cycles when branch taken
		case JR_COND:
			if ((m_af.b.l & jr_conditions[((m_ir >> 3) & 0x07)][0]) == jr_conditions[((m_ir >> 3) & 0x07)][1])
			{
				m_wz.w.l = m_pc.w.l + (s8) m_data_bus;
				m_pc.w.l = m_wz.w.l;
				m_icount -= 5;
			}
			break;
		// LD A,I, takes 1 cycle
		case LD_A_I:
			m_af.b.h = m_i;
			m_af.b.l = (m_af.b.l & CF) | SZ[m_af.b.h] | (m_iff2 << 2);
			m_after_ldair = true;
			m_icount -= 1;
			break;
		// LD A,R, takes 1 cycle
		case LD_A_R:
			m_af.b.h = (m_r & 0x7f) | m_r2;
			m_af.b.l = (m_af.b.l & CF) | SZ[m_af.b.h] | (m_iff2 << 2);
			m_after_ldair = true;
			m_icount -= 1;
			break;
		// LD I,A, takes 1 cycle
		case LD_I_A:
			m_i = m_af.b.h;
			m_icount -= 1;
			break;
		// LD R,A, takes 1 cycle
		case LD_R_A:
			m_r = m_af.b.h;
			m_r2 = m_af.b.h & 0x80;
			m_icount -= 1;
			break;
		// LD SP,HL, takes 2 cycles
		case LD_SP_HL:
			m_sp.w.l = m_hl_index[m_hl_offset].w.l;
			m_icount -= 2;
			break;
		// Set flags and update pointers and counter, takes 2 cycles
		case LDD:
			m_af.b.l &= SF | ZF | CF;
			if ((m_af.b.h + m_data_bus) & 0x02)
				m_af.b.l |= YF; /* bit 1 -> flag 5 */
			if ((m_af.b.h + m_data_bus) & 0x08)
				m_af.b.l |= XF; /* bit 3 -> flag 3 */
			m_hl_index[m_hl_offset].w.l--;
			m_de.w.l--;
			m_bc.w.l--;
			if (m_bc.w.l)
				m_af.b.l |= VF;
			m_icount -= 2;
			break;
		// Set flags and update pointers and counter, takes 2 cycles
		case LDI:
			m_af.b.l &= SF | ZF | CF;
			if ((m_af.b.h + m_data_bus) & 0x02)
				m_af.b.l |= YF; /* bit 1 -> flag 5 */
			if ((m_af.b.h + m_data_bus) & 0x08)
				m_af.b.l |= XF; /* bit 3 -> flag 3 */
			m_hl_index[m_hl_offset].w.l++;
			m_de.w.l++;
			m_bc.w.l--;
			if (m_bc.w.l)
				m_af.b.l |= VF;
			m_icount -= 2;
			break;
		// NEG, takes 0 cycles
		case NEG:
			{
				u8 res = 0 - m_af.b.h;
				m_af.b.l = SZHVC_sub[res];
				m_af.b.h = res;
			}
			break;
		// NMI, takes 0 cycles
		case NMI:
			m_pc.w.l = 0x66;
			break;
		// Perform OR operation on A and data bus, takes 0 cycles
		case OR_DB:
			alu_or(m_data_bus);
			break;
		// Perform OR operation on A and 8 bit register, takes 0 cycles
		case OR_R8:
			alu_or(regs());
			break;
		// Set flags and update pointers and counter and prepare for I/O, takes 0 cycles
		case OUTD:
			{
				m_bc.b.h--;
				m_wz.w.l = m_bc.w.l - 1;
				m_hl_index[m_hl_offset].w.l--;
				m_af.b.l = SZ[m_bc.b.h];
				u16 t = m_hl_index[m_hl_offset].b.l + m_data_bus;
				if (m_data_bus & SF)
					m_af.b.l |= NF;
				if (t & 0x100)
					m_af.b.l |= HF | CF;
				m_af.b.l |= SZP[(t & 0x07) ^ m_bc.b.h] & PF;
			}
			break;
		// Set flags and update pointers and counter and prepare for I/O, takes 0 cycles
		case OUTI:
			{
				m_bc.b.h--;
				m_wz.w.l = m_bc.w.l + 1;
				m_hl_index[m_hl_offset].w.l++;
				m_af.b.l = SZ[m_bc.b.h];
				u16 t = m_hl_index[m_hl_offset].b.l + m_data_bus;
				if (m_data_bus & SF)
					m_af.b.l |= NF;
				if (t & 0x100)
					m_af.b.l |= HF | CF;
				m_af.b.l |= SZP[(t & 0x07) ^ m_bc.b.h] & PF;
			}
			break;
		// Put BC on address bus, assert IORQ and WR signals for output, takes 4 cycles
		case OUTPUT_S_BC:
			output_s(m_bc.w.l);
			break;
		// Put WZ on address, increment WZ, assert IORQ and WR signals for output, takes 4 cycles
		case OUTPUT_S_WZ_INC:
			output_s(m_wz.w.l);
			m_wz.w.l++;
			break;
		// Put PC on address bus, assert M1, increment PC, takes 1 cycle
		case PC_OUT_INC_M1:
			m_address_bus = m_pc.w.l;
			m_address_bus_cb(m_address_bus);
			set_m1();
			m_icount -= 1;
			m_pc.w.l++;
			break;
		// Skip 1 cycle, put PC 8 high bits on data bus, decrement SP, put SP on address bus, assert MREQ and WR signals for write, takes 4 cycles
		case PCH_WRITE_S_SP_DEC:
			m_icount -= 1;
			m_data_bus = m_pc.b.h;
			m_sp.w.l -= 1;
			write_s(m_sp.w.l);
			break;
		// Put PC 8 low bits on data bus, decrement SP, put SP on address bus, assert MREQ and WR signals for write, takes 3 cycles
		case PCL_WRITE_S_SP_DEC:
			m_data_bus = m_pc.b.l;
			m_sp.w.l -= 1;
			write_s(m_sp.w.l);
			break;
		// Put high 8 bits of 16 bit register on data bus, takes 0 cycles
		case R16H_DB:
			r16h_db();
			break;
		// Skip a cycle, put high 8 bits of 16 bit register on data bus, decrement SP, put SP on address bus, assert MREQ and WR signals for write, takes 4 cycles
		case R16H_WRITE_S_SP_DEC:
			m_icount -= 1;
			r16h_db();
			m_sp.w.l -= 1;
			write_s(m_sp.w.l);
			break;
		// Put high 8 bits of 16 bit register on data bus, put WZ on address bus, assert MREQ and WR signals for write, takes 3 cycles
		case R16H_WRITE_S_WZ:
			r16h_db();
			write_s(m_wz.w.l);
			break;
		// Put low 8 bits of 16 bit register on data bus, decrement SP, put SP on address bus, assert MREQ and WR signals for write, takes 3 cycles
		case R16L_WRITE_S_SP_DEC:
			r16l_db();
			m_sp.w.l -= 1;
			write_s(m_sp.w.l);
			break;
		// Put low 8 bits of 16 bit register on data bus, put WZ on address bus, increment WZ, assert MREQ and WR signals for write, takes 3 cycles
		case R16L_WRITE_S_WZ_INC:
			r16l_db();
			write_s(m_wz.w.l);
			m_wz.w.l++;
			break;
		// Put PC on address bus, increment PC, assert M1, MREQ, and RD signals, takes 2 cycles
		case READ_OP1_S:
			read_op_s();
			m_pc.w.l++;
			break;
		// Put PC on address bus, assert M1, MREQ and RD signals for opcodde read, takes 2 cycle
		case READ_OP_S:
			read_op_s();
			break;
		// Special opcode reading while taking an interrupt, takes 4 cycles
		case READ_OP_IRQ:
			// M1 irqack cycle
			set_m1();
			m_icount -= 3;
			set_iorq();
			m_icount -= 1;
			m_check_wait = true;
			break;
		// Store BC/DE in WZ, put WZ on address bus, increment WZ, assert MREQ and RD signals for read cycle, takes 3 cycles
		case READ_S_BCDE_WZ_INC:
			if (m_ir & 0x10) {
				m_wz.w.l = m_de.w.l;
			} else {
				m_wz.w.l = m_bc.w.l;
			}
			read_s(m_wz.w.l);
			m_wz.w.l++;
			break;
		// Put HL on address bus, assert MREQ and RD signals for read cycle, takes 3 cycles
		case READ_S_HL:
			read_s(m_hl_index[m_hl_offset].w.l);
			break;
		// Put PC on address bus, increment PC, assert MREQ and RD signals for read cycle, takes 3 cycles
		case READ_S_PC:
			read_s(m_pc.w.l);
			m_pc.w.l++;
			break;
		// Put SP on address bus, assert MREQ and RD signals for read cycle, takes 3 cycles
		case READ_S_SP:
			read_s(m_sp.w.l);
			break;
		// Put SP on address bus, increment SP, assert MREQ and RD signals for read cycle, takes 3 cycles
		case READ_S_SP_INC:
			read_s(m_sp.w.l);
			m_sp.w.l += 1;
			break;
		// Put vector address on address bus, increment vector address, assert MREQ and RD signals for read cycle, takes 3 cycles
		case READ_S_VEC:
			read_s(m_irq_vector);
			m_irq_vector++;
			break;
		// Put WZ on address bus, assert MREQ and RD signals for read cycle, takes 3 cycles
		case READ_S_WZ:
			read_s(m_wz.w.l);
			break;
		// Put WZ on address bus, increment WZ, assert MREQ and RD signals for read cycle, takes 3 cycles
		case READ_S_WZ_INC:
			read_s(m_wz.w.l);
			m_wz.w.l++;
			break;
		// Refresh RAM, takes 2 cycles
		case REFRESH:
			set_rfsh();
			m_icount -= 1;
			set_mreq();
			m_refresh_cb((m_i << 8) | m_r, 0x00, 0xff);
			m_icount -= 1;
			clear_mreq();
			clear_rfsh();
			m_r++;
			break;
		// Refresh RAM and decode instruction, takes 2 cycles
		case REFRESH_DECODE:
			set_rfsh();
			m_icount -= 1;
			set_mreq();
			m_refresh_cb((m_i << 8) | m_r, 0x00, 0xff);
			m_icount -= 1;
			clear_mreq();
			clear_rfsh();
			m_r++;
			decode();
			break;
		// Store destination register contents on data bus, takes 0 cycles
		case REGD_DB:
			switch (m_ir & 0x38)
			{
			case 0x00:
				m_data_bus = m_bc.b.h;
				break;
			case 0x08:
				m_data_bus = m_bc.b.l;
				break;
			case 0x10:
				m_data_bus = m_de.b.h;
				break;
			case 0x18:
				m_data_bus = m_de.b.l;
				break;
			case 0x20:
				m_data_bus = m_hl_index[m_hl_offset].b.h;
				break;
			case 0x28:
				m_data_bus = m_hl_index[m_hl_offset].b.l;
				break;
			case 0x30:
				fatalerror("REGD_DB: illegal register reference 0x06\n");
				break;
			case 0x38:
				m_data_bus = m_af.b.h;
				break;
			}
			break;
		// Copy source 8 bit register to destination 8 bit register, takes 0 cycles
		case REGS_TMP_REG:
			regd(regs());
			break;
		// 8 bit source register (bits .....xxx) to data bus (not from index registers), takes 0 cycles
		case REGS0_DB:
			regs0_db();
			break;
		// 8 bit source register (bits .....xxx) to data bus (not from index registers), put HL on address bus, assert MREQ and WR signals for write, takes 3 cycles
		case REGS0_WRITE_S_HL:
			regs0_db();
			write_s(m_hl_index[m_hl_offset].w.l);
			break;
		// Move PC 2 steps back if BC != 0, takes 5 cycles
		case REPEAT:
			if (m_bc.w.l != 0)
			{
				m_pc.w.l -= 2;
				// Except for inir, otir, indr, otdr
				if (!BIT(m_ir,1))
				{
					m_wz.w.l = m_pc.w.l + 1;
				}
				m_icount -= 5;
			}
			break;
		// Move PC 2 steps back if BC != 0 and ZF clear, takes 5 cycles
		case REPEATCP:
			if (m_bc.w.l != 0 && !(m_af.b.l & ZF))
			{
				m_pc.w.l -= 2;
				// Except for inir, otir, indr, otdr
				if (!BIT(m_ir,1))
				{
					m_wz.w.l = m_pc.w.l + 1;
				}
				m_icount -= 5;
			}
			break;
		// Move PC 2 steps back if B != 0, takes 5 cycles
		case REPEATIO:
			if (m_bc.b.h != 0) {
				m_pc.w.l -= 2;
				m_icount -= 5;
			}
			break;
		// Perform RES operation on 8 bit register, takes 0 cycles
		case RES_R8:
			regs(alu_res(regs()));
			break;
		// Perform RES operation on data bus, assert MREQ and WR signals for write, takes 4 cycles
		case RES_WRITE_S:
			m_data_bus = alu_res(m_data_bus);
			regs0(m_data_bus);
			m_icount -= 2;
			write_s();
			break;
		// Check condition for RET, put SP on address bus, increment SP, assert MREQ and RD signals for read cycle, takes 1 or 4 cycles
		case RET_COND:
			m_icount -= 1;
			if ((m_af.b.l & jp_conditions[((m_ir >> 3) & 0x07)][0]) != jp_conditions[((m_ir >> 3) & 0x07)][1])
			{
				end_instruction();
			} else {
				read_s(m_sp.w.l);
				m_sp.w.l += 1;
			}
			break;
		// RETI, takes 0 cycles
		case RETI:
			m_iff1 = m_iff2;
			daisy_call_reti_device();
			break;
		// RETN, takes 0 cycles
		case RETN:
			m_iff1 = m_iff2;
			break;
		// Perform RL operation on 8 bit register, takes 0 cycles
		case RL_R8:
			regs(alu_rl(regs()));
			break;
		// Perform RL operation on data bus, assert MREQ and WR signals for write, takes 4 cycles
		case RL_WRITE_S:
			m_data_bus = alu_rl(m_data_bus);
			regs0(m_data_bus);
			m_icount -= 2;
			write_s();
			break;
		// RLA, takes 0 cycles
		case RLA:
			{
				u8 res = (m_af.b.h << 1) | (m_af.b.l & CF);
				m_af.b.l = (m_af.b.l & (SF | ZF | PF)) | ((m_af.b.h & 0x80) ? CF : 0) | (res & (YF | XF));
				m_af.b.h = res;
			}
			break;
		// Perform RLC operation on 8 bit register, takes 0 cycles
		case RLC_R8:
			regs(alu_rlc(regs()));
			break;
		// Perform RLC operation on data bus, assert MREQ and WR signals for write, takes 4 cycles
		case RLC_WRITE_S:
			m_data_bus = alu_rlc(m_data_bus);
			regs0(m_data_bus);
			m_icount -= 2;
			write_s();
			break;
		// RLCA, takes 0 cycles
		case RLCA:
			m_af.b.h = (m_af.b.h << 1) | (m_af.b.h >> 7);
			m_af.b.l = (m_af.b.l & (SF | ZF | PF)) | (m_af.b.h & (YF | XF | CF));
			break;
		// RLD, takes 5 cycles
		case RLD:
			{
				u8 res = (m_data_bus << 4) | (m_af.b.h & 0x0f);
				m_af.b.h = (m_af.b.h & 0xf0) | (m_data_bus >> 4);
				m_af.b.l = (m_af.b.l & CF) | SZP[m_af.b.h];
				m_data_bus = res;
				m_icount -= 5;
			}
			break;
		// Perform RR operation on 8 bit register, takes 0 cycles
		case RR_R8:
			regs(alu_rr(regs()));
			break;
		// Perform RR operation on data bus, assert MREQ and WR signals for write, takes 4 cycles
		case RR_WRITE_S:
			m_data_bus = alu_rr(m_data_bus);
			regs0(m_data_bus);
			m_icount -= 2;
			write_s();
			break;
		// RRA, takes 0 cycles
		case RRA:
			{
				u8 res = (m_af.b.h >> 1) | (m_af.b.l << 7);
				m_af.b.l = (m_af.b.l & (SF | ZF | PF)) | ((m_af.b.h & 0x01) ? CF : 0) | (res & (YF | XF));
				m_af.b.h = res;
			}
			break;
		// Perform RRC operation on 8 bit register, takes 0 cycles
		case RRC_R8:
			regs(alu_rrc(regs()));
			break;
		// Perform RRC operation on data bus, assert MREQ and WR signals for write, takes 4 cycles
		case RRC_WRITE_S:
			m_data_bus = alu_rrc(m_data_bus);
			regs0(m_data_bus);
			m_icount -= 2;
			write_s();
			break;
		// RRCA, takes 0 cycles
		case RRCA:
			m_af.b.l = (m_af.b.l & (SF | ZF | PF)) | (m_af.b.h & CF);
			m_af.b.h = (m_af.b.h >> 1) | (m_af.b.h << 7);
			m_af.b.l |= (m_af.b.h & (YF | XF));
			break;
		// RRD, takes 5 cycles
		case RRD:
			{
				u8 res = (m_data_bus >> 4) | (m_af.b.h << 4);
				m_af.b.h = (m_af.b.h & 0xf0) | (m_data_bus & 0x0f);
				m_af.b.l = (m_af.b.l & CF) | SZP[m_af.b.h];
				m_data_bus = res;
				m_icount -= 5;
			}
			break;
		// Change PC to 0/8/10/18/20/28/30/38, takes 0 cycles
		case RST:
			m_pc.w.l = m_ir & 0x38;
			m_wz.w.l = m_pc.w.l;
			break;
		// Perform SBC operation on A and data bus, takes 0 cycles
		case SBC_DB:
			alu_sbc(m_data_bus);
			break;
		// Perform SBC operation on A and 8 bit register, takes 0 cycles
		case SBC_R8:
			alu_sbc(regs());
			break;
		// 16bit subtraction with carry, takes 7 cycles
		case SBC16:
			sbc16();
			break;
		// SCF
		case SCF:
			m_af.b.l = (m_af.b.l & (SF | ZF | YF | XF | PF)) | CF | (m_af.b.h & (YF | XF));
			break;
		// Perform SET operation on 8 bit register, takes 0 cycles
		case SET_R8:
			regs(alu_set(regs()));
			break;
		// Perform SET operation on data bus, assert MREQ and WR signals for write, takes 4 cycles
		case SET_WRITE_S:
			m_data_bus = alu_set(m_data_bus);
			regs0(m_data_bus);
			m_icount -= 2;
			write_s();
			break;
		// Perform SLA operation on 8 bit register, takes 0 cycles
		case SLA_R8:
			regs(alu_sla(regs()));
			break;
		// Perform SLA operation on data bus, assert MREQ and WR signals for write, takes 4 cycles
		case SLA_WRITE_S:
			m_data_bus = alu_sla(m_data_bus);
			regs0(m_data_bus);
			m_icount -= 2;
			write_s();
			break;
		// Perform SLL operation on 8 bit register, takes 0 cycles
		case SLL_R8:
			regs(alu_sll(regs()));
			break;
		// Perform SLL operation on data bus, assert MREQ and WR signals for write, takes 4 cycles
		case SLL_WRITE_S:
			m_data_bus = alu_sll(m_data_bus);
			regs0(m_data_bus);
			m_icount -= 2;
			write_s();
			break;
		// Perform SRA operation on 8 bit register, takes 0 cycles
		case SRA_R8:
			regs(alu_sra(regs()));
			break;
		// Perform SRA operation on data bus, assert MREQ and WR signals for write, takes 4 cycles
		case SRA_WRITE_S:
			m_data_bus = alu_sra(m_data_bus);
			regs0(m_data_bus);
			m_icount -= 2;
			write_s();
			break;
		// Perform SRL operation on 8 bit register, takes 0 cycles
		case SRL_R8:
			regs(alu_srl(regs()));
			break;
		// Perform SRL operation on data bus, assert MREQ and WR signals for write, takes 4 cycles
		case SRL_WRITE_S:
			m_data_bus = alu_srl(m_data_bus);
			regs0(m_data_bus);
			m_icount -= 2;
			write_s();
			break;
		// Perform SUB operation on A and data bus, takes 0 cycles
		case SUB_DB:
			alu_sub(m_data_bus);
			break;
		// Perform SUB operation on A and 8 bit register, takes 0 cycles
		case SUB_R8:
			alu_sub(regs());
			break;
		// LD (BC),A ($02) and LD (DE),A ($12)
		// Store BC/DE in WZ, put WZ on address bus, increment WZ, store A in W and put on data bus, assert MREQ and WR signals for write, takes 3 cycles
		case WRITE_BCDE_A:
			if (m_ir & 0x10) {
				m_wz.w.l = m_de.w.l;
			} else {
				m_wz.w.l = m_bc.w.l;
			}
			m_data_bus = m_af.b.h;
			write_s(m_wz.w.l);
			m_wz.w.l++;
			m_wz.b.h = m_af.b.h;
			break;
		// Assert MREQ and WR signals for write, takes 2 cycles
		case WRITE_S:
			write_s();
			break;
		// Put DE on address bus, assert MREQ and WR signals for write, takes 3 cycles
		case WRITE_S_DE:
			write_s(m_de.w.l);
			break;
		// Put HL on address bus, assert MREQ and WR signals for write, takes 3 cycles
		case WRITE_S_HL:
			write_s(m_hl_index[m_hl_offset].w.l);
			break;
		// Decrement SP, put SP on address bus, assert MREQ and WR signals for write, takes 3 cycles
		case WRITE_S_SP_DEC:
			m_sp.w.l -= 1;
			write_s(m_sp.w.l);
			break;
		// Put WZ on address bus, assert MREQ and WR signals for write, takes 3 cycles
		case WRITE_S_WZ:
			write_s(m_wz.w.l);
			break;
		// Put WZ on address bus, increment WZ, assert MREQ and WR signals for write, takes 3 cycles
		case WRITE_S_WZ_INC:
			write_s(m_wz.w.l);
			m_wz.w.l++;
			break;
		// Store contents of WZ in HL, takes 0 cycles
		case WZ_HL:
			m_hl_index[m_hl_offset].w.l = m_wz.w.l;
			break;
		// Store contents of WZ in PC, takes 0 cycles
		case WZ_PC:
			m_pc.w.l = m_wz.w.l;
			break;
		// Do nothing, takes 1 cycle
		case X:
			m_icount -= 1;
			break;
		// Wait 1 cycle, put PC on address bus, increment PC, assert MREQ and RD signals for read cycle, takes 4 cycles in total
		case X_READ_S_PC:
			read_s(m_pc.w.l);
			m_pc.w.l++;
			break;
		// Do nothing, takes 2 cycles
		case X2:
			m_icount -= 2;
			break;
		// Perform XOR operation on A and data bus, takes 0 cycles
		case XOR_DB:
			alu_xor(m_data_bus);
			break;
		// Perform XOR operation on A an 8 bit register, takes 0 cycles
		case XOR_R8:
			alu_xor(regs());
			break;
		// put all zeroes on the data bus, takes 0 cycles
		case ZERO_DB:
			m_data_bus = 0;
			break;
		}
		if (step & END) {
			end_instruction();
		}
	} while (m_icount > 0);
}


void z80lle_device::execute_set_input(int inputnum, int state)
{
	switch (inputnum)
	{
	case Z80LLE_INPUT_LINE_BUSRQ:
		m_busrq_state = state;
		break;

	case INPUT_LINE_NMI:
		// mark an NMI pending on the rising edge
		if (m_nmi_state == CLEAR_LINE && state != CLEAR_LINE)
			m_nmi_pending = true;
		m_nmi_state = state;
		break;

	case INPUT_LINE_IRQ0:
		// update the IRQ state via the daisy chain
		m_irq_state = state;
		if (daisy_chain_present())
			m_irq_state = (daisy_update_irq_state() == ASSERT_LINE ) ? ASSERT_LINE : m_irq_state;

		// the main execute loop will take the interrupt
		break;

	case Z80LLE_INPUT_LINE_WAIT:
		m_wait_state = state;
		break;

	default:
		break;
	}
}


///////////////////////////////////////////////////////////////////////////
// STATE IMPORT/EXPORT
///////////////////////////////////////////////////////////////////////////

void z80lle_device::state_import( const device_state_entry &entry )
{
	switch (entry.index())
	{
		case STATE_GENPC:
			m_prvpc = m_pc;
			break;

		case STATE_GENPCBASE:
			m_pc = m_prvpc;
			break;

		case Z80LLE_R:
			m_r = m_rtemp & 0x7f;
			m_r2 = m_rtemp & 0x80;
			break;

		default:
			fatalerror("state_import() called for unexpected value\n");
	}
}


void z80lle_device::state_export( const device_state_entry &entry )
{
	switch (entry.index())
	{
		case Z80LLE_R:
			m_rtemp = (m_r & 0x7f) | (m_r2 & 0x80);
			break;

		default:
			fatalerror("state_export() called for unexpected value\n");
	}
}

void z80lle_device::state_string_export(const device_state_entry &entry, std::string &str) const
{
	switch (entry.index())
	{
		case STATE_GENFLAGS:
			str = string_format("%c%c%c%c%c%c%c%c",
				m_af.b.l & 0x80 ? 'S':'.',
				m_af.b.l & 0x40 ? 'Z':'.',
				m_af.b.l & 0x20 ? 'Y':'.',
				m_af.b.l & 0x10 ? 'H':'.',
				m_af.b.l & 0x08 ? 'X':'.',
				m_af.b.l & 0x04 ? 'P':'.',
				m_af.b.l & 0x02 ? 'N':'.',
				m_af.b.l & 0x01 ? 'C':'.');
			break;
	}
}

//-------------------------------------------------
//  disassemble - call the disassembly
//  helper function
//-------------------------------------------------

std::unique_ptr<util::disasm_interface> z80lle_device::create_disassembler()
{
	return std::make_unique<z80_disassembler>();
}


z80lle_device::z80lle_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: z80lle_device(mconfig, Z80LLE, tag, owner, clock)
{
}


z80lle_device::z80lle_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: cpu_device(mconfig, type, tag, owner, clock)
	, z80_daisy_chain_interface(mconfig, *this)
	, m_program_config("program", ENDIANNESS_LITTLE, 8, 16, 0)
	, m_opcodes_config("opcodes", ENDIANNESS_LITTLE, 8, 16, 0)
	, m_io_config("io", ENDIANNESS_LITTLE, 8, 16, 0)
	, m_irqack_cb(*this)
	, m_refresh_cb(*this)
	, m_halt_cb(*this)
	, m_mreq_cb(*this)
	, m_iorq_cb(*this)
	, m_rd_cb(*this)
	, m_wr_cb(*this)
	, m_m1_cb(*this)
	, m_address_bus_cb(*this)
	, m_m1_wait_states(0)
{
}


device_memory_interface::space_config_vector z80lle_device::memory_space_config() const
{
	if(has_configured_map(AS_OPCODES))
		return space_config_vector {
			std::make_pair(AS_PROGRAM, &m_program_config),
			std::make_pair(AS_OPCODES, &m_opcodes_config),
			std::make_pair(AS_IO,      &m_io_config)
		};
	else
		return space_config_vector {
			std::make_pair(AS_PROGRAM, &m_program_config),
			std::make_pair(AS_IO,      &m_io_config)
		};
}


DEFINE_DEVICE_TYPE(Z80LLE, z80lle_device, "z80lle", "Zilog Z80 LLE")


