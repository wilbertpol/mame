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
 *   TODO:
 *   - Implement the 2 start up cycles
 *   - Add support for interrupts
 *   - Add support for NMI
 *   - WZ is only incremented once for $22 LD (nn),HL?
 *   - Verify A_DB, should it set WH for each instruction?
 *   - Add more instructions
 *   - Move the flag tables into the class definition
 *
 *   Simple improvements:
 *   - See if ALU_xxx and ALU_A sub instructions can be merged into the ALU_xxx sub instructions
 *   - See if PC_OUT and PC_INC can be merged into the PC_OUT sub instruction
 *   - Introduce a WZ_OUT_INC sub instruction to replace WZ_OUT, WZ_INC sequences
 *
 *****************************************************************************/

#include "emu.h"
#include "debugger.h"
#include "z80lle.h"
#include "z80dasm.h"

#define VERBOSE             0

#define LOG(x)  do { if (VERBOSE) logerror x; } while (0)


/****************************************************************************/
/* The Z80 registers. halt is set to 1 when the CPU is halted, the refresh  */
/* register is calculated as follows: refresh=(r&127)|(r2&128)    */
/****************************************************************************/

#define CF      0x01
#define NF      0x02
#define PF      0x04
#define VF      PF
#define XF      0x08
#define HF      0x10
#define YF      0x20
#define ZF      0x40
#define SF      0x80

#define INT_IRQ 0x01
#define NMI_IRQ 0x02

#define PC      m_pc.w.l
#define PC_H    m_pc.b.h
#define PC_L    m_pc.b.l

#define SP      m_sp.w.l
#define SP_H    m_sp.b.h
#define SP_L    m_sp.b.l

#define AF      m_af.w.l
#define A       m_af.b.h
#define F       m_af.b.l

#define BC      m_bc.w.l
#define B       m_bc.b.h
#define C       m_bc.b.l

#define DE      m_de.w.l
#define D       m_de.b.h
#define E       m_de.b.l

#define HL      m_hl_index[m_hl_offset].w.l
#define H       m_hl_index[m_hl_offset].b.h
#define L       m_hl_index[m_hl_offset].b.l

#define WZ      m_wz.w.l
#define WZ_H    m_wz.b.h
#define WZ_L    m_wz.b.l


static bool tables_initialised = false;
static uint8_t SZ[256];       /* zero and sign flags */
static uint8_t SZ_BIT[256];   /* zero, sign and parity/overflow (=zero) flags for BIT opcode */
static uint8_t SZP[256];      /* zero, sign and parity flags */
static uint8_t SZHV_inc[256]; /* zero, sign, half carry and overflow flags INC r8 */
static uint8_t SZHV_dec[256]; /* zero, sign, half carry and overflow flags DEC r8 */

static uint8_t SZHVC_add[2*256*256];
static uint8_t SZHVC_sub[2*256*256];


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

const u8 z80lle_device::insts[5 * 256 + 2][23] = {
	/*****************************************************/
	/* Regular instructions                              */
	/*****************************************************/

	/* 00 */ { END },  // 4 cycles, NOP
	/* 01 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_R16L, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_R16H, END },  // 10 cycles, LD BC,nn
	/* 02 */ { BC_WZ, WZ_OUT, WZ_INC, A_DB, WRITE, CHECK_WAIT, END },  // 7 cycles, LD (BC),A
	/* 03 */ { INC_R16, END },  // 6 cycles, INC BC
	/* 04 */ { REGD_TMP, ALU_INC, ALU_REGD, END },  // 4 cycles, INC B
	/* 05 */ { REGD_TMP, ALU_DEC, ALU_REGD, END },  // 4 cycles, DEC B
	/* 06 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_REG, END },  // 7 cycles, LD B,n
	/* 07 */ { RLCA, END },  // 4 cycles, RLCA
	{ 0 },
	/* 09 */ { ADD16, END },  // 11 cycles, ADD HL,BC
	/* 0a */ { BC_WZ, WZ_OUT, WZ_INC, READ, CHECK_WAIT, DB_A, END },  // 7 cycles, LD A,(BC)
	/* 0b */ { DEC_R16, END },  // 6 cycles, DEC BC
	/* 0c */ { REGD_TMP, ALU_INC, ALU_REGD, END },  // 4 cycles, INC C
	/* 0d */ { REGD_TMP, ALU_DEC, ALU_REGD, END },  // 4 cycles, DEC C
	/* 0e */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_REG, END },  // 7 cycles, LD C,n
	/* 0f */ { RRCA, END },  // 4 cycles, RRCA
	/* 0x10 */
	{ 0 },
	/* 11 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_R16L, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_R16H, END },  // 10 cycles, LD DE,nn
	/* 12 */ { DE_WZ, WZ_OUT, WZ_INC, A_DB, WRITE, CHECK_WAIT, END },  // 7 cycles, LD (DE),A
	/* 13 */ { INC_R16, END },  // 6 cycles, INC DE
	/* 14 */ { REGD_TMP, ALU_INC, ALU_REGD, END },  // 4 cycles, INC D
	/* 15 */ { REGD_TMP, ALU_DEC, ALU_REGD, END },  // 4 cycles, DEC D
	/* 16 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_REG, END },  // 7 cycles, LD D,n
	/* 17 */ { RLA, END },  // 4 cycles, RLA
	/* 18 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, JR_COND, END },  // 12 cycles, JR n
	/* 19 */ { ADD16, END },  // 11 cycles, ADD HL,DE
	/* 1a */ { DE_WZ, WZ_OUT, WZ_INC, READ, CHECK_WAIT, DB_A, END },  // 7 cycles, LD A,(DE)
	/* 1b */ { DEC_R16, END },  // 6 cycles, DEC DE
	/* 1c */ { REGD_TMP, ALU_INC, ALU_REGD, END },  // 4 cycles, INC E
	/* 1d */ { REGD_TMP, ALU_DEC, ALU_REGD, END },  // 4 cycles, DEC E
	/* 1e */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_REG, END },  // 7 cycles, LD E,n
	/* 1f */ { RRA, END },  // 4 cycles, RRA

	/* 20 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, JR_COND, END },  // 7/12 cycles, JR NZ,n
	/* 21 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_R16L, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_R16H, END },  // 10 cycles, LD HL,nn
	/* 22 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, WZ_OUT, WZ_INC, L_DB, WRITE, CHECK_WAIT, WZ_OUT, H_DB, WRITE, CHECK_WAIT, END },  // 16 cycles, LD (nn),HL
	/* 23 */ { INC_R16, END },  // 6 cycles, INC HL
	/* 24 */ { REGD_TMP, ALU_INC, ALU_REGD, END },  // 4 cycles, INC H
	/* 25 */ { REGD_TMP, ALU_DEC, ALU_REGD, END },  // 4 cycles, DEC H
	/* 26 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_REG, END },  // 7 cycles, LD H,n
	/* 27 */ { DAA, END },  // 4 cycles, DAA
	/* 28 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, JR_COND, END },  // 7/12 cycles, JR Z,n
	/* 29 */ { ADD16, END },  // 11 cycles, ADD HL,HL
	/* 2a */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, WZ_OUT, WZ_INC, READ, CHECK_WAIT, DB_R16L, WZ_OUT, READ, CHECK_WAIT, DB_R16H, END },  // 16 cycles, LD HL,(nn)
	/* 2b */ { DEC_R16, END },  // 6 cycles, DEC HL
	/* 2c */ { REGD_TMP, ALU_INC, ALU_REGD, END },  // 4 cycles, INC L
	/* 2d */ { REGD_TMP, ALU_DEC, ALU_REGD, END },  // 4 cycles, DEC L
	/* 2e */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_REG, END },  // 7 cycles, LD L,n
	/* 2f */ { CPL, END },  // 4 cycles, CPL
	/* 0x30 */
	{ 0 },
	/* 31 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_R16L, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_R16H, END },  // 10 cycles, LD SP,nn
	/* 32 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, WZ_OUT, WZ_INC, A_DB, WRITE, CHECK_WAIT, END },  // 13 cycles, LD (nn),A
	/* 33 */ { INC_R16, END },  // 6 cycles, INC SP
	/* 34 */ { HL_OUT, READ, CHECK_WAIT, DB_TMP, ALU_INC, ALU_DB, X, HL_OUT, WRITE, CHECK_WAIT, END },  // 11 cycles, INC (HL)
	/* 35 */ { HL_OUT, READ, CHECK_WAIT, DB_TMP, ALU_DEC, ALU_DB, X, HL_OUT, WRITE, CHECK_WAIT, END },  // 11 cycles, DEC (HL)
	/* 36 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, HL_OUT, WRITE, END },  // 10 cycles, LD (HL),n
	/* 37 */ { SCF, END },  // 4 cycles, SCF
	{ 0 },
	/* 39 */ { ADD16, END },  // 11 cycles, ADD HL,SP
	/* 3a */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, WZ_OUT, WZ_INC, READ, CHECK_WAIT, DB_A, END },  // 13 cycles, LD A,(nn)
	/* 3b */ { DEC_R16, END },  // 6 cycles, DEC SP
	/* 3c */ { REGD_TMP, ALU_INC, ALU_REGD, END },  // 4 cycles, INC A
	/* 3d */ { REGD_TMP, ALU_DEC, ALU_REGD, END },  // 4 cycles, DEC A
	/* 3e */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_REG, END },  // 7 cycles, LD A,n
	/* 3f */ { CCF, END },  // 4 cycles, CCF

	/* 40 */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD B,B
	/* 41 */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD B,C
	/* 42 */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD B,D
	/* 43 */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD B,E
	/* 44 */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD B,H
	/* 45 */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD B,L
	/* 46 */ { HL_OUT, READ, CHECK_WAIT, DB_REG, END },  // 7 cycles, LD B,(HL)
	/* 47 */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD B,A
	/* 48 */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD C,B
	/* 49 */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD C,C
	/* 4a */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD C,D
	/* 4b */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD C,E
	/* 4c */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD C,H
	/* 4d */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD C,L
	/* 4e */ { HL_OUT, READ, CHECK_WAIT, DB_REG, END },  // 7 cycles, LD C,(HL)
	/* 4f */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD C,A

	/* 50 */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD D,B
	/* 51 */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD D,C
	/* 52 */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD D,D
	/* 53 */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD D,E
	/* 54 */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD D,H
	/* 55 */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD D,L
	/* 56 */ { HL_OUT, READ, CHECK_WAIT, DB_REG, END },  // 7 cycles, LD D,(HL)
	/* 57 */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD D,A
	/* 58 */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD E,B
	/* 59 */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD E,C
	/* 5a */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD E,D
	/* 5b */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD E,E
	/* 5c */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD E,H
	/* 5d */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD E,L
	/* 5e */ { HL_OUT, READ, CHECK_WAIT, DB_REG, END },  // 7 cycles, LD E,(HL)
	/* 5f */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD E,A

	/* 60 */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD H,B
	/* 61 */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD H,C
	/* 62 */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD H,D
	/* 63 */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD H,E
	/* 64 */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD H,H
	/* 65 */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD H,L
	/* 66 */ { HL_OUT, READ, CHECK_WAIT, DB_REG, END },  // 7 cycles, LD H,(HL)
	/* 67 */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD H,A
	/* 68 */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD L,B
	/* 69 */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD L,C
	/* 6a */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD L,D
	/* 6b */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD L,E
	/* 6c */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD L,H
	/* 6d */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD L,L
	/* 6e */ { HL_OUT, READ, CHECK_WAIT, DB_REG, END },  // 7 cycles, LD L,(HL)
	/* 6f */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD L,A

	/* 70 */ { HL_OUT, REGS_DB, WRITE, CHECK_WAIT, END },  // 7 cycles, LD (HL),B
	/* 71 */ { HL_OUT, REGS_DB, WRITE, CHECK_WAIT, END },  // 7 cycles, LD (HL),C
	/* 72 */ { HL_OUT, REGS_DB, WRITE, CHECK_WAIT, END },  // 7 cycles, LD (HL),D
	/* 73 */ { HL_OUT, REGS_DB, WRITE, CHECK_WAIT, END },  // 7 cycles, LD (HL),E
	/* 74 */ { HL_OUT, REGS_DB, WRITE, CHECK_WAIT, END },  // 7 cycles, LD (HL),H
	/* 75 */ { HL_OUT, REGS_DB, WRITE, CHECK_WAIT, END },  // 7 cycles, LD (HL),L
	{ 0 },
	/* 77 */ { HL_OUT, REGS_DB, WRITE, CHECK_WAIT, END },  // 7 cycles, LD (HL),A
	/* 78 */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD A,B
	/* 79 */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD A,C
	/* 7a */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD A,D
	/* 7b */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD A,E
	/* 7c */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD A,H
	/* 7d */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD A,L
	/* 7e */ { HL_OUT, READ, CHECK_WAIT, DB_REG, END },  // 7 cycles, LD A,(HL)
	/* 7f */ { REGS_TMP, TMP_REG, END },  // 4 cycles, LD A,A

	/* 80 */ { A_ACT, REGS_TMP, ALU_ADD, ALU_A, END },  // 4 cycles, ADD B
	/* 81 */ { A_ACT, REGS_TMP, ALU_ADD, ALU_A, END },  // 4 cycles, ADD C
	/* 82 */ { A_ACT, REGS_TMP, ALU_ADD, ALU_A, END },  // 4 cycles, ADD D
	/* 83 */ { A_ACT, REGS_TMP, ALU_ADD, ALU_A, END },  // 4 cycles, ADD E
	/* 84 */ { A_ACT, REGS_TMP, ALU_ADD, ALU_A, END },  // 4 cycles, ADD H
	/* 85 */ { A_ACT, REGS_TMP, ALU_ADD, ALU_A, END },  // 4 cycles, ADD L
	/* 86 */ { HL_OUT, READ, CHECK_WAIT, A_ACT, DB_TMP, ALU_ADD, ALU_A, END },  // 7 cycles, ADD (HL)
	/* 87 */ { A_ACT, REGS_TMP, ALU_ADD, ALU_A, END },  // 4 cycles, ADD A
	/* 88 */ { A_ACT, REGS_TMP, ALU_ADC, ALU_A, END },  // 4 cycles, ADC B
	/* 89 */ { A_ACT, REGS_TMP, ALU_ADC, ALU_A, END },  // 4 cycles, ADC C
	/* 8a */ { A_ACT, REGS_TMP, ALU_ADC, ALU_A, END },  // 4 cycles, ADC D
	/* 8b */ { A_ACT, REGS_TMP, ALU_ADC, ALU_A, END },  // 4 cycles, ADC E
	/* 8c */ { A_ACT, REGS_TMP, ALU_ADC, ALU_A, END },  // 4 cycles, ADC H
	/* 8d */ { A_ACT, REGS_TMP, ALU_ADC, ALU_A, END },  // 4 cycles, ADC L
	/* 8e */ { HL_OUT, READ, CHECK_WAIT, A_ACT, DB_TMP, ALU_ADC, ALU_A, END },  // 7 cycles, ADC (HL)
	/* 8f */ { A_ACT, REGS_TMP, ALU_ADC, ALU_A, END },  // 4 cycles, ADC A

	/* 90 */ { A_ACT, REGS_TMP, ALU_SUB, ALU_A, END },  // 4 cycles, SUB B
	/* 91 */ { A_ACT, REGS_TMP, ALU_SUB, ALU_A, END },  // 4 cycles, SUB C
	/* 92 */ { A_ACT, REGS_TMP, ALU_SUB, ALU_A, END },  // 4 cycles, SUB D
	/* 93 */ { A_ACT, REGS_TMP, ALU_SUB, ALU_A, END },  // 4 cycles, SUB E
	/* 94 */ { A_ACT, REGS_TMP, ALU_SUB, ALU_A, END },  // 4 cycles, SUB H
	/* 95 */ { A_ACT, REGS_TMP, ALU_SUB, ALU_A, END },  // 4 cycles, SUB L
	/* 96 */ { HL_OUT, READ, CHECK_WAIT, A_ACT, DB_TMP, ALU_SUB, ALU_A, END },  // 7 cycles, SUB (HL)
	/* 97 */ { A_ACT, REGS_TMP, ALU_SUB, ALU_A, END },  // 4 cycles, SUB A
	/* 98 */ { A_ACT, REGS_TMP, ALU_SBC, ALU_A, END },  // 4 cycles, SBC B
	/* 99 */ { A_ACT, REGS_TMP, ALU_SBC, ALU_A, END },  // 4 cycles, SBC C
	/* 9a */ { A_ACT, REGS_TMP, ALU_SBC, ALU_A, END },  // 4 cycles, SBC D
	/* 9b */ { A_ACT, REGS_TMP, ALU_SBC, ALU_A, END },  // 4 cycles, SBC E
	/* 9c */ { A_ACT, REGS_TMP, ALU_SBC, ALU_A, END },  // 4 cycles, SBC H
	/* 9d */ { A_ACT, REGS_TMP, ALU_SBC, ALU_A, END },  // 4 cycles, SBC L
	/* 9e */ { HL_OUT, READ, CHECK_WAIT, A_ACT, DB_TMP, ALU_SBC, ALU_A, END },  // 7 cycles, SBC (HL)
	/* 9f */ { A_ACT, REGS_TMP, ALU_SBC, ALU_A, END },  // 4 cycles, SBC A

	/* a0 */ { A_ACT, REGS_TMP, ALU_AND, ALU_A, END },  // 4 cycles, AND B
	/* a1 */ { A_ACT, REGS_TMP, ALU_AND, ALU_A, END },  // 4 cycles, AND C
	/* a2 */ { A_ACT, REGS_TMP, ALU_AND, ALU_A, END },  // 4 cycles, AND D
	/* a3 */ { A_ACT, REGS_TMP, ALU_AND, ALU_A, END },  // 4 cycles, AND E
	/* a4 */ { A_ACT, REGS_TMP, ALU_AND, ALU_A, END },  // 4 cycles, AND H
	/* a5 */ { A_ACT, REGS_TMP, ALU_AND, ALU_A, END },  // 4 cycles, AND L
	/* a6 */ { HL_OUT, READ, CHECK_WAIT, A_ACT, DB_TMP, ALU_AND, ALU_A, END },  // 7 cycles, AND (HL)
	/* a7 */ { A_ACT, REGS_TMP, ALU_AND, ALU_A, END },  // 4 cycles, AND A
	/* a8 */ { A_ACT, REGS_TMP, ALU_XOR, ALU_A, END },  // 4 cycles, XOR B
	/* a9 */ { A_ACT, REGS_TMP, ALU_XOR, ALU_A, END },  // 4 cycles, XOR C
	/* aa */ { A_ACT, REGS_TMP, ALU_XOR, ALU_A, END },  // 4 cycles, XOR D
	/* ab */ { A_ACT, REGS_TMP, ALU_XOR, ALU_A, END },  // 4 cycles, XOR E
	/* ac */ { A_ACT, REGS_TMP, ALU_XOR, ALU_A, END },  // 4 cycles, XOR H
	/* ad */ { A_ACT, REGS_TMP, ALU_XOR, ALU_A, END },  // 4 cycles, XOR L
	/* ae */ { HL_OUT, READ, CHECK_WAIT, A_ACT, DB_TMP, ALU_XOR, ALU_A, END },  // 7 cycles, XOR (HL)
	/* af */ { A_ACT, REGS_TMP, ALU_XOR, ALU_A, END },  // 4 cycles, XOR A

	/* b0 */ { A_ACT, REGS_TMP, ALU_OR, ALU_A, END },  // 4 cycles, OR B
	/* b1 */ { A_ACT, REGS_TMP, ALU_OR, ALU_A, END },  // 4 cycles, OR C
	/* b2 */ { A_ACT, REGS_TMP, ALU_OR, ALU_A, END },  // 4 cycles, OR D
	/* b3 */ { A_ACT, REGS_TMP, ALU_OR, ALU_A, END },  // 4 cycles, OR E
	/* b4 */ { A_ACT, REGS_TMP, ALU_OR, ALU_A, END },  // 4 cycles, OR H
	/* b5 */ { A_ACT, REGS_TMP, ALU_OR, ALU_A, END },  // 4 cycles, OR L
	/* b6 */ { HL_OUT, READ, CHECK_WAIT, A_ACT, DB_TMP, ALU_OR, ALU_A, END },  // 7 cycles, OR (HL)
	/* b7 */ { A_ACT, REGS_TMP, ALU_OR, ALU_A, END },  // 4 cycles, OR A
	/* b8 */ { A_ACT, REGS_TMP, ALU_CP, END },  // 4 cycles, CP B
	/* b9 */ { A_ACT, REGS_TMP, ALU_CP, END },  // 4 cycles, CP C
	/* ba */ { A_ACT, REGS_TMP, ALU_CP, END },  // 4 cycles, CP D
	/* bb */ { A_ACT, REGS_TMP, ALU_CP, END },  // 4 cycles, CP E
	/* bc */ { A_ACT, REGS_TMP, ALU_CP, END },  // 4 cycles, CP H
	/* bd */ { A_ACT, REGS_TMP, ALU_CP, END },  // 4 cycles, CP L
	/* be */ { HL_OUT, READ, CHECK_WAIT, A_ACT, DB_TMP, ALU_CP, END },  // 7 cycles, CP (HL)
	/* bf */ { A_ACT, REGS_TMP, ALU_CP, END },  // 4 cycles, CP A

	/* c0 */ { RET_COND, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_Z, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_W, WZ_TO_PC, END },  // 5/11 cycles, RET NZ
	/* c1 */ { SP_OUT, INC_SP, READ, CHECK_WAIT, DB_R16L, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_R16H, END },  // 10 cycles, POP BC
	/* c2 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, JP_COND, END },  // 10 cycles, JP NZ,nn
	/* c3 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, WZ_TO_PC, END },  // 10 cycles, JMP nn
	/* c4 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, CALL_COND, DEC_SP, SP_OUT, PCH_DB, WRITE, CHECK_WAIT, DEC_SP, SP_OUT, PCL_DB, WRITE, CHECK_WAIT, WZ_TO_PC, END },  // 10/17 cycles, CALL NZ,nn
	/* c5 */ { X, DEC_SP, SP_OUT, R16H_DB, WRITE, CHECK_WAIT, DEC_SP, SP_OUT, R16L_DB, WRITE, CHECK_WAIT, END },  // 11 cycles, PUSH BC
	/* c6 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, A_ACT, DB_TMP, ALU_ADD, ALU_A, END },  // 7 cycles, ADD A,n
	{ 0 },
	/* c8 */ { RET_COND, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_Z, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_W, WZ_TO_PC, END },  // 5/11 cycles, RET Z
	/* c9 */ { SP_OUT, INC_SP, READ, CHECK_WAIT, DB_Z, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_W, WZ_TO_PC, END },  // 10 cycles, RET
	/* ca */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, JP_COND, END },  // 10 cycles, JP Z,nn
	/* cb */ { 0 },  // 4 cycles, CB prefix
	/* cc */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, CALL_COND, DEC_SP, SP_OUT, PCH_DB, WRITE, CHECK_WAIT, DEC_SP, SP_OUT, PCL_DB, WRITE, CHECK_WAIT, WZ_TO_PC, END },  // 10/17 cycles, CALL Z,nn
	/* cd */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, X, DEC_SP, SP_OUT, PCH_DB, WRITE, CHECK_WAIT, DEC_SP, SP_OUT, PCL_DB, WRITE, CHECK_WAIT, WZ_TO_PC, END },  // 17 cycles, CALL nn
	/* ce */ { PC_OUT, PC_INC, READ, CHECK_WAIT, A_ACT, DB_TMP, ALU_ADC, ALU_A, END },  // 7 cycles, ADC A,n
	{ 0 },
	/* 0xd0 */
	/* d0 */ { RET_COND, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_Z, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_W, WZ_TO_PC, END },  // 5/11 cycles, RET NC
	/* d1 */ { SP_OUT, INC_SP, READ, CHECK_WAIT, DB_R16L, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_R16H, END },  // 10 cycles, POP DE
	/* d2 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, JP_COND, END },  // 10 cycles, JP NC,nn
	/* d3 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, A_W, WZ_OUT, WZ_INC, A_DB, OUTPUT, CHECK_WAIT, END },  // 11 cycles, OUT (n), A
	/* d4 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, CALL_COND, DEC_SP, SP_OUT, PCH_DB, WRITE, CHECK_WAIT, DEC_SP, SP_OUT, PCL_DB, WRITE, CHECK_WAIT, WZ_TO_PC, END },  // 10/17 cycles, CALL NC,nn
	/* d5 */ { X, DEC_SP, SP_OUT, R16H_DB, WRITE, CHECK_WAIT, DEC_SP, SP_OUT, R16L_DB, WRITE, CHECK_WAIT, END },  // 11 cycles, PUSH DE
	/* d6 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, A_ACT, DB_TMP, ALU_SUB, ALU_A, END },  // 7 cycles, SUB n
	{ 0 },
	/* d8 */ { RET_COND, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_Z, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_W, WZ_TO_PC, END },  // 5/11 cycles, RET C
	{ 0 },
	/* da */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, JP_COND, END },  // 10 cycles, JP C,nn
	{ 0 },
	/* dc */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, CALL_COND, DEC_SP, SP_OUT, PCH_DB, WRITE, CHECK_WAIT, DEC_SP, SP_OUT, PCL_DB, WRITE, CHECK_WAIT, WZ_TO_PC, END },  // 10/17 cycles, CALL C,nn
	{ 0 },
	/* de */ { PC_OUT, PC_INC, READ, CHECK_WAIT, A_ACT, DB_TMP, ALU_SBC, ALU_A, END },  // 7 cycles, SBC n
	{ 0 },
	/* 0xe0 */
	/* e0 */ { RET_COND, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_Z, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_W, WZ_TO_PC, END },  // 5/11 cycles, RET PO
	/* e1 */ { SP_OUT, INC_SP, READ, CHECK_WAIT, DB_R16L, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_R16H, END },  // 10 cycles, POP HL
	/* e2 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, JP_COND, END },  // 10 cycles, JP PO,nn
	{ 0 },
	/* e4 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, CALL_COND, DEC_SP, SP_OUT, PCH_DB, WRITE, CHECK_WAIT, DEC_SP, SP_OUT, PCL_DB, WRITE, CHECK_WAIT, WZ_TO_PC, END },  // 10/17 cycles, CALL PO,nn
	/* e5 */ { X, DEC_SP, SP_OUT, R16H_DB, WRITE, CHECK_WAIT, DEC_SP, SP_OUT, R16L_DB, WRITE, CHECK_WAIT, END },  // 11 cycles, PUSH HL
	/* e6 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, A_ACT, DB_TMP, ALU_AND, ALU_A, END },  // 7 cycles, AND n
	{ 0 },
	/* e8 */ { RET_COND, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_Z, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_W, WZ_TO_PC, END },  // 5/11 cycles, RET PE
	{ 0 },
	/* ea */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, JP_COND, END },  // 10 cycles, JP PE,nn
	/* eb */ { EX_DE_HL, END },  // 4 cycles, EX DE,HL
	/* ec */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, CALL_COND, DEC_SP, SP_OUT, PCH_DB, WRITE, CHECK_WAIT, DEC_SP, SP_OUT, PCL_DB, WRITE, CHECK_WAIT, WZ_TO_PC, END },  // 10/17 cycles, CALL PE,nn
	{ 0 },
	/* ee */ { PC_OUT, PC_INC, READ, CHECK_WAIT, A_ACT, DB_TMP, ALU_XOR, ALU_A, END },  // 7 cycles, XOR n
	{ 0 },
	/* 0xf0 */
	/* f0 */ { RET_COND, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_Z, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_W, WZ_TO_PC, END },  // 5/11 cycles, RET P
	/* f1 */ { SP_OUT, INC_SP, READ, CHECK_WAIT, DB_R16L, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_R16H, END },  // 10 cycles, POP AF
	/* f2 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, JP_COND, END },  // 10 cycles, JP P,nn
	/* f3 */ { DI, END },  // 4 cycles, DI
	/* f4 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, CALL_COND, DEC_SP, SP_OUT, PCH_DB, WRITE, CHECK_WAIT, DEC_SP, SP_OUT, PCL_DB, WRITE, CHECK_WAIT, WZ_TO_PC, END },  // 10/17 cycles, CALL P,nn
	/* f5 */ { X, DEC_SP, SP_OUT, R16H_DB, WRITE, CHECK_WAIT, DEC_SP, SP_OUT, R16L_DB, WRITE, CHECK_WAIT, END },  // 11 cycles, PUSH AF
	/* f6 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, A_ACT, DB_TMP, ALU_OR, ALU_A, END },  // 7 cycles, OR n
	{ 0 },
	/* f8 */ { RET_COND, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_Z, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_W, WZ_TO_PC, END },  // 5/11 cycles, RET M
	{ 0 },
	/* fa */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, JP_COND, END },  // 10 cycles, JP M,nn
	/* fb */ { EI, END },  // 4 cycles, EI
	/* fc */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, CALL_COND, DEC_SP, SP_OUT, PCH_DB, WRITE, CHECK_WAIT, DEC_SP, SP_OUT, PCL_DB, WRITE, CHECK_WAIT, WZ_TO_PC, END },  // 10/17 cycles, CALL M,nn
	{ 0 },
	/* fe */ { PC_OUT, PC_INC, READ, CHECK_WAIT, A_ACT, DB_TMP, ALU_CP, END },  // 7 cycles, CP n
	{ 0 },

	/*****************************************************/
	/* CB prefixed instructions                          */
	/*****************************************************/

	/* cb 00 */ { REGS_TMP, ALU_RLC, ALU_REGS, END },  // 8 cycles, RLC B
	/* cb 01 */ { REGS_TMP, ALU_RLC, ALU_REGS, END },  // 8 cycles, RLC C
	/* cb 02 */ { REGS_TMP, ALU_RLC, ALU_REGS, END },  // 8 cycles, RLC D
	/* cb 03 */ { REGS_TMP, ALU_RLC, ALU_REGS, END },  // 8 cycles, RLC E
	/* cb 04 */ { REGS_TMP, ALU_RLC, ALU_REGS, END },  // 8 cycles, RLC H
	/* cb 05 */ { REGS_TMP, ALU_RLC, ALU_REGS, END },  // 8 cycles, RLC L
	/* cb 06 */ { HL_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RLC, ALU_DB, X2, WRITE, CHECK_WAIT, END },  // 15 cycles, RLC (HL)
	/* cb 07 */ { REGS_TMP, ALU_RLC, ALU_REGS, END },  // 8 cycles, RLC A
	/* cb 08 */ { REGS_TMP, ALU_RRC, ALU_REGS, END },  // 8 cycles, RRC B
	/* cb 09 */ { REGS_TMP, ALU_RRC, ALU_REGS, END },  // 8 cycles, RRC C
	/* cb 0a */ { REGS_TMP, ALU_RRC, ALU_REGS, END },  // 8 cycles, RRC D
	/* cb 0b */ { REGS_TMP, ALU_RRC, ALU_REGS, END },  // 8 cycles, RRC E
	/* cb 0c */ { REGS_TMP, ALU_RRC, ALU_REGS, END },  // 8 cycles, RRC H
	/* cb 0d */ { REGS_TMP, ALU_RRC, ALU_REGS, END },  // 8 cycles, RRC L
	/* cb 0e */ { HL_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RRC, ALU_DB, X2, WRITE, CHECK_WAIT, END },  // 15 cycles, RRC (HL)
	/* cb 0f */ { REGS_TMP, ALU_RRC, ALU_REGS, END },  // 8 cycles, RRC A

	/* cb 10 */ { REGS_TMP, ALU_RL, ALU_REGS, END },  // 8 cycles, RL B
	/* cb 11 */ { REGS_TMP, ALU_RL, ALU_REGS, END },  // 8 cycles, RL C
	/* cb 12 */ { REGS_TMP, ALU_RL, ALU_REGS, END },  // 8 cycles, RL D
	/* cb 13 */ { REGS_TMP, ALU_RL, ALU_REGS, END },  // 8 cycles, RL E
	/* cb 14 */ { REGS_TMP, ALU_RL, ALU_REGS, END },  // 8 cycles, RL H
	/* cb 15 */ { REGS_TMP, ALU_RL, ALU_REGS, END },  // 8 cycles, RL L
	/* cb 16 */ { HL_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RL, ALU_DB, X2, WRITE, CHECK_WAIT, END },  // 15 cycles, RL (HL)
	/* cb 17 */ { REGS_TMP, ALU_RL, ALU_REGS, END },  // 8 cycles, RL A
	/* cb 18 */ { REGS_TMP, ALU_RR, ALU_REGS, END },  // 8 cycles, RR B
	/* cb 19 */ { REGS_TMP, ALU_RR, ALU_REGS, END },  // 8 cycles, RR C
	/* cb 1a */ { REGS_TMP, ALU_RR, ALU_REGS, END },  // 8 cycles, RR D
	/* cb 1b */ { REGS_TMP, ALU_RR, ALU_REGS, END },  // 8 cycles, RR E
	/* cb 1c */ { REGS_TMP, ALU_RR, ALU_REGS, END },  // 8 cycles, RR H
	/* cb 1d */ { REGS_TMP, ALU_RR, ALU_REGS, END },  // 8 cycles, RR L
	/* cb 1e */ { HL_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RR, ALU_DB, X2, WRITE, CHECK_WAIT, END },  // 15 cycles, RR (HL)
	/* cb 1f */ { REGS_TMP, ALU_RR, ALU_REGS, END },  // 8 cycles, RR A

	/* cb 20 */ { REGS_TMP, ALU_SLA, ALU_REGS, END }, // 8 cycles, SLA B
	/* cb 21 */ { REGS_TMP, ALU_SLA, ALU_REGS, END }, // 8 cycles, SLA C
	/* cb 22 */ { REGS_TMP, ALU_SLA, ALU_REGS, END }, // 8 cycles, SLA D
	/* cb 23 */ { REGS_TMP, ALU_SLA, ALU_REGS, END }, // 8 cycles, SLA E
	/* cb 24 */ { REGS_TMP, ALU_SLA, ALU_REGS, END }, // 8 cycles, SLA H
	/* cb 25 */ { REGS_TMP, ALU_SLA, ALU_REGS, END }, // 8 cycles, SLA L
	/* cb 26 */ { HL_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SLA, ALU_DB, X2, WRITE, CHECK_WAIT, END },  // 15 cycles, SLA (HL)
	/* cb 27 */ { REGS_TMP, ALU_SLA, ALU_REGS, END }, // 8 cycles, SLA A
	/* cb 28 */ { REGS_TMP, ALU_SRA, ALU_REGS, END }, // 8 cycles, SRA B
	/* cb 29 */ { REGS_TMP, ALU_SRA, ALU_REGS, END }, // 8 cycles, SRA C
	/* cb 2a */ { REGS_TMP, ALU_SRA, ALU_REGS, END }, // 8 cycles, SRA D
	/* cb 2b */ { REGS_TMP, ALU_SRA, ALU_REGS, END }, // 8 cycles, SRA E
	/* cb 2c */ { REGS_TMP, ALU_SRA, ALU_REGS, END }, // 8 cycles, SRA H
	/* cb 2d */ { REGS_TMP, ALU_SRA, ALU_REGS, END }, // 8 cycles, SRA L
	/* cb 2e */ { HL_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SRA, ALU_DB, X2, WRITE, CHECK_WAIT, END },  // 15 cycles, SRA (HL)
	/* cb 2f */ { REGS_TMP, ALU_SRA, ALU_REGS, END }, // 8 cycles, SRA A

	/* cb 30 */ { REGS_TMP, ALU_SLL, ALU_REGS, END }, // 8 cycles, SLL B
	/* cb 31 */ { REGS_TMP, ALU_SLL, ALU_REGS, END }, // 8 cycles, SLL C
	/* cb 32 */ { REGS_TMP, ALU_SLL, ALU_REGS, END }, // 8 cycles, SLL D
	/* cb 33 */ { REGS_TMP, ALU_SLL, ALU_REGS, END }, // 8 cycles, SLL E
	/* cb 34 */ { REGS_TMP, ALU_SLL, ALU_REGS, END }, // 8 cycles, SLL H
	/* cb 35 */ { REGS_TMP, ALU_SLL, ALU_REGS, END }, // 8 cycles, SLL L
	/* cb 36 */ { HL_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SLL, ALU_DB, X2, WRITE, CHECK_WAIT, END },  // 15 cycles, SLL (HL)
	/* cb 37 */ { REGS_TMP, ALU_SLL, ALU_REGS, END }, // 8 cycles, SLL A
	/* cb 38 */ { REGS_TMP, ALU_SRL, ALU_REGS, END }, // 8 cycles, SRL B
	/* cb 39 */ { REGS_TMP, ALU_SRL, ALU_REGS, END }, // 8 cycles, SRL C
	/* cb 3a */ { REGS_TMP, ALU_SRL, ALU_REGS, END }, // 8 cycles, SRL D
	/* cb 3b */ { REGS_TMP, ALU_SRL, ALU_REGS, END }, // 8 cycles, SRL E
	/* cb 3c */ { REGS_TMP, ALU_SRL, ALU_REGS, END }, // 8 cycles, SRL H
	/* cb 3d */ { REGS_TMP, ALU_SRL, ALU_REGS, END }, // 8 cycles, SRL L
	/* cb 3e */ { HL_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SRL, ALU_DB, X2, WRITE, CHECK_WAIT, END },  // 15 cycles, SRL (HL)
	/* cb 3f */ { REGS_TMP, ALU_SRL, ALU_REGS, END }, // 8 cycles, SRL A

	/* cb 40 */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 0,B
	/* cb 41 */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 0,C
	/* cb 42 */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 0,D
	/* cb 43 */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 0,E
	/* cb 44 */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 0,H
	/* cb 45 */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 0,L
	/* cb 46 */ { HL_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 12 cycles, BIT 0,(HL)
	/* cb 47 */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 0,A
	/* cb 48 */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 1,B
	/* cb 49 */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 1,C
	/* cb 4a */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 1,D
	/* cb 4b */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 1,E
	/* cb 4c */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 1,H
	/* cb 4d */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 1,L
	/* cb 4e */ { HL_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 12 cycles, BIT 1,(HL)
	/* cb 4f */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 1,A

	/* cb 50 */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 2,B
	/* cb 51 */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 2,C
	/* cb 52 */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 2,D
	/* cb 53 */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 2,E
	/* cb 54 */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 2,H
	/* cb 55 */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 2,L
	/* cb 56 */ { HL_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 12 cycles, BIT 2,(HL)
	/* cb 57 */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 2,A
	/* cb 58 */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 3,B
	/* cb 59 */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 3,C
	/* cb 5a */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 3,D
	/* cb 5b */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 3,E
	/* cb 5c */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 3,H
	/* cb 5d */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 3,L
	/* cb 5e */ { HL_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 12 cycles, BIT 3,(HL)
	/* cb 5f */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 3,A

	/* cb 60 */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 4,B
	/* cb 61 */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 4,C
	/* cb 62 */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 4,D
	/* cb 63 */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 4,E
	/* cb 64 */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 4,H
	/* cb 65 */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 4,L
	/* cb 66 */ { HL_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 12 cycles, BIT 4,(HL)
	/* cb 67 */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 4,A
	/* cb 68 */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 5,B
	/* cb 69 */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 5,C
	/* cb 6a */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 5,D
	/* cb 6b */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 5,E
	/* cb 6c */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 5,H
	/* cb 6d */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 5,L
	/* cb 6e */ { HL_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 12 cycles, BIT 5,(HL)
	/* cb 6f */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 5,A

	/* cb 70 */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 6,B
	/* cb 71 */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 6,C
	/* cb 72 */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 6,D
	/* cb 73 */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 6,E
	/* cb 74 */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 6,H
	/* cb 75 */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 6,L
	/* cb 76 */ { HL_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 12 cycles, BIT 6,(HL)
	/* cb 77 */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 6,A
	/* cb 78 */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 7,B
	/* cb 79 */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 7,C
	/* cb 7a */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 7,D
	/* cb 7b */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 7,E
	/* cb 7c */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 7,H
	/* cb 7d */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 7,L
	/* cb 7e */ { HL_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 12 cycles, BIT 7,(HL)
	/* cb 7f */ { REGS_TMP, ALU_BIT, END },  // 8 cycles, BIT 7,A

	/* cb 80 */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 0,B
	/* cb 81 */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 0,C
	/* cb 82 */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 0,D
	/* cb 83 */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 0,E
	/* cb 84 */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 0,H
	/* cb 85 */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 0,L
	/* cb 86 */ { HL_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, ALU_DB, X2, WRITE, CHECK_WAIT, END },  // 15 cycles, RES 0,(HL)
	/* cb 87 */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 0,A
	/* cb 88 */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 1,B
	/* cb 89 */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 1,C
	/* cb 8a */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 1,D
	/* cb 8b */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 1,E
	/* cb 8c */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 1,H
	/* cb 8d */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 1,L
	/* cb 8e */ { HL_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, ALU_DB, X2, WRITE, CHECK_WAIT, END },  // 15 cycles, RES 1,(HL)
	/* cb 8f */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 1,A

	/* cb 90 */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 2,B
	/* cb 91 */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 2,C
	/* cb 92 */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 2,D
	/* cb 93 */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 2,E
	/* cb 94 */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 2,H
	/* cb 95 */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 2,L
	/* cb 96 */ { HL_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, ALU_DB, X2, WRITE, CHECK_WAIT, END },  // 15 cycles, RES 2,(HL)
	/* cb 97 */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 2,A
	/* cb 98 */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 3,B
	/* cb 99 */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 3,C
	/* cb 9a */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 3,D
	/* cb 9b */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 3,E
	/* cb 9c */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 3,H
	/* cb 9d */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 3,L
	/* cb 9e */ { HL_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, ALU_DB, X2, WRITE, CHECK_WAIT, END },  // 15 cycles, RES 3,(HL)
	/* cb 9f */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 3,A

	/* cb a0 */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 4,B
	/* cb a1 */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 4,C
	/* cb a2 */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 4,D
	/* cb a3 */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 4,E
	/* cb a4 */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 4,H
	/* cb a5 */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 4,L
	/* cb a6 */ { HL_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, ALU_DB, X2, WRITE, CHECK_WAIT, END },  // 15 cycles, RES 4,(HL)
	/* cb a7 */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 4,A
	/* cb a8 */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 5,B
	/* cb a9 */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 5,C
	/* cb aa */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 5,D
	/* cb ab */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 5,E
	/* cb ac */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 5,H
	/* cb ad */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 5,L
	/* cb ae */ { HL_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, ALU_DB, X2, WRITE, CHECK_WAIT, END },  // 15 cycles, RES 5,(HL)
	/* cb af */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 5,A

	/* cb b0 */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 6,B
	/* cb b1 */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 6,C
	/* cb b2 */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 6,D
	/* cb b3 */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 6,E
	/* cb b4 */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 6,H
	/* cb b5 */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 6,L
	/* cb b6 */ { HL_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, ALU_DB, X2, WRITE, CHECK_WAIT, END },  // 15 cycles, RES 6,(HL)
	/* cb b7 */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 6,A
	/* cb b8 */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 7,B
	/* cb b9 */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 7,C
	/* cb ba */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 7,D
	/* cb bb */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 7,E
	/* cb bc */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 7,H
	/* cb bd */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 7,L
	/* cb be */ { HL_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, ALU_DB, X2, WRITE, CHECK_WAIT, END },  // 15 cycles, RES 7,(HL)
	/* cb bf */ { REGS_TMP, ALU_RES, ALU_REGS, END },  // 8 cycles, RES 7,A

	/* cb c0 */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 0,B
	/* cb c1 */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 0,C
	/* cb c2 */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 0,D
	/* cb c3 */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 0,E
	/* cb c4 */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 0,H
	/* cb c5 */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 0,L
	/* cb c6 */ { HL_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, ALU_DB, X2, WRITE, CHECK_WAIT, END },  // 15 cycles, SET 0,(HL)
	/* cb c7 */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 0,A
	/* cb c8 */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 1,B
	/* cb c9 */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 1,C
	/* cb ca */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 1,D
	/* cb cb */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 1,E
	/* cb cc */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 1,H
	/* cb cd */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 1,L
	/* cb ce */ { HL_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, ALU_DB, X2, WRITE, CHECK_WAIT, END },  // 15 cycles, SET 1,(HL)
	/* cb cf */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 1,A

	/* cb d0 */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 2,B
	/* cb d1 */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 2,C
	/* cb d2 */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 2,D
	/* cb d3 */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 2,E
	/* cb d4 */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 2,H
	/* cb d5 */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 2,L
	/* cb d6 */ { HL_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, ALU_DB, X2, WRITE, CHECK_WAIT, END },  // 15 cycles, SET 2,(HL)
	/* cb d7 */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 2,A
	/* cb d8 */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 3,B
	/* cb d9 */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 3,C
	/* cb da */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 3,D
	/* cb db */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 3,E
	/* cb dc */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 3,H
	/* cb dd */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 3,L
	/* cb de */ { HL_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, ALU_DB, X2, WRITE, CHECK_WAIT, END },  // 15 cycles, SET 3,(HL)
	/* cb df */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 3,A

	/* cb e0 */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 4,B
	/* cb e1 */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 4,C
	/* cb e2 */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 4,D
	/* cb e3 */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 4,E
	/* cb e4 */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 4,H
	/* cb e5 */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 4,L
	/* cb e6 */ { HL_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, ALU_DB, X2, WRITE, CHECK_WAIT, END },  // 15 cycles, SET 4,(HL)
	/* cb e7 */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 4,A
	/* cb e8 */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 5,B
	/* cb e9 */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 5,C
	/* cb ea */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 5,D
	/* cb eb */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 5,E
	/* cb ec */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 5,H
	/* cb ed */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 5,L
	/* cb ee */ { HL_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, ALU_DB, X2, WRITE, CHECK_WAIT, END },  // 15 cycles, SET 5,(HL)
	/* cb ef */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 5,A

	/* cb f0 */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 6,B
	/* cb f1 */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 6,C
	/* cb f2 */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 6,D
	/* cb f3 */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 6,E
	/* cb f4 */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 6,H
	/* cb f5 */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 6,L
	/* cb f6 */ { HL_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, ALU_DB, X2, WRITE, CHECK_WAIT, END },  // 15 cycles, SET 6,(HL)
	/* cb f7 */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 6,A
	/* cb f8 */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 7,B
	/* cb f9 */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 7,C
	/* cb fa */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 7,D
	/* cb fb */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 7,E
	/* cb fc */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 7,H
	/* cb fd */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 7,L
	/* cb fe */ { HL_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, ALU_DB, X2, WRITE, CHECK_WAIT, END },  // 15 cycles, SET 7,(HL)
	/* cb ff */ { REGS_TMP, ALU_SET, ALU_REGS, END },  // 8 cycles, SET 7,A

	/*****************************************************/
	/* ED-prefixed instructions                          */
	/*****************************************************/

	/* 0x00 */
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* 0x10 */
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* 0x20 */
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* 0x30 */
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* 0x40 */
	{ 0 }, { 0 },
	/* ed 42 */ { SBC16, END },  // 15 cycles, SBC HL,BC
	/* ed 43 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, WZ_OUT, WZ_INC, R16L_DB, WRITE, CHECK_WAIT, WZ_OUT, R16H_DB, WRITE, CHECK_WAIT, END },  // 20 cycles, LD (nn),BC
	/* ed 44 */ { NEG, END },  // 8 cycles, NEG
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* ed 4a */ { ADC16, END },  // 15 cycles, ADC HL,BC
	/* ed 4b */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, WZ_OUT, WZ_INC, READ, CHECK_WAIT, DB_R16L, WZ_OUT, READ, CHECK_WAIT, DB_R16H, END },  // 20 cycles, LD BC,(nn)
	/* ed 4c */ { NEG, END },  // 8 cycles, NEG
	{ 0 }, { 0 }, { 0 },
	/* 0x50 */
	{ 0 }, { 0 },
	/* ed 52 */ { SBC16, END },  // 15 cycles SBC HL,DE
	/* ed 53 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, WZ_OUT, WZ_INC, R16L_DB, WRITE, CHECK_WAIT, WZ_OUT, R16H_DB, WRITE, CHECK_WAIT, END },  // 20 cycles, LD (nn),DE
	/* ed 54 */ { NEG, END },  // 8 cycles, NEG
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* ed 5a */ { ADC16, END },  // 15 cycles, ADC HL,DE
	/* ed 5b */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, WZ_OUT, WZ_INC, READ, CHECK_WAIT, DB_R16L, WZ_OUT, READ, CHECK_WAIT, DB_R16H, END },  // 20 cycles, LD DE,(nn)
	/* ed 5c */ { NEG, END },  // 8 cycles, NEG
	{ 0 }, { 0 }, { 0 },
	/* 0x60 */
	{ 0 }, { 0 },
	/* ed 62 */ { SBC16, END },  // 15 cycles, SBC HL,HL
	/* ed 63 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, WZ_OUT, WZ_INC, R16L_DB, WRITE, CHECK_WAIT, WZ_OUT, R16H_DB, WRITE, CHECK_WAIT, END },  // 20 cycles, LD (nn),HL
	/* ed 64 */ { NEG, END },  // 8 cycles, NEG
	{ 0 }, { 0 },
	/* ed 67 */ { HL_WZ, WZ_OUT, WZ_INC, READ, RRD, WRITE, END },  // 18 cycles, RRD
	{ 0 }, { 0 },
	/* ed 6a */ { ADC16, END },  // 15 cycles, ADC HL,HL
	/* ed 6b */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, WZ_OUT, WZ_INC, READ, CHECK_WAIT, DB_R16L, WZ_OUT, READ, CHECK_WAIT, DB_R16H, END },  // 20 cycles, LD HL,(nn)
	/* ed 6c */ { NEG, END },  // 8 cycles, NEG
	{ 0 }, { 0 },
	/* ed 6f */ { HL_WZ, WZ_OUT, WZ_INC, READ, RLD, WRITE, END },  // 18 cycles, RLD
	/* 0x70 */
	{ 0 }, { 0 },
	/* ed 72 */ { SBC16, END },  // 15 cycles, SBC HL,SP
	/* ed 73 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, WZ_OUT, WZ_INC, R16L_DB, WRITE, CHECK_WAIT, WZ_OUT, R16H_DB, WRITE, CHECK_WAIT, END },  // 20 cycles, LD (nn),SP
	/* ed 74 */ { NEG, END },  // 8 cycles, NEG
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* ed 7a */ { ADC16, END },  // 15 cycles, ADC HL,SP
	/* ed 7b */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, WZ_OUT, WZ_INC, READ, CHECK_WAIT, DB_R16L, WZ_OUT, READ, CHECK_WAIT, DB_R16H, END },  // 20 cycles, LD SP,(nn)
	/* ed 7c */ { NEG, END },  // 8 cycles, NEG
	{ 0 }, { 0 }, { 0 },
	/* 0x80 */
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* 0x90 */
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* 0xa0 */
	/* ed a0 */ { HL_OUT, READ, CHECK_WAIT, DE_OUT, WRITE, CHECK_WAIT, LDI, END },  // 16 cycles, LDI
	/* ed a1 */ { HL_OUT, READ, CHECK_WAIT, CPI, END },  // 16 cycles, CPI
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* ed a8 */ { HL_OUT, READ, CHECK_WAIT, DE_OUT, WRITE, CHECK_WAIT, LDD, END },  // 16 cycles, LDD
	/* ed a9 */ { HL_OUT, READ, CHECK_WAIT, CPD, END },  // 16 cycles, CPD
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* 0xb0 */
	/* ed b0 */ { HL_OUT, READ, CHECK_WAIT, DE_OUT, WRITE, CHECK_WAIT, LDI, REPEAT, END },  // 16/21 cycles, LDIR
	/* ed b1 */ { HL_OUT, READ, CHECK_WAIT, CPI, CPREPEAT, END },  // 16/21 cycles, CPIR
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* ed b8 */ { HL_OUT, READ, CHECK_WAIT, DE_OUT, WRITE, CHECK_WAIT, LDD, REPEAT, END },  // 16//21 cycles, LDDR
	/* ed b9 */ { HL_OUT, READ, CHECK_WAIT, CPD, CPREPEAT, END },  // 16/21 cycles, CPDR
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* 0xc0 */
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* 0xd0 */
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* 0xe0 */
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* 0xf0 */
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },

	/*****************************************************/
	/* DD/FD prefixed instructions                       */
	/* Almost equal to regular instructions              */
	/*****************************************************/

	/* dd/fd 00 */ { END },  // 8 cycles, NOP
	/* dd/fd 01 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_R16L, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_R16H, END },  // 14 cycles, LD BC,nn
	{ 0 },
	/* dd/fd 03 */ { INC_R16, END },  // 10 cycles, INC BC
	/* dd/fd 04 */ { REGD_TMP, ALU_INC, ALU_REGD, END },  // 8 cycles, INC B
	/* dd/fd 05 */ { REGD_TMP, ALU_DEC, ALU_REGD, END },  // 8 cycles, DEC B
	/* dd/fd 06 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_REG, END },  // 11 cycles, LD B,n
	/* dd/fd 07 */ { RLCA, END },  // 8 cycles, RLCA
	{ 0 },
	/* dd/fd 09 */ { ADD16, END },  // 15 cycles, ADD IX/IY,BC
	{ 0 },
	/* dd/fd 0b */ { DEC_R16, END },  // 10 cycles, DEC BC
	/* dd/fd 0c */ { REGD_TMP, ALU_INC, ALU_REGD, END },  // 8 cycles, INC C
	/* dd/fd 0d */ { REGD_TMP, ALU_DEC, ALU_REGD, END },  // 8 cycles, DEC C
	/* dd/fd 0e */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_REG, END },  // 11 cycles, LD C,n
	/* dd/fd 0f */ { RRCA, END },  // 8 cycles, RRCA
	/* 0x10 */
	{ 0 },
	/* dd/fd 11 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_R16L, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_R16H, END },  // 14 cycles, LD DE,nn
	/* dd/fd 12 */ { DE_WZ, WZ_OUT, WZ_INC, A_DB, WRITE, CHECK_WAIT, END },  // 7 cycles, LD (DE),A
	/* dd/fd 13 */ { INC_R16, END },  // 10 cycles, INC DE
	/* dd/fd 14 */ { REGD_TMP, ALU_INC, ALU_REGD, END },  // 8 cycles, INC D
	/* dd/fd 15 */ { REGD_TMP, ALU_DEC, ALU_REGD, END },  // 8 cycles, DEC D
	/* dd/fd 16 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_REG, END },  // 11 cycles, LD D,n
	/* dd/fd 17 */ { RLA, END },  // 8 cycles, RLA
	/* dd/fd 18 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, JR_COND, END },  // 16 cycles, JR n
	/* dd/fd 19 */ { ADD16, END },  // 11 cycles, ADD IX/IY,DE
	/* dd/fd 1a */ { DE_WZ, WZ_OUT, WZ_INC, READ, CHECK_WAIT, DB_A, END },  // 11 cycles, LD A,(DE)
	/* dd/fd 1b */ { DEC_R16, END },  // 10 cycles, DEC DE
	/* dd/fd 1c */ { REGD_TMP, ALU_INC, ALU_REGD, END },  // 8 cycles, INC E
	/* dd/fd 1d */ { REGD_TMP, ALU_DEC, ALU_REGD, END },  // 8 cycles, DEC E
	/* dd/fd 1e */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_REG, END },  // 11 cycles, LD E,n
	/* dd/fd 1f */ { RRA, END },  // 8 cycles, RRA

	/* dd/fd 20 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, JR_COND, END },  // 11/16 cycles, JR NZ,n
	/* dd/fd 21 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_R16L, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_R16H, END },  // 14 cycles, LD IX/IY,nn
	/* dd/fd 22 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, WZ_OUT, WZ_INC, L_DB, WRITE, CHECK_WAIT, WZ_OUT, H_DB, WRITE, CHECK_WAIT, END },  // 20 cycles, LD (nn),IX/IY
	/* dd/fd 23 */ { INC_R16, END },  // 10 cycles, INC IX/IY
	/* dd/fd 24 */ { REGD_TMP, ALU_INC, ALU_REGD, END },  // 8 cycles, INC IXh/IYh
	/* dd/fd 25 */ { REGD_TMP, ALU_DEC, ALU_REGD, END },  // 8 cycles, DEC IXh/IYh
	/* dd/fd 26 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_REG, END },  // 11 cycles, LD IXh/IYh,n
	/* dd/fd 27 */ { DAA, END },  // 8 cycles, DAA
	/* dd/fd 28 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, JR_COND, END },  // 11/16 cycles, JR Z,n
	/* dd/fd 29 */ { ADD16, END },  // 15 cycles, ADD IX/IY,IX/IY
	/* dd/fd 2a */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, WZ_OUT, WZ_INC, READ, CHECK_WAIT, DB_R16L, WZ_OUT, READ, CHECK_WAIT, DB_R16H, END },  // 20 cycles, LD IX/IY,(nn)
	/* dd/fd 2b */ { DEC_R16, END },  // 10 cycles, DEC IX/IY
	/* dd/fd 2c */ { REGD_TMP, ALU_INC, ALU_REGD, END },  // 8 cycles, INC IXl/IYl
	/* dd/fd 2d */ { REGD_TMP, ALU_DEC, ALU_REGD, END },  // 8 cycles, DEC IXl/IYl
	/* dd/fd 2e */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_REG, END },  // 11 cycles, LD IXl/IYl,n
	/* dd/fd 2f */ { CPL, END },  // 8 cycles, CPL
	/* 0x30 */
	{ 0 },
	/* dd/fd 31 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_R16L, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_R16H, END },  // 14 cycles, LD SP,nn
	/* dd/fd 32 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, WZ_OUT, WZ_INC, A_DB, WRITE, CHECK_WAIT, END },  // 17 cycles, LD (nn),A
	/* dd/fd 33 */ { INC_R16, END },  // 10 cycles, INC SP
	/* dd/fd 34 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_TMP, DISP_WZ5, WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_INC, ALU_DB, X, WZ_OUT, WRITE, CHECK_WAIT, END },  // 23 cycles, INC (IX/IY+dd)
	/* dd/fd 35 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_TMP, DISP_WZ5, WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_DEC, ALU_DB, X, WZ_OUT, WRITE, CHECK_WAIT, END },  // 23 cycles, DEC (IX/IY+dd)
	/* dd/fd 36 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_TMP, DISP_WZ2, PC_OUT, PC_INC, READ, CHECK_WAIT, WZ_OUT, WRITE, END },  // 19 cycles, LD (IX/IY+dd),n
	/* dd/fd 37 */ { SCF, END },  // 8 cycles, SCF
	{ 0 },
	/* dd/fd 39 */ { ADD16, END },  // 15 cycles, ADD IX/IY,SP
	/* dd/fd 3a */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, WZ_OUT, WZ_INC, READ, CHECK_WAIT, DB_A, END },  // 17 cycles, LD A,(nn)
	/* dd/fd 3b */ { DEC_R16, END },  // 10 cycles, DEC SP
	/* dd/fd 3c */ { REGD_TMP, ALU_INC, ALU_REGD, END },  // 8 cycles, INC A
	/* dd/fd 3d */ { REGD_TMP, ALU_DEC, ALU_REGD, END },  // 8 cycles, DEC A
	/* dd/fd 3e */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_REG, END },  // 11 cycles, LD A,n
	/* dd/fd 3f */ { CCF, END },  // 8 cycles, CCF

	/* dd/fd 40 */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD B,B
	/* dd/fd 41 */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD B,C
	/* dd/fd 42 */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD B,D
	/* dd/fd 43 */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD B,E
	/* dd/fd 44 */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD B,IXh/IYh
	/* dd/fd 45 */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD B,IXl/IYl
	/* dd/fd 46 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_TMP, DISP_WZ5, WZ_OUT, READ, CHECK_WAIT, DB_REG, END },  // 19 cycles, LD B,(IX/IY+dd)
	/* dd/fd 47 */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD B,A
	/* dd/fd 48 */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD C,B
	/* dd/fd 49 */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD C,C
	/* dd/fd 4a */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD C,D
	/* dd/fd 4b */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD C,E
	/* dd/fd 4c */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD C,IXh/IYh
	/* dd/fd 4d */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD C,IXl/IYl
	/* dd/fd 4e */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_TMP, DISP_WZ5, WZ_OUT, READ, CHECK_WAIT, DB_REG, END },  // 19 cycles, LD C,(IX/IY+dd)
	/* dd/fd 4f */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD C,A

	/* dd/fd 50 */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD D,B
	/* dd/fd 51 */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD D,C
	/* dd/fd 52 */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD D,D
	/* dd/fd 53 */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD D,E
	/* dd/fd 54 */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD D,IXh/IYh
	/* dd/fd 55 */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD D,IXl/IYl
	/* dd/fd 56 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_TMP, DISP_WZ5, WZ_OUT, READ, CHECK_WAIT, DB_REG, END },  // 19 cycles, LD D,(IX/IY+dd)
	/* dd/fd 57 */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD D,A
	/* dd/fd 58 */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD E,B
	/* dd/fd 59 */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD E,C
	/* dd/fd 5a */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD E,D
	/* dd/fd 5b */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD E,E
	/* dd/fd 5c */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD E,IXh/IYh
	/* dd/fd 5d */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD E,IXl/IYl
	/* dd/fd 5e */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_TMP, DISP_WZ5, WZ_OUT, READ, CHECK_WAIT, DB_REG, END },  // 19 cycles, LD E,(IX/IY+dd)
	/* dd/fd 5f */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD E,A

	/* dd/fd 60 */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD IXh/IYh,B
	/* dd/fd 61 */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD IXh/IYh,C
	/* dd/fd 62 */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD IXh/IYh,D
	/* dd/fd 63 */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD IXh/IYh,E
	/* dd/fd 64 */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD IXh/IYh,IXh/IYh
	/* dd/fd 65 */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD IXh/IYh,IXl/IYl
	/* dd/fd 66 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_TMP, DISP_WZ5, WZ_OUT, READ, CHECK_WAIT, DB_REG0, END },  // 19 cycles, LD H,(IX/IY+dd)
	/* dd/fd 67 */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD IXh/IYh,A
	/* dd/fd 68 */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD IXl/IYl,B
	/* dd/fd 69 */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD IXl/IYl,C
	/* dd/fd 6a */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD IXl/IYl,D
	/* dd/fd 6b */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD IXl/IYl,E
	/* dd/fd 6c */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD IXl/IYl,IXh/IYh
	/* dd/fd 6d */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD IXl/IYl,IXl/IYl
	/* dd/fd 6e */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_TMP, DISP_WZ5, WZ_OUT, READ, CHECK_WAIT, DB_REG0, END },  // 19 cycles, LD L,(IX/IY+dd)
	/* dd/fd 6f */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD IXl/IYl,A

	/* dd/fd 70 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_TMP, DISP_WZ5, WZ_OUT, REGS_DB, WRITE, CHECK_WAIT, END },  // 19 cycles, LD (IX/IY+dd),B
	/* dd/fd 71 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_TMP, DISP_WZ5, WZ_OUT, REGS_DB, WRITE, CHECK_WAIT, END },  // 19 cycles, LD (IX/IY+dd),C
	/* dd/fd 72 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_TMP, DISP_WZ5, WZ_OUT, REGS_DB, WRITE, CHECK_WAIT, END },  // 19 cycles, LD (IX/IY+dd),D
	/* dd/fd 73 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_TMP, DISP_WZ5, WZ_OUT, REGS_DB, WRITE, CHECK_WAIT, END },  // 19 cycles, LD (IX/IY+dd),E
	/* dd/fd 74 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_TMP, DISP_WZ5, WZ_OUT, REGS0_DB, WRITE, CHECK_WAIT, END },  // 19 cycles, LD (IX/IY+dd),H
	/* dd/fd 75 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_TMP, DISP_WZ5, WZ_OUT, REGS0_DB, WRITE, CHECK_WAIT, END },  // 19 cycles, LD (IX/IY+dd),L
	{ 0 },
	/* dd/fd 77 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_TMP, DISP_WZ5, WZ_OUT, REGS_DB, WRITE, CHECK_WAIT, END },  // 19 cycles, LD (IX/IY+dd),A
	/* dd/fd 78 */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD A,B
	/* dd/fd 79 */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD A,C
	/* dd/fd 7a */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD A,D
	/* dd/fd 7b */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD A,E
	/* dd/fd 7c */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD A,IXh/IYh
	/* dd/fd 7d */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD A,IXl/IYl
	/* dd/fd 7e */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_TMP, DISP_WZ5, WZ_OUT, READ, CHECK_WAIT, DB_REG, END },  // 19 cycles, LD A,(IX/IY+dd)
	/* dd/fd 7f */ { REGS_TMP, TMP_REG, END },  // 8 cycles, LD A,A

	/* dd/fd 80 */ { A_ACT, REGS_TMP, ALU_ADD, ALU_A, END },  // 8 cycles, ADD B
	/* dd/fd 81 */ { A_ACT, REGS_TMP, ALU_ADD, ALU_A, END },  // 8 cycles, ADD C
	/* dd/fd 82 */ { A_ACT, REGS_TMP, ALU_ADD, ALU_A, END },  // 8 cycles, ADD D
	/* dd/fd 83 */ { A_ACT, REGS_TMP, ALU_ADD, ALU_A, END },  // 8 cycles, ADD E
	/* dd/fd 84 */ { A_ACT, REGS_TMP, ALU_ADD, ALU_A, END },  // 8 cycles, ADD IXh/IYh
	/* dd/fd 85 */ { A_ACT, REGS_TMP, ALU_ADD, ALU_A, END },  // 8 cycles, ADD IXl/IYl
	/* dd/fd 86 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_TMP, DISP_WZ5, WZ_OUT, READ, CHECK_WAIT, A_ACT, DB_TMP, ALU_ADD, ALU_A, END },  // 19 cycles, ADD (IX/IY+dd)
	/* dd/fd 87 */ { A_ACT, REGS_TMP, ALU_ADD, ALU_A, END },  // 8 cycles, ADD A
	/* dd/fd 88 */ { A_ACT, REGS_TMP, ALU_ADC, ALU_A, END },  // 8 cycles, ADC B
	/* dd/fd 89 */ { A_ACT, REGS_TMP, ALU_ADC, ALU_A, END },  // 8 cycles, ADC C
	/* dd/fd 8a */ { A_ACT, REGS_TMP, ALU_ADC, ALU_A, END },  // 8 cycles, ADC D
	/* dd/fd 8b */ { A_ACT, REGS_TMP, ALU_ADC, ALU_A, END },  // 8 cycles, ADC E
	/* dd/fd 8c */ { A_ACT, REGS_TMP, ALU_ADC, ALU_A, END },  // 8 cycles, ADC IXh/IYh
	/* dd/fd 8d */ { A_ACT, REGS_TMP, ALU_ADC, ALU_A, END },  // 8 cycles, ADC IXl/IYl
	/* dd/fd 8e */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_TMP, DISP_WZ5, WZ_OUT, READ, CHECK_WAIT, A_ACT, DB_TMP, ALU_ADC, ALU_A, END },  // 19 cycles, ADC (IX/IY+dd)
	/* dd/fd 8f */ { A_ACT, REGS_TMP, ALU_ADC, ALU_A, END },  // 8 cycles, ADC A

	/* dd/fd 90 */ { A_ACT, REGS_TMP, ALU_SUB, ALU_A, END },  // 8 cycles, SUB B
	/* dd/fd 91 */ { A_ACT, REGS_TMP, ALU_SUB, ALU_A, END },  // 8 cycles, SUB C
	/* dd/fd 92 */ { A_ACT, REGS_TMP, ALU_SUB, ALU_A, END },  // 8 cycles, SUB D
	/* dd/fd 93 */ { A_ACT, REGS_TMP, ALU_SUB, ALU_A, END },  // 8 cycles, SUB E
	/* dd/fd 94 */ { A_ACT, REGS_TMP, ALU_SUB, ALU_A, END },  // 8 cycles, SUB IXh/IYh
	/* dd/fd 95 */ { A_ACT, REGS_TMP, ALU_SUB, ALU_A, END },  // 8 cycles, SUB IXl/IYl
	/* dd/fd 96 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_TMP, DISP_WZ5, WZ_OUT, READ, CHECK_WAIT, A_ACT, DB_TMP, ALU_SUB, ALU_A, END },  // 19 cycles, SUB (IX/IY+dd)
	/* dd/fd 97 */ { A_ACT, REGS_TMP, ALU_SUB, ALU_A, END },  // 8 cycles, SUB A
	/* dd/fd 98 */ { A_ACT, REGS_TMP, ALU_SBC, ALU_A, END },  // 8 cycles, SBC B
	/* dd/fd 99 */ { A_ACT, REGS_TMP, ALU_SBC, ALU_A, END },  // 8 cycles, SBC C
	/* dd/fd 9a */ { A_ACT, REGS_TMP, ALU_SBC, ALU_A, END },  // 8 cycles, SBC D
	/* dd/fd 9b */ { A_ACT, REGS_TMP, ALU_SBC, ALU_A, END },  // 8 cycles, SBC E
	/* dd/fd 9c */ { A_ACT, REGS_TMP, ALU_SBC, ALU_A, END },  // 8 cycles, SBC IXh/IYh
	/* dd/fd 9d */ { A_ACT, REGS_TMP, ALU_SBC, ALU_A, END },  // 8 cycles, SBC IXl/IYl
	/* dd/fd 9e */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_TMP, DISP_WZ5, WZ_OUT, READ, CHECK_WAIT, A_ACT, DB_TMP, ALU_SBC, ALU_A, END },  // 19 cycles, SBC (IX/IY+dd)
	/* dd/fd 9f */ { A_ACT, REGS_TMP, ALU_SBC, ALU_A, END },  // 8 cycles, SBC A

	/* dd/fd a0 */ { A_ACT, REGS_TMP, ALU_AND, ALU_A, END },  // 8 cycles, AND B
	/* dd/fd a1 */ { A_ACT, REGS_TMP, ALU_AND, ALU_A, END },  // 8 cycles, AND C
	/* dd/fd a2 */ { A_ACT, REGS_TMP, ALU_AND, ALU_A, END },  // 8 cycles, AND D
	/* dd/fd a3 */ { A_ACT, REGS_TMP, ALU_AND, ALU_A, END },  // 8 cycles, AND E
	/* dd/fd a4 */ { A_ACT, REGS_TMP, ALU_AND, ALU_A, END },  // 8 cycles, AND IXh/IYh
	/* dd/fd a5 */ { A_ACT, REGS_TMP, ALU_AND, ALU_A, END },  // 8 cycles, AND IXl/IYl
	/* dd/fd a6 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_TMP, DISP_WZ5, WZ_OUT, READ, CHECK_WAIT, A_ACT, DB_TMP, ALU_AND, ALU_A, END },  // 19 cycles, AND (IX/IY+dd)
	/* dd/fd a7 */ { A_ACT, REGS_TMP, ALU_AND, ALU_A, END },  // 4 cycles, AND A
	/* dd/fd a8 */ { A_ACT, REGS_TMP, ALU_XOR, ALU_A, END },  // 8 cycles, XOR B
	/* dd/fd a9 */ { A_ACT, REGS_TMP, ALU_XOR, ALU_A, END },  // 8 cycles, XOR C
	/* dd/fd aa */ { A_ACT, REGS_TMP, ALU_XOR, ALU_A, END },  // 8 cycles, XOR D
	/* dd/fd ab */ { A_ACT, REGS_TMP, ALU_XOR, ALU_A, END },  // 8 cycles, XOR E
	/* dd/fd ac */ { A_ACT, REGS_TMP, ALU_XOR, ALU_A, END },  // 8 cycles, XOR IXh/IYh
	/* dd/fd ad */ { A_ACT, REGS_TMP, ALU_XOR, ALU_A, END },  // 8 cycles, XOR IXl/IYl
	/* dd/fd ae */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_TMP, DISP_WZ5, WZ_OUT, READ, CHECK_WAIT, A_ACT, DB_TMP, ALU_XOR, ALU_A, END },  // 19 cycles, XOR (IX/IY+dd)
	/* dd/fd af */ { A_ACT, REGS_TMP, ALU_XOR, ALU_A, END },  // 8 cycles, XOR A

	/* dd/fd b0 */ { A_ACT, REGS_TMP, ALU_OR, ALU_A, END },  // 8 cycles, OR B
	/* dd/fd b1 */ { A_ACT, REGS_TMP, ALU_OR, ALU_A, END },  // 8 cycles, OR C
	/* dd/fd b2 */ { A_ACT, REGS_TMP, ALU_OR, ALU_A, END },  // 8 cycles, OR D
	/* dd/fd b3 */ { A_ACT, REGS_TMP, ALU_OR, ALU_A, END },  // 8 cycles, OR E
	/* dd/fd b4 */ { A_ACT, REGS_TMP, ALU_OR, ALU_A, END },  // 8 cycles, OR IXh/IYh
	/* dd/fd b5 */ { A_ACT, REGS_TMP, ALU_OR, ALU_A, END },  // 8 cycles, OR IXl/IYl
	/* dd/fd b6 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_TMP, DISP_WZ5, WZ_OUT, READ, CHECK_WAIT, A_ACT, DB_TMP, ALU_OR, ALU_A, END },  // 19 cycles, OR (IX/IY+dd)
	/* dd/fd b7 */ { A_ACT, REGS_TMP, ALU_OR, ALU_A, END },  // 8 cycles, OR A
	/* dd/fd b8 */ { A_ACT, REGS_TMP, ALU_CP, END },  // 8 cycles, CP B
	/* dd/fd b9 */ { A_ACT, REGS_TMP, ALU_CP, END },  // 8 cycles, CP C
	/* dd/fd ba */ { A_ACT, REGS_TMP, ALU_CP, END },  // 8 cycles, CP D
	/* dd/fd bb */ { A_ACT, REGS_TMP, ALU_CP, END },  // 8 cycles, CP E
	/* dd/fd bc */ { A_ACT, REGS_TMP, ALU_CP, END },  // 8 cycles, CP IXh/IYh
	/* dd/fd bd */ { A_ACT, REGS_TMP, ALU_CP, END },  // 8 cycles, CP IXl/IYl
	/* dd/fd be */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_TMP, DISP_WZ5, WZ_OUT, READ, CHECK_WAIT, A_ACT, DB_TMP, ALU_CP, END },  // 19 cycles, CP (IX/IY+dd)
	/* dd/fd bf */ { A_ACT, REGS_TMP, ALU_CP, END },  // 8 cycles, CP A

	/* dd/fd c0 */ { RET_COND, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_Z, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_W, WZ_TO_PC, END },  // 9/15 cycles, RET NZ
	/* dd/fd c1 */ { SP_OUT, INC_SP, READ, CHECK_WAIT, DB_R16L, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_R16H, END },  // 14 cycles, POP BC
	/* dd/fd c2 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, JP_COND, END },  // 14 cycles, JP NZ,nn
	/* dd/fd c3 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, WZ_TO_PC, END },  // 14 cycles, JMP nn
	/* dd/fd c4 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, CALL_COND, DEC_SP, SP_OUT, PCH_DB, WRITE, CHECK_WAIT, DEC_SP, SP_OUT, PCL_DB, WRITE, CHECK_WAIT, WZ_TO_PC, END },  // 14/21 cycles, CALL NZ,nn
	/* dd/fd c5 */ { X, DEC_SP, SP_OUT, R16H_DB, WRITE, CHECK_WAIT, DEC_SP, SP_OUT, R16L_DB, WRITE, CHECK_WAIT, END },  // 15 cycles, PUSH BC
	/* dd/fd c6 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, A_ACT, DB_TMP, ALU_ADD, ALU_A, END },  // 11 cycles, ADD A,n
	{ 0 },
	/* dd/fd c8 */ { RET_COND, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_Z, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_W, WZ_TO_PC, END },  // 9/15 cycles, RET Z
	/* dd/fd c9 */ { SP_OUT, INC_SP, READ, CHECK_WAIT, DB_Z, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_W, WZ_TO_PC, END },  // 14 cycles, RET
	/* dd/fd ca */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, JP_COND, END },  // 14 cycles, JP Z,nn
	/* dd/fd cb */ { 0 },  // 8 cycles, DD/FD + CB prefix
	/* dd/fd cc */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, CALL_COND, DEC_SP, SP_OUT, PCH_DB, WRITE, CHECK_WAIT, DEC_SP, SP_OUT, PCL_DB, WRITE, CHECK_WAIT, WZ_TO_PC, END },  // 14/21 cycles, CALL Z,nn
	/* dd/fd cd */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, X, DEC_SP, SP_OUT, PCH_DB, WRITE, CHECK_WAIT, DEC_SP, SP_OUT, PCL_DB, WRITE, CHECK_WAIT, WZ_TO_PC, END },  // 21 cycles, CALL nn
	/* dd/fd ce */ { PC_OUT, PC_INC, READ, CHECK_WAIT, A_ACT, DB_TMP, ALU_ADC, ALU_A, END },  // 11 cycles, ADC A,n
	{ 0 },
	/* 0xd0 */
	/* dd/fd d0 */ { RET_COND, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_Z, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_W, WZ_TO_PC, END },  // 9/15 cycles, RET NC
	/* dd/fd d1 */ { SP_OUT, INC_SP, READ, CHECK_WAIT, DB_R16L, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_R16H, END },  // 14 cycles, POP DE
	/* dd/fd d2 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, JP_COND, END },  // 14 cycles, JP NC,nn
	/* dd/fd d3 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, A_W, WZ_OUT, WZ_INC, A_DB, OUTPUT, CHECK_WAIT, END },  // 15 cycles, OUT (n), A
	/* dd/fd d4 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, CALL_COND, DEC_SP, SP_OUT, PCH_DB, WRITE, CHECK_WAIT, DEC_SP, SP_OUT, PCL_DB, WRITE, CHECK_WAIT, WZ_TO_PC, END },  // 14/21 cycles, CALL NC,nn
	/* dd/fd d5 */ { X, DEC_SP, SP_OUT, R16H_DB, WRITE, CHECK_WAIT, DEC_SP, SP_OUT, R16L_DB, WRITE, CHECK_WAIT, END },  // 15 cycles, PUSH DE
	/* dd/fd d6 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, A_ACT, DB_TMP, ALU_SUB, ALU_A, END },  // 11 cycles, SUB n
	{ 0 },
	/* dd/fd d8 */ { RET_COND, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_Z, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_W, WZ_TO_PC, END },  // 9/15 cycles, RET C
	{ 0 },
	/* dd/fd da */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, JP_COND, END },  // 14 cycles, JP C,nn
	{ 0 },
	/* dd/fd dc */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, CALL_COND, DEC_SP, SP_OUT, PCH_DB, WRITE, CHECK_WAIT, DEC_SP, SP_OUT, PCL_DB, WRITE, CHECK_WAIT, WZ_TO_PC, END },  // 14/21 cycles, CALL C,nn
	{ 0 },
	/* dd/fd de */ { PC_OUT, PC_INC, READ, CHECK_WAIT, A_ACT, DB_TMP, ALU_SBC, ALU_A, END },  // 11 cycles, SBC n
	{ 0 },
	/* 0xe0 */
	/* dd/fd e0 */ { RET_COND, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_Z, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_W, WZ_TO_PC, END },  // 9/15 cycles, RET PO
	/* dd/fd e1 */ { SP_OUT, INC_SP, READ, CHECK_WAIT, DB_R16L, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_R16H, END },  // 14 cycles, POP IX/IY
	/* dd/fd e2 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, JP_COND, END },  // 14 cycles, JP PO,nn
	{ 0 },
	/* dd/fd e4 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, CALL_COND, DEC_SP, SP_OUT, PCH_DB, WRITE, CHECK_WAIT, DEC_SP, SP_OUT, PCL_DB, WRITE, CHECK_WAIT, WZ_TO_PC, END },  // 14/21 cycles, CALL PO,nn
	/* dd/fd e5 */ { X, DEC_SP, SP_OUT, R16H_DB, WRITE, CHECK_WAIT, DEC_SP, SP_OUT, R16L_DB, WRITE, CHECK_WAIT, END },  // 15 cycles, PUSH IX/IY
	/* dd/fd e6 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, A_ACT, DB_TMP, ALU_AND, ALU_A, END },  // 11 cycles, AND n
	{ 0 },
	/* dd/fd e8 */ { RET_COND, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_Z, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_W, WZ_TO_PC, END },  // 9/15 cycles, RET PE
	{ 0 },
	/* dd/fd ea */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, JP_COND, END },  // 14 cycles, JP PE,nn
	/* dd/fd eb */ { EX_DE_HL, END },  // 8 cycles, EX DE,HL
	/* dd/fd ec */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, CALL_COND, DEC_SP, SP_OUT, PCH_DB, WRITE, CHECK_WAIT, DEC_SP, SP_OUT, PCL_DB, WRITE, CHECK_WAIT, WZ_TO_PC, END },  // 14/21 cycles, CALL PE,nn
	{ 0 },
	/* dd/fd ee */ { PC_OUT, PC_INC, READ, CHECK_WAIT, A_ACT, DB_TMP, ALU_XOR, ALU_A, END },  // 11 cycles, XOR n
	{ 0 },
	/* 0xf0 */
	/* dd/fd f0 */ { RET_COND, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_Z, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_W, WZ_TO_PC, END },  // 9/15 cycles, RET P
	/* dd/fd f1 */ { SP_OUT, INC_SP, READ, CHECK_WAIT, DB_R16L, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_R16H, END },  // 14 cycles, POP AF
	/* dd/fd f2 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, JP_COND, END },  // 14 cycles, JP P,nn
	/* dd/fd f3 */ { DI, END },  // 8 cycles, DI
	/* dd/fd f4 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, CALL_COND, DEC_SP, SP_OUT, PCH_DB, WRITE, CHECK_WAIT, DEC_SP, SP_OUT, PCL_DB, WRITE, CHECK_WAIT, WZ_TO_PC, END },  // 14/21 cycles, CALL P,nn
	/* dd/fd f5 */ { X, DEC_SP, SP_OUT, R16H_DB, WRITE, CHECK_WAIT, DEC_SP, SP_OUT, R16L_DB, WRITE, CHECK_WAIT, END },  // 15 cycles, PUSH AF
	/* dd/fd f6 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, A_ACT, DB_TMP, ALU_OR, ALU_A, END },  // 11 cycles, OR n
	{ 0 },
	/* dd/fd f8 */ { RET_COND, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_Z, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_W, WZ_TO_PC, END },  // 9/15 cycles, RET M
	{ 0 },
	/* dd/fd fa */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, JP_COND, END },  // 14 cycles, JP M,nn
	/* dd/fd fb */ { EI, END },  // 8 cycles, EI
	/* dd/fd fc */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, CALL_COND, DEC_SP, SP_OUT, PCH_DB, WRITE, CHECK_WAIT, DEC_SP, SP_OUT, PCL_DB, WRITE, CHECK_WAIT, WZ_TO_PC, END },  // 14/21 cycles, CALL M,nn
	{ 0 },
	/* dd/fd fe */ { PC_OUT, PC_INC, READ, CHECK_WAIT, A_ACT, DB_TMP, ALU_CP, END },  // 11 cycles, CP n
	{ 0 },

	/*****************************************************/
	/* DD/FD + CB prefixed instructions                  */
	/*****************************************************/

	/* dd/fd cb dd 00 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RLC, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RLC (IX/IY+dd),B
	/* dd/fd cb dd 01 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RLC, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RLC (IX/IY+dd),C
	/* dd/fd cb dd 02 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RLC, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RLC (IX/IY+dd),D
	/* dd/fd cb dd 03 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RLC, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RLC (IX/IY+dd),E
	/* dd/fd cb dd 04 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RLC, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RLC (IX/IY+dd),H
	/* dd/fd cb dd 05 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RLC, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RLC (IX/IY+dd),L
	/* dd/fd cb dd 06 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RLC, X2, ALU_DB, WRITE, CHECK_WAIT, END },  // 23 cycles, RLC (IX/IY+dd)
	/* dd/fd cb dd 07 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RLC, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RLC (IX/IY+dd),A
	/* dd/fd cb dd 08 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RRC, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RRC (IX/IY+dd),B
	/* dd/fd cb dd 09 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RRC, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RRC (IX/IY+dd),C
	/* dd/fd cb dd 0a */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RRC, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RRC (IX/IY+dd),D
	/* dd/fd cb dd 0b */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RRC, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RRC (IX/IY+dd),E
	/* dd/fd cb dd 0c */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RRC, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RRC (IX/IY+dd),H
	/* dd/fd cb dd 0d */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RRC, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RRC (IX/IY+dd),L
	/* dd/fd cb dd 0e */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RRC, X2, ALU_DB, WRITE, CHECK_WAIT, END },  // 23 cycles, RRC (IX/IY+dd)
	/* dd/fd cb dd 0f */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RRC, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RRC (IX/IY+dd),A

	/* dd/fd cb dd 10 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RL, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RL (IX/IY+dd),B
	/* dd/fd cb dd 11 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RL, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RL (IX/IY+dd),C
	/* dd/fd cb dd 12 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RL, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RL (IX/IY+dd),D
	/* dd/fd cb dd 13 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RL, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RL (IX/IY+dd),E
	/* dd/fd cb dd 14 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RL, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RL (IX/IY+dd),H
	/* dd/fd cb dd 15 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RL, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RL (IX/IY+dd),L
	/* dd/fd cb dd 16 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RL, X2, ALU_DB, WRITE, CHECK_WAIT, END },  // 23 cycles, RL (IX/IY+dd)
	/* dd/fd cb dd 17 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RL, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RL (IX/IY+dd),A
	/* dd/fd cb dd 18 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RR, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RR (IX/IY+dd),B
	/* dd/fd cb dd 19 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RR, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RR (IX/IY+dd),C
	/* dd/fd cb dd 1a */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RR, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RR (IX/IY+dd),D
	/* dd/fd cb dd 1b */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RR, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RR (IX/IY+dd),E
	/* dd/fd cb dd 1c */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RR, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RR (IX/IY+dd),H
	/* dd/fd cb dd 1d */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RR, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RR (IX/IY+dd),L
	/* dd/fd cb dd 1e */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RR, X2, ALU_DB, WRITE, CHECK_WAIT, END },  // 23 cycles, RR (IX/IY+dd)
	/* dd/fd cb dd 1f */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RR, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RR (IX/IY+dd),A

	/* dd/fd cb dd 20 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SLA, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SLA (IX/IY+dd),B
	/* dd/fd cb dd 21 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SLA, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SLA (IX/IY+dd),C
	/* dd/fd cb dd 22 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SLA, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SLA (IX/IY+dd),D
	/* dd/fd cb dd 23 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SLA, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SLA (IX/IY+dd),E
	/* dd/fd cb dd 24 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SLA, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SLA (IX/IY+dd),H
	/* dd/fd cb dd 25 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SLA, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SLA (IX/IY+dd),L
	/* dd/fd cb dd 26 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SLA, X2, ALU_DB, WRITE, CHECK_WAIT, END },  // 23 cycles, SLA (IX/IY+dd)
	/* dd/fd cb dd 27 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SLA, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SLA (IX/IY+dd),A
	/* dd/fd cb dd 28 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SRA, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SRA (IX/IY+dd),B
	/* dd/fd cb dd 29 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SRA, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SRA (IX/IY+dd),C
	/* dd/fd cb dd 2a */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SRA, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SRA (IX/IY+dd),D
	/* dd/fd cb dd 2b */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SRA, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SRA (IX/IY+dd),E
	/* dd/fd cb dd 2c */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SRA, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SRA (IX/IY+dd),H
	/* dd/fd cb dd 2d */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SRA, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SRA (IX/IY+dd),L
	/* dd/fd cb dd 2e */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SRA, X2, ALU_DB, WRITE, CHECK_WAIT, END },  // 23 cycles, SRA (IX/IY+dd)
	/* dd/fd cb dd 2f */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SRA, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SRA (IX/IY+dd),A

	/* dd/fd cb dd 30 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SLL, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SLL (IX/IY+dd),B
	/* dd/fd cb dd 31 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SLL, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SLL (IX/IY+dd),C
	/* dd/fd cb dd 32 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SLL, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SLL (IX/IY+dd),D
	/* dd/fd cb dd 33 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SLL, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SLL (IX/IY+dd),E
	/* dd/fd cb dd 34 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SLL, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SLL (IX/IY+dd),H
	/* dd/fd cb dd 35 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SLL, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SLL (IX/IY+dd),L
	/* dd/fd cb dd 36 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SLL, X2, ALU_DB, WRITE, CHECK_WAIT, END },  // 23 cycles, SLL (IX/IY+dd)
	/* dd/fd cb dd 37 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SLL, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SLL (IX/IY+dd),A
	/* dd/fd cb dd 38 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SRL, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SRL (IX/IY+dd),B
	/* dd/fd cb dd 39 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SRL, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SRL (IX/IY+dd),C
	/* dd/fd cb dd 3a */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SRL, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SRL (IX/IY+dd),D
	/* dd/fd cb dd 3b */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SRL, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SRL (IX/IY+dd),E
	/* dd/fd cb dd 3c */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SRL, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SRL (IX/IY+dd),H
	/* dd/fd cb dd 3d */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SRL, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SRL (IX/IY+dd),L
	/* dd/fd cb dd 3e */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SRL, X2, ALU_DB, WRITE, CHECK_WAIT, END },  // 23 cycles, SRL (IX/IY+dd)
	/* dd/fd cb dd 3f */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SRL, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SRL (IX/IY+dd),A

	/* dd/fd cb dd 40 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 0,(IX/IY+dd)*
	/* dd/fd cb dd 41 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 0,(IX/IY+dd)*
	/* dd/fd cb dd 42 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 0,(IX/IY+dd)*
	/* dd/fd cb dd 43 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 0,(IX/IY+dd)*
	/* dd/fd cb dd 44 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 0,(IX/IY+dd)*
	/* dd/fd cb dd 45 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 0,(IX/IY+dd)*
	/* dd/fd cb dd 46 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 0,(IX/IY+dd)
	/* dd/fd cb dd 47 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 0,(IX/IY+dd)*
	/* dd/fd cb dd 48 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 1,(IX/IY+dd)*
	/* dd/fd cb dd 49 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 1,(IX/IY+dd)*
	/* dd/fd cb dd 4a */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 1,(IX/IY+dd)*
	/* dd/fd cb dd 4b */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 1,(IX/IY+dd)*
	/* dd/fd cb dd 4c */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 1,(IX/IY+dd)*
	/* dd/fd cb dd 4d */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 1,(IX/IY+dd)*
	/* dd/fd cb dd 4e */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 1,(IX/IY+dd)
	/* dd/fd cb dd 4f */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 1,(IX/IY+dd)*

	/* dd/fd cb dd 50 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 2,(IX/IY+dd)*
	/* dd/fd cb dd 51 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 2,(IX/IY+dd)*
	/* dd/fd cb dd 52 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 2,(IX/IY+dd)*
	/* dd/fd cb dd 53 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 2,(IX/IY+dd)*
	/* dd/fd cb dd 54 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 2,(IX/IY+dd)*
	/* dd/fd cb dd 55 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 2,(IX/IY+dd)*
	/* dd/fd cb dd 56 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 2,(IX/IY+dd)
	/* dd/fd cb dd 57 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 2,(IX/IY+dd)*
	/* dd/fd cb dd 58 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 3,(IX/IY+dd)*
	/* dd/fd cb dd 59 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 3,(IX/IY+dd)*
	/* dd/fd cb dd 5a */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 3,(IX/IY+dd)*
	/* dd/fd cb dd 5b */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 3,(IX/IY+dd)*
	/* dd/fd cb dd 5c */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 3,(IX/IY+dd)*
	/* dd/fd cb dd 5d */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 3,(IX/IY+dd)*
	/* dd/fd cb dd 5e */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 3,(IX/IY+dd)
	/* dd/fd cb dd 5f */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 3,(IX/IY+dd)*

	/* dd/fd cb dd 60 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 4,(IX/IY+dd)*
	/* dd/fd cb dd 61 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 4,(IX/IY+dd)*
	/* dd/fd cb dd 62 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 4,(IX/IY+dd)*
	/* dd/fd cb dd 63 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 4,(IX/IY+dd)*
	/* dd/fd cb dd 64 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 4,(IX/IY+dd)*
	/* dd/fd cb dd 65 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 4,(IX/IY+dd)*
	/* dd/fd cb dd 66 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 4,(IX/IY+dd)
	/* dd/fd cb dd 67 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 4,(IX/IY+dd)*
	/* dd/fd cb dd 68 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 5,(IX/IY+dd)*
	/* dd/fd cb dd 69 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 5,(IX/IY+dd)*
	/* dd/fd cb dd 6a */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 5,(IX/IY+dd)*
	/* dd/fd cb dd 6b */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 5,(IX/IY+dd)*
	/* dd/fd cb dd 6c */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 5,(IX/IY+dd)*
	/* dd/fd cb dd 6d */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 5,(IX/IY+dd)*
	/* dd/fd cb dd 6e */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 5,(IX/IY+dd)
	/* dd/fd cb dd 6f */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 5,(IX/IY+dd)*

	/* dd/fd cb dd 70 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 6,(IX/IY+dd)*
	/* dd/fd cb dd 71 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 6,(IX/IY+dd)*
	/* dd/fd cb dd 72 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 6,(IX/IY+dd)*
	/* dd/fd cb dd 73 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 6,(IX/IY+dd)*
	/* dd/fd cb dd 74 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 6,(IX/IY+dd)*
	/* dd/fd cb dd 75 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 6,(IX/IY+dd)*
	/* dd/fd cb dd 76 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 6,(IX/IY+dd)
	/* dd/fd cb dd 77 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 6,(IX/IY+dd)*
	/* dd/fd cb dd 78 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 7,(IX/IY+dd)*
	/* dd/fd cb dd 79 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 7,(IX/IY+dd)*
	/* dd/fd cb dd 7a */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 7,(IX/IY+dd)*
	/* dd/fd cb dd 7b */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 7,(IX/IY+dd)*
	/* dd/fd cb dd 7c */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 7,(IX/IY+dd)*
	/* dd/fd cb dd 7d */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 7,(IX/IY+dd)*
	/* dd/fd cb dd 7e */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 7,(IX/IY+dd)
	/* dd/fd cb dd 7f */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_BIT, X, END },  // 20 cycles, BIT 7,(IX/IY+dd)*

	/* dd/fd cb dd 80 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 0,(IX/IY+dd),B
	/* dd/fd cb dd 81 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 0,(IX/IY+dd),C
	/* dd/fd cb dd 82 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 0,(IX/IY+dd),D
	/* dd/fd cb dd 83 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 0,(IX/IY+dd),E
	/* dd/fd cb dd 84 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 0,(IX/IY+dd),H
	/* dd/fd cb dd 85 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 0,(IX/IY+dd),L
	/* dd/fd cb dd 86 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 0,(IX/IY+dd)
	/* dd/fd cb dd 87 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 0,(IX/IY+dd),A
	/* dd/fd cb dd 88 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 1,(IX/IY+dd),B
	/* dd/fd cb dd 89 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 1,(IX/IY+dd),C
	/* dd/fd cb dd 8a */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 1,(IX/IY+dd),D
	/* dd/fd cb dd 8b */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 1,(IX/IY+dd),E
	/* dd/fd cb dd 8c */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 1,(IX/IY+dd),H
	/* dd/fd cb dd 8d */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 1,(IX/IY+dd),L
	/* dd/fd cb dd 8e */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 1,(IX/IY+dd)
	/* dd/fd cb dd 8f */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 1,(IX/IY+dd),A

	/* dd/fd cb dd 90 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 2,(IX/IY+dd),B
	/* dd/fd cb dd 91 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 2,(IX/IY+dd),C
	/* dd/fd cb dd 92 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 2,(IX/IY+dd),D
	/* dd/fd cb dd 93 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 2,(IX/IY+dd),E
	/* dd/fd cb dd 94 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 2,(IX/IY+dd),H
	/* dd/fd cb dd 95 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 2,(IX/IY+dd),L
	/* dd/fd cb dd 96 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 2,(IX/IY+dd)
	/* dd/fd cb dd 97 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 2,(IX/IY+dd),A
	/* dd/fd cb dd 98 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 3,(IX/IY+dd),B
	/* dd/fd cb dd 99 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 3,(IX/IY+dd),C
	/* dd/fd cb dd 9a */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 3,(IX/IY+dd),D
	/* dd/fd cb dd 9b */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 3,(IX/IY+dd),E
	/* dd/fd cb dd 9c */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 3,(IX/IY+dd),H
	/* dd/fd cb dd 9d */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 3,(IX/IY+dd),L
	/* dd/fd cb dd 9e */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 3,(IX/IY+dd)
	/* dd/fd cb dd 9f */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 3,(IX/IY+dd),A

	/* dd/fd cb dd a0 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 4,(IX/IY+dd),B
	/* dd/fd cb dd a1 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 4,(IX/IY+dd),C
	/* dd/fd cb dd a2 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 4,(IX/IY+dd),D
	/* dd/fd cb dd a3 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 4,(IX/IY+dd),E
	/* dd/fd cb dd a4 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 4,(IX/IY+dd),H
	/* dd/fd cb dd a5 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 4,(IX/IY+dd),L
	/* dd/fd cb dd a6 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 4,(IX/IY+dd)
	/* dd/fd cb dd a7 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 4,(IX/IY+dd),A
	/* dd/fd cb dd a8 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 5,(IX/IY+dd),B
	/* dd/fd cb dd a9 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 5,(IX/IY+dd),C
	/* dd/fd cb dd aa */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 5,(IX/IY+dd),D
	/* dd/fd cb dd ab */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 5,(IX/IY+dd),E
	/* dd/fd cb dd ac */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 5,(IX/IY+dd),H
	/* dd/fd cb dd ad */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 5,(IX/IY+dd),L
	/* dd/fd cb dd ae */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 5,(IX/IY+dd)
	/* dd/fd cb dd af */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 5,(IX/IY+dd),A

	/* dd/fd cb dd b0 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 6,(IX/IY+dd),B
	/* dd/fd cb dd b1 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 6,(IX/IY+dd),C
	/* dd/fd cb dd b2 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 6,(IX/IY+dd),D
	/* dd/fd cb dd b3 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 6,(IX/IY+dd),E
	/* dd/fd cb dd b4 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 6,(IX/IY+dd),H
	/* dd/fd cb dd b5 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 6,(IX/IY+dd),L
	/* dd/fd cb dd b6 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 6,(IX/IY+dd)
	/* dd/fd cb dd b7 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 6,(IX/IY+dd),A
	/* dd/fd cb dd b8 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 7,(IX/IY+dd),B
	/* dd/fd cb dd b9 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 7,(IX/IY+dd),C
	/* dd/fd cb dd ba */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 7,(IX/IY+dd),D
	/* dd/fd cb dd bb */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 7,(IX/IY+dd),E
	/* dd/fd cb dd bc */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 7,(IX/IY+dd),H
	/* dd/fd cb dd bd */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 7,(IX/IY+dd),L
	/* dd/fd cb dd be */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 7,(IX/IY+dd)
	/* dd/fd cb dd bf */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_RES, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, RES 7,(IX/IY+dd),A

	/* dd/fd cb dd c0 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 0,(IX/IY+dd),B
	/* dd/fd cb dd c1 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 0,(IX/IY+dd),C
	/* dd/fd cb dd c2 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 0,(IX/IY+dd),D
	/* dd/fd cb dd c3 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 0,(IX/IY+dd),E
	/* dd/fd cb dd c4 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 0,(IX/IY+dd),H
	/* dd/fd cb dd c5 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 0,(IX/IY+dd),L
	/* dd/fd cb dd c6 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 0,(IX/IY+dd)
	/* dd/fd cb dd c7 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 0,(IX/IY+dd),A
	/* dd/fd cb dd c8 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 1,(IX/IY+dd),B
	/* dd/fd cb dd c9 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 1,(IX/IY+dd),C
	/* dd/fd cb dd ca */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 1,(IX/IY+dd),D
	/* dd/fd cb dd cb */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 1,(IX/IY+dd),E
	/* dd/fd cb dd cc */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 1,(IX/IY+dd),H
	/* dd/fd cb dd cd */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 1,(IX/IY+dd),L
	/* dd/fd cb dd ce */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 1,(IX/IY+dd)
	/* dd/fd cb dd cf */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 1,(IX/IY+dd),A

	/* dd/fd cb dd d0 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 2,(IX/IY+dd),B
	/* dd/fd cb dd d1 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 2,(IX/IY+dd),C
	/* dd/fd cb dd d2 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 2,(IX/IY+dd),D
	/* dd/fd cb dd d3 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 2,(IX/IY+dd),E
	/* dd/fd cb dd d4 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 2,(IX/IY+dd),H
	/* dd/fd cb dd d5 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 2,(IX/IY+dd),L
	/* dd/fd cb dd d6 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 2,(IX/IY+dd)
	/* dd/fd cb dd d7 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 2,(IX/IY+dd),A
	/* dd/fd cb dd d8 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 3,(IX/IY+dd),B
	/* dd/fd cb dd d9 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 3,(IX/IY+dd),C
	/* dd/fd cb dd da */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 3,(IX/IY+dd),D
	/* dd/fd cb dd db */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 3,(IX/IY+dd),E
	/* dd/fd cb dd dc */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 3,(IX/IY+dd),H
	/* dd/fd cb dd dd */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 3,(IX/IY+dd),L
	/* dd/fd cb dd de */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 3,(IX/IY+dd)
	/* dd/fd cb dd df */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 3,(IX/IY+dd),A

	/* dd/fd cb dd e0 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 4,(IX/IY+dd),B
	/* dd/fd cb dd e1 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 4,(IX/IY+dd),C
	/* dd/fd cb dd e2 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 4,(IX/IY+dd),D
	/* dd/fd cb dd e3 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 4,(IX/IY+dd),E
	/* dd/fd cb dd e4 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 4,(IX/IY+dd),H
	/* dd/fd cb dd e5 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 4,(IX/IY+dd),L
	/* dd/fd cb dd e6 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 4,(IX/IY+dd)
	/* dd/fd cb dd e7 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 4,(IX/IY+dd),A
	/* dd/fd cb dd e8 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 5,(IX/IY+dd),B
	/* dd/fd cb dd e9 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 5,(IX/IY+dd),C
	/* dd/fd cb dd ea */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 5,(IX/IY+dd),D
	/* dd/fd cb dd eb */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 5,(IX/IY+dd),E
	/* dd/fd cb dd ec */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 5,(IX/IY+dd),H
	/* dd/fd cb dd ed */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 5,(IX/IY+dd),L
	/* dd/fd cb dd ee */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 5,(IX/IY+dd)
	/* dd/fd cb dd ef */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 5,(IX/IY+dd),A

	/* dd/fd cb dd f0 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 6,(IX/IY+dd),B
	/* dd/fd cb dd f1 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 6,(IX/IY+dd),C
	/* dd/fd cb dd f2 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 6,(IX/IY+dd),D
	/* dd/fd cb dd f3 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 6,(IX/IY+dd),E
	/* dd/fd cb dd f4 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 6,(IX/IY+dd),H
	/* dd/fd cb dd f5 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 6,(IX/IY+dd),L
	/* dd/fd cb dd f6 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 6,(IX/IY+dd)
	/* dd/fd cb dd f7 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 6,(IX/IY+dd),A
	/* dd/fd cb dd f8 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 7,(IX/IY+dd),B
	/* dd/fd cb dd f9 */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 7,(IX/IY+dd),C
	/* dd/fd cb dd fa */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 7,(IX/IY+dd),D
	/* dd/fd cb dd fb */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 7,(IX/IY+dd),E
	/* dd/fd cb dd fc */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 7,(IX/IY+dd),H
	/* dd/fd cb dd fd */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 7,(IX/IY+dd),L
	/* dd/fd cb dd fe */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 7,(IX/IY+dd)
	/* dd/fd cb dd ff */ { WZ_OUT, READ, CHECK_WAIT, DB_TMP, ALU_SET, X2, ALU_DB, ALU_REGS0, WRITE, CHECK_WAIT, END },  // 23 cycles, SET 7,(IX/IY+dd),A

	/*****************************************************/
	/* Special sequences                                 */
	/*****************************************************/

	/* M1 */ { PC_OUT, PC_INC, READ_OP, CHECK_WAIT, REFRESH, DECODE },
	/* DD/FD CB */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_TMP, PC_OUT, PC_INC, READ_OP2, CHECK_WAIT, DISP_WZ2, DECODE }, // 8 cycles, read displacement and next opcode
};


inline u16 z80lle_device::adc16(u16 arg1, u16 arg2)
{
	u32 res = arg1 + arg2 + (F & CF);
	WZ = arg1 + 1;
	F = (((arg1 ^ res ^ arg2) >> 8) & HF) |
		((res >> 16) & CF) |
		((res >> 8) & (SF | YF | XF)) |
		((res & 0xffff) ? 0 : ZF) |
		(((arg2 ^ arg1 ^ 0x8000) & (arg2 ^ res) & 0x8000) >> 13);
	return res;
}


inline u16 z80lle_device::add16(u16 arg1, u16 arg2)
{
	u32 res = arg1 + arg2;
	WZ = res + 1;
	F = (F & (SF | ZF | VF)) |
		(((arg1 ^ res ^ arg2) >> 8) & HF) |
		((res >> 16) & CF) | ((res >> 8) & (YF | XF));
	return (u16)res;
}


inline u16 z80lle_device::sbc16(u16 arg1, u16 arg2)
{
	u32 res = arg1 - arg2 - (F & CF);
	WZ = arg1 + 1;
	F = (((arg1 ^ res ^ arg2) >> 8) & HF) | NF |
		((res >> 16) & CF) |
		((res >> 8) & (SF | YF | XF)) |
		((res & 0xffff) ? 0 : ZF) |
		(((arg2 ^ arg1) & (arg1 ^ res) & 0x8000) >> 13);
	return res;
}


/****************************************************************************
 * Processor initialization
 ****************************************************************************/
void z80lle_device::device_start()
{
	if (!tables_initialised)
	{
		uint8_t *padd = &SZHVC_add[  0*256];
		uint8_t *padc = &SZHVC_add[256*256];
		uint8_t *psub = &SZHVC_sub[  0*256];
		uint8_t *psbc = &SZHVC_sub[256*256];
		for (int oldval = 0; oldval < 256; oldval++)
		{
			for (int newval = 0; newval < 256; newval++)
			{
				/* add or adc w/o carry set */
				int val = newval - oldval;
				*padd = (newval) ? ((newval & 0x80) ? SF : 0) : ZF;
				*padd |= (newval & (YF | XF));  /* undocumented flag bits 5+3 */
				if ((newval & 0x0f) < (oldval & 0x0f))
					*padd |= HF;
				if (newval < oldval)
					*padd |= CF;
				if ((val^oldval^0x80) & (val^newval) & 0x80)
					*padd |= VF;
				padd++;

				/* adc with carry set */
				val = newval - oldval - 1;
				*padc = (newval) ? ((newval & 0x80) ? SF : 0) : ZF;
				*padc |= (newval & (YF | XF));  /* undocumented flag bits 5+3 */
				if ((newval & 0x0f) <= (oldval & 0x0f))
					*padc |= HF;
				if (newval <= oldval )
					*padc |= CF;
				if ((val^oldval^0x80) & (val^newval) & 0x80)
					*padc |= VF;
				padc++;

				/* cp, sub or sbc w/o carry set */
				val = oldval - newval;
				*psub = NF | ((newval) ? ((newval & 0x80) ? SF : 0) : ZF);
				*psub |= (newval & (YF | XF));  /* undocumented flag bits 5+3 */
				if ((newval & 0x0f) > (oldval & 0x0f))
					*psub |= HF;
				if (newval > oldval)
					*psub |= CF;
				if ((val^oldval) & (oldval^newval) & 0x80)
					*psub |= VF;
				psub++;

				/* sbc with carry set */
				val = oldval - newval - 1;
				*psbc = NF | ((newval) ? ((newval & 0x80) ? SF : 0) : ZF);
				*psbc |= (newval & (YF | XF));  /* undocumented flag bits 5+3 */
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
			SZ[i] |= (i & (YF | XF));       /* undocumented flag bits 5+3 */
			SZ_BIT[i] = i ? i & SF : ZF | PF;
			SZ_BIT[i] |= (i & (YF | XF));   /* undocumented flag bits 5+3 */
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

	/* Reset registers to their initial values */
	m_prvpc.d = 0;
	m_pc.d = 0;
	m_sp.d = 0;
	m_af.d = 0;
	m_bc.d = 0;
	m_de.d = 0;
	m_hl_index[HL_OFFSET].d = 0;
	m_hl_index[IX_OFFSET].d = 0;
	m_hl_index[IY_OFFSET].d = 0;
	m_wz.d = 0;
	m_af2.d = 0;
	m_bc2.d = 0;
	m_de2.d = 0;
	m_hl2.d = 0;
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
	m_after_ei = 0;
	m_after_ldair = 0;
	m_ea = 0;

	m_program = &space(AS_PROGRAM);
	m_decrypted_opcodes = has_space(AS_OPCODES) ? &space(AS_OPCODES) : m_program;
	m_direct = m_program->direct<0>();
	m_decrypted_opcodes_direct = m_decrypted_opcodes->direct<0>();
	m_io = &space(AS_IO);

	m_hl_index[IX_OFFSET].w.l = m_hl_index[IY_OFFSET].w.l = 0xffff; /* IX and IY are FFFF after a reset! */
	F = ZF;           /* Zero flag is set */

	/* set up the state table */
	state_add(STATE_GENPC,     "PC",        m_pc.w.l).callimport();
	state_add(STATE_GENPCBASE, "CURPC",     m_prvpc.w.l).callimport().noshow();
	state_add(Z80LLE_SP,       "SP",        m_sp.w.l);
	state_add(STATE_GENSP,     "GENSP",     m_sp.w.l).noshow();
	state_add(STATE_GENFLAGS,  "GENFLAGS",  F).noshow().formatstr("%8s");
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
}


/****************************************************************************
 * Do a reset
 ****************************************************************************/
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

	m_wz.d = m_pc.d;

	m_instruction = M1;
	m_instruction_step = 0;
	m_instruction_offset = 0;

	m_tmp = 0;
	m_alu = 0;
	m_hl_offset = HL_OFFSET;
}


/****************************************************************************
 * Execute 'cycles' T-states.
 ****************************************************************************/
void z80lle_device::execute_run()
{
	do
	{
		// check for interrupts before each instruction
		// TODO: Check for start of instruction
//		if (m_nmi_pending)
//			take_nmi();
//		else if (m_irq_state != CLEAR_LINE && m_iff1 && !m_after_ei)
//			take_interrupt();
//
//		m_after_ei = false;
//		m_after_ldair = false;

		if (m_instruction == M1 && m_instruction_step == 0 && m_instruction_offset == 0) {
			m_prvpc.d = m_pc.d;
			debugger_instruction_hook(m_pc.d);
		}

		// Execute steps for instruction
		u8 step = insts[m_instruction][m_instruction_step++];
		switch (step)
		{
		case UNKNOWN:
			fatalerror("Unsupported instruction %d,%02x encountered at address %04x", m_instruction_offset / 256, m_ir, m_prvpc.d);
			break;
		case A_ACT:
			m_act = A;
			break;
		case A_DB:
			m_data_bus = A;
			WZ_H = m_data_bus;
			break;
		case A_W:
			WZ_H = A;
			break;
		case ADC16:
			switch (m_ir & 0x30)
			{
			case 0x00:
				HL = adc16(HL, BC);
				break;
			case 0x10:
				HL = adc16(HL, DE);
				break;
			case 0x20:
				HL = adc16(HL, HL);
				break;
			case 0x30:
				HL = adc16(HL, SP);
				break;
			}
			m_icount -= 7;
			break;
		case ADD16:
			switch (m_ir & 0x30)
			{
			case 0x00:
				HL = add16(HL, BC);
				break;
			case 0x10:
				HL = add16(HL, DE);
				break;
			case 0x20:
				HL = add16(HL, HL);
				break;
			case 0x30:
				HL = add16(HL, SP);
				break;
			}
			m_icount -= 7;
			break;
		case SBC16:
			switch (m_ir & 0x30)
			{
			case 0x00:
				HL = sbc16(HL, BC);
				break;
			case 0x10:
				HL = sbc16(HL, DE);
				break;
			case 0x20:
				HL = sbc16(HL, HL);
				break;
			case 0x30:
				HL = sbc16(HL, SP);
				break;
			}
			m_icount -= 7;
			break;
		case ALU_DB:
			m_data_bus = m_alu;
			break;
		case ALU_A:
			A = m_alu;
			break;
		case ALU_REGS:
			switch (m_ir & 0x07)
			{
			case 0x00:
				B = m_alu;
				break;
			case 0x01:
				C = m_alu;
				break;
			case 0x02:
				D = m_alu;
				break;
			case 0x03:
				E = m_alu;
				break;
			case 0x04:
				H = m_alu;
				break;
			case 0x05:
				L = m_alu;
				break;
			case 0x06:
				fatalerror("ALU_REGS: illegal register reference 0x06\n");
				break;
			case 0x07:
				A = m_alu;
				break;
			}
			break;
		case ALU_REGS0:
			switch (m_ir & 0x07)
			{
			case 0x00:
				B = m_alu;
				break;
			case 0x01:
				C = m_alu;
				break;
			case 0x02:
				D = m_alu;
				break;
			case 0x03:
				E = m_alu;
				break;
			case 0x04:
				m_hl_index[HL_OFFSET].b.h = m_alu;
				break;
			case 0x05:
				m_hl_index[HL_OFFSET].b.l = m_alu;
				break;
			case 0x06:
				fatalerror("ALU_REGS0: illegal register reference 0x06\n");
				break;
			case 0x07:
				A = m_alu;
				break;
			}
			break;
		case ALU_REGD:
			switch (m_ir & 0x38)
			{
			case 0x00:
				B = m_alu;
				break;
			case 0x08:
				C = m_alu;
				break;
			case 0x10:
				D = m_alu;
				break;
			case 0x18:
				E = m_alu;
				break;
			case 0x20:
				H = m_alu;
				break;
			case 0x28:
				L = m_alu;
				break;
			case 0x30:
				fatalerror("ALU_REGD: illegal register reference 0x30\n");
				break;
			case 0x38:
				A = m_alu;
				break;
			}
			break;
		case ALU_ADC:
			m_alu = m_act + m_tmp + (F & CF);
			F = SZHVC_add[((F & CF) << 16) | (m_act << 8) | m_alu];
			break;
		case ALU_ADD:
			m_alu = m_act + m_tmp;
			F = SZHVC_add[(m_act << 8) | m_alu];
			break;
		case ALU_AND:
			m_alu = m_act & m_tmp;
			F = SZP[m_alu] | HF;
			break;
		case ALU_BIT:
			if ((m_ir & 0x07) == 0x06)
			{
				F = (F & CF) | HF | (SZ_BIT[m_tmp & (1 << ((m_ir >> 3) & 0x07))] & ~(YF|XF)) | (WZ_H & (YF|XF));
			}
			else
			{
				F = (F & CF) | HF | (SZ_BIT[m_tmp & (1 << ((m_ir >> 3) & 0x07))] & ~(YF|XF)) | (m_tmp & (YF|XF));
			}
			break;
		case ALU_RES:
			m_alu = m_tmp & ~(1 << ((m_ir >> 3) & 0x07));
			logerror("ALU_RES: m_tmp = %02x, m_alu = %02x\n", m_tmp, m_alu);
			break;
		case ALU_SET:
			m_alu = m_tmp | (1 << ((m_ir >> 3) & 0x07));
			break;
		case ALU_CP:  // Flag handling is slightly different from SUB
			m_alu = m_act - m_tmp;
			F = (SZHVC_sub[(m_act << 8) | m_alu] & ~(YF | XF)) | (m_tmp & (YF | XF));
			break;
		case ALU_DEC:
			m_alu = m_tmp - 1;
			F = (F & CF) | SZHV_dec[m_alu];
			break;
		case ALU_INC:
			m_alu = m_tmp + 1;
			F = (F & CF) | SZHV_inc[m_alu];
			break;
		case ALU_OR:
			m_alu = m_act | m_tmp;
			F = SZP[m_alu];
			break;
		case ALU_RL:
			m_alu = (m_tmp << 1) | (F & CF);
			F = SZP[m_alu] | ((m_tmp & 0x80) ? CF : 0);
			break;
		case ALU_RLC:
			m_alu = (m_tmp << 1) | (m_tmp >> 7);
			F = SZP[m_alu] | ((m_tmp & 0x80) ? CF : 0);
			break;
		case ALU_RR:
			m_alu = (m_tmp >> 1) | (F << 7);
			F = SZP[m_alu] | ((m_tmp & 0x01) ? CF : 0);
			break;
		case ALU_RRC:
			m_alu = (m_tmp >> 1) | (m_tmp << 7);
			F = SZP[m_alu] | ((m_tmp & 0x01) ? CF : 0);
			break;
		case ALU_SBC:
			m_alu = (uint8_t)(m_act - m_tmp - (F & CF));
			F = SZHVC_sub[((F & CF) << 16) | (m_act << 8) | m_alu];
			break;
		case ALU_SLA:
			m_alu = m_tmp << 1;
			F = SZP[m_alu] | ((m_tmp & 0x80) ? CF : 0);
			break;
		case ALU_SLL:
			m_alu = (m_tmp << 1) | 0x01;
			F = SZP[m_alu] | ((m_tmp & 0x80) ? CF : 0);
			break;
		case ALU_SRA:
			m_alu = (m_tmp >> 1) | (m_tmp & 0x80);
			F = SZP[m_alu] | ((m_tmp & 0x01) ? CF : 0);
			break;
		case ALU_SRL:
			m_alu = m_tmp >> 1;
			F = SZP[m_alu] | ((m_tmp & 0x01) ? CF : 0);
			break;
		case ALU_SUB:
			m_alu = m_act - m_tmp;
			F = SZHVC_sub[(m_act << 8) | m_alu];
			break;
		case ALU_XOR:
			m_alu = m_act ^ m_tmp;
			F = SZP[m_alu];
			break;
		case CHECK_WAIT:
			if (!m_wait_state)
			{
				m_icount -= 1;
				// Do not advance to next step
				m_instruction_step--;
			}
			break;
		case DB_REG:
			switch (m_ir & 0x38)
			{
			case 0x00:
				B = m_data_bus;
				break;
			case 0x08:
				C = m_data_bus;
				break;
			case 0x10:
				D = m_data_bus;
				break;
			case 0x18:
				E = m_data_bus;
				break;
			case 0x20:
				H = m_data_bus;
				break;
			case 0x28:
				L = m_data_bus;
				break;
			case 0x30:
				fatalerror("DB_REG: illegal register reference 0x30\n");
				break;
			case 0x38:
				A = m_data_bus;
				break;
			}
			break;
		case DB_REG0:
			switch (m_ir & 0x38)
			{
			case 0x00:
				B = m_data_bus;
				break;
			case 0x08:
				C = m_data_bus;
				break;
			case 0x10:
				D = m_data_bus;
				break;
			case 0x18:
				E = m_data_bus;
				break;
			case 0x20:
				m_hl_index[HL_OFFSET].b.h = m_data_bus;
				break;
			case 0x28:
				m_hl_index[HL_OFFSET].b.l = m_data_bus;
				break;
			case 0x30:
				fatalerror("DB_REG0: illegal register reference 0x30\n");
				break;
			case 0x38:
				A = m_data_bus;
				break;
			}
			break;
		case DB_TMP:
			m_tmp = m_data_bus;
			break;
		case DB_A:
			A = m_data_bus;
			break;
		case DB_W:
			WZ_H = m_data_bus;
			break;
		case DB_Z:
			WZ_L = m_data_bus;
			break;
		case BC_WZ:
			WZ = BC;
			break;
		case DE_WZ:
			WZ = DE;
			break;
		case HL_WZ:
			WZ = HL;
			break;
		case DEC_SP:
			SP -= 1;
			break;
		case INC_SP:
			SP += 1;
			break;
		case DECODE:
			m_instruction = m_instruction_offset | m_ir;
			m_instruction_step = 0;
			if (m_instruction_offset != CB_OFFSET && m_instruction_offset != FDCB_OFFSET)
			{
				if (m_ir == 0xcb)
				{
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
				}
				else if (m_ir == 0xdd)
				{
					m_instruction_offset = FD_OFFSET;
					m_instruction = M1;
					m_hl_offset = IX_OFFSET;
				}
				else if (m_ir == 0xed)
				{
					m_hl_offset = HL_OFFSET;
					m_instruction_offset = ED_OFFSET;
					m_instruction = M1;
				}
				else if (m_ir == 0xfd)
				{
					m_instruction_offset = FD_OFFSET;
					m_instruction = M1;
					m_hl_offset = IY_OFFSET;
				}
			}
			break;
		case DISP_WZ2:
			WZ = HL + (s8)m_tmp;
			m_icount -= 2;
			break;
		case DISP_WZ5:
			WZ = HL + (s8)m_tmp;
			m_icount -= 5;
			break;
		case DI:
			m_iff1 = m_iff2 = 0;
			break;
		case EI:
			m_iff1 = m_iff2 = 1;
			m_after_ei = true;
			break;
		case END:
			end_instruction();
			break;
		case EX_DE_HL:
			{
				u16 tmp = DE;
				DE = HL;
				HL = tmp;
			}
			break;
		case H_DB:
			m_data_bus = H;
			break;
		case DE_OUT:
			m_address_bus = DE;
			m_icount -= 1;
			break;
		case HL_OUT:
			m_address_bus = HL;
			m_icount -= 1;
			break;
		case DEC_R16:
			switch (m_ir & 0x30)
			{
			case 0x00:
				BC--;
				break;
			case 0x10:
				DE--;
				break;
			case 0x20:
				HL--;
				break;
			case 0x30:
				SP--;
				break;
			}
			m_icount -= 2;
			break;
		case INC_R16:
			switch (m_ir & 0x30)
			{
			case 0x00:
				BC++;
				break;
			case 0x10:
				DE++;
				break;
			case 0x20:
				HL++;
				break;
			case 0x30:
				SP++;
				break;
			}
			m_icount -= 2;
			break;
		case JR_COND:
			if ((F & jr_conditions[((m_ir >> 3) & 0x07)][0]) == jr_conditions[((m_ir >> 3) & 0x07)][1])
			{
				WZ = PC + (s8)m_data_bus;
				PC = WZ;
				m_icount -= 5;
			}
			break;
		case JP_COND:
			if ((F & jp_conditions[((m_ir >> 3) & 0x07)][0]) == jp_conditions[((m_ir >> 3) & 0x07)][1])
			{
				PC = WZ;
			}
			break;
		case CALL_COND:
			if ((F & jp_conditions[((m_ir >> 3) & 0x07)][0]) == jp_conditions[((m_ir >> 3) & 0x07)][1])
			{
				m_icount -= 1;
			}
			else
			{
				end_instruction();
			}
			break;
		case RET_COND:
			if ((F & jp_conditions[((m_ir >> 3) & 0x07)][0]) != jp_conditions[((m_ir >> 3) & 0x07)][1])
			{
				end_instruction();
			}
			m_icount -= 1;
			break;
		case L_DB:
			m_data_bus = L;
			break;
		case OUTPUT:
			m_io->write_byte(m_address_bus, m_data_bus);
			m_icount -= 3;
			break;
		case PC_INC:
			PC++;
			break;
		case PC_OUT:
			m_address_bus = PC;
			m_icount -= 1;
			break;
		case PCH_DB:
			m_data_bus = PC_H;
			break;
		case PCL_DB:
			m_data_bus = PC_L;
			break;
		case R16H_DB:
			switch (m_ir & 0x30) {
			case 0x00:
				m_data_bus = B;
				break;
			case 0x10:
				m_data_bus = D;
				break;
			case 0x20:
				m_data_bus = H;
				break;
			case 0x30:
				if (m_ir & 0x80)
					m_data_bus = A;
				else
					m_data_bus = SP_H;
				break;
			}
			break;
		case R16L_DB:
			switch (m_ir & 0x30) {
			case 0x00:
				m_data_bus = C;
				break;
			case 0x10:
				m_data_bus = E;
				break;
			case 0x20:
				m_data_bus = L;
				break;
			case 0x30:
				if (m_ir & 0x80)
					m_data_bus = F;
				else
					m_data_bus = SP_L;
				break;
			}
			break;
		case DB_R16H:
			switch (m_ir & 0x30)
			{
			case 0x00:
				B = m_data_bus;
				break;
			case 0x10:
				D = m_data_bus;
				break;
			case 0x20:
				H = m_data_bus;
				break;
			case 0x30:
				if (m_ir & 0x80)
					A = m_data_bus;
				else
					SP_H = m_data_bus;
				break;
			}
			break;
		case DB_R16L:
			switch (m_ir & 0x30)
			{
			case 0x00:
				C = m_data_bus;
				break;
			case 0x10:
				E = m_data_bus;
				break;
			case 0x20:
				L = m_data_bus;
				break;
			case 0x30:
				if (m_ir & 0x80)
					F = m_data_bus;
				else
					SP_L = m_data_bus;
				break;
			}
			break;
		case READ:
			m_data_bus = m_program->read_byte(m_address_bus);
			m_icount -= 2;
			break;
		case READ_OP:
			m_ir = m_decrypted_opcodes_direct->read_byte(m_address_bus);
			m_icount -= 1;
			break;
		case READ_OP2:
			m_ir = m_decrypted_opcodes_direct->read_byte(m_address_bus);
			m_icount -= 2;
			break;
		case REFRESH:
			m_icount -= 1;
			m_refresh_cb((m_i << 8) | m_r, 0x00, 0xff);
			m_icount -= 1;
			m_r++;
			break;
		case REGS_DB:
			switch (m_ir & 0x07)
			{
			case 0x00:
				m_data_bus = B;
				break;
			case 0x01:
				m_data_bus = C;
				break;
			case 0x02:
				m_data_bus = D;
				break;
			case 0x03:
				m_data_bus = E;
				break;
			case 0x04:
				m_data_bus = H;
				break;
			case 0x05:
				m_data_bus = L;
				break;
			case 0x06:
				fatalerror("REGS_DB: illegal register reference 0x06\n");
				break;
			case 0x07:
				m_data_bus = A;
				break;
			}
			break;
		case REGS0_DB:
			switch (m_ir & 0x07)
			{
			case 0x00:
				m_data_bus = B;
				break;
			case 0x01:
				m_data_bus = C;
				break;
			case 0x02:
				m_data_bus = D;
				break;
			case 0x03:
				m_data_bus = E;
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
				m_data_bus = A;
				break;
			}
			break;
		case REGS_TMP:
			switch (m_ir & 0x07) {
			case 0x00:
				m_tmp = B;
				break;
			case 0x01:
				m_tmp = C;
				break;
			case 0x02:
				m_tmp = D;
				break;
			case 0x03:
				m_tmp = E;
				break;
			case 0x04:
				m_tmp = H;
				break;
			case 0x05:
				m_tmp = L;
				break;
			case 0x06:
				fatalerror("REGS_TMP: illegal register reference 0x06\n");
				break;
			case 0x07:
				m_tmp = A;
				break;
			}
			break;
		case REGD_TMP:
			switch (m_ir & 0x38) {
			case 0x00:
				m_tmp = B;
				break;
			case 0x08:
				m_tmp = C;
				break;
			case 0x10:
				m_tmp = D;
				break;
			case 0x18:
				m_tmp = E;
				break;
			case 0x20:
				m_tmp = H;
				break;
			case 0x28:
				m_tmp = L;
				break;
			case 0x30:
				fatalerror("REGD_TMP: illegal register reference 0x30\n");
				break;
			case 0x38:
				m_tmp = A;
				break;
			}
			break;
		case CCF:
			F = ((F & (SF | ZF | YF | XF | PF | CF)) | ((F & CF) << 4) | (A & (YF | XF))) ^ CF;
			break;
		case CPL:
			A ^= 0xff;
			F = (F & (SF | ZF | PF | CF)) | HF | NF | (A & (YF | XF));
			break;
		case DAA:
			m_alu = A;
			if (F & NF) {
				if ((F&HF) | ((A&0xf)>9))
					m_alu -= 6;
				if ((F&CF) | (A>0x99))
					m_alu -= 0x60;
			}
			else {
				if ((F&HF) | ((A&0xf)>9))
					m_alu += 6;
				if ((F&CF) | (A>0x99))
					m_alu += 0x60;
			}
			F = (F&(CF|NF)) | (A>0x99) | ((A^m_alu)&HF) | SZP[m_alu];
			A = m_alu;
			break;
		case NEG:
			m_alu = 0 - A;
			F = SZHVC_sub[m_alu];
			A = m_alu;
			break;
		case RLA:
			m_alu = (A << 1) | (F & CF);
			F = (F & (SF | ZF | PF)) | ((A & 0x80) ? CF : 0) | (m_alu & (YF | XF));
			A = m_alu;
			break;
		case RLCA:
			A = (A << 1) | (A >> 7);
			F = (F & (SF | ZF | PF)) | (A & (YF | XF | CF));
			break;
		case RRA:
			m_alu = (A >> 1) | (F << 7);
			F = (F & (SF | ZF | PF)) | ((A & 0x01) ? CF : 0) | (m_alu & (YF | XF));
			A = m_alu;
			break;
		case RRCA:
			F = (F & (SF | ZF | PF)) | (A & CF);
			A = (A >> 1) | (A << 7);
			F |= (A & (YF | XF));
			break;
		case RRD:
			m_alu = (m_data_bus >> 4) | (A << 4);
			A = (A & 0xf0) | (m_data_bus & 0x0f);
			F = (F & CF) | SZP[A];
			m_data_bus = m_alu;
			m_icount -= 5;
			break;
		case RLD:
			m_alu = (m_data_bus << 4) | (A & 0x0f);
			A = (A & 0xf0) | (m_data_bus >> 4);
			F = (F & CF) | SZP[A];
			m_data_bus = m_alu;
			m_icount -= 5;
			break;
		case SCF:
			F = (F & (SF | ZF | YF | XF | PF)) | CF | (A & (YF | XF));
			break;
		case SP_OUT:
			m_address_bus = SP;
			m_icount -= 1;
			break;
		case TMP_REG:
			switch (m_ir & 0x38) {
			case 0x00:
				B = m_tmp;
				break;
			case 0x08:
				C = m_tmp;
				break;
			case 0x10:
				D = m_tmp;
				break;
			case 0x18:
				E = m_tmp;
				break;
			case 0x20:
				H = m_tmp;
				break;
			case 0x28:
				L = m_tmp;
				break;
			case 0x30:
				fatalerror("TMP_REG: illegal register reference 0x30\n");
				break;
			case 0x38:
				A = m_tmp;
				break;
			}
			break;
		case WRITE:
			m_icount -= 1;
			m_program->write_byte(m_address_bus, m_data_bus);
			m_icount -= 1;
			break;
		case WZ_INC:
			WZ++;
			break;
		case WZ_OUT:
			m_address_bus = WZ;
			m_icount -= 1;
			break;
		case WZ_TO_PC:
			PC = WZ;
			break;
		case X:
			m_icount -= 1;
			break;
		case X2:
			m_icount -= 2;
			break;
		case CPD:
			m_alu = A - m_data_bus;
			WZ--;
			HL--; BC--;
			F = (F & CF) | (SZ[m_alu] & ~(YF | XF)) | ((A ^ m_data_bus ^ m_alu) & HF) | NF;
			if (F & HF)
				m_alu -= 1;
			if (m_alu & 0x02)
				F |= YF; /* bit 1 -> flag 5 */
			if (m_alu & 0x08)
				F |= XF; /* bit 3 -> flag 3 */
			if (BC)
				F |= VF;
			break;
		case CPI:
			m_alu = A - m_data_bus;
			WZ++;
			HL++; BC--;
			F = (F & CF) | (SZ[m_alu] & ~(YF | XF)) | ((A ^ m_data_bus ^ m_alu) & HF) | NF;
			if (F & HF)
				m_alu -= 1;
			if (m_alu & 0x02)
				F |= YF; /* bit 1 -> flag 5 */
			if (m_alu & 0x08)
				F |= XF; /* bit 3 -> flag 3 */
			if (BC)
				F |= VF;
			break;
		case LDD:
			F &= SF | ZF | CF;
			if ((A + m_data_bus) & 0x02)
				F |= YF; /* bit 1 -> flag 5 */
			if ((A + m_data_bus) & 0x08)
				F |= XF; /* bit 3 -> flag 3 */
			HL--;
			DE--;
			BC--;
			if (BC)
				F |= VF;
			m_icount -= 2;
			break;
		case LDI:
			F &= SF | ZF | CF;
			if ((A + m_data_bus) & 0x02)
				F |= YF; /* bit 1 -> flag 5 */
			if ((A + m_data_bus) & 0x08)
				F |= XF; /* bit 3 -> flag 3 */
			HL++;
			DE++;
			BC--;
			if (BC)
				F |= VF;
			m_icount -= 2;
			break;
		case REPEAT:
			if (BC != 0)
			{
				PC -= 2;
				// Except for inir, otir, indr, otdr
				if (!BIT(m_ir,1))
				{
					WZ = PC + 1;
				}
				m_icount -= 5;
			}
			break;
		case CPREPEAT:
			if (BC != 0 && !(F & ZF))
			{
				PC -= 2;
				// Except for inir, otir, indr, otdr
				if (!BIT(m_ir,1))
				{
					WZ = PC + 1;
				}
				m_icount -= 5;
			}
			break;
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
		/* mark an NMI pending on the rising edge */
		if (m_nmi_state == CLEAR_LINE && state != CLEAR_LINE)
			m_nmi_pending = true;
		m_nmi_state = state;
		break;

	case INPUT_LINE_IRQ0:
		/* update the IRQ state via the daisy chain */
		m_irq_state = state;
		if (daisy_chain_present())
			m_irq_state = (daisy_update_irq_state() == ASSERT_LINE ) ? ASSERT_LINE : m_irq_state;

		/* the main execute loop will take the interrupt */
		break;

	case Z80LLE_INPUT_LINE_WAIT:
		m_wait_state = state;
		break;

	default:
		break;
	}
}


/**************************************************************************
 * STATE IMPORT/EXPORT
 **************************************************************************/

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
			fatalerror("CPU_IMPORT_STATE() called for unexpected value\n");
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
			fatalerror("CPU_EXPORT_STATE() called for unexpected value\n");
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


/**************************************************************************
 * Generic set_info
 **************************************************************************/

z80lle_device::z80lle_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	z80lle_device(mconfig, Z80LLE, tag, owner, clock)
{
}

z80lle_device::z80lle_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock) :
	cpu_device(mconfig, type, tag, owner, clock),
	z80_daisy_chain_interface(mconfig, *this),
	m_program_config("program", ENDIANNESS_LITTLE, 8, 16, 0),
	m_decrypted_opcodes_config("decrypted_opcodes", ENDIANNESS_LITTLE, 8, 16, 0),
	m_io_config("io", ENDIANNESS_LITTLE, 8, 16, 0),
	m_irqack_cb(*this),
	m_refresh_cb(*this),
	m_halt_cb(*this)
{
}

device_memory_interface::space_config_vector z80lle_device::memory_space_config() const
{
	if(has_configured_map(AS_OPCODES))
		return space_config_vector {
			std::make_pair(AS_PROGRAM, &m_program_config),
			std::make_pair(AS_OPCODES, &m_decrypted_opcodes_config),
			std::make_pair(AS_IO,      &m_io_config)
		};
	else
		return space_config_vector {
			std::make_pair(AS_PROGRAM, &m_program_config),
			std::make_pair(AS_IO,      &m_io_config)
		};
}

DEFINE_DEVICE_TYPE(Z80LLE, z80lle_device, "z80lle", "Zilog Z80 LLE")


