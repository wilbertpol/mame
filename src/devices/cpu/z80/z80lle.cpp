// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/*****************************************************************************
 *
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
 *   - Just about everything
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

const u8 z80lle_device::insts[4 * 256 + 1][23] = {
	/* Regular instructions */
	/* 0x00 */
	/* 00 */ { END },  // 4 cycles, NOP
	/* 01 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_R16L, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_R16H, END },  // 10 cycles, LD BC,nn
	{ 0 },
	/* 03 */ { INC_R16, END },  // 6 cycles, INC BC
	/* 04 */ { REGD_TMP, ALU_INC, ALU_REGD, END },  // 4 cycles, INC B
	/* 05 */ { REGD_TMP, ALU_DEC, ALU_REGD, END },  // 4 cycles, DEC B
	/* 06 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_REG, END },  // 7 cycles, LD B,n
	/* 07 */ { RLCA, END },  // 4 cycles, RLCA
	{ 0 },
	/* 09 */ { ADD16, END },  // 11 cycles, ADD HL,BC
	{ 0 },
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
	{ 0 },
	/* 18 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, JR_COND, END },  // 12 cycles, JR n
	/* 19 */ { ADD16, END },  // 11 cycles, ADD HL,DE
	/* 1a */ { DE_WZ, WZ_OUT, WZ_INC, READ, CHECK_WAIT, DB_A, END },  // 7 cycles, LD A,(DE)
	{ 0 },
	/* 1c */ { REGD_TMP, ALU_INC, ALU_REGD, END },  // 4 cycles, INC E
	/* 1d */ { REGD_TMP, ALU_DEC, ALU_REGD, END },  // 4 cycles, DEC E
	/* 1e */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_REG, END },  // 7 cycles, LD E,n
	{ 0 },
	/* 0x20 */
	/* 20 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, JR_COND, END },  // 7/12 cycles, JR NZ,n
	/* 21 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_R16L, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_R16H, END },  // 10 cycles, LD HL,nn
	/* 22 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, WZ_OUT, WZ_INC, L_DB, WRITE, CHECK_WAIT, WZ_OUT, H_DB, WRITE, CHECK_WAIT, END },  // 16 cycles, LD (nn),HL
	/* 23 */ { INC_R16, END },  // 6 cycles, INC HL
	/* 24 */ { REGD_TMP, ALU_INC, ALU_REGD, END },  // 4 cycles, INC H
	/* 25 */ { REGD_TMP, ALU_DEC, ALU_REGD, END },  // 4 cycles, DEC H
	/* 26 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_REG, END },  // 7 cycles, LD H,n
	{ 0 },
	/* 28 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, JR_COND, END },  // 7/12 cycles, JR Z,n
	/* 29 */ { ADD16, END },  // 11 cycles, ADD HL,HL
	/* 2a */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, WZ_OUT, WZ_INC, READ, CHECK_WAIT, DB_R16L, WZ_OUT, READ, CHECK_WAIT, DB_R16H, END },  // 16 cycles, LD HL,(nn)
	/* 2b */ { DEC_R16, END },  // 6 cycles, DEC HL
	/* 2c */ { REGD_TMP, ALU_INC, ALU_REGD, END },  // 4 cycles, INC L
	/* 2d */ { REGD_TMP, ALU_DEC, ALU_REGD, END },  // 4 cycles, DEC L
	/* 2e */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_REG, END },  // 7 cycles, LD L,n
	{ 0 },
	/* 0x30 */
	{ 0 },
	/* 31 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_R16L, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_R16H, END },  // 10 cycles, LD SP,nn
	/* 32 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, WZ_OUT, WZ_INC, A_DB, WRITE, CHECK_WAIT, END },  // 13 cycles, LD (nn),A
	{ 0 },
	/* 34 */ { HL_OUT, READ, CHECK_WAIT, DB_TMP, ALU_INC, ALU_DB, X, HL_OUT, WRITE, CHECK_WAIT, END },  // 11 cycles, INC (HL)
	{ 0 },
	/* 36 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, HL_OUT, WRITE, END },  // 10 cycles, LD (HL),n
	{ 0 }, { 0 },
	/* 39 */ { ADD16, END },  // 11 cycles, ADD HL,SP
	/* 3a */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, WZ_OUT, WZ_INC, READ, CHECK_WAIT, DB_A, END },  // 13 cycles, LD A,(nn)
	{ 0 },
	/* 3c */ { REGD_TMP, ALU_INC, ALU_REGD, END },  // 4 cycles, INC A
	/* 3d */ { REGD_TMP, ALU_DEC, ALU_REGD, END },  // 4 cycles, DEC A
	/* 3e */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_REG, END },  // 7 cycles, LD A,n
	{ 0 },
	/* 0x40 */
	/* 40 */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD B,B
	/* 41 */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD B,C
	/* 42 */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD B,D
	/* 43 */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD B,E
	/* 44 */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD B,H
	/* 45 */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD B,L
	/* 46 */ { HL_OUT, READ, CHECK_WAIT, DB_REG, END },  // 7 cycles, LD B,(HL)
	/* 47 */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD B,A
	/* 48 */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD C,B
	/* 49 */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD C,C
	/* 4a */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD C,D
	/* 4b */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD C,E
	/* 4c */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD C,H
	/* 4d */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD C,L
	/* 4e */ { HL_OUT, READ, CHECK_WAIT, DB_REG, END },  // 7 cycles, LD C,(HL)
	/* 4f */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD C,A
	/* 0x50 */
	/* 50 */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD D,B
	/* 51 */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD D,C
	/* 52 */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD D,D
	/* 53 */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD D,E
	/* 54 */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD D,H
	/* 55 */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD D,L
	/* 56 */ { HL_OUT, READ, CHECK_WAIT, DB_REG, END },  // 7 cycles, LD D,(HL)
	/* 57 */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD D,A
	/* 58 */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD E,B
	/* 59 */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD E,C
	/* 5a */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD E,D
	/* 5b */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD E,E
	/* 5c */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD E,H
	/* 5d */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD E,L
	/* 5e */ { HL_OUT, READ, CHECK_WAIT, DB_REG, END },  // 7 cycles, LD E,(HL)
	/* 5f */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD E,A
	/* 0x60 */
	/* 60 */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD H,B
	/* 61 */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD H,C
	/* 62 */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD H,D
	/* 63 */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD H,E
	/* 64 */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD H,H
	/* 65 */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD H,L
	/* 66 */ { HL_OUT, READ, CHECK_WAIT, DB_REG, END },  // 7 cycles, LD H,(HL)
	/* 67 */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD H,A
	/* 68 */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD L,B
	/* 69 */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD L,C
	/* 6a */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD L,D
	/* 6b */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD L,E
	/* 6c */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD L,H
	/* 6d */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD L,L
	/* 6e */ { HL_OUT, READ, CHECK_WAIT, DB_REG, END },  // 7 cycles, LD L,(HL)
	/* 6f */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD L,A
	/* 0x70 */
	{ 0 }, { 0 }, { 0 },
	/* 73 */ { HL_OUT, REGS_DB, WRITE, CHECK_WAIT, END },  // 7 cycles, LD (HL),E
	{ 0 }, { 0 }, { 0 },
	/* 77 */ { HL_OUT, REGS_DB, WRITE, CHECK_WAIT, END },  // 7 cycles, LD (HL),A
	/* 78 */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD A,B
	/* 79 */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD A,C
	/* 7a */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD A,D
	/* 7b */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD A,E
	/* 7c */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD A,H
	/* 7d */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD A,L
	/* 7e */ { HL_OUT, READ, CHECK_WAIT, DB_REG, END },  // 7 cycles, LD A,(HL)
	/* 7f */ { REG_TMP, TMP_REG, END },  // 4 cycles, LD A,A
	/* 0x80 */
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* 0x90 */
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* 0xa0 */
	/* a0 */ { A_ACT, REG_TMP, ALU_AND, ALU_A, END },  // 4 cycles, AND B
	/* a1 */ { A_ACT, REG_TMP, ALU_AND, ALU_A, END },  // 4 cycles, AND C
	/* a2 */ { A_ACT, REG_TMP, ALU_AND, ALU_A, END },  // 4 cycles, AND D
	/* a3 */ { A_ACT, REG_TMP, ALU_AND, ALU_A, END },  // 4 cycles, AND E
	/* a4 */ { A_ACT, REG_TMP, ALU_AND, ALU_A, END },  // 4 cycles, AND H
	/* a5 */ { A_ACT, REG_TMP, ALU_AND, ALU_A, END },  // 4 cycles, AND L
	/* a6 */ { 0 },
	/* a7 */ { A_ACT, REG_TMP, ALU_AND, ALU_A, END },  // 4 cycles, AND A
	/* a8 */ { A_ACT, REG_TMP, ALU_XOR, ALU_A, END },  // 4 cycles, XOR B
	/* a9 */ { A_ACT, REG_TMP, ALU_XOR, ALU_A, END },  // 4 cycles, XOR C
	/* aa */ { A_ACT, REG_TMP, ALU_XOR, ALU_A, END },  // 4 cycles, XOR D
	/* ab */ { A_ACT, REG_TMP, ALU_XOR, ALU_A, END },  // 4 cycles, XOR E
	/* ac */ { A_ACT, REG_TMP, ALU_XOR, ALU_A, END },  // 4 cycles, XOR H
	/* ad */ { A_ACT, REG_TMP, ALU_XOR, ALU_A, END },  // 4 cycles, XOR L
	/* ae */ { HL_OUT, READ, CHECK_WAIT, A_ACT, DB_TMP, ALU_XOR, ALU_A, END },  // 7 cycles, XOR (HL)
	/* af */ { A_ACT, REG_TMP, ALU_XOR, ALU_A, END },  // 4 cycles, XOR A
	/* 0xb0 */
	/* b0 */ { A_ACT, REG_TMP, ALU_OR, ALU_A, END },  // 4 cycles, OR B
	/* b1 */ { A_ACT, REG_TMP, ALU_OR, ALU_A, END },  // 4 cycles, OR C
	/* b2 */ { A_ACT, REG_TMP, ALU_OR, ALU_A, END },  // 4 cycles, OR D
	/* b3 */ { A_ACT, REG_TMP, ALU_OR, ALU_A, END },  // 4 cycles, OR E
	/* b4 */ { A_ACT, REG_TMP, ALU_OR, ALU_A, END },  // 4 cycles, OR H
	/* b5 */ { A_ACT, REG_TMP, ALU_OR, ALU_A, END },  // 4 cycles, OR L
	/* b6 */ { HL_OUT, READ, CHECK_WAIT, A_ACT, DB_TMP, ALU_OR, ALU_A, END },  // 7 cycles, OR (HL)
	/* b7 */ { A_ACT, REG_TMP, ALU_OR, ALU_A, END },  // 4 cycles, OR A
	/* b8 */ { A_ACT, REG_TMP, ALU_CP, END },  // 4 cycles, CP B
	/* b9 */ { A_ACT, REG_TMP, ALU_CP, END },  // 4 cycles, CP C
	/* ba */ { A_ACT, REG_TMP, ALU_CP, END },  // 4 cycles, CP D
	/* bb */ { A_ACT, REG_TMP, ALU_CP, END },  // 4 cycles, CP E
	/* bc */ { A_ACT, REG_TMP, ALU_CP, END },  // 4 cycles, CP H
	/* bd */ { A_ACT, REG_TMP, ALU_CP, END },  // 4 cycles, CP L
	/* be */ { HL_OUT, READ, CHECK_WAIT, A_ACT, DB_TMP, ALU_CP, END },  // 7 cycles, CP (HL)
	/* bf */ { A_ACT, REG_TMP, ALU_CP, END },  // 4 cycles, CP A
	/* 0xc0 */
	{ 0 },
	/* c1 */ { SP_OUT, INC_SP, READ, CHECK_WAIT, DB_R16L, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_R16H, END },  // 10 cycles, POP BC
	/* c2 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, JP_COND, END },  // 10 cycles, JP NZ,nn
	/* c3 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, WZ_TO_PC, END },  // 10 cycles, JMP nn
	/* c4 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, CALL_COND, DEC_SP, SP_OUT, PCH_DB, WRITE, CHECK_WAIT, DEC_SP, SP_OUT, PCL_DB, WRITE, CHECK_WAIT, WZ_TO_PC, END },  // 10/17 cycles, CALL NZ,nn
	/* c5 */ { X, DEC_SP, SP_OUT, R16H_DB, WRITE, CHECK_WAIT, DEC_SP, SP_OUT, R16L_DB, WRITE, CHECK_WAIT, END },  // 11 cycles, PUSH BC
	/* c6 */ { 0 },  // 7 cycles
	{ 0 },
	/* c8 */ { RET_COND, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_Z, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_W, WZ_TO_PC, END },  // 5/11 cycles, RET Z
	/* c9 */ { SP_OUT, INC_SP, READ, CHECK_WAIT, DB_Z, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_W, WZ_TO_PC, END },  // 10 cycles, RET
	/* ca */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, JP_COND, END },  // 10 cycles, JP Z,nn
	/* cb */ { 0 },
	{ 0 },
	/* cd */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, X, DEC_SP, SP_OUT, PCH_DB, WRITE, CHECK_WAIT, DEC_SP, SP_OUT, PCL_DB, WRITE, CHECK_WAIT, WZ_TO_PC, END },  // 17 cycles, CALL nn
	{ 0 }, { 0 },
	/* 0xd0 */
	{ 0 },
	/* d1 */ { SP_OUT, INC_SP, READ, CHECK_WAIT, DB_R16L, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_R16H, END },  // 10 cycles, POP DE
	{ 0 },
	/* d3 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, A_W, WZ_OUT, WZ_INC, A_DB, OUTPUT, CHECK_WAIT, END },  // 11 cycles, OUT (n), A
	{ 0 },
	/* d5 */ { X, DEC_SP, SP_OUT, R16H_DB, WRITE, CHECK_WAIT, DEC_SP, SP_OUT, R16L_DB, WRITE, CHECK_WAIT, END },  // 11 cycles, PUSH DE
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* dc */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, CALL_COND, DEC_SP, SP_OUT, PCH_DB, WRITE, CHECK_WAIT, DEC_SP, SP_OUT, PCL_DB, WRITE, CHECK_WAIT, WZ_TO_PC, END },  // 10/17 cycles, CALL C,nn
	{ 0 }, { 0 }, { 0 },
	/* 0xe0 */
	{ 0 },
	/* e1 */ { SP_OUT, INC_SP, READ, CHECK_WAIT, DB_R16L, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_R16H, END },  // 10 cycles, POP HL
	{ 0 }, { 0 }, { 0 },
	/* e5 */ { X, DEC_SP, SP_OUT, R16H_DB, WRITE, CHECK_WAIT, DEC_SP, SP_OUT, R16L_DB, WRITE, CHECK_WAIT, END },  // 11 cycles, PUSH HL
	/* e6 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, A_ACT, DB_TMP, ALU_AND, ALU_A, END },  // 7 cycles, AND n
	{ 0 }, { 0 }, { 0 }, { 0 },
	/* eb */ { EX_DE_HL, END },  // 4 cycles, EX DE,HL
	{ 0 }, { 0 }, { 0 }, { 0 },
	/* 0xf0 */
	{ 0 },
	/* f1 */ { SP_OUT, INC_SP, READ, CHECK_WAIT, DB_R16L, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_R16H, END },  // 10 cycles, POP AF
	{ 0 },
	/* f3 */ { DI, END },  // 4 cycles, DI
	{ 0 },
	/* f5 */ { X, DEC_SP, SP_OUT, R16H_DB, WRITE, CHECK_WAIT, DEC_SP, SP_OUT, R16L_DB, WRITE, CHECK_WAIT, END },  // 11 cycles, PUSH AF
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* fb */ { EI, END },  // 4 cycles, EI
	{ 0 }, { 0 },
	/* fe */ { PC_OUT, PC_INC, READ, CHECK_WAIT, A_ACT, DB_TMP, ALU_CP, END },  // 7 cycles, CP n
	{ 0 },

	/* CB prefixed instructions */
	/* 0x00 */
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* 0x10 */
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* 0x20 */
	/* 20 */ { REG_TMP, ALU_SLA, ALU_REG, END }, // 8 cycles, SLA B
	/* 21 */ { REG_TMP, ALU_SLA, ALU_REG, END }, // 8 cycles, SLA C
	/* 22 */ { REG_TMP, ALU_SLA, ALU_REG, END }, // 8 cycles, SLA D
	/* 23 */ { REG_TMP, ALU_SLA, ALU_REG, END }, // 8 cycles, SLA E
	/* 24 */ { REG_TMP, ALU_SLA, ALU_REG, END }, // 8 cycles, SLA H
	/* 25 */ { REG_TMP, ALU_SLA, ALU_REG, END }, // 8 cycles, SLA L
	/* 26 */ { 0 },
	/* 27 */ { REG_TMP, ALU_SLA, ALU_REG, END }, // 8 cycles, SLA A
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* 0x30 */
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* 0x40 */
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* 0x50 */
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* 0x60 */
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* 0x70 */
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* 0x80 */
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* 0x90 */
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* 0xa0 */
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* 0xb0 */
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* 0xc0 */
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* 0xd0 */
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* 0xe0 */
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* 0xf0 */
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },

	/* ED-prefixed instructions */
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
	/* 42 */ { SBC16, END },  // 15 cycles, SBC HL,BC
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* 4a */ { ADC16, END },  // 15 cycles, ADC HL,BC
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* 0x50 */
	{ 0 }, { 0 },
	/* 52 */ { SBC16, END },  // 15 cycles SBC HL,DE
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* 5a */ { ADC16, END },  // 15 cycles, ADC HL,DE
	{ 0 },
	{ 0 }, { 0 }, { 0 }, { 0 },
	/* 0x60 */
	{ 0 }, { 0 },
	/* 62 */ { SBC16, END },  // 15 cycles, SBC HL,HL
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* 6a */ { ADC16, END },  // 15 cycles, ADC HL,HL
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* 0x70 */
	{ 0 }, { 0 },
	/* 72 */ { SBC16, END },  // 15 cycles, SBC HL,SP
	/* 73 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, WZ_OUT, WZ_INC, R16L_DB, WRITE, CHECK_WAIT, WZ_OUT, R16H_DB, WRITE, CHECK_WAIT, END },  // 20 cycles, LD (nn),SP
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* 7a */ { ADC16, END },  // 15 cycles, ADC HL,SP
	/* 7b */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, WZ_OUT, WZ_INC, READ, CHECK_WAIT, DB_R16L, WZ_OUT, READ, CHECK_WAIT, DB_R16H, END },  // 20 cycles, LD SP,(nn)
	{ 0 }, { 0 }, { 0 }, { 0 },
	/* 0x80 */
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* 0x90 */
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* 0xa0 */
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* 0xb0 */
	/* b0 */ { HL_OUT, READ, CHECK_WAIT, DE_OUT, WRITE, CHECK_WAIT, LDI, REPEAT, END },  // 16/21 cycles, LDIR
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* 0xc0 */
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* 0xd0 */
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* 0xe0 */
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* 0xf0 */
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },

	/* DD/FD prefixed instructions, almost equal to regular instructions */
	/* 0x00 */
	/* 00 */ { END },  // 8 cycles, NOP
	/* 01 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_R16L, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_R16H, END },  // 14 cycles, LD BC,nn
	{ 0 },
	/* 03 */ { INC_R16, END },  // 10 cycles, INC BC
	/* 04 */ { REGD_TMP, ALU_INC, ALU_REGD, END },  // 8 cycles, INC B
	/* 05 */ { REGD_TMP, ALU_DEC, ALU_REGD, END },  // 8 cycles, DEC B
	/* 06 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_REG, END },  // 11 cycles, LD B,n
	/* 07 */ { RLCA, END },  // 8 cycles, RLCA
	{ 0 },
	/* 09 */ { ADD16, END },  // 15 cycles, ADD HL,BC
	{ 0 },
	/* 0b */ { DEC_R16, END },  // 10 cycles, DEC BC
	/* 0c */ { REGD_TMP, ALU_INC, ALU_REGD, END },  // 8 cycles, INC C
	/* 0d */ { REGD_TMP, ALU_DEC, ALU_REGD, END },  // 8 cycles, DEC C
	/* 0e */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_REG, END },  // 11 cycles, LD C,n
	/* 0f */ { RRCA, END },  // 8 cycles, RRCA
	/* 0x10 */
	{ 0 },
	/* 11 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_R16L, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_R16H, END },  // 14 cycles, LD DE,nn
	/* 12 */ { DE_WZ, WZ_OUT, WZ_INC, A_DB, WRITE, CHECK_WAIT, END },  // 7 cycles, LD (DE),A
	/* 13 */ { INC_R16, END },  // 10 cycles, INC DE
	/* 14 */ { REGD_TMP, ALU_INC, ALU_REGD, END },  // 8 cycles, INC D
	/* 15 */ { REGD_TMP, ALU_DEC, ALU_REGD, END },  // 8 cycles, DEC D
	/* 16 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_REG, END },  // 11 cycles, LD D,n
	{ 0 },
	/* 18 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, JR_COND, END },  // 16 cycles, JR n
	/* 19 */ { ADD16, END },  // 11 cycles, ADD HL,DE
	/* 1a */ { DE_WZ, WZ_OUT, WZ_INC, READ, CHECK_WAIT, DB_A, END },  // 11 cycles, LD A,(DE)
	{ 0 },
	/* 1c */ { REGD_TMP, ALU_INC, ALU_REGD, END },  // 8 cycles, INC E
	/* 1d */ { REGD_TMP, ALU_DEC, ALU_REGD, END },  // 8 cycles, DEC E
	/* 1e */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_REG, END },  // 11 cycles, LD E,n
	{ 0 },
	/* 0x20 */
	/* 20 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, JR_COND, END },  // 11/16 cycles, JR NZ,n
	/* 21 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_R16L, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_R16H, END },  // 14 cycles, LD HL,nn
	/* 22 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, WZ_OUT, WZ_INC, L_DB, WRITE, CHECK_WAIT, WZ_OUT, H_DB, WRITE, CHECK_WAIT, END },  // 20 cycles, LD (nn),HL
	/* 23 */ { INC_R16, END },  // 10 cycles, INC HL
	/* 24 */ { REGD_TMP, ALU_INC, ALU_REGD, END },  // 8 cycles, INC H
	/* 25 */ { REGD_TMP, ALU_DEC, ALU_REGD, END },  // 8 cycles, DEC H
	/* 26 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_REG, END },  // 11 cycles, LD H,n
	{ 0 },
	/* 28 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, JR_COND, END },  // 11/16 cycles, JR Z,n
	/* 29 */ { ADD16, END },  // 15 cycles, ADD HL,HL
	/* 2a */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, WZ_OUT, WZ_INC, READ, CHECK_WAIT, DB_R16L, WZ_OUT, READ, CHECK_WAIT, DB_R16H, END },  // 20 cycles, LD HL,(nn)
	/* 2b */ { DEC_R16, END },  // 10 cycles, DEC HL
	/* 2c */ { REGD_TMP, ALU_INC, ALU_REGD, END },  // 8 cycles, INC L
	/* 2d */ { REGD_TMP, ALU_DEC, ALU_REGD, END },  // 8 cycles, DEC L
	/* 2e */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_REG, END },  // 11 cycles, LD L,n
	{ 0 },
	/* 0x30 */
	{ 0 },
	/* 31 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_R16L, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_R16H, END },  // 14 cycles, LD SP,nn
	/* 32 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, WZ_OUT, WZ_INC, A_DB, WRITE, CHECK_WAIT, END },  // 17 cycles, LD (nn),A
	{ 0 },
	/* 34 */ { HL_OUT, READ, CHECK_WAIT, DB_TMP, ALU_INC, ALU_DB, X, HL_OUT, WRITE, CHECK_WAIT, END },  // 15 cycles, INC (HL)
	{ 0 },
	/* 36 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, HL_OUT, WRITE, END },  // 14 cycles, LD (HL),n
	{ 0 }, { 0 },
	/* 39 */ { ADD16, END },  // 15 cycles, ADD HL,SP
	/* 3a */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, WZ_OUT, WZ_INC, READ, CHECK_WAIT, DB_A, END },  // 17 cycles, LD A,(nn)
	{ 0 },
	/* 3c */ { REGD_TMP, ALU_INC, ALU_REGD, END },  // 8 cycles, INC A
	/* 3d */ { REGD_TMP, ALU_DEC, ALU_REGD, END },  // 8 cycles, DEC A
	/* 3e */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_REG, END },  // 11 cycles, LD A,n
	{ 0 },
	/* 0x40 */
	/* 40 */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD B,B
	/* 41 */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD B,C
	/* 42 */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD B,D
	/* 43 */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD B,E
	/* 44 */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD B,H
	/* 45 */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD B,L
	/* 46 */ { HL_OUT, READ, CHECK_WAIT, DB_REG, END },  // 11 cycles, LD B,(HL)
	/* 47 */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD B,A
	/* 48 */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD C,B
	/* 49 */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD C,C
	/* 4a */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD C,D
	/* 4b */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD C,E
	/* 4c */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD C,H
	/* 4d */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD C,L
	/* 4e */ { HL_OUT, READ, CHECK_WAIT, DB_REG, END },  // 11 cycles, LD C,(HL)
	/* 4f */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD C,A
	/* 0x50 */
	/* 50 */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD D,B
	/* 51 */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD D,C
	/* 52 */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD D,D
	/* 53 */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD D,E
	/* 54 */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD D,H
	/* 55 */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD D,L
	/* 56 */ { HL_OUT, READ, CHECK_WAIT, DB_REG, END },  // 11 cycles, LD D,(HL)
	/* 57 */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD D,A
	/* 58 */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD E,B
	/* 59 */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD E,C
	/* 5a */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD E,D
	/* 5b */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD E,E
	/* 5c */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD E,H
	/* 5d */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD E,L
	/* 5e */ { HL_OUT, READ, CHECK_WAIT, DB_REG, END },  // 11 cycles, LD E,(HL)
	/* 5f */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD E,A
	/* 0x60 */
	/* 60 */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD H,B
	/* 61 */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD H,C
	/* 62 */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD H,D
	/* 63 */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD H,E
	/* 64 */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD H,H
	/* 65 */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD H,L
	/* 66 */ { HL_OUT, READ, CHECK_WAIT, DB_REG, END },  // 11 cycles, LD H,(HL)
	/* 67 */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD H,A
	/* 68 */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD L,B
	/* 69 */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD L,C
	/* 6a */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD L,D
	/* 6b */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD L,E
	/* 6c */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD L,H
	/* 6d */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD L,L
	/* 6e */ { HL_OUT, READ, CHECK_WAIT, DB_REG, END },  // 11 cycles, LD L,(HL)
	/* 6f */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD L,A
	/* 0x70 */
	{ 0 }, { 0 }, { 0 },
	/* 73 */ { HL_OUT, REGS_DB, WRITE, CHECK_WAIT, END },  // 11 cycles, LD (HL),E
	{ 0 }, { 0 }, { 0 },
	/* 77 */ { HL_OUT, REGS_DB, WRITE, CHECK_WAIT, END },  // 11 cycles, LD (HL),A
	/* 78 */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD A,B
	/* 79 */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD A,C
	/* 7a */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD A,D
	/* 7b */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD A,E
	/* 7c */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD A,H
	/* 7d */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD A,L
	/* 7e */ { HL_OUT, READ, CHECK_WAIT, DB_REG, END },  // 11 cycles, LD A,(HL)
	/* 7f */ { REG_TMP, TMP_REG, END },  // 8 cycles, LD A,A
	/* 0x80 */
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* 0x90 */
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* 0xa0 */
	/* a0 */ { A_ACT, REG_TMP, ALU_AND, ALU_A, END },  // 8 cycles, AND B
	/* a1 */ { A_ACT, REG_TMP, ALU_AND, ALU_A, END },  // 8 cycles, AND C
	/* a2 */ { A_ACT, REG_TMP, ALU_AND, ALU_A, END },  // 8 cycles, AND D
	/* a3 */ { A_ACT, REG_TMP, ALU_AND, ALU_A, END },  // 8 cycles, AND E
	/* a4 */ { A_ACT, REG_TMP, ALU_AND, ALU_A, END },  // 8 cycles, AND H
	/* a5 */ { A_ACT, REG_TMP, ALU_AND, ALU_A, END },  // 8 cycles, AND L
	/* a6 */ { 0 },
	/* a7 */ { A_ACT, REG_TMP, ALU_AND, ALU_A, END },  // 4 cycles, AND A
	/* a8 */ { A_ACT, REG_TMP, ALU_XOR, ALU_A, END },  // 8 cycles, XOR B
	/* a9 */ { A_ACT, REG_TMP, ALU_XOR, ALU_A, END },  // 8 cycles, XOR C
	/* aa */ { A_ACT, REG_TMP, ALU_XOR, ALU_A, END },  // 8 cycles, XOR D
	/* ab */ { A_ACT, REG_TMP, ALU_XOR, ALU_A, END },  // 8 cycles, XOR E
	/* ac */ { A_ACT, REG_TMP, ALU_XOR, ALU_A, END },  // 8 cycles, XOR H
	/* ad */ { A_ACT, REG_TMP, ALU_XOR, ALU_A, END },  // 8 cycles, XOR L
	/* ae */ { HL_OUT, READ, CHECK_WAIT, A_ACT, DB_TMP, ALU_XOR, ALU_A, END },  // 11 cycles, XOR (HL)
	/* af */ { A_ACT, REG_TMP, ALU_XOR, ALU_A, END },  // 8 cycles, XOR A
	/* 0xb0 */
	/* b0 */ { A_ACT, REG_TMP, ALU_OR, ALU_A, END },  // 8 cycles, OR B
	/* b1 */ { A_ACT, REG_TMP, ALU_OR, ALU_A, END },  // 8 cycles, OR C
	/* b2 */ { A_ACT, REG_TMP, ALU_OR, ALU_A, END },  // 8 cycles, OR D
	/* b3 */ { A_ACT, REG_TMP, ALU_OR, ALU_A, END },  // 8 cycles, OR E
	/* b4 */ { A_ACT, REG_TMP, ALU_OR, ALU_A, END },  // 8 cycles, OR H
	/* b5 */ { A_ACT, REG_TMP, ALU_OR, ALU_A, END },  // 8 cycles, OR L
	/* b6 */ { HL_OUT, READ, CHECK_WAIT, A_ACT, DB_TMP, ALU_OR, ALU_A, END },  // 11 cycles, OR (HL)
	/* b7 */ { A_ACT, REG_TMP, ALU_OR, ALU_A, END },  // 8 cycles, OR A
	/* b8 */ { A_ACT, REG_TMP, ALU_CP, END },  // 8 cycles, CP B
	/* b9 */ { A_ACT, REG_TMP, ALU_CP, END },  // 8 cycles, CP C
	/* ba */ { A_ACT, REG_TMP, ALU_CP, END },  // 8 cycles, CP D
	/* bb */ { A_ACT, REG_TMP, ALU_CP, END },  // 8 cycles, CP E
	/* bc */ { A_ACT, REG_TMP, ALU_CP, END },  // 8 cycles, CP H
	/* bd */ { A_ACT, REG_TMP, ALU_CP, END },  // 8 cycles, CP L
	/* be */ { HL_OUT, READ, CHECK_WAIT, A_ACT, DB_TMP, ALU_CP, END },  // 11 cycles, CP (HL)
	/* bf */ { A_ACT, REG_TMP, ALU_CP, END },  // 8 cycles, CP A
	/* 0xc0 */
	{ 0 },
	/* c1 */ { SP_OUT, INC_SP, READ, CHECK_WAIT, DB_R16L, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_R16H, END },  // 14 cycles, POP BC
	/* c2 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, JP_COND, END },  // 14 cycles, JP NZ,nn
	/* c3 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, WZ_TO_PC, END },  // 14 cycles, JMP nn
	/* c4 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, CALL_COND, DEC_SP, SP_OUT, PCH_DB, WRITE, CHECK_WAIT, DEC_SP, SP_OUT, PCL_DB, WRITE, CHECK_WAIT, WZ_TO_PC, END },  // 14/21 cycles, CALL NZ,nn
	/* c5 */ { X, DEC_SP, SP_OUT, R16H_DB, WRITE, CHECK_WAIT, DEC_SP, SP_OUT, R16L_DB, WRITE, CHECK_WAIT, END },  // 15 cycles, PUSH BC
	{ 0 }, { 0 },
	/* c8 */ { RET_COND, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_Z, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_W, WZ_TO_PC, END },  // 9/15 cycles, RET Z
	/* c9 */ { SP_OUT, INC_SP, READ, CHECK_WAIT, DB_Z, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_W, WZ_TO_PC, END },  // 14 cycles, RET
	/* ca */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, JP_COND, END },  // 14 cycles, JP Z,nn
	/* cb */ { 0 },
	{ 0 },
	/* cd */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, X, DEC_SP, SP_OUT, PCH_DB, WRITE, CHECK_WAIT, DEC_SP, SP_OUT, PCL_DB, WRITE, CHECK_WAIT, WZ_TO_PC, END },  // 21 cycles, CALL nn
	{ 0 }, { 0 },
	/* 0xd0 */
	{ 0 },
	/* d1 */ { SP_OUT, INC_SP, READ, CHECK_WAIT, DB_R16L, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_R16H, END },  // 14 cycles, POP DE
	{ 0 },
	/* d3 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, A_W, WZ_OUT, WZ_INC, A_DB, OUTPUT, CHECK_WAIT, END },  // 15 cycles, OUT (n), A
	{ 0 },
	/* d5 */ { X, DEC_SP, SP_OUT, R16H_DB, WRITE, CHECK_WAIT, DEC_SP, SP_OUT, R16L_DB, WRITE, CHECK_WAIT, END },  // 15 cycles, PUSH DE
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* dc */ { PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, CALL_COND, DEC_SP, SP_OUT, PCH_DB, WRITE, CHECK_WAIT, DEC_SP, SP_OUT, PCL_DB, WRITE, CHECK_WAIT, WZ_TO_PC, END },  // 14/21 cycles, CALL C,nn
	{ 0 }, { 0 }, { 0 },
	/* 0xe0 */
	{ 0 },
	/* e1 */ { SP_OUT, INC_SP, READ, CHECK_WAIT, DB_R16L, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_R16H, END },  // 14 cycles, POP HL
	{ 0 }, { 0 }, { 0 },
	/* e5 */ { X, DEC_SP, SP_OUT, R16H_DB, WRITE, CHECK_WAIT, DEC_SP, SP_OUT, R16L_DB, WRITE, CHECK_WAIT, END },  // 15 cycles, PUSH HL
	/* e6 */ { PC_OUT, PC_INC, READ, CHECK_WAIT, A_ACT, DB_TMP, ALU_AND, ALU_A, END },  // 11 cycles, AND n
	{ 0 }, { 0 }, { 0 }, { 0 },
	/* eb */ { EX_DE_HL, END },  // 8 cycles, EX DE,HL
	{ 0 }, { 0 }, { 0 }, { 0 },
	/* 0xf0 */
	{ 0 },
	/* f1 */ { SP_OUT, INC_SP, READ, CHECK_WAIT, DB_R16L, SP_OUT, INC_SP, READ, CHECK_WAIT, DB_R16H, END },  // 14 cycles, POP AF
	{ 0 },
	/* f3 */ { DI, END },  // 8 cycles, DI
	{ 0 },
	/* f5 */ { X, DEC_SP, SP_OUT, R16H_DB, WRITE, CHECK_WAIT, DEC_SP, SP_OUT, R16L_DB, WRITE, CHECK_WAIT, END },  // 15 cycles, PUSH AF
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* fb */ { EI, END },  // 8 cycles, EI
	{ 0 }, { 0 },
	/* fe */ { PC_OUT, PC_INC, READ, CHECK_WAIT, A_ACT, DB_TMP, ALU_CP, END },  // 11 cycles, CP n
	{ 0 },

	/* Special sequences */
	/* M1 */ { PC_OUT, PC_INC, READ_OP, CHECK_WAIT, REFRESH, DECODE },
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
		(((arg2 ^ arg1) & (arg1 ^ res) &0x8000) >> 13);
	return res;
}


/****************************************************************************
 * Processor initialization
 ****************************************************************************/
void z80lle_device::device_start()
{
	if( !tables_initialised )
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
				if( (newval & 0x0f) < (oldval & 0x0f) ) *padd |= HF;
				if( newval < oldval ) *padd |= CF;
				if( (val^oldval^0x80) & (val^newval) & 0x80 ) *padd |= VF;
				padd++;

				/* adc with carry set */
				val = newval - oldval - 1;
				*padc = (newval) ? ((newval & 0x80) ? SF : 0) : ZF;
				*padc |= (newval & (YF | XF));  /* undocumented flag bits 5+3 */
				if( (newval & 0x0f) <= (oldval & 0x0f) ) *padc |= HF;
				if( newval <= oldval ) *padc |= CF;
				if( (val^oldval^0x80) & (val^newval) & 0x80 ) *padc |= VF;
				padc++;

				/* cp, sub or sbc w/o carry set */
				val = oldval - newval;
				*psub = NF | ((newval) ? ((newval & 0x80) ? SF : 0) : ZF);
				*psub |= (newval & (YF | XF));  /* undocumented flag bits 5+3 */
				if( (newval & 0x0f) > (oldval & 0x0f) ) *psub |= HF;
				if( newval > oldval ) *psub |= CF;
				if( (val^oldval) & (oldval^newval) & 0x80 ) *psub |= VF;
				psub++;

				/* sbc with carry set */
				val = oldval - newval - 1;
				*psbc = NF | ((newval) ? ((newval & 0x80) ? SF : 0) : ZF);
				*psbc |= (newval & (YF | XF));  /* undocumented flag bits 5+3 */
				if( (newval & 0x0f) >= (oldval & 0x0f) ) *psbc |= HF;
				if( newval >= oldval ) *psbc |= CF;
				if( (val^oldval) & (oldval^newval) & 0x80 ) *psbc |= VF;
				psbc++;
			}
		}

		for (int i = 0; i < 256; i++)
		{
			int p = 0;
			if( i&0x01 ) ++p;
			if( i&0x02 ) ++p;
			if( i&0x04 ) ++p;
			if( i&0x08 ) ++p;
			if( i&0x10 ) ++p;
			if( i&0x20 ) ++p;
			if( i&0x40 ) ++p;
			if( i&0x80 ) ++p;
			SZ[i] = i ? i & SF : ZF;
			SZ[i] |= (i & (YF | XF));       /* undocumented flag bits 5+3 */
			SZ_BIT[i] = i ? i & SF : ZF | PF;
			SZ_BIT[i] |= (i & (YF | XF));   /* undocumented flag bits 5+3 */
			SZP[i] = SZ[i] | ((p & 1) ? 0 : PF);
			SZHV_inc[i] = SZ[i];
			if( i == 0x80 ) SZHV_inc[i] |= VF;
			if( (i & 0x0f) == 0x00 ) SZHV_inc[i] |= HF;
			SZHV_dec[i] = SZ[i] | NF;
			if( i == 0x7f ) SZHV_dec[i] |= VF;
			if( (i & 0x0f) == 0x0f ) SZHV_dec[i] |= HF;
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
	state_add(Z80_SP,          "SP",        m_sp.w.l);
	state_add(STATE_GENSP,     "GENSP",     m_sp.w.l).noshow();
	state_add(STATE_GENFLAGS,  "GENFLAGS",  F).noshow().formatstr("%8s");
	state_add(Z80_A,           "A",         m_af.b.h).noshow();
	state_add(Z80_B,           "B",         m_bc.b.h).noshow();
	state_add(Z80_C,           "C",         m_bc.b.l).noshow();
	state_add(Z80_D,           "D",         m_de.b.h).noshow();
	state_add(Z80_E,           "E",         m_de.b.l).noshow();
	state_add(Z80_H,           "H",         m_hl_index[HL_OFFSET].b.h).noshow();
	state_add(Z80_L,           "L",         m_hl_index[HL_OFFSET].b.l).noshow();
	state_add(Z80_AF,          "AF",        m_af.w.l);
	state_add(Z80_BC,          "BC",        m_bc.w.l);
	state_add(Z80_DE,          "DE",        m_de.w.l);
	state_add(Z80_HL,          "HL",        m_hl_index[HL_OFFSET].w.l);
	state_add(Z80_IX,          "IX",        m_hl_index[IX_OFFSET].w.l);
	state_add(Z80_IY,          "IY",        m_hl_index[IY_OFFSET].w.l);
	state_add(Z80_AF2,         "AF2",       m_af2.w.l);
	state_add(Z80_BC2,         "BC2",       m_bc2.w.l);
	state_add(Z80_DE2,         "DE2",       m_de2.w.l);
	state_add(Z80_HL2,         "HL2",       m_hl2.w.l);
	state_add(Z80_WZ,          "WZ",        m_wz.w.l);
	state_add(Z80_R,           "R",         m_rtemp).callimport().callexport();
	state_add(Z80_I,           "I",         m_i);
	state_add(Z80_IM,          "IM",        m_im).mask(0x3);
	state_add(Z80_IFF1,        "IFF1",      m_iff1).mask(0x1);
	state_add(Z80_IFF2,        "IFF2",      m_iff2).mask(0x1);
	state_add(Z80_HALT,        "HALT",      m_halt).mask(0x1);

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
		case ALU_REG:
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
				fatalerror("ALU_REG: illegal register reference 0x06\n");
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
		case ALU_AND:
			m_alu = m_act & m_tmp;
			F = SZP[m_alu] | HF;
			break;
		case ALU_CP:  // Flag handling is slightly different from SUB
			m_alu = m_act - m_tmp;
			F = (SZHVC_sub[(m_act << 8) | m_alu] & ~(YF | XF)) |
				(m_tmp & (YF | XF));
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
		case ALU_XOR:
			m_alu = m_act ^ m_tmp;
			F = SZP[m_alu];
			break;
		case ALU_SLA:
			m_alu = m_tmp << 1;
			F = SZP[m_alu] | ((m_tmp & 0x80) ? CF : 0);
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
		case DE_WZ:
			WZ = DE;
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
			if (m_ir == 0xcb)
			{
				m_instruction_offset = CB_OFFSET;
				m_instruction = M1;
			}
			else if (m_ir == 0xdd)
			{
				m_instruction_offset = FD_OFFSET;
				m_instruction = M1;
				m_hl_offset = IX_OFFSET;
			}
			else if (m_ir == 0xed)
			{
				m_instruction_offset = ED_OFFSET;
				m_instruction = M1;
			}
			else if (m_ir == 0xfd)
			{
				m_instruction_offset = FD_OFFSET;
				m_instruction = M1;
				m_hl_offset = IY_OFFSET;
			}
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
		case REG_TMP:
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
				fatalerror("REG_TMP: illegal register reference 0x06\n");
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
		case RLCA:
			A = (A << 1) | (A >> 7);
			F = (F & (SF | ZF | PF)) | (A & (YF | XF | CF));
			break;
		case RRCA:
			F = (F & (SF | ZF | PF)) | (A & CF);
			A = (A >> 1) | (A << 7);
			F |= (A & (YF | XF));
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
		}
	} while (m_icount > 0);
}


void z80lle_device::execute_set_input(int inputnum, int state)
{
	switch (inputnum)
	{
	case Z80_INPUT_LINE_BUSRQ:
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

	case Z80_INPUT_LINE_WAIT:
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

		case Z80_R:
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
		case Z80_R:
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


