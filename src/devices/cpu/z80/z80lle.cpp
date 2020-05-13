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
 *   - Split the RFSH and MREQ cycles of REFRESH
 *   - Implement the 2 start up cycles after a RESET
 *   - RETI: When should the daisy chain be notified?
 *   - Add support for interrupt mode 0
 *   - Add support for interrupt mode 2
 *   - Group sub-instructions for readability and/or move code out into functions
 *   - These instructions are untested:
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

u8 z80lle_device::SZ[256];       /* zero and sign flags */
u8 z80lle_device::SZ_BIT[256];   /* zero, sign and parity/overflow (=zero) flags for BIT opcode */
u8 z80lle_device::SZP[256];      /* zero, sign and parity flags */
u8 z80lle_device::SZHV_inc[256]; /* zero, sign, half carry and overflow flags INC r8 */
u8 z80lle_device::SZHV_dec[256]; /* zero, sign, half carry and overflow flags DEC r8 */

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


void z80lle_device::setup_instructions() {
	instructions =
	{
	/*****************************************************/
	/* Regular instructions                              */
	/*****************************************************/

		// 00, 4 cycles, NOP
		{ [] () { } },
		// 01, 10 cycles, LD BC,nn
		//  5 T1 AB:1235 DB:--
		//  6 T2 AB:1235 DB:XX MREQ RD
		//  7 T3 AB:1235 DB:XX MREQ RD
		//  8 T1 AB:1236 DB:--
		//  9 T2 AB:1236 DB:YY MREQ RD
		// 10 T3 AB:1236 DB:YY MREQ RD
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_bc.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_bc.b.h = m_data_bus; }
		},
		// 02, 7 cycles, LD (BC),A
		//  5 T1 AB:5678 DB:--
		//  6 T2 AB:5678 DB:AA MREQ
		//  7 T3 AB:5678 DB:AA MREQ WR
		{
			[this] () { m_wz.w.l = m_bc.w.l; write_s(m_wz.w.l, m_af.b.h); m_wz.b.h = m_af.b.h; m_wz.w.l++; }
		},
		// 03, 6 cycles, INC BC
		//  5 T5 AB:1234 DB:--
		//  6 T6 AB:1234 DB:--
		{ [this] () { m_bc.w.l++; m_icount -= 2; } },
		// 04, 4 cycles, INC B
		{ [this] () { m_bc.b.h = alu_inc(m_bc.b.h); } },
		// 05, 4 cycles, DEC B
		{ [this] () { m_bc.b.h = alu_dec(m_bc.b.h); } },
		// 06, 7 cycles, LD B,n
		//  5 T1 AB:1235 DB:--
		//  6 T2 AB:1235 DB:nn MREQ RD
		//  7 T3 AB:1235 DB:nn MREQ RD
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_bc.b.h = m_data_bus; }
		},
		// 07, 4 cycles, RLCA
		{ [this] () { rlca(); } },

		// 08, 4 cycles, EX AF,AF'
		{ [this] () { ex_af_af(); } },
		// 09, 11 cycles, ADD HL,BC
		//  5 T1 AB:1234 DB:--
		//  6 T2 AB:1234 DB:--
		//  7 T3 AB:1234 DB:--
		//  8 T4 AB:1234 DB:--
		//  9 T1 AB:1234 DB:--
		// 10 T2 AB:1234 DB:--
		// 11 T3 AB:1234 DB:--
		{ [this] () { add16(); } },
		// 0a, 7 cycles, LD A,(BC)
		//  5 T1 AB:5678 DB:--
		//  6 T2 AB:5678 DB:XX MREQ RD
		//  7 T3 AB:5678 DN:XX MREQ RD
		{
			[this] () { m_wz.w.l = m_bc.w.l; read_s(m_wz.w.l++); },
			[this] () { m_af.b.h = m_data_bus; }
		},
		// 0b, 6 cycles, DEC BC
		//  5 T5 AB:1234 DB:--
		//  6 T6 AB:1234 DB:--
		{ [this] () { m_bc.w.l--; m_icount -= 2; } },
		// 0c, 4 cycles, INC C
		{ [this] () { m_bc.b.l = alu_inc(m_bc.b.l); } },
		// 0d, 4 cycles, DEC C
		{ [this] () { m_bc.b.l = alu_dec(m_bc.b.l); } },
		// 0e, 7 cycles, LD C,n, see 06 for timing
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_bc.b.l = m_data_bus; }
		},
		// 0f, 4 cycles, RRCA
		{ [this] () { rrca(); } },

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
		{
			[this] () { m_icount -= 1; read_s(m_pc.w.l++); },
			[this] () { djnz(); }
		},
		// 11, 10 cycles, LD DE,nn, see 01 for timing
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_de.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_de.b.h = m_data_bus; }
		},
		// 12, 7 cycles, LD (DE),A, see 02 for timing
		{
			[this] () { m_wz.w.l = m_de.w.l; write_s(m_wz.w.l, m_af.b.h); m_wz.b.h = m_af.b.h; m_wz.w.l++; }
		},
		// 13, 6 cycles, INC DE, see 03 for timing
		{ [this] () { m_de.w.l++; m_icount -= 2; } },
		// 14, 4 cycles, INC D
		{ [this] () { m_de.b.h = alu_inc(m_de.b.h); } },
		// 15, 4 cycles, DEC D
		{ [this] () { m_de.b.h = alu_dec(m_de.b.h); } },
		// 16, 7 cycles, LD D,n, see 06 for timing
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_de.b.h = m_data_bus; }
		},
		// 17, 4 cycles, RLA
		{ [this] () { rla(); } },

		// 18, 12 cycles, JR n
		//  5 T1 AB:1235 DB:--
		//  6 T2 AB:1235 DB:nn MREQ RD
		//  7 T3 AB:1235 DB:nn MREQ RD
		//  8 T1 AB:1235 DB:--
		//  9 T2 AB:1235 DB:--
		// 10 T3 AB:1235 DB:--
		// 11 T4 AB:1235 DB:--
		// 12 T5 AB:1235 DB:--
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { jr_cond(); }
		},
		// 19, 11 cycles, ADD HL,DE
		{ [this] () { add16(); } },
		// 1a, 7 cycles, LD A,(DE), see 0a for timing
		{
			[this] () { m_wz.w.l = m_de.w.l; read_s(m_wz.w.l++); },
			[this] () { m_af.b.h = m_data_bus; }
		},
		// 1b, 6 cycles, DEC DE, see 0b for timing
		{ [this] () { m_de.w.l--; m_icount -= 2; } },
		// 1c, 4 cycles, INC E
		{ [this] () { m_de.b.l = alu_inc(m_de.b.l); } },
		// 1d, 4 cycles, DEC E
		{ [this] () { m_de.b.l = alu_dec(m_de.b.l); } },
		// 1e, 7 cycles, LD E,n, see 06 for timing
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_de.b.l = m_data_bus; }
		},
		// 1f, 4 cycles, RRA
		{ [this] () { rra(); } },

		// 20, 7/12 cycles, JR NZ,n
		//  5 T1 AB:1235 DB:--
		//  6 T2 AB:1235 DB:nn MREQ RD
		//  7 T3 AB:1235 DB:nn MREQ RD
		//  8 T1 AB:1235 DB:-- *8-12 when jump taken
		//  9 T2 AB:1235 DB:--
		// 10 T3 AB:1235 DB:--
		// 11 T4 AB:1235 DB:--
		// 12 T5 AB:1235 DB:--
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { jr_cond(); }
		},
		// 21, 10 cycles, LD HL,nn, see 01 for timing
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_hl_index[m_hl_offset].b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_hl_index[m_hl_offset].b.h = m_data_bus; }
		},
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
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; write_s(m_wz.w.l, m_hl_index[m_hl_offset].b.l); m_wz.w.l++; },
			[this] () { write_s(m_wz.w.l, m_hl_index[m_hl_offset].b.h); }
		},
		// 23, 6 cycles, INC HL
		{ [this] () { m_hl_index[m_hl_offset].w.l++; m_icount -= 2; } },
		// 24, 4 cycles, INC H
		{ [this] () { m_hl_index[m_hl_offset].b.h = alu_inc(m_hl_index[m_hl_offset].b.h); } },
		// 25, 4 cycles, DEC H
		{ [this] () { m_hl_index[m_hl_offset].b.h = alu_dec(m_hl_index[m_hl_offset].b.h); } },
		// 26, 7 cycles, LD H,n, see 06 for timing
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_hl_index[m_hl_offset].b.h = m_data_bus; }
		},
		// 27, 4 cycles, DAA
		{ [this] () { daa(); } },

		// 28, 7/12 cycles, JR Z,n, see 20 for timing
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { jr_cond(); }
		},
		// 29, 11 cycles, ADD HL,HL
		{ [this] () { add16(); } },
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
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; read_s(m_wz.w.l++); },
			[this] () { m_hl_index[m_hl_offset].b.l = m_data_bus; read_s(m_wz.w.l); },
			[this] () { m_hl_index[m_hl_offset].b.h = m_data_bus; }
		},
		// 2b, 6 cycles, DEC HL
		{ [this] () { m_hl_index[m_hl_offset].w.l--; m_icount -= 2; } },
		// 2c, 4 cycles, INC L
		{ [this] () { m_hl_index[m_hl_offset].b.l = alu_inc(m_hl_index[m_hl_offset].b.l); } },
		// 2d, 4 cycles, DEC L
		{ [this] () { m_hl_index[m_hl_offset].b.l = alu_dec(m_hl_index[m_hl_offset].b.l); } },
		// 2e, 7 cycles, LD L,n, see 06 for timing
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_hl_index[m_hl_offset].b.l = m_data_bus; }
		},
		// 2f, 4 cycles, CPL
		{ [this] () { cpl(); } },

		// 30, 7/12 cycles, JR NC,n, see 20 for timing
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { jr_cond(); }
		},
		// 31, 0 cycles, LD SP,nn, see 01 for timing
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_sp.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_sp.b.h = m_data_bus; }
		},
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
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; write_s(m_wz.w.l, m_af.b.h); m_wz.b.h = m_af.b.h; m_wz.w.l++; }
		},
		// 33, 6 cycles, INC SP
		{ [this] () { m_sp.w.l++; m_icount -= 2; } },
		// 34, 11 cycles, INC (HL)
		//  5 T1 AB:hhll DB:--
		//  6 T2 AB:hhll DB:xx MREQ RD
		//  7 T3 AB:hhll DB:xx MREQ RD
		//  8 T4 AB:hhll DB:--
		//  9 T1 AB:hhll DB:--
		// 10 T2 AB:hhll DB:yy MREQ
		// 11 T3 AB:hhll DB:yy MREQ WR
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { m_icount -= 2; write_s(alu_inc(m_data_bus)); }
		},
		// 35, 11 cycles, DEC (HL)
		//  5 T1 AB:hhll DB:--
		//  6 T2 AB:hhll DB:xx MREQ RD
		//  7 T3 AB:hhll DB:xx MREQ RD
		//  8 T4 AB:hhll DB:--
		//  9 T1 AB:hhll DB:--
		// 10 T2 AB:hhll DB:yy MREQ
		// 11 T3 AB:hhll DB:yy MREQ WR
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { m_icount -= 2; write_s(alu_dec(m_data_bus)); }
		},
		// 36, 10 cycles, LD (HL),n
		//  5 T1 AB:1235 DB:--
		//  6 T2 AB:1235 DB:nn MREQ RD
		//  7 T3 AB:1235 DB:nn MREQ RD
		//  8 T1 AB:hhll DB:--
		//  9 T2 AB:hhll DB:nn MREQ
		// 10 T3 AB:hhll DB:nn MREQ WR
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { write_s(m_hl_index[m_hl_offset].w.l, m_data_bus); }
		},
		// 37, 4 cycles, SCF
		{ [this] () { scf(); } },

		// 38, 7/12 cycles, JR C,n, see 20 for timing
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { jr_cond(); }
		},
		// 39, 11 cycles, ADD HL,SP
		{ [this] () { add16(); } },
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
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; read_s(m_wz.w.l++); },
			[this] () { m_af.b.h = m_data_bus; }
		},
		// 3b, 6 cycles, DEC SP
		{ [this] () { m_sp.w.l--; m_icount -= 2; } },
		// 3c, 4 cycles, INC A
		{ [this] () { m_af.b.h = alu_inc(m_af.b.h); } },
		// 3d, 4 cycles, DEC A
		{ [this] () { m_af.b.h = alu_dec(m_af.b.h); } },
		// 3e, 7 cycles, LD A,n, see 06 for timing
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_af.b.h = m_data_bus; }
		},
		// 3f, 4 cycles, CCF
		{ [this] () { ccf(); } },

		// 40, 4 cycles, LD B,B
		{ [this] () { m_bc.b.h = m_bc.b.h; } },
		// 41, 4 cycles, LD B,C
		{ [this] () { m_bc.b.h = m_bc.b.l; } },
		// 42, 4 cycles, LD B,D
		{ [this] () { m_bc.b.h = m_de.b.h; } },
		// 43, 4 cycles, LD B,E
		{ [this] () { m_bc.b.h = m_de.b.l; } },
		// 44, 4 cycles, LD B,H
		{ [this] () { m_bc.b.h = m_hl_index[m_hl_offset].b.h; } },
		// 45, 4 cycles, LD B,L
		{ [this] () { m_bc.b.h = m_hl_index[m_hl_offset].b.l; } },
		// 46, 7 cycles, LD B,(HL)
		// 5 T1 AB:hhll DB:--
		// 6 T2 AB:hhll DB:xx MREQ RD
		// 7 T3 AB:hhll DB:xx MREQ RD
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { m_bc.b.h = m_data_bus; }
		},
		// 47, 4 cycles, LD B,A
		{ [this] () { m_bc.b.h = m_af.b.h; } },
		// 48, 4 cycles, LD C,B
		{ [this] () { m_bc.b.l = m_bc.b.h; } },
		// 49, 4 cycles, LD C,C
		{ [this] () { m_bc.b.l = m_bc.b.l; } },
		// 4a, 4 cycles, LD C,D
		{ [this] () { m_bc.b.l = m_de.b.h; } },
		// 4b, 4 cycles, LD C,E
		{ [this] () { m_bc.b.l = m_de.b.l; } },
		// 4c, 4 cycles, LD C,H
		{ [this] () { m_bc.b.l = m_hl_index[m_hl_offset].b.h; } },
		// 4d, 4 cycles, LD C,L
		{ [this] () { m_bc.b.l = m_hl_index[m_hl_offset].b.l; } },
		// 4e, 7 cycles, LD C,(HL)
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { m_bc.b.l = m_data_bus; }
		},
		// 4f, 4 cycles, LD C,A
		{ [this] () { m_bc.b.l = m_af.b.h; } },

		// 50, 4 cycles, LD D,B
		{ [this] () { m_de.b.h = m_bc.b.h; } },
		// 51, 4 cycles, LD D,C
		{ [this] () { m_de.b.h = m_bc.b.l; } },
		// 52, 4 cycles, LD D,D
		{ [this] () { m_de.b.h = m_de.b.h; } },
		// 53, 4 cycles, LD D,E
		{ [this] () { m_de.b.h = m_de.b.l; } },
		// 54, 4 cycles, LD D,H
		{ [this] () { m_de.b.h = m_hl_index[m_hl_offset].b.h; } },
		// 55, 4 cycles, LD D,L
		{ [this] () { m_de.b.h = m_hl_index[m_hl_offset].b.l; } },
		// 56, 7 cycles, LD D,(HL)
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { m_de.b.h = m_data_bus; }
		},
		// 57, 4 cycles, LD D,A
		{ [this] () { m_de.b.h = m_af.b.h; } },
		// 58, 4 cycles, LD E,B
		{ [this] () { m_de.b.l = m_bc.b.h; } },
		// 59, 4 cycles, LD E,C
		{ [this] () { m_de.b.l = m_bc.b.l; } },
		// 5a, 4 cycles, LD E,D
		{ [this] () { m_de.b.l = m_de.b.h; } },
		// 5b, 4 cycles, LD E,E
		{ [this] () { m_de.b.l = m_de.b.l; } },
		// 5c, 4 cycles, LD E,H
		{ [this] () { m_de.b.l = m_hl_index[m_hl_offset].b.h; } },
		// 5d, 4 cycles, LD E,L
		{ [this] () { m_de.b.l = m_hl_index[m_hl_offset].b.l; } },
		// 5e, 7 cycles, LD E,(HL)
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { m_de.b.l = m_data_bus; }
		},
		// 5f, 4 cycles, LD E,A
		{ [this] () { m_de.b.l = m_af.b.h; } },

		// 60, 4 cycles, LD H,B
		{ [this] () { m_hl_index[m_hl_offset].b.h = m_bc.b.h; } },
		// 61, 4 cycles, LD H,C
		{ [this] () { m_hl_index[m_hl_offset].b.h = m_bc.b.l; } },
		// 62, 4 cycles, LD H,D
		{ [this] () { m_hl_index[m_hl_offset].b.h = m_de.b.h; } },
		// 63, 4 cycles, LD H,E
		{ [this] () { m_hl_index[m_hl_offset].b.h = m_de.b.l; } },
		// 64, 4 cycles, LD H,H
		{ [this] () { m_hl_index[m_hl_offset].b.h = m_hl_index[m_hl_offset].b.h; } },
		// 65, 4 cycles, LD H,L
		{ [this] () { m_hl_index[m_hl_offset].b.h = m_hl_index[m_hl_offset].b.l; } },
		// 66, 7 cycles, LD H,(HL)
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { m_hl_index[m_hl_offset].b.h = m_data_bus; }
		},
		// 67, 4 cycles, LD H,A
		{ [this] () { m_hl_index[m_hl_offset].b.h = m_af.b.h; } },

		// 68, 4 cycles, LD L,B
		{ [this] () { m_hl_index[m_hl_offset].b.l = m_bc.b.h; } },
		// 69, 4 cycles, LD L,C
		{ [this] () { m_hl_index[m_hl_offset].b.l = m_bc.b.l; } },
		// 6a, 4 cycles, LD L,D
		{ [this] () { m_hl_index[m_hl_offset].b.l = m_de.b.h; } },
		// 6b, 4 cycles, LD L,E
		{ [this] () { m_hl_index[m_hl_offset].b.l = m_de.b.l; } },
		// 6c, 4 cycles, LD L,H
		{ [this] () { m_hl_index[m_hl_offset].b.l = m_hl_index[m_hl_offset].b.h; } },
		// 6d, 4 cycles, LD L,L
		{ [this] () { m_hl_index[m_hl_offset].b.l = m_hl_index[m_hl_offset].b.l; } },
		// 6e, 7 cycles, LD L,(HL)
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { m_hl_index[m_hl_offset].b.l = m_data_bus; }
		},
		// 6f, 4 cycles, LD L,A
		{ [this] () { m_hl_index[m_hl_offset].b.l = m_af.b.h; } },

		// 70, 7 cycles, LD (HL),B
		// 5 T1 AB:hhll DB:--
		// 6 T2 AB:hhll DB:bb MREQ
		// 7 T3 AB:hhll DB:bb MREQ WR
		{
			[this] () { write_s(m_hl_index[m_hl_offset].w.l, m_bc.b.h); }
		},
		// 71, 7 cycles, LD (HL),C
		{
			[this] () { write_s(m_hl_index[m_hl_offset].w.l, m_bc.b.l); }
		},
		// 72, 7 cycles, LD (HL),D
		{
			[this] () { write_s(m_hl_index[m_hl_offset].w.l, m_de.b.h); }
		},
		// 73, 7 cycles, LD (HL),E
		{
			[this] () { write_s(m_hl_index[m_hl_offset].w.l, m_de.b.l); }
		},
		// 74, 7 cycles, LD (HL),H
		{
			[this] () { write_s(m_hl_index[m_hl_offset].w.l, m_hl_index[m_hl_offset].b.h); }
		},
		// 75, 7 cycles, LD (HL),L
		{
			[this] () { write_s(m_hl_index[m_hl_offset].w.l, m_hl_index[m_hl_offset].b.l); }
		},
		// 76, 4 cycles, HALT
		{
			[this] () { halt(); }
		},
		// 77, 7 cycles, LD (HL),A
		{
			[this] () { write_s(m_hl_index[m_hl_offset].w.l, m_af.b.h); }
		},

		// 78, 4 cycles, LD A,B
		{ [this] () { m_af.b.h = m_bc.b.h; } },
		// 79, 4 cycles, LD A,C
		{ [this] () { m_af.b.h = m_bc.b.l; } },
		// 7a, 4 cycles, LD A,D
		{ [this] () { m_af.b.h = m_de.b.h; } },
		// 7b, 4 cycles, LD A,E
		{ [this] () { m_af.b.h = m_de.b.l; } },
		// 7c, 4 cycles, LD A,H
		{ [this] () { m_af.b.h = m_hl_index[m_hl_offset].b.h; } },
		// 7d, 4 cycles, LD A,L
		{ [this] () { m_af.b.h = m_hl_index[m_hl_offset].b.l; } },
		// 7e, 7 cycles, LD A,(HL)
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { m_af.b.h = m_data_bus; }
		},
		// 7f, 4 cycles, LD A,A
		{ [this] () { m_af.b.h = m_af.b.h; } },

		// 80, 4 cycles, ADD B
		{ [this] () { alu_add(m_bc.b.h); } },
		// 81, 4 cycles, ADD C
		{ [this] () { alu_add(m_bc.b.l); } },
		// 82, 4 cycles, ADD D
		{ [this] () { alu_add(m_de.b.h); } },
		// 83, 4 cycles, ADD E
		{ [this] () { alu_add(m_de.b.l); } },
		// 84, 4 cycles, ADD H
		{ [this] () { alu_add(m_hl_index[m_hl_offset].b.h); } },
		// 85, 4 cycles, ADD L
		{ [this] () { alu_add(m_hl_index[m_hl_offset].b.l); } },
		// 86, 7b cycles, ADD (HL)
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { alu_add(m_data_bus); }
		},
		// 87, 4 cycles, ADD A
		{ [this] () { alu_add(m_af.b.h); } },

		// 88, 4 cycles, ADC B
		{ [this] () { alu_adc(m_bc.b.h); } },
		// 89, 4 cycles, ADC C
		{ [this] () { alu_adc(m_bc.b.l); } },
		// 8a, 4 cycles, ADC D
		{ [this] () { alu_adc(m_de.b.h); } },
		// 8b, 4 cycles, ADC E
		{ [this] () { alu_adc(m_de.b.l); } },
		// 8c, 4 cycles, ADC H
		{ [this] () { alu_adc(m_hl_index[m_hl_offset].b.h); } },
		// 8d, 4 cycles, ADC L
		{ [this] () { alu_adc(m_hl_index[m_hl_offset].b.l); } },
		// 8e, 7 cycles, ADC (HL)
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { alu_adc(m_data_bus); }
		},
		// 8f, 4 cycles, ADC A
		{ [this] () { alu_adc(m_af.b.h); } },

		// 90, 4 cycles, SUB B
		{ [this] () { alu_sub(m_bc.b.h); } },
		// 91, 4 cycles, SUB C
		{ [this] () { alu_sub(m_bc.b.l); } },
		// 92, 4 cycles, SUB D
		{ [this] () { alu_sub(m_de.b.h); } },
		// 93, 4 cycles, SUB E
		{ [this] () { alu_sub(m_de.b.l); } },
		// 94, 4 cycles, SUB H
		{ [this] () { alu_sub(m_hl_index[m_hl_offset].b.h); } },
		// 95, 4 cycles, SUB L
		{ [this] () { alu_sub(m_hl_index[m_hl_offset].b.l); } },
		// 96, 7 cycles, SUB (HL)
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { alu_sub(m_data_bus); }
		},
		// 97, 4 cycles, SUB A
		{ [this] () { alu_sub(m_af.b.h); } },

		// 98, 4 cycles, SBC B
		{ [this] () { alu_sbc(m_bc.b.h); } },
		// 99, 4 cycles, SBC C
		{ [this] () { alu_sbc(m_bc.b.l); } },
		// 9a, 4 cycles, SBC D
		{ [this] () { alu_sbc(m_de.b.h); } },
		// 9b, 4 cycles, SBC E
		{ [this] () { alu_sbc(m_de.b.l); } },
		// 9c, 4 cycles, SBC H
		{ [this] () { alu_sbc(m_hl_index[m_hl_offset].b.h); } },
		// 9d, 4 cycles, SBC L
		{ [this] () { alu_sbc(m_hl_index[m_hl_offset].b.l); } },
		// 9e, 7 cycles, SBC (HL)
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { alu_sbc(m_data_bus); }
		},
		// 9f, 4 cycles, SBC A
		{ [this] () { alu_sbc(m_af.b.h); } },

		// a0, 4 cycles, AND B
		{ [this] () { alu_and(m_bc.b.h); } },
		// a1, 4 cycles, AND C
		{ [this] () { alu_and(m_bc.b.l); } },
		// a2, 4 cycles, AND D
		{ [this] () { alu_and(m_de.b.h); } },
		// a3, 4 cycles, AND E
		{ [this] () { alu_and(m_de.b.l); } },
		// a4, 4 cycles, AND H
		{ [this] () { alu_and(m_hl_index[m_hl_offset].b.h); } },
		// a5, 4 cycles, AND L
		{ [this] () { alu_and(m_hl_index[m_hl_offset].b.l); } },
		// a6, 7 cycles, AND (HL)
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { alu_and(m_data_bus); }
		},
		// a7, 4 cycles, AND A
		{ [this] () { alu_and(m_af.b.h); } },

		// a8, 4 cycles, XOR B
		{ [this] () { alu_xor(m_bc.b.h); } },
		// a9, 4 cycles, XOR C
		{ [this] () { alu_xor(m_bc.b.l); } },
		// aa, 4 cycles, XOR D
		{ [this] () { alu_xor(m_de.b.h); } },
		// ab, 4 cycles, XOR E
		{ [this] () { alu_xor(m_de.b.l); } },
		// ac, 4 cycles, XOR H
		{ [this] () { alu_xor(m_hl_index[m_hl_offset].b.h); } },
		// ad, 4 cycles, XOR L
		{ [this] () { alu_xor(m_hl_index[m_hl_offset].b.l); } },
		// ae, 7 cycles, XOR (HL)
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { alu_xor(m_data_bus); }
		},
		// af, 4 cycles, XOR A
		{ [this] () { alu_xor(m_af.b.h); } },

		// b0, 4 cycles, OR B
		{ [this] () { alu_or(m_bc.b.h); } },
		// b1, 4 cycles, OR C
		{ [this] () { alu_or(m_bc.b.l); } },
		// b2, 4 cycles, OR D
		{ [this] () { alu_or(m_de.b.h); } },
		// b3, 4 cycles, OR E
		{ [this] () { alu_or(m_de.b.l); } },
		// b4, 4 cycles, OR H
		{ [this] () { alu_or(m_hl_index[m_hl_offset].b.h); } },
		// b5, 4 cycles, OR L
		{ [this] () { alu_or(m_hl_index[m_hl_offset].b.l); } },
		// b6. 7 cycles, OR (HL)
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { alu_or(m_data_bus); }
		},
		// b7, 4 cycles, OR A
		{ [this] () { alu_or(m_af.b.h); } },

		// b8, 4 cycles, CP B
		{ [this] () { alu_cp(m_bc.b.h); } },
		// b9, 4 cycles, CP C
		{ [this] () { alu_cp(m_bc.b.l); } },
		// ba, 4 cycles, CP D
		{ [this] () { alu_cp(m_de.b.h); } },
		// bb, 4 cycles, CP E
		{ [this] () { alu_cp(m_de.b.l); } },
		// bc, 4 cycles, CP H
		{ [this] () { alu_cp(m_hl_index[m_hl_offset].b.h); } },
		// bd, 4 cycles, CP L
		{ [this] () { alu_cp(m_hl_index[m_hl_offset].b.l); } },
		// be, 7 cycles, CP (HL
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { alu_cp(m_data_bus); }
		},
		// bf, 4 cycles, CP A
		{ [this] () { alu_cp(m_af.b.h); } },

		// c0, 5/11 cycles, RET NZ
		// cycles 6-11 only taken when condition is true
		//  5 T5 AB:1234 DB:--
		//  6 T1 AB:5678 DB:--
		//  7 T2 AB:5678 DB:xx MREQ RD
		//  8 T3 AB:5678 DB:xx MREQ RD
		//  9 T1 AB:5679 DB:--
		// 10 T2 AB:5679 DB:yy MREQ RD
		// 11 T3 AB:5679 DB:yy MREQ RD
		{
			[this] () { if (ret_cond()) { read_s(m_sp.w.l); m_sp.w.l += 1; } },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_wz.b.h = m_data_bus; m_pc.w.l = m_wz.w.l; }
		},
		// c1, 10 cycles, POP BC
		//  5 T1 AB:5678 DB:--
		//  6 T2 AB:5678 DB:xx MREQ RD
		//  7 T3 AB:5678 DB:xx MREQ RD
		//  8 T1 AB:5679 DB:--
		//  9 T2 AB:5679 DB:yy MREQ RD
		// 10 T3 AB:5679 DB:yy MREQ RD
		{
			[this] () { read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_bc.b.l = m_data_bus; read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_bc.b.h = m_data_bus; }
		},
		// c2, 10 cycles, JP NZ,nn
		//  5 T1 AB:1235 DB:--
		//  6 T2 AB:1235 DB:xx MREQ RD
		//  7 T3 AB:1235 DB:xx MREQ RD
		//  8 T1 AB:1236 DB:--
		//  9 T2 AB:1236 DB:yy MREQ RD
		// 10 T3 AB:1236 DB:yy MREQ RD
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; jp_cond(); }
		},
		// c3, 10 cycles, JMP nn
		//  5 T1 AB:1235 DB:--
		//  6 T2 AB:1235 DB:xx MREQ RD
		//  7 T3 AB:1235 DB:xx MREQ RD
		//  8 T1 AB:1236 DB:--
		//  9 T2 AB:1236 DB:yy MREQ RD
		// 10 T3 AB:1236 DB:yy MREQ RD
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; m_pc.w.l = m_wz.w.l; }
		},
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
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; if (call_cond()) { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.h); } },
			[this] () { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.l); },
			[this] () { m_pc.w.l = m_wz.w.l; }
		},
		// c5, 11 cycles, PUSH BC
		//  5 T5 AB:1234 DB:--
		//  6 T1 AB:5677 DB:--
		//  7 T2 AB:5677 DB:cc MREQ
		//  8 T3 AB:5677 DB:cc MREQ WR
		//  9 T1 AB:5676 DB:--
		// 10 T2 AB:5676 DB:bb MREQ
		// 11 T3 AB:5676 DB:bb MREQ WR
		{
			[this] () { m_icount -= 1; m_sp.w.l -= 1; write_s(m_sp.w.l, m_bc.b.h); },
			[this] () { m_sp.w.l -= 1; write_s(m_sp.w.l, m_bc.b.l); }
		},
		// c6, 7 cycles, ADD A,n
		//  5 T1 AB:1235 DB:--
		//  6 T2 AB:1235 DB:nn MREQ RD
		//  7 T3 AB:1235 DB:nn MREQ RD
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { alu_add(m_data_bus); }
		},
		// c7, 11 cycles, RST 0H
		//  5 T5 AB:1234 DB:--
		//  6 T1 AB:5677 DB:--
		//  7 T2 AB:5677 DB:cc MREQ
		//  8 T3 AB:5677 DB:cc MREQ WR
		//  9 T1 AB:5676 DB--
		// 10 T2 AB:5676 DB:pp MREQ
		// 11 T3 AB:5676 DB:pp MREQ WR
		{
			[this] () { m_icount -= 1; m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.h); },
			[this] () { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.l); },
			[this] () { rst(); }
		},

		// c8, 5/11 cycles, RET Z, see c0 for timing
		{
			[this] () { if (ret_cond()) { read_s(m_sp.w.l); m_sp.w.l += 1; } },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_wz.b.h = m_data_bus; m_pc.w.l = m_wz.w.l; }
		},
		/* c9, 10 cycles, RET */
		//  5 T1 AB:5678 DB:--
		//  6 T2 AB:5678 DB:xx MREQ RD
		//  7 T3 AB:5678 DB:xx MREQ RD
		//  8 T1 AB:5679 DB:--
		//  9 T2 AB:5679 DB:yy MREQ RD
		// 10 T3 AB:5679 DB:yy MREQ RD
		{
			[this] () { read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_wz.b.h = m_data_bus; m_pc.w.l = m_wz.w.l; }
		},
		// ca, 10 cycles, JP Z,nn, see c2 for timing
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; jp_cond(); }
		},
		// cb, +4 cycles, CB prefix
		{ },
		// cc, 10/17 cycles, CALL Z,nn, see c4 for timing
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; if (call_cond()) { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.h); } },
			[this] () { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.l); },
			[this] () { m_pc.w.l = m_wz.w.l; }
		},
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
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; m_icount -= 1; m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.h); },
			[this] () { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.l); },
			[this] () { m_pc.w.l = m_wz.w.l; }
		},
		// ce, 7 cycles, ADC A,n, see c6 for timing
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { alu_adc(m_data_bus); }
		},
		// cf, 11 cycles, RST 8H, see c7 for timing
		{
			[this] () { m_icount -= 1; m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.h); },
			[this] () { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.l); },
			[this] () { rst(); }
		},

		// d0, 5/11 cycles, RET NC, see c0 for timing
		{
			[this] () { if (ret_cond()) { read_s(m_sp.w.l); m_sp.w.l += 1; } },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_wz.b.h = m_data_bus; m_pc.w.l = m_wz.w.l; }
		},
		// d1, 10 cycles, POP DE, see c1 for timing
		{
			[this] () { read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_de.b.l = m_data_bus; read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_de.b.h = m_data_bus; }
		},
		// d2, 10 cycles, JP NC,nn, see c2 for timing
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; jp_cond(); }
		},
		// d3, 11 cycles, OUT (n), A
		//  5 T1 AB:1235 DB:--
		//  6 T2 AB:1235 DB:nn MREQ RD
		//  7 T3 AB:1235 DB:nn MREQ RD
		//  8 T1 AB:1235 DB:--
		//  9 T2 AB:aann DB:aa         WR IORQ
		// 10 T3 AB:aann DB:aa         WR IORQ
		// 11 T4 AB:aann DB:aa         WR IORQ
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; a_w(); output_s(m_wz.w.l, m_af.b.h); m_wz.b.h = m_af.b.h; m_wz.w.l++; }
		},
		// d4, 10/17 cycles, CALL NC,nn, see c4 for timing
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; if (call_cond()) { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.h); } },
			[this] () { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.l); },
			[this] () { m_pc.w.l = m_wz.w.l; }
		},
		// d5, 11 cycles, PUSH DE, see c5 for timing
		{
			[this] () { m_icount -= 1; m_sp.w.l -= 1; write_s(m_sp.w.l, m_de.b.h); },
			[this] () { m_sp.w.l -= 1; write_s(m_sp.w.l, m_de.b.l); }
		},
		// d6, 7 cycles, SUB n, see c6 for timing
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { alu_sub(m_data_bus); }
		},
		// d7, 11 cycles, RST 10H, see c7 for timing
		{
			[this] () { m_icount -= 1; m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.h); },
			[this] () { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.l); },
			[this] () { rst(); }
		},

		// d8, 5/11 cycles, RET C, see c0 for timing
		{
			[this] () { if (ret_cond()) { read_s(m_sp.w.l); m_sp.w.l += 1; } },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_wz.b.h = m_data_bus; m_pc.w.l = m_wz.w.l; }
		},
		// d9, 4 cycles, EXX
		{ [this] () { exx(); } },
		// da, 10 cycles, JP C,nn, see c2 for timing
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; jp_cond(); }
		},
		// db, 11 cycles, IN A,(n)
		//  5 T1 AB:1235 DB:--
		//  6 T2 AB:1235 DB:nn MREQ RD
		//  7 T3 AB:1235 DB:nn MREG RD
		//  8 T1 AB:1235 DB:--
		//  9 T2 AB:aann DB:xx      RD IORQ
		// 10 T3 AB:aann DB:xx      RD IORQ
		// 11 T4 AB:aann DB:xx      RD IORQ
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; a_w(); input_s(m_wz.w.l); m_wz.w.l++; },
			[this] () { input_a(); }
		},
		// dc, 10/17 cycles, CALL C,nn, see c4 for timing
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; if (call_cond()) { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.h); } },
			[this] () { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.l); },
			[this] () { m_pc.w.l = m_wz.w.l; }
		},
		// dd, +4 cycles, DD prefix
		{ [] () { } },
		// de, 7 cycles, SBC n, see c6 for timing
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { alu_sbc(m_data_bus); }
		},
		// df, 11 cycles, RST 18H, see c7 for timing
		{
			[this] () { m_icount -= 1; m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.h); },
			[this] () { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.l); },
			[this] () { rst(); }
		},

		// e0, 5/11 cycles, RET PO, see c0 for timing
		{
			[this] () { if (ret_cond()) { read_s(m_sp.w.l); m_sp.w.l += 1; } },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_wz.b.h = m_data_bus; m_pc.w.l = m_wz.w.l; }
		},
		// e1, 10 cycles, POP HL, see c1 for timng
		{
			[this] () { read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_hl_index[m_hl_offset].b.l = m_data_bus; read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_hl_index[m_hl_offset].b.h = m_data_bus; }
		},
		// e2, 10 cycles, JP PO,nn, see c2 for timing
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; jp_cond(); }
		},
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
		{
			[this] () { read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_sp.w.l); },
			[this] () { m_wz.b.h = m_data_bus; m_icount -= 2; write_s(m_hl_index[m_hl_offset].b.h); },
			[this] () { m_sp.w.l -= 1; write_s(m_sp.w.l, m_hl_index[m_hl_offset].b.l); },
			[this] () { m_icount -= 2; m_hl_index[m_hl_offset].w.l = m_wz.w.l; }
		},
		// e4, 10/17 cycles, CALL PO,nn, see c4 for timing
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; if (call_cond()) { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.h); } },
			[this] () { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.l); },
			[this] () { m_pc.w.l = m_wz.w.l; }
		},
		// e5, 11 cycles, PUSH HL, see c5 for timing
		{
			[this] () { m_icount -= 1; m_sp.w.l -= 1; write_s(m_sp.w.l, m_hl_index[m_hl_offset].b.h); },
			[this] () { m_sp.w.l -= 1; write_s(m_sp.w.l, m_hl_index[m_hl_offset].b.l); }
		},
		// e6, 7 cycles, AND n, see c6 for timing
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { alu_and(m_data_bus); }
		},
		// e7, 11 cycles, RST 20H, see c7 for timing
		{
			[this] () { m_icount -= 1; m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.h); },
			[this] () { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.l); },
			[this] () { rst(); }
		},

		// e8, 5/11 cycles, RET PE, see c0 for timing
		{
			[this] () { if (ret_cond()) { read_s(m_sp.w.l); m_sp.w.l += 1; } },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_wz.b.h = m_data_bus; m_pc.w.l = m_wz.w.l; }
		},
		// e9, 4 cycles, JP (HL)
		{ [this] () { m_pc.w.l = m_hl_index[m_hl_offset].w.l; } },
		// ea, 10 cycles, JP PE,nn, see c2 for timing
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; jp_cond(); }
		},
		// eb, 4 cycles, EX DE,HL
		{ [this] () { ex_de_hl(); } },
		// ec, 10/17 cycles, CALL PE,nn, see c4 for timing
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; if (call_cond()) { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.h); } },
			[this] () { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.l); },
			[this] () { m_pc.w.l = m_wz.w.l; }
		},
		// ed, +4 cycles, ED prefix
		{ [] () { } },
		// ee, 7 cycles, XOR n, see c6 for timing
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { alu_xor(m_data_bus); }
		},
		// ef, 11 cycles, RST 28H, see c7 for timing
		{
			[this] () { m_icount -= 1; m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.h); },
			[this] () { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.l); },
			[this] () { rst(); }
		},

		// f0, 5/11 cycles, RET P, see c0 for timing
		{
			[this] () { if (ret_cond()) { read_s(m_sp.w.l); m_sp.w.l += 1; } },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_wz.b.h = m_data_bus; m_pc.w.l = m_wz.w.l; }
		},
		// f1, 10 cycles, POP AF, see c1 for timing
		{
			[this] () { read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_af.b.l = m_data_bus; read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_af.b.h = m_data_bus; }
		},
		// f2, 10 cycles, JP P,nn, see c2 for timing
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; jp_cond(); }
		},
		// f3, 4 cycles, DI
		{ [this] () { di(); } },
		// f4, 10/17 cycles, CALL P,nn, see c4 for timing
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; if (call_cond()) { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.h); } },
			[this] () { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.l); },
			[this] () { m_pc.w.l = m_wz.w.l; }
		},
		// f5, 11 cycles, PUSH AF, see c5 for timing
		{
			[this] () { m_icount -= 1; m_sp.w.l -= 1; write_s(m_sp.w.l, m_af.b.h); },
			[this] () { m_sp.w.l -= 1; write_s(m_sp.w.l, m_af.b.l); }
		},
		// f6, 7 cycles, OR n, see c6 for timing
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { alu_or(m_data_bus); }
		},
		// f7, 11 cycles, RST 30H, see c7 for timing
		{
			[this] () { m_icount -= 1; m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.h); },
			[this] () { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.l); },
			[this] () { rst(); }
		},

		// f8, 5/11 cycles, RET M, see c0 for timing
		{
			[this] () { if (ret_cond()) { read_s(m_sp.w.l); m_sp.w.l += 1; } },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_wz.b.h = m_data_bus; m_pc.w.l = m_wz.w.l; }
		},
		// f9, 6 cycles, LD SP,HL
		{ [this] () { m_sp.w.l = m_hl_index[m_hl_offset].w.l; m_icount -= 2; } },
		// fa, 10 cycles, JP M,nn, see c2 for timing
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; jp_cond(); }
		},
		// fb, 4 cycles, EI
		{ [this] () { ei(); } },
		// fc, 10/17 cycles, CALL M,nn, see c4 for timing
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; if (call_cond()) { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.h); } },
			[this] () { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.l); },
			[this] () { m_pc.w.l = m_wz.w.l; }
		},
		// fd, +4 cycles, FD prefix
		{ [] () { } },
		// fe, 7 cycles, CP n, see c6 for timing
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { alu_cp(m_data_bus); }
		},
		// ff, 11 cycles, RST 38H, see c7 for timing
		{
			[this] () { m_icount -= 1; m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.h); },
			[this] () { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.l); },
			[this] () { rst(); }
		},

		/*****************************************************/
		/* CB prefixed instructions                          */
		/*****************************************************/

		// cb 00, 8 cycles, RLC B
		{ [this] () { m_bc.b.h = alu_rlc(m_bc.b.h); } },
		// cb 01, 8 cycles, RLC C
		{ [this] () { m_bc.b.l = alu_rlc(m_bc.b.l); } },
		// cb 02, 8 cycles, RLC D
		{ [this] () { m_de.b.h = alu_rlc(m_de.b.h); } },
		// cb 03, 8 cycles, RLC E
		{ [this] () { m_de.b.l = alu_rlc(m_de.b.l); } },
		// cb 04, 8 cycles, RLC H
		{ [this] () { m_hl_index[m_hl_offset].b.h = alu_rlc(m_hl_index[m_hl_offset].b.h); } },
		// cb 05, 8 cycles, RLC L
		{ [this] () { m_hl_index[m_hl_offset].b.l = alu_rlc(m_hl_index[m_hl_offset].b.l); } },
		// cb 06, 15 cycles, RLC (HL)
		//  9 T1 AB:hhll DB:--
		// 10 T2 AB:hhll DB:xx MREQ RD
		// 11 T3 AB:hhll DB:xx MREQ RD
		// 12 T4 AB:hhll DB:--
		// 13 T1 AB:hhll DB:--
		// 14 T2 AB:hhll DB:yy MREQ
		// 15 T3 AB:hhll DB:yy MREQ WR
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { m_icount -= 2; write_s(alu_rlc(m_data_bus)); }
		},
		// cb 07, 8 cycles, RLC A
		{ [this] () { m_af.b.h = alu_rlc(m_af.b.h); } },

		// cb 08, 8 cycles, RRC B
		{ [this] () { m_bc.b.h = alu_rrc(m_bc.b.h); } },
		// cb 09, 8 cycles, RRC C
		{ [this] () { m_bc.b.l = alu_rrc(m_bc.b.l); } },
		// cb 0a, 8 cycles, RRC D
		{ [this] () { m_de.b.h = alu_rrc(m_de.b.h); } },
		// cb 0b, 8 cycles, RRC E
		{ [this] () { m_de.b.l = alu_rrc(m_de.b.l); } },
		// cb 0c, 8 cycles, RRC H
		{ [this] () { m_hl_index[m_hl_offset].b.h = alu_rrc(m_hl_index[m_hl_offset].b.h); } },
		// cb 0d, 8 cycles, RRC L
		{ [this] () { m_hl_index[m_hl_offset].b.l = alu_rrc(m_hl_index[m_hl_offset].b.l); } },
		// cb 0e, 15 cycles, RRC (HL), see cb 06 for timing
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { m_icount -= 2; write_s(alu_rrc(m_data_bus)); }
		},
		// cb 0f, 8 cycles, RRC A
		{ [this] () { m_af.b.h = alu_rrc(m_af.b.h); } },

		// cb 10, 8 cycles, RL B
		{ [this] () { m_bc.b.h = alu_rl(m_bc.b.h); } },
		// cb 11, 8 cycles, RL C
		{ [this] () { m_bc.b.l = alu_rl(m_bc.b.l); } },
		// cb 12, 8 cycles, RL D
		{ [this] () { m_de.b.h = alu_rl(m_de.b.h); } },
		// cb 13, 8 cycles, RL E
		{ [this] () { m_de.b.l = alu_rl(m_de.b.l); } },
		// cb 14, 8 cycles, RL H
		{ [this] () { m_hl_index[m_hl_offset].b.h = alu_rl(m_hl_index[m_hl_offset].b.h); } },
		// cb 15, 8 cycles, RL L
		{ [this] () { m_hl_index[m_hl_offset].b.l = alu_rl(m_hl_index[m_hl_offset].b.l); } },
		// cb 16, 15 cycles, RL (HL), see cb 06 for timing
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { m_icount -= 2; write_s(alu_rl(m_data_bus)); }
		},
		// cb 17, 8 cycles, RL A
		{ [this] () { m_af.b.h = alu_rl(m_af.b.h); } },

		// cb 18, 8 cycles, RR B
		{ [this] () { m_bc.b.h = alu_rr(m_bc.b.h); } },
		// cb 19, 8 cycles, RR C
		{ [this] () { m_bc.b.l = alu_rr(m_bc.b.l); } },
		// cb 1a, 8 cycles, RR D
		{ [this] () { m_de.b.h = alu_rr(m_de.b.h); } },
		// cb 1b, 8 cycles, RR E
		{ [this] () { m_de.b.l = alu_rr(m_de.b.l); } },
		// cb 1c, 8 cycles, RR H
		{ [this] () { m_hl_index[m_hl_offset].b.h = alu_rr(m_hl_index[m_hl_offset].b.h); } },
		// cb 1d, 8 cycles, RR L
		{ [this] () { m_hl_index[m_hl_offset].b.l = alu_rr(m_hl_index[m_hl_offset].b.l); } },
		// cb 1e, 15 cycles, RR (HL), see cb 06 for timing
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { m_icount -= 2; write_s(alu_rr(m_data_bus)); }
		},
		// cb 1f, 8 cycles, RR A
		{ [this] () { m_af.b.h = alu_rr(m_af.b.h); } },

		// cb 20, 8 cycles, SLA B
		{ [this] () { m_bc.b.h = alu_sla(m_bc.b.h); } },
		// cb 21, 8 cycles, SLA C
		{ [this] () { m_bc.b.l = alu_sla(m_bc.b.l); } },
		// cb 22, 8 cycles, SLA D
		{ [this] () { m_de.b.h = alu_sla(m_de.b.h); } },
		// cb 23, 8 cycles, SLA E
		{ [this] () { m_de.b.l = alu_sla(m_de.b.l); } },
		// cb 24, 8 cycles, SLA H
		{ [this] () { m_hl_index[m_hl_offset].b.h = alu_sla(m_hl_index[m_hl_offset].b.h); } },
		// cb 25, 8 cycles, SLA L
		{ [this] () { m_hl_index[m_hl_offset].b.l = alu_sla(m_hl_index[m_hl_offset].b.l); } },
		// cb 26, 15 cycles, SLA (HL), see cb 06 for timing
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { m_icount -= 2; write_s(alu_sla(m_data_bus)); }
		},
		// cb 27, 8 cycles, SLA A
		{ [this] () { m_af.b.h = alu_sla(m_af.b.h); } },

		// cb 28, 8 cycles, SRA B
		{ [this] () { m_bc.b.h = alu_sra(m_bc.b.h); } },
		// cb 29, 8 cycles, SRA C
		{ [this] () { m_bc.b.l = alu_sra(m_bc.b.l); } },
		// cb 2a, 8 cycles, SRA D
		{ [this] () { m_de.b.h = alu_sra(m_de.b.h); } },
		// cb 2b, 8 cycles, SRA E
		{ [this] () { m_de.b.l = alu_sra(m_de.b.l); } },
		// cb 2c, 8 cycles, SRA H
		{ [this] () { m_hl_index[m_hl_offset].b.h = alu_sra(m_hl_index[m_hl_offset].b.h); } },
		// cb 2d, 8 cycles, SRA L
		{ [this] () { m_hl_index[m_hl_offset].b.l = alu_sra(m_hl_index[m_hl_offset].b.l); } },
		// cb 2e, 15 cycles, SRA (HL), see cb 06 for timing
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { m_icount -= 2; write_s(alu_sra(m_data_bus)); }
		},
		// cb 2f, 8 cycles, SRA A
		{ [this] () { m_af.b.h = alu_sra(m_af.b.h); } },

		// cb 30, 8 cycles, SLL B
		{ [this] () { m_bc.b.h = alu_sll(m_bc.b.h); } },
		// cb 31, 8 cycles, SLL C
		{ [this] () { m_bc.b.l = alu_sll(m_bc.b.l); } },
		// cb 32, 8 cycles, SLL D
		{ [this] () { m_de.b.h = alu_sll(m_de.b.h); } },
		// cb 33, 8 cycles, SLL E
		{ [this] () { m_de.b.l = alu_sll(m_de.b.l); } },
		// cb 34, 8 cycles, SLL H
		{ [this] () { m_hl_index[m_hl_offset].b.h = alu_sll(m_hl_index[m_hl_offset].b.h); } },
		// cb 35, 8 cycles, SLL L
		{ [this] () { m_hl_index[m_hl_offset].b.l = alu_sll(m_hl_index[m_hl_offset].b.l); } },
		// cb 36, 15 cycles, SLL (HL), see cb 06 for timing
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { m_icount -= 2; write_s(alu_sll(m_data_bus)); }
		},
		// cb 37, 8 cycles, SLL A
		{ [this] () { m_af.b.h = alu_sll(m_af.b.h); } },

		// cb 38, 8 cycles, SRL B
		{ [this] () { m_bc.b.h = alu_srl(m_bc.b.h); } },
		// cb 39, 8 cycles, SRL C
		{ [this] () { m_bc.b.l = alu_srl(m_bc.b.l); } },
		// cb 3a, 8 cycles, SRL D
		{ [this] () { m_de.b.h = alu_srl(m_de.b.h); } },
		// cb 3b, 8 cycles, SRL E
		{ [this] () { m_de.b.l = alu_srl(m_de.b.l); } },
		// cb 3c, 8 cycles, SRL H
		{ [this] () { m_hl_index[m_hl_offset].b.h = alu_srl(m_hl_index[m_hl_offset].b.h); } },
		// cb 3d, 8 cycles, SRL L
		{ [this] () { m_hl_index[m_hl_offset].b.l = alu_srl(m_hl_index[m_hl_offset].b.l); } },
		// cb 3e, 15 cycles, SRL (HL), see cb 06 for timing
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { m_icount -= 2; write_s(alu_srl(m_data_bus)); }
		},
		// cb 3f, 8 cycles, SRL A
		{ [this] () { m_af.b.h = alu_srl(m_af.b.h); } },

		// cb 40, 8 cycles, BIT 0,B
		{ [this] () { alu_bit(m_bc.b.h); } },
		// cb 41, 8 cycles, BIT 0,C
		{ [this] () { alu_bit(m_bc.b.l); } },
		// cb 42, 8 cycles, BIT 0,D
		{ [this] () { alu_bit(m_de.b.h); } },
		// cb 43, 8 cycles, BIT 0,E
		{ [this] () { alu_bit(m_de.b.l); } },
		// cb 44, 8 cycles, BIT 0,H
		{ [this] () { alu_bit(m_hl_index[m_hl_offset].b.h); } },
		// cb 45, 8 cycles, BIT 0,L
		{ [this] () { alu_bit(m_hl_index[m_hl_offset].b.l); } },
		// cb 46, 12 cycles, BIT 0,(HL)
		//  9 T1 AB:hhll DB:--
		// 10 T2 AB:hhll DB:xx MREQ RD
		// 11 T3 AB:hhll DB:xx MREQ RD
		// 12 T4 AB:hhll DB:--
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// cb 47, 8 cycles, BIT 0,A
		{ [this] () { alu_bit(m_af.b.h); } },

		// cb 48, 8 cycles, BIT 1,B
		{ [this] () { alu_bit(m_bc.b.h); } },
		// cb 49, 8 cycles, BIT 1,C
		{ [this] () { alu_bit(m_bc.b.l); } },
		// cb 4a, 8 cycles, BIT 1,D
		{ [this] () { alu_bit(m_de.b.h); } },
		// cb 4b, 8 cycles, BIT 1,E
		{ [this] () { alu_bit(m_de.b.l); } },
		// cb 4c, 8 cycles, BIT 1,H
		{ [this] () { alu_bit(m_hl_index[m_hl_offset].b.h); } },
		// cb 4d, 8 cycles, BIT 1,L
		{ [this] () { alu_bit(m_hl_index[m_hl_offset].b.l); } },
		// cb 4e, 12 cycles, BIT 1,(HL), see cb 46 for timing
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// cb 4f, 8 cycles, BIT 1,A
		{ [this] () { alu_bit(m_af.b.h); } },

		// cb 50, 8 cycles, BIT 2,B
		{ [this] () { alu_bit(m_bc.b.h); } },
		// cb 51, 8 cycles, BIT 2,C
		{ [this] () { alu_bit(m_bc.b.l); } },
		// cb 52, 8 cycles, BIT 2,D
		{ [this] () { alu_bit(m_de.b.h); } },
		// cb 53, 8 cycles, BIT 2,E
		{ [this] () { alu_bit(m_de.b.l); } },
		// cb 54, 8 cycles, BIT 2,H
		{ [this] () { alu_bit(m_hl_index[m_hl_offset].b.h); } },
		// cb 55, 8 cycles, BIT 2,L
		{ [this] () { alu_bit(m_hl_index[m_hl_offset].b.l); } },
		// cb 56, 12 cycles, BIT 2,(HL), see cb 46 for timing
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// cb 57, 8 cycles, BIT 2,A
		{ [this] () { alu_bit(m_af.b.h); } },

		// cb 58, 8 cycles, BIT 3,B
		{ [this] () { alu_bit(m_bc.b.h); } },
		// cb 59, 8 cycles, BIT 3,C
		{ [this] () { alu_bit(m_bc.b.l); } },
		// cb 5a, 8 cycles, BIT 3,D
		{ [this] () { alu_bit(m_de.b.h); } },
		// cb 5b, 8 cycles, BIT 3,E
		{ [this] () { alu_bit(m_de.b.l); } },
		// cb 5c, 8 cycles, BIT 3,H
		{ [this] () { alu_bit(m_hl_index[m_hl_offset].b.h); } },
		// cb 5d, 8 cycles, BIT 3,L
		{ [this] () { alu_bit(m_hl_index[m_hl_offset].b.l); } },
		// cb 5e, 12 cycles, BIT 3,(HL), see cb 46 for timing
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// cb 5f, 8 cycles, BIT 3,A
		{ [this] () { alu_bit(m_af.b.h); } },

		// cb 60, 8 cycles, BIT 4,B
		{ [this] () { alu_bit(m_bc.b.h); } },
		// cb 61, 8 cycles, BIT 4,C
		{ [this] () { alu_bit(m_bc.b.l); } },
		// cb 62, 8 cycles, BIT 4,D
		{ [this] () { alu_bit(m_de.b.h); } },
		// cb 63, 8 cycles, BIT 4,E
		{ [this] () { alu_bit(m_de.b.l); } },
		// cb 64, 8 cycles, BIT 4,H
		{ [this] () { alu_bit(m_hl_index[m_hl_offset].b.h); } },
		// cb 65, 8 cycles, BIT 4,L
		{ [this] () { alu_bit(m_hl_index[m_hl_offset].b.l); } },
		// cb 66, 12 cycles, BIT 4,(HL), see cb 46 for timing
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// cb 67, 8 cycles, BIT 4,A
		{ [this] () { alu_bit(m_af.b.h); } },

		// cb 68, 8 cycles, BIT 5,B
		{ [this] () { alu_bit(m_bc.b.h); } },
		// cb 69, 8 cycles, BIT 5,C
		{ [this] () { alu_bit(m_bc.b.l); } },
		// cb 6a, 8 cycles, BIT 5,D
		{ [this] () { alu_bit(m_de.b.h); } },
		// cb 6b, 8 cycles, BIT 5,E
		{ [this] () { alu_bit(m_de.b.l); } },
		// cb 6c, 8 cycles, BIT 5,H
		{ [this] () { alu_bit(m_hl_index[m_hl_offset].b.h); } },
		// cb 6d, 8 cycles, BIT 5,L
		{ [this] () { alu_bit(m_hl_index[m_hl_offset].b.l); } },
		// cb 6e, 12 cycles, BIT 5,(HL), see cb 46 for timing
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// cb 6f, 8 cycles, BIT 5,A
		{ [this] () { alu_bit(m_af.b.h); } },

		// cb 70, 8 cycles, BIT 6,B
		{ [this] () { alu_bit(m_bc.b.h); } },
		// cb 71, 8 cycles, BIT 6,C
		{ [this] () { alu_bit(m_bc.b.l); } },
		// cb 72, 8 cycles, BIT 6,D
		{ [this] () { alu_bit(m_de.b.h); } },
		// cb 73, 8 cycles, BIT 6,E
		{ [this] () { alu_bit(m_de.b.l); } },
		// cb 74, 8 cycles, BIT 6,H
		{ [this] () { alu_bit(m_hl_index[m_hl_offset].b.h); } },
		// cb 75, 8 cycles, BIT 6,L
		{ [this] () { alu_bit(m_hl_index[m_hl_offset].b.l); } },
		// cb 76, 12 cycles, BIT 6,(HL), see cb 46 for timing
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// cb 77, 8 cycles, BIT 6,A
		{ [this] () { alu_bit(m_af.b.h); } },

		// cb 78, 8 cycles, BIT 7,B
		{ [this] () { alu_bit(m_bc.b.h); } },
		// cb 79, 8 cycles, BIT 7,C
		{ [this] () { alu_bit(m_bc.b.l); } },
		// cb 7a, 8 cycles, BIT 7,D
		{ [this] () { alu_bit(m_de.b.h); } },
		// cb 7b, 8 cycles, BIT 7,E
		{ [this] () { alu_bit(m_de.b.l); } },
		// cb 7c, 8 cycles, BIT 7,H
		{ [this] () { alu_bit(m_hl_index[m_hl_offset].b.h); } },
		// cb 7d, 8 cycles, BIT 7,L
		{ [this] () { alu_bit(m_hl_index[m_hl_offset].b.l); } },
		// cb 7e, 12 cycles, BIT 7,(HL), see cb 46 for timing
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// cb 7f, 8 cycles, BIT 7,A
		{ [this] () { alu_bit(m_af.b.h); } },

		// cb 80, 8 cycles, RES 0,B
		{ [this] () { m_bc.b.h = alu_res(m_bc.b.h); } },
		// cb 81, 8 cycles, RES 0,C
		{ [this] () { m_bc.b.l = alu_res(m_bc.b.l); } },
		// cb 82, 8 cycles, RES 0,D
		{ [this] () { m_de.b.h = alu_res(m_de.b.h); } },
		// cb 83, 8 cycles, RES 0,E
		{ [this] () { m_de.b.l = alu_res(m_de.b.l); } },
		// cb 84, 8 cycles, RES 0,H
		{ [this] () { m_hl_index[m_hl_offset].b.h = alu_res(m_hl_index[m_hl_offset].b.h); } },
		// cb 85, 8 cycles, RES 0,L
		{ [this] () { m_hl_index[m_hl_offset].b.l = alu_res(m_hl_index[m_hl_offset].b.l); } },
		// cb 86, 15 cycles, RES 0,(HL)
		//  9 T1 AB:hhll DB:--
		// 10 T2 AB:hhll DB:xx MREQ RD
		// 11 T3 AB:hhll DB:xx MREQ RD
		// 12 T4 AB:hhll DB:--
		// 13 T1 AB:hhll DB:--
		// 14 T2 AB:hhll DB:yy MREG
		// 15 T3 AB:hhll DB:yy MREQ WR
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { m_icount -= 2; write_s(alu_res(m_data_bus)); }
		},
		// cb 87, 8 cycles, RES 0,A
		{ [this] () { m_af.b.h = alu_res(m_af.b.h); } },

		// cb 88, 8 cycles, RES 1,B
		{ [this] () { m_bc.b.h = alu_res(m_bc.b.h); } },
		// cb 89, 8 cycles, RES 1,C
		{ [this] () { m_bc.b.l = alu_res(m_bc.b.l); } },
		// cb 8a, 8 cycles, RES 1,D
		{ [this] () { m_de.b.h = alu_res(m_de.b.h); } },
		// cb 8b, 8 cycles, RES 1,E
		{ [this] () { m_de.b.l = alu_res(m_de.b.l); } },
		// cb 8c, 8 cycles, RES 1,H
		{ [this] () { m_hl_index[m_hl_offset].b.h = alu_res(m_hl_index[m_hl_offset].b.h); } },
		// cb 8d, 8 cycles, RES 1,L
		{ [this] () { m_hl_index[m_hl_offset].b.l = alu_res(m_hl_index[m_hl_offset].b.l); } },
		// cb 8e, 15 cycles, RES 1,(HL), see cb 86 for timing
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { m_icount -= 2; write_s(alu_res(m_data_bus)); }
		},
		// cb 8f, 8 cycles, RES 1,A
		{ [this] () { m_af.b.h = alu_res(m_af.b.h); } },

		// cb 90, 8 cycles, RES 2,B
		{ [this] () { m_bc.b.h = alu_res(m_bc.b.h); } },
		// cb 91, 8 cycles, RES 2,C
		{ [this] () { m_bc.b.l = alu_res(m_bc.b.l); } },
		// cb 92, 8 cycles, RES 2,D
		{ [this] () { m_de.b.h = alu_res(m_de.b.h); } },
		// cb 93, 8 cycles, RES 2,E
		{ [this] () { m_de.b.l = alu_res(m_de.b.l); } },
		// cb 94, 8 cycles, RES 2,H
		{ [this] () { m_hl_index[m_hl_offset].b.h = alu_res(m_hl_index[m_hl_offset].b.h); } },
		// cb 95, 8 cycles, RES 2,L
		{ [this] () { m_hl_index[m_hl_offset].b.l = alu_res(m_hl_index[m_hl_offset].b.l); } },
		// cb 96, 15 cycles, RES 2,(HL), see cb 86 for timing
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { m_icount -= 2; write_s(alu_res(m_data_bus)); }
		},
		// cb 97, 8 cycles, RES 2,A
		{ [this] () { m_af.b.h = alu_res(m_af.b.h); } },

		// cb 98, 8 cycles, RES 3,B
		{ [this] () { m_bc.b.h = alu_res(m_bc.b.h); } },
		// cb 99, 8 cycles, RES 3,C
		{ [this] () { m_bc.b.l = alu_res(m_bc.b.l); } },
		// cb 9a, 8 cycles, RES 3,D
		{ [this] () { m_de.b.h = alu_res(m_de.b.h); } },
		// cb 9b, 8 cycles, RES 3,E
		{ [this] () { m_de.b.l = alu_res(m_de.b.l); } },
		// cb 9c, 8 cycles, RES 3,H
		{ [this] () { m_hl_index[m_hl_offset].b.h = alu_res(m_hl_index[m_hl_offset].b.h); } },
		// cb 9d, 8 cycles, RES 3,L
		{ [this] () { m_hl_index[m_hl_offset].b.l = alu_res(m_hl_index[m_hl_offset].b.l); } },
		// cb 9e, 15 cycles, RES 3,(HL), see cb 86 for timing
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { m_icount -= 2; write_s(alu_res(m_data_bus)); }
		},
		// cb 9f, 8 cycles, RES 3,A
		{ [this] () { m_af.b.h = alu_res(m_af.b.h); } },

		// cb a0, 8 cycles, RES 4,B
		{ [this] () { m_bc.b.h = alu_res(m_bc.b.h); } },
		// cb a1, 8 cycles, RES 4,C
		{ [this] () { m_bc.b.l = alu_res(m_bc.b.l); } },
		// cb a2, 8 cycles, RES 4,D
		{ [this] () { m_de.b.h = alu_res(m_de.b.h); } },
		// cb a3, 8 cycles, RES 4,E
		{ [this] () { m_de.b.l = alu_res(m_de.b.l); } },
		// cb a4, 8 cycles, RES 4,H
		{ [this] () { m_hl_index[m_hl_offset].b.h = alu_res(m_hl_index[m_hl_offset].b.h); } },
		// cb a5, 8 cycles, RES 4,L
		{ [this] () { m_hl_index[m_hl_offset].b.l = alu_res(m_hl_index[m_hl_offset].b.l); } },
		// cb a6, 15 cycles, RES 4,(HL), see cb 86 for timing
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { m_icount -= 2; write_s(alu_res(m_data_bus)); }
		},
		// cb a7, 8 cycles, RES 4,A
		{ [this] () { m_af.b.h = alu_res(m_af.b.h); } },

		// cb a8, 8 cycles, RES 5,B
		{ [this] () { m_bc.b.h = alu_res(m_bc.b.h); } },
		// cb a9, 8 cycles, RES 5,C
		{ [this] () { m_bc.b.l = alu_res(m_bc.b.l); } },
		// cb aa, 8 cycles, RES 5,D
		{ [this] () { m_de.b.h = alu_res(m_de.b.h); } },
		// cb ab, 8 cycles, RES 5,E
		{ [this] () { m_de.b.l = alu_res(m_de.b.l); } },
		// cb ac, 8 cycles, RES 5,H
		{ [this] () { m_hl_index[m_hl_offset].b.h = alu_res(m_hl_index[m_hl_offset].b.h); } },
		// cb ad, 8 cycles, RES 5,L
		{ [this] () { m_hl_index[m_hl_offset].b.l = alu_res(m_hl_index[m_hl_offset].b.l); } },
		// cb ae, 15 cycles, RES 5,(HL), for cb 86 for timing
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { m_icount -= 2; write_s(alu_res(m_data_bus)); }
		},
		// cb af, 8 cycles, RES 5,A
		{ [this] () { m_af.b.h = alu_res(m_af.b.h); } },

		// cb b0, 8 cycles, RES 6,B
		{ [this] () { m_bc.b.h = alu_res(m_bc.b.h); } },
		// cb b1, 8 cycles, RES 6,C
		{ [this] () { m_bc.b.l = alu_res(m_bc.b.l); } },
		// cb b2, 8 cycles, RES 6,D
		{ [this] () { m_de.b.h = alu_res(m_de.b.h); } },
		// cb b3, 8 cycles, RES 6,E
		{ [this] () { m_de.b.l = alu_res(m_de.b.l); } },
		// cb b4, 8 cycles, RES 6,H
		{ [this] () { m_hl_index[m_hl_offset].b.h = alu_res(m_hl_index[m_hl_offset].b.h); } },
		// cb b5, 8 cycles, RES 6,L
		{ [this] () { m_hl_index[m_hl_offset].b.l = alu_res(m_hl_index[m_hl_offset].b.l); } },
		// cb b6, 15 cycles, RES 6,(HL), see cb 86 for timing
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { m_icount -= 2; write_s(alu_res(m_data_bus)); }
		},
		// cb b7, 8 cycles, RES 6,A
		{ [this] () { m_af.b.h = alu_res(m_af.b.h); } },

		// cb b8, 8 cycles, RES 7,B
		{ [this] () { m_bc.b.h = alu_res(m_bc.b.h); } },
		// cb b9, 8 cycles, RES 7,C
		{ [this] () { m_bc.b.l = alu_res(m_bc.b.l); } },
		// cb ba, 8 cycles, RES 7,D
		{ [this] () { m_de.b.h = alu_res(m_de.b.h); } },
		// cb bb, 8 cycles, RES 7,E
		{ [this] () { m_de.b.l = alu_res(m_de.b.l); } },
		// cb bc, 8 cycles, RES 7,H
		{ [this] () { m_hl_index[m_hl_offset].b.h = alu_res(m_hl_index[m_hl_offset].b.h); } },
		// cb bd, 8 cycles, RES 7,L
		{ [this] () { m_hl_index[m_hl_offset].b.l = alu_res(m_hl_index[m_hl_offset].b.l); } },
		// cb be, 15 cycles, RES 7,(HL), see cb 86 for timing
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { m_icount -= 2; write_s(alu_res(m_data_bus)); }
		},
		// cb bf, 8 cycles, RES 7,A
		{ [this] () { m_af.b.h = alu_res(m_af.b.h); } },

		// cb c0, 8 cycles, SET 0,B
		{ [this] () { m_bc.b.h = alu_set(m_bc.b.h); } },
		// cb c1, 8 cycles, SET 0,C
		{ [this] () { m_bc.b.l = alu_set(m_bc.b.l); } },
		// cb c2, 8 cycles, SET 0,D
		{ [this] () { m_de.b.h = alu_set(m_de.b.h); } },
		// cb c3, 8 cycles, SET 0,E
		{ [this] () { m_de.b.l = alu_set(m_de.b.l); } },
		// cb c4, 8 cycles, SET 0,H
		{ [this] () { m_hl_index[m_hl_offset].b.h = alu_set(m_hl_index[m_hl_offset].b.h); } },
		// cb c5, 8 cycles, SET 0,L
		{ [this] () { m_hl_index[m_hl_offset].b.l = alu_set(m_hl_index[m_hl_offset].b.l); } },
		// cb c6, 15 cycles, SET 0,(HL), see cb 86 for timing
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { m_icount -= 2; write_s(alu_set(m_data_bus)); }
		},
		// cb c7, 8 cycles, SET 0,A
		{ [this] () { m_af.b.h = alu_set(m_af.b.h); } },

		// cb c8, 8 cycles, SET 1,B
		{ [this] () { m_bc.b.h = alu_set(m_bc.b.h); } },
		// cb c9, 8 cycles, SET 1,C
		{ [this] () { m_bc.b.l = alu_set(m_bc.b.l); } },
		// cb ca, 8 cycles, SET 1,D
		{ [this] () { m_de.b.h = alu_set(m_de.b.h); } },
		// cb cb, 8 cycles, SET 1,E
		{ [this] () { m_de.b.l = alu_set(m_de.b.l); } },
		// cb cc, 8 cycles, SET 1,H
		{ [this] () { m_hl_index[m_hl_offset].b.h = alu_set(m_hl_index[m_hl_offset].b.h); } },
		// cb cd, 8 cycles, SET 1,L
		{ [this] () { m_hl_index[m_hl_offset].b.l = alu_set(m_hl_index[m_hl_offset].b.l); } },
		// cb ce, 15 cycles, SET 1,(HL), see cb 86 for timing
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { m_icount -= 2; write_s(alu_set(m_data_bus)); }
		},
		// cb cf, 8 cycles, SET 1,A
		{ [this] () { m_af.b.h = alu_set(m_af.b.h); } },

		// cb d0, 8 cycles, SET 2,B
		{ [this] () { m_bc.b.h = alu_set(m_bc.b.h); } },
		// cb d1, 8 cycles, SET 2,C
		{ [this] () { m_bc.b.l = alu_set(m_bc.b.l); } },
		// cb d2, 8 cycles, SET 2,D
		{ [this] () { m_de.b.h = alu_set(m_de.b.h); } },
		// cb d3, 8 cycles, SET 2,E
		{ [this] () { m_de.b.l = alu_set(m_de.b.l); } },
		// cb d4, 8 cycles, SET 2,H
		{ [this] () { m_hl_index[m_hl_offset].b.h = alu_set(m_hl_index[m_hl_offset].b.h); } },
		// cb d5, 8 cycles, SET 2,L
		{ [this] () { m_hl_index[m_hl_offset].b.l = alu_set(m_hl_index[m_hl_offset].b.l); } },
		// cb d6, 15 cycles, SET 2,(HL), see cb 86 for timing
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { m_icount -= 2; write_s(alu_set(m_data_bus)); }
		},
		// cb d7, 8 cycles, SET 2,A
		{ [this] () { m_af.b.h = alu_set(m_af.b.h); } },

		// cb d8, 8 cycles, SET 3,B
		{ [this] () { m_bc.b.h = alu_set(m_bc.b.h); } },
		// cb d9, 8 cycles, SET 3,C
		{ [this] () { m_bc.b.l = alu_set(m_bc.b.l); } },
		// cb da, 8 cycles, SET 3,D
		{ [this] () { m_de.b.h = alu_set(m_de.b.h); } },
		// cb db, 8 cycles, SET 3,E
		{ [this] () { m_de.b.l = alu_set(m_de.b.l); } },
		// cb dc, 8 cycles, SET 3,H
		{ [this] () { m_hl_index[m_hl_offset].b.h = alu_set(m_hl_index[m_hl_offset].b.h); } },
		// cb dd, 8 cycles, SET 3,L
		{ [this] () { m_hl_index[m_hl_offset].b.l = alu_set(m_hl_index[m_hl_offset].b.l); } },
		// cb de, 15 cycles, SET 3,(HL), see cb 86 for timing
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { m_icount -= 2; write_s(alu_set(m_data_bus)); }
		},
		// cb df, 8 cycles, SET 3,A
		{ [this] () { m_af.b.h = alu_set(m_af.b.h); } },

		// cb e0, 8 cycles, SET 4,B
		{ [this] () { m_bc.b.h = alu_set(m_bc.b.h); } },
		// cb e1, 8 cycles, SET 4,C
		{ [this] () { m_bc.b.l = alu_set(m_bc.b.l); } },
		// cb e2, 8 cycles, SET 4,D
		{ [this] () { m_de.b.h = alu_set(m_de.b.h); } },
		// cb e3, 8 cycles, SET 4,E
		{ [this] () { m_de.b.l = alu_set(m_de.b.l); } },
		// cb e4, 8 cycles, SET 4,H
		{ [this] () { m_hl_index[m_hl_offset].b.h = alu_set(m_hl_index[m_hl_offset].b.h); } },
		// cb e5, 8 cycles, SET 4,L
		{ [this] () { m_hl_index[m_hl_offset].b.l = alu_set(m_hl_index[m_hl_offset].b.l); } },
		// cb e6, 15 cycles, SET 4,(HL), see cb 86 for timing
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { m_icount -= 2; write_s(alu_set(m_data_bus)); }
		},
		// cb e7, 8 cycles, SET 4,A
		{ [this] () { m_af.b.h = alu_set(m_af.b.h); } },

		// cb e8, 8 cycles, SET 5,B
		{ [this] () { m_bc.b.h = alu_set(m_bc.b.h); } },
		// cb e9, 8 cycles, SET 5,C
		{ [this] () { m_bc.b.l = alu_set(m_bc.b.l); } },
		// cb ea, 8 cycles, SET 5,D
		{ [this] () { m_de.b.h = alu_set(m_de.b.h); } },
		// cb eb, 8 cycles, SET 5,E
		{ [this] () { m_de.b.l = alu_set(m_de.b.l); } },
		// cb ec, 8 cycles, SET 5,H
		{ [this] () { m_hl_index[m_hl_offset].b.h = alu_set(m_hl_index[m_hl_offset].b.h); } },
		// cb ed, 8 cycles, SET 5,L
		{ [this] () { m_hl_index[m_hl_offset].b.l = alu_set(m_hl_index[m_hl_offset].b.l); } },
		// cb ee, 15 cycles, SET 5,(HL), see cb 86 for timing
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { m_icount -= 2; write_s(alu_set(m_data_bus)); }
		},
		// cb ef, 8 cycles, SET 5,A
		{ [this] () { m_af.b.h = alu_set(m_af.b.h); } },

		// cb f0, 8 cycles, SET 6,B
		{ [this] () { m_bc.b.h = alu_set(m_bc.b.h); } },
		// cb f1, 8 cycles, SET 6,C
		{ [this] () { m_bc.b.l = alu_set(m_bc.b.l); } },
		// cb f2, 8 cycles, SET 6,D
		{ [this] () { m_de.b.h = alu_set(m_de.b.h); } },
		// cb f3, 8 cycles, SET 6,E
		{ [this] () { m_de.b.l = alu_set(m_de.b.l); } },
		// cb f4, 8 cycles, SET 6,H
		{ [this] () { m_hl_index[m_hl_offset].b.h = alu_set(m_hl_index[m_hl_offset].b.h); } },
		// cb f5, 8 cycles, SET 6,L
		{ [this] () { m_hl_index[m_hl_offset].b.l = alu_set(m_hl_index[m_hl_offset].b.l); } },
		// cb f6, 15 cycles, SET 6,(HL), see cb 86 for timing
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { m_icount -= 2; write_s(alu_set(m_data_bus)); }
		},
		// cb f7, 8 cycles, SET 6,A
		{ [this] () { m_af.b.h = alu_set(m_af.b.h); } },

		// cb f8, 8 cycles, SET 7,B
		{ [this] () { m_bc.b.h = alu_set(m_bc.b.h); } },
		// cb f9, 8 cycles, SET 7,C
		{ [this] () { m_bc.b.l = alu_set(m_bc.b.l); } },
		// cb fa, 8 cycles, SET 7,D
		{ [this] () { m_de.b.h = alu_set(m_de.b.h); } },
		// cb fb, 8 cycles, SET 7,E
		{ [this] () { m_de.b.l = alu_set(m_de.b.l); } },
		// cb fc, 8 cycles, SET 7,H
		{ [this] () { m_hl_index[m_hl_offset].b.h = alu_set(m_hl_index[m_hl_offset].b.h); } },
		// cb fd, 8 cycles, SET 7,L
		{ [this] () { m_hl_index[m_hl_offset].b.l = alu_set(m_hl_index[m_hl_offset].b.l); } },
		// cb fe, 15 cycles, SET 7,(HL), see cb 86 for timing
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { m_icount -= 2; write_s(alu_set(m_data_bus)); }
		},
		// cb ff, 8 cycles, SET 7,A
		{ [this] () { m_af.b.h = alu_set(m_af.b.h); } },

		/*****************************************************/
		/* ED-prefixed instructions                          */
		/*****************************************************/

		// ed 00-07
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		// ed 08-0f
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },

		// ed 10-17
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		// ed 18-1f
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },

		// ed 20-27
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		// ed 28-2f
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },

		// ed 30-37
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		// ed 38-3f
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },

		// ed 40, 12 cycles, IN B,(C)
		//  9 T1 AB:bbcc DB:--
		// 10 T2 AB:bbcc DB:xx RD IORQ
		// 11 T3 AB:bbcc DB:xx RD IORQ
		// 12 T4 AB:bbcc DB:xx RD IORQ
		{
			[this] () { input_s(m_bc.w.l); },
			[this] () { m_bc.b.h = m_data_bus; m_af.b.l = (m_af.b.l & CF) | SZP[m_data_bus]; }
		},
		// ed 41, 12 cycles, OUT (C),B
		//  9 T1 AB:bbcc DB:--
		// 10 T2 AB:bbcc DB:xx WR IORQ
		// 11 T3 AB:bbcc DB:xx WR IORQ
		// 12 T4 AB:bbcc DB:xx WR IORQ
		{
			[this] () { output_s(m_bc.w.l, m_bc.b.h); }
		},
		// ed 42, 15 cycles, SBC HL,BC
		//  9 T1 AB:1235 DB:--
		// 10 T2 AB:1235 DB:--
		// 11 T3 AB:1235 DB:--
		// 12 T4 AB:1235 DB:--
		// 13 T1 AB:1235 DB:--
		// 14 T2 AB:1235 DB:--
		// 15 T3 AB:1235 DB:--
		{ [this] () { sbc16(); } },
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
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; write_s(m_wz.w.l, m_bc.b.l); m_wz.w.l++; },
			[this] () { write_s(m_wz.w.l, m_bc.b.h); }
		},
		// ed 44, 8 cycles, NEG
		{ [this] () { neg(); } },
		// ed 45, 14 cycles, RETN
		//  9 T1 AB:5678 DB:--
		// 10 T2 AB:5678 DB:yy MREQ RD
		// 11 T3 AB:5678 DB:yy MREQ RD
		// 12 T1 AB:5679 DB:--
		// 13 T2 AB:5679 DB:xx MREQ RD
		// 14 T3 AB:5679 DB:xx MREQ RD
		{
			[this] () { retn(); read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_wz.b.h = m_data_bus; m_pc.w.l = m_wz.w.l; }
		},
		// ed 46, 8 cycles, IM 0
		{ [this] () { im(); } },
		// ed 47, 9 cycles, LD I,A
		// 9 AB:1235 DB:--
		{ [this] () { ld_i_a(); } },

		// ed 48, 12 cycles, IN C,(C), see ed 40 for timing
		{
			[this] () { input_s(m_bc.w.l); },
			[this] () { m_bc.b.l = m_data_bus; m_af.b.l = (m_af.b.l & CF) | SZP[m_data_bus]; }
		},
		// ed 49, 12 cycles, OUT (C),C, see ed 41 for timing
		{
			[this] () { output_s(m_bc.w.l, m_bc.b.l); }
		},
		// ed 4a, 15 cycles, ADC HL,BC, see ed 42 for timing
		{ [this] () { adc16(); } },
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
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; read_s(m_wz.w.l++); },
			[this] () { m_bc.b.l = m_data_bus; read_s(m_wz.w.l); },
			[this] () { m_bc.b.h = m_data_bus; }
		},
		// ed 4c, 8 cycles, NEG
		{ [this] () { neg(); } },
		// ed 4d, 14 cycles, RETI, sed ed 45 for timing
		{
			[this] () { reti(); read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_wz.b.h = m_data_bus; m_pc.w.l = m_wz.w.l; }
		},
		// ed 4e, 8 cycles, IM 0
		{ [this] () { im(); } },
		// ed 4f, 9 cycles, LD R,A
		{ [this] () { ld_r_a(); } },

		// ed 50, 12 cycles, IN D,(C), see ed 40 for timing
		{
			[this] () { input_s(m_bc.w.l); },
			[this] () { m_de.b.h = m_data_bus; m_af.b.l = (m_af.b.l & CF) | SZP[m_data_bus]; }
		},
		// ed 51, 12 cycles, OUT (C),D, see ed 41 for timing
		{
			[this] () { output_s(m_bc.w.l, m_de.b.h); }
		},
		// ed 52, 15 cycles SBC HL,DE, see ed 42 for timing
		{ [this] () { sbc16(); } },
		// ed 53, 20 cycles, LD (nn),DE, see ed 43 for timing
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; write_s(m_wz.w.l, m_de.b.l); m_wz.w.l++; },
			[this] () { write_s(m_wz.w.l, m_de.b.h); }
		},
		// ed 54, 8 cycles, NEG
		{ [this] () { neg(); } },
		// ed 55, 14 cycles, RETN, see ed 45 for timing
		{
			[this] () { retn(); read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_wz.b.h = m_data_bus; m_pc.w.l = m_wz.w.l; }
		},
		// ed 56, 8 cycles, IM 1
		{ [this] () { im(); } },
		// ed 57, 9 cycles, LD A,I
		{ [this] () { ld_a_i(); } },

		// ed 58, 12 cycles, IN E,(C), see ed 40 for timing
		{
			[this] () { input_s(m_bc.w.l); },
			[this] () { m_de.b.l = m_data_bus; m_af.b.l = (m_af.b.l & CF) | SZP[m_data_bus]; }
		},
		// ed 59, 12 cycles, OUT (C),E, see ed 41 for timing
		{
			[this] () { output_s(m_bc.w.l, m_de.b.l); }
		},
		// ed 5a, 15 cycles, ADC HL,DE, see ed 42 for timing
		{ [this] () { adc16(); } },
		// ed 5b, 20 cycles, LD DE,(nn), see ed 4b for timing
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; read_s(m_wz.w.l++); },
			[this] () { m_de.b.l = m_data_bus; read_s(m_wz.w.l); },
			[this] () { m_de.b.h = m_data_bus; }
		},
		// ed 5c, 8 cycles, NEG
		{ [this] () { neg(); } },
		// ed 5d, 14 cycles, RETN, see ed 42 for timing
		{
			[this] () { retn(); read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_wz.b.h = m_data_bus; m_pc.w.l = m_wz.w.l; }
		},
		// ed 5e, 8 cycles, IM 2
		{ [this] () { im(); } },
		// ed 5f, 9 cycles, LD A,R
		{ [this] () { ld_a_r(); } },

		// ed 60, 12 cycles, IN H,(C), see ed 40 for timing
		{
			[this] () { input_s(m_bc.w.l); },
			[this] () { m_hl_index[m_hl_offset].b.h = m_data_bus; m_af.b.l = (m_af.b.l & CF) | SZP[m_data_bus]; }
		},
		// ed 61, 12 cycles, OUT (C),H, see ed 41 for timing
		{
			[this] () { output_s(m_bc.w.l, m_hl_index[m_hl_offset].b.h); }
		},
		// ed 62, 15 cycles, SBC HL,HL, see ed 42 for timing
		{ [this] () { sbc16(); } },
		// ed 63, 20 cycles, LD (nn),HL, see ed 43 for timing
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; write_s(m_wz.w.l, m_hl_index[m_hl_offset].b.l); m_wz.w.l++; },
			[this] () { write_s(m_wz.w.l, m_hl_index[m_hl_offset].b.h); }
		},
		// ed 64, 8 cycles, NEG
		{ [this] () { neg(); } },
		// ed 65, 14 cycles, RETN, see ed 45 for timing
		{
			[this] () { retn(); read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_wz.b.h = m_data_bus; m_pc.w.l = m_wz.w.l; }
		},
		// ed 66, 8 cycles, IM 0
		{ [this] () { im(); } },
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
		{
			[this] () { m_wz.w.l = m_hl_index[m_hl_offset].w.l; read_s(m_wz.w.l++); },
			[this] () { rrd(); write_s(m_data_bus); }
		},

		// ed 68, 12 cycles, IN L,(C), see ed 40 for timing
		{
			[this] () { input_s(m_bc.w.l); },
			[this] () { m_hl_index[m_hl_offset].b.l = m_data_bus; m_af.b.l = (m_af.b.l & CF) | SZP[m_data_bus]; }
		},
		// ed 69, 12 cycles, OUT (C),L, see ed 41 for timing
		{
			[this] () { output_s(m_bc.w.l, m_hl_index[m_hl_offset].b.l); }
		},
		// ed 6a, 15 cycles, ADC HL,HL, see ed 42 for timing
		{ [this] () { adc16(); } },
		// ed 6b, 20 cycles, LD HL,(nn), see ed 4b for timing
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; read_s(m_wz.w.l++); },
			[this] () { m_hl_index[m_hl_offset].b.l = m_data_bus; read_s(m_wz.w.l); },
			[this] () { m_hl_index[m_hl_offset].b.h = m_data_bus; }
		},
		// ed 6c, 8 cycles, NEG
		{ [this] () { neg(); } },
		// ed 6d, 14 cycles, RETN, see ed 45 for timing
		{
			[this] () { retn(); read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_wz.b.h = m_data_bus; m_pc.w.l = m_wz.w.l; }
		},
		// ed 6e, 8 cycles, IM 0
		{ [this] () { im(); } },
		// ed 6f, 18 cycles, RLD, see ed 67 for timing
		{
			[this] () { m_wz.w.l = m_hl_index[m_hl_offset].w.l; read_s(m_wz.w.l++); },
			[this] () { rld(); write_s(m_data_bus); }
		},

		// ed 70, 12 cycles, IN F,(C), see ed 40 for timing
		{
			[this] () { input_s(m_bc.w.l); },
			[this] () { m_af.b.l = (m_af.b.l & CF) | SZP[m_data_bus]; }
		},
		// ed 71, 12 cycles, OUT (C),0, see ed 41 for timing
		{
			[this] () { output_s(m_bc.w.l, 0); }
		},
		// ed 72, 15 cycles, SBC HL,SP, see ed 42 for timing
		{ [this] () { sbc16(); } },
		// ed 73, 20 cycles, LD (nn),SP, see ed 43 for timing
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; write_s(m_wz.w.l, m_sp.b.l); m_wz.w.l++; },
			[this] () { write_s(m_wz.w.l, m_sp.b.h); }
		},
		// ed 74, 8 cycles, NEG
		{ [this] () { neg(); } },
		// ed 75, 14 cycles, RETN, see ed 45 for timing
		{
			[this] () { retn(); read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_wz.b.h = m_data_bus; m_pc.w.l = m_wz.w.l; }
		},
		// ed 76, 8 cycles, IM 1
		{ [this] () { im(); } },
		// ed 77, 8 cycles, illegal
		{ [] () { } },

		// ed 78, 12 cycles, IN A,(C), see ed 40 for timing
		{
			[this] () { input_s(m_bc.w.l); },
			[this] () { m_af.b.h = m_data_bus; m_af.b.l = (m_af.b.l & CF) | SZP[m_data_bus]; }
		},
		// ed 79, 12 cycles, OUT (C),A, see ed 41 for timing
		{
			[this] () { output_s(m_bc.w.l, m_af.b.h); }
		},
		// ed 7a, 15 cycles, ADC HL,SP, see ed 42 for timing
		{ [this] () { adc16(); } },
		// ed 7b, 20 cycles, LD SP,(nn), see ed 4b for timing
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; read_s(m_wz.w.l++); },
			[this] () { m_sp.b.l = m_data_bus; read_s(m_wz.w.l); },
			[this] () { m_sp.b.h = m_data_bus; }
		},
		// ed 7c, 8 cycles, NEG
		{ [this] () { neg(); } },
		// ed 7d, 14 cycles, RETN, see ed 45 for timing
		{
			[this] () { retn(); read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_wz.b.h = m_data_bus; m_pc.w.l = m_wz.w.l; }
		},
		// ed 7e, 8 cycles, IM 2
		{ [this] () { im(); } },
		// ed 7f, 8 cycles, illegal
		{ [] () { } },

		// ed 80-87
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		// ed 88-8f
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },

		// ed 90-97
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		// ed 98-9f
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },

		// ed a0, 16 cycles, LDI
		//  9 T1 AB:hhll DB:--
		// 10 T2 AB:hhll DB:xx MREQ RD
		// 11 T3 AB:hhll DB:xx MREQ RD
		// 12 T1 AB:ddee DB:--
		// 13 T2 AB:ddee DB:xx MREQ
		// 14 T3 AB:ddee DB:xx MREQ WR
		// 15 T4 AB:ddee DB:--
		// 16 T5 AB:ddee DB:--
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { write_s(m_de.w.l, m_data_bus); },
			[this] () { ldi(); }
		},
		// ed a1, 16 cycles, CPI
		//  9 T1 AB:hhll DB:--
		// 10 T2 AB:hhll DB:xx MREQ RD
		// 11 T3 AB:hhll DB:xx MREQ RD
		// 12 T1 AB:hhll DB:--
		// 13 T2 AB:hhll DB:--
		// 14 T3 AB:hhll DB:--
		// 15 T4 AB:hhll DB:--
		// 16 T5 AB:hhll DB:--
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { cpi(); }
		},
		// ed a2, 16 cycles, INI
		//  9 T5 AB:1235 DB:--
		// 10 T1 AB:bbcc DB:--
		// 11 T2 AB:bbcc DB:xx RD IORQ
		// 12 T3 AB:bbcc DB:xx RD IORQ
		// 13 T4 AB:bbcc DB:xx RD IORQ
		// 14 T1 AB:hhll DB:--
		// 15 T2 AB:hhll DB:xx MREQ
		// 16 T3 AB:hhll DB:xx MREQ WR
		{
			[this] () { m_icount -= 1; input_s(m_bc.w.l); },
			[this] () { write_s(m_hl_index[m_hl_offset].w.l, m_data_bus); },
			[this] () { ini(); }
		},
		// ed a3, 16 cycles, OUTI
		//  9 T5 AB:1235 DB:--
		// 10 T1 AB:hhll DB:--
		// 11 T2 AB:hhll DB:xx MREQ RD
		// 12 T3 AB:hhll DB:xx MREQ RD
		// 13 T1 AB:bbcc DB:--
		// 14 T2 AB:bbcc DB:xx WR IORQ
		// 15 T3 AB:bbcc DB:xx WR IORQ
		// 16 T4 AB:bbcc DB:xx WR IORQ
		{
			[this] () { m_icount -= 1; read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { outi(); output_s(m_data_bus); }
		},
		// ed a4-a7
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },

		// ed a8, 16 cycles, LDD
		//  9 T1 AB:hhll DB:--
		// 10 T2 AB:hhll DB:xx MREQ RD
		// 11 T3 AB:hhll DB:xx MREQ RD
		// 12 T1 AB:ddee DB:--
		// 13 T2 AB:ddee DB:xx MREQ
		// 14 T3 AB:ddee DB:xx MREQ WR
		// 15 T4 AB:ddee DB:--
		// 16 T5 AB:ddee DB:--
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { write_s(m_de.w.l, m_data_bus); },
			[this] () { ldd(); }
		},
		// ed a9, 16 cycles, CPD
		//  9 T1 AB:hhll DB:--
		// 10 T2 AB:hhll DB:xx MREQ RD
		// 11 T3 AB:hhll DB:xx MREQ RD
		// 12 T1 AB:hhll DB:--
		// 13 T2 AB:hhll DB:--
		// 14 T3 AB:hhll DB:--
		// 15 T4 AB:hhll DB:--
		// 16 T5 AB:hhll DB:--
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { cpd(); }
		},
		// ed aa, 16 cycles, IND
		//  9 T5 AB:1235 DB:--
		// 10 T1 AB:bbcc DB:--
		// 11 T2 AB:bbcc DB:xx RD IORQ
		// 12 T3 AB:bbcc DB:xx RD IORQ
		// 13 T4 AB:bbcc DB:xx RD IORQ
		// 14 T1 AB:hhll DB:--
		// 15 T2 AB:hhll DB:xx MREQ
		// 16 T3 AB:hhll DB:xx MREQ WR
		{
			[this] () { m_icount -= 1; input_s(m_bc.w.l); },
			[this] () { write_s(m_hl_index[m_hl_offset].w.l, m_data_bus); },
			[this] () { ind(); }
		},
		// ed ab, 16 cycles, OUTD
		//  9 T5 AB:1235 DB:--
		// 10 T1 AB:hhll DB:--
		// 11 T2 AB:hhll DB:xx MREQ RD
		// 12 T3 AB:hhll DB:xx MREQ RD
		// 13 T1 AB:bbcc DB:--
		// 14 T2 AB:bbcc DB:xx WR IORQ
		// 15 T3 AB:bbcc DB:xx WR IORQ
		// 16 T4 AB:bbcc DB:xx WR IORQ
		{
			[this] () { m_icount -= 1; read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { outd(); output_s(m_data_bus); }
		},
		// ed ac-af
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },

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
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { write_s(m_de.w.l, m_data_bus); },
			[this] () { ldi(); repeat(); }
		},
		//* ed b1, 16/21 cycles, CPIR
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
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { cpi(); repeatcp(); }
		},
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
		{
			[this] () { m_icount -= 1; input_s(m_bc.w.l); },
			[this] () { write_s(m_hl_index[m_hl_offset].w.l, m_data_bus); },
			[this] () { ini(); repeatio(); }
		},
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
		{
			[this] () { m_icount -= 1; read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { outi(); output_s(m_data_bus); },
			[this] () { repeatio(); }
		},
		// ed b4-b7
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },

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
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { write_s(m_de.w.l, m_data_bus); },
			[this] () { ldd(); repeat(); }
		},
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
		{
			[this] () { read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { cpd(); repeatcp(); }
		},
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
		{
			[this] () { m_icount -= 1; input_s(m_bc.w.l); },
			[this] () { write_s(m_hl_index[m_hl_offset].w.l, m_data_bus); },
			[this] () { ind(); repeatio(); }
		},
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
		{
			[this] () { m_icount -= 1; read_s(m_hl_index[m_hl_offset].w.l); },
			[this] () { outd(); output_s(m_data_bus); },
			[this] () { repeatio(); }
		},
		// ed bc-bf
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },

		// ed c0-c7
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		// ed c8-cf
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },

		// ed d0-d7
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		// ed d8-df
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },

		// ed e0-e7
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		// ed e8-ef
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },

		// ed f0-f7
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		// ed f8-ff
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },
		{ [] () { } },

		/*****************************************************/
		/* DD/FD prefixed instructions                       */
		/* Almost equal to regular instructions              */
		/*****************************************************/

		// dd/fd 00, 8 cycles, NOP
		{ [] () { } },
		// dd/fd 01, 14 cycles, LD BC,nn
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_bc.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_bc.b.h = m_data_bus; }
		},
		// dd/fd 02, 11 cycles, LD (BC),A
		{
			[this] () { m_wz.w.l = m_bc.w.l; write_s(m_wz.w.l, m_af.b.h); m_wz.b.h = m_af.b.h; m_wz.w.l++; }
		},
		// dd/fd 03, 10 cycles, INC BC
		{ [this] () { m_bc.w.l++; m_icount -= 2; } },
		// dd/fd 04, 8 cycles, INC B
		{ [this] () { m_bc.b.h = alu_inc(m_bc.b.h); } },
		// dd/fd 05, 8 cycles, DEC B
		{ [this] () { m_bc.b.h = alu_dec(m_bc.b.h); } },
		// dd/fd 06, 11 cycles, LD B,n
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_bc.b.h = m_data_bus; }
		},
		// dd/fd 07, 8 cycles, RLCA
		{ [this] () { rlca(); } },

		// dd/fd 08, 8 cycles, EX AF,AF'
		{ [this] () { ex_af_af(); } },
		// dd/fd 09, 15 cycles, ADD IX/IY,BC
		{ [this] () { add16(); } },
		// dd/fd 0a, 11 cycles, LD A,(BC)
		{
			[this] () { m_wz.w.l = m_bc.w.l; read_s(m_wz.w.l++); },
			[this] () { m_af.b.h = m_data_bus; }
		},
		// dd/fd 0b, 10 cycles, DEC BC
		{ [this] () { m_bc.w.l--; m_icount -= 2; } },
		// dd/fd 0c, 8 cycles, INC C
		{ [this] () { m_bc.b.l = alu_inc(m_bc.b.l); } },
		// dd/fd 0d, 8 cycles, DEC C
		{ [this] () { m_bc.b.l = alu_dec(m_bc.b.l); } },
		// dd/fd 0e, 11 cycles, LD C,n
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_bc.b.l = m_data_bus; }
		},
		// dd/fd 0f, 8 cycles, RRCA
		{ [this] () { rrca(); } },

		// dd/fd 10, 12/17 cycles, DJNZ n
		{
			// TODO: double check m_icount -= 1
			[this] () { m_icount -= 1; read_s(m_pc.w.l++); },
			[this] () { djnz(); }
		},
		// dd/fd 11, 14 cycles, LD DE,nn
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_de.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_de.b.h = m_data_bus; }
		},
		// dd/fd 12, 7 cycles, LD (DE),A
		{
			[this] () { m_wz.w.l = m_de.w.l; write_s(m_wz.w.l, m_af.b.h); m_wz.b.h = m_af.b.h; m_wz.w.l++; }
		},
		// dd/fd 13, 10 cycles, INC DE
		{ [this] () { m_de.w.l++; m_icount -= 2; } },
		// dd/fd 14, 8 cycles, INC D
		{ [this] () { m_de.b.h = alu_inc(m_de.b.h); } },
		// dd/fd 15, 8 cycles, DEC D
		{ [this] () { m_de.b.h = alu_dec(m_de.b.h); } },
		// dd/fd 16, 11 cycles, LD D,n
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_de.b.h = m_data_bus; }
		},
		// dd/fd 17, 8 cycles, RLA
		{ [this] () { rla(); } },

		// dd/fd 18, 16 cycles, JR n
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { jr_cond(); }
		},
		// dd/fd 19, 11 cycles, ADD IX/IY,DE
		{ [this] () { add16(); } },
		// dd/fd 1a, 11 cycles, LD A,(DE)
		{
			[this] () { m_wz.w.l = m_de.w.l; read_s(m_wz.w.l++); },
			[this] () { m_af.b.h = m_data_bus; }
		},
		// dd/fd 1b, 10 cycles, DEC DE
		{ [this] () { m_de.w.l--; m_icount -= 2; } },
		// dd/fd 1c, 8 cycles, INC E
		{ [this] () { m_de.b.l = alu_inc(m_de.b.l); } },
		// dd/fd 1d, 8 cycles, DEC E
		{ [this] () { m_de.b.l = alu_dec(m_de.b.l); } },
		// dd/fd 1e, 11 cycles, LD E,n
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_de.b.l = m_data_bus; }
		},
		// dd/fd 1f, 8 cycles, RRA
		{ [this] () { rra(); } },

		// dd/fd 20, 11/16 cycles, JR NZ,n
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { jr_cond(); }
		},
		// dd/fd 21, 14 cycles, LD IX/IY,nn
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_hl_index[m_hl_offset].b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_hl_index[m_hl_offset].b.h = m_data_bus; }
		},
		// dd/fd 22, 20 cycles, LD (nn),IX/IY
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; write_s(m_wz.w.l, m_hl_index[m_hl_offset].b.l); m_wz.w.l++; },
			[this] () { write_s(m_wz.w.l, m_hl_index[m_hl_offset].b.h); }
		},
		// dd/fd 23, 10 cycles, INC IX/IY
		{ [this] () { m_hl_index[m_hl_offset].w.l++; m_icount -= 2; } },
		// dd/fd 24, 8 cycles, INC IXh/IYh
		{ [this] () { m_hl_index[m_hl_offset].b.h = alu_inc(m_hl_index[m_hl_offset].b.h); } },
		// dd/fd 25, 8 cycles, DEC IXh/IYh
		{ [this] () { m_hl_index[m_hl_offset].b.h = alu_dec(m_hl_index[m_hl_offset].b.h); } },
		// dd/fd 26, 11 cycles, LD IXh/IYh,n
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_hl_index[m_hl_offset].b.h = m_data_bus; }
		},
		// dd/fd 27, 8 cycles, DAA
		{ [this] () { daa(); } },

		// dd/fd 28, 11/16 cycles, JR Z,n
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { jr_cond(); }
		},
		// dd/fd 29, 15 cycles, ADD IX/IY,IX/IY
		{ [this] () { add16(); } },
		// dd/fd 2a, 20 cycles, LD IX/IY,(nn)
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; read_s(m_wz.w.l++); },
			[this] () { m_hl_index[m_hl_offset].b.l = m_data_bus; read_s(m_wz.w.l); },
			[this] () { m_hl_index[m_hl_offset].b.h = m_data_bus; }
		},
		// dd/fd 2b, 10 cycles, DEC IX/IY
		{ [this] () { m_hl_index[m_hl_offset].w.l--; m_icount -= 2; } },
		// dd/fd 2c, 8 cycles, INC IXl/IYl
		{ [this] () { m_hl_index[m_hl_offset].b.l = alu_inc(m_hl_index[m_hl_offset].b.l); } },
		// dd/fd 2d, 8 cycles, DEC IXl/IYl
		{ [this] () { m_hl_index[m_hl_offset].b.l = alu_dec(m_hl_index[m_hl_offset].b.l); } },
		// dd/fd 2e, 11 cycles, LD IXl/IYl,n
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_hl_index[m_hl_offset].b.l = m_data_bus; }
		},
		// dd/fd 2f, 8 cycles, CPL
		{ [this] () { cpl(); } },

		// dd/fd 30, 11/16 cycles, JR NC,n
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { jr_cond(); }
		},
		// dd/fd 31, 14 cycles, LD SP,nn
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_sp.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_sp.b.h = m_data_bus; }
		},
		// dd/fd 32, 17 cycles, LD (nn),A
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; write_s(m_wz.w.l, m_af.b.h); m_wz.b.h = m_af.b.h; m_wz.w.l++; }
		},
		// dd/fd 33, 10 cycles, INC SP
		{ [this] () { m_sp.w.l++; m_icount -= 2; } },
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
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { disp_wz5(); read_s(m_wz.w.l); },
			[this] () { m_icount -= 2; write_s(alu_inc(m_data_bus)); }
		},
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
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { disp_wz5(); read_s(m_wz.w.l); },
			[this] () { m_icount -= 2; write_s(alu_dec(m_data_bus)); }
		},
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
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { db_tmp(); disp_wz2(); read_s(m_pc.w.l++); },
			[this] () { write_s(m_wz.w.l, m_data_bus); }
		},
		// dd/fd 37, 8 cycles, SCF
		{ [this] () { scf(); } },

		// dd/fd 38, 11/16 cycles, JR C,n
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { jr_cond(); }
		},
		// dd/fd 39, 15 cycles, ADD IX/IY,SP
		{ [this] () { add16(); } },
		// dd/fd 3a, 17 cycles, LD A,(nn)
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; read_s(m_wz.w.l++); },
			[this] () { m_af.b.h = m_data_bus; }
		},
		// dd/fd 3b, 10 cycles, DEC SP
		{ [this] () { m_sp.w.l--; m_icount -= 2; } },
		// dd/fd 3c, 8 cycles, INC A
		{ [this] () { m_af.b.h = alu_inc(m_af.b.h); } },
		// dd/fd 3d, 8 cycles, DEC A
		{ [this] () { m_af.b.h = alu_dec(m_af.b.h); } },
		// dd/fd 3e, 11 cycles, LD A,n
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_af.b.h = m_data_bus; }
		},
		// dd/fd 3f, 8 cycles, CCF
		{ [this] () { ccf(); } },

		// dd/fd 40, 8 cycles, LD B,B
		{ [this] () { m_bc.b.h = m_bc.b.h; } },
		// dd/fd 41, 8 cycles, LD B,C
		{ [this] () { m_bc.b.h = m_bc.b.l; } },
		// dd/fd 42, 8 cycles, LD B,D
		{ [this] () { m_bc.b.h = m_de.b.h; } },
		// dd/fd 43, 8 cycles, LD B,E
		{ [this] () { m_bc.b.h = m_de.b.l; } },
		// dd/fd 44, 8 cycles, LD B,IXh/IYh
		{ [this] () { m_bc.b.h = m_hl_index[m_hl_offset].b.h; } },
		// dd/fd 45, 8 cycles, LD B,IXl/IYl
		{ [this] () { m_bc.b.h = m_hl_index[m_hl_offset].b.l; } },
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
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { disp_wz5(); read_s(m_wz.w.l); },
			[this] () { m_bc.b.h = m_data_bus; }
		},
		// dd/fd 47, 8 cycles, LD B,A
		{ [this] () { m_bc.b.h = m_af.b.h; } },

		// dd/fd 48, 8 cycles, LD C,B
		{ [this] () { m_bc.b.l = m_bc.b.h; } },
		// dd/fd 49, 8 cycles, LD C,C
		{ [this] () { m_bc.b.l = m_bc.b.l; } },
		// dd/fd 4a, 8 cycles, LD C,D
		{ [this] () { m_bc.b.l = m_de.b.h; } },
		// dd/fd 4b, 8 cycles, LD C,E
		{ [this] () { m_bc.b.l = m_de.b.l; } },
		// dd/fd 4c, 8 cycles, LD C,IXh/IYh
		{ [this] () { m_bc.b.l = m_hl_index[m_hl_offset].b.h; } },
		// dd/fd 4d, 8 cycles, LD C,IXl/IYl
		{ [this] () { m_bc.b.l = m_hl_index[m_hl_offset].b.l; } },
		// dd/fd 4e, 19 cycles, LD C,(IX/IY+dd)
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { disp_wz5(); read_s(m_wz.w.l); },
			[this] () { m_bc.b.l = m_data_bus; }
		},
		// dd/fd 4f, 8 cycles, LD C,A
		{ [this] () { m_bc.b.l = m_af.b.h; } },

		// dd/fd 50, 8 cycles, LD D,B
		{ [this] () { m_de.b.h = m_bc.b.h; } },
		// dd/fd 51, 8 cycles, LD D,C
		{ [this] () { m_de.b.h = m_bc.b.l; } },
		// dd/fd 52, 8 cycles, LD D,D
		{ [this] () { m_de.b.h = m_de.b.h; } },
		// dd/fd 53, 8 cycles, LD D,E
		{ [this] () { m_de.b.h = m_de.b.l; } },
		// dd/fd 54, 8 cycles, LD D,IXh/IYh
		{ [this] () { m_de.b.h = m_hl_index[m_hl_offset].b.h; } },
		// dd/fd 55, 8 cycles, LD D,IXl/IYl
		{ [this] () { m_de.b.h = m_hl_index[m_hl_offset].b.l; } },
		// dd/fd 56, 19 cycles, LD D,(IX/IY+dd)
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { disp_wz5(); read_s(m_wz.w.l); },
			[this] () { m_de.b.h = m_data_bus; }
		},
		// dd/fd 57, 8 cycles, LD D,A
		{ [this] () { m_de.b.h = m_af.b.h; } },

		// dd/fd 58, 8 cycles, LD E,B
		{ [this] () { m_de.b.l = m_bc.b.h; } },
		// dd/fd 59, 8 cycles, LD E,C
		{ [this] () { m_de.b.l = m_bc.b.l; } },
		// dd/fd 5a, 8 cycles, LD E,D
		{ [this] () { m_de.b.l = m_de.b.h; } },
		// dd/fd 5b, 8 cycles, LD E,E
		{ [this] () { m_de.b.l = m_de.b.l; } },
		// dd/fd 5c, 8 cycles, LD E,IXh/IYh
		{ [this] () { m_de.b.l = m_hl_index[m_hl_offset].b.h; } },
		// dd/fd 5d, 8 cycles, LD E,IXl/IYl
		{ [this] () { m_de.b.l = m_hl_index[m_hl_offset].b.l; } },
		// dd/fd 5e, 19 cycles, LD E,(IX/IY+dd)
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { disp_wz5(); read_s(m_wz.w.l); },
			[this] () { m_de.b.l = m_data_bus; }
		},
		// dd/fd 5f, 8 cycles, LD E,A
		{ [this] () { m_de.b.l = m_af.b.h; } },

		// dd/fd 60, 8 cycles, LD IXh/IYh,B
		{ [this] () { m_hl_index[m_hl_offset].b.h = m_bc.b.h; } },
		// dd/fd 61, 8 cycles, LD IXh/IYh,C
		{ [this] () { m_hl_index[m_hl_offset].b.h = m_bc.b.l; } },
		// dd/fd 62, 8 cycles, LD IXh/IYh,D
		{ [this] () { m_hl_index[m_hl_offset].b.h = m_de.b.h; } },
		// dd/fd 63, 8 cycles, LD IXh/IYh,E
		{ [this] () { m_hl_index[m_hl_offset].b.h = m_de.b.l; } },
		// dd/fd 64, 8 cycles, LD IXh/IYh,IXh/IYh
		{ [this] () { m_hl_index[m_hl_offset].b.h = m_hl_index[m_hl_offset].b.h; } },
		// dd/fd 65, 8 cycles, LD IXh/IYh,IXl/IYl
		{ [this] () { m_hl_index[m_hl_offset].b.h = m_hl_index[m_hl_offset].b.l; } },
		// dd/fd 66, 19 cycles, LD H,(IX/IY+dd)
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { disp_wz5(); read_s(m_wz.w.l); },
			[this] () { m_hl_index[HL_OFFSET].b.h = m_data_bus; }
		},
		// dd/fd 67, 8 cycles, LD IXh/IYh,A
		{ [this] () { m_hl_index[m_hl_offset].b.h = m_af.b.h; } },

		// dd/fd 68, 8 cycles, LD IXl/IYl,B
		{ [this] () { m_hl_index[m_hl_offset].b.l = m_bc.b.h; } },
		// dd/fd 69, 8 cycles, LD IXl/IYl,C
		{ [this] () { m_hl_index[m_hl_offset].b.l = m_bc.b.l; } },
		// dd/fd 6a, 8 cycles, LD IXl/IYl,D
		{ [this] () { m_hl_index[m_hl_offset].b.l = m_de.b.h; } },
		// dd/fd 6b, 8 cycles, LD IXl/IYl,E
		{ [this] () { m_hl_index[m_hl_offset].b.l = m_de.b.l; } },
		// dd/fd 6c, 8 cycles, LD IXl/IYl,IXh/IYh
		{ [this] () { m_hl_index[m_hl_offset].b.l = m_hl_index[m_hl_offset].b.h; } },
		// dd/fd 6d, 8 cycles, LD IXl/IYl,IXl/IYl
		{ [this] () { m_hl_index[m_hl_offset].b.l = m_hl_index[m_hl_offset].b.l; } },
		// dd/fd 6e, 19 cycles, LD L,(IX/IY+dd)
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { disp_wz5(); read_s(m_wz.w.l); },
			[this] () { m_hl_index[HL_OFFSET].b.l = m_data_bus; }
		},
		// dd/fd 6f, 8 cycles, LD IXl/IYl,A
		{ [this] () { m_hl_index[m_hl_offset].b.l = m_af.b.h; } },

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
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { disp_wz5(); write_s(m_wz.w.l, m_bc.b.h); }
		},
		// dd/fd 71, 19 cycles, LD (IX/IY+dd),C
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { disp_wz5(); write_s(m_wz.w.l, m_bc.b.l); }
		},
		// dd/fd 72, 19 cycles, LD (IX/IY+dd),D
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { disp_wz5(); write_s(m_wz.w.l, m_de.b.h); }
		},
		// dd/fd 73, 19 cycles, LD (IX/IY+dd),E
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { disp_wz5(); write_s(m_wz.w.l, m_de.b.l); }
		},
		// dd/fd 74, 19 cycles, LD (IX/IY+dd),H
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { disp_wz5(); write_s(m_wz.w.l, m_hl_index[HL_OFFSET].b.h); }
		},
		// dd/fd 75, 19 cycles, LD (IX/IY+dd),L
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { disp_wz5(); write_s(m_wz.w.l, m_hl_index[HL_OFFSET].b.l); }
		},
		// dd/fd 76, 8 cycles, HALT
		{
			[this] () { halt(); }
		},
		// dd/fd 77, 19 cycles, LD (IX/IY+dd),A
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { disp_wz5(); write_s(m_wz.w.l, m_af.b.h); }
		},

		// dd/fd 78, 8 cycles, LD A,B
		{ [this] () { m_af.b.h = m_bc.b.h; } },
		// dd/fd 79, 8 cycles, LD A,C
		{ [this] () { m_af.b.h = m_bc.b.l; } },
		// dd/fd 7a, 8 cycles, LD A,D
		{ [this] () { m_af.b.h = m_de.b.h; } },
		// dd/fd 7b, 8 cycles, LD A,E
		{ [this] () { m_af.b.h = m_de.b.l; } },
		// dd/fd 7c, 8 cycles, LD A,IXh/IYh
		{ [this] () { m_af.b.h = m_hl_index[m_hl_offset].b.h; } },
		// dd/fd 7d, 8 cycles, LD A,IXl/IYl
		{ [this] () { m_af.b.h = m_hl_index[m_hl_offset].b.l; } },
		// dd/fd 7e, 19 cycles, LD A,(IX/IY+dd)
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { disp_wz5(); read_s(m_wz.w.l); },
			[this] () { m_af.b.h = m_data_bus; }
		},
		// dd/fd 7f, 8 cycles, LD A,A
		{ [this] () { m_af.b.h = m_af.b.h; } },

		// dd/fd 80, 8 cycles, ADD B
		{ [this] () { alu_add(m_bc.b.h); } },
		// dd/fd 81, 8 cycles, ADD C
		{ [this] () { alu_add(m_bc.b.l); } },
		// dd/fd 82, 8 cycles, ADD D
		{ [this] () { alu_add(m_de.b.h); } },
		// dd/fd 83, 8 cycles, ADD E
		{ [this] () { alu_add(m_de.b.l); } },
		// dd/fd 84, 8 cycles, ADD IXh/IYh
		{ [this] () { alu_add(m_hl_index[m_hl_offset].b.h); } },
		// dd/fd 85, 8 cycles, ADD IXl/IYl
		{ [this] () { alu_add(m_hl_index[m_hl_offset].b.l); } },
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
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { disp_wz5(); read_s(m_wz.w.l); },
			[this] () { alu_add(m_data_bus); }
		},
		// dd/fd 87, 8 cycles, ADD A
		{ [this] () { alu_add(m_af.b.h); } },

		// dd/fd 88, 8 cycles, ADC B
		{ [this] () { alu_adc(m_bc.b.h); } },
		// dd/fd 89, 8 cycles, ADC C
		{ [this] () { alu_adc(m_bc.b.l); } },
		// dd/fd 8a, 8 cycles, ADC D
		{ [this] () { alu_adc(m_de.b.h); } },
		// dd/fd 8b, 8 cycles, ADC E
		{ [this] () { alu_adc(m_de.b.l); } },
		// dd/fd 8c, 8 cycles, ADC IXh/IYh
		{ [this] () { alu_adc(m_hl_index[m_hl_offset].b.h); } },
		// dd/fd 8d, 8 cycles, ADC IXl/IYl
		{ [this] () { alu_adc(m_hl_index[m_hl_offset].b.l); } },
		// dd/fd 8e, 19 cycles, ADC (IX/IY+dd)
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { disp_wz5(); read_s(m_wz.w.l); },
			[this] () { alu_adc(m_data_bus); }
		},
		// dd/fd 8f, 8 cycles, ADC A
		{ [this] () { alu_adc(m_af.b.h); } },

		// dd/fd 90, 8 cycles, SUB B
		{ [this] () { alu_sub(m_bc.b.h); } },
		// dd/fd 91, 8 cycles, SUB C
		{ [this] () { alu_sub(m_bc.b.l); } },
		// dd/fd 92, 8 cycles, SUB D
		{ [this] () { alu_sub(m_de.b.h); } },
		// dd/fd 93, 8 cycles, SUB E
		{ [this] () { alu_sub(m_de.b.l); } },
		// dd/fd 94, 8 cycles, SUB IXh/IYh
		{ [this] () { alu_sub(m_hl_index[m_hl_offset].b.h); } },
		// dd/fd 95, 8 cycles, SUB IXl/IYl
		{ [this] () { alu_sub(m_hl_index[m_hl_offset].b.l); } },
		// dd/fd 96, 19 cycles, SUB (IX/IY+dd)
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { disp_wz5(); read_s(m_wz.w.l); },
			[this] () { alu_sub(m_data_bus); }
		},
		// dd/fd 97, 8 cycles, SUB A
		{ [this] () { alu_sub(m_af.b.h); } },

		// dd/fd 98, 8 cycles, SBC B
		{ [this] () { alu_sbc(m_bc.b.h); } },
		// dd/fd 99, 8 cycles, SBC C
		{ [this] () { alu_sbc(m_bc.b.l); } },
		// dd/fd 9a, 8 cycles, SBC D
		{ [this] () { alu_sbc(m_de.b.h); } },
		// dd/fd 9b, 8 cycles, SBC E
		{ [this] () { alu_sbc(m_de.b.l); } },
		// dd/fd 9c, 8 cycles, SBC IXh/IYh
		{ [this] () { alu_sbc(m_hl_index[m_hl_offset].b.h); } },
		// dd/fd 9d, 8 cycles, SBC IXl/IYl
		{ [this] () { alu_sbc(m_hl_index[m_hl_offset].b.l); } },
		// dd/fd 9e, 19 cycles, SBC (IX/IY+dd)
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { disp_wz5(); read_s(m_wz.w.l); },
			[this] () { alu_sbc(m_data_bus); }
		},
		// dd/fd 9f, 8 cycles, SBC A
		{ [this] () { alu_sbc(m_af.b.h); } },

		// dd/fd a0, 8 cycles, AND B
		{ [this] () { alu_and(m_bc.b.h); } },
		// dd/fd a1, 8 cycles, AND C
		{ [this] () { alu_and(m_bc.b.l); } },
		// dd/fd a2, 8 cycles, AND D
		{ [this] () { alu_and(m_de.b.h); } },
		// dd/fd a3, 8 cycles, AND E
		{ [this] () { alu_and(m_de.b.l); } },
		// dd/fd a4, 8 cycles, AND IXh/IYh
		{ [this] () { alu_and(m_hl_index[m_hl_offset].b.h); } },
		// dd/fd a5, 8 cycles, AND IXl/IYl
		{ [this] () { alu_and(m_hl_index[m_hl_offset].b.l); } },
		// dd/fd a6, 19 cycles, AND (IX/IY+dd)
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { disp_wz5(); read_s(m_wz.w.l); },
			[this] () { alu_and(m_data_bus); }
		},
		// dd/fd a7, 8 cycles, AND A
		{ [this] () { alu_and(m_af.b.h); } },

		// dd/fd a8, 8 cycles, XOR B
		{ [this] () { alu_xor(m_bc.b.h); } },
		// dd/fd a9, 8 cycles, XOR C
		{ [this] () { alu_xor(m_bc.b.l); } },
		// dd/fd aa, 8 cycles, XOR D
		{ [this] () { alu_xor(m_de.b.h); } },
		// dd/fd ab, 8 cycles, XOR E
		{ [this] () { alu_xor(m_de.b.l); } },
		// dd/fd ac, 8 cycles, XOR IXh/IYh
		{ [this] () { alu_xor(m_hl_index[m_hl_offset].b.h); } },
		// dd/fd ad, 8 cycles, XOR IXl/IYl
		{ [this] () { alu_xor(m_hl_index[m_hl_offset].b.l); } },
		// dd/fd ae, 19 cycles, XOR (IX/IY+dd)
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { disp_wz5(); read_s(m_wz.w.l); },
			[this] () { alu_xor(m_data_bus); }
		},
		// dd/fd af, 8 cycles, XOR A
		{ [this] () { alu_xor(m_af.b.h); } },

		// dd/fd b0, 8 cycles, OR B
		{ [this] () { alu_or(m_bc.b.h); } },
		// dd/fd b1, 8 cycles, OR C
		{ [this] () { alu_or(m_bc.b.l); } },
		// dd/fd b2, 8 cycles, OR D
		{ [this] () { alu_or(m_de.b.h); } },
		// dd/fd b3, 8 cycles, OR E
		{ [this] () { alu_or(m_de.b.l); } },
		// dd/fd b4, 8 cycles, OR IXh/IYh
		{ [this] () { alu_or(m_hl_index[m_hl_offset].b.h); } },
		// dd/fd b5, 8 cycles, OR IXl/IYl
		{ [this] () { alu_or(m_hl_index[m_hl_offset].b.l); } },
		// dd/fd b6, 19 cycles, OR (IX/IY+dd)
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { disp_wz5(); read_s(m_wz.w.l); },
			[this] () { alu_or(m_data_bus); }
		},
		// dd/fd b7, 8 cycles, OR A
		{ [this] () { alu_or(m_af.b.h); } },

		// dd/fd b8, 8 cycles, CP B
		{ [this] () { alu_cp(m_bc.b.h); } },
		// dd/fd b9, 8 cycles, CP C
		{ [this] () { alu_cp(m_bc.b.l); } },
		// dd/fd ba, 8 cycles, CP D
		{ [this] () { alu_cp(m_de.b.h); } },
		// dd/fd bb, 8 cycles, CP E
		{ [this] () { alu_cp(m_de.b.l); } },
		// dd/fd bc, 8 cycles, CP IXh/IYh
		{ [this] () { alu_cp(m_hl_index[m_hl_offset].b.h); } },
		// dd/fd bd, 8 cycles, CP IXl/IYl
		{ [this] () { alu_cp(m_hl_index[m_hl_offset].b.l); } },
		// dd/fd be, 19 cycles, CP (IX/IY+dd)
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { disp_wz5(); read_s(m_wz.w.l); },
			[this] () { alu_cp(m_data_bus); }
		},
		// dd/fd bf, 8 cycles, CP A
		{ [this] () { alu_cp(m_af.b.h); } },

		// dd/fd c0, 9/15 cycles, RET NZ
		{
			[this] () { if (ret_cond()) { read_s(m_sp.w.l); m_sp.w.l += 1; } },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_wz.b.h = m_data_bus; m_pc.w.l = m_wz.w.l; }
		},
		// dd/fd c1, 14 cycles, POP BC
		{
			[this] () { read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_bc.b.l = m_data_bus; read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_bc.b.h = m_data_bus; }
		},
		// dd/fd c2, 14 cycles, JP NZ,nn
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; jp_cond(); }
		},
		// dd/fd c3, 14 cycles, JMP nn
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; m_pc.w.l = m_wz.w.l; }
		},
		// dd/fd c4, 14/21 cycles, CALL NZ,nn
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; if (call_cond()) { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.h); } },
			[this] () { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.l); },
			[this] () { m_pc.w.l = m_wz.w.l; }
		},
		// dd/fd c5, 15 cycles, PUSH BC
		{
			[this] () { m_icount -= 1; m_sp.w.l -= 1; write_s(m_sp.w.l, m_bc.b.h); },
			[this] () { m_sp.w.l -= 1; write_s(m_sp.w.l, m_bc.b.l); }
		},
		// dd/fd c6, 11 cycles, ADD A,n
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { alu_add(m_data_bus); }
		},
		// dd/fd c7, 15 cycles, RST 0H
		{
			[this] () { m_icount -= 1; m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.h); },
			[this] () { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.l); },
			[this] () { rst(); }
		},

		// dd/fd c8, 9/15 cycles, RET Z
		{
			[this] () { if (ret_cond()) { read_s(m_sp.w.l); m_sp.w.l += 1; } },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_wz.b.h = m_data_bus; m_pc.w.l = m_wz.w.l; }
		},
		// dd/fd c9, 14 cycles, RET
		{
			[this] () { read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_wz.b.h = m_data_bus; m_pc.w.l = m_wz.w.l; }
		},
		// dd/fd ca, 14 cycles, JP Z,nn
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; jp_cond(); }
		},
		// dd/fd cb, +4 cycles, DD/FD + CB prefix
		{ [] () { } },
		// dd/fd cc, 14/21 cycles, CALL Z,nn
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; if (call_cond()) { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.h); } },
			[this] () { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.l); },
			[this] () { m_pc.w.l = m_wz.w.l; }
		},
		// dd/fd cd, 21 cycles, CALL nn
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; m_icount -= 1; m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.h); },
			[this] () { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.l); },
			[this] () { m_pc.w.l = m_wz.w.l; }
		},
		// dd/fd ce, 11 cycles, ADC A,n
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { alu_adc(m_data_bus); }
		},
		// dd/fd cf, 15 cycles, RST 8H
		{
			[this] () { m_icount -= 1; m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.h); },
			[this] () { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.l); },
			[this] () { rst(); }
		},

		// dd/fd d0, 9/15 cycles, RET NC
		{
			[this] () { if (ret_cond()) { read_s(m_sp.w.l); m_sp.w.l += 1; } },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_wz.b.h = m_data_bus; m_pc.w.l = m_wz.w.l; }
		},
		// dd/fd d1, 14 cycles, POP DE
		{
			[this] () { read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_de.b.l = m_data_bus; read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_de.b.h = m_data_bus; }
		},
		// dd/fd d2, 14 cycles, JP NC,nn
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; jp_cond(); }
		},
		// dd/fd d3, 15 cycles, OUT (n), A
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; a_w(); output_s(m_wz.w.l, m_af.b.h); m_wz.b.h = m_af.b.h; m_wz.w.l++; }
		},
		// dd/fd d4, 14/21 cycles, CALL NC,nn
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; if (call_cond()) { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.h); } },
			[this] () { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.l); },
			[this] () { m_pc.w.l = m_wz.w.l; }
		},
		// dd/fd d5, 15 cycles, PUSH DE
		{
			[this] () { m_icount -= 1; m_sp.w.l -= 1; write_s(m_sp.w.l, m_de.b.h); },
			[this] () { m_sp.w.l -= 1; write_s(m_sp.w.l, m_de.b.l); }
		},
		// dd/fd d6, 11 cycles, SUB n
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { alu_sub(m_data_bus); }
		},
		// dd/fd d7, 15 cycles, RST 10H
		{
			[this] () { m_icount -= 1; m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.h); },
			[this] () { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.l); },
			[this] () { rst(); }
		},

		// dd/fd d8, 9/15 cycles, RET C
		{
			[this] () { if (ret_cond()) { read_s(m_sp.w.l); m_sp.w.l += 1; } },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_wz.b.h = m_data_bus; m_pc.w.l = m_wz.w.l; }
		},
		// dd/fd d9, 8 cycles, EXX
		{ [this] () { exx(); } },
		// dd/fd da, 14 cycles, JP C,nn
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; jp_cond(); }
		},
		// dd/fd db, 15 cycles, IN A,(n)
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; a_w(); input_s(m_wz.w.l); m_wz.w.l++; },
			[this] () { input_a(); }
		},
		// dd/fd dc, 14/21 cycles, CALL C,nn
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; if (call_cond()) { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.h); } },
			[this] () { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.l); },
			[this] () { m_pc.w.l = m_wz.w.l; }
		},
		// dd/fd dd, +4 cycles, DD prefix
		{ [] () {} },
		// dd/fd de, 11 cycles, SBC n
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { alu_sbc(m_data_bus); }
		},
		// dd/fd df, 15 cycles, RST 18H
		{
			[this] () { m_icount -= 1; m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.h); },
			[this] () { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.l); },
			[this] () { rst(); }
		},

		// dd/fd e0, 9/15 cycles, RET PO
		{
			[this] () { if (ret_cond()) { read_s(m_sp.w.l); m_sp.w.l += 1; } },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_wz.b.h = m_data_bus; m_pc.w.l = m_wz.w.l; }
		},
		// dd/fd e1, 14 cycles, POP IX/IY
		{
			[this] () { read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_hl_index[m_hl_offset].b.l = m_data_bus; read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_hl_index[m_hl_offset].b.h = m_data_bus; }
		},
		// dd/fd e2, 14 cycles, JP PO,nn
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; jp_cond(); }
		},
		// dd/fd e3, 23 cycles, EX (SP),IX/IY
		{
			[this] () { read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_sp.w.l); },
			[this] () { m_wz.b.h = m_data_bus; m_icount -= 2; write_s(m_hl_index[m_hl_offset].b.h); },
			[this] () { m_sp.w.l -= 1; write_s(m_sp.w.l, m_hl_index[m_hl_offset].b.l); },
			[this] () { m_icount -= 2; m_hl_index[m_hl_offset].w.l = m_wz.w.l; }
		},
		// dd/fd e4, 14/21 cycles, CALL PO,nn
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; if (call_cond()) { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.h); } },
			[this] () { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.l); },
			[this] () { m_pc.w.l = m_wz.w.l; }
		},
		// dd/fd e5, 15 cycles, PUSH IX/IY
		{
			[this] () { m_icount -= 1; m_sp.w.l -= 1; write_s(m_sp.w.l, m_hl_index[m_hl_offset].b.h); },
			[this] () { m_sp.w.l -= 1; write_s(m_sp.w.l, m_hl_index[m_hl_offset].b.l); }
		},
		// dd/fd e6, 11 cycles, AND n
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { alu_and(m_data_bus); }
		},
		// dd/fd e7, 15 cycles, RST 20H
		{
			[this] () { m_icount -= 1; m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.h); },
			[this] () { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.l); },
			[this] () { rst(); }
		},

		// dd/fd e8, 9/15 cycles, RET PE
		{
			[this] () { if (ret_cond()) { read_s(m_sp.w.l); m_sp.w.l += 1; } },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_wz.b.h = m_data_bus; m_pc.w.l = m_wz.w.l; }
		},
		// dd/fd e9, 8 cycles, JP (HL)
		{ [this] () { m_pc.w.l = m_hl_index[m_hl_offset].w.l; } },
		// dd/fd ea, 14 cycles, JP PE,nn
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; jp_cond(); }
		},
		// dd/fd eb, 8 cycles, EX DE,HL
		{ [this] () { ex_de_hl(); } },
		// dd/fd ec, 14/21 cycles, CALL PE,nn
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; if (call_cond()) { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.h); } },
			[this] () { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.l); },
			[this] () { m_pc.w.l = m_wz.w.l; }
		},
		// dd/fd ed, +4 cycles, ED prefix
		{ [] () { } },
		// dd/fd ee, 11 cycles, XOR n
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { alu_xor(m_data_bus); }
		},
		// dd/fd ef, 15 cycles, RST 28H
		{
			[this] () { m_icount -= 1; m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.h); },
			[this] () { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.l); },
			[this] () { rst(); }
		},

		// dd/fd f0, 9/15 cycles, RET P
		{
			[this] () { if (ret_cond()) { read_s(m_sp.w.l); m_sp.w.l += 1; } },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_wz.b.h = m_data_bus; m_pc.w.l = m_wz.w.l; }
		},
		// dd/fd f1, 14 cycles, POP AF
		{
			[this] () { read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_af.b.l = m_data_bus; read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_af.b.h = m_data_bus; }
		},
		// dd/fd f2, 14 cycles, JP P,nn
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; jp_cond(); }
		},
		// dd/fd f3, 8 cycles, DI
		{ [this] () { di(); } },
		// dd/fd f4, 14/21 cycles, CALL P,nn
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; if (call_cond()) { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.h); } },
			[this] () { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.l); },
			[this] () { m_pc.w.l = m_wz.w.l; }
		},
		// dd/fd f5, 15 cycles, PUSH AF
		{
			[this] () { m_icount -= 1; m_sp.w.l -= 1; write_s(m_sp.w.l, m_af.b.h); },
			[this] () { m_sp.w.l -= 1; write_s(m_sp.w.l, m_af.b.l); }
		},
		// dd/fd f6, 11 cycles, OR n
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { alu_or(m_data_bus); }
		},
		// dd/fd f7, 15 cycles, RST 30H
		{
			[this] () { m_icount -= 1; m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.h); },
			[this] () { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.l); },
			[this] () { rst(); }
		},

		// dd/fd f8, 9/15 cycles, RET M
		{
			[this] () { if (ret_cond()) { read_s(m_sp.w.l); m_sp.w.l += 1; } },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_sp.w.l); m_sp.w.l += 1; },
			[this] () { m_wz.b.h = m_data_bus; m_pc.w.l = m_wz.w.l; }
		},
		// dd/fd f9, 10 cycles, LD SP,IX/IY
		{ [this] () { m_sp.w.l = m_hl_index[m_hl_offset].w.l; m_icount -= 2; } },
		// dd/fd fa, 14 cycles, JP M,nn
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; jp_cond(); }
		},
		// dd/fd fb, 8 cycles, EI
		{ [this] () { ei(); } },
		// dd/fd fc, 14/21 cycles, CALL M,nn
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { m_wz.b.l = m_data_bus; read_s(m_pc.w.l++); },
			[this] () { m_wz.b.h = m_data_bus; if (call_cond()) { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.h); } },
			[this] () { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.l); },
			[this] () { m_pc.w.l = m_wz.w.l; }
		},
		// dd/fd fd, +4 cycles, FD prefix
		{ [] () { } },
		// dd/fd fe, 11 cycles, CP n
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { alu_cp(m_data_bus); }
		},
		// dd/fd ff, 15 cycles, RST 38H
		{
			[this] () { m_icount -= 1; m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.h); },
			[this] () { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.l); },
			[this] () { rst(); }
		},

		/*****************************************************/
		/* DD/FD + CB prefixed instructions                  */
		/*****************************************************/

		// dd/fd cb dd 00, 23 cycles, RLC (IX/IY+dd),B
		// 17 T1 AB:5678 DB:--
		// 18 T2 AB:5678 DB:xx MREQ RD
		// 19 T3 AB:5678 DB:xx MREQ RD
		// 20 T4 AB:5678 DB:--
		// 21 T1 AB:5678 DB:--
		// 22 T2 AB:5678 DB:yy MREQ
		// 23 T3 AB:5678 DB:yy MREQ WR
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_rlc(m_data_bus); m_icount -= 2; m_bc.b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 01, 23 cycles, RLC (IX/IY+dd),C
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_rlc(m_data_bus); m_icount -= 2; m_bc.b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 02, 23 cycles, RLC (IX/IY+dd),D
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_rlc(m_data_bus); m_icount -= 2; m_de.b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 03, 23 cycles, RLC (IX/IY+dd),E
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_rlc(m_data_bus); m_icount -= 2; m_de.b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 04, 23 cycles, RLC (IX/IY+dd),H
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_rlc(m_data_bus); m_icount -= 2; m_hl_index[HL_OFFSET].b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 05, 23 cycles, RLC (IX/IY+dd),L
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_rlc(m_data_bus); m_icount -= 2; m_hl_index[HL_OFFSET].b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 06, 23 cycles, RLC (IX/IY+dd)
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { m_icount -= 2; write_s(alu_rlc(m_data_bus)); }
		},
		// dd/fd cb dd 07, 23 cycles, RLC (IX/IY+dd),A
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_rlc(m_data_bus); m_icount -= 2; m_af.b.h = tmp; write_s(tmp); }
		},

		// dd/fd cb dd 08, 23 cycles, RRC (IX/IY+dd),B
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_rrc(m_data_bus); m_icount -= 2; m_bc.b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 09, 23 cycles, RRC (IX/IY+dd),C
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_rrc(m_data_bus); m_icount -= 2; m_bc.b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 0a, 23 cycles, RRC (IX/IY+dd),D
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_rrc(m_data_bus); m_icount -= 2; m_de.b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 0b, 23 cycles, RRC (IX/IY+dd),E
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_rrc(m_data_bus); m_icount -= 2; m_de.b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 0c, 23 cycles, RRC (IX/IY+dd),H
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_rrc(m_data_bus); m_icount -= 2; m_hl_index[HL_OFFSET].b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 0d, 23 cycles, RRC (IX/IY+dd),L
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_rrc(m_data_bus); m_icount -= 2; m_hl_index[HL_OFFSET].b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 0e, 23 cycles, RRC (IX/IY+dd)
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { m_icount -= 2; write_s(alu_rrc(m_data_bus)); }
		},
		// dd/fd cb dd 0f, 23 cycles, RRC (IX/IY+dd),A
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_rrc(m_data_bus); m_icount -= 2; m_af.b.h = tmp; write_s(tmp); }
		},

		// dd/fd cb dd 10, 23 cycles, RL (IX/IY+dd),B
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_rl(m_data_bus); m_icount -= 2; m_bc.b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 11, 23 cycles, RL (IX/IY+dd),C
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_rl(m_data_bus); m_icount -= 2; m_bc.b.l = tmp; write_s(tmp); }		
		},
		// dd/fd cb dd 12, 23 cycles, RL (IX/IY+dd),D
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_rl(m_data_bus); m_icount -= 2; m_de.b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 13, 23 cycles, RL (IX/IY+dd),E
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_rl(m_data_bus); m_icount -= 2; m_de.b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 14, 23 cycles, RL (IX/IY+dd),H
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_rl(m_data_bus); m_icount -= 2; m_hl_index[HL_OFFSET].b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 15, 23 cycles, RL (IX/IY+dd),L
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_rl(m_data_bus); m_icount -= 2; m_hl_index[HL_OFFSET].b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 16, 23 cycles, RL (IX/IY+dd)
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { m_icount -= 2; write_s(alu_rl(m_data_bus)); }
		},
		// dd/fd cb dd 17, 23 cycles, RL (IX/IY+dd),A
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_rl(m_data_bus); m_icount -= 2; m_af.b.h = tmp; write_s(tmp); }
		},

		// dd/fd cb dd 18, 23 cycles, RR (IX/IY+dd),B
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_rr(m_data_bus); m_icount -= 2; m_bc.b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 19, 23 cycles, RR (IX/IY+dd),C
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_rr(m_data_bus); m_icount -= 2; m_bc.b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 1a, 23 cycles, RR (IX/IY+dd),D
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_rr(m_data_bus); m_icount -= 2; m_de.b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 1b, 23 cycles, RR (IX/IY+dd),E
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_rr(m_data_bus); m_icount -= 2; m_de.b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 1c, 23 cycles, RR (IX/IY+dd),H
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_rr(m_data_bus); m_icount -= 2; m_hl_index[HL_OFFSET].b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 1d, 23 cycles, RR (IX/IY+dd),L
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_rr(m_data_bus); m_icount -= 2; m_hl_index[HL_OFFSET].b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 1e, 23 cycles, RR (IX/IY+dd)
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { m_icount -= 2; write_s(alu_rr(m_data_bus)); }
		},
		// dd/fd cb dd 1f, 23 cycles, RR (IX/IY+dd),A
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_rr(m_data_bus); m_icount -= 2; m_af.b.h = tmp; write_s(tmp); }
		},

		// dd/fd cb dd 20, 23 cycles, SLA (IX/IY+dd),B
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_sla(m_data_bus); m_icount -= 2; m_bc.b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 21, 23 cycles, SLA (IX/IY+dd),C
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_sla(m_data_bus); m_icount -= 2; m_bc.b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 22, 23 cycles, SLA (IX/IY+dd),D
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_sla(m_data_bus); m_icount -= 2; m_de.b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 23, 23 cycles, SLA (IX/IY+dd),E
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_sla(m_data_bus); m_icount -= 2; m_de.b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 24, 23 cycles, SLA (IX/IY+dd),H
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_sla(m_data_bus); m_icount -= 2; m_hl_index[HL_OFFSET].b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 25, 23 cycles, SLA (IX/IY+dd),L
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_sla(m_data_bus); m_icount -= 2; m_hl_index[HL_OFFSET].b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 26, 23 cycles, SLA (IX/IY+dd)
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { m_icount -= 2; write_s(alu_sla(m_data_bus)); }
		},
		// dd/fd cb dd 27, 23 cycles, SLA (IX/IY+dd),A
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_sla(m_data_bus); m_icount -= 2; m_af.b.h = tmp; write_s(tmp); }
		},

		// dd/fd cb dd 28, 23 cycles, SRA (IX/IY+dd),B
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_sra(m_data_bus); m_icount -= 2; m_bc.b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 29, 23 cycles, SRA (IX/IY+dd),C
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_sra(m_data_bus); m_icount -= 2; m_bc.b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 2a, 23 cycles, SRA (IX/IY+dd),D
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_sra(m_data_bus); m_icount -= 2; m_de.b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 2b, 23 cycles, SRA (IX/IY+dd),E
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_sra(m_data_bus); m_icount -= 2; m_de.b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 2c, 23 cycles, SRA (IX/IY+dd),H
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_sra(m_data_bus); m_icount -= 2; m_hl_index[HL_OFFSET].b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 2d, 23 cycles, SRA (IX/IY+dd),L
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_sra(m_data_bus); m_icount -= 2; m_hl_index[HL_OFFSET].b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 2e, 23 cycles, SRA (IX/IY+dd)
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { m_icount -= 2; write_s(alu_sra(m_data_bus)); }
		},
		// dd/fd cb dd 2f, 23 cycles, SRA (IX/IY+dd),A
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_sra(m_data_bus); m_icount -= 2; m_af.b.h = tmp; write_s(tmp); }
		},

		// dd/fd cb dd 30, 23 cycles, SLL (IX/IY+dd),B
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_sll(m_data_bus); m_icount -= 2; m_bc.b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 31, 23 cycles, SLL (IX/IY+dd),C
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_sll(m_data_bus); m_icount -= 2; m_bc.b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 32, 23 cycles, SLL (IX/IY+dd),D
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_sll(m_data_bus); m_icount -= 2; m_de.b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 33, 23 cycles, SLL (IX/IY+dd),E
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_sll(m_data_bus); m_icount -= 2; m_de.b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 34, 23 cycles, SLL (IX/IY+dd),H
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_sll(m_data_bus); m_icount -= 2; m_hl_index[HL_OFFSET].b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 35, 23 cycles, SLL (IX/IY+dd),L
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_sll(m_data_bus); m_icount -= 2; m_hl_index[HL_OFFSET].b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 36, 23 cycles, SLL (IX/IY+dd)
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { m_icount -= 2; write_s(alu_sll(m_data_bus)); }
		},
		// dd/fd cb dd 37, 23 cycles, SLL (IX/IY+dd),A
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_sll(m_data_bus); m_icount -= 2; m_af.b.h = tmp; write_s(tmp); }
		},

		// dd/fd cb dd 38, 23 cycles, SRL (IX/IY+dd),B
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_srl(m_data_bus); m_icount -= 2; m_bc.b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 39, 23 cycles, SRL (IX/IY+dd),C
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_srl(m_data_bus); m_icount -= 2; m_bc.b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 3a, 23 cycles, SRL (IX/IY+dd),D
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_srl(m_data_bus); m_icount -= 2; m_de.b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 3b, 23 cycles, SRL (IX/IY+dd),E
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_srl(m_data_bus); m_icount -= 2; m_de.b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 3c, 23 cycles, SRL (IX/IY+dd),H
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_srl(m_data_bus); m_icount -= 2; m_hl_index[HL_OFFSET].b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 3d, 23 cycles, SRL (IX/IY+dd),L
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_srl(m_data_bus); m_icount -= 2; m_hl_index[HL_OFFSET].b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 3e, 23 cycles, SRL (IX/IY+dd)
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { m_icount -= 2; write_s(alu_srl(m_data_bus)); }
		},
		// dd/fd cb dd 3f, 23 cycles, SRL (IX/IY+dd),A
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_srl(m_data_bus); m_icount -= 2; m_af.b.h = tmp; write_s(tmp); }
		},

		// dd/fd cb dd 40, 20 cycles, BIT 0,(IX/IY+dd)*
		// 17 T1 AB:5678 DB:--
		// 18 T2 AB:5678 DB:xx MREQ RD
		// 19 T3 AB:5678 DB:xx MREQ RD
		// 20 T4 AB:5678 DB:--
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 41, 20 cycles, BIT 0,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 42, 20 cycles, BIT 0,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 43, 20 cycles, BIT 0,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 44, 20 cycles, BIT 0,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 45, 20 cycles, BIT 0,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 46, 20 cycles, BIT 0,(IX/IY+dd)
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 47, 20 cycles, BIT 0,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},

		// dd/fd cb dd 48, 20 cycles, BIT 1,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 49, 20 cycles, BIT 1,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 4a, 20 cycles, BIT 1,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 4b, 20 cycles, BIT 1,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 4c, 20 cycles, BIT 1,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 4d, 20 cycles, BIT 1,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 4e, 20 cycles, BIT 1,(IX/IY+dd)
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 4f, 20 cycles, BIT 1,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},

		// dd/fd cb dd 50, 20 cycles, BIT 2,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 51, 20 cycles, BIT 2,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 52, 20 cycles, BIT 2,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 53, 20 cycles, BIT 2,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 54, 20 cycles, BIT 2,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 55, 20 cycles, BIT 2,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 56, 20 cycles, BIT 2,(IX/IY+dd)
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 57, 20 cycles, BIT 2,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},

		// dd/fd cb dd 58, 20 cycles, BIT 3,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 59, 20 cycles, BIT 3,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 5a, 20 cycles, BIT 3,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 5b, 20 cycles, BIT 3,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 5c, 20 cycles, BIT 3,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 5d, 20 cycles, BIT 3,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 5e, 20 cycles, BIT 3,(IX/IY+dd)
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 5f, 20 cycles, BIT 3,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},

		// dd/fd cb dd 60, 20 cycles, BIT 4,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 61, 20 cycles, BIT 4,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 62, 20 cycles, BIT 4,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 63, 20 cycles, BIT 4,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 64, 20 cycles, BIT 4,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 65, 20 cycles, BIT 4,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 66, 20 cycles, BIT 4,(IX/IY+dd)
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 67, 20 cycles, BIT 4,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},

		// dd/fd cb dd 68, 20 cycles, BIT 5,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 69, 20 cycles, BIT 5,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 6a, 20 cycles, BIT 5,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 6b, 20 cycles, BIT 5,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 6c, 20 cycles, BIT 5,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 6d, 20 cycles, BIT 5,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 6e, 20 cycles, BIT 5,(IX/IY+dd)
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 6f, 20 cycles, BIT 5,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},

		// dd/fd cb dd 70, 20 cycles, BIT 6,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 71, 20 cycles, BIT 6,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 72, 20 cycles, BIT 6,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 73, 20 cycles, BIT 6,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 74, 20 cycles, BIT 6,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 75, 20 cycles, BIT 6,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 76, 20 cycles, BIT 6,(IX/IY+dd)
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 77, 20 cycles, BIT 6,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},

		// dd/fd cb dd 78, 20 cycles, BIT 7,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 79, 20 cycles, BIT 7,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 7a, 20 cycles, BIT 7,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 7b, 20 cycles, BIT 7,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 7c, 20 cycles, BIT 7,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 7d, 20 cycles, BIT 7,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 7e, 20 cycles, BIT 7,(IX/IY+dd)
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},
		// dd/fd cb dd 7f, 20 cycles, BIT 7,(IX/IY+dd)*
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { alu_bit(m_data_bus); m_icount -= 1; }
		},

		// dd/fd cb dd 80, 23 cycles, RES 0,(IX/IY+dd),B
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_bc.b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 81, 23 cycles, RES 0,(IX/IY+dd),C
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_bc.b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 82, 23 cycles, RES 0,(IX/IY+dd),D
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_de.b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 83, 23 cycles, RES 0,(IX/IY+dd),E
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_de.b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 84, 23 cycles, RES 0,(IX/IY+dd),H
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_hl_index[HL_OFFSET].b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 85, 23 cycles, RES 0,(IX/IY+dd),L
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_hl_index[HL_OFFSET].b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 86, 23 cycles, RES 0,(IX/IY+dd)
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { m_icount -= 2; write_s(alu_res(m_data_bus)); }
		},
		// dd/fd cb dd 87, 23 cycles, RES 0,(IX/IY+dd),A
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_af.b.h = tmp; write_s(tmp); }
		},

		// dd/fd cb dd 88, 23 cycles, RES 1,(IX/IY+dd),B
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_bc.b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 89, 23 cycles, RES 1,(IX/IY+dd),C
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_bc.b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 8a, 23 cycles, RES 1,(IX/IY+dd),D
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_de.b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 8b, 23 cycles, RES 1,(IX/IY+dd),E
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_de.b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 8c, 23 cycles, RES 1,(IX/IY+dd),H
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_hl_index[HL_OFFSET].b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 8d, 23 cycles, RES 1,(IX/IY+dd),L
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_hl_index[HL_OFFSET].b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 8e, 23 cycles, RES 1,(IX/IY+dd)
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { m_icount -= 2; write_s(alu_res(m_data_bus)); }
		},
		// dd/fd cb dd 8f, 23 cycles, RES 1,(IX/IY+dd),A
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_af.b.h = tmp; write_s(tmp); }
		},

		// dd/fd cb dd 90, 23 cycles, RES 2,(IX/IY+dd),B
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_bc.b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 91, 23 cycles, RES 2,(IX/IY+dd),C
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_bc.b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 92, 23 cycles, RES 2,(IX/IY+dd),D
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_de.b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 93, 23 cycles, RES 2,(IX/IY+dd),E
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_de.b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 94, 23 cycles, RES 2,(IX/IY+dd),H
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_hl_index[HL_OFFSET].b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 95, 23 cycles, RES 2,(IX/IY+dd),L
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_hl_index[HL_OFFSET].b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 96, 23 cycles, RES 2,(IX/IY+dd)
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { m_icount -= 2; write_s(alu_res(m_data_bus)); }
		},
		// dd/fd cb dd 97, 23 cycles, RES 2,(IX/IY+dd),A
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_af.b.h = tmp; write_s(tmp); }
		},

		// dd/fd cb dd 98, 23 cycles, RES 3,(IX/IY+dd),B
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_bc.b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 99, 23 cycles, RES 3,(IX/IY+dd),C
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_bc.b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 9a, 23 cycles, RES 3,(IX/IY+dd),D
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_de.b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 9b, 23 cycles, RES 3,(IX/IY+dd),E
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_de.b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 9c, 23 cycles, RES 3,(IX/IY+dd),H
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_hl_index[HL_OFFSET].b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 9d, 23 cycles, RES 3,(IX/IY+dd),L
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_hl_index[HL_OFFSET].b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd 9e, 23 cycles, RES 3,(IX/IY+dd)
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { m_icount -= 2; write_s(alu_res(m_data_bus)); }
		},
		// dd/fd cb dd 9f, 23 cycles, RES 3,(IX/IY+dd),A
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_af.b.h = tmp; write_s(tmp); }
		},

		// dd/fd cb dd a0, 23 cycles, RES 4,(IX/IY+dd),B
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_bc.b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd a1, 23 cycles, RES 4,(IX/IY+dd),C
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_bc.b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd a2, 23 cycles, RES 4,(IX/IY+dd),D
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_de.b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd a3, 23 cycles, RES 4,(IX/IY+dd),E
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_de.b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd a4, 23 cycles, RES 4,(IX/IY+dd),H
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_hl_index[HL_OFFSET].b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd a5, 23 cycles, RES 4,(IX/IY+dd),L
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_hl_index[HL_OFFSET].b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd a6, 23 cycles, RES 4,(IX/IY+dd)
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { m_icount -= 2; write_s(alu_res(m_data_bus)); }
		},
		// dd/fd cb dd a7, 23 cycles, RES 4,(IX/IY+dd),A
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_af.b.h = tmp; write_s(tmp); }
		},

		// dd/fd cb dd a8, 23 cycles, RES 5,(IX/IY+dd),B
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_bc.b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd a9, 23 cycles, RES 5,(IX/IY+dd),C
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_bc.b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd aa, 23 cycles, RES 5,(IX/IY+dd),D
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_de.b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd ab, 23 cycles, RES 5,(IX/IY+dd),E
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_de.b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd ac, 23 cycles, RES 5,(IX/IY+dd),H
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_hl_index[HL_OFFSET].b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd ad, 23 cycles, RES 5,(IX/IY+dd),L
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_hl_index[HL_OFFSET].b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd ae, 23 cycles, RES 5,(IX/IY+dd)
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { m_icount -= 2; write_s(alu_res(m_data_bus)); }
		},
		// dd/fd cb dd af, 23 cycles, RES 5,(IX/IY+dd),A
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_af.b.h = tmp; write_s(tmp); }
		},

		// dd/fd cb dd b0, 23 cycles, RES 6,(IX/IY+dd),B
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_bc.b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd b1, 23 cycles, RES 6,(IX/IY+dd),C
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_bc.b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd b2, 23 cycles, RES 6,(IX/IY+dd),D
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_de.b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd b3, 23 cycles, RES 6,(IX/IY+dd),E
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_de.b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd b4, 23 cycles, RES 6,(IX/IY+dd),H
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_hl_index[HL_OFFSET].b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd b5, 23 cycles, RES 6,(IX/IY+dd),L
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_hl_index[HL_OFFSET].b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd b6, 23 cycles, RES 6,(IX/IY+dd)
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { m_icount -= 2; write_s(alu_res(m_data_bus)); }
		},
		// dd/fd cb dd b7, 23 cycles, RES 6,(IX/IY+dd),A
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_af.b.h = tmp; write_s(tmp); }
		},

		// dd/fd cb dd b8, 23 cycles, RES 7,(IX/IY+dd),B
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_bc.b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd b9, 23 cycles, RES 7,(IX/IY+dd),C
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_bc.b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd ba, 23 cycles, RES 7,(IX/IY+dd),D
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_de.b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd bb, 23 cycles, RES 7,(IX/IY+dd),E
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_de.b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd bc, 23 cycles, RES 7,(IX/IY+dd),H
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_hl_index[HL_OFFSET].b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd bd, 23 cycles, RES 7,(IX/IY+dd),L
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_hl_index[HL_OFFSET].b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd be, 23 cycles, RES 7,(IX/IY+dd)
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { m_icount -= 2; write_s(alu_res(m_data_bus)); }
		},
		// dd/fd cb dd bf, 23 cycles, RES 7,(IX/IY+dd),A
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_res(m_data_bus); m_icount -= 2; m_af.b.h = tmp; write_s(tmp); }
		},

		// dd/fd cb dd c0, 23 cycles, SET 0,(IX/IY+dd),B
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_bc.b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd c1, 23 cycles, SET 0,(IX/IY+dd),C
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_bc.b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd c2, 23 cycles, SET 0,(IX/IY+dd),D
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_de.b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd c3, 23 cycles, SET 0,(IX/IY+dd),E
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_de.b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd c4, 23 cycles, SET 0,(IX/IY+dd),H
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_hl_index[HL_OFFSET].b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd c5, 23 cycles, SET 0,(IX/IY+dd),L
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_hl_index[HL_OFFSET].b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd c6, 23 cycles, SET 0,(IX/IY+dd)
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { m_icount -= 2; write_s(alu_set(m_data_bus)); }
		},
		// dd/fd cb dd c7, 23 cycles, SET 0,(IX/IY+dd),A
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_af.b.h = tmp; write_s(tmp); }
		},

		// dd/fd cb dd c8, 23 cycles, SET 1,(IX/IY+dd),B
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_bc.b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd c9, 23 cycles, SET 1,(IX/IY+dd),C
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_bc.b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd ca, 23 cycles, SET 1,(IX/IY+dd),D
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_de.b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd cb, 23 cycles, SET 1,(IX/IY+dd),E
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_de.b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd cc, 23 cycles, SET 1,(IX/IY+dd),H
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_hl_index[HL_OFFSET].b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd cd, 23 cycles, SET 1,(IX/IY+dd),L
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_hl_index[HL_OFFSET].b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd ce, 23 cycles, SET 1,(IX/IY+dd)
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { m_icount -= 2; write_s(alu_set(m_data_bus)); }
		},
		// dd/fd cb dd cf, 23 cycles, SET 1,(IX/IY+dd),A
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_af.b.h = tmp; write_s(tmp); }
		},

		// dd/fd cb dd d0, 23 cycles, SET 2,(IX/IY+dd),B
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_bc.b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd d1, 23 cycles, SET 2,(IX/IY+dd),C
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_bc.b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd d2, 23 cycles, SET 2,(IX/IY+dd),D
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_de.b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd d3, 23 cycles, SET 2,(IX/IY+dd),E
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_de.b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd d4, 23 cycles, SET 2,(IX/IY+dd),H
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_hl_index[HL_OFFSET].b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd d5, 23 cycles, SET 2,(IX/IY+dd),L
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_hl_index[HL_OFFSET].b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd d6, 23 cycles, SET 2,(IX/IY+dd)
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { m_icount -= 2; write_s(alu_set(m_data_bus)); }
		},
		// dd/fd cb dd d7, 23 cycles, SET 2,(IX/IY+dd),A
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_af.b.h = tmp; write_s(tmp); }
		},

		// dd/fd cb dd d8, 23 cycles, SET 3,(IX/IY+dd),B
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_bc.b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd d9, 23 cycles, SET 3,(IX/IY+dd),C
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_bc.b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd da, 23 cycles, SET 3,(IX/IY+dd),D
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_de.b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd db, 23 cycles, SET 3,(IX/IY+dd),E
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_de.b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd dc, 23 cycles, SET 3,(IX/IY+dd),H
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_hl_index[HL_OFFSET].b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd dd, 23 cycles, SET 3,(IX/IY+dd),L
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_hl_index[HL_OFFSET].b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd de, 23 cycles, SET 3,(IX/IY+dd)
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { m_icount -= 2; write_s(alu_set(m_data_bus)); }
		},
		// dd/fd cb dd df, 23 cycles, SET 3,(IX/IY+dd),A
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_af.b.h = tmp; write_s(tmp); }
		},

		// dd/fd cb dd e0, 23 cycles, SET 4,(IX/IY+dd),B
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_bc.b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd e1, 23 cycles, SET 4,(IX/IY+dd),C
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_bc.b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd e2, 23 cycles, SET 4,(IX/IY+dd),D
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_de.b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd e3, 23 cycles, SET 4,(IX/IY+dd),E
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_de.b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd e4, 23 cycles, SET 4,(IX/IY+dd),H
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_hl_index[HL_OFFSET].b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd e5, 23 cycles, SET 4,(IX/IY+dd),L
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_hl_index[HL_OFFSET].b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd e6, 23 cycles, SET 4,(IX/IY+dd)
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { m_icount -= 2; write_s(alu_set(m_data_bus)); }
		},
		// dd/fd cb dd e7, 23 cycles, SET 4,(IX/IY+dd),A
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_af.b.h = tmp; write_s(tmp); }
		},

		// dd/fd cb dd e8, 23 cycles, SET 5,(IX/IY+dd),B
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_bc.b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd e9, 23 cycles, SET 5,(IX/IY+dd),C
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_bc.b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd ea, 23 cycles, SET 5,(IX/IY+dd),D
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_de.b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd eb, 23 cycles, SET 5,(IX/IY+dd),E
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_de.b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd ec, 23 cycles, SET 5,(IX/IY+dd),H
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_hl_index[HL_OFFSET].b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd ed, 23 cycles, SET 5,(IX/IY+dd),L
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_hl_index[HL_OFFSET].b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd ee, 23 cycles, SET 5,(IX/IY+dd)
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { m_icount -= 2; write_s(alu_set(m_data_bus)); }
		},
		// dd/fd cb dd ef, 23 cycles, SET 5,(IX/IY+dd),A
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_af.b.h = tmp; write_s(tmp); }
		},

		// dd/fd cb dd f0, 23 cycles, SET 6,(IX/IY+dd),B
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_bc.b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd f1, 23 cycles, SET 6,(IX/IY+dd),C
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_bc.b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd f2, 23 cycles, SET 6,(IX/IY+dd),D
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_de.b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd f3, 23 cycles, SET 6,(IX/IY+dd),E
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_de.b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd f4, 23 cycles, SET 6,(IX/IY+dd),H
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_hl_index[HL_OFFSET].b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd f5, 23 cycles, SET 6,(IX/IY+dd),L
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_hl_index[HL_OFFSET].b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd f6, 23 cycles, SET 6,(IX/IY+dd)
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { m_icount -= 2; write_s(alu_set(m_data_bus)); }
		},
		// dd/fd cb dd f7, 23 cycles, SET 6,(IX/IY+dd),A
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_af.b.h = tmp; write_s(tmp); }
		},

		// dd/fd cb dd f8, 23 cycles, SET 7,(IX/IY+dd),B
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_bc.b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd f9, 23 cycles, SET 7,(IX/IY+dd),C
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_bc.b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd fa, 23 cycles, SET 7,(IX/IY+dd),D
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_de.b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd fb, 23 cycles, SET 7,(IX/IY+dd),E
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_de.b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd fc, 23 cycles, SET 7,(IX/IY+dd),H
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_hl_index[HL_OFFSET].b.h = tmp; write_s(tmp); }
		},
		// dd/fd cb dd fd, 23 cycles, SET 7,(IX/IY+dd),L
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_hl_index[HL_OFFSET].b.l = tmp; write_s(tmp); }
		},
		// dd/fd cb dd fe, 23 cycles, SET 7,(IX/IY+dd)
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { m_icount -= 2; write_s(alu_set(m_data_bus)); }
		},
		// dd/fd cb dd ff, 23 cycles, SET 7,(IX/IY+dd),A
		{
			[this] () { read_s(m_wz.w.l); },
			[this] () { u8 tmp = alu_set(m_data_bus); m_icount -= 2; m_af.b.h = tmp; write_s(tmp); }
		},

		/*****************************************************/
		/* Special sequences                                 */
		/*****************************************************/

		// CB/ED/DD/FD prefixed instructions have 2 M1 cycles taking an initial total of 8 cycles
		/* M1, 4 cycles */
		// 1 T1 AB:1234 DB:-- M1
		// 2 T2 AB:1234 DB:YY M1      MREQ RD
		// 3 T3 AB:1234 DB:--    RFSH
		// 4 T4 AB:1234 DB:--    RFSH MREQ
		{
			[this] () { read_op1_s(); },
			[this] () { refresh_decode(); }
		},
		/* DD/FD CB, 8 cycles, read displacement and next opcode */
		//  9 T1 AB:1236 DB:--
		// 10 T2 AB:1236 DB:dd
		// 11 T3 AB:1236 DB:dd
		// 12 T1 AB:1237 DB:--
		// 13 T2 AB:1237 DB:op
		// 14 T3 AB:1237 DB:op
		// 15 T4 AB:1237 DB:--
		// 16 T5 AB:1237 DB:--
		{
			[this] () { read_s(m_pc.w.l++); },
			[this] () { db_tmp(); read_op2_s(m_pc.w.l); m_pc.w.l++; },
			[this] () { disp_wz2(); decode(); }
		},
		/* Take IRQ, 6 cycles, Taking IRQ */
		{
			[this] () { read_op_irq(); },
			[this] () { refresh_decode(); }
		},
		/* Take NMI, 11 cycles, opcode is read but ignored, the PC_OUT also asserts M1 */
		// TODO: M1 signal
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
		{
			[this] () { pc_out_m1(); read_op_s(); },
			[this] () { refresh(); m_icount -= 1; m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.h); },
			[this] () { m_sp.w.l -= 1; write_s(m_sp.w.l, m_pc.b.l); },
			[this] () { m_pc.w.l = 0x66; }
		},

	};

}


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


inline void z80lle_device::a_db()
{
	m_data_bus = m_af.b.h; m_wz.b.h = m_data_bus;
}


inline void z80lle_device::a_w()
{
	m_wz.b.h = m_af.b.h;
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


inline void z80lle_device::alu_adc(u8 arg2)
{
	m_alu = m_af.b.h + arg2 + (m_af.b.l & CF);
	m_af.b.l = SZHVC_add[((m_af.b.l & CF) << 16) | (m_af.b.h << 8) | m_alu];
	m_af.b.h = m_alu;
}


inline void z80lle_device::alu_add(u8 arg2)
{
	m_alu = m_af.b.h + arg2;
	m_af.b.l = SZHVC_add[(m_af.b.h << 8) | m_alu];
	m_af.b.h = m_alu;
}


inline void z80lle_device::alu_and(u8 arg2)
{
	m_alu = m_af.b.h & arg2;
	m_af.b.l = SZP[m_alu] | HF;
	m_af.b.h = m_alu;
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


inline void z80lle_device::alu_cp(u8 arg2)
{
	// Flag handling is slightly different from SUB
	m_alu = m_af.b.h - arg2;
	m_af.b.l = (SZHVC_sub[(m_af.b.h << 8) | m_alu] & ~(YF | XF)) | (arg2 & (YF | XF));
}


inline u8 z80lle_device::alu_dec(u8 arg)
{
	m_alu = arg - 1;
	m_af.b.l = (m_af.b.l & CF) | SZHV_dec[m_alu];
	return m_alu;
}


inline u8 z80lle_device::alu_inc(u8 arg)
{
	m_alu = arg + 1;
	m_af.b.l = (m_af.b.l & CF) | SZHV_inc[m_alu];
	return m_alu;
}


inline void z80lle_device::alu_or(u8 arg2)
{
	m_alu = m_af.b.h | arg2;
	m_af.b.l = SZP[m_alu];
	m_af.b.h = m_alu;
}


inline u8 z80lle_device::alu_res(u8 arg)
{
	m_alu = arg & ~(1 << ((m_ir >> 3) & 0x07));
	return m_alu;
}


inline u8 z80lle_device::alu_rl(u8 arg)
{
	m_alu = (arg << 1) | (m_af.b.l & CF);
	m_af.b.l = SZP[m_alu] | ((arg & 0x80) ? CF : 0);
	return m_alu;
}


inline u8 z80lle_device::alu_rlc(u8 arg)
{
	m_alu = (arg << 1) | (arg >> 7);
	m_af.b.l = SZP[m_alu] | ((arg & 0x80) ? CF : 0);
	return m_alu;
}


inline u8 z80lle_device::alu_rr(u8 arg)
{
	m_alu = (arg >> 1) | (m_af.b.l << 7);
	m_af.b.l = SZP[m_alu] | ((arg & 0x01) ? CF : 0);
	return m_alu;
}


inline u8 z80lle_device::alu_rrc(u8 arg)
{
	m_alu = (arg >> 1) | (arg << 7);
	m_af.b.l = SZP[m_alu] | ((arg & 0x01) ? CF : 0);
	return m_alu;
}


inline void z80lle_device::alu_sbc(u8 arg2)
{
	m_alu = m_af.b.h - arg2 - (m_af.b.l & CF);
	m_af.b.l = SZHVC_sub[((m_af.b.l & CF) << 16) | (m_af.b.h << 8) | m_alu];
	m_af.b.h = m_alu;
}


inline u8 z80lle_device::alu_set(u8 arg)
{
	m_alu = arg | (1 << ((m_ir >> 3) & 0x07));
	return m_alu;
}


inline u8 z80lle_device::alu_sla(u8 arg)
{
	m_alu = arg << 1;
	m_af.b.l = SZP[m_alu] | ((arg & 0x80) ? CF : 0);
	return m_alu;
}


inline u8 z80lle_device::alu_sll(u8 arg)
{
	m_alu = (arg << 1) | 0x01;
	m_af.b.l = SZP[m_alu] | ((arg & 0x80) ? CF : 0);
	return m_alu;
}


inline u8 z80lle_device::alu_sra(u8 arg)
{
	m_alu = (arg >> 1) | (arg & 0x80);
	m_af.b.l = SZP[m_alu] | ((arg & 0x01) ? CF : 0);
	return m_alu;
}


inline u8 z80lle_device::alu_srl(u8 arg)
{
	m_alu = arg >> 1;
	m_af.b.l = SZP[m_alu] | ((arg & 0x01) ? CF : 0);
	return m_alu;
}


inline void z80lle_device::alu_sub(u8 arg2)
{
	m_alu = m_af.b.h - arg2;
	m_af.b.l = SZHVC_sub[(m_af.b.h << 8) | m_alu];
	m_af.b.h = m_alu;
}


inline void z80lle_device::alu_xor(u8 arg2)
{
	m_alu = m_af.b.h ^ arg2;
	m_af.b.l = SZP[m_alu];
	m_af.b.h = m_alu;
}


inline bool z80lle_device::call_cond()
{
	if ((m_af.b.l & jp_conditions[((m_ir >> 3) & 0x07)][0]) == jp_conditions[((m_ir >> 3) & 0x07)][1])
	{
		m_icount -= 1;
		return true;
	}
	else
	{
		end_instruction();
		return false;
	}
}


inline void z80lle_device::ccf()
{
	m_af.b.l = ((m_af.b.l & (SF | ZF | YF | XF | PF | CF)) | ((m_af.b.l & CF) << 4) | (m_af.b.h & (YF | XF))) ^ CF;
}


inline void z80lle_device::cpd()
{
	m_alu = m_af.b.h - m_data_bus;
	m_wz.w.l--;
	m_hl_index[m_hl_offset].w.l--;
	m_bc.w.l--;
	m_af.b.l = (m_af.b.l & CF) | (SZ[m_alu] & ~(YF | XF)) | ((m_af.b.h ^ m_data_bus ^ m_alu) & HF) | NF;
	if (m_af.b.l & HF)
		m_alu -= 1;
	if (m_alu & 0x02)
		m_af.b.l |= YF; /* bit 1 -> flag 5 */
	if (m_alu & 0x08)
		m_af.b.l |= XF; /* bit 3 -> flag 3 */
	if (m_bc.w.l)
		m_af.b.l |= VF;
}


inline void z80lle_device::cpi()
{
	m_alu = m_af.b.h - m_data_bus;
	m_wz.w.l++;
	m_hl_index[m_hl_offset].w.l++;
	m_bc.w.l--;
	m_af.b.l = (m_af.b.l & CF) | (SZ[m_alu] & ~(YF | XF)) | ((m_af.b.h ^ m_data_bus ^ m_alu) & HF) | NF;
	if (m_af.b.l & HF)
		m_alu -= 1;
	if (m_alu & 0x02)
		m_af.b.l |= YF; /* bit 1 -> flag 5 */
	if (m_alu & 0x08)
		m_af.b.l |= XF; /* bit 3 -> flag 3 */
	if (m_bc.w.l)
		m_af.b.l |= VF;
}


inline void z80lle_device::cpl()
{
	m_af.b.h ^= 0xff;
	m_af.b.l = (m_af.b.l & (SF | ZF | PF | CF)) | HF | NF | (m_af.b.h & (YF | XF));
}


inline void z80lle_device::daa()
{
	m_alu = m_af.b.h;
	if (m_af.b.l & NF)
	{
		if ((m_af.b.l & HF) | ((m_af.b.h & 0xf) > 9))
			m_alu -= 6;
		if ((m_af.b.l & CF) | (m_af.b.h > 0x99))
			m_alu -= 0x60;
	}
	else
	{
		if ((m_af.b.l & HF) | ((m_af.b.h & 0xf) > 9))
			m_alu += 6;
		if ((m_af.b.l & CF) | (m_af.b.h > 0x99))
			m_alu += 0x60;
	}
	m_af.b.l = (m_af.b.l & (CF | NF)) | (m_af.b.h > 0x99) | ((m_af.b.h ^ m_alu) & HF) | SZP[m_alu];
	m_af.b.h = m_alu;
}


inline void z80lle_device::db_tmp()
{
	m_tmp = m_data_bus;
}


inline void z80lle_device::di()
{
	m_iff1 = m_iff2 = 0;
}


inline void z80lle_device::disp_wz2()
{
	m_wz.w.l = m_hl_index[m_hl_offset].w.l + (s8) m_tmp;
	m_icount -= 2;
}


inline void z80lle_device::disp_wz5()
{
	db_tmp();
	m_wz.w.l = m_hl_index[m_hl_offset].w.l + (s8) m_tmp;
	m_icount -= 5;
}


inline void z80lle_device::ei()
{
	m_iff1 = m_iff2 = 1;
	m_after_ei = true;
}


inline void z80lle_device::ex_af_af()
{
	PAIR tmp = m_af;
	m_af = m_af2;
	m_af2 = tmp;
}


inline void z80lle_device::ex_de_hl()
{
	u16 tmp = m_de.w.l;
	m_de.w.l = m_hl_index[m_hl_offset].w.l;
	m_hl_index[m_hl_offset].w.l = tmp;
}


inline void z80lle_device::exx()
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


inline void z80lle_device::jp_cond()
{
	if ((m_af.b.l & jp_conditions[((m_ir >> 3) & 0x07)][0]) == jp_conditions[((m_ir >> 3) & 0x07)][1])
	{
		m_pc.w.l = m_wz.w.l;
	}
}


inline void z80lle_device::jr_cond()
{
	if ((m_af.b.l & jr_conditions[((m_ir >> 3) & 0x07)][0]) == jr_conditions[((m_ir >> 3) & 0x07)][1])
	{
		m_wz.w.l = m_pc.w.l + (s8) m_data_bus;
		m_pc.w.l = m_wz.w.l;
		m_icount -= 5;
	}
}


inline void z80lle_device::djnz()
{
	m_bc.b.h--;
	if (m_bc.b.h)
	{
		m_wz.w.l = m_pc.w.l + (s8) m_data_bus;
		m_pc.w.l = m_wz.w.l;
		m_icount -= 5;
	}
}


inline void z80lle_device::halt()
{
	m_pc.w.l--;
	if (!m_halt)
	{
		m_halt = 1;
		m_halt_cb(1);
	}
}


inline void z80lle_device::im()
{
	m_im = (m_ir >> 3) & 0x03;
	if (m_im)
		m_im--;
}


inline void z80lle_device::ind()
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


inline void z80lle_device::ini()
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


inline void z80lle_device::input_a()
{
	// TODO: Flags?
	m_af.b.h = m_data_bus;
}


void z80lle_device::input_s(u16 address)
{
	m_address_bus = address;
	m_address_bus_cb(m_address_bus);
	m_icount -= 1;
	set_iorq();
	set_rd();
	m_check_wait = true;
	m_icount -= 3;
}


inline void z80lle_device::ld_a_i()
{
	m_af.b.h = m_i;
	m_af.b.l = (m_af.b.l & CF) | SZ[m_af.b.h] | (m_iff2 << 2);
	m_after_ldair = true;
	m_icount -= 1;
}


inline void z80lle_device::ld_a_r()
{
	m_af.b.h = (m_r & 0x7f) | m_r2;
	m_af.b.l = (m_af.b.l & CF) | SZ[m_af.b.h] | (m_iff2 << 2);
	m_after_ldair = true;
	m_icount -= 1;
}


inline void z80lle_device::ld_i_a()
{
	m_i = m_af.b.h;
	m_icount -= 1;
}


inline void z80lle_device::ld_r_a()
{
	m_r = m_af.b.h;
	m_r2 = m_af.b.h & 0x80;
	m_icount -= 1;
}


inline void z80lle_device::ldd()
{
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
}


inline void z80lle_device::ldi()
{
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
}


inline void z80lle_device::neg()
{
	m_alu = 0 - m_af.b.h;
	m_af.b.l = SZHVC_sub[m_alu];
	m_af.b.h = m_alu;
}


inline void z80lle_device::outd()
{
	m_bc.b.h--;
	m_address_bus = m_bc.w.l;
	m_address_bus_cb(m_address_bus);
	m_wz.w.l = m_bc.w.l - 1;
	m_hl_index[m_hl_offset].w.l--;
	m_af.b.l = SZ[m_bc.b.h];
	u16 t = m_hl_index[m_hl_offset].b.l + m_data_bus;
	if (m_data_bus & SF)
		m_af.b.l |= NF;
	if (t & 0x100)
		m_af.b.l |= HF | CF;
	m_af.b.l |= SZP[(t & 0x07) ^ m_bc.b.h] & PF;
	m_icount -= 1;
}


inline void z80lle_device::outi()
{
	m_bc.b.h--;
	m_address_bus = m_bc.w.l;
	m_address_bus_cb(m_address_bus);
	m_wz.w.l = m_bc.w.l + 1;
	m_hl_index[m_hl_offset].w.l++;
	m_af.b.l = SZ[m_bc.b.h];
	u16 t = m_hl_index[m_hl_offset].b.l + m_data_bus;
	if (m_data_bus & SF)
		m_af.b.l |= NF;
	if (t & 0x100)
		m_af.b.l |= HF | CF;
	m_af.b.l |= SZP[(t & 0x07) ^ m_bc.b.h] & PF;
	m_icount -= 1;
}


void z80lle_device::output_s(u8 data)
{
	set_iorq();
	set_wr();
	m_check_wait = true;
	m_data_bus = data;
	m_icount -= 3;
}


void z80lle_device::output_s(u16 address, u8 data)
{
	m_address_bus = address;
	m_address_bus_cb(m_address_bus);
	m_icount -= 1;
	set_iorq();
	set_wr();
	m_check_wait = true;
	m_data_bus = data;
	m_icount -= 3;
}


inline void z80lle_device::pc_out_m1()
{
	m_address_bus = m_pc.w.l;
	m_address_bus_cb(m_address_bus);
	set_m1();
	m_icount -= 1;
}


inline void z80lle_device::read_op_irq()
{
	// What is put on the address bus when taking IRQ?
	m_icount -= 1;
	// M1 irqack cycle
	switch (m_im)
	{
	case 0:
		// TODO
		break;
	case 1:
		// Interrupt mode 1, RST 38H (0xff)
		m_ir = 0xff;
		break;
	case 2:
		// TODO
		break;
	}
	m_icount -= 1;
	// 2 extra WAIT states
	m_icount -= 2;
	m_check_wait = true;
}


void z80lle_device::read_op_s()
{
	set_mreq();
	set_rd();
	m_icount -= 1;
	m_icount -= m_m1_wait_states;
	m_opcode_read = true;
	m_check_wait = true;
}


void z80lle_device::read_op1_s()
{
	m_address_bus = m_pc.w.l;
	m_address_bus_cb(m_address_bus);
	set_m1();
	m_icount -= 1;
	m_pc.w.l++;
	read_op_s();
}


void z80lle_device::read_op2_s(u16 address)
{
	// This is a regular read but the result ends up in the instruction register (for DDCB / FDCB instructions)
	m_address_bus = address;
	m_address_bus_cb(m_address_bus);
	m_icount -= 1;
	m_opcode_read = true;
	set_mreq();
	set_rd();
	m_icount -= 2;
	m_check_wait = true;
}


void z80lle_device::read_s(u16 address)
{
	m_address_bus = address;
	m_address_bus_cb(m_address_bus);
	m_icount -= 1;
	set_mreq();
	set_rd();
	m_icount -= 2;
	m_check_wait = true;
}


inline void z80lle_device::refresh()
{
	// TODO: Assert RFSH signal
	//set_rfsh();
	m_icount -= 1;
	//set_mreq();
	m_refresh_cb((m_i << 8) | m_r, 0x00, 0xff);
	m_icount -= 1;
	//clear_mreq();
	//clear_rfsh();
	m_r++;
}


inline void z80lle_device::refresh_decode()
{
	// TODO: Assert RFSH signal
	//set_rfsh();
	m_icount -= 1;
	//set_mreq();
	m_refresh_cb((m_i << 8) | m_r, 0x00, 0xff);
	m_icount -= 1;
	//clear_mreq();
	//clear_rfsh();
	m_r++;
	decode();
}


inline void z80lle_device::repeat()
{
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
}


inline void z80lle_device::repeatcp()
{
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
}


inline void z80lle_device::repeatio()
{
	if (m_bc.b.h != 0) {
		m_pc.w.l -= 2;
		m_icount -= 5;
	}
}


inline bool z80lle_device::ret_cond()
{
	if ((m_af.b.l & jp_conditions[((m_ir >> 3) & 0x07)][0]) != jp_conditions[((m_ir >> 3) & 0x07)][1])
	{
		end_instruction();
		return false;
	}
	m_icount -= 1;
	return true;
}


inline void z80lle_device::reti()
{
	m_iff1 = m_iff2;
	daisy_call_reti_device();
}


inline void z80lle_device::retn()
{
	m_iff1 = m_iff2;
}


inline void z80lle_device::rla()
{
	m_alu = (m_af.b.h << 1) | (m_af.b.l & CF);
	m_af.b.l = (m_af.b.l & (SF | ZF | PF)) | ((m_af.b.h & 0x80) ? CF : 0) | (m_alu & (YF | XF));
	m_af.b.h = m_alu;
}


inline void z80lle_device::rlca()
{
	m_af.b.h = (m_af.b.h << 1) | (m_af.b.h >> 7);
	m_af.b.l = (m_af.b.l & (SF | ZF | PF)) | (m_af.b.h & (YF | XF | CF));
}


inline void z80lle_device::rld()
{
	m_alu = (m_data_bus << 4) | (m_af.b.h & 0x0f);
	m_af.b.h = (m_af.b.h & 0xf0) | (m_data_bus >> 4);
	m_af.b.l = (m_af.b.l & CF) | SZP[m_af.b.h];
	m_data_bus = m_alu;
	m_icount -= 5;
}


inline void z80lle_device::rra()
{
	m_alu = (m_af.b.h >> 1) | (m_af.b.l << 7);
	m_af.b.l = (m_af.b.l & (SF | ZF | PF)) | ((m_af.b.h & 0x01) ? CF : 0) | (m_alu & (YF | XF));
	m_af.b.h = m_alu;
}


inline void z80lle_device::rrca()
{
	m_af.b.l = (m_af.b.l & (SF | ZF | PF)) | (m_af.b.h & CF);
	m_af.b.h = (m_af.b.h >> 1) | (m_af.b.h << 7);
	m_af.b.l |= (m_af.b.h & (YF | XF));
}


inline void z80lle_device::rrd()
{
	m_alu = (m_data_bus >> 4) | (m_af.b.h << 4);
	m_af.b.h = (m_af.b.h & 0xf0) | (m_data_bus & 0x0f);
	m_af.b.l = (m_af.b.l & CF) | SZP[m_af.b.h];
	m_data_bus = m_alu;
	m_icount -= 5;
}


inline void z80lle_device::rst()
{
	m_pc.w.l = m_ir & 0x38;
	m_wz.w.l = m_pc.w.l;
}


inline void z80lle_device::scf()
{
	m_af.b.l = (m_af.b.l & (SF | ZF | YF | XF | PF)) | CF | (m_af.b.h & (YF | XF));
}


void z80lle_device::write_s(u8 data)
{
	set_mreq();
	m_icount -= 1;
	set_wr();
	m_icount -= 1;
	m_data_bus = data;
	m_check_wait = true;
}


void z80lle_device::write_s(u16 address, u8 data)
{
	m_address_bus = address;
	m_address_bus_cb(m_address_bus);
	m_icount -= 1;
	set_mreq();
	m_icount -= 1;
	set_wr();
	m_icount -= 1;
	m_data_bus = data;
	m_check_wait = true;
}


inline void z80lle_device::decode()
{
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
		m_irqack_cb(true);

//		// fetch the IRQ vector
//		device_z80daisy_interface *intf = daisy_get_irq_device();
//		int irq_vector = (intf != nullptr) ? intf->z80daisy_irq_ack() : standard_irq_callback_member(*this, 0);

		if (m_im == 2)
		{
			fatalerror("Taking IRQs in mode 2 is not supported yet!\n");
		}
		else if (m_im == 1)
		{
			// Interrupt mode 1. RST 38h
			m_instruction = TAKE_IRQ;
		}
		else
		{
			fatalerror("Taking IRQs in mode 0 is not supported yet!\n");
		}
	}

	m_after_ei = false;
	m_after_ldair = false;
}

/****************************************************************************
 * Processor initialization
 ****************************************************************************/
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
	save_item(NAME(m_tmp));
	save_item(NAME(m_alu));
	save_item(NAME(m_mreq));
	save_item(NAME(m_iorq));
	save_item(NAME(m_rd));
	save_item(NAME(m_wr));
	save_item(NAME(m_m1));
	save_item(NAME(m_opcode_read));

	setup_instructions();

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
	m_after_ei = false;
	m_after_ldair = false;
	m_mreq = false;
	m_iorq = false;
	m_rd = false;
	m_wr = false;
	m_m1 = false;
	m_opcode_read = false;

	m_program = &space(AS_PROGRAM);
	m_opcodes = has_space(AS_OPCODES) ? &space(AS_OPCODES) : m_program;
	m_cache = m_program->cache<0, 0, ENDIANNESS_LITTLE>();
	m_opcodes_cache = m_opcodes->cache<0, 0, ENDIANNESS_LITTLE>();
	m_io = &space(AS_IO);

	m_hl_index[IX_OFFSET].w.l = m_hl_index[IY_OFFSET].w.l = 0xffff; /* IX and IY are FFFF after a reset! */
	m_af.b.l = ZF;           /* Zero flag is set */

	/* set up the state table */
	state_add(STATE_GENPC,     "PC",        m_pc.w.l).callimport();
	state_add(STATE_GENPCBASE, "CURPC",     m_prvpc.w.l).callimport().noshow();
	state_add(Z80LLE_SP,       "SP",        m_sp.w.l);
	state_add(STATE_GENSP,     "GENSP",     m_sp.w.l).noshow();
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
	m_check_wait = false;
	m_opcode_read = false;

	clear_mreq();
	clear_iorq();
	clear_rd();
	clear_wr();
	clear_m1();
}


/****************************************************************************
 * Execute 'cycles' T-states.
 ****************************************************************************/
void z80lle_device::execute_run()
{
	do
	{
		if (m_check_wait) {
			if (!m_wait_state) {
				// Wait for an external source to release the WAIT input
				m_icount = 0;
				return;
			}
			m_check_wait = false;

			// Check if we should perform a memory or io read or write
			if (m_mreq) {
				if (m_rd) {
					if (m_m1 || m_opcode_read) {
						m_data_bus = m_opcodes_cache->read_byte(m_address_bus);
						m_ir = m_data_bus;
						if (m_m1) {
							clear_m1();
						}
						m_opcode_read = false;
					} else {
						m_data_bus = m_program->read_byte(m_address_bus);
					}
					clear_mreq();
					clear_rd();
				} else if (m_wr) {
					m_program->write_byte(m_address_bus, m_data_bus);
					clear_mreq();
					clear_wr();
				}
			} else if (m_iorq) {
				if (m_rd) {
					m_data_bus = m_io->read_byte(m_address_bus);
					clear_iorq();
					clear_rd();
				} else if (m_wr) {
					m_io->write_byte(m_address_bus, m_data_bus);
					clear_iorq();
					clear_wr();
				}
			}
		}

		if (m_instruction == M1 && m_instruction_step == 0 && m_instruction_offset == 0) {
			check_interrupts();

			m_prvpc.d = m_pc.d;
			debugger_instruction_hook(m_pc.d);
		}

		// Execute steps for instruction
		instructions[m_instruction][m_instruction_step++]();

		if (m_instruction_step >= instructions[m_instruction].size()) {
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


/**************************************************************************
 * Generic set_info
 **************************************************************************/

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


