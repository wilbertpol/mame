// license:BSD-3-Clause
// copyright-holders:Juergen Buchmueller
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

#define PRVPC   m_prvpc.d     /* previous program counter */

#define PCD     m_pc.d
#define PC      m_pc.w.l

#define SPD     m_sp.d
#define SP      m_sp.w.l

#define AFD     m_af.d
#define AF      m_af.w.l
#define A       m_af.b.h
#define F       m_af.b.l

#define BCD     m_bc.d
#define BC      m_bc.w.l
#define B       m_bc.b.h
#define C       m_bc.b.l

#define DED     m_de.d
#define DE      m_de.w.l
#define D       m_de.b.h
#define E       m_de.b.l

#define HLD     m_hl.d
#define HL      m_hl.w.l
#define H       m_hl.b.h
#define L       m_hl.b.l

#define IXD     m_ix.d
#define IX      m_ix.w.l
#define HX      m_ix.b.h
#define LX      m_ix.b.l

#define IYD     m_iy.d
#define IY      m_iy.w.l
#define HY      m_iy.b.h
#define LY      m_iy.b.l

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


// Partial list of Z80 operations from Programming the Z80 by Rodnay Zaks
//
// 00000000 : { X } // NOP
// 00000001 : { X; PC OUT; ARG_READ; DATABUS -> C; PC OUT; ARG_READ; DATABUS -> B } // LD BC,nn
// 00000010 : { X; BC OUT; X (check WAIT); DATABUS -> A } // LD (BC),A
//
// 00000100 : { B -> TMP; X; TMP + 1 -> B } // INC B  (X, operation overlap with next fetch)
// 00000101 : { B -> TMP; X; TMP +-1 -> B } // DEC B  (X, operation overlap with next fetch)
// 00000110 : { X; PC OUT; ARG_READ; DATABUS -> B } // LD B, n
//
// 00001001 : { X; C -> ACT; L -> TMP; ACT + TMP -> L, Cy; B -> ACT; H -> TMP; ACT + TMP + Cy -> H, Cy } // ADD HL,BC  should be 11 cycles
// 00001010 : { X; BC OUT; X (check WAIT); A -> DATABUS } // LD A,(BC)
//
// 00001100 : { C -> TMP, X; TMP + 1 -> C } // INC C  (X, operation overlap with next fetch)
// 00001101 : { C -> TMP, X; TMP - 1 -> C } // DEC C  (X, operation overlap with next fetch)
// 00001110 : { X; PC OUT; ARG_READ; DATABUS -> C } // LD C, n
//
// 00010001 : { X; PC OUT; ARG_READ; DATABUS -> E; PC OUT; ARG_READ; DATABUS -> D } // LD DE,nn
// 00010010 : { X; BC OUT; X (check WAIT); DATABUS -> A } // LD (DE),A
//
// 00010100 : { D -> TMP; X; TMP + 1 -> D } // INC D  (X, operation overlap with next fetch)
// 00010101 : { D -> TMP; X; TMP - 1 -> D } // DEC D  (X, operation overlap with next fetch)
// 00010110 : { X; PC OUT; ARG_READ; DATABUS -> D } // LD D, n
//
// 00011001 : { X; E -> ACT; L -> TMP; ACT + TMP -> L, Cy; D -> ACT; H -> TMP; ACT + TMP + Cy -> H, Cy } // ADD HL,DE  should be 11 cycles
// 00011010 : { X; BC OUT; X (check WAIT); A -> DATABUS } // LD A,(DE)
//
// 00011100 : { E -> TMP; X; TMP + 1 -> E } // INC E  (X, operation overlap with next fetch)
// 00011101 : { E -> TMP; X; TMP - 1 -> E } // DEC E  (X, operation overlap with next fetch)
// 00011110 : { X; PC OUT; ARG_READ; DATABUS -> E } // LD E, n
//
// 00100001 : { X; PC OUT; ARG_READ; DATABUS -> L; PC OUT; ARG_READ; DATABUS -> H } // LD HL,nn
// 00100010 : { X; PC OUT; ARG_READ; DATABUS -> Z; PC OUT; ARG_READ; DATABUS -> W; WZ OUT; X (check WAIT); L -> DATABUS; WZ OUT; X (check WAIT); H -> DATABUS } // LD (nn),HL
//
// 00100100 : { H -> TMP; X; TMP + 1 -> H } // INC H  (X, operation overlap with next fetch)
// 00100101 : { H -> TMP; X; TMP - 1 -> H } // DEC H  (X, operation overlap with next fetch)
// 00100110 : { X; PC OUT; ARG_READ; DATABUS -> H } // LD H, n
// 00100111 : { DAA -> A } // DAA
//
// 00101001 : { X; L -> ACT; L -> TMP; ACT + TMP -> L, Cy; H -> ACT; H -> TMP; ACT + TMP + Cy -> H, Cy } // ADD HL,HL  should be 11 cycles
// 00101010 : { X; PC OUT; ARG_READ; DATABUS -> Z; PC OUT; ARG_READ; DATABUS -> W; WZ OUT; X (check WAIT); DATABUS -> L; WZ OUT; X (check WAIT); DATABUS -> H } // LD HL,(nn)
//
// 00101100 : { L -> TMP; X; TMP + 1 -> L } // INC L  (X, operation overlap with next fetch)
// 00101101 : { L -> TMP; X; TMP - 1 -> L } // DEC L  (X, operation overlap with next fetch)
// 00101110 : { X; PC OUT; ARG_READ; DATABUS -> L } // LD L, n
//
// 00110001 : { X; PC OUT; ARG_READ; DATABUS -> P; PC OUT; ARG_READ; DATABUS -> S } // LD SP,nn
// 00110010 : { X; PC OUT; ARG_READ; DATABUS -> Z; PC OUT; ARG_READ; DATABUS -> W; WZ OUT; X (check WAIT); A -> DATABUS } // LD (nn),A
//
// 00110100 : { X; HL OUT; X (check WAIT); DATABUS -> TMP; HL OUT; X (check WAIT); TMP + 1 -> DATABUS } // INC (HL)
// 00110101 : { X; HL OUT; X (check WAIT); DATABUS -> TMP; HL OUT; X (check WAIT); TMP - 1 -> DATABUS } // DEC (HL)
// 00110110 : { X; PC OUT; ARG_READ; DATABUS -> TMP; HL OUT; X (check WAIT); TMP -> DATABUS } // LD (HL), n
//
// 00111001 : { X; P -> ACT; L -> TMP; ACT + TMP -> L, Cy; S -> ACT; H -> TMP; ACT + TMP + Cy -> H, Cy } // ADD HL,SP  should be 11 cycles
// 00111010 : { X; PC OUT; ARG_READ; DATABUS -> Z; PC OUT; ARG_READ; DATABUS -> W; WZ OUT; X (check WAIT); DATABUS -> A } // LD A,(nn)
//
// 00111100 : { A -> TMP; X; TMP + 1 -> A } // INC A  (X, operation overlap with next fetch)
// 00111101 : { A -> TMP; X; TMP - 1 -> A } // DEC A  (X, operation overlap with next fetch)
// 00111110 : { X; PC OUT; ARG_READ; DATABUS -> A } // LD A, n
//
// 01000000 : { B -> TMP; TMP -> B } // LD B, B  (last store overlaps with next fetch)
// 01000001 : { C -> TMP; TMP -> B } // LD B, C  (last store overlaps with next fetch)
// 01000010 : { D -> TMP; TMP -> B } // LD B, D  (last store overlaps with next fetch)
// 01000011 : { E -> TMP; TMP -> B } // LD B, E  (last store overlaps with next fetch)
// 01000100 : { H -> TMP; TMP -> B } // LD B, H  (last store overlaps with next fetch)
// 01000101 : { L -> TMP; TMP -> B } // LD B, L  (last store overlaps with next fetch)
// 01000110 : { X; HL OUT; DATA_READ; DATABUS -> B } // LD B,(HL)
// 01000111 : { A -> TMP; TMP -> B } // LD B, A  (last store overlaps with next fetch)
// 01001000 : { B -> TMP; TMP -> C } // LD C, B  (last store overlaps with next fetch)
// 01001001 : { C -> TMP; TMP -> C } // LD C, C  (last store overlaps with next fetch)
// 01001010 : { D -> TMP; TMP -> C } // LD C, D  (last store overlaps with next fetch)
// 01001011 : { E -> TMP; TMP -> C } // LD C, E  (last store overlaps with next fetch)
// 01001100 : { H -> TMP; TMP -> C } // LD C, H  (last store overlaps with next fetch)
// 01001101 : { L -> TMP; TMP -> C } // LD C, L  (last store overlaps with next fetch)
// 01001110 : { X; HL OUT; DATA_READ; DATABUS -> C } // LD C,(HL)
// 01001111 : { A -> TMP; TMP -> C } // LD C, A  (last store overlaps with next fetch)
// 01010000 : { B -> TMP; TMP -> D } // LD D, B  (last store overlaps with next fetch)
// 01010001 : { C -> TMP; TMP -> D } // LD D, C  (last store overlaps with next fetch)
// 01010010 : { D -> TMP; TMP -> D } // LD D, D  (last store overlaps with next fetch)
// 01010011 : { E -> TMP; TMP -> D } // LD D, E  (last store overlaps with next fetch)
// 01010100 : { H -> TMP; TMP -> D } // LD D, H  (last store overlaps with next fetch)
// 01010101 : { L -> TMP; TMP -> D } // LD D, L  (last store overlaps with next fetch)
// 01010110 : { X; HL OUT; DATA_READ; DATABUS -> D } // LD D,(HL)
// 01010111 : { A -> TMP; TMP -> D } // LD D, A  (last store overlaps with next fetch)
// 01011000 : { B -> TMP; TMP -> E } // LD E, B  (last store overlaps with next fetch)
// 01011001 : { C -> TMP; TMP -> E } // LD E, C  (last store overlaps with next fetch)
// 01011010 : { D -> TMP; TMP -> E } // LD E, D  (last store overlaps with next fetch)
// 01011011 : { E -> TMP; TMP -> E } // LD E, E  (last store overlaps with next fetch)
// 01011100 : { H -> TMP; TMP -> E } // LD E, H  (last store overlaps with next fetch)
// 01011101 : { L -> TMP; TMP -> E } // LD E, L  (last store overlaps with next fetch)
// 01011110 : { X; HL OUT; DATA_READ; DATABUS -> E } // LD E,(HL)
// 01011111 : { A -> TMP; TMP -> E } // LD E, A  (last store overlaps with next fetch)
// 01100000 : { B -> TMP; TMP -> H } // LD H, B  (last store overlaps with next fetch)
// 01100001 : { C -> TMP; TMP -> H } // LD H, C  (last store overlaps with next fetch)
// 01100010 : { D -> TMP; TMP -> H } // LD H, D  (last store overlaps with next fetch)
// 01100011 : { E -> TMP; TMP -> H } // LD H, E  (last store overlaps with next fetch)
// 01100100 : { H -> TMP; TMP -> H } // LD H, H  (last store overlaps with next fetch)
// 01100101 : { L -> TMP; TMP -> H } // LD H, L  (last store overlaps with next fetch)
// 01100110 : { X; HL OUT; DATA_READ; DATABUS -> H } // LD H,(HL)
// 01100111 : { A -> TMP; TMP -> H } // LD H, A  (last store overlaps with next fetch)
// 01101000 : { B -> TMP; TMP -> L } // LD L, B  (last store overlaps with next fetch)
// 01101001 : { C -> TMP; TMP -> L } // LD L, C  (last store overlaps with next fetch)
// 01101010 : { D -> TMP; TMP -> L } // LD L, D  (last store overlaps with next fetch)
// 01101011 : { E -> TMP; TMP -> L } // LD L, E  (last store overlaps with next fetch)
// 01101100 : { H -> TMP; TMP -> L } // LD L, H  (last store overlaps with next fetch)
// 01101101 : { L -> TMP; TMP -> L } // LD L, L  (last store overlaps with next fetch)
// 01101110 : { X; HL OUT; DATA_READ; DATABUS -> L } // LD L,(HL)
// 01101111 : { A -> TMP; TMP -> L } // LD L, A  (last store overlaps with next fetch)
// 01110000 : { B -> TMP; HL OUT; X (check WAIT); TMP -> DATABUS } // LD (HL),B
// 01110001 : { C -> TMP; HL OUT; X (check WAIT); TMP -> DATABUS } // LD (HL),C
// 01110010 : { D -> TMP; HL OUT; X (check WAIT); TMP -> DATABUS } // LD (HL),D
// 01110011 : { E -> TMP; HL OUT; X (check WAIT); TMP -> DATABUS } // LD (HL),E
// 01110100 : { H -> TMP; HL OUT; X (check WAIT); TMP -> DATABUS } // LD (HL),H
// 01110101 : { L -> TMP; HL OUT; X (check WAIT); TMP -> DATABUS } // LD (HL),L
//
// 01110111 : { A -> TMP; HL OUT; X (check WAIT); TMP -> DATABUS } // LD (HL),A
// 01111000 : { B -> TMP; TMP -> A } // LD A, B  (last store overlaps with next fetch)
// 01111001 : { C -> TMP; TMP -> A } // LD A, C  (last store overlaps with next fetch)
// 01111010 : { D -> TMP; TMP -> A } // LD A, D  (last store overlaps with next fetch)
// 01111011 : { E -> TMP; TMP -> A } // LD A, E  (last store overlaps with next fetch)
// 01111100 : { H -> TMP; TMP -> A } // LD A, H  (last store overlaps with next fetch)
// 01111101 : { L -> TMP; TMP -> A } // LD A, L  (last store overlaps with next fetch)
// 01111110 : { X; HL OUT; DATA_READ; DATABUS -> A } // LD A,(HL)
// 01111111 : { A -> TMP; TMP -> A } // LD A, A  (last store overlaps with next fetch)
// 10000000 : { B -> TMP; A -> ACT; X; ACT + TMP -> A } // ADD A,B  (X, operation overlaps with next fetch)
// 10000001 : { C -> TMP; A -> ACT; X; ACT + TMP -> A } // ADD A,C  (X, operation overlaps with next fetch)
// 10000010 : { D -> TMP; A -> ACT; X; ACT + TMP -> A } // ADD A,D  (X, operation overlaps with next fetch)
// 10000011 : { E -> TMP; A -> ACT; X; ACT + TMP -> A } // ADD A,E  (X, operation overlaps with next fetch)
// 10000100 : { H -> TMP; A -> ACT; X; ACT + TMP -> A } // ADD A,H  (X, operation overlaps with next fetch)
// 10000101 : { L -> TMP; A -> ACT; X; ACT + TMP -> A } // ADD A,L  (X, operation overlaps with next fetch)
// 10000110 : { A -> ACT; HL OUT; X (check WAIT); DATABUS -> TMP; X; ACT + TMP -> A } // ADD A,(HL)  (X, operation overlaps with next fetch)
// 10000111 : { A -> TMP; A -> ACT; X; ACT + TMP -> A } // ADD A,A  (X, operation overlaps with next fetch)
// 10001000 : { B -> TMP; A -> ACT; X; ACT + TMP + Cy -> A } // ADC A,B  (X, operation overlaps with next fetch)
// 10001001 : { C -> TMP; A -> ACT; X; ACT + TMP + Cy -> A } // ADC A,C  (X, operation overlaps with next fetch)
// 10001010 : { D -> TMP; A -> ACT; X; ACT + TMP + Cy -> A } // ADC A,D  (X, operation overlaps with next fetch)
// 10001011 : { E -> TMP; A -> ACT; X; ACT + TMP + Cy -> A } // ADC A,E  (X, operation overlaps with next fetch)
// 10001100 : { H -> TMP; A -> ACT; X; ACT + TMP + Cy -> A } // ADC A,H  (X, operation overlaps with next fetch)
// 10001101 : { L -> TMP; A -> ACT; X; ACT + TMP + Cy -> A } // ADC A,L  (X, operation overlaps with next fetch)
// 10001110 : { A -> ACT; HL OUT; X (check WAIT); DATABUS -> TMP; X; ACT + TMP + Cy -> A } // ADC A,(HL)  (X, operation overlaps with next fetch)
// 10001111 : { A -> TMP; A -> ACT; X; ACT + TMP + Cy -> A } // ADC A,A  (X, operation overlaps with next fetch)
// 10010000 : { B -> TMP; A -> ACT; X; ACT - TMP -> A } // SUB A,B  (X, operation overlaps with next fetch)
// 10010001 : { C -> TMP; A -> ACT; X; ACT - TMP -> A } // SUB A,C  (X, operation overlaps with next fetch)
// 10010010 : { D -> TMP; A -> ACT; X; ACT - TMP -> A } // SUB A,D  (X, operation overlaps with next fetch)
// 10010011 : { E -> TMP; A -> ACT; X; ACT - TMP -> A } // SUB A,E  (X, operation overlaps with next fetch)
// 10010100 : { H -> TMP; A -> ACT; X; ACT - TMP -> A } // SUB A,H  (X, operation overlaps with next fetch)
// 10010101 : { L -> TMP; A -> ACT; X; ACT - TMP -> A } // SUB A,L  (X, operation overlaps with next fetch)
// 10010110 : { A -> ACT; HL OUT; X (check WAIT); DATABUS -> TMP; X; ACT - TMP -> A } // SUB A,(HL)  (X, operation overlaps with next fetch)
// 10010111 : { A -> TMP; A -> ACT; X; ACT - TMP -> A } // SUB A,A  (X, operation overlaps with next fetch)
//
// 10100000 : { B -> TMP, A -> ACT; X, ACT AND TMP -> A } // AND A,B  (X, operation overlaps with next fetch)
// 10100001 : { C -> TMP, A -> ACT; X, ACT AND TMP -> A } // AND A,C  (X, operation overlaps with next fetch)
// 10100010 : { D -> TMP, A -> ACT; X, ACT AND TMP -> A } // AND A,D  (X, operation overlaps with next fetch)
// 10100011 : { E -> TMP, A -> ACT; X, ACT AND TMP -> A } // AND A,E  (X, operation overlaps with next fetch)
// 10100100 : { H -> TMP, A -> ACT; X, ACT AND TMP -> A } // AND A,H  (X, operation overlaps with next fetch)
// 10100101 : { L -> TMP, A -> ACT; X, ACT AND TMP -> A } // AND A,L  (X, operation overlaps with next fetch)
// 10100110 : { A -> ACT; HL OUT; X (check WAIT); DATABUS -> TMP; X; ACT AND TMP -> A } // AND A,(HL)  (X, operation overlaps with next fetch)
// 10100111 : { A -> TMP, A -> ACT; X, ACT AND TMP -> A } // AND A,A  (X, operation overlaps with next fetch)
//
// 10111000 : { B -> TMP; A -> ACT; X; ACT - TMP - Cy -> A } // SBC A,B  (X, operation overlaps with next fetch)
// 10111001 : { C -> TMP; A -> ACT; X; ACT - TMP - Cy -> A } // SBC A,C  (X, operation overlaps with next fetch)
// 10111010 : { D -> TMP; A -> ACT; X; ACT - TMP - Cy -> A } // SBC A,D  (X, operation overlaps with next fetch)
// 10111011 : { E -> TMP; A -> ACT; X; ACT - TMP - Cy -> A } // SBC A,E  (X, operation overlaps with next fetch)
// 10111100 : { H -> TMP; A -> ACT; X; ACT - TMP - Cy -> A } // SBC A,H  (X, operation overlaps with next fetch)
// 10111101 : { L -> TMP; A -> ACT; X; ACT - TMP - Cy -> A } // SBC A,L  (X, operation overlaps with next fetch)
// 10111110 : { A -> ACT; HL OUT; X (check WAIT); DATABUS -> TMP; X; ACT - TMP - Cy -> A } // SBC A,(HL)  (X, operation overlaps with next fetch)
// 10111111 : { A -> TMP; A -> ACT; X; ACT - TMP - Cy -> A } // SBC A,A  (X, operation overlaps with next fetch)
//
// 11000011 : { X; PC OUT; ARG_READ; STORE_Z; PC OUT; ARG_READ; STORE_W; WZ_TO_PC_0 (0 cycles) } // JMP nn
//
// 11000110 : { A -> ACT; PC OUT; ARG_READ; DATABUS -> TMP; X; ACT + TMP -> A } // ADD A,n  (X, operation overlaps with next fetch)
//
// 11001110 : { A -> ACT; PC OUT; ARG_READ; DATABUS -> TMP; X; ACT + TMP + Cy -> A } // ADC A,n  (X, operation overlaps with next fetch)
//
// 11010110 : { A -> ACT; PC OUT; ARG_READ; DATABUS -> TMP; X; ACT - TMP -> A } // SUB A,n  (X, operation overlaps with next fetch)
//
// 11011110 : { A -> ACT; PC OUT; ARG_READ; DATABUS -> TMP; X; ACT - TMP - Cy -> A } // SBC A,n  (X, operation overlaps with next fetch)
//
// 11101011 : { HL <-> DE } // EX HL,DE
//
// 11111001 : { X; HL -> INCDEC; INCDEC -> SP } // LD SP,HL

const uint8_t z80lle_device::insts[256][17] = {
	/* 0x00 */
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	{ X, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_REG, END },  // 0x06, 7 cycles, LD B,n
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	{ X, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_REG, END },  // 0x0e, 7 cycles, LD C,n
	{ 0 },
	/* 0x10 */
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	{ X, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_REG, END },  // 0x16, 7 cycles, LD D,n
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	{ X, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_REG, END },  // 0x1e, 7 cycles, LD E,n
	{ 0 },
	/* 0x20 */
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	{ X, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_REG, END },  // 0x26, 7 cycles, LD H,n
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	{ X, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_REG, END },  // 0x2e, 7 cycles, LD L,n
	{ 0 },
	/* 0x30 */
	{ 0 }, { 0 },
	{ X, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, WZ_OUT, WZ_INC, A_DB, WRITE, CHECK_WAIT, END },  // 0x32, 13 cycles, LD (nn),A
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	{ X, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, WZ_OUT, WZ_INC, READ, CHECK_WAIT, DB_A, END },  // 0x3a, 13 cycles, LD A,(nn)
	{ 0 }, { 0 }, { 0 },
	{ X, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_REG, END },  // 0x3e, 7 cycles, LD A,n
	{ 0 },
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
	{ 0 }, { 0 }, { 0 },
	{ X, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_W, WZ_TO_PC, END },  // 0xc3, 10 cycles, JMP nn
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* 0xd0 */
	{ 0 }, { 0 }, { 0 },
	{ X, PC_OUT, PC_INC, READ, CHECK_WAIT, DB_Z, A_W, WZ_OUT, WZ_INC, A_DB_0, OUTPUT, CHECK_WAIT, END },  // 0xd3, 11 cycles, OUT (n), A
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* 0xe0 */
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
	/* 0xf0 */
	{ 0 }, { 0 }, { 0 },
	{ DI },  // 0xf3, 4 cycles, DI
	{ 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 },
};


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
	save_item(NAME(PC));
	save_item(NAME(SP));
	save_item(NAME(AF));
	save_item(NAME(BC));
	save_item(NAME(DE));
	save_item(NAME(HL));
	save_item(NAME(IX));
	save_item(NAME(IY));
	save_item(NAME(WZ));
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
	PRVPC = 0;
	PCD = 0;
	SPD = 0;
	AFD = 0;
	BCD = 0;
	DED = 0;
	HLD = 0;
	IXD = 0;
	IYD = 0;
	WZ = 0;
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

	IX = IY = 0xffff; /* IX and IY are FFFF after a reset! */
	F = ZF;           /* Zero flag is set */

	/* set up the state table */
	state_add(STATE_GENPC,     "PC",        m_pc.w.l).callimport();
	state_add(STATE_GENPCBASE, "CURPC",     m_prvpc.w.l).callimport().noshow();
	state_add(Z80_SP,          "SP",        SP);
	state_add(STATE_GENSP,     "GENSP",     SP).noshow();
	state_add(STATE_GENFLAGS,  "GENFLAGS",  F).noshow().formatstr("%8s");
	state_add(Z80_A,           "A",         A).noshow();
	state_add(Z80_B,           "B",         B).noshow();
	state_add(Z80_C,           "C",         C).noshow();
	state_add(Z80_D,           "D",         D).noshow();
	state_add(Z80_E,           "E",         E).noshow();
	state_add(Z80_H,           "H",         H).noshow();
	state_add(Z80_L,           "L",         L).noshow();
	state_add(Z80_AF,          "AF",        AF);
	state_add(Z80_BC,          "BC",        BC);
	state_add(Z80_DE,          "DE",        DE);
	state_add(Z80_HL,          "HL",        HL);
	state_add(Z80_IX,          "IX",        IX);
	state_add(Z80_IY,          "IY",        IY);
	state_add(Z80_AF2,         "AF2",       m_af2.w.l);
	state_add(Z80_BC2,         "BC2",       m_bc2.w.l);
	state_add(Z80_DE2,         "DE2",       m_de2.w.l);
	state_add(Z80_HL2,         "HL2",       m_hl2.w.l);
	state_add(Z80_WZ,          "WZ",        WZ);
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
	PC = 0x0000;
	m_i = 0;
	m_r = 0;
	m_r2 = 0;
	m_nmi_pending = false;
	m_after_ei = false;
	m_after_ldair = false;
	m_iff1 = 0;
	m_iff2 = 0;

	WZ=PCD;

	m_execution_state = FETCH;
	m_fetch_state = M1_SET_ADDRESS;
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

		// TODO: Create a "program" for the M1 opcode fetching

		switch (m_execution_state) {
		case FETCH:
			// Set up PC address, read, wait for WAIT lines, read instruction
			// when done, decode instruction, optionally redo fetch for more opcode fetches
			switch (m_fetch_state) {
			case M1_SET_ADDRESS:
				PRVPC = PCD;
				// TODO: Only do this when starting an instruction
				debugger_instruction_hook(PCD);
				// cycle #0: Output PC on address lines
				m_icount -= 1;
				m_address_bus = PC;
				m_fetch_state = M1_READ_OP;
				break;
			case M1_READ_OP:
				// Assert MREQ and RD signals.
				// Now we may be reading a bit too early, the opcode should really be read when /WAIT is high again.
				if (m_wait_state)
				{
					m_data_bus = m_decrypted_opcodes_direct->read_byte(m_address_bus);
					PC++;
					// /WAIT might have been pulled low by the read
					m_fetch_state = m_wait_state ? M1_REFRESH : M1_WAIT_STATE;
				}
				m_icount -= 1;
				break;
			case M1_WAIT_STATE:
				if (m_wait_state)
				{
					m_fetch_state = M1_REFRESH;
				}
				m_icount -= 1;
				break;
			case M1_REFRESH:
				// do RAM refresh
				m_ir = m_data_bus;
				m_instruction_step = 0;
				m_refresh_cb((m_i << 8) | (m_r2 & 0x80) | (m_r & 0x7f), 0x00, 0xff);
				m_r++;
				m_icount -= 1;
				// TODO Perform decode here
				m_fetch_state = M1_SET_ADDRESS;
				m_execution_state = EXECUTE;
				break;
			case DECODE:
				// Not needed... TODO: Remove
				// TODO: Proper decoding and handling of prefixes
				if (m_ir == 0xDD || m_ir == 0xCB || m_ir == 0xED) {
					m_fetch_state = M1_SET_ADDRESS;
				} else {
					// TODO: Do decode logic
					m_fetch_state = M1_SET_ADDRESS;
					m_execution_state = EXECUTE;
				}
				m_icount -= 1;
				break;
			}
			break;
		case EXECUTE:
			// Execute steps for instruction
			if (insts[m_ir][0] == END) {
				fatalerror("Unsupported instruction %02x encounted at address %04x", m_ir, PRVPC);
			}
			while (m_icount > 0 && insts[m_ir][m_instruction_step] != END) {
				switch (insts[m_ir][m_instruction_step]) {
				case A_DB:
					m_data_bus = A;
					WZ_H = m_data_bus;
					m_icount -= 1;
					logerror("A_DB\n");
					break;
				case A_DB_0:
					m_data_bus = A;
					WZ_H = m_data_bus;
					logerror("A_DB_0\n");
					break;
				case A_W:
					WZ_H = A;
					logerror("A_W\n");
					break;
				case CHECK_WAIT:
					if (!m_wait_state) {
						m_icount -=1;
						// Do not advance to next step
						m_instruction_step--;
					}
					logerror("CHECK_WAIT\n");
					break;
				case DB_REG:
					switch(m_ir & 0x38) {
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
					m_icount -= 1;
					logerror("DB_REG\n");
					break;
				case DB_A:
					A = m_data_bus;
					m_icount -= 1;
					logerror("DB_A\n");
					break;
				case DB_W:
					WZ_H = m_data_bus;
					m_icount -= 1;
					logerror("DB_W: WZ = %04x\n", WZ);
					break;
				case DB_Z:
					WZ_L = m_data_bus;
					m_icount -= 1;
					logerror("DB_Z: WZ = %04x\n", WZ);
					break;
				case DI:
					m_iff1 = m_iff2 = 0;
					m_icount -= 1;
					logerror("DI\n");
					break;
				case OUTPUT:
					m_io->write_byte(m_address_bus, m_data_bus);
					m_icount -= 3;
					logerror("OUTPUT\n");
					break;
				case PC_INC:
					PC++;
					logerror("PC_INC\n");
					break;
				case PC_OUT:
					m_address_bus = PC;
					m_icount -= 1;
					logerror("PC_OUT\n");
					break;
				case READ:
					m_data_bus = m_program->read_byte(m_address_bus);
					m_icount -= 1;
					logerror("READ: read %02x from %04x\n", m_data_bus, m_address_bus);
					break;
				case WRITE:
					m_program->write_byte(m_address_bus, m_data_bus);
					m_icount -= 1;
					logerror("WRITE: write %02x to %04x\n", m_data_bus, m_address_bus);
					break;
				case WZ_INC:
					WZ++;
					logerror("WZ_INC\n");
					break;
				case WZ_OUT:
					m_address_bus = WZ;
					m_icount -= 1;
					logerror("WZ_OUT\n");
					break;
				case WZ_TO_PC:
					PC = WZ;
					logerror("WZ_TO_PC\n");
					break;
				case X:
					m_icount -= 1;
					logerror("X, skip cycle\n");
					break;
				}
				m_instruction_step++;
			}
			if (insts[m_ir][m_instruction_step] == END) {
				m_execution_state = M1_SET_ADDRESS;
			}
			break;
		}
//		EXEC(op,rop());
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
				F & 0x80 ? 'S':'.',
				F & 0x40 ? 'Z':'.',
				F & 0x20 ? 'Y':'.',
				F & 0x10 ? 'H':'.',
				F & 0x08 ? 'X':'.',
				F & 0x04 ? 'P':'.',
				F & 0x02 ? 'N':'.',
				F & 0x01 ? 'C':'.');
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


