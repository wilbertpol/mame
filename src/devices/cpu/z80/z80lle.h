// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
#ifndef MAME_CPU_Z80_Z80LLE_H
#define MAME_CPU_Z80_Z80LLE_H

#pragma once

#include "machine/z80daisy.h"

#define MCFG_Z80LLE_SET_IRQACK_CALLBACK(_devcb) \
	devcb = &downcast<z80lle_device &>(*device).set_irqack_cb(DEVCB_##_devcb);

#define MCFG_Z80LLE_SET_REFRESH_CALLBACK(_devcb) \
	devcb = &downcast<z80lle_device &>(*device).set_refresh_cb(DEVCB_##_devcb);

#define MCFG_Z80LLE_SET_HALT_CALLBACK(_devcb) \
	devcb = &downcast<z80lle_device &>(*device).set_halt_cb(DEVCB_##_devcb);

enum
{
	Z80LLE_INPUT_LINE_WAIT = INPUT_LINE_IRQ0 + 1,
	Z80LLE_INPUT_LINE_BOGUSWAIT, /* WAIT pin implementation used to be nonexistent, please remove this when all drivers are updated with Z80_INPUT_LINE_WAIT */
	Z80LLE_INPUT_LINE_BUSRQ
};

enum
{
	Z80LLE_PC = STATE_GENPC, Z80LLE_SP = 1,
	Z80LLE_A, Z80LLE_B, Z80LLE_C, Z80LLE_D, Z80LLE_E, Z80LLE_H, Z80LLE_L,
	Z80LLE_AF, Z80LLE_BC, Z80LLE_DE, Z80LLE_HL,
	Z80LLE_IX, Z80LLE_IY, Z80LLE_AF2, Z80LLE_BC2, Z80LLE_DE2, Z80LLE_HL2,
	Z80LLE_R, Z80LLE_I, Z80LLE_IM, Z80LLE_IFF1, Z80LLE_IFF2, Z80LLE_HALT,
	Z80LLE_DC0, Z80LLE_DC1, Z80LLE_DC2, Z80LLE_DC3, Z80LLE_WZ
};

class z80lle_device : public cpu_device, public z80_daisy_chain_interface
{
public:
	z80lle_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	auto irqack_cb() { return m_irqack_cb.bind(); }
	auto refresh_cb() { return m_refresh_cb.bind(); }
	auto halt_cb() { return m_halt_cb.bind(); }
	void set_m1_wait_states(u8 m1_wait_states) { m_m1_wait_states = m1_wait_states; }

protected:
	z80lle_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// device_execute_interface overrides
	virtual uint32_t execute_min_cycles() const override { return 2; }
	virtual uint32_t execute_max_cycles() const override { return 16; }
	virtual uint32_t execute_input_lines() const override { return 4; }
	virtual uint32_t execute_default_irq_vector(int inputnum) const override { return 0xff; }
	virtual void execute_run() override;
	virtual void execute_set_input(int inputnum, int state) override;

	// device_memory_interface overrides
	virtual space_config_vector memory_space_config() const override;

	// device_state_interface overrides
	virtual void state_import(const device_state_entry &entry) override;
	virtual void state_export(const device_state_entry &entry) override;
	virtual void state_string_export(const device_state_entry &entry, std::string &str) const override;

	// device_disasm_interface overrides
	virtual std::unique_ptr<util::disasm_interface> create_disassembler() override;

	// address spaces
	const address_space_config m_program_config;
	const address_space_config m_decrypted_opcodes_config;
	const address_space_config m_io_config;
	address_space *m_program;
	address_space *m_opcodes;
	address_space *m_io;
	memory_access_cache<0, 0, ENDIANNESS_LITTLE> *m_cache;
	memory_access_cache<0, 0, ENDIANNESS_LITTLE> *m_opcodes_cache;
	devcb_write_line m_irqack_cb;
	devcb_write8 m_refresh_cb;
	devcb_write_line m_halt_cb;

	// Sub instructions
	enum {
		UNKNOWN=0,
		A_ACT,       // register A to ACT input for ALU
		A_DB,        // register A to data bus, also writes to W always?
		A_W,         // register A to W
		ADC16,       // 16bit addition with carry, takes 7 cycles
		ADD16,       // 16bit addition, takes 7 cycles
		ALU_A,       // ALU output to A
		ALU_DB,      // ALU output to data bus
		ALU_ADC,     // ALU operation: ADC
		ALU_ADD,     // ALU operation: ADD
		ALU_AND,     // ALU operation: AND
		ALU_BIT,     // ALU operation: BIT
		ALU_RES,     // ALU operation: RES
		ALU_SET,     // ALU operation: SET
		ALU_CP,      // ALU operation: CP
		ALU_DEC,     // ALU operation: DEC (decrements TMP input)
		ALU_INC,     // ALU operation: INC (increments TMP input)
		ALU_OR,      // ALU operation: OR
		ALU_REGS,    // ALU output to source register (bits .....xxx)
		ALU_REGS0,   // ALU output to source register (bits .....xxx), not to index registers
		ALU_REGD,    // ALU output to destination register (bits ..xxx...)
		ALU_RL,      // ALU operation: RL
		ALU_RLC,     // ALU operation: RLC
		ALU_RR,      // ALU operation: RR
		ALU_RRC,     // ALU operation: RRC
		ALU_SBC,     // ALU operation: SBC
		ALU_SLA,     // ALU operation: SLA
		ALU_SLL,     // ALU operation: SLL
		ALU_SRA,     // ALU operation: SRA
		ALU_SRL,     // ALU operation: SRL
		ALU_SUB,     // ALU operation: SUB
		ALU_XOR,     // ALU operation: XOR
		BC_OUT,      // Put BC on address bus, takes 1 cycle
		BC_WZ,       // Store BC in WZ
		CALL_COND,   // Check condition for CALL, takes 1 cycle when condition is true
		CCF,         // CCF
		CPD,         // Set flags and update pointers and counter, takes 5 cycles
		CPI,         // Set flags and update pointers and counter, takes 5 cycles
		CPL,         // CPL
		DAA,         // DAA
		DB_A,        // Store data bus in A
		DB_REGD,     // Store data bus in 8bit register (bits ..xxx...)
		DB_REGD0,    // Store data bus in 8bit register (bits ..xxx...), not to index registers
		DB_REGD_INPUT, // Store data bus in 8bit register (bits ..xxx...)
		DB_TMP,      // Store data bus in TMP
		DB_W,        // Store data bus in W
		DB_Z,        // Store data bus in Z
		DE_OUT,      // Put DE on address bus, takes 1 cycle
		DE_WZ,       // Store DE in WZ
		DEC_R8,      // Decrement an 8 bit register
		DEC_R16,     // Decrement a 16 bit register, takes 2 cycles
		DEC_SP,      // Decrement SP (for PUSH)
		DECODE,      // Decode instruction
		DISP_WZ2,    // Calculate IX/IY displacement into WZ, takes 2 cycles (in DD CB xx II instructions)
		DISP_WZ5,    // Calculate IX/IY displacement into WZ, takes 5 cycles (in DD xx instructions)
		DI,          // Reset interrupt flip flops
		DJNZ,        // Decrement B and jump when not zero, takes 5 cycles when branch taken
		EI,          // Set interrupt flip flops
		EX_AF_AF,    // Swap AF and AF'
		EX_DE_HL,    // Swap DE and HL
		EXX,         // Swap BC, DE, HL and BC2, DE2, HL2
		HALT,        // HALT
		H_DB,        // register H to data bus
		HL_OUT,      // Put HL on address bus, takes 1 cycle
		HL_PC,       // Store HL in PC
		HL_WZ,       // Store HL in WZ
		IM,          // IM
		INC_R8,      // Increment an 8 bit register
		INC_R16,     // Increment a 16 bit register, takes 2 cycles
		INC_SP,      // Increment SP (for POP)
		IND,         // Set flags and update pointers and counter, takes no cycles
		INI,         // Set flags and update pointers and counter, takes no cycles
		INPUT,       // Read data bus from input, takes 3 cycles
		INPUT_REGD,  // Read data bus from input, store in 8 bit register, takes 3 cycles
		JR_COND,     // Check condition (Z, NZ, etc) for JR and perform jump, 5 cycles when branch taken
		JP_COND,     // Check condition for JP and perform jump
		L_DB,        // register L to data bus
		LD_A_I,      // LD A,I, takes 1 cycle
		LD_A_R,      // LD A,R, takes 1 cycle
		LD_I_A,      // LD I,A, takes 1 cycle
		LD_R_A,      // LD R,A, takes 1 cycle
		LD_SP_HL,    // LD SP,HL, takes 2 cycles
		LDD,         // Set flags and update pointers and counter, takes 2 cycles
		LDI,         // Set flags and update pointers and counter, takes 2 cycles
		NEG,         // NEG
		NMI,         // NMI
		OUTD,        // Set flags and update pointers and counter and prepare for I/O, takes 1 cycles
		OUTI,        // Set flags and update pointers and counter and prepare for I/O, takes 1 cycles
		OUTPUT,      // Write data bus to output, takes 3 cycles
		PC_OUT,      // Put PC on address bus, takes 1 cycle
		PC_OUT_INC,  // Put PC on address bus, takes 1 cycle, increment PC
		PC_OUT_INC_M1,  // Put PC on address bus, assert M1, takes 1 cycle, increment PC
		PCH_DB,      // Put PC 8 high bits on data bus
		PCL_DB,      // Put PC 8 low bits on data bus
		R16H_DB,     // Put high 8 bits of 16 bit register on data bus
		R16L_DB,     // Put low 8 bits of 16 bit register on data bus
		READ,        // Read memory from m_address_bus, storing result in m_data_bus, takes 2 cycles
		READ_A,      // Read memory from m_address_bus, storing result in A, takes 2 cycles
		READ_OP,     // M1 - read memory, takes 1 cycle
		READ_OP2,    // Opcode read as part of DD/FD CB dd xx instructions, takes 2 cycles
		READ_OP_IRQ, // Special opcode reading while taking an interrupt
		READ_R16H,   // Read memory from m_address_bus, storing result in high 8 bits of 16 bit register, takes 2 cycles
		READ_R16L,   // Read memory from m_address_bus, storing result in low 8 bits of 16 bit register, takes 2 cycles
		READ_REGD,   // Read memory from m_address_bus, storing result in 8 bit register, takes 2 cycles
		READ_REGD0,  // Read memory from m_address_bus, storing result in 8 bit register, takes 2 cycles (Also H and L for DD/FD prefixed instructions)
		READ_TMP,    // Read memory from m_address_bus, storing result in temp, takes 2 cycles
		READ_W,      // Read memory from m_address_bus, storing result in W, takes 2 cycles
		READ_Z,      // Read memory from m_address_bus, storing result in Z, takes 2 cycles
		REFRESH,     // Refresh RAM, takes 2 cycles
		REGD_DB,     // 8 bit source register (bits ..xxx...) to data bus
		REGD_TMP,    // 8 bit destination register (bits ..xxx...) to TMP
		REGS_DB,     // 8 bit source register (bits .....xxx) to data bus
		REGS0_DB,    // 8 bit source register (bits .....xxx) to data bus not to index registers
		REGS_TMP,    // 8 bit source register (bits .....xxx) to TMP
		REPEAT,      // Move PC 2 steps back if BC != 0, takes 5 cycles
		REPEATCP,    // Move PC 2 steps back if BC != 0 and ZF clear, takes 5 cycles
		REPEATIO,    // Move PC 2 steps back if B != 0, takes 5 cycles
		RET_COND,    // Check condition for RET, takes 1 cycle
		RETI,        // RETI
		RETN,        // RETN
		RLA,         // RLA
		RLCA,        // RLCA
		RLD,         // RLD, takes 5 cycles
		RRA,         // RRA
		RRCA,        // RRCA
		RRD,         // RRD, takes 5 cycles
		RST,         // Change PC to 0/8/10/18/20/28/30/38
		SBC16,       // 16bit subtraction with carry, takes 7 cycles
		SCF,         // SCF
		SP_OUT,      // Put SP on address bus, takes 1 cycle
		SP_OUT_DEC,  // Decrement SP and put SP on address bus, takes 1 cycle
		SP_OUT_INC,  // Put SP on address bus and increment, takes 1 cycle
		TMP_REG,     // TMP to 8 bit register
		WRITE,       // Write data bus to memory, takes 2 cycles
		WZ_HL,       // Store contents of WZ in HL
		WZ_OUT,      // Put WZ on address bus, takes 1 cycle
		WZ_OUT_INC,  // Put WZ on address bus and increment WZ, takes 1 cycle
		BC_WZ_OUT_INC,  // Store BC in WZ, put WZ on address bus and increment WZ, takes 1 cycle
		DE_WZ_OUT_INC,  // Store DE in WZ, put WZ on address bus and increment WZ, takes 1 cycle
		WZ_PC,       // Store contents of WZ in PC
		X,           // Do nothing, takes 1 cycle
		X2,          // Do nothing, takes 2 cycles
		ZERO_DB,     // put all zeroes on the data bus

		// Testing optimizations
		ADD_R8, // A_ACT, REGS_TMP, ALU_ADD, ALU_A
		ADD_TMP, // A_ACT, ALU_ADD, ALU_A
		ADC_R8, // A_ACT, REGS_TMP, ALU_ADC, ALU_A
		ADC_TMP, // A_ACT, ALU_ADC, ALU_A
		SUB_R8, // A_ACT, REGS_TMP, ALU_SUB, ALU_A
		SUB_TMP, // A_ACT, ALU_SUB, ALU_A
		SBC_R8, // A_ACT, REGS_TMP, ALU_SBC, ALU_A
		SBC_TMP, // A_ACT, ALU_SBC, ALU_A
		AND_R8, // A_ACT, REGS_TMP, ALU_AND, ALU_A
		AND_TMP, // A_ACT, ALU_AND, ALU_A
		XOR_R8, // A_ACT, REGS_TMP, ALU_XOR, ALU_A
		XOR_TMP, // A_ACT, ALU_XOR, ALU_A
		OR_R8, // A_ACT, REGS_TMP, ALU_OR, ALU_A
		OR_TMP, // A_ACT, ALU_OR, ALU_A
		CP_R8, // A_ACT, REGS_TMP, ALU_CP
		CP_TMP, // A_ACT, ALU_CP
		BIT_R8, // REGS_TMP, ALU_BIT
		REGS_TMP_REG, // REGS_TMP, TMP_REG
		RES_R8, // REGS_TMP, ALU_RES, ALU_REGS
		RL_R8, // REGS_TMP, ALU_RL, ALU_REGS
		RLC_R8, // REGS_TMP, ALU_RLC, ALU_REGS
		RR_R8, // REGS_TMP, ALU_RR, ALU_REGS
		RRC_R8, // REGS_TMP, ALU_RRC, ALU_REGS
		SET_R8, // REGS_TMP, ALU_SET, ALU_REGS
		SLA_R8, // REGS_TMP, ALU_SLA, ALU_REGS
		SLL_R8, // REGS_TMP, ALU_SLL, ALU_REGS
		SRA_R8, // REGS_TMP, ALU_SRA, ALU_REGS
		SRL_R8, // REGS_TMP, ALU_SRL, ALU_REGS
	};

	// Flag result lookup tables
	static bool tables_initialised;
	static u8 SZ[256];       /* zero and sign flags */
	static u8 SZ_BIT[256];   /* zero, sign and parity/overflow (=zero) flags for BIT opcode */
	static u8 SZP[256];      /* zero, sign and parity flags */
	static u8 SZHV_inc[256]; /* zero, sign, half carry and overflow flags INC r8 */
	static u8 SZHV_dec[256]; /* zero, sign, half carry and overflow flags DEC r8 */
	static u8 SZHVC_add[2*256*256];
	static u8 SZHVC_sub[2*256*256];

	static const u16 insts[5*256 + 4][17];
	static const u8 jr_conditions[8][2];
	static const u8 jp_conditions[8][2];
	static constexpr unsigned CB_OFFSET = 1 * 256;
	static constexpr unsigned ED_OFFSET = 2 * 256;
	static constexpr unsigned FD_OFFSET = 3 * 256;
	static constexpr unsigned FDCB_OFFSET = 4 * 256;
	static constexpr unsigned M1 = 5 * 256 + 0;
	static constexpr unsigned DD_FD_CB = 5 * 256 + 1;
	static constexpr unsigned TAKE_IRQ = 5 * 256 + 2;
	static constexpr unsigned TAKE_NMI = 5 * 256 + 3;
	static constexpr unsigned END = 0x8000;
	static constexpr unsigned HL_OFFSET = 0;
	static constexpr unsigned IX_OFFSET = 1;
	static constexpr unsigned IY_OFFSET = 2;
	// Flags
	static constexpr unsigned CF = 0x01;
	static constexpr unsigned NF = 0x02;
	static constexpr unsigned PF = 0x04;
	static constexpr unsigned VF = 0x04;
	static constexpr unsigned XF = 0x08;
	static constexpr unsigned HF = 0x10;
	static constexpr unsigned YF = 0x20;
	static constexpr unsigned ZF = 0x40;
	static constexpr unsigned SF = 0x80;

	u16               m_address_bus;
	u8                m_data_bus;
	u8                m_instruction_step;
	u16               m_instruction_offset;
	u16               m_instruction;
	u8                m_m1_wait_states; // Wait states during an M1 cycle (default 0)
	u8                m_ir;
	u8                m_act;    // ACT input into ALU
	u8                m_tmp;    // TMP input into ALU
	u8                m_alu;    // ALU output
	PAIR              m_prvpc;
	PAIR              m_pc;
	PAIR              m_sp;
	PAIR              m_af;
	PAIR              m_bc;
	PAIR              m_de;
	u8                m_hl_offset;     // Are we using HL, or IX or IY instead of HL
	PAIR              m_hl_index[3];   // HL, IX, IY
	PAIR              m_wz;
	PAIR              m_af2;
	PAIR              m_bc2;
	PAIR              m_de2;
	PAIR              m_hl2;
	u8                m_r;
	u8                m_r2;      // Keeps bit 7 of what was stored in R
	u8                m_iff1;
	u8                m_iff2;
	bool              m_check_wait;
	u8                m_halt;
	u8                m_im;
	u8                m_i;
	u8                m_nmi_state;          // nmi line state
	bool              m_nmi_pending;        // nmi pending
	u8                m_irq_state;          // irq line state
	int               m_wait_state;         // wait line state
	int               m_busrq_state;        // bus request line state
	bool              m_after_ei;           // are we in the EI shadow?
	bool              m_after_ldair;        // same, but for LD A,I or LD A,R
	int               m_icount;

	// Temporary state for the debugger
	u8                m_rtemp;

	void setup_flag_tables();
	u16 adc16(u16 arg1, u16 arg2);
	u16 add16(u16 arg1, u16 arg2);
	u16 sbc16(u16 arg1, u16 arg2);
	inline void end_instruction()
	{
		m_instruction = M1;
		m_instruction_offset = 0;
		m_instruction_step = 0;
		m_hl_offset = HL_OFFSET;
	}
	void leave_halt();
	void check_interrupts();
	void a_act();
	void alu_a();
	void alu_adc();
	void alu_add();
	void alu_and();
	void alu_bit();
	void alu_cp();
	void alu_dec();
	void alu_inc();
	void alu_or();
	void alu_regd();
	void alu_regs();
	void alu_res();
	void alu_rl();
	void alu_rlc();
	void alu_rr();
	void alu_rrc();
	void alu_sbc();
	void alu_set();
	void alu_sla();
	void alu_sll();
	void alu_sra();
	void alu_srl();
	void alu_sub();
	void alu_xor();
	void bc_wz();
	void db_a();
	void db_r16h();
	void db_r16l();
	void db_regd();
	void db_regd0();
	void db_regd_input();
	void db_tmp();
	void db_w();
	void db_z();
	void de_wz();
	void dec_sp();
	void inc_sp();
	void read();
	void regd_tmp();
	void regs_tmp();
	void sp_out();
	void tmp_reg();
	void wz_out_inc();
};

DECLARE_DEVICE_TYPE(Z80LLE, z80lle_device)


#endif // MAME_CPU_Z80_Z80LLE_H
