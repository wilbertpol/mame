// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
#ifndef MAME_CPU_Z80_Z80LLE_H
#define MAME_CPU_Z80_Z80LLE_H

#pragma once

#include "z80daisy.h"

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

	template<class Object> devcb_base &set_irqack_cb(Object &&cb) { return m_irqack_cb.set_callback(std::forward<Object>(cb)); }
	template<class Object> devcb_base &set_refresh_cb(Object &&cb) { return m_refresh_cb.set_callback(std::forward<Object>(cb)); }
	template<class Object> devcb_base &set_halt_cb(Object &&cb) { return m_halt_cb.set_callback(std::forward<Object>(cb)); }

protected:
	z80lle_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// device_execute_interface overrides
	virtual uint32_t execute_min_cycles() const override { return 2; }
	virtual uint32_t execute_max_cycles() const override { return 16; }
	virtual uint32_t execute_input_lines() const override { return 4; }
	virtual uint32_t execute_default_irq_vector() const override { return 0xff; }
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
	address_space *m_decrypted_opcodes;
	address_space *m_io;
	direct_read_data<0> *m_direct;
	direct_read_data<0> *m_decrypted_opcodes_direct;
	devcb_write_line m_irqack_cb;
	devcb_write8 m_refresh_cb;
	devcb_write_line m_halt_cb;

	uint8_t           m_r2;
	uint8_t           m_iff1;
	uint8_t           m_iff2;
	uint8_t           m_halt;
	uint8_t           m_im;
	uint8_t           m_i;
	uint8_t           m_nmi_state;          /* nmi line state */
	uint8_t           m_nmi_pending;        /* nmi pending */
	uint8_t           m_irq_state;          /* irq line state */
	int             m_wait_state;         // wait line state
	int             m_busrq_state;        // bus request line state
	uint8_t           m_after_ei;           /* are we in the EI shadow? */
	uint8_t           m_after_ldair;        /* same, but for LD A,I or LD A,R */
	uint32_t          m_ea;

	int             m_icount;
	uint8_t           m_rtemp;
	const uint8_t *   m_cc_op;
	const uint8_t *   m_cc_cb;
	const uint8_t *   m_cc_ed;
	const uint8_t *   m_cc_xy;
	const uint8_t *   m_cc_xycb;
	const uint8_t *   m_cc_ex;

	// New internal state
	// Sub instructions
	enum {
		UNKNOWN=0,
		END,	     // End of instructions
		A_ACT,       // register A to ACT input for ALU
		A_DB,        // register A to data bus, also writes to W always?
		A_W,         // register A to W
		ADC16,       // 16bit addition with carry, takes 7 cycles
		ADD16,       // 16bit addition, takes 7 cycles
		SBC16,       // 16bit subtraction with carry, takes 7 cycles
		ALU_A,       // ALU output to A
		ALU_DB,      // ALU output to data bus
		ALU_REGS,    // ALU output to source register (bits .....xxx)
		ALU_REGS0,   // ALU output to source register (bits .....xxx), not to index registers
		ALU_REGD,    // ALU output to destination register (bits ..xxx...)
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
		CHECK_WAIT,	 // Check if a wait state should be taken, can take cycles
		DB_A,        // Store data bus in A
		DB_REG,      // Store data bus in 8bit register
		DB_REG0,     // Store data bus in 8bit register, not to index registers
		DB_R16H,     // Store data bus in high 8 bits of 16 bit register
		DB_R16L,     // Store data bus in low 8 bits of 16 bit register
		DB_TMP,      // Store data bus in TMP
		DB_W,        // Store data bus in W
		DB_Z,        // Store data bus in Z
		DE_OUT,      // Put DE on address bus, takes 1 cycle
		BC_WZ,       // Store BC in WZ
		DE_WZ,       // Store DE in WZ
		HL_WZ,       // Store HL in WZ
		DEC_SP,      // Decrement SP (for PUSH)
		DECODE,      // Decode instruction
		DISP_WZ2,    // Calculate IX/IY displacement into WZ, takes 2 cycles (in DD CB xx II instructions)
		DISP_WZ5,    // Calculate IX/IY displacement into WZ, takes 5 cycles (in DD xx instructions)
		DI,          // Reset interrupt flip flops
		EI,          // Set interrupt flip flops
		EX_DE_HL,    // Swap DE and HL
		H_DB,        // register H to data bus
		HL_OUT,      // Put HL on address bus, takes 1 cycle
		INC_SP,      // Increment SP (for POP)
		DEC_R16,     // Decrement a 16 bit register, takes 2 cycles
		INC_R16,     // Increment a 16 bit register, takes 2 cycles
		CALL_COND,   // Check condition for CALL, takes 1 cycle when condition is true
		JR_COND,     // Check condition (Z, NZ, etc) for JR and perform jump, 5 cycles when branch taken
		JP_COND,     // Check condition for JP and perform jump
		RET_COND,    // Check condition for RET, takes 1 cycle
		L_DB,        // register L to data bus
		OUTPUT,      // Write data bus to output, takes 3 cycles
		PC_INC,      // Increment PC, maybe combines this with PC_OUT
		PC_OUT,      // Put PC on address bus, takes 1 cycle
		PCH_DB,      // Put PC 8 high bits on data bus
		PCL_DB,      // Put PC 8 low bits on data bus
		R16H_DB,     // Put high 8 bits of 16 bit register on data bus
		R16L_DB,     // Put low 8 bits of 16 bit register on data bus
		READ,        // Read memory from m_address_bus, storing result in m_data_bus, takes 2 cycle
		READ_OP,     // M1 - read memory, takes 1 cycle
		READ_OP2,    // Opcode read as part of DD/FD CB dd xx instructions, takes 2 cycles
		REFRESH,     // Refresh RAM, takes 2 cycles
		REGS_DB,     // 8 bit source register (bits .....xxx) to data bus
		REGS0_DB,    // 8 bit source register (bits .....xxx) to data bus not to index registers
		REGS_TMP,    // 8 bit source register (bits .....xxx) to TMP
		REGD_TMP,    // 8 bit destination register (bits ..xxx...) to TMP
		CCF,         // CCF
		CPL,         // CPL
		DAA,         // DAA
		NEG,         // NEG
		RLA,         // RLA
		RLCA,        // RLCA
		RRA,         // RRA
		RRCA,        // RRCA
		RRD,         // RRD, takes 5 cycles
		RLD,         // RLD, takes 5 cycles
		SCF,         // SCF
		SP_OUT,      // Put SP on address bus, takes 1 cycle
		TMP_REG,     // TMP to 8 bit register
		WRITE,       // Write data bus to memory, takes 2 cycle
		WZ_INC,      // Increment WZ, maybe combine this with WZ_OUT
		WZ_OUT,      // Put WZ on address bus, takes 1 cycle
		WZ_TO_PC,    // Store contents of WZ in PC
		X,           // Do nothing, takes 1 cycle
		X2,          // Do nothing, takes 2 cycle
		CPD,         // Set flags and update pointers and counter, takes 5 cycles
		CPI,         // Set flags and update pointers and counter, takes 5 cycles
		LDD,         // Set flags and update pointers and counter, takes 2 cycles
		LDI,         // Set flags and update pointers and counter, takes 2 cycles
		REPEAT,      // Move PC 2 steps back if BC != 0, takes 5 cycles
		CPREPEAT,    // Move PC 2 steps back if BC != 0 and ZF clear, takes 5 cycles
	};

	static const u8 insts[5*256 + 2][23];
	static const u8 jr_conditions[8][2];
	static const u8 jp_conditions[8][2];
	static constexpr unsigned CB_OFFSET = 1 * 256;
	static constexpr unsigned ED_OFFSET = 2 * 256;
	static constexpr unsigned FD_OFFSET = 3 * 256;
	static constexpr unsigned FDCB_OFFSET = 4 * 256;
	static constexpr unsigned M1 = 5 * 256 + 0;
	static constexpr unsigned DD_FD_CB = 5 * 256 + 1;
	static constexpr unsigned HL_OFFSET = 0;
	static constexpr unsigned IX_OFFSET = 1;
	static constexpr unsigned IY_OFFSET = 2;

	u16               m_address_bus;
	u8                m_data_bus;
	u8                m_instruction_step;
	u16               m_instruction_offset;
	u16               m_instruction;
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
};

DECLARE_DEVICE_TYPE(Z80LLE, z80lle_device)


#endif // MAME_CPU_Z80_Z80LLE_H
