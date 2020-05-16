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

	template <typename... T> void set_memory_map(T &&... args) { set_addrmap(AS_PROGRAM, std::forward<T>(args)...); }
	template <typename... T> void set_m1_map(T &&... args) { set_addrmap(AS_OPCODES, std::forward<T>(args)...); }
	template <typename... T> void set_io_map(T &&... args) { set_addrmap(AS_IO, std::forward<T>(args)...); }
	void set_m1_wait_states(u8 m1_wait_states) { m_m1_wait_states = m1_wait_states; }
	auto irqack_cb() { return m_irqack_cb.bind(); }
	auto refresh_cb() { return m_refresh_cb.bind(); }
	auto halt_cb() { return m_halt_cb.bind(); }
	auto mreq_cb() { return m_mreq_cb.bind(); }
	auto iorq_cb() { return m_iorq_cb.bind(); }
	auto rd_cb() { return m_rd_cb.bind(); }
	auto wr_cb() { return m_wr_cb.bind(); }
	auto m1_cb() { return m_m1_cb.bind(); }
	auto address_bus_cb() { return m_address_bus_cb.bind(); }

protected:
	z80lle_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// device_execute_interface overrides
	virtual u32 execute_min_cycles() const noexcept override { return 2; }
	virtual u32 execute_max_cycles() const noexcept override { return 16; }
	virtual u32 execute_input_lines() const noexcept override { return 4; }
	virtual u32 execute_default_irq_vector(int inputnum) const noexcept override { return 0xff; }
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
	const address_space_config m_opcodes_config;
	const address_space_config m_io_config;
	address_space *m_program;
	address_space *m_opcodes;
	address_space *m_io;
	memory_access_cache<0, 0, ENDIANNESS_LITTLE> *m_cache;
	memory_access_cache<0, 0, ENDIANNESS_LITTLE> *m_opcodes_cache;
	devcb_write_line m_irqack_cb;
	devcb_write8 m_refresh_cb;
	devcb_write_line m_halt_cb;
	devcb_write_line m_mreq_cb;
	devcb_write_line m_iorq_cb;
	devcb_write_line m_rd_cb;
	devcb_write_line m_wr_cb;
	devcb_write_line m_m1_cb;
	devcb_write16 m_address_bus_cb;

	// Sub instructions
	enum {
		UNKNOWN=0,
		A_DB,        // register A to data bus, also writes to W always?
		A_W,         // register A to W
		ADC16,       // 16bit addition with carry, takes 7 cycles
		ADD16,       // 16bit addition, takes 7 cycles
		BC_WZ,       // Store BC in WZ
		CALL_COND,   // Check condition for CALL, takes 1 cycle when condition is true
		CCF,         // CCF
		CPD,         // Set flags and update pointers and counter, takes 5 cycles
		CPI,         // Set flags and update pointers and counter, takes 5 cycles
		CPL,         // CPL
		DAA,         // DAA
		DB_A,        // Store data bus in A
		DB_R16H,     // Store data bus in high 8 bits of 16 bit register, takes no cycles
		DB_R16L,     // Store data bus in low 8 bits of 16 bit register, takes no cycles
		DB_REGD,     // Store data bus in 8bit register (bits ..xxx...), takes no cycles
		DB_REGD0,    // Store data bus in 8bit register (bits ..xxx...), not to index registers, takes no cycles
		DB_REGD_INPUT, // Store data bus in 8bit register (bits ..xxx...)
		DB_W,        // Store data bus in W, takes no cycles
		DB_Z,        // Store data bus in Z, takes no cycles
		DE_WZ,       // Store DE in WZ
		DEC_R8,      // Decrement an 8 bit register
		DEC_R16,     // Decrement a 16 bit register, takes 2 cycles
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
		HL_PC,       // Store HL in PC
		HL_WZ,       // Store HL in WZ
		IM,          // IM
		INC_R8,      // Increment an 8 bit register
		INC_R16,     // Increment a 16 bit register, takes 2 cycles
		IND,         // Set flags and update pointers and counter, takes no cycles
		INI,         // Set flags and update pointers and counter, takes no cycles
		INPUT_A,     // Read data bus from input, store in A, takes no cycles
		INPUT_REGD,  // Read data bus from input, store in 8 bit register, takes no cycles
		INPUT_S_BC,  // Put BC on address bus, assert IORQ and RD signals for input cycle, takes 4 cycles
		INPUT_S_WZ_INC, // Put WZ on address bus, assert IORQ and RD signals for inoput cycle, takes 4 cycles
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
		OUTD,        // Set flags and update pointers and counter and prepare for I/O, takes no cycles
		OUTI,        // Set flags and update pointers and counter and prepare for I/O, takes no cycles
		OUTPUT_S_BC, // Put BC on address bus, assert IORQ and WR signals for output, takes 4 cycles
		OUTPUT_S_WZ_INC, // Put WZ on address, increment WZ, assert IORQ and WR signals for output, takes 4 cycles
		PC_OUT,      // Put PC on address bus, takes 1 cycle
		PC_OUT_INC_M1,  // Put PC on address bus, assert M1, takes 1 cycle, increment PC
		PCH_DB,      // Put PC 8 high bits on data bus
		PCL_DB,      // Put PC 8 low bits on data bus
		R16H_DB,     // Put high 8 bits of 16 bit register on data bus
		R16L_DB,     // Put low 8 bits of 16 bit register on data bus
		READ_OP1_S,  // Put PC on address bus, increment PC, assert M1, MREQ, and RD signals, takes 2 cycles
		READ_OP_S,   // Assert MREQ and RD signals for opcodde read, takes 1 cycle
		READ_OP2_S,  // Assert MREQ and signals for opcode read as part of DD/FD CB dd xx instructions, takes 2 cycles
		READ_OP_IRQ, // Special opcode reading while taking an interrupt
		READ_S_HL,   // Put HL on address bus, assert MREQ and RD signals for read cycle, takes 3 cycles
		READ_S_PC,   // Put PC on address bus, increment PC, assert MREQ and RD signals for read cycle, takes 3 cycles
		READ_S_SP,   // Put SP on address bus, assert MREQ and RD signals for read cycle, takes 3 cycles
		READ_S_SP_INC,   // Put SP on address bus, increment SP, assert MREQ and RD signals for read cycle, takes 3 cycles
		READ_S_WZ,   // Put WZ on address bus, assert MREQ and RD signals for read cycle, takes 3 cycles
		READ_S_WZ_INC, // Put WZ on address bus, increment WZ, assert MREQ and RD signals for read cycle, takes 3 cycles
		REFRESH,     // Refresh RAM, takes 2 cycles
		REFRESH_DECODE, // Refresh RAM and decode instruction, takes 2 cycles
		REGD_DB,     // 8 bit source register (bits ..xxx...) to data bus
		REGS0_DB,    // 8 bit source register (bits .....xxx) to data bus (not from index registers)
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
		WRITE_S,     // Assert MREQ and WR signals for write, takes 2 cycles
		WRITE_S_DE,  // Put DE on address bus, assert MREQ and WR signals for write, takes 3 cycles
		WRITE_S_HL,  // Put HL on address bus, assert MREQ and WR signals for write, takes 3 cycles
		WRITE_S_SP_DEC, // Decrement SP, put SP on address bus, assert MREQ and WR signals for write, takes 3 cycles
		WRITE_S_WZ,  // Put WZ on address bus, assert MREQ and WR signals for write, takes 3 cycles
		WRITE_S_WZ_INC, // Put WZ on address bus, increment WZ, assert MREQ and WR signals for write, takes 3 cycles
		WZ_HL,       // Store contents of WZ in HL
		WZ_PC,       // Store contents of WZ in PC
		X,           // Do nothing, takes 1 cycle
		X2,          // Do nothing, takes 2 cycles
		ZERO_DB,     // put all zeroes on the data bus

		// Testing optimizations
		ADD_DB, // Perform ADD operation on A and data bus
		ADD_R8, // Perform ADD operation on A and 8 bit register
		ADC_DB, // Perform ADC operation on A and data bus
		ADC_R8, // Perform ADC operation on A and 8 bit register
		SUB_DB, // Perform SUB operation on A and data bus
		SUB_R8, // Perform SUB operation on A and 8 bit register
		SBC_DB, // Perform SBC operation on A and data bus
		SBC_R8, // Perform SBC operation on A and 8 bit register
		AND_DB, // Perform AND operation on A and data bus
		AND_R8, // Perform AND operation on A and 8 bit register
		XOR_DB, // Perform XOR operation on A and data bus
		XOR_R8, // Perform XOR operation on A an 8 bit register
		OR_DB,  // Perform OR operation on A and data bus
		OR_R8,  // Perform OR operation on A and 8 bit register
		CP_DB,  // Perform CP operation on A and data bus
		CP_R8,  // Perform CP operation on A and 8 bit register
		BIT_DB, // Perform BIT operation on data bus, takes 1 cycle
		BIT_R8, // Perform BIT operation on 8 bit register
		REGS_TMP_REG,
		RES_DB, // Perform RES operation on data bus, takes 2 cycles
		RES_DB_REGS0, // Perform RES operation on data bus, takes 2 cycles, also stores result in 8 bit register
		RES_R8, // Perform RES operation on 8 bit register
		RL_DB,  // Perform RL operation on data bus, takes 2 cycles
		RL_DB_REGS0,  // Perform RL operation on data bus, takes 2 cycles, also stores result in 8 bit register
		RL_R8,  // Perform RL operation on 8 bit register
		RLC_DB, // Perform RLC operation on data bus, takes 2 cycles
		RLC_DB_REGS0, // Perform RLC operation on data bus, takes 2 cycles, also stores result in 8 bit register
		RLC_R8, // Perform RLC operation on 8 bit register
		RR_DB,  // Perform RR operation on data bus, takes 2 cycles
		RR_DB_REGS0,  // Perform RR operation on data bus, takes 2 cycles, also stores result in 8 bit register
		RR_R8,  // Perform RR operation on 8 bit register
		RRC_DB, // Perform RRC operation on data bus, takes 2 cycles
		RRC_DB_REGS0, // Perform RRC operation on data bus, takes 2 cycles, also stores result in 8 bit register
		RRC_R8, // Perform RRC operation on 8 bit register
		SET_DB, // Perform SET operation on data bus, takes 2 cycles
		SET_DB_REGS0, // Perform SET operation on data bus, takes 2 cycles, also stores result in 8 bit register
		SET_R8, // Perform SET operation on 8 bit register
		SLA_DB, // Perform SLA operation on data bus, takes 2 cycles
		SLA_DB_REGS0, // Perform SLA operation on data bus, takes 2 cycles, also stores result in 8 bit register
		SLA_R8, // Perform SLA operation on 8 bit register
		SLL_DB, // Perform SLL operation on data bus, takes 2 cycles
		SLL_DB_REGS0, // Perform SLL operation on data bus, takes 2 cycles, also stores result in 8 bit register
		SLL_R8, // Perform SLL operation on 8 bit register
		SRA_DB, // Perform SRA operation on data bus, takes 2 cycles
		SRA_DB_REGS0, // Perform SRA operation on data bus, takes 2 cycles, also stores result in 8 bit register
		SRA_R8, // Perform SRA operation on 8 bit register
		SRL_DB, // Perform SRL operation on data bus, takes 2 cycles
		SRL_DB_REGS0, // Perform SRL operation on data bus, takes 2 cycles, also stores result in 8 bit register
		SRL_R8, // Perform SRL operation on 8 bit register
		DEC_DB, // Perforn DEC operation on data bus, takes 2 cycles
		INC_DB, // Perform INC operation on data bus, takes 2 cycles
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
	bool              m_mreq;               // MREQ output line state (active low)
	bool              m_iorq;               // IORQ output line state (active low)
	bool              m_rd;                 // RD output line state (active low)
	bool              m_wr;                 // WR output line state (active low)
	bool              m_m1;                 // M1 output line state (active low)
	bool              m_opcode_read;        // Should we read from opcode_cache

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
	void a_db();
	void a_w();
	void adc16();
	void add16();
	void sbc16();
	void alu_adc(u8 arg);
	void alu_add(u8 arg);
	void alu_and(u8 arg);
	void alu_bit(u8 arg);
	void alu_cp(u8 arg);
	u8 alu_dec(u8 arg);
	u8 alu_inc(u8 arg);
	void alu_or(u8 arg);
	void alu_regd(u8 data);
	void alu_regs(u8 data);
	void alu_regs0(u8 data);
	u8 alu_res(u8 arg);
	u8 alu_rl(u8 arg);
	u8 alu_rlc(u8 arg);
	u8 alu_rr(u8 arg);
	u8 alu_rrc(u8 arg);
	void alu_sbc(u8 arg);
	u8 alu_set(u8 arg);
	u8 alu_sla(u8 arg);
	u8 alu_sll(u8 arg);
	u8 alu_sra(u8 arg);
	u8 alu_srl(u8 arg);
	void alu_sub(u8 arg);
	void alu_xor(u8 arg);
	void bc_wz();
	void db_a();
	void db_ir();
	void db_r16h();
	void db_r16l();
	void db_regd();
	void db_regd0();
	void db_regd_input();
	void input_s(u16 address);
	void output_s(u16 address);
	void read();
	void read_op_s();
	void read_s();
	void read_s(u16 address);
	u8 regd_tmp();
	u8 regs_tmp();
	void sp_out();
	void tmp_reg(u8 data);
	void write_s();
	void write_s(u16 address);
	void decode();

	inline void set_m1() { m_m1 = true; m_m1_cb(!m_m1); }
	inline void clear_m1() { m_m1 = false; m_m1_cb(!m_m1); }
	inline void set_mreq() { m_mreq = true; m_mreq_cb(!m_mreq); }
	inline void clear_mreq() { m_mreq = false; m_mreq_cb(!m_mreq); }
	inline void set_iorq() { m_iorq = true; m_iorq_cb(!m_iorq); }
	inline void clear_iorq() { m_iorq = false; m_iorq_cb(!m_iorq); }
	inline void set_rd() { m_rd = true; m_rd_cb(!m_rd); }
	inline void clear_rd() { m_rd = false; m_rd_cb(!m_rd); }
	inline void set_wr() { m_wr = true; m_wr_cb(!m_wr); }
	inline void clear_wr() { m_wr = false; m_wr_cb(!m_wr); }
	inline void set_rfsh() { /* TODO */ }
	inline void clear_rfsh() { /* TODO */ }
};

DECLARE_DEVICE_TYPE(Z80LLE, z80lle_device)


#endif // MAME_CPU_Z80_Z80LLE_H
