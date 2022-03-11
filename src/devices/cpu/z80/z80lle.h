// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
#ifndef MAME_CPU_Z80_Z80LLE_H
#define MAME_CPU_Z80_Z80LLE_H

#pragma once

#include "emu.h"
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
	memory_access<16, 0, 0, ENDIANNESS_LITTLE>::cache m_args;
	memory_access<16, 0, 0, ENDIANNESS_LITTLE>::cache m_opcodes;
	memory_access<16, 0, 0, ENDIANNESS_LITTLE>::specific m_data;
	memory_access<16, 0, 0, ENDIANNESS_LITTLE>::specific m_io;

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
		A_DB,
		A_W,
		ADC_DB,
		ADC_R8,
		ADC16,
		ADD_DB,
		ADD_R8,
		ADD16,
		AND_DB,
		AND_R8,
		BIT_DB,
		BIT_R8,
		CCF,
		CP_DB,
		CP_R8,
		CPD,
		CPI,
		CPL,
		DAA,
		DB_A,
		DB_R16H,
		DB_R16L,
		DB_R16L_READ_S_PC,
		DB_R16L_READ_S_SP_INC,
		DB_R16L_READ_S_WZ,
		DB_REGD,
		DB_REGD_INPUT,
		DB_REGD0,
		DB_W,
		DB_W_CALL_COND,
		DB_W_JP_COND,
		DB_W_A_DB_WRITE_S_WZ_INC,
		DB_W_L_DB_WRITE_S_WZ_INC,
		DB_W_READ_S_WZ_INC,
		DB_W_WZ_PC,
		DB_Z,
		DB_Z_READ_S_PC,
		DB_Z_READ_S_SP,
		DB_Z_READ_S_SP_INC,
		DB_Z_READ_S_VEC,
		DEC_DB_WRITE_S,
		DEC_R8,
		DEC_R16,
		DECODE_X2,
		DI,
		DISP_WZ2,
		DISP_WZ2_READ_OP2_S,
		DISP_WZ5,
		DISP_WZ5_READ_S_WZ,
		DJNZ,
		EI,
		EX_AF_AF,
		EX_DE_HL,
		EX_SP_WRITE_S,
		EXX,
		H_DB_WRITE_S_WZ,
		HALT,
		HL_PC,
		HL_WZ,
		IM,
		INC_DB_WRITE_S,
		INC_R8,
		INC_R16,
		IND,
		INI,
		INPUT_A,
		INPUT_REGD,
		INPUT_S_BC,
		INPUT_S_WZ_INC,
		JP_COND,
		JR_COND,
		LD_A_I,
		LD_A_R,
		LD_I_A,
		LD_R_A,
		LD_SP_HL,
		LDD,
		LDI,
		NEG,
		NMI,
		OR_DB,
		OR_R8,
		OUTD,
		OUTI,
		OUTPUT_S_BC,
		OUTPUT_S_WZ_INC,
		PC_OUT_INC_M1,
		PCH_WRITE_S_SP_DEC,
		PCL_WRITE_S_SP_DEC,
		R16H_DB,
		R16H_WRITE_S_SP_DEC,
		R16H_WRITE_S_WZ,
		R16L_WRITE_S_SP_DEC,
		R16L_WRITE_S_WZ_INC,
		READ_OP1_S,
		READ_OP_S,
		READ_OP_IRQ,
		READ_S_BCDE_WZ_INC,
		READ_S_HL,
		READ_S_PC,
		READ_S_SP,
		READ_S_SP_INC,
		READ_S_VEC,
		READ_S_WZ,
		READ_S_WZ_INC,
		REFRESH,
		REFRESH_DECODE,
		REGD_DB,
		REGS_TMP_REG,
		REGS0_DB,
		REGS0_WRITE_S_HL,
		REPEAT,
		REPEATCP,
		REPEATIO,
		RES_R8,
		RES_WRITE_S,
		RET_COND,
		RETI,
		RETN,
		RL_R8,
		RL_WRITE_S,
		RLA,
		RLC_R8,
		RLC_WRITE_S,
		RLCA,
		RLD,
		RR_R8,
		RR_WRITE_S,
		RRA,
		RRC_R8,
		RRC_WRITE_S,
		RRCA,
		RRD,
		RST,
		SBC_DB,
		SBC_R8,
		SBC16,
		SCF,
		SET_R8,
		SET_WRITE_S,
		SLA_R8,
		SLA_WRITE_S,
		SLL_R8,
		SLL_WRITE_S,
		SRA_R8,
		SRA_WRITE_S,
		SRL_R8,
		SRL_WRITE_S,
		SUB_DB,
		SUB_R8,
		WRITE_BCDE_A,
		WRITE_S,
		WRITE_S_DE,
		WRITE_S_HL,
		WRITE_S_SP_DEC,
		WRITE_S_WZ,
		WRITE_S_WZ_INC,
		WZ_HL,
		WZ_PC,
		X,
		X_READ_S_PC,
		X2,
		XOR_DB,
		XOR_R8,
		ZERO_DB,
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

	static const u16 insts[5*256 + 6][10];
	static const u8 jr_conditions[8][2];
	static const u8 jp_conditions[8][2];
	static constexpr unsigned CB_OFFSET = 1 * 256;
	static constexpr unsigned ED_OFFSET = 2 * 256;
	static constexpr unsigned FD_OFFSET = 3 * 256;
	static constexpr unsigned FDCB_OFFSET = 4 * 256;
	static constexpr unsigned M1 = 5 * 256 + 0;
	static constexpr unsigned DD_FD_CB = 5 * 256 + 1;
	static constexpr unsigned TAKE_IRQ_0 = 5 * 256 + 2;
	static constexpr unsigned TAKE_IRQ_1 = 5 * 256 + 3;
	static constexpr unsigned TAKE_IRQ_2 = 5 * 256 + 4;
	static constexpr unsigned TAKE_NMI = 5 * 256 + 5;
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
	u32               m_irq_vector;         // For multibyte irq "vector" in interrupt mode 0
	                                        // Reading the additional bytes is done with regular memory reads but the rest
											// of the system should be designed to not have the memory respond.
	bool              m_in_irq_acknowledge;
	bool              m_reset;              // when CPU was reset

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
		m_in_irq_acknowledge = false;
	}
	void leave_halt();
	void check_interrupts();
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
	void db_r16h();
	void db_r16l();
	void db_regd();
	void db_regd0();
	void db_regd_input();
	void input_s(u16 address);
	void output_s(u16 address);
	void r16h_db();
	void r16l_db();
	void read_op_s();
	void read_s(u16 address);
	u8 regd();
	void regd(u8 data);
	u8 regs();
	void regs(u8 data);
	void regs0(u8 data);
	void regs0_db();
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
