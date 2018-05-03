// license:BSD-3-Clause
// copyright-holders:Juergen Buchmueller
#ifndef MAME_CPU_Z80_Z80LLE_H
#define MAME_CPU_Z80_Z80LLE_H

#pragma once

#include "z80daisy.h"

#define MCFG_Z80_SET_IRQACK_CALLBACK(_devcb) \
	devcb = &downcast<z80lle_device &>(*device).set_irqack_cb(DEVCB_##_devcb);

#define MCFG_Z80_SET_REFRESH_CALLBACK(_devcb) \
	devcb = &downcast<z80lle_device &>(*device).set_refresh_cb(DEVCB_##_devcb);

#define MCFG_Z80_SET_HALT_CALLBACK(_devcb) \
	devcb = &downcast<z80lle_device &>(*device).set_halt_cb(DEVCB_##_devcb);

enum
{
	NSC800_RSTA = INPUT_LINE_IRQ0 + 1,
	NSC800_RSTB,
	NSC800_RSTC,
	Z80_INPUT_LINE_WAIT,
	Z80_INPUT_LINE_BOGUSWAIT, /* WAIT pin implementation used to be nonexistent, please remove this when all drivers are updated with Z80_INPUT_LINE_WAIT */
	Z80_INPUT_LINE_BUSRQ
};

enum
{
	Z80_PC = STATE_GENPC, Z80_SP = 1,
	Z80_A, Z80_B, Z80_C, Z80_D, Z80_E, Z80_H, Z80_L,
	Z80_AF, Z80_BC, Z80_DE, Z80_HL,
	Z80_IX, Z80_IY, Z80_AF2, Z80_BC2, Z80_DE2, Z80_HL2,
	Z80_R, Z80_I, Z80_IM, Z80_IFF1, Z80_IFF2, Z80_HALT,
	Z80_DC0, Z80_DC1, Z80_DC2, Z80_DC3, Z80_WZ
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

	PAIR            m_prvpc;
	PAIR            m_pc;
	PAIR            m_sp;
	PAIR            m_af;
	PAIR            m_bc;
	PAIR            m_de;
	PAIR            m_hl;
	PAIR            m_ix;
	PAIR            m_iy;
	PAIR            m_wz;
	PAIR            m_af2;
	PAIR            m_bc2;
	PAIR            m_de2;
	PAIR            m_hl2;
	uint8_t           m_r;
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
	// Main CPU state
	enum {
		FETCH,
		EXECUTE,
		MEMORY_READ,
		MEMORY_WRITE
	};
	// Fetch substates
	enum {
		M1_SET_ADDRESS,
		M1_READ_OP,
		M1_WAIT_STATE,
		M1_REFRESH,
		DECODE
	};
	// Sub instructions
	enum {
		END=0,	     // End of instructions
		A_DB,        // register A to data bus, takes 1 cycle, also writes to W always?
		A_DB_0,      // register A to data bus without taking a cycle, also writes to W always?
		A_W,         // register A to W
		CHECK_WAIT,	 // Check if a wait state should be taken, can take cycles
		DB_A,        // Store data bus in A, takes 1 cycle
		DB_REG,      // Store data bus in 8bit register, takes 1 cycle
		DB_W,        // Store data bus in W, takes 1 cycle
		DB_Z,        // Store data bus in Z, takes 1 cycle
		DI,          // Reset interrupt flip flops, takes 1 cycle
		EI,          // Set interrupt flip fliops, takes 1 cycle
		OUTPUT,      // Write data bus to output, takes 3 cycles
		PC_INC,      // Increment PC, maybe combines this with PC_OUT
		PC_OUT,      // Put PC on address bus, takes 1 cycle
		READ,        // Read memory from m_address_bus, storing result in m_data_bus, takes 1 cycle
		WRITE,       // Write data bus to memory, takes 1 cycle
		WZ_INC,      // Increment WZ, maybe combine this with WZ_OUT
		WZ_OUT,      // Put WZ on address bus, takes 1 cycle
		WZ_TO_PC,    // Store contents of WZ in PC
		X,           // Do nothing, takes 1 cycle
	};

	static const uint8_t insts[256][17];
	int               m_execution_state;
	int               m_fetch_state;
	u16               m_address_bus;
	u8                m_data_bus;
	u8                m_instruction_step;
	u8                m_ir;
};

DECLARE_DEVICE_TYPE(Z80LLE, z80lle_device)


#endif // MAME_CPU_Z80_Z80LLE_H
