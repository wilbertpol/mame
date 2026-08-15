// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/**********************************************************************

    Raven CPU (TI Explorer I cpu)

**********************************************************************/

#ifndef MAME_CPU_RAVEN_RAVEN_H
#define MAME_CPU_RAVEN_RAVEN_H

#pragma once


class raven_cpu_device : public cpu_device
{
public:
	raven_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	u32 nubus_flag_r();
	void irq_w(offs_t offset, u32 data);

	u32 nubus_unmapped_r(offs_t offset, u32 mem_mask);
	void nubus_unmapped_w(offs_t offset, u32 data, u32 mem_mask);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	virtual u64 execute_clocks_to_cycles(u64 clocks) const noexcept override { return (clocks + 4 - 1) / 4; }
	virtual u64 execute_cycles_to_clocks(u64 cycles) const noexcept override { return (cycles * 4); }
	virtual u32 execute_min_cycles() const noexcept override { return 1; } // TODO
	virtual u32 execute_max_cycles() const noexcept override { return 2; } // TODO
	virtual void execute_run() override;
	virtual void execute_set_input(int linenum, int state) override;

	virtual space_config_vector memory_space_config() const override;

	virtual void state_string_export(const device_state_entry &entry, std::string &str) const override;

	virtual std::unique_ptr<util::disasm_interface> create_disassembler() override;

private:
	static constexpr u8 ADDRESS_BITS = 14;
	static constexpr u8 EXTERNAL_ADDRESS_BITS = 32;

	address_space_config m_program_config;
	address_space_config m_data_config;
	memory_view m_inst_view;
	memory_access<ADDRESS_BITS, 3, -3, ENDIANNESS_BIG>::specific m_program;
	memory_access<EXTERNAL_ADDRESS_BITS, 2, 0, ENDIANNESS_BIG>::specific m_data;

	required_shared_ptr<u64> m_control_store; // 16K x 56 bits RAM
	u16 m_pc; // 14 bits
	u16 m_prev_pc;
	u16 m_next_pc;
	bool m_n;
	u64 m_ir;
	u32 m_a; // 32 bits
	u32 m_a_mem[0x400]; // 1024 x 32 bits
	u32 m_m; // 32 bits
	u32 m_m_mem[0x40]; // 64 x 32 bits
	u32 m_t_memory[16]; // Tag-classifier RAM
	u32 m_o_bus;
	u32 m_q;
	u32 m_md;
	u8 m_sp;
	u32 m_stack[0x40]; // 64 x 20 bits
	u32 m_mcr;
	u32 m_imod_lo;
	u32 m_imod_hi;
	u32 m_vma;
	u32 m_pdl[0x400]; // 1024 x 32 bits
	u16 m_pdl_pointer; // 10 bits
	u16 m_pdl_index; // 10 bits
	u32 m_lc; // 32 bits?
	u32 m_ibuf; // 32 bits?

	// Virtual memory level 1 map - 4096 x 32 bits?
	// xxxxxxxx xxxxxxxx -------- -------- Unused (0)
	// -------- -------- x------- -------- Last access mapped / unmapped cycle
	// -------- -------- -x------ -------- Last access forced / not (forced cycle)
	// -------- -------- --x----- -------- Last access write fault
	// -------- -------- ---x---- -------- Last access access fault
	// -------- -------- ----x--- -------- Map entry valid
	// -------- -------- -----x-- -------- Oldspace meta bit; 0 = oldspace / reserved
	// -------- -------- ------xx x------- Garbage collector volatility bits, bit 9 = GC valid bit)
	// -------- -------- -------- -xxxxxxx Level 2 block number
	u32 m_vma_lvl1_map[0x1000];

	// Virtual memory level 2 control - 4096 x 32 bits?
	// xxxxxxxx xxxxxxxx -------- -------- Unused (0)
	// -------- -------- x------- -------- Last access TM0
	// -------- -------- -x------ -------- Last access TM1
	// -------- -------- --x----- -------- Last access locked
	// -------- -------- ---xx--- -------- Garbage collector volatility bits
	// -------- -------- -----x-- -------- Force allowed / forced access bit
	// -------- -------- ------xx -------- Read / Write access rights
	// -------- -------- -------- xx------ Map status
	// -------- -------- -------- --xxxxxx Meta bits (software controlled memory management)
	u32 m_vma_lvl2_control[0x1000];

	// Virtual memory levvel 2 map - 4096 x 32 bits?
	// xxxxxxxx xx------ -------- -------- Unused (0)
	// -------- --xxxxxx xxxxxxxx xxxxxxxx Physical page number
	u32 m_vma_lvl2_map[0x1000];

	u32 m_dispatch[0x1000]; // 4096 x 17 bits
	u16 m_dispatch_constant; // 10 bits
	bool m_page_fault;
	u32 m_read_data;
	u8 m_memory_busy_counter;
	u16 m_pending_interrupts;
	bool m_nubus_error;

	int m_icount;

	enum {
		MEM_READ,
		MEM_WRITE
	};

	void program_map(address_map &map) ATTR_COLD;
	void read();
	void write();
	void read_unmapped();
	void write_unmapped();
	template <int Action> u32 vm_resolve_address();
	u16 map2_addr();
	u32 get_m_source();
	void add32(u32 a, u32 m, u32 carry_in, u32 &res, u32 &carry_out, u32 &fixnum_overflow);
	void sub32(u32 a, u32 m, u32 carry_in, u32 &res, u32 &carry_out, u32 &fixnum_overflow);
	void alu_operation(u32 &result, u32 &carry_out, u32 &fixnum_overflow);
	bool is_condition(u32 alu_out, u32 carry_out, u32 fixnum_overflow);
	bool active_int() const;
	void push(u32 pc);
	void pop();
	void handle_popj14();
	void perform_abj();
	u32 shifter(bool rotate_r, bool rotate_mask, int rot_count);
	void set_o_bus(u32 alu_out, u32 carry_out);
	void store_o_bus();
	void execute_alu();
	void execute_jump();
	void execute_dispatch();
	void execute_byte();
};


DECLARE_DEVICE_TYPE(RAVEN, raven_cpu_device);


#endif
