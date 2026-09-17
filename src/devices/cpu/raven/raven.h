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

	u32 config_register_r();
	void config_register_w(offs_t offset, u32 data, u32 mem_mask);

	// The CPU board's lamps. Both the six-lamp state code and the fault LED are
	// driven from registers that live in here (the MCR and the NuBus
	// configuration register), so the processor signals them out and the board
	// owns the lamps - see update_leds(). The state code is active high by the
	// time it gets here; the MCR's own bits are low true.
	auto out_state_leds_cb() { return m_state_leds.bind(); }
	auto out_fault_led_cb() { return m_fault_led.bind(); }

	static constexpr int AS_LOCAL_BUS = AS_OPCODES + 1;

	// Table 4-19's condition 01011, "Bus error on last transfer attempt". One
	// flag for both hardware paths on purpose: a local-bus board asserts BERR-
	// (the memory board does that for a parity failure, via NUERR-), while a
	// card in a slot below 3 is not on the local bus at all and can only report
	// an error as a NuBus TM0-/TM1- termination. The microcode has a single
	// condition for the two, so this is the whole of what it can see.
	void assert_bus_error() { m_bus_error = true; }

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

	void update_leds();

	address_space_config m_program_config;
	address_space_config m_data_config;
	address_space_config m_local_bus_config;
	devcb_write8 m_state_leds;
	devcb_write_line m_fault_led;
	memory_view m_inst_view;
	memory_access<ADDRESS_BITS, 3, -3, ENDIANNESS_BIG>::specific m_program;
	memory_access<EXTERNAL_ADDRESS_BITS, 2, 0, ENDIANNESS_LITTLE>::specific m_data;
	memory_access<EXTERNAL_ADDRESS_BITS, 2, 0, ENDIANNESS_LITTLE>::specific m_local_bus;
	bool m_local_bus_miss = false;

	required_shared_ptr<u64> m_control_store; // 16K x 56 bits RAM
	u16 m_pc = 0; // 14 bits
	u16 m_prev_pc = 0;
	u16 m_next_pc = 0;
	bool m_n = false;
	u64 m_ir = 0;
	u32 m_a = 0; // 32 bits
	u32 m_a_mem[0x400]{}; // 1024 x 32 bits
	u32 m_m = 0; // 32 bits
	u32 m_m_mem[0x40]{}; // 64 x 32 bits
	u32 m_t_memory[16]{}; // Tag-classifier RAM
	u32 m_o_bus = 0;
	u32 m_q = 0;
	u32 m_md = 0;
	u8 m_sp = 0;
	u32 m_stack[0x40]{}; // 64 x 20 bits
	u32 m_mcr = 0;
	u32 m_config_register = 0;
	u32 m_imod_lo = 0;
	u32 m_imod_hi = 0;
	u32 m_vma = 0;
	u32 m_pdl[0x400]{}; // 1024 x 32 bits
	u16 m_pdl_pointer = 0; // 10 bits
	u16 m_pdl_index = 0; // 10 bits
	u32 m_lc = 0; // 32 bits?
	u32 m_ibuf = 0; // 32 bits?

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
	u32 m_vma_lvl1_map[0x1000]{};

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
	u32 m_vma_lvl2_control[0x1000]{};

	// Virtual memory levvel 2 map - 4096 x 32 bits?
	// xxxxxxxx xx------ -------- -------- Unused (0)
	// -------- --xxxxxx xxxxxxxx xxxxxxxx Physical page number
	u32 m_vma_lvl2_map[0x1000]{};

	u32 m_dispatch[0x1000]{}; // 4096 x 17 bits
	u16 m_dispatch_constant = 0; // 10 bits
	// GC volatility of the page most recently translated (level-2 control bits
	// 12:11), used by the GC-volatility dispatch in execute_dispatch().
	u8 m_cached_gc_volatility = 0;
	// Level-1 map output latch - see the MEMORY-MAP-LEVEL-1 M source and
	// update_cached_lvl1_from_md().
	u32 m_cached_lvl1 = 0;
	bool m_page_fault = false;
	u32 m_read_data = 0;
	u8 m_memory_busy_counter = 0;
	bool m_read_pending = false;

	// Macroinstruction-chaining POPJ prefetch, deferred - see handle_popj14().
	u32 m_pj14_fetch_vma = 0;
	u32 m_pj14_fetch_addr = 0;
	bool m_pj14_fetch_pending = false;
	bool m_pj14_fetch_go = false;
	u16 m_pending_interrupts = 0;
	bool m_bus_error = false;

	int m_icount = 0;

	enum {
		MEM_READ,
		MEM_WRITE
	};

	void program_map(address_map &map) ATTR_COLD;
	// Neither the NuBus nor the local bus carries an "unmapped" signal - nothing
	// on the backplane tells the processor that no card answered a cycle. The
	// processor works that out for itself, by timing the cycle out, so these are
	// its own behavior and belong in its own space configuration rather than in
	// a map whatever board it sits on has to remember to supply.
	void data_map(address_map &map) ATTR_COLD;
	void local_bus_map(address_map &map) ATTR_COLD;

	u32 nubus_unmapped_r(offs_t offset, u32 mem_mask);
	void nubus_unmapped_w(offs_t offset, u32 data, u32 mem_mask);
	u32 local_bus_miss_r(offs_t offset, u32 mem_mask);
	void local_bus_miss_w(offs_t offset, u32 data, u32 mem_mask);
	bool memory_cycle_enabled();
	void read();
	void write();
	u32 unmapped_mem_mask() const;
	void read_unmapped();
	void write_unmapped();
	void read_unmapped_byte();
	void write_unmapped_byte();
	template <int Action> u32 vm_resolve_address();
	void update_cached_lvl1_from_md();
	u16 map2_addr();
	u32 get_m_source();
	void add32(u32 a, u32 m, u32 carry_in, u32 &res, u32 &carry_out, u32 &fixnum_overflow);
	void sub32(u32 a, u32 m, u32 carry_in, u32 &res, u32 &carry_out, u32 &fixnum_overflow);
	void alu_operation(u32 &result, u32 &carry_out, u32 &fixnum_overflow);
	bool is_condition(u32 alu_out, u32 carry_out, u32 fixnum_overflow);
	bool active_int() const;
	void push(u32 pc);
	void pop(bool after_next);
	void handle_popj14(bool after_next);
	void service_pj14_fetch();
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
