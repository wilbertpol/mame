// license:BSD-3-Clause
// copyright-holders:Wilbert Pol

#ifndef MAME_CPU_RAVEN_RAVEN_DASM_H
#define MAME_CPU_RAVEN_RAVEN_DASM_H

#pragma once

class raven_disassembler : public util::disasm_interface
{
public:
	raven_disassembler() = default;
	virtual ~raven_disassembler() = default;

	virtual u32 opcode_alignment() const override;
	virtual offs_t disassemble(std::ostream &stream, offs_t pc, const data_buffer &opcodes, const data_buffer &params) override;

private:
	static const char *const o_bus_destination[64];
	static const char *const m_mf_source[64];
	static const char *const o_bus_control[8];
	static const char *const abj[8];
	static const char *const q_control[4];
	static const char *const byte_operation[4];
	static const char *const jump_operation[8];
	static const char *const dispatch_src[4];
	static const char *const dispatch_op[4];
	static constexpr u8 A_USED = (1 << 0);
	static constexpr u8 M_USED = (1 << 1);
	struct desc_info {
		const char *name;
		const u8 source_used;
	};
	static const desc_info condition[16];
	static const desc_info alu_operation[32];

	void disassemble_alu(std::ostream &stream, u64 op);
	void disassemble_byte(std::ostream &stream, u64 op);
	void disassemble_jump(std::ostream &stream, u64 op);
	void disassemble_dispatch(std::ostream &stream, u64 op);
	void destination(std::ostream &stream, u64 op);
	void sources(std::ostream &stream, u64 op);
	void conditional(std::ostream &stream, u64 op);
};

#endif
