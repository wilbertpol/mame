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
	void disassemble_alu(std::ostream &stream, u64 op);
	void disassemble_byte(std::ostream &stream, u64 op);
	void disassemble_jump(std::ostream &stream, u64 op);
	void disassemble_dispatch(std::ostream &stream, u64 op);
	void destination(std::ostream &stream, u64 op);
	void sources(std::ostream &stream, u64 op);
	void conditional(std::ostream &stream, u64 op);
};

#endif
