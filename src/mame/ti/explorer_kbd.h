// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/**********************************************************************

    TI Explorer keyboard.

**********************************************************************/

#ifndef MAME_TI_EXPLORER_KBD_H
#define MAME_TI_EXPLORER_KBD_H

#pragma once

#include "diserial.h"
#include "machine/keyboard.h"


class explorer_keyboard_device : public device_t,
	public device_buffered_serial_interface<16U>,
	protected device_matrix_keyboard_interface<8U>
{
public:
	explorer_keyboard_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	auto txd_handler() { return m_write_txd.bind(); }

	void rxd_w(int state) { device_buffered_serial_interface::rx_w(state); }

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

	virtual void tra_callback() override { m_write_txd(transmit_register_get_data_bit()); }

	virtual void key_make(u8 row, u8 column) override { transmit_byte(0x80 | ((row << 4) | column)); }
	virtual void key_break(u8 row, u8 column) override { transmit_byte((row << 4) | column); }

private:
	virtual void received_byte(u8 byte) override;

	devcb_write_line m_write_txd;
};

DECLARE_DEVICE_TYPE(EXPLORER_KEYBOARD, explorer_keyboard_device)

#endif // MAME_TI_EXPLORER_KBD_H
