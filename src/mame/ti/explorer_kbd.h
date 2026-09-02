// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/**********************************************************************

    TI Explorer keyboard.

    The real keyboard contains its own scan/communication microcomputer
    (2243145-0001A SI General Description, section 4.4.13 "Special Chord
    Detector": "It forces a local scan/communication microcomputer
    located at the keyboard..."). That microcontroller's ROM has not
    been dumped, so this device is a high-level stand-in: it only
    emulates the byte-level serial protocol behavior needed to satisfy
    the SIB firmware, not the real key-matrix scanning or the documented
    special-chord escape sequences. Once a ROM dump of the real keyboard
    microcontroller becomes available, this should be replaced with a
    low-level emulation of that microcontroller instead.

    Known protocol byte(s), reverse-engineered rather than documented in
    the SI General Description manual:
    - 0x00 sent by the SIB is a keyboard initialization/reset code. The
      keyboard is expected to reply with 0x70 to acknowledge it. The SIB
      firmware's boot-time self-test polls the USART receive-ready status
      waiting for this reply; without it the poll never completes.
    - Key transmission: a key-down sends its scancode with the high bit
      set (scancode | 0x80); a key-up sends the bare scancode. Scancode
      values below match the SCAN-CODE-* constants in the real TI kernel
      source (keyboard-chars.lisp, KBD-MAKE-TI-TABLE) - authoritative,
      not reverse-engineered. Cross-checked against Meroko's own
      independently reverse-engineered map (svn/sib.c,
      init_sdl_to_keysym_map()): every key both sides define agrees,
      except Meroko's Tab (0x35), which the kernel source shows is
      actually Keypad-Tab - the real Tab is 0x38.

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
