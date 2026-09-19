// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/**********************************************************************

    TI Explorer keyboard.

    See explorer_kbd.h for background on the real hardware and the
    scope of this high-level emulation.

**********************************************************************/

#include "emu.h"
#include "explorer_kbd.h"
#include "machine/keyboard.ipp"

DEFINE_DEVICE_TYPE(EXPLORER_KEYBOARD, explorer_keyboard_device, "explorer_kbd", "TI Explorer Keyboard")

explorer_keyboard_device::explorer_keyboard_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock) :
	device_t(mconfig, EXPLORER_KEYBOARD, tag, owner, clock),
	device_buffered_serial_interface(mconfig, *this),
	device_matrix_keyboard_interface(mconfig, *this, "ROW0", "ROW1", "ROW2", "ROW3", "ROW4", "ROW5", "ROW6", "ROW7"),
	m_write_txd(*this)
{
}

// Every key on the keyboard, and nothing that is not on it. The positions come
// from the kernel source's own table (keyboard-chars.lisp, DEFCONSTANT
// SCAN-CODE-* and the big character table keyed by octal scan code) -
// key_make() sends 0x80 | (row << 4) | column, so a key's matrix position *is*
// its scan code and the two have to agree exactly. The octal code is quoted
// against each key so it can be rechecked without re-deriving it, and the
// gaps are the table's own "not used" entries.
//
// The 112 keys below are the whole keyboard, checked against the photograph of
// it - Figure 3-4, "Explorer Keyboard", in Introduction to the Explorer System
// (book 3-4). That figure has 113 key positions, of which 111 carry a legend:
//   - the unlabelled cap in the middle of the arrow cross is HOME (136), which
//     sits between the left and right arrows in the matrix as well as on the
//     keyboard, and has no character in the kernel table either;
//   - the unlabelled cap in the left-hand column, between ABORT and
//     HYPER/SUPER, is a blank - no scan code anywhere, so nothing here.
// Key names follow that figure's keycaps rather than the kernel's constant
// names where the two differ (TERM, LINE FEED, ITALIC LOCK).
//
// Keys with no obvious host equivalent are present but carry no PORT_CODE, so
// they read as unassigned in the input configuration and can be bound by hand
// (IPT_KEYBOARD's default sequence is empty). That is the whole keypad, the
// Symbol/Greek shifts, the lock keys, and the Lisp keys - LINE FEED, ABORT,
// BREAK, RESUME, HELP, UNDO, END, HOME, Hyper, the SYSTEM/NETWORK/STATUS/TERM
// bank and the LEFT/MIDDLE/RIGHT keys that stand in for the mouse buttons.
//
// Three things that are not the PC layout, and are deliberate:
//   - Parentheses are UNSHIFTED and brackets are shifted (codes 103/104) -
//     it is a Lisp machine keyboard.
//   - Shift-backquote is '{' and there is a separate tilde key whose shift is
//     '}' (codes 060/061).
//   - There is no Backspace key. The host's is bound to RUBOUT (code 117),
//     which is the nearest thing the keyboard has.
//
// Only the keys that were needed carry PORT_CHAR, because a character may be
// claimed once per keyboard - in particular the keypad digits must not claim
// the digits the main row already has.
static INPUT_PORTS_START( explorer_kbd )
	PORT_START("ROW0")
	PORT_BIT(0x0001, IP_ACTIVE_HIGH, IPT_UNUSED) // 000 not used
	PORT_BIT(0x0002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("HELP") // 001
	PORT_BIT(0x0004, IP_ACTIVE_HIGH, IPT_UNUSED) // 002 not used
	// The four lock keys. They send make/break like any other key and the band
	// keeps the state in LOCK-BITS; at least MODE LOCK has a lamp in the keycap
	// ("the light on the MODE LOCK key is lit", MODE-LOCK-MAPPING-ALIST), which
	// nothing here drives - the lamps are on the far side of the undumped
	// keyboard microcontroller (see explorer_kbd.h).
	PORT_BIT(0x0008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("CAPS LOCK") // 003
	PORT_BIT(0x0010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("BOLD LOCK") // 004
	PORT_BIT(0x0020, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("ITALIC LOCK") // 005
	PORT_BIT(0x0040, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("MODE LOCK") // 006
	PORT_BIT(0x0080, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("LEFT HYPER") // 007
	PORT_BIT(0x0100, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("SYSTEM") // 010
	PORT_BIT(0x0200, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("NETWORK") // 011
	PORT_BIT(0x0400, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("STATUS") // 012
	PORT_BIT(0x0800, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("TERM") // 013
	PORT_BIT(0x1000, IP_ACTIVE_HIGH, IPT_UNUSED) // 014 not used
	PORT_BIT(0x2000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("CLEAR SCREEN") // 015
	PORT_BIT(0x4000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("CLEAR INPUT") // 016
	PORT_BIT(0x8000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("UNDO") // 017

	PORT_START("ROW1")
	PORT_BIT(0x0001, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("END") // 020
	// Keyboard keys that produce mouse button characters (#\MOUSE-L-1 and so
	// on) - the band turns them into button events, they are not wired to the
	// SIB's own mouse port.
	PORT_BIT(0x0002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("LEFT") // 021
	PORT_BIT(0x0004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("MIDDLE") // 022
	PORT_BIT(0x0008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("RIGHT") // 023
	// The keyboard has exactly four function keys, codes 024-027. GDOS's key
	// function summary uses all of them as alternatives to its control chords
	// (F2 exits GDOS, F4 changes operational parameters, F8 in that table is a
	// shifted F-key rather than a fifth one).
	//
	// MAME's own defaults put UI actions on F1-F4, but they do not shadow
	// these: ui.cpp starts with UI controls inactive for any machine that has
	// an emulated keyboard, so the keystrokes reach the machine and Scroll
	// Lock is what toggles between the two.
	PORT_BIT(0x0010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F1") PORT_CODE(KEYCODE_F1) // 024
	PORT_BIT(0x0020, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F2") PORT_CODE(KEYCODE_F2) // 025
	PORT_BIT(0x0040, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F3") PORT_CODE(KEYCODE_F3) // 026
	PORT_BIT(0x0080, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F4") PORT_CODE(KEYCODE_F4) // 027
	PORT_BIT(0x0300, IP_ACTIVE_HIGH, IPT_UNUSED) // 030/031 not used
	// The Lisp modifier bank, codes 032-037. Like the shift keys these are
	// ordinary keys that send their own make/break codes - the band holds the
	// state (keyboard-chars.lisp gives them bit-15 soft characters, #o100004
	// for Left Control and so on, which KBD-BIT-15-ON folds into
	// KBD-LEFT-SHIFTS/KBD-RIGHT-SHIFTS rather than delivering as input).
	// Without them nothing above the plain character set is reachable: GDOS's
	// own status line offers Control-T to exit and Control-F for the next
	// screen, and neither can be typed.
	// Hyper (007 and 040) is still unmapped along with the keypad and the
	// other Lisp keys - there is no host key left that suggests itself.
	PORT_BIT(0x0400, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("LEFT SUPER") PORT_CODE(KEYCODE_LWIN) // 032
	PORT_BIT(0x0800, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("LEFT META") PORT_CODE(KEYCODE_LALT) // 033
	PORT_BIT(0x1000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("LEFT CONTROL") PORT_CODE(KEYCODE_LCONTROL) // 034
	PORT_BIT(0x2000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("RIGHT CONTROL") PORT_CODE(KEYCODE_RCONTROL) // 035
	PORT_BIT(0x4000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("RIGHT META") PORT_CODE(KEYCODE_RALT) // 036
	PORT_BIT(0x8000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("RIGHT SUPER") PORT_CODE(KEYCODE_RWIN) // 037

	PORT_START("ROW2")
	PORT_BIT(0x0001, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("RIGHT HYPER") // 040
	PORT_BIT(0x0002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("RESUME") // 041
	PORT_BIT(0x0004, IP_ACTIVE_HIGH, IPT_UNUSED) // 042 not used
	PORT_BIT(0x0008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("ESC") PORT_CODE(KEYCODE_ESC) PORT_CHAR(27) // 043
	// Shifted digits from the same table, codes 44-55. Nine and zero shift to
	// "(" and ")" there as well, but those already have dedicated keys of their
	// own (codes 103/104) and a character may only be claimed once, so they are
	// left off here - which costs nothing, since PORT_CHAR only decides how the
	// natural keyboard reaches a key, not what the key itself sends.
	PORT_BIT(0x0010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_1) PORT_CHAR('1') PORT_CHAR('!') // 044
	PORT_BIT(0x0020, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_2) PORT_CHAR('2') PORT_CHAR('@') // 045
	PORT_BIT(0x0040, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_3) PORT_CHAR('3') PORT_CHAR('#') // 046
	PORT_BIT(0x0080, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_4) PORT_CHAR('4') PORT_CHAR('$') // 047
	PORT_BIT(0x0100, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_5) PORT_CHAR('5') PORT_CHAR('%') // 050
	PORT_BIT(0x0200, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_6) PORT_CHAR('6') PORT_CHAR('^') // 051
	PORT_BIT(0x0400, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_7) PORT_CHAR('7') PORT_CHAR('&') // 052
	PORT_BIT(0x0800, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_8) PORT_CHAR('8') PORT_CHAR('*') // 053
	PORT_BIT(0x1000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_9) PORT_CHAR('9') // 054
	PORT_BIT(0x2000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_0) PORT_CHAR('0') // 055
	PORT_BIT(0x4000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_MINUS) PORT_CHAR('-') PORT_CHAR('_') // 056
	PORT_BIT(0x8000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_EQUALS) PORT_CHAR('=') PORT_CHAR('+') // 057

	PORT_START("ROW3")
	PORT_BIT(0x0001, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_TILDE) PORT_CHAR('`') PORT_CHAR('{') // 060
	PORT_BIT(0x0002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("~  }") PORT_CHAR('~') PORT_CHAR('}') // 061
	PORT_BIT(0x0004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("KEYPAD =") // 062
	PORT_BIT(0x0008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("KEYPAD +") // 063
	PORT_BIT(0x0010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("KEYPAD SPACE") // 064
	// Not the Tab key - that is 070. Meroko has these two the wrong way round.
	PORT_BIT(0x0020, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("KEYPAD TAB") // 065
	PORT_BIT(0x0040, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("BREAK") // 066
	PORT_BIT(0x0080, IP_ACTIVE_HIGH, IPT_UNUSED) // 067 not used
	PORT_BIT(0x0100, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("TAB") PORT_CODE(KEYCODE_TAB) PORT_CHAR('\t') // 070
	PORT_BIT(0x0200, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_Q) PORT_CHAR('q') PORT_CHAR('Q')
	PORT_BIT(0x0400, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_W) PORT_CHAR('w') PORT_CHAR('W')
	PORT_BIT(0x0800, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_E) PORT_CHAR('e') PORT_CHAR('E')
	PORT_BIT(0x1000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_R) PORT_CHAR('r') PORT_CHAR('R')
	PORT_BIT(0x2000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_T) PORT_CHAR('t') PORT_CHAR('T')
	PORT_BIT(0x4000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_Y) PORT_CHAR('y') PORT_CHAR('Y')
	PORT_BIT(0x8000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_U) PORT_CHAR('u') PORT_CHAR('U')

	PORT_START("ROW4")
	PORT_BIT(0x0001, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_I) PORT_CHAR('i') PORT_CHAR('I')
	PORT_BIT(0x0002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_O) PORT_CHAR('o') PORT_CHAR('O')
	PORT_BIT(0x0004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_P) PORT_CHAR('p') PORT_CHAR('P')
	PORT_BIT(0x0008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_OPENBRACE) PORT_CHAR('(') PORT_CHAR('[') // 103
	PORT_BIT(0x0010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_CLOSEBRACE) PORT_CHAR(')') PORT_CHAR(']') // 104
	PORT_BIT(0x0020, IP_ACTIVE_HIGH, IPT_UNUSED) // 105 not used
	PORT_BIT(0x0040, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_BACKSLASH) PORT_CHAR('\\') PORT_CHAR('|') // 106
	PORT_BIT(0x0080, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME(u8"↑") PORT_CODE(KEYCODE_UP) PORT_CHAR(UCHAR_MAMEKEY(UP)) // 107
	PORT_BIT(0x0100, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("KEYPAD 7") // 110
	PORT_BIT(0x0200, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("KEYPAD 8") // 111
	PORT_BIT(0x0400, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("KEYPAD 9") // 112
	PORT_BIT(0x0800, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("KEYPAD -") // 113
	PORT_BIT(0x1000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("ABORT") // 114
	PORT_BIT(0x6000, IP_ACTIVE_HIGH, IPT_UNUSED) // 115/116 not used
	// The keyboard has no Backspace key; RUBOUT is the nearest thing, so the
	// host's Backspace is bound here.
	PORT_BIT(0x8000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("RUBOUT") PORT_CODE(KEYCODE_BACKSPACE) PORT_CHAR(8) // 117

	PORT_START("ROW5")
	PORT_BIT(0x0001, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_A) PORT_CHAR('a') PORT_CHAR('A')
	PORT_BIT(0x0002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_S) PORT_CHAR('s') PORT_CHAR('S')
	PORT_BIT(0x0004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_D) PORT_CHAR('d') PORT_CHAR('D')
	PORT_BIT(0x0008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_F) PORT_CHAR('f') PORT_CHAR('F')
	PORT_BIT(0x0010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_G) PORT_CHAR('g') PORT_CHAR('G')
	PORT_BIT(0x0020, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_H) PORT_CHAR('h') PORT_CHAR('H')
	PORT_BIT(0x0040, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_J) PORT_CHAR('j') PORT_CHAR('J')
	PORT_BIT(0x0080, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_K) PORT_CHAR('k') PORT_CHAR('K')
	PORT_BIT(0x0100, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_L) PORT_CHAR('l') PORT_CHAR('L')
	PORT_BIT(0x0200, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_COLON) PORT_CHAR(';') PORT_CHAR(':') // 131
	PORT_BIT(0x0400, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_QUOTE) PORT_CHAR('\'') PORT_CHAR('"') // 132
	PORT_BIT(0x0800, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("RETURN") PORT_CODE(KEYCODE_ENTER) PORT_CHAR(13) // 133
	PORT_BIT(0x1000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("LINE FEED") // 134
	PORT_BIT(0x2000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME(u8"←") PORT_CODE(KEYCODE_LEFT) PORT_CHAR(UCHAR_MAMEKEY(LEFT)) // 135
	// The blank cap in the middle of the arrow cross on Figure 3-4.
	PORT_BIT(0x4000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("HOME") // 136
	PORT_BIT(0x8000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME(u8"→") PORT_CODE(KEYCODE_RIGHT) PORT_CHAR(UCHAR_MAMEKEY(RIGHT)) // 137

	PORT_START("ROW6")
	PORT_BIT(0x0001, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("KEYPAD 4") // 140
	PORT_BIT(0x0002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("KEYPAD 5") // 141
	PORT_BIT(0x0004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("KEYPAD 6") // 142
	PORT_BIT(0x0008, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("KEYPAD ,") // 143
	PORT_BIT(0x0030, IP_ACTIVE_HIGH, IPT_UNUSED) // 144/145 not used
	// SYMBOL, which the kernel table calls "Left Greek (Symb)": it selects the
	// third column of every key's entry on its own, and the fourth - the Greek
	// letters - together with SHIFT.
	PORT_BIT(0x0040, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("LEFT SYMBOL") // 146
	// The shift keys send their own make/break codes like any other key - the
	// band tracks the state itself. UCHAR_SHIFT_1 is what lets MAME's natural
	// keyboard reach every second PORT_CHAR above, uppercase letters included.
	PORT_BIT(0x0080, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("LEFT SHIFT") PORT_CODE(KEYCODE_LSHIFT) PORT_CHAR(UCHAR_SHIFT_1) // 147
	PORT_BIT(0x0100, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_Z) PORT_CHAR('z') PORT_CHAR('Z')
	PORT_BIT(0x0200, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_X) PORT_CHAR('x') PORT_CHAR('X')
	PORT_BIT(0x0400, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_C) PORT_CHAR('c') PORT_CHAR('C')
	PORT_BIT(0x0800, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_V) PORT_CHAR('v') PORT_CHAR('V')
	PORT_BIT(0x1000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_B) PORT_CHAR('b') PORT_CHAR('B')
	PORT_BIT(0x2000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_N) PORT_CHAR('n') PORT_CHAR('N')
	PORT_BIT(0x4000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_M) PORT_CHAR('m') PORT_CHAR('M')
	PORT_BIT(0x8000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_COMMA) PORT_CHAR(',') PORT_CHAR('<') // 157

	PORT_START("ROW7")
	PORT_BIT(0x0001, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_STOP) PORT_CHAR('.') PORT_CHAR('>') // 160
	PORT_BIT(0x0002, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(KEYCODE_SLASH) PORT_CHAR('/') PORT_CHAR('?') // 161
	PORT_BIT(0x0004, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("RIGHT SHIFT") PORT_CODE(KEYCODE_RSHIFT) PORT_CHAR(UCHAR_SHIFT_1) // 162
	PORT_BIT(0x0008, IP_ACTIVE_HIGH, IPT_UNUSED) // 163 not used
	PORT_BIT(0x0010, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("RIGHT SYMBOL") // 164
	PORT_BIT(0x0020, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME(u8"↓") PORT_CODE(KEYCODE_DOWN) PORT_CHAR(UCHAR_MAMEKEY(DOWN)) // 165
	PORT_BIT(0x0040, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("KEYPAD 1") // 166
	PORT_BIT(0x0080, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("KEYPAD 2") // 167
	PORT_BIT(0x0100, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("KEYPAD 3") // 170
	PORT_BIT(0x0600, IP_ACTIVE_HIGH, IPT_UNUSED) // 171/172 not used
	PORT_BIT(0x0800, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("SPACE") PORT_CODE(KEYCODE_SPACE) PORT_CHAR(' ') // 173
	PORT_BIT(0x1000, IP_ACTIVE_HIGH, IPT_UNUSED) // 174 not used
	PORT_BIT(0x2000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("KEYPAD 0") // 175
	PORT_BIT(0x4000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("KEYPAD .") // 176
	PORT_BIT(0x8000, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("KEYPAD ENTER") // 177
INPUT_PORTS_END

ioport_constructor explorer_keyboard_device::device_input_ports() const
{
	return INPUT_PORTS_NAME( explorer_kbd );
}

void explorer_keyboard_device::device_start()
{
}

void explorer_keyboard_device::device_reset()
{
	// Matches the SIB's USART mode instruction (SI General Description,
	// Figure 4-22): 1 stop bit, parity enabled, even parity, 8-bit
	// characters, clock/64 (153600 Hz / 64 = 2400 baud).
	set_data_frame(1, 8, PARITY_EVEN, STOP_BITS_1);
	set_rcv_rate(2400);
	set_tra_rate(2400);
	receive_register_reset();
	transmit_register_reset();

	clear_fifo();
	reset_key_state();
	start_processing(attotime::from_hz(1200));

	m_write_txd(1);
}

void explorer_keyboard_device::received_byte(u8 byte)
{
	// The SIB's self-test sequence sends a hardware BREAK (SBRK) on this same
	// link before switching it into diagnostic loopback for its own USART
	// test. A held-low break is indistinguishable from a real start bit to
	// device_buffered_serial_interface's generic byte framer - it happens to
	// land on a byte boundary and gets delivered here as if it were genuine
	// data, spuriously acking a break as if it were the real 0x00 init byte.
	// Real keyboard hardware has proper break detection and wouldn't do
	// this; discard framing-error receptions the same way.
	if (is_receive_framing_error())
		return;

	switch (byte)
	{
	case 0x00: // keyboard initialization/reset code
		transmit_byte(0x70); // acknowledge
		break;

	default:
		break;
	}
}
