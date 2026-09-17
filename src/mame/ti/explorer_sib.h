// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/**********************************************************************

    TI Explorer System Interface Board (SIB).

**********************************************************************/

#ifndef MAME_TI_EXPLORER_SIB_H
#define MAME_TI_EXPLORER_SIB_H

#pragma once

#include "ti_nubus.h"
#include "explorer_kbd.h"
#include "machine/mm58167.h"
#include "screen.h"
#include "bus/centronics/ctronics.h"
#include "bus/rs232/rs232.h"
#include "machine/clock.h"
#include "machine/i8251.h"
#include "machine/nvram.h"
#include "machine/pit8253.h"
#include "machine/z80scc.h"
#include "sound/sn76496.h"
#include "video/crt9007.h"


class explorer_sib_device : public device_t, public device_ti_nubus_card_interface
{
public:
	explorer_sib_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	DECLARE_INPUT_CHANGED_MEMBER(mouse_x_changed);
	DECLARE_INPUT_CHANGED_MEMBER(mouse_y_changed);
	DECLARE_INPUT_CHANGED_MEMBER(mouse_button_changed);

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

private:
	void nubus_map(address_map &map) ATTR_COLD;
	void graphics_bitmap_map(address_map &map) ATTR_COLD;
	void local_bus_map(address_map &map) ATTR_COLD;
	u32 video_ram_r(offs_t offset);
	void video_ram_w(offs_t offset, u32 data, u32 mem_mask);
	void video_ram_rmw_w(offs_t offset, u32 data, u32 mem_mask);
	void event_generator_map(address_map &map) ATTR_COLD;
	u8 crtc_r(offs_t offset);
	void crtc_w(offs_t offset, u8 data);
	void printer_map(address_map &map) ATTR_COLD;
	u32 printer_status_r();
	void printer_control_w(u32 data);
	void centronics_busy_w(int state);
	void centronics_perror_w(int state);
	void centronics_select_w(int state);
	void centronics_fault_w(int state);
	void centronics_ack_w(int state);
	void update_speaker_amplifier();
	void mouse_map(address_map &map) ATTR_COLD;
	u32 motion_keyswitch_r();
	void post_mouse_motion_event();
	void rtc_map(address_map &map) ATTR_COLD;
	void timers_map(address_map &map) ATTR_COLD;
	void nvram_map(address_map &map) ATTR_COLD;
	void rs232c_map(address_map &map) ATTR_COLD;
	void configuration_rom_map(address_map &map) ATTR_COLD;
	uint32_t screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);
	void i8251_txd_w(int state);
	void keyboard_txd_w(int state);
	bool diagnostic_loopback_active() const { return (m_interrupt_diag_control & 0x0c) != 0; }
	u32 diagnostic_loopback_value() const { return BIT(m_diagnostic_data, 8) ? 0x00 : 0xff; }
	u32 event_vector_r(offs_t offset);
	void event_vector_w(offs_t offset, u32 data, u32 mem_mask);
	void update_leds();
	// Event Generator (section 4.4.5): on a monitored interrupt condition, becomes
	// NuBus master and writes a plain 0xFF byte to the preprogrammed address stored
	// in m_event_vector[cause] (Table 4-4 gives the cause->index mapping) - not a
	// real CPU interrupt line, software just polls that memory location. The cause
	// is one of the EVENT_* constants in explorer_sib.cpp, which carry Table 4-4's
	// names; nine of the sixteen are wired, and the comments beside the remaining
	// registers say why each of the rest is not.
	void post_event(int cause);
	void post_voice_sample(u8 data);
	void pit_out2_w(int state);
	void rtc_irq_w(int state);
	void scc_int_w(int state);
	void crtc_int_w(int state);
	void usart_rxrdy_w(int state);
	void usart_txrdy_w(int state);

	required_device<screen_device> m_screen;
	// "Video processor controller 9007" per Figure 1-11 of the Explorer System
	// Field Maintenance manual - an SMC CRT9007 VPAC, which is the part Table
	// 4-13's register list and TI's own Lisp register names both describe. Used
	// here for what the board actually uses it for: raster timing and the
	// vertical-retrace interrupt. Its cursor, smooth-scroll, light-pen and DMA
	// features are dead on this board ("other CRT controller functions
	// suggested by the register names in Table 4-13 are not functional due to
	// hardware constraints"), which is why nothing but int_callback is wired.
	required_device<crt9007_device> m_crt9007;
	required_device<i8251_device> m_i8251;
	required_device<explorer_keyboard_device> m_keyboard;
	required_device<mm58167_device> m_mm58167;
	required_device<pit8253_device> m_pit;
	// "The RS-232C serial data port uses a fully programmable Zilog Z8530
	// serial communications controller" (4.4.15) - one of the two chips TI
	// names outright, the other being the CRT9007. Figure 1-11 of the Field
	// Maintenance manual shows it on the board as a Z0853004PSC ("Z85030PS" as
	// the ordering code reads on the package), and 4.4.15.2 cites the
	// Z8030/Z8530 SCC Technical Manual.
	//
	// Only channel A is a communications channel. Channel B "is wired only to
	// provide some auxiliary control line input/output and is never used as a
	// communications channel" - its receive and transmit data buffers are
	// unused, and its modem-control pins carry the EIA signals the Z8530 has no
	// channel A pin for (Figure 4-33). See device_add_mconfig().
	required_device<scc8530_device> m_z85030ps;
	required_device<rs232_port_device> m_rs232;
	required_device<clock_device> m_usart_clock;
	required_device<sn76496_device> m_sn76496;
	required_device<nvram_device> m_nvram;
	// Paragraph 4.4.9: "an SI board interface that transmits 8-bit parallel data
	// and handles control and handshake signals between the SI board and a
	// Centronics-compatible external device (usually a printer)". Table 4-9's
	// signal list is the Centronics one line for line. The connector is not on
	// this board - the signals go to P3 and the fibre-optic board adapts them to
	// the DIN connector at the back of the enclosure (4.4.9.1), which is the
	// "connector for parallel cable" of Figure F-5 in the System Introduction.
	required_device<centronics_device> m_centronics;
	required_device<output_latch_device> m_centronics_data_out;
	required_ioport m_mouse_buttons;
	required_ioport m_mouse_x_axis;
	required_ioport m_mouse_y_axis;
	memory_share_creator<u32> m_video_ram;
	memory_share_creator<u8> m_nv_ram;
	// The board's two fault indicators, driven from configuration register bits
	// 2 and 8: the red self-test fault LED and the yellow monitor fault LED.
	output_finder<> m_fault_led;
	output_finder<> m_monitor_led;
	u32 m_configuration_register = 0;
	u32 m_event_vector[16]{};
	// Reverse video (bit 1) but *not* blanked, which is a deliberate deviation
	// from one manual in favour of another - the two disagree, see below.
	//
	// The NOTE beside Figure 4-12 says "when the system is powered up or the
	// board is reset, the attributes register will be set to reverse video and
	// a blanked video display", i.e. 0x03, and that the display must be
	// unblanked to view video. But nothing in the boot path unblanks it for the
	// first eight seconds: the only write is 0x02 from the board's own
	// ROM-resident diagnostic, which the processor runs when the slot scan
	// reaches slot 5 - long after it has printed "Slot 6 TESTING SYSTEM", "Slot
	// 0 passed" and so on into the bit map. (Verified: every access into this
	// slot's window was logged for the whole boot, there is no earlier write,
	// and Meroko's own log shows its "Video State Changed to 2" at exactly the
	// same point in the same command stream, so this is the boot code's real
	// behaviour and not a divergence.)
	//
	// The System Field Maintenance manual documents what a real machine
	// actually shows, and it is not a screen that stays dark that long: Table
	// 1-1, "Power-Up Sequence of LED and Video Display Actions" (book OP 1-22/
	// 1-23), has the display blank for the first two steps, "Goes white" at
	// step 3, and then each self-test line appearing in turn - SLOT 6 TESTING
	// SYSTEM, SLOT 0 PASSED, SLOT 2 PASSED ... up to SLOT 5 PASSED at step 10,
	// which is the step where this board's own diagnostic finishes. So the bit
	// map is on screen well before the write that the NOTE says is needed, and
	// starting blanked would hide the entire power-up self-test display (see
	// Figure 1-14 for what it is supposed to look like).
	//
	// Whatever the board really does there is not modelled, so start unblanked
	// and keep the documented reverse video: an empty bit map then reads as the
	// white screen of Table 1-1 step 3, and the self-test text appears black on
	// white as the manual's figures show.
	u32 m_attribute_register = 0x02;
	u32 m_mask_register = 0;
	u32 m_operation_register = 0;
	// The 16-bit up/down position counters of paragraph 4.4.11.2, and the
	// keyswitch half of the motion/keyswitch data register (Figure 4-14: LKEY
	// 06, MKEY 05, RKEY 04). The quadrature bits 03-00 that drive the counters
	// on real hardware are not modelled - the motion detector is, so there is
	// nothing left for them to do, and the software "normally masks off" the
	// raw motion data anyway.
	u32 m_mouse_y_position = 0;
	u32 m_mouse_x_position = 0;
	u8 m_mouse_keyswitches = 0;
	// KOUT, bit 07 of the same register: the serial keyboard data line the
	// monitor sends back over the fiber-optic link, tapped ahead of the USART
	// so that software can sample it directly. Idles high (mark).
	int m_keyboard_txd = 1;
	// One event per motion/keyswitch condition until the host reads the
	// register that reports it - see post_mouse_motion_event().
	bool m_mouse_motion_event_pending = false;
	bool m_mouse_keyswitch_event_pending = false;
	u32 m_interrupt_diag_control = 0;
	u32 m_monitor_control = 0;
	u32 m_diagnostic_data = 0;
	// The voice register of paragraph 4.4.11.8: VO<07:00> plus VO08, the
	// data-present bit. VO08 is not stored data - it is the state of the voice
	// interrupt itself, set when the interrupt controller loads a sample and
	// cleared by the read that takes the sample off the I bus, which is the only
	// acknowledge the hardware has. See post_voice_sample().
	u32 m_voice_data_register = 0;
	bool m_voice_data_present = false;
	// Register 0 of the printer port, and the write half of register 1
	// (Table 4-10 / Table 4-11). The control register starts with every active-
	// low output deasserted and interrupts off - the state the polled and
	// interrupt sequences of 4.4.9.2 both start from - rather than at zero,
	// which would sit on INIT- and DATSTRB- and hold the printer in reset until
	// software first wrote the register.
	u32 m_printer_data = 0;
	u32 m_printer_control = 0x07;
	// The four handshake inputs of Table 4-9, latched as the peripheral drives
	// them, and read back through the status half of register 1. They start at
	// their idle levels so that a slot with no peripheral in it still reads as
	// the ready printer the board's own diagnostic expects (busy and paper-out
	// deasserted, SELECT asserted, FAULT- deasserted = 0x0c).
	u8 m_centronics_busy = 0;
	u8 m_centronics_perror = 0;
	u8 m_centronics_select = 1;
	u8 m_centronics_fault = 1;
	u8 m_centronics_ack = 1;
	u32 m_sound_control = 0;
	u32 m_speech_register = 0;
	// Keyboard USART interrupt, Table 4-4 cause 6 ("Ready to transmit/receive").
	bool m_usart_rxrdy = false;
	bool m_usart_txrdy = false;
};

DECLARE_DEVICE_TYPE(SIB, explorer_sib_device)

#endif // MAME_TI_EXPLORER_SIB_H
