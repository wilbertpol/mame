// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/**********************************************************************

    TI Explorer System Interface Board (SIB).

**********************************************************************/

#ifndef MAME_TI_EXPLORER_SIB_H
#define MAME_TI_EXPLORER_SIB_H

#pragma once

#include "explorer_kbd.h"
#include "screen.h"
#include "ti_nubus.h"
#include "bus/centronics/ctronics.h"
#include "bus/rs232/rs232.h"
#include "machine/clock.h"
#include "machine/i8251.h"
#include "machine/mm58167.h"
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
	u32 video_test_register();
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
	u8 sound_control_parity();
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
	void post_event(int cause);
	void post_voice_sample(u8 data);
	void pit_out0_w(int state);
	void pit_out2_w(int state);
	void rtc_irq_w(int state);
	void scc_int_w(int state);
	void crtc_int_w(int state);
	void usart_rxrdy_w(int state);
	void usart_txrdy_w(int state);

	required_device<screen_device> m_screen;
	required_device<crt9007_device> m_crt9007;
	required_device<i8251_device> m_i8251;
	required_device<explorer_keyboard_device> m_keyboard;
	required_device<mm58167_device> m_mm58167;
	required_device<pit8253_device> m_pit;
	required_device<scc8530_device> m_z85030ps;
	required_device<rs232_port_device> m_rs232;
	required_device<clock_device> m_usart_clock;
	required_device<sn76496_device> m_sn76496;
	required_device<nvram_device> m_nvram;
	required_device<centronics_device> m_centronics;
	required_device<output_latch_device> m_centronics_data_out;
	required_ioport m_mouse_buttons;
	required_ioport m_mouse_x_axis;
	required_ioport m_mouse_y_axis;
	memory_share_creator<u32> m_video_ram;
	memory_share_creator<u8> m_nv_ram;
	output_finder<> m_fault_led;
	output_finder<> m_monitor_led;
	u32 m_configuration_register = 0;
	u32 m_event_vector[16]{};
	u32 m_attribute_register = 0x02;
	u32 m_mask_register = 0;
	u32 m_operation_register = 0;
	u32 m_mouse_y_position = 0;
	u32 m_mouse_x_position = 0;
	u8 m_mouse_keyswitches = 0;
	int m_keyboard_txd = 1;
	bool m_mouse_motion_event_pending = false;
	bool m_mouse_keyswitch_event_pending = false;
	u32 m_interrupt_diag_control = 0;
	u32 m_monitor_control = 0;
	u32 m_diagnostic_data = 0;
	u32 m_voice_data_register = 0;
	bool m_voice_data_present = false;
	u32 m_printer_data = 0;
	u32 m_printer_control = 0x07;
	u8 m_centronics_busy = 0;
	u8 m_centronics_perror = 0;
	u8 m_centronics_select = 1;
	u8 m_centronics_fault = 1;
	u8 m_centronics_ack = 1;
	u32 m_sound_control = 0;
	u32 m_speech_register = 0;
	bool m_usart_rxrdy = false;
	bool m_usart_txrdy = false;
};

DECLARE_DEVICE_TYPE(SIB, explorer_sib_device)

#endif // MAME_TI_EXPLORER_SIB_H
