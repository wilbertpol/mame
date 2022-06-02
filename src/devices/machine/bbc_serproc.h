// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/***************************************************************************

    Ferranti 2C199E / VLSI VC2026 Serial ULA emulation

****************************************************************************
                            _____   _____
                    D0   1 |*    \_/     | 28  VI
                    D1   2 |             | 27  CASOUT
                    D2   3 |             | 26  TXC
                    D3   4 |             | 25  CLK
                    D4   5 |             | 24  RTSI
                    D5   6 |             | 23  RTSO
                    D6   7 |  ULA 2C199E | 22  TXD
                    D7   8 |             | 21  DOUT
                   _CS   9 |             | 20  CTSI
                     E  10 |             | 19  CTSO
                 CASMO  11 |             | 18  RXC
                 CASIN  12 |             | 17  RXD
                   DCD  13 |             | 16  DIN
                   GND  14 |_____________| 15  CR

***************************************************************************/
#ifndef DEVICES_MACHINE_BBC_SERPROC_H
#define DEVICES_MACHINE_BBC_SERPROC_H

#pragma once

#include "machine/clock.h"


class bbc_serproc_device : public device_t
{
public:
	bbc_serproc_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	auto out_casmo_callback() { return m_out_casmo_cb.bind(); }
	auto out_cts_callback() { return m_out_cts_cb.bind(); }
	auto out_dcd_callback() { return m_out_dcd_cb.bind(); }
	auto out_rxc_callback() { return m_out_rxc_cb.bind(); }
	auto out_rxd_callback() { return m_out_rxd_cb.bind(); }
	auto out_txc_callback() { return m_out_txc_cb.bind(); }

	void casin(double tap_val);
	void write(uint8_t data);

	DECLARE_WRITE_LINE_MEMBER(din_w);
	DECLARE_WRITE_LINE_MEMBER(ctsi_w);

	DECLARE_WRITE_LINE_MEMBER(tx_clock_w);
	DECLARE_WRITE_LINE_MEMBER(rx_clock_w);

	static constexpr int SAMPLING_FREQUENCY = 48'000;

protected:
	virtual void device_add_mconfig(machine_config &config) override;
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	devcb_write_line m_out_casmo_cb;
	devcb_write_line m_out_cts_cb;
	devcb_write_line m_out_dcd_cb;
	devcb_write_line m_out_rxc_cb;
	devcb_write_line m_out_rxd_cb;
	devcb_write_line m_out_txc_cb;

	required_device<clock_device> m_rx_clock;
	required_device<clock_device> m_tx_clock;

	emu_timer *m_timeout_timer;
	emu_timer *m_dcd_timer;

	uint8_t m_control = 0;
	int m_cass_rxc = 1;
	int m_cass_rxd = 0;
	int m_cass_dcd = 0;
	int m_din = 0;
	int m_ctsi = 0;
	int m_rxc = 0;
	double m_last_tap_val = 0.0;
	bool m_timeout = false;
	bool m_skip_edge = false;

	int m_out_dcd = 0;
	int m_out_rxc = 0;
	int m_out_rxd = 0;

	void update_cts();
	void update_dcd();
	void update_rxc();
	void update_rxd();
	void cass_pulse_rxc();
	TIMER_CALLBACK_MEMBER(timeout);
	TIMER_CALLBACK_MEMBER(cass_dcd);
};


DECLARE_DEVICE_TYPE(BBC_SERPROC, bbc_serproc_device)


#endif // DEVICES_MACHINE_BBC_SERPROC_H
