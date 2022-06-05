// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/*****************************************************************************
 *
 * BBC Micro Serial ULA
 * Ferranti 2C199E / VLSI VC2026 Serial
 * 
 * Based on findings from https://stardot.org.uk/forums/viewtopic.php?f=3&t=22935
 * or https://web.archive.org/web/20220301095659/https://stardot.org.uk/forums/viewtopic.php?f=3&t=22935&sid=a3c873c2d1d97807717b9a5abd5ef49d
 * 
 ****************************************************************************/

#include "emu.h"
#include "debugger.h"
#include "bbc_serproc.h"


DEFINE_DEVICE_TYPE(BBC_SERPROC, bbc_serproc_device, "bbc_serproc", "2c199 BBC Serial ULA")


bbc_serproc_device::bbc_serproc_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, BBC_SERPROC, tag, owner, clock)
	, m_out_casmo_cb(*this)
	, m_out_cts_cb(*this)
	, m_out_dcd_cb(*this)
	, m_out_dout_cb(*this)
	, m_out_rtso_cb(*this)
	, m_out_rxc_cb(*this)
	, m_out_rxd_cb(*this)
	, m_out_txc_cb(*this)
	, m_out_txd_cb(*this)
	, m_out_cass_out_enabled(*this)
	, m_rx_clock(*this, "rx_clock")
	, m_tx_clock(*this, "tx_clock")
{
}


void bbc_serproc_device::device_add_mconfig(machine_config &config)
{
	CLOCK(config, m_tx_clock, DERIVED_CLOCK(1, 1));
	m_tx_clock->signal_handler().set(FUNC(bbc_serproc_device::tx_clock_w));
	CLOCK(config, m_rx_clock, DERIVED_CLOCK(1, 1));
	m_rx_clock->signal_handler().set(FUNC(bbc_serproc_device::rx_clock_w));
}


void bbc_serproc_device::device_start()
{
	m_timeout_timer = machine().scheduler().timer_alloc(timer_expired_delegate(FUNC(bbc_serproc_device::timeout),this));
	m_dcd_timer = machine().scheduler().timer_alloc(timer_expired_delegate(FUNC(bbc_serproc_device::cass_dcd),this));

	// resolve callbacks
	m_out_casmo_cb.resolve_safe();
	m_out_cts_cb.resolve_safe();
	m_out_dcd_cb.resolve_safe();
	m_out_dout_cb.resolve_safe();
	m_out_rtso_cb.resolve_safe();
	m_out_rxc_cb.resolve_safe();
	m_out_rxd_cb.resolve_safe();
	m_out_txc_cb.resolve_safe();
	m_out_txd_cb.resolve_safe();
	m_out_cass_out_enabled.resolve_safe();

	save_item(NAME(m_control));
	save_item(NAME(m_cass_rxc));
	save_item(NAME(m_cass_rxd));
	save_item(NAME(m_cass_dcd));
	save_item(NAME(m_din));
	save_item(NAME(m_ctsi));
	save_item(NAME(m_rtsi));
	save_item(NAME(m_rxc));
	save_item(NAME(m_txd));
	save_item(NAME(m_last_tap_val));
	save_item(NAME(m_timeout));
	save_item(NAME(m_skip_edge));
}


void bbc_serproc_device::device_reset()
{
	m_control = 0;
	m_cass_rxc = 1;
	m_cass_rxd = 0;
	m_cass_dcd = 0;
	m_din = 0;
	m_ctsi = 0;
	m_rxc = 1;
	m_last_tap_val = 0.0;
	m_timeout = false;
	m_skip_edge = false;
}


void bbc_serproc_device::casin(int tap_val)
{
	// bbcb:
	// g FB61 - starting motor
	// wpset fe09,1,r
	// Do edge detection
//	machine().debugger().debug_break();
	if (m_last_tap_val != tap_val)
	{
		logerror("casin: edge detected, timeout = %s, skip_edge = %s\n", m_timeout ? "true" : "false", m_skip_edge ? "true" : "false");
//		machine().debugger().debug_break();
		if (m_timeout)
		{
			if (m_cass_rxd != 0)
			{
				m_cass_rxd = 0;
				update_rxd();
				m_dcd_timer->adjust(attotime::never);
			}
			m_timeout = false;
		}
		else
		{
			if (m_skip_edge)
			{
				m_skip_edge = false;
			}
			else
			{
				if (m_cass_rxd != 1)
				{
					m_cass_rxd = 1;
					update_rxd();
					// DCD goes high after approx. 200msec on the Ferranti ULA and 50msec on the VLSI ULA.
					m_dcd_timer->adjust(clocks_to_attotime(256 * 1024));  // Not verified, but 212msec close to 200msec
				}
			}
		}
		cass_pulse_rxc();
		// A longer timeout makes more software works perhaps the original tapes were getting bad??
		m_timeout_timer->adjust(clocks_to_attotime(/*320*/410));
	}
	m_last_tap_val = tap_val;

	// 4 pulse 1.625us low, 1.625us high = 3.25usec = 2 + 2 cycles = 4 cycles => 16 cycles
	// 1790Hz reliably detected as high tone, 1780Hz not
	// DCD
	// high/low timeout = 320 cycles
	// 200us DCD

}


void bbc_serproc_device::write(uint8_t data)
{
	logerror("write %02x\n", data);
	m_control = data;

	static const int serial_clock_dividers[8] =
	{
		1,    // 000
		16,   // 001
		4,    // 010
		128,  // 011
		2,    // 100
		64,   // 101
		8,    // 110
		256   // 111
	};

	update_rxd();
	update_dcd();
	update_dout();
	update_rxc();
	update_rts();
	update_cts();
	m_out_casmo_cb(BIT(m_control, 7));

	// Set transmit clock rate
	m_tx_clock->set_clock_scale((double) 1 / serial_clock_dividers[data & 0x07]);
	// Set receive clock rate
	m_rx_clock->set_clock_scale((double) 1 / serial_clock_dividers[(data >> 3) & 0x07]);
}


WRITE_LINE_MEMBER(bbc_serproc_device::din_w)
{
	m_din = state;
	update_rxd();
}


WRITE_LINE_MEMBER(bbc_serproc_device::ctsi_w)
{
	m_ctsi = state;
	update_cts();
}


WRITE_LINE_MEMBER(bbc_serproc_device::rx_clock_w)
{
//	m_rxc = state;
//	update_rxc();
}


WRITE_LINE_MEMBER(bbc_serproc_device::tx_clock_w)
{
	m_out_txc_cb(state);
}


WRITE_LINE_MEMBER(bbc_serproc_device::txd_w)
{
	m_txd = state;
	update_dout();
	// HACK
	m_out_txd_cb(state);
}


WRITE_LINE_MEMBER(bbc_serproc_device::rtsi_w)
{
	m_rtsi = state;
	// TODO
}


TIMER_CALLBACK_MEMBER(bbc_serproc_device::timeout)
{
	logerror("timeout\n");
	m_timeout = true;
	m_skip_edge = true;
	cass_pulse_rxc();
}


TIMER_CALLBACK_MEMBER(bbc_serproc_device::cass_dcd)
{
	if (m_cass_dcd) 
	{
		m_cass_dcd = 0;
		update_dcd();
//		machine().debugger().debug_break();
	}
	else
	{
		logerror("Trigger DCD\n");
//		machine().debugger().debug_break();
		m_cass_dcd = 1;
		update_dcd();
		// DCD goes low again after approx. 200usec
		m_dcd_timer->adjust(clocks_to_attotime(256));
	}
}


void bbc_serproc_device::cass_pulse_rxc()
{
	// do 4 rxc pulses, TODO: perform on a timer?
	for (int i = 0; i < 4; i++)
	{
		m_cass_rxc = 0;	// TODO lasts 2 cycles
		update_rxc();
		m_cass_rxc = 1; // TODO lasts 2 cycles
		update_rxc();
	}

}


void bbc_serproc_device::update_cts()
{
	// TODO output cts when writing to tape
	m_out_cts_cb(BIT(m_control, 6) ? m_ctsi : 0);
}


void bbc_serproc_device::update_dcd()
{
	m_out_dcd = BIT(m_control, 6) ? 0 : m_cass_dcd;
	logerror("DCD = %d, RXC = %d, RXD = %d\n", m_out_dcd, m_out_rxc, m_out_rxd);
	m_out_dcd_cb(m_out_dcd);
}


void bbc_serproc_device::update_dout()
{
	m_out_dout_cb(BIT(m_control, 6) ? m_txd : 0);
}


void bbc_serproc_device::update_rts()
{
	// TODO trigger cass out enabled in driver to signal that we are going to write?
	int cass_out_enabled = 0;
	m_out_cass_out_enabled(cass_out_enabled);

	m_out_rtso_cb(BIT(m_control, 6) ? m_rtsi : 1);
}


void bbc_serproc_device::update_rxc()
{
	m_out_rxc = BIT(m_control, 6) ? m_rxc : m_cass_rxc;
	logerror("DCD = %d, RXC = %d, RXD = %d\n", m_out_dcd, m_out_rxc, m_out_rxd);
	m_out_rxc_cb(BIT(m_control, 6) ? m_rxc : m_cass_rxc);
}


void bbc_serproc_device::update_rxd()
{
	m_out_rxd = BIT(m_control, 6) ? m_din : m_cass_rxd;
	logerror("DCD = %d, RXC = %d, RXD = %d\n", m_out_dcd, m_out_rxc, m_out_rxd);
	m_out_rxd_cb(BIT(m_control, 6) ? m_din : m_cass_rxd);
}

