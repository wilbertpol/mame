// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/*****************************************************************************
 *
 * Common transformation of the analog cassette input to the digital CASIN input
 * for the SERPROC ULA (BBC) or the main system ULA (electron).
 *
 * The analog cassette signal goes through some filters to produce a digital input:
 * - a second order high-pass filter,
 * - a second order low-pass filter,
 * - and a high gain amplifier (to create a square wave)
 *
 * From https://stardot.org.uk/forums/viewtopic.php?f=3&t=22935&sid=7431154c1ce0ff41e1a9942a34e7b92b&start=60 :
 * For high tone (2400Hz) there is almost no phase shift; the zero crossing of the cassette waveform
 * and the casin signal coincide.
 * For low tone (1200Hz) there is phase shift; the zero crossing of the casin signal almost coincides
 * with the peak of the cassette waveform.
 *
 ****************************************************************************/

#include "emu.h"
#include "bbc_elk_casin.h"


DEFINE_DEVICE_TYPE(BBC_ELK_CASIN, bbc_elk_casin_device, "bbc_elk_casin", "BBC/Electron CASIN")


bbc_elk_casin_device::bbc_elk_casin_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, BBC_ELK_CASIN, tag, owner, clock)
{
}


void bbc_elk_casin_device::device_start()
{
	assert(sampling_frequency > 0);

	setup_hpf();
	setup_lpf();

	save_item(NAME(m_hpf_x));
	save_item(NAME(m_hpf_y));
	save_item(NAME(m_lpf_x));
	save_item(NAME(m_lpf_y));
	save_item(NAME(m_last_tap_val));
	save_item(NAME(m_tap_val_length));
	save_item(NAME(m_len));
	save_item(NAME(m_casin));
	save_item(NAME(m_timeout));
}


void bbc_elk_casin_device::setup_hpf()
{
	// Configuration of the filters
	/*
	 * Parameters for the second order high pass filter
	 *
	 *          +-----------R1--------+
	 *          |             |\      |
	 * o---C1---+---C2---+----|+\     |
	 *                   |    |  \----+-------o
	 * |                 |    |  /    |
	 * |                 |  +-|-/     |       |
	 * in                R2 | |/      |       |
	 * |                 |  +-----RA--+      out
	 * |                 |  |                 |
	 * |                 |  RB                |
	 *                   |  |
	 * o-----------------+--+-----------------o
	 *
	*/
	const double r1 = 150'000;
	const double r2 = 150'000;
//	const double ra = 220'000;
//	const double rb = 820'000;
	const double c1 = 820.0/1'000'000'000'000;
	const double c2 = 820.0/1'000'000'000'000;

	const double theta = 1 / (sampling_frequency * sqrt(r1 * r2 * c1 * c2));
	const double d = 1 / 0.707;
	const double beta = 0.5 * ((1 - ((d / 2) * sin(theta))) / (1 + ((d / 2 ) * sin(theta))));
	const double gamma = (0.5 + beta) * cos(theta);
	hpf_a0 = (0.5 + beta + gamma) / 2.0;
	hpf_a1 = -(0.5 + beta + gamma);
	hpf_a2 = hpf_a0;
	hpf_b1 = -2 * gamma;
	hpf_b2 = 2 * beta;
}


void bbc_elk_casin_device::setup_lpf()
{
	/*
	 * Parameters for the second order low pass filter
	 *
	 *          +-----------C1--------+
	 *          |             |\      |
	 * o---R1---+---R2---+----|+\     |
	 *                   |    |  \----+-------o
	 * |                 |    |  /    |
	 * |                 |  +-|-/     |       |
	 * in                C2 | |/      |       |
	 * |                 |  +-----RA--+      out
	 * |                 |  |                 |
	 * |                 |  RB                |
	 *                   |  |
	 * o-----------------+--+-----------------o
	 *
	*/
	const double r1 = 8'200;
	const double r2 = 8'200;
//	const double ra = 10'000;
//	const double rb = 39'000;
	const double c1 = 4.7/1'000'000'000;
	const double c2 = 4.7/1'000'000'000;

	const double theta = 1 / ( sampling_frequency * sqrt(r1 * r2 * c1 * c2));
	const double d = 1 / 0.707;
	const double beta = 0.5 * ((1 - ((d / 2) * sin(theta))) / (1 + ((d / 2 ) * sin(theta))));
	const double gamma = (0.5 + beta) * cos(theta);
	lpf_a0 = (0.5 + beta - gamma) / 2.0;
	lpf_a1 = 0.5 + beta - gamma;
	lpf_a2 = lpf_a0;
	lpf_b1 = -2 * gamma;
	lpf_b2 = 2 * beta;
}


	/*
	 * cassout filter
	 *
	 *            
	 *                        |\       
	 * o---R1---+--------+----|+\      
	 *          |        |    |  \----+--C2---o
	 * |        |        |    |  /    |
	 * |        |        |  +-|-/     |       |
	 * in       C1       R2 | |/      |       |
	 * |        |        |  +---------+      out
	 * |        |        |                    |
	 * |        |        |                    |
	 *          |        |   
	 * o--------+--------+--------------------o
	 *
	 * R1 (R77) = 100k
	 * R2 (R76) = 10k
	 * C1 (C29) = 2n2
	 * C2 (C34) = 47n
	 * low pass filter R1 + C1 ?
	*/


void bbc_elk_casin_device::device_reset()
{
	m_hpf_x[0] = 0.0;
	m_hpf_x[1] = 0.0;
	m_hpf_y[0] = 0.0;
	m_hpf_y[1] = 0.0;
	reset();
}


void bbc_elk_casin_device::reset()
{
	m_last_tap_val = 0.0;
	m_tap_val_length = 0;
	m_casin = 0;
	for (int i = 0; i <= 3; i++)
		m_len[i] = 0;
}


int bbc_elk_casin_device::cassette_input(double tap_val)
{
	const double hpf_yn = (hpf_a0 * tap_val) + (hpf_a1 * m_hpf_x[0] + (hpf_a2 * m_hpf_x[1])) - (hpf_b1 * m_hpf_y[0]) - (hpf_b2 * m_hpf_y[1]);
	m_hpf_x[1] = m_hpf_x[0];
	m_hpf_x[0] = tap_val;
	m_hpf_y[1] = m_hpf_y[0];
	m_hpf_y[0] = hpf_yn;

	const double lpf_yn = (lpf_a0 * hpf_yn) + (lpf_a1 * m_lpf_x[0] + (lpf_a2 * m_lpf_x[1])) - (lpf_b1 * m_lpf_y[0]) - (lpf_b2 * m_lpf_y[1]);
	m_lpf_x[1] = m_lpf_x[0];
	m_lpf_x[0] = hpf_yn;
	m_lpf_y[1] = m_lpf_y[0];
	m_lpf_y[0] = lpf_yn;

	// The high gain amplifier creates a square wave from the output of the two filters.
	return lpf_yn < 0.0 ? 1 : 0;
}


bool bbc_elk_casin_device::input(double tap_val)
{
	bool bit_received = false;
	if ((tap_val >= 0.0 && m_last_tap_val < 0.0) || (tap_val < 0.0 && m_last_tap_val >= 0.0))
	{
		if (m_tap_val_length > (9 * 3))
		{
			for (int i = 0; i <= 3; i++)
				m_len[i] = 0;
			m_tap_val_length = 0;
			m_timeout = true;
		}
		else
		{
			m_timeout = false;
		}

		for (int i = 3; i > 0; i--)
			m_len[i] = m_len[i-1];
		m_len[0] = m_tap_val_length;

		m_tap_val_length = 0;

		if ((m_len[0] + m_len[1]) >= (18 + 18 - 5))
		{
			m_casin = 0;
			bit_received = true;

			for (int i = 0; i <= 3; i++)
				m_len[i] = 0;
		}

		if (((m_len[0] + m_len[1] + m_len[2] + m_len[3]) <= (18 + 18 + 5)) && (m_len[3] != 0))
		{
			m_casin = 1;
			bit_received = true;

			for (int i = 0; i <= 3; i++)
				m_len[i] = 0;
		}
	}
	m_tap_val_length++;
	m_last_tap_val = tap_val;
	return bit_received;
}
