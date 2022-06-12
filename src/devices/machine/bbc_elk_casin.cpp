// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/*****************************************************************************
 *
 * Common transformation of the analog cassette input to the digital CASIN input
 * for the SERPROC ULA (BBC) or the main system ULA (electron).
 *
 * The analog cassette signal goes through some filters to produce a digital input for
 * the serproc ula:
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
	assert(hpf_r1 > 0.0 && hpf_r2 > 0.0 && hpf_c1 > 0.0 && hpf_c2 > 0.0);
	assert(lpf_r1 > 0.0 && lpf_r2 > 0.0 && lpf_c1 > 0.0 && lpf_c2 > 0.0);

	double pi = acos(-1.0);
	double hpf_fc = 1 / (2 * pi * sqrt(hpf_r1 * hpf_r2 * hpf_c1 * hpf_c2));
	logerror("HPF cut-off frequency: %f\n", hpf_fc);
	double hpf_theta = (2 * pi * hpf_fc) / sampling_frequency;
	double hpf_d = 1 / 0.707;
	double hpf_beta = 0.5 * ((1 - ((hpf_d / 2) * sin(hpf_theta))) / (1 + ((hpf_d / 2 ) * sin(hpf_theta))));
	double hpf_gamma = (0.5 + hpf_beta) * cos(hpf_theta);
	hpf_a0 = (0.5 + hpf_beta + hpf_gamma) / 2.0;
	hpf_a1 = -(0.5 + hpf_beta + hpf_gamma);
	hpf_a2 = hpf_a0;
	hpf_b1 = -2 * hpf_gamma;
	hpf_b2 = 2 * hpf_beta;

	double lpf_fc = 1 / (2 * pi * sqrt(lpf_r1 * lpf_r2 * lpf_c1 * lpf_c2));
	logerror("LPF cut-off frequency: %f\n", lpf_fc);
	double lpf_theta = (2 * pi * lpf_fc) / sampling_frequency;
	double lpf_d = 1 / 0.707;
	double lpf_beta = 0.5 * ((1 - ((lpf_d / 2) * sin(lpf_theta))) / (1 + ((lpf_d / 2 ) * sin(lpf_theta))));
	double lpf_gamma = (0.5 + lpf_beta) * cos(lpf_theta);
	lpf_a0 = (0.5 + lpf_beta - lpf_gamma) / 2.0;
	lpf_a1 = 0.5 + lpf_beta - lpf_gamma;
	lpf_a2 = lpf_a0;
	lpf_b1 = -2 * lpf_gamma;
	lpf_b2 = 2 * lpf_beta;

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
	double hpf_yn = (hpf_a0 * tap_val) + (hpf_a1 * m_hpf_x[0] + (hpf_a2 * m_hpf_x[1])) - (hpf_b1 * m_hpf_y[0]) - (hpf_b2 * m_hpf_y[1]);
	m_hpf_x[1] = m_hpf_x[0];
	m_hpf_x[0] = tap_val;
	m_hpf_y[1] = m_hpf_y[0];
	m_hpf_y[0] = hpf_yn;

	double lpf_yn = (lpf_a0 * hpf_yn) + (lpf_a1 * m_lpf_x[0] + (lpf_a2 * m_lpf_x[1])) - (lpf_b1 * m_lpf_y[0]) - (lpf_b2 * m_lpf_y[1]);
	m_lpf_x[1] = m_lpf_x[0];
	m_lpf_x[0] = hpf_yn;
	m_lpf_y[1] = m_lpf_y[0];
	m_lpf_y[0] = lpf_yn;

	// The high gain amplifier creates a square wave from the output of the two filters.
	return lpf_yn < -0.0 ? 1 : 0;
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
