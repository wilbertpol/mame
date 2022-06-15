// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
#ifndef DEVICES_MACHINE_BBC_ELK_CASIN_H
#define DEVICES_MACHINE_BBC_ELK_CASIN_H

#pragma once


class bbc_elk_casin_device : public device_t
{
public:
	bbc_elk_casin_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);

	void set_sampling_frequency(int frequency)
	{
		sampling_frequency = frequency;
	}

	// tap_val must be between -1.0 and +1.0, result is 0 or 1.
	int cassette_input(double tap_val);

	// Deprecated
	bool input(double tap_val);
	int casin() { return m_casin; }
	bool timeout() { return m_timeout; }
	void reset();

protected:
	virtual void device_start() override;
	virtual void device_reset() override;

private:

	int sampling_frequency = 0;

	// Things that are only calculated on start.
	double hpf_a0 = 0.0;
	double hpf_a1 = 0.0;
	double hpf_a2 = 0.0;
	double hpf_b1 = 0.0;
	double hpf_b2 = 0.0;

	double lpf_a0 = 0.0;
	double lpf_a1 = 0.0;
	double lpf_a2 = 0.0;
	double lpf_b1 = 0.0;
	double lpf_b2 = 0.0;

	double m_hpf_x[2]{};
	double m_hpf_y[2]{};
	double m_lpf_x[2]{};
	double m_lpf_y[2]{};

	// Deprecated
	double m_last_tap_val = 0.0;
	int m_tap_val_length = 0;
	int m_len[4]{};
	int m_casin = 0;
	bool m_timeout = false;

	void setup_hpf();
	void setup_lpf();
};


DECLARE_DEVICE_TYPE(BBC_ELK_CASIN, bbc_elk_casin_device)

#endif // DEVICES_MACHINE_BBC_ELK_CASIN_H
