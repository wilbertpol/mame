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
	void set_hpf(double r1, double r2, double ra, double rb, double c1, double c2)
	{
		hpf_r1 = r1;
		hpf_r2 = r2;
		hpf_ra = ra;
		hpf_rb = rb;
		hpf_c1 = c1;
		hpf_c2 = c2;
	}

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
	void set_lpf(double r1, double r2, double ra, double rb, double c1, double c2)
	{
		lpf_r1 = r1;
		lpf_r2 = r2;
		lpf_ra = ra;
		lpf_rb = rb;
		lpf_c1 = c1;
		lpf_c2 = c2;
	}

	// tap_val must be between -1.0 and +1.0, result is 0 or 1.
	int cassette_input(double tap_val);

	// Deprecated
	bool input(double tap_val);
	int casin() { return m_casin; }
	bool timeout() { return m_timeout; }
	void reset();

//	static constexpr int SAMPLING_FREQUENCY = 48'000;

protected:
	virtual void device_start() override;
	virtual void device_reset() override;

private:

	int sampling_frequency = 0;

	// Configuration of the filters
	double hpf_r1 = 0.0;
	double hpf_r2 = 0.0;
	double hpf_ra = 0.0;
	double hpf_rb = 0.0;
	double hpf_c1 = 0.0;
	double hpf_c2 = 0.0;

	double lpf_r1 = 0.0;
	double lpf_r2 = 0.0;
	double lpf_ra = 0.0;
	double lpf_rb = 0.0;
	double lpf_c1 = 0.0;
	double lpf_c2 = 0.0;

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
};


DECLARE_DEVICE_TYPE(BBC_ELK_CASIN, bbc_elk_casin_device)

#endif // DEVICES_MACHINE_BBC_ELK_CASIN_H
