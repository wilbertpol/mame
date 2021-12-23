// license:BSD-3-Clause
// copyright-holders:Wilbert Pol, Anthony Kruize
// thanks-to:Shay Green
/**************************************************************************************
* Game Boy sound emulation (c) Anthony Kruize (trandor@labyrinth.net.au)
*
* Anyways, sound on the Game Boy consists of 4 separate 'channels'
*   Sound1 = Quadrangular waves with SWEEP and ENVELOPE functions  (NR10,11,12,13,14)
*   Sound2 = Quadrangular waves with ENVELOPE functions (NR21,22,23,24)
*   Sound3 = Wave patterns from WaveRAM (NR30,31,32,33,34)
*   Sound4 = White noise with an envelope (NR41,42,43,44)
*
* Each sound channel has 2 modes, namely ON and OFF...  whoa
*
* These tend to be the two most important equations in
* converting between Hertz and GB frequency registers:
* (Sounds will have a 2.4% higher frequency on Super GB.)
*       gb = 2048 - (131072 / Hz)
*       Hz = 131072 / (2048 - gb)
*
* Changes:
*
*   10/2/2002       AK - Preliminary sound code.
*   13/2/2002       AK - Added a hack for mode 4, other fixes.
*   23/2/2002       AK - Use lookup tables, added sweep to mode 1. Re-wrote the square
*                        wave generation.
*   13/3/2002       AK - Added mode 3, better lookup tables, other adjustments.
*   15/3/2002       AK - Mode 4 can now change frequencies.
*   31/3/2002       AK - Accidently forgot to handle counter/consecutive for mode 1.
*    3/4/2002       AK - Mode 1 sweep can still occur if shift is 0.  Don't let frequency
*                        go past the maximum allowed value. Fixed Mode 3 length table.
*                        Slight adjustment to Mode 4's period table generation.
*    5/4/2002       AK - Mode 4 is done correctly, using a polynomial counter instead
*                        of being a total hack.
*    6/4/2002       AK - Slight tweak to mode 3's frequency calculation.
*   13/4/2002       AK - Reset envelope value when sound is initialized.
*   21/4/2002       AK - Backed out the mode 3 frequency calculation change.
*                        Merged init functions into gameboy_sound_w().
*   14/5/2002       AK - Removed magic numbers in the fixed point math.
*   12/6/2002       AK - Merged SOUNDx structs into one SOUND struct.
*  26/10/2002       AK - Finally fixed channel 3!
* xx/4-5/2016       WP - Rewrote sound core. Most of the code is not optimized yet.

TODO:
- Implement different behavior of CGB-02.
- Implement different behavior of CGB-05.
- Perform more tests on real hardware to figure out when the frequency counters are
  reloaded.
- Perform more tests on real hardware to understand when changes to the noise divisor
  and shift kick in.
- Optimize the channel update methods.

***************************************************************************************/

#include "emu.h"
#include "gb.h"


/***************************************************************************
    CONSTANTS
***************************************************************************/


/* Represents wave duties of 12.5%, 25%, 50% and 75% */
const int gameboy_sound_device::wave_duty_table[4][8] =
{
	{ -1, -1, -1, -1, -1, -1, -1,  1},
	{  1, -1, -1, -1, -1, -1, -1,  1},
	{  1, -1, -1, -1, -1,  1,  1,  1},
	{ -1,  1,  1,  1,  1,  1,  1, -1}
};

// device type definitions
DEFINE_DEVICE_TYPE(DMG_APU, dmg_apu_device, "dmg_apu", "LR35902 APU")
//DEFINE_DEVICE_TYPE(CGB02_APU, cgb02_apu_device, "cgb02_apu", fullname)
DEFINE_DEVICE_TYPE(CGB04_APU, cgb04_apu_device, "cgb04_apu", "CGB04 APU")
//DEFINE_DEVICE_TYPE(CGB05_APU, cgb05_apu_device, "cgb05_apu", fullname)

//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  gameboy_sound_device - constructor
//-------------------------------------------------

gameboy_sound_device::gameboy_sound_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, type, tag, owner, clock)
	, device_sound_interface(mconfig, *this)
{
}


dmg_apu_device::dmg_apu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: gameboy_sound_device(mconfig, DMG_APU, tag, owner, clock)
{
}


cgb04_apu_device::cgb04_apu_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: gameboy_sound_device(mconfig, CGB04_APU, tag, owner, clock)
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void gameboy_sound_device::device_start()
{
	m_channel = stream_alloc(0, 2, SAMPLE_RATE_OUTPUT_ADAPTIVE);
	m_timer = machine().scheduler().timer_alloc(timer_expired_delegate(FUNC(gameboy_sound_device::timer_callback),this));
	m_timer->adjust(clocks_to_attotime(FRAME_CYCLES/128), 0, clocks_to_attotime(FRAME_CYCLES/128));

	save_item(NAME(m_last_updated));
	save_item(NAME(m_snd_regs));
	// sound control
	save_item(STRUCT_MEMBER(m_snd_control, on));
	save_item(STRUCT_MEMBER(m_snd_control, vol_left));
	save_item(STRUCT_MEMBER(m_snd_control, vol_right));
	save_item(STRUCT_MEMBER(m_snd_control, mode1_left));
	save_item(STRUCT_MEMBER(m_snd_control, mode1_right));
	save_item(STRUCT_MEMBER(m_snd_control, mode2_left));
	save_item(STRUCT_MEMBER(m_snd_control, mode2_right));
	save_item(STRUCT_MEMBER(m_snd_control, mode3_left));
	save_item(STRUCT_MEMBER(m_snd_control, mode3_right));
	save_item(STRUCT_MEMBER(m_snd_control, mode4_left));
	save_item(STRUCT_MEMBER(m_snd_control, mode4_right));
	save_item(STRUCT_MEMBER(m_snd_control, frame_cycles));
	save_item(STRUCT_MEMBER(m_snd_control, frame));

	save_channel(SQUARE1);
	save_channel(SQUARE2);
	save_channel(WAVE);
	save_channel(NOISE);
}

void gameboy_sound_device::save_channel(int channel)
{
	save_item(STRUCT_MEMBER(m_snd[channel], reg), channel);
	save_item(STRUCT_MEMBER(m_snd[channel], on), channel);
	save_item(STRUCT_MEMBER(m_snd[channel], channel), channel);
	save_item(STRUCT_MEMBER(m_snd[channel], length), channel);
	save_item(STRUCT_MEMBER(m_snd[channel], length_mask), channel);
	save_item(STRUCT_MEMBER(m_snd[channel], length_counting), channel);
	save_item(STRUCT_MEMBER(m_snd[channel], length_enabled), channel);
	save_item(STRUCT_MEMBER(m_snd[channel], cycles_left), channel);
	save_item(STRUCT_MEMBER(m_snd[channel], duty), channel);
	save_item(STRUCT_MEMBER(m_snd[channel], envelope_enabled), channel);
	save_item(STRUCT_MEMBER(m_snd[channel], envelope_value), channel);
	save_item(STRUCT_MEMBER(m_snd[channel], envelope_direction), channel);
	save_item(STRUCT_MEMBER(m_snd[channel], envelope_time), channel);
	save_item(STRUCT_MEMBER(m_snd[channel], envelope_count), channel);
	save_item(STRUCT_MEMBER(m_snd[channel], signal), channel);
	save_item(STRUCT_MEMBER(m_snd[channel], frequency), channel);
	save_item(STRUCT_MEMBER(m_snd[channel], frequency_counter), channel);
	save_item(STRUCT_MEMBER(m_snd[channel], sweep_enabled), channel);
	save_item(STRUCT_MEMBER(m_snd[channel], sweep_neg_mode_used), channel);
	save_item(STRUCT_MEMBER(m_snd[channel], sweep_shift), channel);
	save_item(STRUCT_MEMBER(m_snd[channel], sweep_direction), channel);
	save_item(STRUCT_MEMBER(m_snd[channel], sweep_time), channel);
	save_item(STRUCT_MEMBER(m_snd[channel], sweep_count), channel);
	save_item(STRUCT_MEMBER(m_snd[channel], level), channel);
	save_item(STRUCT_MEMBER(m_snd[channel], offset), channel);
	save_item(STRUCT_MEMBER(m_snd[channel], duty_count), channel);
	save_item(STRUCT_MEMBER(m_snd[channel], current_sample), channel);
	save_item(STRUCT_MEMBER(m_snd[channel], sample_reading), channel);
	save_item(STRUCT_MEMBER(m_snd[channel], noise_short), channel);
	save_item(STRUCT_MEMBER(m_snd[channel], noise_lfsr), channel);
}

//-------------------------------------------------
//  device_clock_changed
//-------------------------------------------------

void gameboy_sound_device::device_clock_changed()
{
	m_timer->adjust(clocks_to_attotime(FRAME_CYCLES / 128), 0, clocks_to_attotime(FRAME_CYCLES / 128));
}


//-------------------------------------------------
//  device_reset
//-------------------------------------------------

void gameboy_sound_device::device_reset()
{
	std::fill(std::begin(m_snd), std::end(m_snd), SOUND{});

	m_snd[SQUARE1].channel = 1;
	m_snd[SQUARE1].length_mask = 0x3f;
	m_snd[SQUARE2].channel = 2;
	m_snd[SQUARE2].length_mask = 0x3f;
	m_snd[WAVE].channel = 3;
	m_snd[WAVE].length_mask = 0xff;
	m_snd[NOISE].channel = 4;
	m_snd[NOISE].length_mask = 0x3f;

	sound_w_internal(NR52, 0x00);
	m_snd_regs[AUD3W0] = 0xac;
	m_snd_regs[AUD3W1] = 0xdd;
	m_snd_regs[AUD3W2] = 0xda;
	m_snd_regs[AUD3W3] = 0x48;
	m_snd_regs[AUD3W4] = 0x36;
	m_snd_regs[AUD3W5] = 0x02;
	m_snd_regs[AUD3W6] = 0xcf;
	m_snd_regs[AUD3W7] = 0x16;
	m_snd_regs[AUD3W8] = 0x2c;
	m_snd_regs[AUD3W9] = 0x04;
	m_snd_regs[AUD3WA] = 0xe5;
	m_snd_regs[AUD3WB] = 0x2c;
	m_snd_regs[AUD3WC] = 0xac;
	m_snd_regs[AUD3WD] = 0xdd;
	m_snd_regs[AUD3WE] = 0xda;
	m_snd_regs[AUD3WF] = 0x48;
}


void cgb04_apu_device::device_reset()
{
	gameboy_sound_device::device_reset();

	m_snd_regs[AUD3W0] = 0x00;
	m_snd_regs[AUD3W1] = 0xff;
	m_snd_regs[AUD3W2] = 0x00;
	m_snd_regs[AUD3W3] = 0xff;
	m_snd_regs[AUD3W4] = 0x00;
	m_snd_regs[AUD3W5] = 0xff;
	m_snd_regs[AUD3W6] = 0x00;
	m_snd_regs[AUD3W7] = 0xff;
	m_snd_regs[AUD3W8] = 0x00;
	m_snd_regs[AUD3W9] = 0xff;
	m_snd_regs[AUD3WA] = 0x00;
	m_snd_regs[AUD3WB] = 0xff;
	m_snd_regs[AUD3WC] = 0x00;
	m_snd_regs[AUD3WD] = 0xff;
	m_snd_regs[AUD3WE] = 0x00;
	m_snd_regs[AUD3WF] = 0xff;
}


/***************************************************************************
    IMPLEMENTATION
***************************************************************************/

TIMER_CALLBACK_MEMBER(gameboy_sound_device::timer_callback)
{
	m_channel->update();
	update_state();
}


void gameboy_sound_device::restart_timer()
{
	m_channel->update();
	update_state();
	m_snd_control.frame_cycles &= ~(FRAME_CYCLES - 1);
	m_timer->adjust(clocks_to_attotime(FRAME_CYCLES/128), 0, clocks_to_attotime(FRAME_CYCLES/128));
}


void gameboy_sound_device::tick_length(int channel)
{
	if (m_snd[channel].length_enabled)
	{
		m_snd[channel].length = (m_snd[channel].length + 1) & m_snd[channel].length_mask;
		if (m_snd[channel].length == 0)
		{
			m_snd[channel].on = false;
			m_snd[channel].length_counting = false;
		}
	}
}


int32_t gameboy_sound_device::calculate_next_sweep(int channel)
{
	m_snd[channel].sweep_neg_mode_used = (m_snd[channel].sweep_direction < 0);
	int32_t new_frequency = m_snd[channel].frequency + m_snd[channel].sweep_direction * (m_snd[channel].frequency >> m_snd[channel].sweep_shift);

	if (new_frequency > 0x7ff)
	{
		m_snd[channel].on = false;
	}

	return new_frequency;
}


void gameboy_sound_device::apply_next_sweep(int channel)
{
	int32_t new_frequency = calculate_next_sweep(channel);

	if (m_snd[channel].on && m_snd[channel].sweep_shift > 0)
	{
		m_snd[channel].frequency = new_frequency;
		m_snd[channel].reg[3] = m_snd[channel].frequency & 0xff;
	}
}


void gameboy_sound_device::tick_sweep(int channel)
{
	m_snd[channel].sweep_count = (m_snd[channel].sweep_count - 1) & 0x07;
	if (m_snd[channel].sweep_count == 0)
	{
		m_snd[channel].sweep_count = m_snd[channel].sweep_time;

		if (m_snd[channel].sweep_enabled && m_snd[channel].sweep_time > 0)
		{
			apply_next_sweep(channel);
			calculate_next_sweep(channel);
		}
	}
}


void gameboy_sound_device::tick_envelope(int channel)
{
	if (m_snd[channel].envelope_enabled)
	{
		m_snd[channel].envelope_count = (m_snd[channel].envelope_count - 1) & 0x07;

		if (m_snd[channel].envelope_count == 0)
		{
			m_snd[channel].envelope_count = m_snd[channel].envelope_time;

			if (m_snd[channel].envelope_count)
			{
				int8_t new_envelope_value = m_snd[channel].envelope_value + m_snd[channel].envelope_direction;

				if (new_envelope_value >= 0 && new_envelope_value <= 15)
				{
					m_snd[channel].envelope_value = new_envelope_value;
				}
				else
				{
					m_snd[channel].envelope_enabled = false;
				}
			}
		}
	}
}


bool gameboy_sound_device::dac_enabled(struct SOUND &snd)
{
	return (snd.channel != 3) ? snd.reg[2] & 0xf8 : snd.reg[0] & 0x80;
}


void gameboy_sound_device::update_square_channel(int channel, uint64_t cycles)
{
	if (m_snd[channel].on)
	{
		// compensate for leftover cycles
		if (m_snd[channel].cycles_left > 0)
		{
			// Emit sample(s)
			if (cycles <= m_snd[channel].cycles_left)
			{
				m_snd[channel].cycles_left -= cycles;
				cycles = 0;
			}
			else
			{
				cycles -= m_snd[channel].cycles_left;
				m_snd[channel].cycles_left = 0;
			}
		}

		if (cycles & 3)
		{
			m_snd[channel].cycles_left = 4 - (cycles & 3);
		}
		cycles >>= 2;
		m_snd[channel].frequency_counter += cycles;
		while (m_snd[channel].frequency_counter >= 0x800)
		{
			m_snd[channel].duty_count = (m_snd[channel].duty_count + 1) & 0x07;
			m_snd[channel].signal = wave_duty_table[m_snd[channel].duty][m_snd[channel].duty_count];
			m_snd[channel].frequency_counter -= (0x800 - m_snd[channel].frequency);
		}
	}	
}


void dmg_apu_device::update_wave_channel(uint64_t cycles)
{
	if (m_snd[WAVE].on)
	{
		// compensate for leftover cycles
		if (m_snd[WAVE].cycles_left > 0)
		{
			if (cycles <= m_snd[WAVE].cycles_left)
			{
				// Emit samples
				m_snd[WAVE].cycles_left -= cycles;
				cycles = 0;
			}
			else
			{
				// Emit samples

				cycles -= m_snd[WAVE].cycles_left;
				m_snd[WAVE].cycles_left = 0;
			}
		}

		while (cycles > 0)
		{
			// Emit current sample

			// cycles -= 2
			if (cycles < 2)
			{
				m_snd[WAVE].cycles_left = 2 - cycles;
				cycles = 0;
			}
			else
			{
				cycles -= 2;

				// Calculate next state
				m_snd[WAVE].frequency_counter = (m_snd[WAVE].frequency_counter + 1) & 0x7ff;
				m_snd[WAVE].sample_reading = false;
				if (m_snd[WAVE].frequency_counter == 0x7ff)
				{
					m_snd[WAVE].offset = (m_snd[WAVE].offset + 1) & 0x1f;
				}
				if (m_snd[WAVE].frequency_counter == 0)
				{
					// Read next sample
					m_snd[WAVE].sample_reading = true;
					m_snd[WAVE].current_sample = m_snd_regs[AUD3W0 + (m_snd[WAVE].offset/2)];
					if (!(m_snd[WAVE].offset & 0x01))
					{
						m_snd[WAVE].current_sample >>= 4;
					}
					m_snd[WAVE].current_sample = (m_snd[WAVE].current_sample & 0x0f) - 8;

					m_snd[WAVE].signal = m_snd[WAVE].level ? m_snd[WAVE].current_sample / (1 << (m_snd[WAVE].level - 1)) : 0;

					// Reload frequency counter
					m_snd[WAVE].frequency_counter = m_snd[WAVE].frequency;
				}
			}
		}
	}
}


void cgb04_apu_device::update_wave_channel(uint64_t cycles)
{
	if (m_snd[WAVE].on)
	{
		// compensate for left over cycles
		if (m_snd[WAVE].cycles_left > 0)
		{
			if (cycles <= m_snd[WAVE].cycles_left)
			{
				// Emit samples
				m_snd[WAVE].cycles_left -= cycles;
				cycles = 0;
			}
			else
			{
				// Emit samples

				cycles -= m_snd[WAVE].cycles_left;
				m_snd[WAVE].cycles_left = 0;
			}
		}

		if (cycles & 1)
		{
			m_snd[WAVE].cycles_left = 1;
		}
		cycles >>= 1;
		uint16_t    distance = 0x800 - m_snd[WAVE].frequency_counter;
		if (cycles >= distance)
		{
			cycles -= distance;
			distance = 0x800 - m_snd[WAVE].frequency;
			// How many times the condition snd.frequency_counter == 0 is true
			uint64_t    counter = 1 + cycles / distance;

			m_snd[WAVE].offset = (m_snd[WAVE].offset + counter) & 0x1F;
			m_snd[WAVE].current_sample = m_snd_regs[AUD3W0 + m_snd[WAVE].offset / 2];
			if (!(m_snd[WAVE].offset & 1))
			{
				m_snd[WAVE].current_sample >>= 4;
			}
			m_snd[WAVE].current_sample = (m_snd[WAVE].current_sample & 0x0f) - 8;
			m_snd[WAVE].signal = m_snd[WAVE].level ? m_snd[WAVE].current_sample / (1 << (m_snd[WAVE].level - 1)) : 0;

			cycles %= distance;
			m_snd[WAVE].sample_reading = cycles ? false : true;

			m_snd[WAVE].frequency_counter = m_snd[WAVE].frequency + cycles;
		}
		else
		{
			m_snd[WAVE].frequency_counter += cycles;
		}
	}
}


void gameboy_sound_device::update_noise_channel(uint64_t cycles)
{
	if (cycles >= m_snd[NOISE].cycles_left)
	{
		cycles -= m_snd[NOISE].cycles_left;
		uint64_t    period = noise_period_cycles();
		uint64_t    counter = 1 + cycles / period, i = 0;
		uint16_t    start = m_snd[NOISE].noise_lfsr;
		while (i < counter) {
			// Using a Polynomial Counter (aka Linear Feedback Shift Register)
			 // Mode 4 has a 15 bit counter so we need to shift the
			 // bits around accordingly
			uint16_t feedback = ((m_snd[NOISE].noise_lfsr >> 1) ^ m_snd[NOISE].noise_lfsr) & 1;
			m_snd[NOISE].noise_lfsr = (m_snd[NOISE].noise_lfsr >> 1) | (feedback << 14);
			if (m_snd[NOISE].noise_short)
			{
				m_snd[NOISE].noise_lfsr = (m_snd[NOISE].noise_lfsr & ~(1 << 6)) | (feedback << 6);
			}
			i += 1;
			if (m_snd[NOISE].noise_lfsr == start)
			{
				counter %= i;
				i = 0;
			}
		}
		m_snd[NOISE].signal = (m_snd[NOISE].noise_lfsr & 1) ? -1 : 1;
		m_snd[NOISE].cycles_left = period - cycles % period;
	}
	else
	{
		m_snd[NOISE].cycles_left -= cycles;
	}
}


void gameboy_sound_device::update_state()
{
	attotime now = machine().time();

	// No time travelling
	if (now <= m_last_updated)
	{
		return;
	}

	if (m_snd_control.on)
	{
		uint64_t cycles = attotime_to_clocks(now - m_last_updated);

		uint64_t old_frame_cycles = m_snd_control.frame_cycles;
		m_snd_control.frame_cycles += cycles;

		if (m_snd_control.frame_cycles >= FRAME_CYCLES)
		{
			m_snd_control.frame_cycles -= FRAME_CYCLES;
			m_snd_control.frame = (m_snd_control.frame + 1) & 7;
			// Left over cycles in current frame
			uint64_t cycles_current_frame = FRAME_CYCLES - old_frame_cycles;

			update_square_channel(SQUARE1, cycles_current_frame);
			update_square_channel(SQUARE2, cycles_current_frame);
			update_wave_channel(cycles_current_frame);
			update_noise_channel(cycles_current_frame);

			cycles -= cycles_current_frame;

			if (!(m_snd_control.frame & 1)) {
				// length
				tick_length(SQUARE1);
				tick_length(SQUARE2);
				tick_length(WAVE);
				tick_length(NOISE);
			}
			if (m_snd_control.frame == 2 || m_snd_control.frame == 6) {
				// sweep
				tick_sweep(SQUARE1);
			} else if (m_snd_control.frame == 7) {
				// update envelope
				tick_envelope(SQUARE1);
				tick_envelope(SQUARE2);
				tick_envelope(NOISE);
			}
		}

		update_square_channel(SQUARE1, cycles);
		update_square_channel(SQUARE2, cycles);
		update_wave_channel(cycles);
		update_noise_channel(cycles);
	}

	m_last_updated = now;
}


uint64_t gameboy_sound_device::noise_period_cycles()
{
	static const int divisor[8] = { 8, 16,32, 48, 64, 80, 96, 112 };
	return divisor[m_snd[NOISE].reg[3] & 7] << (m_snd[NOISE].reg[3] >> 4);
}


u8 dmg_apu_device::wave_r(offs_t offset)
{
	m_channel->update();
	update_state();
	u8 data = m_snd_regs[AUD3W0 + offset];

	if (m_snd[WAVE].on)
	{
		data = m_snd[WAVE].sample_reading ? m_snd_regs[AUD3W0 + (m_snd[WAVE].offset/2)] : 0xff;
	}
	printf("wave_r data = %02x, m_snd[WAVE].frequency_counter = %04x\n", data, m_snd[WAVE].frequency_counter);

	return data;
}


u8 cgb04_apu_device::wave_r(offs_t offset)
{
	m_channel->update();
	update_state();

	if (m_snd[WAVE].on)
	{
		return m_snd_regs[AUD3W0 + (m_snd[WAVE].offset/2)];
	}

	return m_snd_regs[AUD3W0 + offset];
}


u8 gameboy_sound_device::sound_r(offs_t offset)
{
	static const uint8_t read_mask[0x40] =
	{
		0x80,0x3f,0x00,0xff,0xbf,0xff,0x3f,0x00,0xff,0xbf,0x7f,0xff,0x9f,0xff,0xbf,0xff,
		0xff,0x00,0x00,0xbf,0x00,0x00,0x70,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
	};

	// Make sure we are up to date.
	m_channel->update();
	update_state();

	if (m_snd_control.on)
	{
		if (offset == NR52)
		{
			return (m_snd_regs[NR52] & 0xf0) | (m_snd[SQUARE1].on ? 1 : 0) | (m_snd[SQUARE2].on ? 2 : 0) | (m_snd[WAVE].on ? 4 : 0) | (m_snd[NOISE].on ? 8 : 0) | 0x70;
		}
		return m_snd_regs[offset] | read_mask[offset & 0x3f];
	}
	else
	{
		return read_mask[offset & 0x3f];
	}
}


void dmg_apu_device::wave_w(offs_t offset, u8 data)
{
	m_channel->update();
	update_state();

	if (m_snd[WAVE].on)
	{
		if (m_snd[WAVE].sample_reading)
		{
			m_snd_regs[AUD3W0 + (m_snd[WAVE].offset/2)] = data;
		}
	}
	else
	{
		m_snd_regs[AUD3W0 + offset] = data;
	}
}


void cgb04_apu_device::wave_w(offs_t offset, u8 data)
{
	m_channel->update();
	update_state();

	if (m_snd[WAVE].on)
	{
		m_snd_regs[AUD3W0 + (m_snd[WAVE].offset/2)] = data;
	}
	else
	{
		m_snd_regs[AUD3W0 + offset] = data;
	}
}


void dmg_apu_device::sound_w(offs_t offset, u8 data)
{
	/* change in registers so update first */
	m_channel->update();
	update_state();

	/* Only register NR52 is accessible if the sound controller is disabled */
	if (!m_snd_control.on && offset != NR52 && offset != NR11 && offset != NR21 && offset != NR31 && offset != NR41)
		return;

	sound_w_internal(offset, data);
}


void cgb04_apu_device::sound_w(offs_t offset, u8 data)
{
	/* change in registers so update first */
	m_channel->update();
	update_state();

	/* Only register NR52 is accessible if the sound controller is disabled */
	if (!m_snd_control.on && offset != NR52)
		return;

	sound_w_internal(offset, data);
}


void dmg_apu_device::corrupt_wave_ram()
{
	if (m_snd[WAVE].offset < 8)
	{
		m_snd_regs[AUD3W0] = m_snd_regs[AUD3W0 + (m_snd[WAVE].offset/2)];
	}
	else
	{
		for (int i = 0; i < 4; i++)
		{
			m_snd_regs[AUD3W0 + i] = m_snd_regs[AUD3W0 + ((m_snd[WAVE].offset / 2) & ~0x03) + i];
		}
	}
}


void gameboy_sound_device::sound_w_internal( int offset, uint8_t data )
{
	/* Store the value */
	uint8_t old_data = m_snd_regs[offset];

	if (m_snd_control.on)
	{
		m_snd_regs[offset] = data;
	}

	switch (offset)
	{
	/*MODE 1 */
	case NR10: /* Sweep (R/W) */
		m_snd[SQUARE1].reg[0] = data;
		m_snd[SQUARE1].sweep_shift = data & 0x7;
		m_snd[SQUARE1].sweep_direction = (data & 0x8) ? -1 : 1;
		m_snd[SQUARE1].sweep_time = (data & 0x70) >> 4;
		if ((old_data & 0x08) && !(data & 0x08) && m_snd[SQUARE1].sweep_neg_mode_used)
		{
			m_snd[SQUARE1].on = false;
		}
		break;
	case NR11: /* Sound length/Wave pattern duty (R/W) */
		m_snd[SQUARE1].reg[1] = data;
		if (m_snd_control.on)
		{
			m_snd[SQUARE1].duty = (data & 0xc0) >> 6;
		}
		m_snd[SQUARE1].length = data & 0x3f;
		m_snd[SQUARE1].length_counting = true;
		break;
	case NR12: /* Envelope (R/W) */
		m_snd[SQUARE1].reg[2] = data;
		m_snd[SQUARE1].envelope_value = data >> 4;
		m_snd[SQUARE1].envelope_direction = (data & 0x8) ? 1 : -1;
		m_snd[SQUARE1].envelope_time = data & 0x07;
		if (!dac_enabled(m_snd[SQUARE1]))
		{
			m_snd[SQUARE1].on = false;
		}
		break;
	case NR13: /* Frequency lo (R/W) */
		m_snd[SQUARE1].reg[3] = data;
		// Only enabling the frequency line breaks blarggs's sound test #5
		// This condition may not be correct
		if (!m_snd[SQUARE1].sweep_enabled)
		{
			m_snd[SQUARE1].frequency = ((m_snd[SQUARE1].reg[4] & 0x7) << 8) | m_snd[SQUARE1].reg[3];
		}
		break;
	case NR14: /* Frequency hi / Initialize (R/W) */
		m_snd[SQUARE1].reg[4] = data;
		{
			bool length_was_enabled = m_snd[SQUARE1].length_enabled;

			m_snd[SQUARE1].length_enabled = (data & 0x40) ? true : false;
			m_snd[SQUARE1].frequency = ((m_snd_regs[NR14] & 0x7) << 8) | m_snd[SQUARE1].reg[3];

			if (!length_was_enabled && !(m_snd_control.frame & 1) && m_snd[SQUARE1].length_counting)
			{
				if (m_snd[SQUARE1].length_enabled)
				{
					tick_length(SQUARE1);
				}
			}

			if (data & 0x80)
			{
				m_snd[SQUARE1].on = true;
				m_snd[SQUARE1].envelope_enabled = true;
				m_snd[SQUARE1].envelope_value = m_snd[SQUARE1].reg[2] >> 4;
				m_snd[SQUARE1].envelope_count = m_snd[SQUARE1].envelope_time;
				m_snd[SQUARE1].sweep_count = m_snd[SQUARE1].sweep_time;
				m_snd[SQUARE1].sweep_neg_mode_used = false;
				m_snd[SQUARE1].signal = 0;
				m_snd[SQUARE1].length_counting = true;
				m_snd[SQUARE1].frequency = ((m_snd[SQUARE1].reg[4] & 0x7) << 8) | m_snd[SQUARE1].reg[3];
				m_snd[SQUARE1].frequency_counter = m_snd[SQUARE1].frequency;
				m_snd[SQUARE1].cycles_left = 0;
				m_snd[SQUARE1].duty_count = 0;
				m_snd[SQUARE1].sweep_enabled = (m_snd[SQUARE1].sweep_shift != 0) || (m_snd[SQUARE1].sweep_time != 0);
				if (!dac_enabled(m_snd[SQUARE1]))
				{
					m_snd[SQUARE1].on = false;
				}
				if (m_snd[SQUARE1].sweep_shift > 0)
				{
					calculate_next_sweep(SQUARE1);
				}

				if (m_snd[SQUARE1].length == 0 && m_snd[SQUARE1].length_enabled && !(m_snd_control.frame & 1))
				{
					tick_length(SQUARE1);
				}
			}
			else
			{
				// This condition may not be correct
				if (!m_snd[SQUARE1].sweep_enabled)
				{
					m_snd[SQUARE1].frequency = ((m_snd[SQUARE1].reg[4] & 0x7) << 8) | m_snd[SQUARE1].reg[3];
				}
			}
		}
		break;

	/*MODE 2 */
	case NR21: /* Sound length/Wave pattern duty (R/W) */
		m_snd[SQUARE2].reg[1] = data;
		if (m_snd_control.on)
		{
			m_snd[SQUARE2].duty = (data & 0xc0) >> 6;
		}
		m_snd[SQUARE2].length = data & 0x3f;
		m_snd[SQUARE2].length_counting = true;
		break;
	case NR22: /* Envelope (R/W) */
		m_snd[SQUARE2].reg[2] = data;
		m_snd[SQUARE2].envelope_value = data >> 4;
		m_snd[SQUARE2].envelope_direction = (data & 0x8) ? 1 : -1;
		m_snd[SQUARE2].envelope_time = data & 0x07;
		if (!dac_enabled(m_snd[SQUARE2]))
		{
			m_snd[SQUARE2].on = false;
		}
		break;
	case NR23: /* Frequency lo (R/W) */
		m_snd[SQUARE2].reg[3] = data;
		m_snd[SQUARE2].frequency = ((m_snd[SQUARE2].reg[4] & 0x7) << 8) | m_snd[SQUARE2].reg[3];
		break;
	case NR24: /* Frequency hi / Initialize (R/W) */
		m_snd[SQUARE2].reg[4] = data;
		{
			bool length_was_enabled = m_snd[SQUARE2].length_enabled;

			m_snd[SQUARE2].length_enabled = (data & 0x40) ? true : false;

			if (!length_was_enabled && !(m_snd_control.frame & 1) && m_snd[SQUARE2].length_counting)
			{
				if (m_snd[SQUARE2].length_enabled)
				{
					tick_length(SQUARE2);
				}
			}

			if (data & 0x80)
			{
				m_snd[SQUARE2].on = true;
				m_snd[SQUARE2].envelope_enabled = true;
				m_snd[SQUARE2].envelope_value = m_snd[SQUARE2].reg[2] >> 4;
				m_snd[SQUARE2].envelope_count = m_snd[SQUARE2].envelope_time;
				m_snd[SQUARE2].frequency = ((m_snd[SQUARE2].reg[4] & 0x7) << 8) | m_snd[SQUARE2].reg[3];
				m_snd[SQUARE2].frequency_counter = m_snd[SQUARE2].frequency;
				m_snd[SQUARE2].cycles_left = 0;
				m_snd[SQUARE2].duty_count = 0;
				m_snd[SQUARE2].signal = 0;
				m_snd[SQUARE2].length_counting = true;

				if (!dac_enabled(m_snd[SQUARE2]))
				{
					m_snd[SQUARE2].on = false;
				}

				if (m_snd[SQUARE2].length == 0 && m_snd[SQUARE2].length_enabled && !(m_snd_control.frame & 1))
				{
					tick_length(SQUARE2);
				}
			}
			else
			{
				m_snd[SQUARE2].frequency = ((m_snd[SQUARE2].reg[4] & 0x7) << 8) | m_snd[SQUARE2].reg[3];
			}
		}
		break;

	/*MODE 3 */
	case NR30: /* Sound On/Off (R/W) */
		m_snd[WAVE].reg[0] = data;
		if (!dac_enabled(m_snd[WAVE]))
		{
			m_snd[WAVE].on = false;
		}
		break;
	case NR31: /* Sound Length (R/W) */
		m_snd[WAVE].reg[1] = data;
		m_snd[WAVE].length = data;
		m_snd[WAVE].length_counting = true;
		break;
	case NR32: /* Select Output Level */
		m_snd[WAVE].reg[2] = data;
		m_snd[WAVE].level = (data & 0x60) >> 5;
		break;
	case NR33: /* Frequency lo (W) */
		m_snd[WAVE].reg[3] = data;
		m_snd[WAVE].frequency = ((m_snd[WAVE].reg[4] & 0x7) << 8) | m_snd[WAVE].reg[3];
		break;
	case NR34: /* Frequency hi / Initialize (W) */
		m_snd[WAVE].reg[4] = data;
		{
			bool length_was_enabled = m_snd[WAVE].length_enabled;

			m_snd[WAVE].length_enabled = BIT(data, 6) ? true : false;

			if (!length_was_enabled && !(m_snd_control.frame & 1) && m_snd[WAVE].length_counting)
			{
				if (m_snd[WAVE].length_enabled)
				{
					tick_length(WAVE);
				}
			}

			if (BIT(data, 7))
			{
				if (m_snd[WAVE].on && m_snd[WAVE].frequency_counter == 0x7ff)
				{
					corrupt_wave_ram();
				}
				m_snd[WAVE].on = true;
				m_snd[WAVE].offset = 0;
				m_snd[WAVE].duty = 1;
				m_snd[WAVE].duty_count = 0;
				m_snd[WAVE].length_counting = true;
				m_snd[WAVE].frequency = ((m_snd[WAVE].reg[4] & 0x7) << 8) | m_snd[WAVE].reg[3];
				m_snd[WAVE].frequency_counter = m_snd[WAVE].frequency;
				// There is a tiny bit of delay in starting up the wave channel(?)
				//
				// Results from older code where corruption of wave ram was triggered when sample_reading == true:
				// 4 breaks test 09 (read wram), fixes test 10 (write trigger), breaks test 12 (write wram)
				// 6 fixes test 09 (read wram), breaks test 10 (write trigger), fixes test 12 (write wram)
				m_snd[WAVE].cycles_left = 0 + 6;
				m_snd[WAVE].sample_reading = true;

				if (!dac_enabled(m_snd[WAVE]))
				{
					m_snd[WAVE].on = false;
				}

				if (m_snd[WAVE].length == 0 && m_snd[WAVE].length_enabled && !(m_snd_control.frame & 1))
				{
					tick_length(WAVE);
				}
			}
			else
			{
				m_snd[WAVE].frequency = ((m_snd[WAVE].reg[4] & 0x7) << 8) | m_snd[WAVE].reg[3];
			}
		}
		break;

	/*MODE 4 */
	case NR41: /* Sound Length (R/W) */
		m_snd[NOISE].reg[1] = data;
		m_snd[NOISE].length = data & 0x3f;
		m_snd[NOISE].length_counting = true;
		break;
	case NR42: /* Envelope (R/W) */
		m_snd[NOISE].reg[2] = data;
		m_snd[NOISE].envelope_value = data >> 4;
		m_snd[NOISE].envelope_direction = (data & 0x8) ? 1 : -1;
		m_snd[NOISE].envelope_time = data & 0x07;
		if (!dac_enabled(m_snd[NOISE]))
		{
			m_snd[NOISE].on = false;
		}
		break;
	case NR43: /* Polynomial Counter/Frequency */
		m_snd[NOISE].reg[3] = data;
		m_snd[NOISE].noise_short = (data & 0x8);
		break;
	case NR44: /* Counter/Consecutive / Initialize (R/W)  */
		m_snd[NOISE].reg[4] = data;
		{
			bool length_was_enabled = m_snd[NOISE].length_enabled;

			m_snd[NOISE].length_enabled = (data & 0x40) ? true : false;

			if (!length_was_enabled && !(m_snd_control.frame & 1) && m_snd[NOISE].length_counting)
			{
				if (m_snd[NOISE].length_enabled)
				{
					tick_length(NOISE);
				}
			}

			if (data & 0x80)
			{
				m_snd[NOISE].on = true;
				m_snd[NOISE].envelope_enabled = true;
				m_snd[NOISE].envelope_value = m_snd[NOISE].reg[2] >> 4;
				m_snd[NOISE].envelope_count = m_snd[NOISE].envelope_time;
				m_snd[NOISE].frequency_counter = 0;
				m_snd[NOISE].cycles_left = noise_period_cycles();
				m_snd[NOISE].signal = -1;
				m_snd[NOISE].noise_lfsr = 0x7fff;
				m_snd[NOISE].length_counting = true;

				if (!dac_enabled(m_snd[NOISE]))
				{
					m_snd[NOISE].on = false;
				}

				if (m_snd[NOISE].length == 0 && m_snd[NOISE].length_enabled && !(m_snd_control.frame & 1))
				{
					tick_length(NOISE);
				}
			}
		}
		break;

	/* CONTROL */
	case NR50: /* Channel Control / On/Off / Volume (R/W)  */
		m_snd_control.vol_left = data & 0x7;
		m_snd_control.vol_right = (data & 0x70) >> 4;
		break;
	case NR51: /* Selection of Sound Output Terminal */
		m_snd_control.mode1_right = BIT(data, 0);
		m_snd_control.mode2_right = BIT(data, 1);
		m_snd_control.mode3_right = BIT(data, 2);
		m_snd_control.mode4_right = BIT(data, 3);
		m_snd_control.mode1_left = BIT(data, 4);
		m_snd_control.mode2_left = BIT(data, 5);
		m_snd_control.mode3_left = BIT(data, 6);
		m_snd_control.mode4_left = BIT(data, 7);
		break;
	case NR52: // Sound On/Off (R/W)
		// Only bit 7 is writable, writing to bits 0-3 does NOT enable or disable sound. They are read-only.
		if (!BIT(data, 7))
		{
			// On DMG the length counters are not affected and not clocked
			// powering off should actually clear all registers
			apu_power_off();
		}
		else
		{
			if (!m_snd_control.on)
			{
				// When switching on, the next step should be 0.
				m_snd_control.frame = 7;
			}
		}
		m_snd_control.on = BIT(data, 7) ? true : false;
		m_snd_regs[NR52] = data & 0x80;
		break;
	}
}


void dmg_apu_device::apu_power_off()
{
	sound_w_internal(NR10, 0x00);
	m_snd[SQUARE1].duty = 0;
	m_snd_regs[NR11] = 0;
	sound_w_internal(NR12, 0x00);
	sound_w_internal(NR13, 0x00);
	sound_w_internal(NR14, 0x00);
	m_snd[SQUARE1].length_counting = false;
	m_snd[SQUARE1].sweep_neg_mode_used = false;

	m_snd_regs[NR21] = 0;
	sound_w_internal(NR22, 0x00);
	sound_w_internal(NR23, 0x00);
	sound_w_internal(NR24, 0x00);
	m_snd[SQUARE2].length_counting = false;

	sound_w_internal(NR30, 0x00);
	sound_w_internal(NR32, 0x00);
	sound_w_internal(NR33, 0x00);
	sound_w_internal(NR34, 0x00);
	m_snd[WAVE].length_counting = false;
	m_snd[WAVE].current_sample = 0;

	m_snd_regs[NR41] = 0;
	sound_w_internal(NR42, 0x00);
	sound_w_internal(NR43, 0x00);
	sound_w_internal(NR44, 0x00);
	m_snd[NOISE].length_counting = false;
	m_snd[NOISE].cycles_left = noise_period_cycles();

	m_snd[SQUARE1].on = false;
	m_snd[SQUARE2].on = false;
	m_snd[WAVE].on = false;
	m_snd[NOISE].on = false;

	m_snd_control.wave_ram_locked = false;

	for (int i = NR44 + 1; i < NR52; i++)
	{
		sound_w_internal(i, 0x00);
	}
}


void cgb04_apu_device::apu_power_off()
{
	sound_w_internal(NR10, 0x00);
	m_snd[SQUARE1].duty = 0;
	sound_w_internal(NR11, 0x00);
	sound_w_internal(NR12, 0x00);
	sound_w_internal(NR13, 0x00);
	sound_w_internal(NR14, 0x00);
	m_snd[SQUARE1].length_counting = false;
	m_snd[SQUARE1].sweep_neg_mode_used = false;

	sound_w_internal(NR21, 0x00);
	sound_w_internal(NR22, 0x00);
	sound_w_internal(NR23, 0x00);
	sound_w_internal(NR24, 0x00);
	m_snd[SQUARE2].length_counting = false;

	sound_w_internal(NR30, 0x00);
	sound_w_internal(NR31, 0x00);
	sound_w_internal(NR32, 0x00);
	sound_w_internal(NR33, 0x00);
	sound_w_internal(NR34, 0x00);
	m_snd[WAVE].length_counting = false;
	m_snd[WAVE].current_sample = 0;

	sound_w_internal(NR41, 0x00);
	sound_w_internal(NR42, 0x00);
	sound_w_internal(NR43, 0x00);
	sound_w_internal(NR44, 0x00);
	m_snd[NOISE].length_counting = false;
	m_snd[NOISE].cycles_left = noise_period_cycles();

	m_snd[SQUARE1].on = false;
	m_snd[SQUARE2].on = false;
	m_snd[WAVE].on = false;
	m_snd[NOISE].on = false;

	m_snd_control.wave_ram_locked = false;

	for (int i = NR44 + 1; i < NR52; i++)
	{
		sound_w_internal(i, 0x00);
	}
}

u8 cgb04_apu_device::pcm12_r()
{
	m_channel->update();
	update_state();
	return (m_snd[SQUARE2].on && m_snd[SQUARE2].signal > 0 ? (m_snd[SQUARE2].envelope_value << 4) : 0x00) |
	 (m_snd[SQUARE1].on && m_snd[SQUARE1].signal > 0 ? m_snd[SQUARE1].envelope_value : 0x00);
}

u8 cgb04_apu_device::pcm34_r()
{
	m_channel->update();
	update_state();
	return 0xfd;
}


//-------------------------------------------------
//  sound_stream_update - handle a stream update
//-------------------------------------------------

void gameboy_sound_device::sound_stream_update(sound_stream &stream, std::vector<read_stream_view> const &inputs, std::vector<write_stream_view> &outputs)
{
	auto &outputl = outputs[0];
	auto &outputr = outputs[1];
	for (int sampindex = 0; sampindex < outputl.samples(); sampindex++)
	{
		s32 sample;
		s32 left = 0;
		s32 right = 0;

		/* Mode 1 - Wave with Envelope and Sweep */
		if (m_snd[SQUARE1].on)
		{
			sample = m_snd[SQUARE1].signal * m_snd[SQUARE1].envelope_value;

			if (m_snd_control.mode1_left)
				left += sample;
			if (m_snd_control.mode1_right)
				right += sample;
		}

		/* Mode 2 - Wave with Envelope */
		if (m_snd[SQUARE2].on)
		{
			sample = m_snd[SQUARE2].signal * m_snd[SQUARE2].envelope_value;
			if (m_snd_control.mode2_left)
				left += sample;
			if (m_snd_control.mode2_right)
				right += sample;
		}

		/* Mode 3 - Wave patterns from WaveRAM */
		if (m_snd[WAVE].on)
		{
			sample = m_snd[WAVE].signal;
			if (m_snd_control.mode3_left)
				left += sample;
			if (m_snd_control.mode3_right)
				right += sample;
		}

		/* Mode 4 - Noise with Envelope */
		if (m_snd[NOISE].on)
		{
			sample = m_snd[NOISE].signal * m_snd[NOISE].envelope_value;
			if (m_snd_control.mode4_left)
				left += sample;
			if (m_snd_control.mode4_right)
				right += sample;
		}

		/* Adjust for master volume */
		left *= m_snd_control.vol_left;
		right *= m_snd_control.vol_right;

		/* Update the buffers */
		outputl.put_int(sampindex, left, 32768 / 64);
		outputr.put_int(sampindex, right, 32768 / 64);
	}
}
