// license:BSD-3-Clause
// copyright-holders:Miodrag Milanovic
/*

    Tape support for Glaksija GTP format

    Miodrag Milanovic
*/

#include "gtp_cas.h"


#define GTP_WAV_FREQUENCY   44100
#define WAVE_LOW        -0x5a9e
#define WAVE_HIGH       0x5a9e
#define WAVE_NULL       0

#define GTP_BLOCK_STANDARD  0x00
#define GTP_BLOCK_TURBO     0x01
#define GTP_BLOCK_NAME      0x10

#define PULSE_WIDTH     30
#define PERIOD_BASE     150
#define PERIOD_1        75
#define PERIOD_0        150

#define INTERBYTE_PAUSE     225
#define INTERBLOCK_PAUSE    100000


static void gtp_output_wave(std::vector<int16_t> &samples, int16_t wave_data, int length)
{
	for( ; length > 0; length-- )
		samples.push_back(wave_data);
}


static void gtp_mod_1(std::vector<int16_t> &samples)
{
	gtp_output_wave(samples, WAVE_LOW, PULSE_WIDTH);
	gtp_output_wave(samples, WAVE_HIGH, PULSE_WIDTH);
	gtp_output_wave(samples, WAVE_NULL, PERIOD_1 - 2 * PULSE_WIDTH);
	gtp_output_wave(samples, WAVE_LOW, PULSE_WIDTH);
	gtp_output_wave(samples, WAVE_HIGH, PULSE_WIDTH);
	gtp_output_wave(samples, WAVE_NULL, PERIOD_1 - 2 * PULSE_WIDTH);
}


static void gtp_mod_0(std::vector<int16_t> &samples)
{
	gtp_output_wave(samples, WAVE_LOW, PULSE_WIDTH);
	gtp_output_wave(samples, WAVE_HIGH, PULSE_WIDTH);
	gtp_output_wave(samples, WAVE_NULL, PERIOD_0 - 2 * PULSE_WIDTH);
}


static void gtp_byte(std::vector<int16_t> &samples, uint8_t val)
{
	for (int j = 0; j < 8; j++) {
		uint8_t b = (val >> j) & 1;
		if (b==0) {
			gtp_mod_0(samples);
		} else {
			gtp_mod_1(samples);
		}
	}
}


static void gtp_sync(std::vector<int16_t> &samples)
{
	for (int i = 0; i < 100; i++) {
		if (i) {
			// Interbyte pause
			gtp_output_wave(samples, WAVE_NULL, INTERBYTE_PAUSE);
		}
		gtp_byte(samples, 0);
	}
}


static void gtp_cas_fill_wave(std::vector<int16_t> &samples, std::vector<uint8_t> &bytes)
{
	int n = 0;

	while (n < bytes.size())
	{
		if (n + 5 >= bytes.size())
			break;
		int block_type = bytes[n];
		int block_size = bytes[n+2]*256 + bytes[n+1];
		n+=5;
		if (block_type == GTP_BLOCK_STANDARD) {
			// Interblock pause
			gtp_output_wave(samples, WAVE_NULL, INTERBLOCK_PAUSE);
			gtp_sync(samples);

			for (int i = 0; i < block_size; i++) {
				// Interbyte pause
				gtp_output_wave(samples, WAVE_NULL, INTERBYTE_PAUSE);

				gtp_byte(samples, bytes[n]);
				n++;
			}
		} else {
			n += block_size;
		}
	}
}


static cassette_image::error gtp_cassette_identify(cassette_image *cassette, cassette_image::Options *opts) {
	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = GTP_WAV_FREQUENCY;
	return cassette_image::error::SUCCESS;
}


static cassette_image::error gtp_cassette_load(cassette_image *cassette) {
		uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> bytes(file_size);
	cassette->image_read(&bytes[0], 0, file_size);
	std::vector<int16_t> samples;

	gtp_cas_fill_wave(samples, bytes);

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / GTP_WAV_FREQUENCY, samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


static const cassette_image::Format gtp_cassette_format =
{
	"gtp",
	gtp_cassette_identify,
	gtp_cassette_load,
	nullptr
};


CASSETTE_FORMATLIST_START(gtp_cassette_formats)
	CASSETTE_FORMAT(gtp_cassette_format)
CASSETTE_FORMATLIST_END
