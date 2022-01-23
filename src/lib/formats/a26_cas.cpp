// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/*

Atari 2600 SuperCharger support

*/

#include "formats/a26_cas.h"


#define A26_CAS_SIZE            8448
#define A26_WAV_FREQUENCY       44100
#define BIT_ZERO_LENGTH         10
#define BIT_ONE_LENGTH          15
#define ZEROS_ONES              2755


static const uint16_t one_wave[BIT_ONE_LENGTH] =
{
	0x2AE5, 0x4E60, 0x644E, 0x68E4, 0x5B56, 0x3DFE, 0x15ED, 0xEA13, 0xC202, 0xA4AA,
	0x971C, 0x9BB2, 0xB1A0, 0xD51B, 0x0000
};


static const uint16_t zero_wave[BIT_ZERO_LENGTH] =
{
	0x3DFE, 0x644E, 0x644E, 0x3DFE, 0x0000, 0xC202, 0x9BB2, 0x9BB2, 0xC202, 0x0000
};


static void a26_cas_output_wave(std::vector<int16_t> &samples, int16_t wave_data, int length)
{
	for (int i = 0 ; i < length; i++ )
		samples.push_back(wave_data);
}


static void a26_cas_output_bit(std::vector<int16_t> &samples, int bit)
{
	int bit_size = bit ? BIT_ONE_LENGTH : BIT_ZERO_LENGTH;
	const int16_t *p = bit ? (const int16_t *)one_wave : (const int16_t *)zero_wave;

	for (int i = 0; i < bit_size; i++)
		a26_cas_output_wave(samples, p[i], 1);
}


static void a26_cas_output_byte(std::vector<int16_t> &samples, uint8_t byte)
{
	for (int i = 0; i < 8; i++, byte <<= 1)
		a26_cas_output_bit(samples, (byte & 0x80) ? 1 : 0);
}


static void a26_cas_clearing_tone(std::vector<int16_t> &samples)
{
	a26_cas_output_wave(samples, 0, A26_WAV_FREQUENCY);
}


static void a26_cas_zeros_ones(std::vector<int16_t> &samples)
{
	for (int i = 0; i < ZEROS_ONES; i++)
	{
		a26_cas_output_bit(samples, 0);
		a26_cas_output_bit(samples, 1);
	}
	a26_cas_output_bit(samples, 0);
	a26_cas_output_bit(samples, 0);
}


static void a26_cas_output_contents(std::vector<int16_t> &samples, std::vector<uint8_t> &bytes)
{
	// There are 8 header bytes
	for (int i = 0; i < 8; i++)
		a26_cas_output_byte(samples, bytes[0x2000 + i]);

	const uint8_t pages = bytes[0x2003];

	// Output each page prefixed with a small page header
	for (int i = 0; i < pages; i++)
	{
		a26_cas_output_byte(samples, bytes[0x2010 + i]);
		a26_cas_output_byte(samples, bytes[0x2040 + i]);
		for (int j = 0; j < 256; j++)
		{
			a26_cas_output_byte(samples, bytes[i * 256 + j]);
		}
	}
}


static void a26_cas_do_work(std::vector<int16_t> &samples, std::vector<uint8_t> &bytes)
{
	// Output clearing tone
	a26_cas_clearing_tone(samples);

	// Output header tone, alternating 1s and 0s for about a second ending with two 0s
	a26_cas_zeros_ones(samples);

	// Output the actual contents of the tape
	a26_cas_output_contents(samples, bytes);

	// Output footer tone, alternating 1s and 0s for about a second ending with two 0s
	a26_cas_zeros_ones(samples);
}


static cassette_image::error a26_cassette_identify(cassette_image *cassette, cassette_image::Options *opts)
{
	if (cassette->image_size() == A26_CAS_SIZE) {
		opts->channels = 1;
		opts->bits_per_sample = 16;
		opts->sample_frequency = A26_WAV_FREQUENCY;
		return cassette_image::error::SUCCESS;
	}
	return cassette_image::error::INVALID_IMAGE;
}


static cassette_image::error a26_cassette_load(cassette_image *cassette)
{
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> bytes(file_size);
	cassette->image_read(&bytes[0], 0, file_size);
	std::vector<int16_t> samples;

	if (file_size != A26_CAS_SIZE)
		return cassette_image::error::INVALID_IMAGE;

	a26_cas_do_work(samples, bytes);

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / A26_WAV_FREQUENCY, samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


static const cassette_image::Format a26_cassette_format =
{
	"a26",
	a26_cassette_identify,
	a26_cassette_load,
	nullptr
};


CASSETTE_FORMATLIST_START(a26_cassette_formats)
	CASSETTE_FORMAT(a26_cassette_format)
CASSETTE_FORMATLIST_END
