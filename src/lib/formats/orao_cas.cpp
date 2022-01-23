// license:BSD-3-Clause
// copyright-holders:Olivier Galibert
/*

    Tape support for Orao  TAP format

*/

#include "orao_cas.h"


#define ORAO_WAV_FREQUENCY  44100
#define WAVE_HIGH       -24576
#define WAVE_LOW        24576

#define ORAO_WAVE_ONE   17
#define ORAO_WAVE_ZERO  9

#define ORAO_HEADER_SIZE 360


static void orao_output_wave(std::vector<int16_t> &samples, int16_t wave_data, int length)
{
	for ( ; length > 0; length--)
		samples.push_back(wave_data);
}


static void orao_cas_fill_wave(std::vector<int16_t> &samples, std::vector<uint8_t> &bytes)
{
	int startpos = 0;
	bool newformat = true;

	if (bytes[0]==0x68 && bytes[1]==0x01 && bytes[2]==0x00)
	{
		startpos = ORAO_HEADER_SIZE;
		newformat = false;
	}
	for (int i = startpos; i < bytes.size(); i++)
	{
		for (int j = 0; j < 8; j++)
		{
			int k = newformat ? (7 - j) : j;
			uint8_t b = (bytes[i] >> k) & 1;
			if (b == 0)
			{
				orao_output_wave(samples, WAVE_LOW, ORAO_WAVE_ZERO);
				orao_output_wave(samples, WAVE_HIGH, ORAO_WAVE_ZERO);
			}
			else
			{
				orao_output_wave(samples, WAVE_LOW, ORAO_WAVE_ONE);
				orao_output_wave(samples, WAVE_HIGH, ORAO_WAVE_ONE);
			}
		}
	}
}


static cassette_image::error orao_cassette_identify(cassette_image *cassette, cassette_image::Options *opts)
{
	if (cassette->image_size() < ORAO_HEADER_SIZE)
		return cassette_image::error::INVALID_IMAGE;

	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = ORAO_WAV_FREQUENCY;
	return cassette_image::error::SUCCESS;
}


static cassette_image::error orao_cassette_load(cassette_image *cassette)
{
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> bytes(file_size);
	cassette->image_read(&bytes[0], 0, file_size);
	std::vector<int16_t> samples;

	if (file_size < ORAO_HEADER_SIZE)
		return cassette_image::error::INVALID_IMAGE;

	orao_cas_fill_wave(samples, bytes);

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / ORAO_WAV_FREQUENCY, samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


static const cassette_image::Format orao_cassette_format =
{
	"tap",
	orao_cassette_identify,
	orao_cassette_load,
	nullptr
};


CASSETTE_FORMATLIST_START(orao_cassette_formats)
	CASSETTE_FORMAT(orao_cassette_format)
CASSETTE_FORMATLIST_END
