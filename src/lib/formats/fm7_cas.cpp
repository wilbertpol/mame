// license:BSD-3-Clause
// copyright-holders:Barry Rodewald
/*
 * Fujitsu FM-7 series cassette handling
 */

#include "fm7_cas.h"

#include <cstring>

#define WAVE_HIGH        0x5a9e
#define WAVE_LOW        -0x5a9e


static void fm7_fill_wave(std::vector<int16_t> &samples, uint8_t high, uint8_t low)
{
	uint16_t data = (high << 8) + low;
	int count = (data & 0x7fff);

	if (data & 0x8000)
	{
		for (int x = 0; x < count; x++)
		{
			samples.push_back(WAVE_HIGH);
		}
	}
	else
	{
		for (int x = 0; x < count; x++)
		{
			samples.push_back(WAVE_LOW);
		}
	}
}


static cassette_image::error fm7_handle_t77(std::vector<int16_t> &samples, std::vector<uint8_t> &bytes)
{
	if (bytes.size() < 16 || memcmp(&bytes[0], "XM7 TAPE IMAGE 0",16))  // header check
		return cassette_image::error::INVALID_IMAGE;

	int data_pos = 16;
	while (data_pos + 1 < bytes.size())
	{
		fm7_fill_wave(samples, bytes[data_pos], bytes[data_pos + 1]);
		data_pos += 2;
	}
	return cassette_image::error::SUCCESS;
}


static cassette_image::error fm7_cas_identify(cassette_image *cassette, cassette_image::Options *opts)
{
	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = 110250;
	return cassette_image::error::SUCCESS;
}


static cassette_image::error fm7_cas_load(cassette_image *cassette)
{
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> bytes(file_size);
	cassette->image_read(&bytes[0], 0, file_size);
	std::vector<int16_t> samples;

	cassette_image::error err = fm7_handle_t77(samples, bytes);
	if (err != cassette_image::error::SUCCESS)
		return err;

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / 110250, samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


static const cassette_image::Format fm7_cassette_format = {
	"t77",
	fm7_cas_identify,
	fm7_cas_load,
	nullptr
};


CASSETTE_FORMATLIST_START(fm7_cassette_formats)
	CASSETTE_FORMAT(fm7_cassette_format)
CASSETTE_FORMATLIST_END
