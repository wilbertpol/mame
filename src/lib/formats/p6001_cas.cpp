// license:BSD-3-Clause
// copyright-holders:Angelo Salese
/*
 * NEC PC-6001 cassette format handling
 */

#include "p6001_cas.h"

#define WAVE_HIGH        0x5a9e
#define WAVE_LOW        -0x5a9e


static void pc6001_fill_wave(std::vector<int16_t> &samples, uint8_t data)
{
	// one byte = 8 samples
	for (int x = 0; x < 8;x++)
	{
		samples.push_back(((data >> (7-x)) & 1) ? WAVE_HIGH : WAVE_LOW);
	}

}


static void pc6001_handle_cas(std::vector<int16_t> &samples, std::vector<uint8_t> &bytes)
{
	for (int pos = 0; pos < bytes.size(); pos++)
	{
		pc6001_fill_wave(samples, bytes[pos]);
	}
}


static cassette_image::error pc6001_cas_identify(cassette_image *cassette, cassette_image::Options *opts)
{
	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = 8000;
	return cassette_image::error::SUCCESS;
}


static cassette_image::error pc6001_cas_load(cassette_image *cassette)
{
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> bytes(file_size);
	cassette->image_read(&bytes[0], 0, file_size);
	std::vector<int16_t> samples;

	pc6001_handle_cas(samples, bytes);

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / 8000, samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


static const cassette_image::Format pc6001_cassette_format = {
	"cas",
	pc6001_cas_identify,
	pc6001_cas_load,
	nullptr
};


CASSETTE_FORMATLIST_START(pc6001_cassette_formats)
	CASSETTE_FORMAT(pc6001_cassette_format)
CASSETTE_FORMATLIST_END
