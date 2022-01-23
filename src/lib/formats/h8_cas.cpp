// license:BSD-3-Clause
// copyright-holders:Robbbert
/********************************************************************

Support for Heathkit H8 H8T cassette images


Standard Kansas City format (300 baud)

We output a leader, followed by the contents of the H8T file.

********************************************************************/

#include "h8_cas.h"

#define WAVEENTRY_LOW  -32768
#define WAVEENTRY_HIGH  32767

#define H8_WAV_FREQUENCY   9600


static void h8_put_samples(std::vector<int16_t> &samples, int count, int level)
{
	for (int i = 0; i < count; i++)
		samples.push_back(level);
}


static void h8_output_bit(std::vector<int16_t> &samples, bool bit)
{
	for (uint8_t i = 0; i < 4; i++)
	{
		if (bit)
		{
			h8_put_samples(samples, 2, WAVEENTRY_LOW);
			h8_put_samples(samples, 2, WAVEENTRY_HIGH);
			h8_put_samples(samples, 2, WAVEENTRY_LOW);
			h8_put_samples(samples, 2, WAVEENTRY_HIGH);
		}
		else
		{
			h8_put_samples(samples, 4, WAVEENTRY_LOW);
			h8_put_samples(samples, 4, WAVEENTRY_HIGH);
		}
	}
}


static void h8_output_byte(std::vector<int16_t> &samples, uint8_t byte)
{
	// start bit
	h8_output_bit(samples, 0);

	// data bits
	for (int i = 0; i < 8; i++)
		h8_output_bit(samples, (byte >> i) & 1);

	// stop bits
	for (int i = 0; i < 2; i++)
		h8_output_bit(samples, 1);
}


static void h8_handle_cassette(std::vector<int16_t> &samples, std::vector<uint8_t> &bytes)
{
	// leader
	for (int i = 0; i < 2000; i++)
		h8_output_bit(samples, 1);

	// data
	for (int i = 0; i < bytes.size(); i++)
		h8_output_byte(samples, bytes[i]);
}


static cassette_image::error h8_cassette_identify(cassette_image *cassette, cassette_image::Options *opts)
{
	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = H8_WAV_FREQUENCY;
	return cassette_image::error::SUCCESS;
}


static cassette_image::error h8_cassette_load(cassette_image *cassette)
{
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> bytes(file_size);
	cassette->image_read(&bytes[0], 0, file_size);
	std::vector<int16_t> samples;

	h8_handle_cassette(samples, bytes);

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / H8_WAV_FREQUENCY, samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


static const cassette_image::Format h8_cassette_image_format =
{
	"h8t",
	h8_cassette_identify,
	h8_cassette_load,
	nullptr
};


CASSETTE_FORMATLIST_START(h8_cassette_formats)
	CASSETTE_FORMAT(h8_cassette_image_format)
CASSETTE_FORMATLIST_END
