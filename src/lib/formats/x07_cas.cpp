// license:BSD-3-Clause
// copyright-holders:Sandro Ronco
/********************************************************************

    Support for Canon X-07 cassette images

********************************************************************/

#include "x07_cas.h"

#define WAVEENTRY_LOW  -32768
#define WAVEENTRY_HIGH  32767

#define X07_WAV_FREQUENCY   4800
#define X07_TIMER_FREQUENCY 1200
#define X07_BIT_LENGTH      (X07_WAV_FREQUENCY/X07_TIMER_FREQUENCY)
#define X07_HEADER_BYTES    16


static void x07_put_samples(std::vector<int16_t> &samples, int count, int level)
{
	for (int i = 0; i < count; i++)
		samples.push_back(level);
}


static void x07_output_bit(std::vector<int16_t> &samples, uint8_t bit)
{
	if (bit)
	{
		x07_put_samples(samples, X07_BIT_LENGTH/4, WAVEENTRY_HIGH);
		x07_put_samples(samples, X07_BIT_LENGTH/4, WAVEENTRY_LOW);
		x07_put_samples(samples, X07_BIT_LENGTH/4, WAVEENTRY_HIGH);
		x07_put_samples(samples, X07_BIT_LENGTH/4, WAVEENTRY_LOW);
	}
	else
	{
		x07_put_samples(samples, X07_BIT_LENGTH/2, WAVEENTRY_HIGH);
		x07_put_samples(samples, X07_BIT_LENGTH/2, WAVEENTRY_LOW);
	}
}


static void x07_output_byte(std::vector<int16_t> &samples, uint8_t byte)
{
	/* start */
	x07_output_bit(samples, 0);

	/* data */
	for (int i = 0; i < 8; i++)
		x07_output_bit(samples, (byte >> i) & 0x01);

	/* stop */
	x07_output_bit(samples, 1);
	x07_output_bit(samples, 1);
	x07_output_bit(samples, 1);
}


static void x07_handle_cassette(std::vector<int16_t> &samples, std::vector<uint8_t> &bytes)
{
	int img_start = 0;

	/* start */
	for (int i = 0; i < X07_WAV_FREQUENCY; i++)
		x07_output_bit(samples, 1);

	/* header */
	if (bytes[0] == 0xd3 && bytes[1] == 0xd3 && bytes[2] == 0xd3 && bytes[3] == 0xd3)
	{
		// valid header
		for (int i = 0; i < X07_HEADER_BYTES; i++)
			x07_output_byte(samples, bytes[i]);

		img_start = X07_HEADER_BYTES;
	}
	else
	{
		// remove the nullptr chars at start
		while (!bytes[img_start])
			img_start++;

		// insert 0xd3 bytes
		for (int i = 0; i < 10; i++)
			x07_output_byte(samples, 0xd3);

		// fake filename
		for (int i = 0; i < 6; i++)
			x07_output_byte(samples, 'A');
	}

	/* pause */
	for (int i = 0; i < X07_WAV_FREQUENCY/16; i++)
		x07_output_bit(samples, 1);

	/* data */
	for (int i = img_start; i < bytes.size(); i++)
		x07_output_byte(samples, bytes[i]);

	/* end */
	for (int i = 0; i < X07_WAV_FREQUENCY/8; i++)
		x07_output_bit(samples, 1);
}


static cassette_image::error x07_cassette_identify(cassette_image *cassette, cassette_image::Options *opts)
{
	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = X07_WAV_FREQUENCY;
	return cassette_image::error::SUCCESS;
}


static cassette_image::error x07_cassette_load(cassette_image *cassette)
{
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> bytes(file_size);
	cassette->image_read(&bytes[0], 0, file_size);
	std::vector<int16_t> samples;

	if (file_size < 4)
		return cassette_image::error::INVALID_IMAGE;

	x07_handle_cassette(samples, bytes);

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / X07_WAV_FREQUENCY, samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


static const cassette_image::Format x07_cassette_image_format =
{
	"k7,lst,cas",
	x07_cassette_identify,
	x07_cassette_load,
	nullptr
};


CASSETTE_FORMATLIST_START(x07_cassette_formats)
	CASSETTE_FORMAT(x07_cassette_image_format)
CASSETTE_FORMATLIST_END
