// license:BSD-3-Clause
// copyright-holders:Robbbert
/********************************************************************

Support for Goldstar FC-100 cassette images


We don't actually have any info on the cassette frequencies, so
it's all a guess.

********************************************************************/

#include "fc100_cas.h"

#define WAVEENTRY_LOW  -32768
#define WAVEENTRY_HIGH  32767

#define FC100_WAV_FREQUENCY   9600
#define FC100_HEADER_BYTES    16


static void fc100_put_samples(std::vector<int16_t> &samples, int count, int level)
{
	for (int i = 0; i < count; i++)
		samples.push_back(level);
}


static void fc100_output_bit(std::vector<int16_t> &samples, bool bit)
{
	if (bit)
	{
		fc100_put_samples(samples, 2, WAVEENTRY_LOW);
		fc100_put_samples(samples, 2, WAVEENTRY_HIGH);
		fc100_put_samples(samples, 2, WAVEENTRY_LOW);
		fc100_put_samples(samples, 2, WAVEENTRY_HIGH);
	}
	else
	{
		fc100_put_samples(samples, 4, WAVEENTRY_LOW);
		fc100_put_samples(samples, 4, WAVEENTRY_HIGH);
	}
}


static void fc100_output_byte(std::vector<int16_t> &samples, uint8_t byte)
{
	// start
	fc100_output_bit(samples, 0);

	// data
	for (int i = 0; i < 8; i++)
		fc100_output_bit(samples, (byte >> i) & 1);

	// stop
	for (int i = 0; i < 4; i++)
		fc100_output_bit(samples, 1);
}


static void fc100_handle_cassette(std::vector<int16_t> &samples, std::vector<uint8_t> &bytes)
{
	// start
	for (int i = 0; i < 2155; i++)
		fc100_output_bit(samples, 1);

	// header
	for (int i = 0; i < FC100_HEADER_BYTES; i++)
		fc100_output_byte(samples, bytes[i]);

	// pause
	for (int i = 0; i < 1630; i++)
		fc100_output_bit(samples, 1);

	// data
	for (int i = FC100_HEADER_BYTES; i < bytes.size(); i++)
		fc100_output_byte(samples, bytes[i]);
}


static cassette_image::error fc100_cassette_identify(cassette_image *cassette, cassette_image::Options *opts)
{
	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = FC100_WAV_FREQUENCY;

	if (cassette->image_size() < FC100_HEADER_BYTES)
		return cassette_image::error::INVALID_IMAGE;

	return cassette_image::error::SUCCESS;
}


static cassette_image::error fc100_cassette_load(cassette_image *cassette)
{
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> bytes(file_size);
	cassette->image_read(&bytes[0], 0, file_size);
	std::vector<int16_t> samples;

	if (file_size < FC100_HEADER_BYTES)
		return cassette_image::error::INVALID_IMAGE;

	fc100_handle_cassette(samples, bytes);

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / FC100_WAV_FREQUENCY, samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


static const cassette_image::Format fc100_cassette_image_format =
{
	"cas",
	fc100_cassette_identify,
	fc100_cassette_load,
	nullptr
};


CASSETTE_FORMATLIST_START(fc100_cassette_formats)
	CASSETTE_FORMAT(fc100_cassette_image_format)
CASSETTE_FORMATLIST_END
