// license:BSD-3-Clause
// copyright-holders:Robbbert
/********************************************************************

Support for Exidy Sorcerer cassette images


Sorcerer tapes consist of these sections:
1. A high tone whenever idle
2. A header
3. The data, in blocks of 256 bytes plus a CRC byte
4. The last block may be shorter, depending on the number of bytes
   left to save.

Each byte has 1 start bit, 8 data bits (0-7), 2 stop bits.

The default speed is 1200 baud, which is what we emulate here.
A high bit is 1 cycle of 1200 Hz, while a low bit is half a cycle
of 600 Hz.

Formats:
TAPE - this contains a byte for each real byte, including all the
header and leader bytes.


********************************************************************/

#include "sorc_cas.h"


#define WAVEENTRY_LOW  -32768
#define WAVEENTRY_HIGH  32767

#define SORCERER_WAV_FREQUENCY   4788


static void sorcerer_put_samples(std::vector<int16_t> &samples, bool &level, int count)
{
	for (int i=0; i < count; i++)
		samples.push_back(level ? WAVEENTRY_LOW : WAVEENTRY_HIGH);

	level ^= 1;
}


static void sorcerer_output_bit(std::vector<int16_t> &samples, bool &level, bool bit)
{
	if (bit)
	{
		sorcerer_put_samples(samples, level, 2);
		sorcerer_put_samples(samples, level, 2);
	}
	else
	{
		sorcerer_put_samples(samples, level, 4);
	}
}


static void sorcerer_output_byte(std::vector<int16_t> &samples, bool &level, uint8_t byte)
{
	/* start */
	sorcerer_output_bit(samples, level, 0);

	/* data */
	for (int i = 0; i < 8; i++)
		sorcerer_output_bit(samples, level, (byte >> i) & 1);

	/* stop */
	for (int i = 0; i < 2; i++)
		sorcerer_output_bit(samples, level, 1);
}


static void sorcerer_handle_cassette(std::vector<int16_t> &samples, std::vector<uint8_t> &bytes)
{
	bool level = false;
	/* idle */
	for (int i = 0; i < 2000; i++)
		sorcerer_output_bit(samples, level, 1);

	/* data */
	for (int i = 0; i < bytes.size(); i++)
		sorcerer_output_byte(samples, level, bytes[i]);
}


static cassette_image::error sorcerer_cassette_identify(cassette_image *cassette, cassette_image::Options *opts)
{
	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = SORCERER_WAV_FREQUENCY;
	return cassette_image::error::SUCCESS;
}


static cassette_image::error sorcerer_cassette_load(cassette_image *cassette)
{
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> bytes(file_size);
	cassette->image_read(&bytes[0], 0, file_size);
	std::vector<int16_t> samples;

	sorcerer_handle_cassette(samples, bytes);

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / SORCERER_WAV_FREQUENCY, samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


static const cassette_image::Format sorcerer_cassette_image_format =
{
	"tape",
	sorcerer_cassette_identify,
	sorcerer_cassette_load,
	nullptr
};


CASSETTE_FORMATLIST_START(sorcerer_cassette_formats)
	CASSETTE_FORMAT(sorcerer_cassette_image_format)
CASSETTE_FORMATLIST_END
