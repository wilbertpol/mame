// license:BSD-3-Clause
// copyright-holders:Robbbert
/********************************************************************

Support for Sanyo PHC25 cassette images

phc images consist of 5 sections
1. 10 A5 bytes
2. 6  name of the program
3. The basic program file. Each line is delimited by null. This
   section is terminated by 3 nulls (1 of them for the preceeding
   line, the other 2 indicate the end).
4. The line numbers and pointers to them. It ends at the image size-1
5. A 1-byte trailer of FF which we do not pass on

Each byte after conversion becomes a start bit, bit 0,1,etc to 7,
then 4 stop bits.

An actual tape consists of 6 sections
a. 2.656secs of silence
b. 4.862secs of high bits
c. The header which is parts 1 and 2 above
d. 0.652secs of high bits
e. The main program which is parts 3 and 4 above
f. 1.771secs of silence

We don't emulate the full silence and high-bits periods, only just
enough to make it work.

********************************************************************/

#include "phc25_cas.h"

#define WAVEENTRY_LOW  -32768
#define WAVEENTRY_HIGH  32767

#define PHC25_WAV_FREQUENCY   9600
#define PHC25_HEADER_BYTES    16


static void phc25_put_samples(std::vector<int16_t> &samples, int count, int level)
{
	for (int i = 0; i < count; i++)
		samples.push_back(level);
}


static void phc25_output_bit(std::vector<int16_t> &samples, bool bit)
{
	if (bit)
	{
		phc25_put_samples(samples, 2, WAVEENTRY_LOW);
		phc25_put_samples(samples, 2, WAVEENTRY_HIGH);
		phc25_put_samples(samples, 2, WAVEENTRY_LOW);
		phc25_put_samples(samples, 2, WAVEENTRY_HIGH);
	}
	else
	{
		phc25_put_samples(samples, 4, WAVEENTRY_LOW);
		phc25_put_samples(samples, 4, WAVEENTRY_HIGH);
	}
}


static void phc25_output_byte(std::vector<int16_t> &samples, uint8_t byte)
{
	// start
	phc25_output_bit(samples, 0);

	// data
	for (int i = 0; i < 8; i++)
		phc25_output_bit(samples, (byte >> i) & 1);

	// stop
	for (int i = 0; i < 4; i++)
		phc25_output_bit(samples, 1);
}


static void phc25_handle_cassette(std::vector<int16_t> &samples, std::vector<uint8_t> &bytes)
{
	uint32_t byte_count = 0;

	// silence
//  sample_count += phc25_put_samples(buffer, 6640, 2, WAVEENTRY_HIGH);

	// start
//  for (i=0; i<12155; i++)
	for (int i = 0; i < 2155; i++)
		phc25_output_bit(samples, 1);

	// header
	for (int i = 0; i < PHC25_HEADER_BYTES && i < bytes.size(); i++)
		phc25_output_byte(samples, bytes[i]);

	byte_count = PHC25_HEADER_BYTES;

	// pause
	for (int i = 0; i < 1630; i++)
		phc25_output_bit(samples, 1);

	// data
	for (int i = byte_count; i < bytes.size() - 1; i++)
		phc25_output_byte(samples, bytes[i]);

	// silence
	// The original code put the bit of silence at sample position 1000, was that really needed? The
	// images seem to load fine without it.
	//sample_count += phc25_put_samples(buffer, 1000, 2, WAVEENTRY_HIGH);
}


static cassette_image::error phc25_cassette_identify(cassette_image *cassette, cassette_image::Options *opts)
{
	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = PHC25_WAV_FREQUENCY;
	return cassette_image::error::SUCCESS;
}


static cassette_image::error phc25_cassette_load(cassette_image *cassette)
{
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> bytes(file_size);
	cassette->image_read(&bytes[0], 0, file_size);
	std::vector<int16_t> samples;

	 phc25_handle_cassette(samples, bytes);

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / PHC25_WAV_FREQUENCY, samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


static const cassette_image::Format phc25_cassette_image_format =
{
	"phc",
	phc25_cassette_identify,
	phc25_cassette_load,
	nullptr
};


CASSETTE_FORMATLIST_START(phc25_cassette_formats)
	CASSETTE_FORMAT(phc25_cassette_image_format)
CASSETTE_FORMATLIST_END
