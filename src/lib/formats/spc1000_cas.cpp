// license:BSD-3-Clause
// copyright-holders:Robbbert
/********************************************************************

Support for Samsung SPC-1000 cassette images


Tape formats:

TAP: This is a series of 0x30 and 0x31 bytes, representing binary
     0 and 1. It includes the header and leaders.

CAS: Files in this format consist of a 16 bytes header (SPC-1000.CASfmt )
     followed by cassette bits packed together (each byte of a .cas file
     are 8 bits, most significant bit first)

STA: This format has not been investigated yet, but is assumed to
     be the save state of some other emulator.

IPL: This seems a quickload format containing RAM dump, not a real tape

********************************************************************/

#include "spc1000_cas.h"


#define WAVEENTRY_LOW  -32768
#define WAVEENTRY_HIGH  32767

#define SPC1000_WAV_FREQUENCY   17000


static void spc1000_put_samples(std::vector<int16_t> &samples, int count, int level)
{
	for (int i=0; i<count; i++)
		samples.push_back(level);
}


static void spc1000_output_bit(std::vector<int16_t> &samples, bool bit)
{
	if (bit)
	{
		spc1000_put_samples(samples, 9, WAVEENTRY_LOW);
		spc1000_put_samples(samples, 9, WAVEENTRY_HIGH);
	}
	else
	{
		spc1000_put_samples(samples, 4, WAVEENTRY_LOW);
		spc1000_put_samples(samples, 4, WAVEENTRY_HIGH);
	}
}


static void spc1000_handle_tap(std::vector<int16_t> &samples, std::vector<uint8_t> &bytes)
{
	/* data */
	for (uint32_t i = 0; i < bytes.size(); i++)
		spc1000_output_bit(samples, bytes[i] & 1);
}


static void spc1000_handle_cas(std::vector<int16_t> &samples, std::vector<uint8_t> &bytes)
{
	/* data (skipping first 16 bytes, which is CAS header) */
	for (uint32_t i = 0x10; i < bytes.size(); i++)
		for (int j = 0; j < 8; j++)
			spc1000_output_bit(samples, (bytes[i] >> (7 - j)) & 1);
}


/*******************************************************************
   Formats
 ********************************************************************/


// TAP
static cassette_image::error spc1000_tap_cassette_identify(cassette_image *cassette, cassette_image::Options *opts)
{
	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = SPC1000_WAV_FREQUENCY;
	return cassette_image::error::SUCCESS;
}


static cassette_image::error spc1000_tap_cassette_load(cassette_image *cassette)
{
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> bytes(file_size);
	cassette->image_read(&bytes[0], 0, file_size);
	std::vector<int16_t> samples;

	spc1000_handle_tap(samples, bytes);

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / SPC1000_WAV_FREQUENCY, samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


static const cassette_image::Format spc1000_tap_cassette_image_format =
{
	"tap",
	spc1000_tap_cassette_identify,
	spc1000_tap_cassette_load,
	nullptr
};


// CAS
static cassette_image::error spc1000_cas_cassette_identify(cassette_image *cassette, cassette_image::Options *opts)
{
	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = SPC1000_WAV_FREQUENCY;
	return cassette_image::error::SUCCESS;
}


static cassette_image::error spc1000_cas_cassette_load(cassette_image *cassette)
{
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> bytes(file_size);
	cassette->image_read(&bytes[0], 0, file_size);
	std::vector<int16_t> samples;

	if (bytes.size() < 0x10)
		return cassette_image::error::INVALID_IMAGE;

	spc1000_handle_cas(samples, bytes);

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / SPC1000_WAV_FREQUENCY, samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


static const cassette_image::Format spc1000_cas_cassette_image_format =
{
	"cas",
	spc1000_cas_cassette_identify,
	spc1000_cas_cassette_load,
	nullptr
};


CASSETTE_FORMATLIST_START(spc1000_cassette_formats)
	CASSETTE_FORMAT(spc1000_tap_cassette_image_format)
	CASSETTE_FORMAT(spc1000_cas_cassette_image_format)
CASSETTE_FORMATLIST_END
