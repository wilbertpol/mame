// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/********************************************************************

Support for EACA Colour Genie .cas cassette images

Current state: Not working. Only the sync signal and 0x66 byte get
               recognized.

NOTE: There exist multiples type of .cas files for Colour Genie
 - the original one from Jurgen's emu, which starts with TAPE_HEADER
   below, followed by the sync signal, without the 255 leading 0xaa
   bytes (which are added at loading time)
 - a newer type from Genieous emu, which does not start with TAPE_HEADER
   but contains the 255 leading 0xaa bytes (which are now skipped below)
 - an alternative type (from Genieous as well?) without TAPE_HEADER
   and without the 255 leading 0xaa bytes
We now support these three types below...

********************************************************************/
#include "cgen_cas.h"

#include <cstring>


#define TAPE_HEADER "Colour Genie - Virtual Tape File"

#define SMPLO   -32768
#define SMPHI   32767


static void cgenie_output_byte(std::vector<int16_t> &samples, bool &level, uint8_t data)
{
	for (int i = 0; i < 8; i++)
	{
		// Output bit boundary
		level = !level;
		samples.push_back(level ? SMPHI : SMPLO);

		// Output bit
		if (data & 0x80)
			level = !level;
		samples.push_back(level ? SMPHI : SMPLO);

		data <<= 1;
	}
}


static cassette_image::error cgenie_handle_cas(std::vector<int16_t> &samples, std::vector<uint8_t> &bytes)
{
	int data_pos = 0;
	bool level = false;

	// Check for presence of optional header
	if (bytes.size() >= sizeof(TAPE_HEADER) && !memcmp(&bytes[0], TAPE_HEADER, sizeof(TAPE_HEADER) - 1))
	{
		// Search for 0x00 or end of file
		while (data_pos < bytes.size() && bytes[data_pos])
			data_pos++;

		// If we're at the end of the file it's not a valid .cas file
		if (data_pos == bytes.size())
			return cassette_image::error::INVALID_IMAGE;

		// Skip the 0x00 byte
		data_pos++;
	}

	// If we're at the end of the file it's not a valid .cas file
	if (data_pos == bytes.size())
		return cassette_image::error::INVALID_IMAGE;

	// Check for beginning of tape file marker (possibly skipping the 0xaa header)
	if (bytes[data_pos] != 0x66 && data_pos + 0xff <= bytes.size() && bytes[data_pos + 0xff] != 0x66)
		return cassette_image::error::INVALID_IMAGE;

	// Create header, if not present in the file
	if (bytes[data_pos] == 0x66)
		for (int i = 0; i < 256; i++)
			cgenie_output_byte(samples, level, 0xaa);

	// Start outputting data
	while (data_pos < bytes.size())
	{
		cgenie_output_byte(samples, level, bytes[data_pos]);
		data_pos++;
	}
	cgenie_output_byte(samples, level, 0x00);

	return cassette_image::error::SUCCESS;
}


static cassette_image::error cgenie_cas_identify(cassette_image *cassette, cassette_image::Options *opts)
{
	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = 2400;
	return cassette_image::error::SUCCESS;
}


static cassette_image::error cgenie_cas_load(cassette_image *cassette)
{
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> bytes(file_size);
	cassette->image_read(&bytes[0], 0, file_size);
	std::vector<int16_t> samples;

	cassette_image::error err = cgenie_handle_cas(samples, bytes);
	if (err != cassette_image::error::SUCCESS)
		return err;

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / 2400, samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


static const cassette_image::Format cgenie_cas_format =
{
	"cas",
	cgenie_cas_identify,
	cgenie_cas_load,
	nullptr
};


CASSETTE_FORMATLIST_START(cgenie_cassette_formats)
	CASSETTE_FORMAT(cgenie_cas_format)
CASSETTE_FORMATLIST_END
