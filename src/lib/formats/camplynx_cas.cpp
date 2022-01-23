// license:BSD-3-Clause
// copyright-holders:Robbbert,Nigel Barnes
/********************************************************************

Support for Camputers Lynx cassette images


We support TAP files used by the Pale and Jynx emulators.

Tape format:
- about 7 seconds of zeroes
- A5 byte
- 22 byte
- program name
- 22 byte
- about 7 seconds of zeroes
- A5 byte
- header
- main program
- checksum

Each byte is 8 bits (MSB first) with no start or stop bits.

********************************************************************/

#include "camplynx_cas.h"

#include "osdcore.h" // osd_printf_*


#define WAVEENTRY_LOW  -32768
#define WAVEENTRY_HIGH  32767

#define LYNX48K_WAV_FREQUENCY   4000
#define LYNX128K_WAV_FREQUENCY  8000


static void camplynx_put_samples(std::vector<int16_t> &samples, int count, int level)
{
	for (int i = 0; i < count; i++)
		samples.push_back(level);
}

static void camplynx_output_bit(std::vector<int16_t> &samples, bool bit)
{
	if (bit)
	{
		camplynx_put_samples(samples, 4, WAVEENTRY_HIGH);
		camplynx_put_samples(samples, 4, WAVEENTRY_LOW);
	}
	else
	{
		camplynx_put_samples(samples, 2, WAVEENTRY_HIGH);
		camplynx_put_samples(samples, 2, WAVEENTRY_LOW);
	}
}


static void camplynx_output_byte(std::vector<int16_t> &samples, uint8_t byte)
{
	for (int i = 0; i < 8; i++)
		camplynx_output_bit(samples, (byte >> (7-i)) & 1);
}


static void camplynx_handle_cassette(std::vector<int16_t> &samples, std::vector<uint8_t> &bytes)
{
	uint32_t byte_count = 0;
	uint8_t file_type;
	std::string pgmname = "";

	while (byte_count < bytes.size())
	{
		/* initial SYNC + A5 applies to all file types */
		for (int i = 0; i < 555; i++)
			camplynx_output_byte(samples, 0);
		camplynx_output_byte(samples, 0xA5);

		/* some TAPs have a spurious A5 at the start, ignore */
		while (byte_count < bytes.size() && bytes[byte_count] == 0xA5)
			byte_count++;

		if (byte_count >= bytes.size())
			break;

		if (bytes[byte_count] == 0x22)
		{
			pgmname = " LOAD \"";
			byte_count++;
			camplynx_output_byte(samples, 0x22);

			/* output program name - include protection in case tape is corrupt */
			for (int i = byte_count; i < bytes.size() && bytes[i] != 0x22; i++)
			{
				camplynx_output_byte(samples, bytes[i]);
				pgmname.append(1, (char)bytes[i]);
				byte_count++;
			}

			if (byte_count >= bytes.size())
				break;

			pgmname.append(1, (char)0x22);
			camplynx_output_byte(samples, bytes[byte_count++]); // should be 0x22

			if (byte_count >= bytes.size())
				break;
			// read file type letter, should be 'B' or 'M'
			file_type = bytes[byte_count];

			// if a machine-language program, say to use MLOAD
			if (file_type == 'M') pgmname[0] = 'M';

			// tell user how to load the tape
			osd_printf_info("%s\n", pgmname);

			/* second SYNC + A5 */
			for (int i = 0; i < 555; i++)
				camplynx_output_byte(samples, 0);
			camplynx_output_byte(samples, 0xA5);
		}

		/* read file type letter, should be 'A', 'B' or 'M' */
		file_type = bytes[byte_count];

		/* determine the data size (as recorded in the file) + extra bytes per file type */
		uint32_t data_size = 0;
		switch (file_type)
		{
		case 'A':
			if (byte_count + 4 >= bytes.size())
				break;
			data_size = 5 + ((bytes[byte_count + 4]) << 8 | bytes[byte_count + 3]) + 12;
			break;
		case 'B':
			if (byte_count + 2 >= bytes.size())
				break;
			data_size = 3 + ((bytes[byte_count + 2]) << 8 | bytes[byte_count + 1]) + 3;
			break;
		case 'M':
			if (byte_count + 2 >= bytes.size())
				break;
			data_size = 3 + ((bytes[byte_count + 2]) << 8 | bytes[byte_count + 1]) + 7;
			break;
		}

		// output data  - include protection in case tape is corrupt
		for (int i = byte_count; i < byte_count + data_size; i++)
		{
			if (i < bytes.size())
			{
				camplynx_output_byte(samples, bytes[i]);
			}
		}
		byte_count += data_size;

		// some TAPs have a spurious 00 at the end, ignore
		while (byte_count < bytes.size() && bytes[byte_count] == 0x00)
			byte_count++;
	}
}


static cassette_image::error lynx48k_cassette_identify(cassette_image *cassette, cassette_image::Options *opts)
{
	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = LYNX48K_WAV_FREQUENCY;
	return cassette_image::error::SUCCESS;
}


static cassette_image::error lynx128k_cassette_identify(cassette_image *cassette, cassette_image::Options *opts)
{
	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = LYNX128K_WAV_FREQUENCY;
	return cassette_image::error::SUCCESS;
}


static cassette_image::error lynx48k_cassette_load(cassette_image *cassette)
{
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> bytes(file_size);
	cassette->image_read(&bytes[0], 0, file_size);
	std::vector<int16_t> samples;

	camplynx_handle_cassette(samples, bytes);

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / LYNX48K_WAV_FREQUENCY, samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


static cassette_image::error lynx128k_cassette_load(cassette_image *cassette)
{
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> bytes(file_size);
	cassette->image_read(&bytes[0], 0, file_size);
	std::vector<int16_t> samples;

	camplynx_handle_cassette(samples, bytes);

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / LYNX128K_WAV_FREQUENCY, samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


static const cassette_image::Format lynx48k_cassette_image_format =
{
	"tap",
	lynx48k_cassette_identify,
	lynx48k_cassette_load,
	nullptr
};


static const cassette_image::Format lynx128k_cassette_image_format =
{
	"tap",
	lynx128k_cassette_identify,
	lynx128k_cassette_load,
	nullptr
};


CASSETTE_FORMATLIST_START(lynx48k_cassette_formats)
	CASSETTE_FORMAT(lynx48k_cassette_image_format)
CASSETTE_FORMATLIST_END


CASSETTE_FORMATLIST_START(lynx128k_cassette_formats)
	CASSETTE_FORMAT(lynx128k_cassette_image_format)
CASSETTE_FORMATLIST_END
