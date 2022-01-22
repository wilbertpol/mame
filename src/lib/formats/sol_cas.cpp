// license:BSD-3-Clause
// copyright-holders:Robbbert
/********************************************************************

Support for SOL-20 cassette images


SOL20 tapes consist of these sections:
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
SVT - The full explanation may be found on the Solace web site,
      however this is a summary of what we support.
      C (carrier) time in decaseconds
      D (data bytes) in ascii text
      H (header) tape header info
      Multiple programs
      Unsupported:
      B (set baud rate) B 300 or B 1200
      F load ENT file
      S (silence) time in decaseconds
      bad-byte symbols
      escaped characters

********************************************************************/

#include "sol_cas.h"


#define WAVEENTRY_LOW  -32768
#define WAVEENTRY_HIGH  32767

#define SOL20_WAV_FREQUENCY   4800


static void sol20_put_samples(std::vector<int16_t> &samples, bool &level, int count)
{
	for (int i=0; i < count; i++)
		samples.push_back(level ? WAVEENTRY_LOW : WAVEENTRY_HIGH);

	level ^= 1;
}


static void sol20_output_bit(std::vector<int16_t> &samples, bool &level, bool bit)
{
	if (bit)
	{
		sol20_put_samples(samples, level, 2);
		sol20_put_samples(samples, level, 2);
	}
	else
	{
		sol20_put_samples(samples, level, 4);
	}
}


static void sol20_output_byte(std::vector<int16_t> &samples, bool &level, uint8_t byte)
{
	/* start */
	sol20_output_bit(samples, level, 0);

	/* data */
	for (int i = 0; i < 8; i++)
		sol20_output_bit(samples, level, (byte >> i) & 1);

	/* stop */
	for (int i = 0; i < 2; i++)
		sol20_output_bit(samples, level, 1);
}


// Calculate checksum
static uint8_t sol20_calc_cksm(uint8_t cksm, uint8_t data)
{
	data -= cksm;
	cksm = data;
	data ^= cksm;
	data ^= 0xff;
	data -= cksm;
	return data;
}


// Ignore remainder of line
static void sol20_scan_to_eol(std::vector<uint8_t> &bytes, uint32_t &sol20_byte_num)
{
	bool t = true;
	while (t)
	{
		if (sol20_byte_num >= bytes.size())
		{
			sol20_byte_num = 0;
			t = false;
		}
		else
			if (bytes[sol20_byte_num] == 0x0d)
				t = false;
			else
				sol20_byte_num++;
	}
}


// skip spaces and symbols looking for a hex digit
static void sol20_scan_to_hex(std::vector<uint8_t> &bytes, uint32_t &sol20_byte_num)
{
	bool t = true;
	while (t)
	{
		if (sol20_byte_num >= bytes.size())
		{
			t = false;
		}
		else
		{
			uint8_t chr = bytes[sol20_byte_num];
			if (chr == 0x0d)
				t = false;
			else
			if (((chr >= '0') && (chr <= '9')) || ((chr >= 'A') && (chr <= 'F')))
				t = false;
			else
				sol20_byte_num++;
		}
	}
}


// Turn n digits into hex
static int sol20_read_hex(std::vector<uint8_t> &bytes, uint8_t numdigits, uint32_t &sol20_byte_num)
{
	int data = 0;
	uint8_t chr;

	for (int i = 0; i < numdigits; i++)
	{
		chr = bytes[sol20_byte_num];
		if ((chr >= '0') && (chr <= '9'))
		{
			data = (data << 4) | (chr-48);
			sol20_byte_num++;
		}
		else
		if ((chr >= 'A') && (chr <= 'F'))
		{
			data = (data << 4) | (chr-55);
			sol20_byte_num++;
		}
		else
			i = numdigits;
	}
	return data;
}


// Turn digits into decimal
static int sol20_read_dec(std::vector<uint8_t> &bytes, uint32_t &sol20_byte_num)
{
	int data = 0;

	while (sol20_byte_num < bytes.size() && (bytes[sol20_byte_num] >= '0') && (bytes[sol20_byte_num] <= '9'))
	{
		data = data * 10 + bytes[sol20_byte_num] - 48;
		sol20_byte_num++;
	}

	return data;
}


static void sol20_handle_cassette(std::vector<int16_t> &samples, std::vector<uint8_t> &bytes)
{
	uint32_t i = 0,t = 0;
	uint16_t cc = 0;
	uint32_t sol20_byte_num = 0;
	bool process_d = 0;
	uint16_t length = 0;
	bool level = false;
	uint8_t sol20_cksm_byte;
	uint8_t sol20_header[16];

	// ignore remainder of line
	sol20_scan_to_eol(bytes, sol20_byte_num);

	// process the commands
	while (sol20_byte_num < bytes.size())
	{
		if (sol20_byte_num + 2 >= bytes.size())
			break;
		sol20_byte_num += 2; // bump to start of next line
		uint8_t chr = bytes[sol20_byte_num];  // Get command
		switch (chr)
		{
			case 0x0d:
				break;
			case 'C': // carrier
				{
					if (cc) // if this is the next file, clean up after the previous one
					{
						sol20_output_byte(samples, level, sol20_cksm_byte); // final checksum if needed
						cc = 0;
					}

					sol20_byte_num += 2; // bump to parameter
					t = sol20_read_dec(bytes, sol20_byte_num) * 140; // convert 10th of seconds to number of ones
					for (i = 0; i < t; i++)
						sol20_output_bit(samples, level, 1);
					sol20_scan_to_eol(bytes, sol20_byte_num);
					break;
				}
			case 'H': // header
				{
					if (cc) // if this is the next file, clean up after the previous one
					{
						sol20_output_byte(samples, level, sol20_cksm_byte); // final checksum if needed
						cc = 0;
					}

					sol20_byte_num += 2; // bump to file name
					for (i = 0; i < 5; i++)
						sol20_header[i] = 0x20;
					for (i = 0; i < 5; i++)
					{
						sol20_header[i] = bytes[sol20_byte_num++];
						if (sol20_header[i] == 0x20)
							break;
					}
					sol20_header[5] = 0;
					sol20_scan_to_hex(bytes, sol20_byte_num); // bump to file type
					sol20_header[6] = sol20_read_hex(bytes, 2, sol20_byte_num);
					sol20_scan_to_hex(bytes, sol20_byte_num); // bump to length
					length = sol20_read_hex(bytes, 4, sol20_byte_num);
					sol20_header[7] = length;
					sol20_header[8] = length >> 8;
					sol20_scan_to_hex(bytes, sol20_byte_num); // bump to load-address
					i = sol20_read_hex(bytes, 4, sol20_byte_num);
					sol20_header[9] = i;
					sol20_header[10] = i >> 8;
					sol20_scan_to_hex(bytes, sol20_byte_num); // bump to exec-address
					i = sol20_read_hex(bytes, 4, sol20_byte_num);
					sol20_header[11] = i;
					sol20_header[12] = i >> 8;
					sol20_header[13] = 0;
					sol20_header[14] = 0;
					sol20_header[15] = 0;
					sol20_cksm_byte = 0;
					for (i = 0; i < 16; i++)
						sol20_cksm_byte = sol20_calc_cksm(sol20_cksm_byte, sol20_header[i]);
					// write leader
					for (i = 0; i < 100; i++)
						sol20_output_byte(samples, level, 0);
					// write SOH
					sol20_output_byte(samples, level, 1);
					// write Header
					for (i = 0; i < 16; i++)
						sol20_output_byte(samples, level, sol20_header[i]);
					// write checksum
					sol20_output_byte(samples, level, sol20_cksm_byte);

					sol20_cksm_byte = 0;
					process_d = 1;
					sol20_scan_to_eol(bytes, sol20_byte_num);
					break;
				}
			case 'D':  // data
				{
					sol20_byte_num += 2; // bump to first byte
					while (sol20_byte_num < bytes.size() && (bytes[sol20_byte_num] != 0x0d) && process_d)
					{
						t = sol20_read_hex(bytes, 2, sol20_byte_num);
						sol20_output_byte(samples, level, t);
						cc++;
						// if it's a data byte reduce remaining length and calculate checksum;
						// tape supplies checksums except last one
						if (cc < 257)
						{
							length--;
							sol20_cksm_byte = sol20_calc_cksm(sol20_cksm_byte, t);
						}
						else
						// didnt need it, throw away
						{
							cc = 0;
							sol20_cksm_byte = 0;
						}
						// see if finished tape
						if (!length)
							process_d = 0;
						// bump to next byte
						sol20_scan_to_hex(bytes, sol20_byte_num);
					}
				}
				[[fallthrough]];
			default:  // everything else is ignored
				sol20_scan_to_eol(bytes, sol20_byte_num);
				break;
		}
	}

	if (cc)  // reached the end of the svt file
		sol20_output_byte(samples, level, sol20_cksm_byte); // final checksum if needed
}


static cassette_image::error sol20_cassette_identify(cassette_image *cassette, cassette_image::Options *opts)
{
	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = SOL20_WAV_FREQUENCY;
	return cassette_image::error::SUCCESS;
}


static cassette_image::error sol20_cassette_load(cassette_image *cassette)
{
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> bytes(file_size);
	cassette->image_read(&bytes[0], 0, file_size);
	std::vector<int16_t> samples;

	// 1st line of file must say SVT
	if (!((bytes[0] == 'S') && (bytes[1] == 'V') && (bytes[2] == 'T')))
		return cassette_image::error::INVALID_IMAGE;

	sol20_handle_cassette(samples, bytes);

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / SOL20_WAV_FREQUENCY, samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


static const cassette_image::Format sol20_cassette_image_format =
{
	"svt",
	sol20_cassette_identify,
	sol20_cassette_load,
	nullptr
};


CASSETTE_FORMATLIST_START(sol20_cassette_formats)
	CASSETTE_FORMAT(sol20_cassette_image_format)
CASSETTE_FORMATLIST_END
