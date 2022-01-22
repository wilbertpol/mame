// license:BSD-3-Clause
// copyright-holders:Robbbert
/********************************************************************

Support for Microbee cassette images

Microbee tapes consist of 3 sections
1. A leader of 63 zeroes
2. A header which contains the program name and other info
3. The main program

Each byte after conversion becomes a start bit, bit 0,1,etc to 7,
then 2 stop bits.

At 1200 baud, a high = 2 cycles of 2400Hz and a low = 1 cycle of 1200Hz
At 300 baud, a high = 8 cycles of 2400Hz and a low = 4 cycles of 1200Hz

The header bytes are arranged thus:
1 (SOH) 0x01
6 File name
1 file type (M=machine language, B=Basic)
2 length
2 load address
2 exec address
1 tape speed (0 = 300 baud; other = 1200 baud)
1 auto-start (0 = no)
1 unassigned byte
1 CRC byte

The header is always at 300 baud; the program will be at the
speed indicated by the speed byte.

By coincidence (or not), the header is the same format as that
of the Sorcerer and SOL-20. In these, the speed and auto-start
bytes are unassigned. The CRC uses the same algorithm.

The main program is broken into blocks of 256, with each block
having its own CRC byte.

Microbee tape and quickload formats:

BEE - straight binary dump to address 0900, no header. For Machine
      Language programs.

BIN - the standard z80bin format.

COM - straight binary dump to address 0100, no header. For Machine
      Language programs.

MWB - straight binary dump to address 08C0, no header. For BASIC
      programs.

TAP - has an ID header of TAP_DGOS_BEE or MBEE, null terminated.
      This is followed by the binary dump with the leader and CRC
      bytes included.

********************************************************************/

#include "mbee_cas.h"

#define WAVEENTRY_LOW  -32768
#define WAVEENTRY_HIGH  32767

#define MBEE_WAV_FREQUENCY   9600


static void mbee_put_samples(std::vector<int16_t> &samples, int count, int level)
{
	for (int i=0; i<count; i++)
		samples.push_back(level);
}


static void mbee_output_bit(std::vector<int16_t> &samples, bool mbee_speed, bool bit)
{
	if (mbee_speed)
	{
		if (bit)
		{
			mbee_put_samples(samples, 2, WAVEENTRY_LOW);
			mbee_put_samples(samples, 2, WAVEENTRY_HIGH);
			mbee_put_samples(samples, 2, WAVEENTRY_LOW);
			mbee_put_samples(samples, 2, WAVEENTRY_HIGH);
		}
		else
		{
			mbee_put_samples(samples, 4, WAVEENTRY_LOW);
			mbee_put_samples(samples, 4, WAVEENTRY_HIGH);
		}
	}
	else
	{
		if (bit)
		{
			for (int i = 0; i < 8; i++)
			{
				mbee_put_samples(samples, 2, WAVEENTRY_LOW);
				mbee_put_samples(samples, 2, WAVEENTRY_HIGH);
			}
		}
		else
		{
			for (int i = 0; i < 4; i++)
			{
				mbee_put_samples(samples, 4, WAVEENTRY_LOW);
				mbee_put_samples(samples, 4, WAVEENTRY_HIGH);
			}
		}
	}
}


static void mbee_output_byte(std::vector<int16_t> &samples, bool mbee_speed, uint8_t byte)
{
	// start
	mbee_output_bit(samples, mbee_speed, 0);

	// data
	for (int i = 0; i < 8; i++)
		mbee_output_bit(samples, mbee_speed, (byte >> i) & 1);

	// stop 
	for (int i = 0; i < 2; i++)
		mbee_output_bit(samples, mbee_speed, 1);
}


static void mbee_handle_tap(std::vector<int16_t> &samples, std::vector<uint8_t> &bytes)
{
	uint32_t byte_count = 0;
	bool mbee_speed;

	// TAP file starts with a null-terminate ID string. We just skip this.
	while (byte_count < bytes.size() && bytes[byte_count])
		byte_count++;

	// there can be a library of files, loop through them all
	while (byte_count < bytes.size())
	{
		mbee_speed = false;

		// now output the leader
		while (byte_count < bytes.size() && !bytes[byte_count])
			mbee_output_byte(samples, mbee_speed, bytes[byte_count++]);

		// make sure SOH is where we expect
		if (bytes[byte_count] != 1 || byte_count + 18 >= bytes.size())
			break;

		// store the size for later
		uint8_t temp_blocks = bytes[byte_count + 9];
		uint16_t temp_size = bytes[byte_count + 8] + temp_blocks*256;

		// store the speed for later
		bool temp_speed = bytes[byte_count + 15];

		// header
		for (int i = 0; i < 18; i++)
			mbee_output_byte(samples, mbee_speed, bytes[byte_count++]);

		// change speed
		mbee_speed = temp_speed;

		// calculate size of program including CRC bytes
		temp_size = temp_size + temp_blocks + 1;

		// data
		for (int i=0; i < temp_size && byte_count < bytes.size(); i++)
			mbee_output_byte(samples, mbee_speed, bytes[byte_count++]);
	}
}


static cassette_image::error mbee_tap_identify(cassette_image *cassette, cassette_image::Options *opts)
{
	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = MBEE_WAV_FREQUENCY;
	return cassette_image::error::SUCCESS;
}


static cassette_image::error mbee_tap_load(cassette_image *cassette)
{
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> bytes(file_size);
	cassette->image_read(&bytes[0], 0, file_size);
	std::vector<int16_t> samples;

	mbee_handle_tap(samples, bytes);

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / MBEE_WAV_FREQUENCY, samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


static const cassette_image::Format mbee_tap_image_format =
{
	"tap",
	mbee_tap_identify,
	mbee_tap_load,
	nullptr
};


CASSETTE_FORMATLIST_START(mbee_cassette_formats)
	CASSETTE_FORMAT(mbee_tap_image_format)
CASSETTE_FORMATLIST_END
