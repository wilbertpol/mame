// license:GPL-2.0+
// copyright-holders:Juergen Buchmueller, Krzysztof Strzecha, Robbbert
/*****************************************************************************

Taken from nocash ZX81 docs by Martin Korth.

ZX81 Cassette File Structure
  5 seconds    pilot
  1-127 bytes  name (bit7 set in last char)
  LEN bytes    data, loaded to address 4009h, LEN=(4014h)-4009h.
  1 pulse      video retrace signal (only if display was enabled)
The data field contains the system area, the basic program, the video memory,
and VARS area.

ZX80 Cassette File Structure
  5 seconds    pilot
  LEN bytes    data, loaded to address 4000h, LEN=(400Ah)-4000h.
ZX80 files do not have filenames, and video memory is not included in the file.

Bits and Bytes
Each byte consists of 8 bits (MSB first) without any start and stop bits,
directly followed by the next byte. A "0" bit consists of four high pulses,
a "1" bit of nine pulses, either one followed by a silence period.
  0:  /\/\/\/\________
  1:  /\/\/\/\/\/\/\/\/\________
Each pulse is split into a 150us High period, and 150us Low period. The
duration of the silence between each bit is 1300us. The baud rate is thus 400
bps (for a "0" filled area) downto 250 bps (for a "1" filled area). Average
medium transfer rate is approx. 307 bps (38 bytes/sec) for files that contain
50% of "0" and "1" bits each.

*****************************************************************************/

#include "zx81_p.h"
#include "tzx_cas.h"


#define WAVEENTRY_LOW   -32768
#define WAVEENTRY_HIGH   32767
#define WAVEENTRY_ZERO       0

#define ZX81_WAV_FREQUENCY  44100

/* all following are in samples */
#define ZX81_PULSE_LENGTH   16
#define ZX81_PAUSE_LENGTH   56
#define ZX81_PILOT_LENGTH   220500

#define ZX81_LOW_BIT_LENGTH (ZX81_PULSE_LENGTH*4+ZX81_PAUSE_LENGTH)
#define ZX81_HIGH_BIT_LENGTH    (ZX81_PULSE_LENGTH*9+ZX81_PAUSE_LENGTH)

#define ZX81_START_LOAD_ADDRESS 0x4009
#define ZX80_START_LOAD_ADDRESS 0x4000
#define ZX81_DATA_LENGTH_OFFSET 0x0b
#define ZX80_DATA_LENGTH_OFFSET 0x04


static void zx81_emit_level(std::vector<int16_t> &samples, int count, int level)
{
	for (int i = 0; i < count; i++) samples.push_back(level);
}


static void zx81_emit_pulse(std::vector<int16_t> &samples)
{
	zx81_emit_level(samples, ZX81_PULSE_LENGTH/8, WAVEENTRY_LOW);
	zx81_emit_level(samples, ZX81_PULSE_LENGTH/8, WAVEENTRY_LOW);
	zx81_emit_level(samples, ZX81_PULSE_LENGTH/8, WAVEENTRY_ZERO);
	zx81_emit_level(samples, ZX81_PULSE_LENGTH/8, WAVEENTRY_HIGH);
	zx81_emit_level(samples, ZX81_PULSE_LENGTH/8, WAVEENTRY_HIGH);
	zx81_emit_level(samples, ZX81_PULSE_LENGTH/8, WAVEENTRY_ZERO);
	zx81_emit_level(samples, ZX81_PULSE_LENGTH/8, WAVEENTRY_LOW);
	zx81_emit_level(samples, ZX81_PULSE_LENGTH/8, WAVEENTRY_LOW);
}


static void zx81_emit_pause(std::vector<int16_t> &samples)
{
	zx81_emit_level(samples, ZX81_PAUSE_LENGTH, WAVEENTRY_ZERO);
}


static void zx81_output_bit(std::vector<int16_t> &samples, uint8_t bit)
{
	if (bit)
		for (int i = 0; i < 9; i++)
			zx81_emit_pulse(samples);
	else
		for (int i = 0; i < 4; i++)
			zx81_emit_pulse(samples);

	zx81_emit_pause(samples);
}


static void zx81_output_byte(std::vector<int16_t> &samples, uint8_t byte)
{
	for (int i = 0; i < 8; i++)
		zx81_output_bit(samples, (byte >> (7 - i)) & 0x01);
}

/* ZX-81 functions */

static const uint8_t zx81_chars[]={
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*00h-07h*/
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*08h-0fh*/
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*10h-17h*/
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /*18h-1fh*/
	0x00, 0x00, 0x0b, 0x00, 0x0d, 0x00, 0x00, 0x00, /*20h-27h*/
	0x10, 0x11, 0x17, 0x15, 0x1a, 0x16, 0x1b, 0x18, /*28h-2fh*/
	0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, /*30h-37h*/
	0x24, 0x25, 0x0e, 0x19, 0x13, 0x14, 0x12, 0x0f, /*38h-3fh*/
	0x00, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, /*40h-47h*/
	0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33, 0x34, /*48h-4fh*/
	0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, /*50h-57h*/
	0x3d, 0x3e, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, /*58h-5fh*/
	0x00, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, /*60h-67h*/
	0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33, 0x34, /*68h-6fh*/
	0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, /*70h-77h*/
	0x3d, 0x3e, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, /*78h-7fh*/
};


static uint8_t zx81_fill_file_name(const char* name, uint8_t *zx_file_name)
{
	uint8_t length;
	for (length = 0; (length<128) && name[length]; length++)
		zx_file_name[length] = ((uint8_t) name[length]<0x80) ? zx81_chars[(uint8_t) name[length]] : 0x00;
	zx_file_name[length - 1] |= 0x80;
	return length;
}


static cassette_image::error zx81_p_identify(cassette_image *cassette, cassette_image::Options *opts)
{
	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = ZX81_WAV_FREQUENCY;
	return cassette_image::error::SUCCESS;
}


static cassette_image::error zx81_p_load(cassette_image *cassette)
{
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> file_contents(file_size);
	cassette->image_read(&file_contents[0], 0, file_size);

	std::vector<int16_t> samples;
	uint8_t zx_file_name[128];
	/* The filename of the file is used to create the wave stream for the emulated machine. Why is this information not
	   part of the image file itself?
	   Hardcoding this to "cassette".
	*/
	uint8_t zx_file_name_length = zx81_fill_file_name("cassette", zx_file_name);

	/* pilot */
	zx81_emit_level(samples, ZX81_PILOT_LENGTH, WAVEENTRY_ZERO);

	/* name */
	for (int i = 0; i < zx_file_name_length; i++)
		zx81_output_byte(samples, zx_file_name[i]);

	/* data */
	for (uint64_t i = 0; i < file_size; i++)
		zx81_output_byte(samples, file_contents[i]);

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / ZX81_WAV_FREQUENCY, samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


static const cassette_image::Format zx81_p_image_format =
{
	"p,81",
	zx81_p_identify,
	zx81_p_load,
	nullptr
};


CASSETTE_FORMATLIST_START(zx81_p_format)
	CASSETTE_FORMAT(zx81_p_image_format)
CASSETTE_FORMATLIST_END


CASSETTE_FORMATLIST_START(zx81_cassette_formats)
	CASSETTE_FORMAT(zx81_p_image_format)
	CASSETTE_FORMAT(tzx_cassette_format)
CASSETTE_FORMATLIST_END

/* ZX-80 functions */

static cassette_image::error zx80_o_identify(cassette_image *cassette, cassette_image::Options *opts)
{
	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = ZX81_WAV_FREQUENCY;
	return cassette_image::error::SUCCESS;
}


static cassette_image::error zx80_o_load(cassette_image *cassette)
{
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> file_contents(file_size);
	cassette->image_read(&file_contents[0], 0, file_size);
	std::vector<int16_t> samples;

	/* pilot */
	zx81_emit_level(samples, ZX81_PILOT_LENGTH, WAVEENTRY_ZERO);

	/* data */
	for (uint64_t i = 0; i < file_size; i++)
		zx81_output_byte(samples, file_contents[i]);

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / ZX81_WAV_FREQUENCY, samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


static const cassette_image::Format zx80_o_image_format =
{
	"o,80",
	zx80_o_identify,
	zx80_o_load,
	nullptr
};


CASSETTE_FORMATLIST_START(zx80_o_format)
	CASSETTE_FORMAT(zx80_o_image_format)
CASSETTE_FORMATLIST_END
