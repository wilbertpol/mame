// license:BSD-3-Clause
// copyright-holders:Barry Rodewald
/*
 *
 *   Sharp X1 TAP image format
 *
 *   "New" header format:
 *   0x00: Index - must be "TAPE" (4 bytes)
 *   0x04: Image title. (null-terminated string, 17 bytes)
 *   0x15: Reserved (5 bytes)
 *   0x1a: Write protect (bit 4, 1 byte)
 *   0x1b: Format (bit 0, 1 byte)
 *            if bit 0 is high, uses "speed limit sampling method"
 *   0x1c: Sample rate, per bit (in Hz, 4 bytes)
 *   0x20: Tape data size, in bits (4 bytes)
 *   0x24: Tape position (4 bytes, usually 0)
 *   0x28: Tape data (data size / 8)
 *
 *   "Old" header format:
 *   0x00: Sampling rate (4 bytes)
 *
 */
#include "x1_tap.h"
#include "imageutl.h"

#include <cstring>


#define WAVE_HIGH        0x5a9e
#define WAVE_LOW        -0x5a9e


static void x1_fill_wave(std::vector<int16_t> &samples, uint8_t data)
{
	// one byte = 8 samples
	for (int x = 0; x < 8; x++)
	{
		samples.push_back((data & (1 << (7-x))) ? WAVE_HIGH : WAVE_LOW);
	}
}


static cassette_image::error parse_header(cassette_image *cassette, int &sample_rate, bool &new_format)
{
	uint64_t file_size = cassette->image_size();

	if (file_size < 4)
		return cassette_image::error::INVALID_IMAGE;

	std::vector<uint8_t> bytes(0x20);
	cassette->image_read(&bytes[0], 0, 0x20);
	if (!memcmp(&bytes[0], "TAPE", 4))
	{
		if (file_size <= 0x28)
			return cassette_image::error::INVALID_IMAGE;
		sample_rate = bytes[0x1c] | (bytes[0x1d] << 8) | (bytes[0x1e] << 16) | (bytes[0x1f] << 24);
		new_format = true;
	}
	else
	{
		sample_rate = bytes[0x00] | (bytes[0x01] << 8) | (bytes[0x02] << 16) | (bytes[0x03] << 24);
		new_format = false;
	}
	return cassette_image::error::SUCCESS;
}


static cassette_image::error x1_cas_identify(cassette_image *cassette, cassette_image::Options *opts)
{
	int sample_rate;
	bool new_format;

	cassette_image::error err = parse_header(cassette, sample_rate, new_format);
	if (err != cassette_image::error::SUCCESS)
		return err;

	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = sample_rate;
	return cassette_image::error::SUCCESS;
}


static cassette_image::error x1_cas_load(cassette_image *cassette)
{
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> casdata(file_size);
	cassette->image_read(&casdata[0], 0, file_size);
	std::vector<int16_t> samples;

	int sample_rate;
	bool new_format;

	cassette_image::error err = parse_header(cassette, sample_rate, new_format);
	if (err != cassette_image::error::SUCCESS)
		return err;

	if (sample_rate != 8000)
	{
		LOG_FORMATS("TAP: Supprt for images that are not 8000Hz is preliminary\n");
	}

	for (uint64_t data_pos = new_format ? 0x28 : 0x04; data_pos < file_size; data_pos++)
	{
		x1_fill_wave(samples, casdata[data_pos]);
	}

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / sample_rate, samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


static const cassette_image::Format x1_cassette_format = {
	"tap",
	x1_cas_identify,
	x1_cas_load,
	nullptr
};


CASSETTE_FORMATLIST_START(x1_cassette_formats)
	CASSETTE_FORMAT(x1_cassette_format)
CASSETTE_FORMATLIST_END
