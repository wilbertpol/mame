// license:BSD-3-Clause
// copyright-holders:Krzysztof Strzecha
/* .PTP Microkey Primo tape images */

#include "primoptp.h"
#include "imageutl.h"


#define PRIMO_WAVEENTRY_LOW     -32768
#define PRIMO_WAVEENTRY_HIGH        32767
#define PRIMO_WAVEENTRY_ZERO        0

#define PRIMO_WAV_FREQUENCY     22050
#define PRIMO_BIT_1_PERIOD      (312*2*0.000001)
#define PRIMO_BIT_0_PERIOD              (3*PRIMO_BIT_1_PERIOD)

#define PRIMO_BIT_1_LENGTH      (PRIMO_BIT_1_PERIOD*PRIMO_WAV_FREQUENCY)
#define PRIMO_BIT_0_LENGTH      (PRIMO_BIT_0_PERIOD*PRIMO_WAV_FREQUENCY)
/*
#define PRIMO_BIT_1_LENGTH      6
#define PRIMO_BIT_0_LENGTH      16
*/
#define PRIMO_PAUSE_LENGTH      2000
#define PRIMO_FILE_PILOT_LENGTH     ((4*PRIMO_BIT_1_LENGTH + 4*PRIMO_BIT_0_LENGTH)*512)
#define PRIMO_BLOCK_PILOT_LENGTH    ((8*PRIMO_BIT_1_LENGTH)*96 + (5*PRIMO_BIT_1_LENGTH + 3*PRIMO_BIT_0_LENGTH)*3)


static void primo_emit_level(std::vector<int16_t> &samples, int count, int level)
{
	for (int i = 0; i < count; i++)
		samples.push_back(level);
}


static void primo_output_bit(std::vector<int16_t> &samples, uint8_t bit)
{
	if (bit)
	{
		primo_emit_level(samples, PRIMO_BIT_1_LENGTH/2, PRIMO_WAVEENTRY_HIGH);
		primo_emit_level(samples, PRIMO_BIT_1_LENGTH/2, PRIMO_WAVEENTRY_LOW);
	}
	else
	{
		primo_emit_level(samples, PRIMO_BIT_0_LENGTH/2, PRIMO_WAVEENTRY_HIGH);
		primo_emit_level(samples, PRIMO_BIT_0_LENGTH/2, PRIMO_WAVEENTRY_LOW);
	}
}


static void primo_output_byte(std::vector<int16_t> &samples, uint8_t byte)
{
	for (int i = 0; i < 8; i++)
		primo_output_bit(samples, (byte >> (7-i)) & 0x01);
}


static void primo_cassette_fill_wave(std::vector<int16_t> &samples, std::vector<uint8_t> &bytes)
{
	int pos = 0;

	while (pos < bytes.size())
	{
		printf("pos = %d\n", pos);
		LOG_FORMATS ("Beginning Primo file\n");
		// unknown byte + 2 bytes for size
		if (pos + 3 >= bytes.size())
		{
			pos += 3;
			break;
		}
		// pause
		primo_emit_level(samples, PRIMO_PAUSE_LENGTH, PRIMO_WAVEENTRY_ZERO);

		// file pilot
		for (int k = 0; k < 512; k++)
			primo_output_byte(samples, 0xaa);

		// file size with header
		// What is at bytes[pos]?
		uint16_t file_size = bytes[pos + 1] + (bytes[pos + 2] << 8);
		pos += 3;

		LOG_FORMATS ("File size: %u\n", file_size);

		// pos is now set on the first data byte of file
		// it means first byte (block type) of block header

		int file_end = pos + file_size - 3;
		while (pos < bytes.size() && pos < file_end)
		{
			// block pilot
			for (int k = 0; k < 96; k++)
				primo_output_byte(samples, 0xff);
			for (int k = 0; k < 3; k++)
				primo_output_byte(samples, 0xd3);

			// block size without header but including CRC byte
			if (pos + 3 >= bytes.size())
			{
				pos += 3;
				break;
			}
			// What is at bytes[pos]?
			uint16_t block_size = bytes[pos + 1] + (bytes[pos + 2] << 8);
			pos += 3;

			for (int k = 0; k < block_size && pos < bytes.size(); k++)
				primo_output_byte(samples, bytes[pos++]);
		}

		LOG_FORMATS ("Primo file finished\n");
	}

	LOG_FORMATS ("End of fill_wave/n");
}


static cassette_image::error primo_ptp_identify(cassette_image *cassette, cassette_image::Options *opts)
{
	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = PRIMO_WAV_FREQUENCY;
	return cassette_image::error::SUCCESS;
}


static cassette_image::error primo_ptp_load(cassette_image *cassette)
{
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> bytes(file_size);
	cassette->image_read(&bytes[0], 0, file_size);
	std::vector<int16_t> samples;

	primo_cassette_fill_wave(samples, bytes);

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / PRIMO_WAV_FREQUENCY, samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


static const cassette_image::Format primo_ptp_image_format =
{
	"ptp",
	primo_ptp_identify,
	primo_ptp_load,
	nullptr
};


CASSETTE_FORMATLIST_START(primo_ptp_format)
	CASSETTE_FORMAT(primo_ptp_image_format)
CASSETTE_FORMATLIST_END
