// license:BSD-3-Clause
// copyright-holders:Sandro Ronco
/********************************************************************

    Support for PMD 85 cassette images

    Supported formats:
    - pmd: raw image
    - ptp: PMD 85 tape package

********************************************************************/

#include "pmd_cas.h"

#define WAVEENTRY_LOW  -32768
#define WAVEENTRY_HIGH  32767

#define PMD85_WAV_FREQUENCY 7200
#define PMD85_TIMER_FREQUENCY   1200
#define PMD85_BIT_LENGTH    (PMD85_WAV_FREQUENCY/PMD85_TIMER_FREQUENCY)
#define PMD85_PILOT_BITS    (PMD85_TIMER_FREQUENCY*3)
#define PMD85_PAUSE_BITS    (PMD85_TIMER_FREQUENCY/2)
#define PMD85_HEADER_BYTES  63
#define PMD85_BITS_PER_BYTE 11


static void pmd85_emit_level(std::vector<int16_t> &samples, int count, int level)
{
	for (int i = 0; i < count; i++)
		samples.push_back(level);
}


static void pmd85_output_bit(std::vector<int16_t> &samples, uint8_t bit)
{
	if (bit)
	{
		pmd85_emit_level(samples, PMD85_BIT_LENGTH/2, WAVEENTRY_LOW);
		pmd85_emit_level(samples, PMD85_BIT_LENGTH/2, WAVEENTRY_HIGH);
	}
	else
	{
		pmd85_emit_level(samples, PMD85_BIT_LENGTH/2, WAVEENTRY_HIGH);
		pmd85_emit_level(samples, PMD85_BIT_LENGTH/2, WAVEENTRY_LOW);
	}
}


static void pmd85_output_byte(std::vector<int16_t> &samples, uint8_t byte)
{
	// start
	pmd85_output_bit(samples, 0);

	// data
	for (int i = 0; i < 8; i++)
		pmd85_output_bit(samples, (byte>>i) & 0x01);

	// stop
	pmd85_output_bit(samples, 1);
	pmd85_output_bit(samples, 1);
}


static bool pmd85_is_header_block(std::vector<uint8_t> &bytes, int data_pos)
{
	for (int i = data_pos; i < data_pos + 0x10; i++)
	{
		if (bytes.size() < data_pos + 0x30 || bytes[i] != 0xff || bytes[i + 0x10] != 0x00 || bytes[i + 0x20] != 0x55)
			return false;
	}

	return true;
}


static void pmd85_handle_cassette(std::vector<int16_t> &samples, std::vector<uint8_t> &bytes)
{
	if (pmd85_is_header_block(bytes, 0))
	{
		// PMD file

		// pilot
		for (int i = 0; i < PMD85_PILOT_BITS; i++)
			pmd85_output_bit(samples, 1);

		// header
		for (int i = 0; i < PMD85_HEADER_BYTES; i++)
			pmd85_output_byte(samples, bytes[i]);

		// pause
		for (int i = 0; i < PMD85_PAUSE_BITS; i++)
			pmd85_output_bit(samples, 1);

		// data
		for (int i = PMD85_HEADER_BYTES; i < bytes.size(); i++)
			pmd85_output_byte(samples, bytes[i]);
	}
	else
	{
		// PTP file

		// pilot
		for (int i = 0; i < PMD85_PILOT_BITS; i++)
			pmd85_output_bit(samples, 1);

		int data_pos = 0;
		while (data_pos < bytes.size())
		{
			if (data_pos + 2 >= bytes.size())
				break;
			uint16_t block_size = (bytes[data_pos + 1] << 8) | bytes[data_pos];
			int pause_len = PMD85_PAUSE_BITS;

			data_pos += 2;

			if (pmd85_is_header_block(bytes, data_pos))
			{
				pause_len *= 2;
			}

			for (int i = 0; i < pause_len; i++)
				pmd85_output_bit(samples, 1);

			for (int i = 0; i < block_size && data_pos + i < bytes.size(); i++)
				pmd85_output_byte(samples, bytes[data_pos + i]);

			data_pos += block_size;
		}
	}
}


static cassette_image::error pmd85_cassette_identify(cassette_image *cassette, cassette_image::Options *opts)
{
	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = PMD85_WAV_FREQUENCY;
	return cassette_image::error::SUCCESS;
}


static cassette_image::error pmd85_cassette_load(cassette_image *cassette)
{
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> bytes(file_size);
	cassette->image_read(&bytes[0], 0, file_size);
	std::vector<int16_t> samples;

	pmd85_handle_cassette(samples, bytes);

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / PMD85_WAV_FREQUENCY, samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


static const cassette_image::Format pmd85_cassette_image_format =
{
	"pmd,tap,ptp",
	pmd85_cassette_identify,
	pmd85_cassette_load,
	nullptr
};


CASSETTE_FORMATLIST_START(pmd85_cassette_formats)
	CASSETTE_FORMAT(pmd85_cassette_image_format)
CASSETTE_FORMATLIST_END
