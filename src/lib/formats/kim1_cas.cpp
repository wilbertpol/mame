// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
#include "kim1_cas.h"

#include <cstring>

#define SMPLO   -32768
#define SMPHI   32767


static void kim1_output_signal(std::vector<int16_t> &samples, int high)
{
	if (high)
	{
		// high frequency (~3600Hz)
		for (int i = 0; i < 9; i++)
		{
			for (int j = 0; j < 6; j++)
				samples.push_back(SMPHI);

			for (int j = 0; j < 6; j++)
				samples.push_back(SMPLO);
		}
	}
	else
	{
		// low frequency (~2400Hz)
		for (int i = 0; i < 6; i++)
		{
			for (int j = 0; j < 9; j++)
				samples.push_back(SMPHI);

			for (int j = 0; j < 9; j++)
				samples.push_back(SMPLO);
		}
	}
}


static void kim1_output_byte(std::vector<int16_t> &samples, uint8_t byte)
{
	for (int i = 0; i < 8; i++ )
	{
		kim1_output_signal(samples, 1 );
		kim1_output_signal(samples, byte & 0x01 ? 0 : 1 );
		kim1_output_signal(samples, 0 );
		byte >>= 1;
	}
}


static void kim1_handle_kim(std::vector<int16_t> &samples, std::vector<uint8_t> &casdata)
{
	static const uint8_t encoding[16] = { 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46 };

	uint16_t address = casdata[4] | ( casdata[5] << 8 );
	uint16_t size = casdata[6] | ( casdata[7] << 8 );
	uint16_t file_id = casdata[8];
	int data_pos = 9;

	uint16_t checksum = casdata[4] + casdata[5];

	// First output a sync header: 100 x 16h
	for (int i = 0; i < 100; i++ )
		kim1_output_byte(samples, 0x16);

	// Output end of sync: 2Ah
	kim1_output_byte(samples, 0x2A);

	// Output ID
	kim1_output_byte(samples, encoding[file_id >> 4]);
	kim1_output_byte(samples, encoding[file_id & 0x0f]);

	// Output starting address
	kim1_output_byte(samples, encoding[(address & 0xff) >> 4]);
	kim1_output_byte(samples, encoding[address & 0x0f]);
	kim1_output_byte(samples, encoding[address >> 12]);
	kim1_output_byte(samples, encoding[(address >> 8) & 0x0f]);

	// Output the data
	while (data_pos < casdata.size() && data_pos + 9 < size)
	{
		uint8_t data = casdata[data_pos];

		kim1_output_byte(samples, encoding[data >> 4]);
		kim1_output_byte(samples, encoding[data & 0x0f]);
		checksum += data;
		data_pos++;
	}

	// Output end of data marker: 2Fh
	kim1_output_byte(samples, 0x2f);

	// Output checksum
	kim1_output_byte(samples, encoding[( checksum & 0xff) >> 4]);
	kim1_output_byte(samples, encoding[checksum & 0x0f]);
	kim1_output_byte(samples, encoding[checksum >> 12]);
	kim1_output_byte(samples, encoding[(checksum >> 8) & 0x0f]);

	// Output end of transmission marker: 2 x 04h
	kim1_output_byte(samples, 0x04);
	kim1_output_byte(samples, 0x04);
}


static cassette_image::error kim1_kim_identify(cassette_image *cassette, cassette_image::Options *opts)
{
	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = 44100;
	return cassette_image::error::SUCCESS;
}


static cassette_image::error kim1_kim_load(cassette_image *cassette)
{
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> bytes(file_size);
	cassette->image_read(&bytes[0], 0, file_size);
	std::vector<int16_t> samples;

	if (file_size < 9 || memcmp(&bytes[0], "KIM1", 4))
		return cassette_image::error::INVALID_IMAGE;

	kim1_handle_kim(samples, bytes);

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / 44100, samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


static const cassette_image::Format kim1_kim_format =
{
	"kim,kim1",
	kim1_kim_identify,
	kim1_kim_load,
	nullptr
};


CASSETTE_FORMATLIST_START(kim1_cassette_formats)
	CASSETTE_FORMAT(kim1_kim_format)
CASSETTE_FORMATLIST_END
