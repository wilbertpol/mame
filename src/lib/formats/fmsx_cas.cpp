// license:BSD-3-Clause
// copyright-holders:Sean Young

#include "fmsx_cas.h"

#include <cstring>


#define CAS_PERIOD        (16)
#define CAS_HEADER_PERIODS (4000)
#define CAS_EMPTY_PERIODS (1000)

static const uint8_t CAS_HEADER[8] = { 0x1F,0xA6,0xDE,0xBA,0xCC,0x13,0x7D,0x74 };


static void fmsx_cas_fill_wave(std::vector<int16_t> &samples, std::vector<uint8_t> &bytes)
{
	int state = 1;
	int cas_pos = 0;

	while (cas_pos < bytes.size())
	{
		// Check if we need to output a header
		if (cas_pos + 8 < bytes.size())
		{
			if (!memcmp(&bytes[cas_pos], CAS_HEADER, 8 ))
			{
				// Write CAS_EMPTY_PERIODS of silence
				int n = CAS_EMPTY_PERIODS * CAS_PERIOD;
				while (n--)
					samples.push_back(0);

				// Write CAS_HEADER_PERIODS of header (high frequency)
				for (int i = 0; i <CAS_HEADER_PERIODS * 4; i++)
				{
					for (int n=0; n <CAS_PERIOD / 4;n++)
						samples.push_back(state ? 32767 : -32767);

					state = !state;
				}

				cas_pos += 8;
			}
		}

		for (int i=0; i <= 11; i++)
		{
			int bit = 0;
			if (i == 0) bit = 0;
			else if (i < 9) bit = (bytes[cas_pos] & (1 << (i - 1) ) );
			else bit = 1;

			// write this one bit
			for (int n = 0; n < (bit ? 4 : 2); n++)
			{
				int size = (bit ? CAS_PERIOD / 4 : CAS_PERIOD / 2);
				for (int p = 0; p < size;p++)
				{
					samples.push_back((state ? 32767 : -32767));
				}
				state = !state;
			}
		}
		cas_pos++;
	}
}


static cassette_image::error fmsx_cas_identify(cassette_image *cassette, cassette_image::Options *opts)
{
	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = 22050;
	return cassette_image::error::SUCCESS;
}


static cassette_image::error fmsx_cas_load(cassette_image *cassette)
{
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> bytes(file_size);
	cassette->image_read(&bytes[0], 0, file_size);
	std::vector<int16_t> samples;

	fmsx_cas_fill_wave(samples, bytes);

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / 22050, samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


static const cassette_image::Format fmsx_cas_format =
{
	"tap,cas",
	fmsx_cas_identify,
	fmsx_cas_load,
	nullptr
};


CASSETTE_FORMATLIST_START(fmsx_cassette_formats)
	CASSETTE_FORMAT(fmsx_cas_format)
CASSETTE_FORMATLIST_END
