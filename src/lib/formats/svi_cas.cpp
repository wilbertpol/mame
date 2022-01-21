// license:BSD-3-Clause
// copyright-holders:Sean Young
#include "svi_cas.h"

#include <cstring>

#define CAS_PERIOD_0        (37)
#define CAS_PERIOD_1        (18)
#define CAS_HEADER_PERIODS (1600)
#define CAS_EMPTY_SAMPLES (24220)
#define CAS_INIT_SAMPLES    (200)

static const uint8_t CasHeader[17] =
{
	0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
	0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x7f
};

#define SMPLO   -32768
#define SMPHI   32767

/*******************************************************************
   Generate samples for the tape image
********************************************************************/
static void svi_cas_fill_wave(std::vector<int16_t> &samples, std::vector<uint8_t> &bytes)
{
	int cas_pos, samples_pos, n, i;

	cas_pos = 17;
	samples_pos = 0;

	/* write CAS_INIT_SAMPLES of silence */
	n = CAS_INIT_SAMPLES; while (n--) samples.push_back(0);

	while (cas_pos < bytes.size())
	{
		/* write CAS_HEADER_PERIODS of header */
		for (int i = 0; i < CAS_HEADER_PERIODS; i++)
		{
			/* write a "0" */
			n = !(i % 4) ? 21 : 18;
			while (n--) samples.push_back(SMPHI);
			n = 19; while (n--) samples.push_back(SMPLO);
			/* write a "1" */
			n = 9; while (n--) samples.push_back(SMPHI);
			n = 9; while (n--) samples.push_back(SMPLO);
		}

		/* write "0x7f" */
		/* write a "0" */
		n = 21; while (n--) samples.push_back(SMPHI);
		n = 19; while (n--) samples.push_back(SMPLO);

		for (i=0;i<7;i++)
		{
			/* write a "1" */
			n = 9; while (n--) samples.push_back(SMPHI);
			n = 9; while (n--) samples.push_back(SMPLO);
		}

		while (cas_pos < bytes.size())
		{
			n = 21; while (n--) samples.push_back(SMPHI);
			n = 19; while (n--) samples.push_back(SMPLO);

			for (i = 0; i < 8; i++)
			{
				int bit = (bytes[cas_pos] & (0x80 >> i));

				/* write this one bit */
				if (bit)
				{
					/* write a "1" */
					n = 9; while (n--) samples.push_back(SMPHI);
					n = 9; while (n--) samples.push_back(SMPLO);
				}
				else
				{
					/* write a "0" */
					n = 18; while (n--) samples.push_back(SMPHI);
					n = 19; while (n--) samples.push_back(SMPLO);
				}
			}

			cas_pos++;

			/* check if we've hit a new header (or end of block) */
			if ( (cas_pos + 17) < bytes.size())
			{
				if (!memcmp (&bytes[cas_pos], CasHeader, 17) )
				{
					cas_pos += 17;

					/* end of block marker */
					n = CAS_EMPTY_SAMPLES; while (n--) samples.push_back(SMPHI);

					break; /* falls back to loop above; plays header again */
				}
			}
		}
	}

	/* end of block marker */
	n = CAS_EMPTY_SAMPLES; while (n--) samples.push_back(SMPHI);
}


static cassette_image::error svi_cas_identify(cassette_image *cassette, cassette_image::Options *opts)
{
	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = 44100;
	return cassette_image::error::SUCCESS;
}


static cassette_image::error svi_cas_load(cassette_image *cassette)
{
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> bytes(file_size);
	cassette->image_read(&bytes[0], 0, file_size);
	std::vector<int16_t> samples;

	svi_cas_fill_wave(samples, bytes);

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / 44100, samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


static const cassette_image::Format svi_cas_format =
{
	"cas",
	svi_cas_identify,
	svi_cas_load,
	nullptr
};


CASSETTE_FORMATLIST_START(svi_cassette_formats)
	CASSETTE_FORMAT(svi_cas_format)
CASSETTE_FORMATLIST_END
