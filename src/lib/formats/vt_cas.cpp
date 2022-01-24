// license:GPL-2.0+
// copyright-holders:Juergen Buchmueller
#include "formats/vt_cas.h"

/*********************************************************************
    vtech 1/2 agnostic
*********************************************************************/

#define SILENCE 8000


static void generic_fill_wave(std::vector<int16_t> &samples, std::vector<uint8_t> &bytes, int bitsamples, int bytesamples, int lo, void (*fill_wave_byte)(std::vector<int16_t> &samples, uint8_t byte))
{
	int nullbyte = 0;

	for (int i = 0; i < SILENCE; i++)
		samples.push_back(0);

	for (int i = 0; i < bytes.size(); i++)
	{
		fill_wave_byte(samples, bytes[i]);
		if (!nullbyte && bytes[i] == 0)
		{
			for (int i = 0; i < 2*bitsamples; i++)
				samples.push_back(lo);
			nullbyte = 1;
		}
	}

	// silence at the end
	for (int i = 0; i < 600 * bitsamples; i++)
		samples.push_back(0);
}

/*********************************************************************
    vtech 1
*********************************************************************/

#define V1_LO   -32768
#define V1_HI   +32767

#define V1_BITSAMPLES   6
#define V1_BYTESAMPLES  8*V1_BITSAMPLES


static void vtech1_fill_wave_byte(std::vector<int16_t> &samples, uint8_t byte)
{
	for (int i = 7; i >= 0; i--)
	{
		samples.push_back(V1_HI);  // initial cycle
		samples.push_back(V1_LO);
		if ((byte >> i) & 1)
		{
			samples.push_back(V1_HI); // two more cycles
			samples.push_back(V1_LO);
			samples.push_back(V1_HI);
			samples.push_back(V1_LO);
		}
		else
		{
			samples.push_back(V1_HI); // one slow cycle
			samples.push_back(V1_HI);
			samples.push_back(V1_LO);
			samples.push_back(V1_LO);
		}
	}
}


static cassette_image::error vtech1_cas_identify(cassette_image *cassette, cassette_image::Options *opts)
{
	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = 600 * V1_BITSAMPLES;
	return cassette_image::error::SUCCESS;
}


static cassette_image::error vtech1_cas_load(cassette_image *cassette)
{
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> bytes(file_size);
	cassette->image_read(&bytes[0], 0, file_size);
	std::vector<int16_t> samples;

	generic_fill_wave(samples, bytes, V1_BITSAMPLES, V1_BYTESAMPLES, V1_LO, vtech1_fill_wave_byte);

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / (600 * V1_BITSAMPLES), samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


static const cassette_image::Format vtech1_cas_format =
{
	"cas",
	vtech1_cas_identify,
	vtech1_cas_load,
	nullptr
};


CASSETTE_FORMATLIST_START(vtech1_cassette_formats)
	CASSETTE_FORMAT(vtech1_cas_format)
CASSETTE_FORMATLIST_END

/*********************************************************************
    vtech 2
*********************************************************************/

#define VT2_LO  -20000
#define VT2_HI  +20000

#define VT2_BITSAMPLES  18
#define VT2_BYTESAMPLES 8*VT2_BITSAMPLES

static const int16_t vtech2_bit0[VT2_BITSAMPLES] =
{
	/* short cycle, long cycles */
	VT2_HI,VT2_HI,VT2_HI,VT2_LO,VT2_LO,VT2_LO,
	VT2_HI,VT2_HI,VT2_HI,VT2_HI,VT2_HI,VT2_HI,
	VT2_LO,VT2_LO,VT2_LO,VT2_LO,VT2_LO,VT2_LO
};

static const int16_t vtech2_bit1[VT2_BITSAMPLES] =
{
	/* three short cycle */
	VT2_HI,VT2_HI,VT2_HI,VT2_LO,VT2_LO,VT2_LO,
	VT2_HI,VT2_HI,VT2_HI,VT2_LO,VT2_LO,VT2_LO,
	VT2_HI,VT2_HI,VT2_HI,VT2_LO,VT2_LO,VT2_LO
};


static void vtech2_fill_wave_bit(std::vector<int16_t> &samples, uint8_t bit)
{
	if (bit)
	{
		for (int i = 0; i < VT2_BITSAMPLES; i++)
			samples.push_back(vtech2_bit1[i]);
	}
	else
	{
		for (int i = 0; i < VT2_BITSAMPLES; i++)
			samples.push_back(vtech2_bit0[i]);
	}
}


static void vtech2_fill_wave_byte(std::vector<int16_t> &samples, uint8_t byte)
{
	for (int i = 0 ; i < 8; i++)
		vtech2_fill_wave_bit(samples, (byte >> (7-i)) & 1);
}


static cassette_image::error vtech2_cas_identify(cassette_image *cassette, cassette_image::Options *opts)
{
	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = 600 * VT2_BITSAMPLES;
	return cassette_image::error::SUCCESS;
}


static cassette_image::error vtech2_cas_load(cassette_image *cassette)
{
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> bytes(file_size);
	cassette->image_read(&bytes[0], 0, file_size);
	std::vector<int16_t> samples;

	generic_fill_wave(samples, bytes, VT2_BITSAMPLES, VT2_BYTESAMPLES, VT2_LO, vtech2_fill_wave_byte);

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / (600 * VT2_BITSAMPLES), samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


static const cassette_image::Format vtech2_cas_format =
{
	"cas",
	vtech2_cas_identify,
	vtech2_cas_load,
	nullptr
};


CASSETTE_FORMATLIST_START(vtech2_cassette_formats)
	CASSETTE_FORMAT(vtech2_cas_format)
CASSETTE_FORMATLIST_END
