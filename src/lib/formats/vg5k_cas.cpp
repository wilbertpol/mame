// license:BSD-3-Clause
// copyright-holders:Sandro Ronco
/********************************************************************

    Support for VG-5000 .k7 cassette images

********************************************************************/

#include "vg5k_cas.h"


#define SMPLO   -32768
#define SILENCE 0
#define SMPHI   32767


/*******************************************************************
   Generate one high-low cycle of sample data
********************************************************************/
static inline void vg5k_cas_cycle(std::vector<int16_t> &samples, int len)
{
	for (int i = 0; i < len; i++)
		samples.push_back(SMPHI);

	for (int i = 0; i < len; i++)
		samples.push_back(SMPLO);
}


/*******************************************************************
   Generate n samples of silence
********************************************************************/
static inline void vg5k_cas_silence(std::vector<int16_t> &samples, int len)
{
	for (int i = 0; i < len; i++)
		samples.push_back(SILENCE);
}


/*******************************************************************
   Generate the end-byte samples
********************************************************************/
static inline void vg5k_cas_eob(std::vector<int16_t> &samples)
{
	for (int i = 0; i < 4; i++)
		vg5k_cas_cycle(samples, 5);

	vg5k_cas_cycle(samples, 10);
}


static inline void vg5k_cas_byte(std::vector<int16_t> &samples, uint8_t data)
{
	/* Writing an entire byte */
	for (int i = 0; i < 8; i++)
	{
		if (data & 0x01)
		{
			vg5k_cas_cycle(samples, 5);
			vg5k_cas_cycle(samples, 5);
		}
		else
		{
			vg5k_cas_cycle(samples, 10);
		}

		data >>= 1;
	}
}


/*******************************************************************
   Generate n sample of synchro
********************************************************************/
static inline void vg5k_k7_synchro(std::vector<int16_t> &samples, int len)
{
	for (int i = 0; i < len ; i++ )
			vg5k_cas_cycle(samples, 5);

	vg5k_cas_eob(samples);
}


static cassette_image::error vg5k_handle_tap(std::vector<int16_t> &samples, std::vector<uint8_t> &casdata)
{
	int data_pos = 0;

	/* on the entire file*/
	while (data_pos < casdata.size())
	{
		uint16_t  block_size = 0;

		/* Identify type of block */
		if (casdata[data_pos] == 0xd3)
		{
			/* head block have fixed size of 32 byte */
			block_size = 0x20;

			/* 1 sec of silence before the head block */
			vg5k_cas_silence(samples, 44100);

			/* head block starts with 30000 samples of synchro */
			vg5k_k7_synchro(samples, 30000);
		}
		else if (casdata[data_pos] == 0xd6)
		{
			/* data block size is defined in head block */
			block_size = (casdata[data_pos - 4] | casdata[data_pos - 3] << 8) + 20;

			/* 10000 samples of silence before the data block */
			vg5k_cas_silence(samples, 10000);

			/* data block starts with 7200 samples of synchro */
			vg5k_k7_synchro(samples, 7200);
		}
		else
		{
			/* tries to handle files that do not respect the size declared in the head block */
			while (data_pos < casdata.size() && casdata[data_pos] != 0xd3 && casdata[data_pos] != 0xd6)
				data_pos++;
		}

		/* Data samples */
		for (; block_size; data_pos++, block_size--)
		{
			/* Make sure there are enough bytes left */
			if (data_pos > casdata.size())
				return cassette_image::error::INVALID_IMAGE;

			vg5k_cas_byte(samples, casdata[data_pos]);

			/* generate the end-byte samples */
			vg5k_cas_eob(samples);
		}
	}

	/* Finish with 10000 samples of silence */
	vg5k_cas_silence(samples, 10000);

	return cassette_image::error::SUCCESS;
}


static cassette_image::error vg5k_k7_identify(cassette_image *cassette, cassette_image::Options *opts)
{
	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = 44100;
	return cassette_image::error::SUCCESS;
}


static cassette_image::error vg5k_k7_load(cassette_image *cassette)
{
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> bytes(file_size);
	cassette->image_read(&bytes[0], 0, file_size);
	std::vector<int16_t> samples;

	if (file_size < 4)
		return cassette_image::error::INVALID_IMAGE;

	if (bytes[0] != 0xd3  || bytes[1] != 0xd3 || bytes[2] != 0xd3)
		return cassette_image::error::INVALID_IMAGE;

	cassette_image::error err = vg5k_handle_tap(samples, bytes);
	if (err != cassette_image::error::SUCCESS)
		return err;

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / 44100, samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


static const cassette_image::Format vg5k_k7_format =
{
	"k7",
	vg5k_k7_identify,
	vg5k_k7_load,
	nullptr
};


CASSETTE_FORMATLIST_START(vg5k_cassette_formats)
	CASSETTE_FORMAT(vg5k_k7_format)
CASSETTE_FORMATLIST_END
