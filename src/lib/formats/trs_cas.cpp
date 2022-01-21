// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/********************************************************************

Support for TRS80 .cas cassette images
Types handled:
- Model 1 Level I: 250 baud
- Model 1 Level II: 500 baud
- Model 3/4: 1500 baud.

Level I and II tape format is completely identical, apart from the
baud rate. The contents are specific to each system though.
The Model 3 and 4 can load either Level II tapes (by answering L
to the Cass? prompt), or the fast format by hitting enter at Cass?

********************************************************************/

#include "formats/trs_cas.h"


#define SILENCE 0
#define SMPLO   -32768
#define SMPHI   32767


/*******************************************************************
   Generate one high-low cycle of sample data
********************************************************************/
static inline void trs80m1_cas_cycle(std::vector<int16_t> &samples, bool bit)
{
	for (uint8_t i = 0; i < 32; i++)
		samples.push_back(SILENCE);
	for (uint8_t i = 0; i < 6; i++)
		samples.push_back(bit ? SMPHI : SILENCE);
	for (uint8_t i = 0; i < 6; i++)
		samples.push_back(bit ? SMPLO : SILENCE);
}


static cassette_image::error trs80m1_handle_cas(std::vector<int16_t> &samples, std::vector<uint8_t> &casdata)
{
	int data_pos = 0;
	bool sw = false;

	// Make sure this is a trs80 tape
	// Should have some zero bytes then one 0xA5
	while ((data_pos < casdata.size()) && (casdata[data_pos] == 0x00))
		data_pos++;
	if (casdata[data_pos] != 0xA5)
		return cassette_image::error::INVALID_IMAGE;

	data_pos = 0;
	while (data_pos < casdata.size())
	{
		uint8_t data = casdata[data_pos];

		for (uint8_t i = 0; i < 8; i++ )
		{
			/* Signal code */
			trs80m1_cas_cycle(samples, true);

			/* Bit code */
			trs80m1_cas_cycle(samples, data >> 7);

			data <<= 1;
		}

		if (!sw && (casdata[data_pos] == 0xA5))
		{
			sw = true;
			// Need 1ms silence here while rom is busy
			trs80m1_cas_cycle(samples, false);
		}

		data_pos++;
	}

	// Specification requires a short silence to indicate EOF
	trs80m1_cas_cycle(samples, false);
	trs80m1_cas_cycle(samples, false);
	return cassette_image::error::SUCCESS;
}


static inline void trs80m3_cas_cycle(std::vector<int16_t> &samples, bool bit)
{
	uint8_t counts = bit ? 8 : 16;

	for (uint8_t i = 0; i < counts; i++)
		samples.push_back(SMPHI);
	for (uint8_t i = 0; i < counts; i++)
		samples.push_back(SMPLO);
}


static cassette_image::error trs80m3_handle_cas(std::vector<int16_t> &samples, std::vector<uint8_t> &casdata)
{
	int data_pos = 0;
	uint8_t sw = 0, bitout = 0, byteout = 0;

	// Make sure this is a trs80m3 tape
	// Should have ~256 0x55 then one 0x7f
	// It's possible that 0x57 could be encountered instead,
	//   but no working tapes with it have been found.
	// Other bit-shifted variants might exist too.
	while ((data_pos < casdata.size()) && (casdata[data_pos] == 0x55))
		data_pos++;
	if (casdata[data_pos] != 0x7f)
		return cassette_image::error::INVALID_IMAGE;

	data_pos = 0;
	while (data_pos < casdata.size())
	{
		uint8_t data = casdata[data_pos];

		for (uint8_t i = 0; i < 8; i++)
		{
			trs80m3_cas_cycle(samples, data >> 7);

			// This paragraph unscrambles and prints the SYSTEM name.
			// If the first character is U, type SYSTEM, then the next 6 characters; otherwise use CLOAD.
			if (sw == 1)
			{
				if (bitout)
				{
					byteout = (byteout << 1) | (data >> 7);
					if (bitout == 8)
					{
						if (byteout == 0)
							sw = 2;
						printf("%c",byteout);
						byteout = 0;
						bitout = 0;
					}
					else
						bitout++;
				}
				else bitout++;
			}

			data <<= 1;
		}

		if (!sw && (casdata[data_pos] != 0x55))
		{
			sw = 1;
			// This 1ms of silence isn't absolutely necessary, but the system
			// writes it, so we may as well emulate it.
			trs80m1_cas_cycle(samples, false);
		}

		data_pos++;
	}

	// Specification requires a short silence to indicate EOF
	trs80m1_cas_cycle(samples, false);
	trs80m1_cas_cycle(samples, false);
	return cassette_image::error::SUCCESS;
}


static cassette_image::error trs80l1_cas_identify(cassette_image *cassette, cassette_image::Options *opts)
{
	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = 22050;
	return cassette_image::error::SUCCESS;
}


static cassette_image::error trs80l1_cas_load(cassette_image *cassette)
{
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> bytes(file_size);
	cassette->image_read(&bytes[0], 0, file_size);
	std::vector<int16_t> samples;

	if (!file_size)
		return cassette_image::error::INVALID_IMAGE;

	cassette_image::error err;
	if (bytes[0] == 0x55)
		err = trs80m3_handle_cas(samples, bytes);
	else
		err = trs80m1_handle_cas(samples, bytes);
	if (err != cassette_image::error::SUCCESS)
		return err;

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / 22050, samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


static const cassette_image::Format trs80l1_cas_format =
{
	"cas",
	trs80l1_cas_identify,
	trs80l1_cas_load,
	nullptr
};


static cassette_image::error trs80l2_cas_identify(cassette_image *cassette, cassette_image::Options *opts)
{
	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = 44100;
	return cassette_image::error::SUCCESS;
}


static cassette_image::error trs80l2_cas_load(cassette_image *cassette)
{
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> bytes(file_size);
	cassette->image_read(&bytes[0], 0, file_size);
	std::vector<int16_t> samples;

	if (!file_size)
		return cassette_image::error::INVALID_IMAGE;

	cassette_image::error err;
	if (bytes[0] == 0x55)
		err = trs80m3_handle_cas(samples, bytes);
	else
		err = trs80m1_handle_cas(samples, bytes);
	if (err != cassette_image::error::SUCCESS)
		return err;

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / 44100, samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


static const cassette_image::Format trs80l2_cas_format =
{
	"cas",
	trs80l2_cas_identify,
	trs80l2_cas_load,
	nullptr
};


CASSETTE_FORMATLIST_START(trs80l1_cassette_formats)
	CASSETTE_FORMAT(trs80l1_cas_format)
CASSETTE_FORMATLIST_END


CASSETTE_FORMATLIST_START(trs80l2_cassette_formats)
	CASSETTE_FORMAT(trs80l2_cas_format)
CASSETTE_FORMATLIST_END
