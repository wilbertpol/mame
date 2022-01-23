// license:BSD-3-Clause
// copyright-holders:JJ Stacino
/********************************************************************

Support for Micronique machine .K7 and *.FOR cassette images

Note that the usual type for hector cassette is *.K7,
     the *.FOR type is only for programming screen in forth format.

You can find some *.K7 file on serveral server in France
(Micronique is a French factory) Enjoy !

 jj.stacino@aliceadsl.fr

Updated 3/1/10 : use real value for timing.
********************************************************************/

#include "hect_tap.h"


#define SMPLO   -32768
#define SILENCE 0
#define SMPHI   32767


enum
{
	HEADER_CYCLES = 77, /* Valeur Th?orique 66 = 44100 * 1.5 / 1000  // mesur? sur jeu Formule1 = 1,75ms*/
	ZERO_CYCLES =   27, /* Valeur Th?orique 17 = 44100 * 0.4 / 1000  // mesur? sur jeu Formule1 = 0,61ms*/
	UN_CYCLES =     50  /* Valeur Th?orique 40 = 44100 * 0.9 / 1000  // mesur? sur jeu Formule1 = 1,13ms*/
};

/* Here I prefer use the value that I read on a real tape, and not the theorical value; note that these
   value work best on my HRX...    Yo_fr   (jj.stacino@aliceadsl.fr)  */

/*******************************************************************
   Generate one high-low cycle of sample data
********************************************************************/
static void hector_tap_cycle(std::vector<int16_t> &samples, int high, int low)
{
	int i = 0;

	while (i < high)
	{
		samples.push_back(SMPHI);
		i++;
	}

	while (i < high + low)
	{
		samples.push_back(SMPLO);
		i++;
	}
}


static void hector_tap_byte(std::vector<int16_t> &samples, uint8_t data)
{
	// Writing an entire byte
	for (int i = 0; i < 8; i++ )
	{
		if (data & 0x01)
			hector_tap_cycle(samples, UN_CYCLES / 2, UN_CYCLES / 2);
		else
			hector_tap_cycle(samples, ZERO_CYCLES / 2, ZERO_CYCLES / 2);

		data >>= 1;
	}
}


static void hector_tap_synchro(std::vector<int16_t> &samples, int nb_synchro)
{
	for (int i = 0; i < nb_synchro ; i++ )
			hector_tap_cycle(samples, HEADER_CYCLES / 2, HEADER_CYCLES / 2);
}


static void hector_handle_tap(std::vector<int16_t> &samples, std::vector<uint8_t> &bytes)
{
	int data_pos = 0;
	int previous_block = 0;

	// First 768 cycle of synchro 
	hector_tap_synchro(samples, 768 - 4);

	// on the entire file
	while (data_pos < bytes.size())
	{
		if (previous_block == 0xFE)
			// Starting a block with 150 cycle of synchro to let time to Hector to do the job !
			hector_tap_synchro(samples, 150);
		else
			// Starting a block with 4 cycle of synchro
			hector_tap_synchro(samples, 4);

		if (data_pos > 1)
			previous_block = bytes[data_pos - 1];

		// Handle block length on tape data
		uint16_t block_size = bytes[data_pos];
		if (block_size == 0)
			block_size = 256;

		hector_tap_byte(samples, bytes[data_pos]);
		data_pos++;

		// Data samples
		for ( ; block_size ; data_pos++, block_size--)
		{
			if (data_pos < bytes.size())
				hector_tap_byte(samples, bytes[data_pos]);
		}
	}
	// Finish by a zero
	hector_tap_byte(samples, 0);
}

/*******************************************************************
////  FORTH DATA CASSETTE
*******************************************************************/


static void hector_handle_forth_tap(std::vector<int16_t> &samples, std::vector<uint8_t> &bytes)
{
	int data_pos = 0;

	// on the entire file
	while (data_pos < bytes.size())
	{
		// Starting a block with 768 cycle of synchro
		hector_tap_synchro(samples, 768);

		// Handle block length on tape data
		uint16_t block_size = 822 ; // Fixed size for the forth

		// Data samples
		for ( ; block_size ; data_pos++, block_size--)
		{
			if (data_pos < bytes.size())
				hector_tap_byte(samples, bytes[data_pos]);

		}
	}

	// Finish by a zero
	hector_tap_byte(samples, 0);
}


/*******************************************************************
/////  END FORTH DATA CASSETTE
*******************************************************************/


static cassette_image::error hector_k7_identify(cassette_image *cassette, cassette_image::Options *opts)
{
	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = 44100;
	return cassette_image::error::SUCCESS;
}


static cassette_image::error hector_k7_load(cassette_image *cassette)
{
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> bytes(file_size);
	cassette->image_read(&bytes[0], 0, file_size);
	std::vector<int16_t> samples;

	hector_handle_tap(samples, bytes);

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / 44100, samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


static cassette_image::error hector_k7forth_identify(cassette_image *cassette, cassette_image::Options *opts)
{
	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = 44100;

	if ((cassette->image_size() % 822) != 0)
		return cassette_image::error::INVALID_IMAGE;

	return cassette_image::error::SUCCESS;
}


static cassette_image::error hector_k7forth_load(cassette_image *cassette)
{
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> bytes(file_size);
	cassette->image_read(&bytes[0], 0, file_size);
	std::vector<int16_t> samples;

	// Out if len of file not modulo 822 octets
	if ((file_size % 822) != 0)
		return cassette_image::error::INVALID_IMAGE;

	hector_handle_forth_tap(samples, bytes);

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / 44100, samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


static const cassette_image::Format hector_k7_format =
{
	"k7,cin",
	hector_k7_identify,
	hector_k7_load,
	nullptr
};

static const cassette_image::Format hector_k7Forth_format =
{
	"for",
	hector_k7forth_identify,
	hector_k7forth_load,
	nullptr
};


CASSETTE_FORMATLIST_START(hector_cassette_formats)
	CASSETTE_FORMAT(hector_k7_format)
	CASSETTE_FORMAT(hector_k7Forth_format)
CASSETTE_FORMATLIST_END
