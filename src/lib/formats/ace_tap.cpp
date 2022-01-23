// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/********************************************************************

Support for Jupiter Ace .tap cassette images

For more information see:
- http://www.jupiter-ace.co.uk/faq_ace_tap_format.html
- http://www.jupiter-ace.co.uk/doc_AceTapeFormat.html

********************************************************************/

#include "ace_tap.h"


#define SMPLO   -32768
#define SILENCE 0
#define SMPHI   32767


/*******************************************************************
   Generate one high-low cycle of sample data
********************************************************************/
static void ace_tap_cycle(std::vector<int16_t> &samples, int high, int low)
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


static void ace_tap_silence(std::vector<int16_t> &samples, int count)
{
	for (int i = 0; i < count; i++)
		samples.push_back(SILENCE);
}


static void ace_tap_byte(std::vector<int16_t> &samples, uint8_t data)
{
	for (int i = 0; i < 8; i++)
	{
		if (data & 0x80)
			ace_tap_cycle(samples, 21, 22);
		else
			ace_tap_cycle(samples, 10, 11);

		data <<= 1;
	}
}


static cassette_image::error ace_handle_tap(std::vector<int16_t> &samples, std::vector<uint8_t> &bytes)
{
	// Make sure the file starts with a valid header
	if (bytes.size() < 0x1c)
		return cassette_image::error::INVALID_IMAGE;
	if (bytes[0] != 0x1a || bytes[1] != 0x00)
		return cassette_image::error::INVALID_IMAGE;

	int data_pos = 0;

	while (data_pos + 1 < bytes.size())
	{

		// Handle a block of tape data
		uint16_t block_size = bytes[data_pos] + (bytes[data_pos + 1] << 8);
		data_pos += 2;

		// Make sure there are enough bytes left
		if (data_pos > bytes.size())
			return cassette_image::error::INVALID_IMAGE;

		// 2 seconds silence
		ace_tap_silence(samples, 2 * 44100);

		// Add pilot tone samples: 4096 for header, 512 for data
		for (int i = (block_size == 0x001a) ? 4096 : 512; i; i--)
			ace_tap_cycle(samples, 27, 27);

		// Sync samples
		ace_tap_cycle(samples, 8, 11);

		// Output block type identification byte
		ace_tap_byte(samples, (block_size != 0x001a) ? 0xFF : 0x00);

		// Data samples
		for ( ; block_size && data_pos < bytes.size() ; data_pos++, block_size--)
			ace_tap_byte(samples, bytes[data_pos]);

		// End mark samples
		ace_tap_cycle(samples, 12, 57);

		// 3 seconds silence
		ace_tap_silence(samples, 3 * 44100);
	}
	return cassette_image::error::SUCCESS;
}


static cassette_image::error ace_tap_identify(cassette_image *cassette, cassette_image::Options *opts)
{
	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = 44100;
	return cassette_image::error::SUCCESS;
}


static cassette_image::error ace_tap_load(cassette_image *cassette)
{
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> bytes(file_size);
	cassette->image_read(&bytes[0], 0, file_size);
	std::vector<int16_t> samples;

	cassette_image::error err = ace_handle_tap(samples, bytes);
	if (err != cassette_image::error::SUCCESS)
		return err;

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / 44100, samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


static const cassette_image::Format ace_tap_format =
{
	"tap",
	ace_tap_identify,
	ace_tap_load,
	nullptr
};


CASSETTE_FORMATLIST_START(ace_cassette_formats)
	CASSETTE_FORMAT(ace_tap_format)
CASSETTE_FORMATLIST_END
