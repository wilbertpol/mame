// license:BSD-3-Clause
// copyright-holders:Krzysztof Strzecha
/* .LVT tape images */

#include "lviv_lvt.h"

#define WAVEENTRY_LOW  -32768
#define WAVEENTRY_HIGH  32767

#define LVIV_LVT_BIT_SAMPLES                60
#define LVIV_LVT_HEADER_PILOT_SAMPLES       5190*60
#define LVIV_LVT_HEADER_DATA_SAMPLES        16*11*60
#define LVIV_LVT_PAUSE_SAMPLES              69370
#define LVIV_LVT_BLOCK_PILOT_SAMPLES        1298*60

#define LVIV_LVT_HEADER_PILOT_LENGTH        5190
#define LVIV_LVT_BLOCK_PILOT_LENGTH         1298


static void lviv_emit_level(std::vector<int16_t> &samples, int count, int level)
{
	for (int i = 0; i < count; i++)
	{
		samples.push_back(level);
	}
}


static void lviv_output_bit(std::vector<int16_t> &samples, uint8_t b)
{
	if (b)
	{
		lviv_emit_level(samples, 15, WAVEENTRY_HIGH);
		lviv_emit_level(samples, 15, WAVEENTRY_LOW);
		lviv_emit_level(samples, 15, WAVEENTRY_HIGH);
		lviv_emit_level(samples, 15, WAVEENTRY_LOW);
	}
	else
	{
		lviv_emit_level(samples, 30, WAVEENTRY_HIGH);
		lviv_emit_level(samples, 30, WAVEENTRY_LOW);
	}
}


static void lviv_output_byte(std::vector<int16_t> &samples, uint8_t byte)
{
	lviv_output_bit(samples, 0);

	for (int i = 0; i < 8; i++)
		lviv_output_bit(samples, (byte>>i) & 0x01);

	lviv_output_bit(samples, 1);
	lviv_output_bit(samples, 1);
}


static void lviv_cassette_fill_wave(std::vector<int16_t> &samples, std::vector<uint8_t> &bytes)
{
	for (int i = 0; i < LVIV_LVT_HEADER_PILOT_LENGTH; i++)
		lviv_output_bit(samples, 1);

	for (int i = 0; i < 10; i++)
		lviv_output_byte(samples, bytes[0x09]);

	for (int i=0; i<6; i++)
		lviv_output_byte(samples, bytes[0x0a+i]);

	lviv_emit_level(samples, LVIV_LVT_PAUSE_SAMPLES, WAVEENTRY_HIGH);

	for (int i = 0; i < LVIV_LVT_BLOCK_PILOT_LENGTH; i++)
		lviv_output_bit(samples, 1);

	for (int i = 0x10; i < bytes.size(); i++)
		lviv_output_byte(samples, bytes[i]);
}


static cassette_image::error lviv_lvt_identify(cassette_image *cassette, cassette_image::Options *opts)
{
	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = 44100;
	return cassette_image::error::SUCCESS;
}


static cassette_image::error lviv_lvt_load(cassette_image *cassette)
{
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> bytes(file_size);
	cassette->image_read(&bytes[0], 0, file_size);
	std::vector<int16_t> samples;

	if (file_size < 0x10)
		return cassette_image::error::INVALID_IMAGE;

	lviv_cassette_fill_wave(samples, bytes);

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / 44100, samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


static const cassette_image::Format lviv_lvt_image_format =
{
	"lvt,lvr,lv0,lv1,lv2,lv3",
	lviv_lvt_identify,
	lviv_lvt_load,
	nullptr
};


CASSETTE_FORMATLIST_START(lviv_lvt_format)
	CASSETTE_FORMAT(lviv_lvt_image_format)
CASSETTE_FORMATLIST_END
