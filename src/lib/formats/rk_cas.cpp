// license:BSD-3-Clause
// copyright-holders:Miodrag Milanovic
/*

    Tape support for RK format

*/

#include "rk_cas.h"


#define RK_WAV_FREQUENCY    44000
#define WAVE_HIGH       32767
#define WAVE_LOW        -32768

#define RK_HEADER_LEN   256

#define RK_SIZE_20 20
#define RK_SIZE_22 22
#define RK_SIZE_60 60


static void rk_emit_level(std::vector<int16_t> &samples, int count, int level)
{

	for (int i=0; i<count; i++)
	{
		samples.push_back(level);
	}
}


static void rk_output_bit(std::vector<int16_t> &samples, uint8_t b, int bitsize)
{
	if (b)
	{
		rk_emit_level(samples, bitsize, WAVE_HIGH);
		rk_emit_level(samples, bitsize, WAVE_LOW);
	}
	else
	{
		rk_emit_level(samples, bitsize, WAVE_LOW);
		rk_emit_level(samples, bitsize, WAVE_HIGH);

	}
}


static void rk_output_byte(std::vector<int16_t> &samples, uint8_t byte, int bitsize)
{
	for (int i = 7; i >= 0; i--)
		rk_output_bit(samples, (byte>>i) & 0x01, bitsize);
}


static void rk20_cas_fill_wave(std::vector<int16_t> &samples, std::vector<uint8_t> &bytes) {
	for (int i = 0; i < RK_HEADER_LEN; i++) {
		rk_output_byte(samples, 0x00, RK_SIZE_20);
	}

	rk_output_byte(samples, 0xE6, RK_SIZE_20);

	for (int i = 0; i < bytes.size(); i++)
		rk_output_byte(samples, bytes[i], RK_SIZE_20);
}


static void rk22_cas_fill_wave(std::vector<int16_t> &samples, std::vector<uint8_t> &bytes) {
	for (int i = 0; i < RK_HEADER_LEN; i++) {
		rk_output_byte(samples, 0x00, RK_SIZE_22 );
	}

	rk_output_byte(samples, 0xE6, RK_SIZE_22);

	for (int i = 0; i < bytes.size(); i++)
		rk_output_byte(samples, bytes[i], RK_SIZE_22);
}


static void rk60_cas_fill_wave(std::vector<int16_t> &samples, std::vector<uint8_t> &bytes) {
	for (int i = 0; i < RK_HEADER_LEN; i++) {
		rk_output_byte(samples, 0x00, RK_SIZE_60 );
	}

	rk_output_byte(samples, 0xE6, RK_SIZE_60);

	for (int i = 0; i < bytes.size(); i++)
		rk_output_byte(samples, bytes[i], RK_SIZE_60);
}


static void gam_cas_fill_wave(std::vector<int16_t> &samples, std::vector<uint8_t> &bytes) {
	for (int i = 0; i < RK_HEADER_LEN; i++) {
		rk_output_byte(samples, 0x00, RK_SIZE_20 );
	}

	for (int i = 0; i < bytes.size(); i++)
		rk_output_byte(samples, bytes[i], RK_SIZE_20);
}


static cassette_image::error rk20_cassette_identify( cassette_image *cassette, cassette_image::Options *opts ) {
	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = RK_WAV_FREQUENCY;
	return cassette_image::error::SUCCESS;
}


static cassette_image::error rk20_cassette_load( cassette_image *cassette ) {
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> bytes(file_size);
	cassette->image_read(&bytes[0], 0, file_size);
	std::vector<int16_t> samples;

	rk20_cas_fill_wave(samples, bytes);

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / RK_WAV_FREQUENCY, samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


static cassette_image::error rk22_cassette_identify( cassette_image *cassette, cassette_image::Options *opts ) {
	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = RK_WAV_FREQUENCY;
	return cassette_image::error::SUCCESS;
}

static cassette_image::error rk22_cassette_load( cassette_image *cassette ) {
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> bytes(file_size);
	cassette->image_read(&bytes[0], 0, file_size);
	std::vector<int16_t> samples;

	rk22_cas_fill_wave(samples, bytes);

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / RK_WAV_FREQUENCY, samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


static cassette_image::error gam_cassette_identify( cassette_image *cassette, cassette_image::Options *opts ) {
	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = RK_WAV_FREQUENCY;
	return cassette_image::error::SUCCESS;
}


static cassette_image::error gam_cassette_load( cassette_image *cassette ) {
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> bytes(file_size);
	cassette->image_read(&bytes[0], 0, file_size);
	std::vector<int16_t> samples;

	gam_cas_fill_wave(samples, bytes);

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / RK_WAV_FREQUENCY, samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


static cassette_image::error rk60_cassette_identify( cassette_image *cassette, cassette_image::Options *opts ) {
	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = RK_WAV_FREQUENCY;
	return cassette_image::error::SUCCESS;
}


static cassette_image::error rk60_cassette_load( cassette_image *cassette ) {
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> bytes(file_size);
	cassette->image_read(&bytes[0], 0, file_size);
	std::vector<int16_t> samples;

	rk60_cas_fill_wave(samples, bytes);

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / RK_WAV_FREQUENCY, samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


static const cassette_image::Format rku_cassette_format = {
	"rku",
	rk20_cassette_identify,
	rk20_cassette_load,
	nullptr
};


static const cassette_image::Format rk8_cassette_format = {
	"rk8",
	rk60_cassette_identify,
	rk60_cassette_load,
	nullptr
};


static const cassette_image::Format rks_cassette_format = {
	"rks",
	rk20_cassette_identify,
	rk20_cassette_load,
	nullptr
};


static const cassette_image::Format rko_cassette_format = {
	"rko",
	rk20_cassette_identify,
	rk20_cassette_load,
	nullptr
};


static const cassette_image::Format rkr_cassette_format = {
	"rk,rkr",
	rk20_cassette_identify,
	rk20_cassette_load,
	nullptr
};


static const cassette_image::Format rka_cassette_format = {
	"rka",
	rk20_cassette_identify,
	rk20_cassette_load,
	nullptr
};


static const cassette_image::Format rkm_cassette_format = {
	"rkm",
	rk22_cassette_identify,
	rk22_cassette_load,
	nullptr
};


static const cassette_image::Format rkp_cassette_format = {
	"rkp",
	rk20_cassette_identify,
	rk20_cassette_load,
	nullptr
};


static const cassette_image::Format gam_cassette_format = {
	"gam,g16,pki",
	gam_cassette_identify,
	gam_cassette_load,
	nullptr
};


CASSETTE_FORMATLIST_START(rku_cassette_formats)
	CASSETTE_FORMAT(rku_cassette_format)
CASSETTE_FORMATLIST_END


CASSETTE_FORMATLIST_START(rk8_cassette_formats)
	CASSETTE_FORMAT(rk8_cassette_format)
CASSETTE_FORMATLIST_END


CASSETTE_FORMATLIST_START(rks_cassette_formats)
	CASSETTE_FORMAT(rks_cassette_format)
CASSETTE_FORMATLIST_END


CASSETTE_FORMATLIST_START(rko_cassette_formats)
	CASSETTE_FORMAT(rko_cassette_format)
CASSETTE_FORMATLIST_END


CASSETTE_FORMATLIST_START(rkr_cassette_formats)
	CASSETTE_FORMAT(rkr_cassette_format)
	CASSETTE_FORMAT(gam_cassette_format)
CASSETTE_FORMATLIST_END


CASSETTE_FORMATLIST_START(rka_cassette_formats)
	CASSETTE_FORMAT(rka_cassette_format)
CASSETTE_FORMATLIST_END


CASSETTE_FORMATLIST_START(rkm_cassette_formats)
	CASSETTE_FORMAT(rkm_cassette_format)
CASSETTE_FORMATLIST_END


CASSETTE_FORMATLIST_START(rkp_cassette_formats)
	CASSETTE_FORMAT(rkp_cassette_format)
CASSETTE_FORMATLIST_END


CASSETTE_FORMATLIST_START(gam_cassette_formats)
	CASSETTE_FORMAT(gam_cassette_format)
CASSETTE_FORMATLIST_END
