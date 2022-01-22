// license:BSD-3-Clause
// copyright-holders:Nathan Woods
#include "mz_cas.h"

#include <cstring>

#ifndef VERBOSE
#define VERBOSE 0
#endif

//#define LOG(N,M,A)
//  if(VERBOSE>=N){ if( M )LOG_FORMATS("%11.6f: %-24s",machine.time().as_double(), (const char*)M ); LOG_FORMATS A; }

#define LOG(N,M,A)  \
	if(VERBOSE>=N){ if( M )printf("%-24s",(const char*)M ); printf A; }

#define LO  -32768
#define HI  +32767

#define SHORT_PULSE 2
#define LONG_PULSE  4

#define BYTE_SAMPLES (LONG_PULSE+8*LONG_PULSE)

#define SILENCE     8000

/* long gap and tape mark */
#define LGAP        22000
#define LTM_1       40
#define LTM_0       40
#define LTM_L       1

/* short gap and tape mark */
#define SGAP        11000
#define STM_1       20
#define STM_0       20
#define STM_L       1


static void fill_wave_1(std::vector<int16_t> &samples)
{
	samples.push_back(HI);
	samples.push_back(HI);
	samples.push_back(LO);
	samples.push_back(LO);
}


static void fill_wave_0(std::vector<int16_t> &samples)
{
	samples.push_back(HI);
	samples.push_back(LO);
}


static void fill_wave_b(std::vector<int16_t> &samples, uint8_t byte)
{
	// data bits are preceded by a long pulse
	fill_wave_1(samples);

	for (int i = 7; i >= 0; i--)
	{
		if ((byte >> i) & 1)
			fill_wave_1(samples);
		else
			fill_wave_0(samples);
	}
}


static void fill_wave(std::vector<int16_t> &samples, std::vector<uint8_t> &bytes)
{
	// header
	LOG(1,"mz700 fill_wave",("LGAP %d samples\n", LGAP * SHORT_PULSE));
	// fill long gap - LGAP
	for (int i = 0; i < LGAP; i++)
		fill_wave_0(samples);

	// make a long tape mark - LTM
	LOG(1,"mz700_fill_wave",("LTM 1 %d samples\n", LTM_1 * LONG_PULSE));
	for (int i = 0; i < LTM_1; i++)
		fill_wave_1(samples);

	LOG(1,"mz700_fill_wave",("LTM 0 %d samples\n", LTM_0 * SHORT_PULSE));
	for (int i = 0; i < LTM_0; i++)
		fill_wave_0(samples);

	// L
	LOG(1,"mz700_fill_wave",("L %d samples\n", LONG_PULSE));
	fill_wave_1(samples);

	// HDR begins here
	int pos = 0;
	uint16_t csum = 0;
	while (pos < bytes.size() && pos < 128)
	{
		if (bytes[pos] & 0x01) csum++;
		if (bytes[pos] & 0x02) csum++;
		if (bytes[pos] & 0x04) csum++;
		if (bytes[pos] & 0x08) csum++;
		if (bytes[pos] & 0x10) csum++;
		if (bytes[pos] & 0x20) csum++;
		if (bytes[pos] & 0x40) csum++;
		if (bytes[pos] & 0x80) csum++;

		fill_wave_b(samples, bytes[pos++]);
	}

	if (pos == 128)
	{
		LOG(1,"mz700_fill_wave",("CHKH 0x%04X\n", csum & 0xffff));
		// CHKH
		fill_wave_b(samples, (csum >> 8) & 0xff);
		fill_wave_b(samples, csum & 0xff);

		LOG(1,"mz700_fill_wave",("L %d samples\n", LONG_PULSE));
		// L
		fill_wave_1(samples);

		// 256S
		LOG(1,"mz700_fill_wave",("256S\n"));
		for (int i = 0; i < 256; i++)
			fill_wave_0(samples);

		// copy of HDR - HDRC
		pos = 0;
		uint32_t hdrc_start = samples.size();
		while (pos < bytes.size() && pos < 128)
		{
			fill_wave_b(samples, bytes[pos++]);
		}
		LOG(1,"mz700_fill_wave",("HDRC %lu samples\n", samples.size() - hdrc_start));

		LOG(1,"mz700_fill_wave",("CHKH 0x%04X\n", csum & 0xffff));
		fill_wave_b(samples, (csum >> 8) & 0xff);
		fill_wave_b(samples, csum & 0xff);

		// L
		LOG(1,"mz700_fill_wave",("L %d samples\n", LONG_PULSE));
		fill_wave_1(samples);

		LOG(1,"mz700_fill_wave",("SILENCE %d samples\n", SILENCE));
		// fill silence
		for (int i = 0; i < SILENCE; i++)
			samples.push_back(0);

		LOG(1,"mz700_fill_wave",("SGAP %d samples\n", SGAP * SHORT_PULSE));
		// fill short gap - SGAP
		for (int i = 0; i < SGAP; i++)
			fill_wave_0(samples);

		// make a short tape mark - STM

		LOG(1,"mz700_fill_wave",("STM 1 %d samples\n", STM_1 * LONG_PULSE));
		for (int i = 0; i < STM_1; i++)
			fill_wave_1(samples);

		LOG(1,"mz700_fill_wave",("STM 0 %d samples\n", STM_0 * SHORT_PULSE));
		for (int i = 0; i < STM_0; i++)
			fill_wave_0(samples);

		// L
		LOG(1,"mz700_fill_wave",("L %d samples\n", LONG_PULSE));
		fill_wave_1(samples);
	}

	// FILE begins here
	csum = 0;
	int file_start = pos;

	while (pos < bytes.size())
	{
		if (bytes[pos] & 0x01) csum++;
		if (bytes[pos] & 0x02) csum++;
		if (bytes[pos] & 0x04) csum++;
		if (bytes[pos] & 0x08) csum++;
		if (bytes[pos] & 0x10) csum++;
		if (bytes[pos] & 0x20) csum++;
		if (bytes[pos] & 0x40) csum++;
		if (bytes[pos] & 0x80) csum++;

		fill_wave_b(samples, bytes[pos++]);
	}

	// trailer
	{
		// CHKF
		LOG(1,"mz700_fill_wave",("CHKF 0x%04X\n", csum));
		fill_wave_b(samples, csum >> 8);
		fill_wave_b(samples, csum & 0xff);

		// L
		LOG(1,"mz700_fill_wave",("L\n"));
		fill_wave_1(samples);

		// 256S
		LOG(1,"mz700_fill_wave",("256S\n"));
		for (int i = 0; i < 256; i++)
			fill_wave_0(samples);

		// FILEC
		pos = file_start;
		uint32_t filec_start = samples.size();
		while (pos < bytes.size())
		{
			fill_wave_b(samples, bytes[pos++]);
		}

		LOG(1,"mz700_fill_wave",("FILEC %lu samples\n", samples.size() - filec_start));

		// CHKF
		LOG(1,"mz700_fill_wave",("CHKF 0x%04X\n", csum));
		fill_wave_b(samples, csum >> 8);
		fill_wave_b(samples, csum & 0xff);

		// L
		LOG(1,"mz700_fill_wave",("L %d samples\n", LONG_PULSE));
		fill_wave_1(samples);

		LOG(1,"mz700_fill_wave",("silence %d samples\n", SILENCE));
		// silence at the end
		for (int i = 0; i < SILENCE; i++)
			samples.push_back(0);
	}
}


static cassette_image::error mz700_cas_identify(cassette_image *cassette, cassette_image::Options *opts)
{
	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = 4400;
	return cassette_image::error::SUCCESS;
}


static cassette_image::error mz700_cas_load(cassette_image *cassette)
{
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> bytes(file_size);
	cassette->image_read(&bytes[0], 0, file_size);
	std::vector<int16_t> samples;

	fill_wave(samples, bytes);

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / 4400, samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


static const cassette_image::Format mz700_cas_format =
{
	"m12,mzf,mzt",
	mz700_cas_identify,
	mz700_cas_load,
	nullptr
};


CASSETTE_FORMATLIST_START(mz700_cassette_formats)
	CASSETTE_FORMAT(mz700_cas_format)
CASSETTE_FORMATLIST_END
