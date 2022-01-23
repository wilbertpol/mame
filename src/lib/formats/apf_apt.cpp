// license:BSD-3-Clause
// copyright-holders: Original author, Robbbert
/********************************************************************

Support for APF Imagination Machine cassette images

CPF and CAS images consist of the screen and then the program,
and are exactly 1E00 bytes in length.

APT images are much the same, however it includes a series of FF
bytes as a header. There's also a large amount of what seems to
be rubbish at the end.

APW images are not emulated, and are used by the closed-source
emulator APF_EMUW. Quote: "They allow recording in special formats
and recording audio. They are audio files sampled at 11025 Hz 8 bits
unsigned mono, without header. The bit 1 stores the state of the
recording head."

S19 images are not emulated, however there's no need to as they
are only used to hold cartridge hex dumps.

TXT images can be copy/pasted by using the Paste menu option.

Each byte after conversion becomes bit 7,6,etc to 0, There are
no start or stop bits.

An actual tape consists of 6 sections
a. silence until you press Enter (no offset)
b. 11secs of high bits then 1 low bit
c. The screen ram
d. The program ram
e. A checksum byte (8-bit addition)

********************************************************************/

#include "formats/apf_apt.h"

#define WAVEENTRY_LOW  -32768
#define WAVEENTRY_HIGH  32767

/* frequency of wave */
#define APF_WAV_FREQUENCY   8000

// 500 microsecond of bit 0 and 1000 microsecond of bit 1


static void apf_put_samples(std::vector<int16_t> &samples, int count, int level)
{
	for (int i = 0; i < count; i++)
		samples.push_back(level);
}


static void apf_output_bit(std::vector<int16_t> &samples, bool bit)
{
	if (bit)
	{
		apf_put_samples(samples, 4, WAVEENTRY_HIGH);
		apf_put_samples(samples, 4, WAVEENTRY_LOW);
	}
	else
	{
		apf_put_samples(samples, 2, WAVEENTRY_HIGH);
		apf_put_samples(samples, 2, WAVEENTRY_LOW);
	}
}


static void apf_output_byte(std::vector<int16_t> &samples, uint8_t byte)
{
	for (int i = 0; i < 8; i++)
		apf_output_bit(samples, (byte >> (7-i)) & 1);
}


static void apf_apt_handle_cassette(std::vector<int16_t> &samples, std::vector<uint8_t> &bytes)
{
	uint8_t cksm = 0;
	uint32_t temp = 0;

	// silence
	apf_put_samples(samples, 12000, 0);

	for (int i = 0; i < bytes.size(); i++)
	{
		apf_output_byte(samples, bytes[i]);
		if (bytes[i] == 0xfe)
		{
			temp = i+1;
			i = bytes.size();
		}
	}

	// data
	for (uint32_t i = temp; i < (temp + 0x1e00) && i < bytes.size(); i++)
	{
		cksm += bytes[i];
		apf_output_byte(samples, bytes[i]);
	}

	// checksum byte
	apf_output_byte(samples, cksm);
}


static void apf_cpf_handle_cassette(std::vector<int16_t> &samples, std::vector<uint8_t> &bytes)
{
	uint8_t cksm = 0;

	// silence
	apf_put_samples(samples, 12000, 0);

	// start
	for (int i = 0; i < 10000; i++)
		apf_output_bit(samples, 1);

	apf_output_bit(samples, 0);

	// data
	for (int i = 0; i < bytes.size(); i++)
	{
		cksm += bytes[i];
		apf_output_byte(samples, bytes[i]);
	}

	// checksum byte
	apf_output_byte(samples, cksm);
}


static cassette_image::error apf_cpf_identify(cassette_image *cassette, cassette_image::Options *opts)
{
	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = APF_WAV_FREQUENCY;
	return cassette_image::error::SUCCESS;
}


static cassette_image::error apf_cpf_load(cassette_image *cassette)
{
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> bytes(file_size);
	cassette->image_read(&bytes[0], 0, file_size);
	std::vector<int16_t> samples;

	apf_cpf_handle_cassette(samples, bytes);

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / APF_WAV_FREQUENCY, samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


static const cassette_image::Format apf_cpf_format =
{
	"cas,cpf",
	apf_cpf_identify,
	apf_cpf_load,
	nullptr
};


static cassette_image::error apf_apt_identify(cassette_image *cassette, cassette_image::Options *opts)
{
	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = APF_WAV_FREQUENCY;
	return cassette_image::error::SUCCESS;
}


static cassette_image::error apf_apt_load(cassette_image *cassette)
{
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> bytes(file_size);
	cassette->image_read(&bytes[0], 0, file_size);
	std::vector<int16_t> samples;

	apf_apt_handle_cassette(samples, bytes);

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / APF_WAV_FREQUENCY, samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


static const cassette_image::Format apf_apt_format =
{
	"apt",
	apf_apt_identify,
	apf_apt_load,
	nullptr
};


CASSETTE_FORMATLIST_START(apf_cassette_formats)
	CASSETTE_FORMAT(apf_cpf_format)
	CASSETTE_FORMAT(apf_apt_format)
CASSETTE_FORMATLIST_END
