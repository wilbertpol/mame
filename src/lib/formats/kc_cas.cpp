// license:BSD-3-Clause
// copyright-holders:Sandro Ronco
/********************************************************************

    Support for KC85 cassette images

    Supported formats:
    - kcc: raw cassette image without ID and checksum
    - tap: cassette image from KC-Emulator with head and ID
    - tp2: cassette image with ID and checksum (130 bytes block)
    - kcm: same as tp2 but without head
    - sss: BASIC data without head (miss the first 11 bytes)

********************************************************************/

#include "kc_cas.h"

#include <cstring>

#define SMPLO       -32768
#define SMPHI       32767
#define SILENCE     0

#define KC_WAV_FREQUENCY        44100

// from documentation
#define FREQ_BIT_0          2400
#define FREQ_BIT_1          1200
#define FREQ_SEPARATOR      600

// file formats
enum
{
	KC_IMAGE_KCC,
	KC_IMAGE_TP2,
	KC_IMAGE_TAP,
	KC_IMAGE_KCM
};


/*******************************************************************
   Generate one high-low cycle of sample data
********************************************************************/
static void kc_cas_cycle(std::vector<int16_t> &samples, int len)
{
	int num_samples = KC_WAV_FREQUENCY / (len * 2);

	for (int i = 0; i < num_samples; i++)
		samples.push_back(SMPHI);

	for (int i = 0; i < num_samples; i++)
		samples.push_back(SMPLO);
}


/*******************************************************************
   Generate n samples of silence
********************************************************************/
static void kc_cas_silence(std::vector<int16_t> &samples, int len)
{
	for (int i = 0; i < len; i++)
		samples.push_back(SILENCE);
}


/*******************************************************************
   Generate samples for 1 byte
********************************************************************/
static void kc_cas_byte(std::vector<int16_t> &samples, uint8_t data)
{
	// write the byte
	for (int i = 0; i < 8; i++)
	{
		if (data & 0x01)
		{
			kc_cas_cycle(samples, FREQ_BIT_1);
		}
		else
		{
			kc_cas_cycle(samples, FREQ_BIT_0);
		}

		data >>= 1;
	}

	// byte separator
	kc_cas_cycle(samples, FREQ_SEPARATOR);
}


static void kc_handle_cass(std::vector<int16_t> &samples, std::vector<uint8_t> &bytes, int type)
{
	int data_pos = (type == KC_IMAGE_KCC || type == KC_IMAGE_KCM) ? 0 : 16;
	int block_id = 1;

	// 1 sec of silence at start
	kc_cas_silence(samples, KC_WAV_FREQUENCY);

	// 8000 cycles of BIT_1 for synchronization
	for (int i = 0; i < 8000; i++)
		kc_cas_cycle(samples, FREQ_BIT_1);

	// on the entire file
	while (data_pos < bytes.size())
	{
		uint8_t checksum = 0;

		// 200 cycles of BIT_1 every block
		for (int i = 0; i < 200; i++)
			kc_cas_cycle(samples, FREQ_BIT_1);

		// separator
		kc_cas_cycle(samples, FREQ_SEPARATOR);

		// in TAP and TP2 file the first byte is the ID
		if (type == KC_IMAGE_TAP || type == KC_IMAGE_TP2 || type == KC_IMAGE_KCM)
			block_id = bytes[data_pos++];

		// is the last block ?
		if (data_pos + 128 >= bytes.size() && type == KC_IMAGE_KCC)
			block_id = 0xff;

		// write the block ID
		kc_cas_byte(samples, block_id);

		// write the 128 bytes of the block
		for (int i = 0; i < 128; i++)
		{
			uint8_t data = 0;

			if (data_pos < bytes.size())
				data = bytes[data_pos++];

			// calculate the checksum
			checksum += data;

			// write a byte
			kc_cas_byte(samples, data);
		}

		// TP2 and KCM files also have the checksum byte
		if ((type == KC_IMAGE_TP2 || type == KC_IMAGE_KCM) && data_pos < bytes.size())
			checksum = bytes[data_pos++];

		// 8bit checksum
		kc_cas_byte(samples, checksum);

		// more TAP and TP2 can be combined into the same file
		if ((type == KC_IMAGE_TAP || type == KC_IMAGE_TP2) && block_id == 0xff && data_pos < bytes.size())
		{
			if (bytes[data_pos] == 0xc3 || bytes[data_pos] == 0x4b)
			{
				kc_cas_silence(samples, KC_WAV_FREQUENCY/10);

				data_pos += 16;
			}
		}

		block_id++;
	}

	kc_cas_cycle(samples, FREQ_SEPARATOR);

	// 1 sec of silence
	kc_cas_silence(samples, KC_WAV_FREQUENCY);
}


static cassette_image::error kc_handle_tap(std::vector<int16_t> &samples, std::vector<uint8_t> &bytes)
{
	if (bytes.size() > 13 && !strncmp((const char *)&bytes[1], "KC-TAPE by AF", 13))
	{
		kc_handle_cass(samples, bytes, KC_IMAGE_TAP);
		return cassette_image::error::SUCCESS;
	}
	else if (bytes.size() > 4 && !strncmp((const char *)&bytes[0], "KC85", 4))
	{
		kc_handle_cass(samples, bytes, KC_IMAGE_TP2);
		return cassette_image::error::SUCCESS;
	}
	else if (bytes.size() > 1 && bytes[0] == 0x01)
	{
		kc_handle_cass(samples, bytes, KC_IMAGE_KCM);
		return cassette_image::error::SUCCESS;
	}
	return cassette_image::error::INVALID_IMAGE;
}


static void kc_handle_sss(std::vector<int16_t> &samples, std::vector<uint8_t> &bytes)
{
	std::vector<uint8_t> sss(bytes.size() + 11);

	// tries to generate the missing head
	memset(&sss[0], 0xd3, 3);
	memset(&sss[3], 0x20, 8);
	memcpy(&sss[11], &bytes[0], bytes.size());

	// set an arbitrary filename
	sss[3] = 'A';

	kc_handle_cass(samples, sss, KC_IMAGE_KCC);
}


static cassette_image::error kc_kcc_identify(cassette_image *cassette, cassette_image::Options *opts)
{
	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = KC_WAV_FREQUENCY;
	return cassette_image::error::SUCCESS;
}


static cassette_image::error kc_kcc_load(cassette_image *cassette)
{
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> bytes(file_size);
	cassette->image_read(&bytes[0], 0, file_size);
	std::vector<int16_t> samples;

	kc_handle_cass(samples, bytes, KC_IMAGE_KCC);

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / KC_WAV_FREQUENCY, samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


static const cassette_image::Format kc_kcc_format =
{
	"kcc,kcb",
	kc_kcc_identify,
	kc_kcc_load,
	nullptr
};


static cassette_image::error kc_tap_identify(cassette_image *cassette, cassette_image::Options *opts)
{
	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = KC_WAV_FREQUENCY;
	return cassette_image::error::SUCCESS;
}


static cassette_image::error kc_tap_load(cassette_image *cassette)
{
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> bytes(file_size);
	cassette->image_read(&bytes[0], 0, file_size);
	std::vector<int16_t> samples;

	cassette_image::error err = kc_handle_tap(samples, bytes);
	if (err != cassette_image::error::SUCCESS)
		return err;

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / KC_WAV_FREQUENCY, samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


static const cassette_image::Format kc_tap_format =
{
	"tap,853,854,855,tp2,kcm",
	kc_tap_identify,
	kc_tap_load,
	nullptr
};


static cassette_image::error kc_sss_identify(cassette_image *cassette, cassette_image::Options *opts)
{
	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = KC_WAV_FREQUENCY;
	return cassette_image::error::SUCCESS;
}


static cassette_image::error kc_sss_load(cassette_image *cassette)
{
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> bytes(file_size);
	cassette->image_read(&bytes[0], 0, file_size);
	std::vector<int16_t> samples;

	kc_handle_sss(samples, bytes);

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / KC_WAV_FREQUENCY, samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


static const cassette_image::Format kc_sss_format =
{
	"sss",
	kc_sss_identify,
	kc_sss_load,
	nullptr
};


CASSETTE_FORMATLIST_START(kc_cassette_formats)
	CASSETTE_FORMAT(kc_kcc_format)
	CASSETTE_FORMAT(kc_tap_format)
	CASSETTE_FORMAT(kc_sss_format)
CASSETTE_FORMATLIST_END
