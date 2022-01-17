// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/*********************************************************************

    flacfile.cpp

    Format code for fLaC (*.flac) files

To investigate:
Not all flac files work as expected. For instance, converting
the abcstack wav file from the abc80 softlist directly into a
flac file results in a flac file will not load in the emulation.
When converting the wav to a 44100 flac then it does work.
Converting the directly converted flac back to a wav also works.
*********************************************************************/

#include "flacfile.h"
#include "flac.h"


cassette_image::error flacfile_identify(cassette_image *cassette, cassette_image::Options *opts)
{
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> file_contents(file_size);
	cassette->image_read(&file_contents[0], 0, file_size);
	flac_decoder decoder(&file_contents[0], file_size);
	decoder.reset();
	int channels = decoder.channels();
	int sample_rate = decoder.sample_rate();
	int bits_per_sample = decoder.bits_per_sample();
	int total_samples = decoder.total_samples();
	decoder.finish();

	opts->channels = channels;
	opts->sample_frequency = sample_rate;
	opts->bits_per_sample = bits_per_sample;

	if (channels > 0 && sample_rate > 0 && total_samples > 0)
	{
		return cassette_image::error::SUCCESS;
	}
	else 
	{
		return cassette_image::error::INVALID_IMAGE;
	}
}


static cassette_image::error flacfile_load(cassette_image *cassette)
{
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> file_contents(file_size);
	cassette->image_read(&file_contents[0], 0, file_size);
	flac_decoder decoder(&file_contents[0], file_size);
	decoder.reset();
	int channels = decoder.channels();
	int total_samples = decoder.total_samples();

	std::vector<int16_t> samples[channels];
	int16_t *channel_samples[channels];
	for (int channel = 0; channel < channels; channel++)
	{
		samples[channel].resize(total_samples);
		channel_samples[channel] = &samples[channel][0];
	}
	decoder.decode(channel_samples, decoder.total_samples(), false);
	for (int channel = 0; channel < channels; channel++)
	{
		cassette_image::error err = cassette->put_samples(channel, 0.0,
			(double)total_samples/decoder.sample_rate(), total_samples, 2,
			channel_samples[channel], cassette_image::WAVEFORM_16BITLE);
		if (err != cassette_image::error::SUCCESS)
			return err;
	}

	/* success! */
	return cassette_image::error::SUCCESS;
}


const cassette_image::Format cassette_image::flacfile_format =
{
	"flac",
	flacfile_identify,
	flacfile_load,
	nullptr
};
