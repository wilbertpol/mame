// license:BSD-3-Clause
// copyright-holders:Kevin Thacker
#include "oric_tap.h"
#include "imageutl.h"


#define ORIC_WAV_DEBUG 0
#define LOG(x) do { if (ORIC_WAV_DEBUG) printf x; } while (0)

/* this code based heavily on tap2wav by Fabrice Frances */
#define ORIC_SYNC_BYTE  0x016

/* frequency of wave */
/* tapes use 1200Hz and 2400Hz samples */
#define ORIC_WAV_FREQUENCY  4800

/* 13 bits define a byte on the cassette */
/* 1 start bit, 8 data bits, 1 parity bit and 3 stop bits */
#define ORIC_BYTE_TO_BITS_ON_CASSETTE 13

#define ORIC_WAVESAMPLES_HEADER  3000
#define ORIC_WAVESAMPLES_TRAILER 1000

enum
{
	ORIC_CASSETTE_SEARCHING_FOR_SYNC_BYTE,
	ORIC_CASSETTE_GOT_SYNC_BYTE,
	ORIC_CASSETTE_READ_HEADER,
	ORIC_CASSETTE_READ_FILENAME,
	ORIC_CASSETTE_WRITE_DATA
};

#define WAVEENTRY_LOW  -32768
#define WAVEENTRY_HIGH  32767
#define WAVEENTRY_NULL  0

#define ORIC_LEADER_LENGTH 512


struct oric_t
{
	int cassette_state;
	int data_count;
	int data_length;
};


/* to write a bit to the tape, the rom routines output either 4 periods at 1200 Hz for a 0 or 8 periods at 2400 Hz for a 1  */
/* 4800 is twice 2400Hz */

/* 8 periods at 2400Hz */
/* hi,lo, hi,lo, hi,lo, hi,lo */

static void oric_emit_level(std::vector<int16_t> &samples, int count, int16_t wave_state)
{
	for (int i = 0; i < count; i++)
	{
		samples.push_back(wave_state);
	}
}

/* 4 periods at 1200Hz */
static void oric_output_bit(std::vector<int16_t> &samples, uint8_t b)
{
	oric_emit_level(samples, 1, WAVEENTRY_HIGH);
	oric_emit_level(samples, b ? 1 : 2, WAVEENTRY_LOW);
}


/*  each byte on cassette is stored as:

    start bit       0 * 1
    data bits       8 * x (x is 0 or 1, and depends on data-bit value)
    parity bit      1 * x (x is 0 or 1, and depends on the parity of the data bits)
    stop bits       1 * 3

    if data has even parity, parity bit will be 1.
    if data has odd parity, parity bit will be 0.
*/

/*
    512 * data byte 0x016       -> leader
    1   * data byte 0x024       -> sync byte
    9   * data byte             -> header
    delay (of last pulse written)
    x   * data byte         -> length


    header structure:
    3   * ?         ->      ???
    1   * ?         ->      ???
    1   * x         ->      end address high byte
    1   * x         ->      end address low byte
    1   * x         ->      start address high byte
    1   * x         ->      start address low byte
    1   * ?         ->      ???
*/


static void oric_output_byte(std::vector<int16_t> &samples, uint8_t byte)
{
	// start bit
	oric_output_bit(samples, 0);

	// set initial parity
	uint8_t parity = 1;

	// data bits, written bit 0, bit 1...bit 7
	uint8_t data = byte;
	for (int i = 0; i < 8; i++)
	{
		uint8_t data_bit = data & 0x01;

		parity = parity + data_bit;

		oric_output_bit(samples, data_bit);
		data = data >> 1;
	}

	// parity
	oric_output_bit(samples, parity & 0x01);

	// stop bits
	oric_output_bit(samples, 1);
	oric_output_bit(samples, 1);
	oric_output_bit(samples, 1);
	oric_output_bit(samples, 1);
}


static void oric_fill_pause(std::vector<int16_t> &samples, int sample_count)
{
	for (int i = 0; i < sample_count; i++)
	{
		samples.push_back(WAVEENTRY_NULL);
	}
}


static int oric_seconds_to_samples(float seconds)
{
	return (int)((float)seconds*(float)ORIC_WAV_FREQUENCY);
}


static void oric_cassette_fill_wave(std::vector<int16_t> &samples, std::vector<uint8_t> &bytes)
{
	oric_t oric = { 0 };
	unsigned char header[9];

	// header and trailer act as pauses */
	// the trailer is required so that the via sees the last bit of the last byte
	// header samples
	for (int i = 0; i < ORIC_WAVESAMPLES_HEADER; i++)
		samples.push_back(WAVEENTRY_NULL);

	oric.cassette_state = ORIC_CASSETTE_SEARCHING_FOR_SYNC_BYTE;
	int pos = 0;

	while (pos < bytes.size())
	{
		uint8_t data = bytes[pos++];

		switch (oric.cassette_state)
		{
			case ORIC_CASSETTE_SEARCHING_FOR_SYNC_BYTE:
			{
				if (data == ORIC_SYNC_BYTE)
				{
					LOG_FORMATS("found sync byte!\n");
					// found first sync byte
					oric.cassette_state = ORIC_CASSETTE_GOT_SYNC_BYTE;
				}
			}
			break;

			case ORIC_CASSETTE_GOT_SYNC_BYTE:
			{
				if (data != ORIC_SYNC_BYTE)
				{
					// 0.25 second pause
					oric_fill_pause(samples, oric_seconds_to_samples(0.25));

					LOG_FORMATS("found end of sync bytes!\n");
					// found end of sync bytes
					for (int i = 0; i < ORIC_LEADER_LENGTH; i++)
					{
						oric_output_byte(samples, 0x016);
					}

					if (data == 0x024)
					{
						//LOG_FORMATS("reading header!\n");
						oric_output_byte(samples, data);
						oric.cassette_state = ORIC_CASSETTE_READ_HEADER;
						oric.data_count = 0;
						oric.data_length = 9;
					}
				}
			}
			break;

			case ORIC_CASSETTE_READ_HEADER:
			{
				header[oric.data_count] = data;
				oric_output_byte(samples, data);
				oric.data_count++;

				if (oric.data_count == oric.data_length)
				{
					//LOG_FORMATS("finished reading header!\n");
					oric.cassette_state = ORIC_CASSETTE_READ_FILENAME;
				}
			}
			break;

			case ORIC_CASSETTE_READ_FILENAME:
			{
				oric_output_byte(samples, data);

				// got end of filename?
				if (data == 0)
				{
					uint16_t end, start;
					LOG_FORMATS("got end of filename\n");

					// oric includes a small delay, but I don't see it being 1 bits
					for (int i=0; i<100; i++)
					{
						oric_output_bit(samples, 1);
					}

					oric.cassette_state = ORIC_CASSETTE_WRITE_DATA;
					oric.data_count = 0;

					end = (((header[4] & 0x0ff)<<8) | (header[5] & 0x0ff));
					start = (((header[6] & 0x0ff)<<8) | (header[7] & 0x0ff));
					LOG(("start (from header): %02x\n",start));
					LOG(("end (from header): %02x\n",end));
					oric.data_length = end - start + 1;
				}
			}
			break;

			case ORIC_CASSETTE_WRITE_DATA:
			{
				oric_output_byte(samples, data);
				oric.data_count++;

				if (oric.data_count == oric.data_length)
				{
					LOG_FORMATS("finished writing data!\n");
					oric.cassette_state = ORIC_CASSETTE_SEARCHING_FOR_SYNC_BYTE;
				}
			}
			break;

		}
	}

	// trailer samples
	for (int i = 0; i < ORIC_WAVESAMPLES_TRAILER; i++)
		samples.push_back(WAVEENTRY_NULL);
}


static cassette_image::error oric_tap_identify(cassette_image *cassette, cassette_image::Options *opts)
{
	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = ORIC_WAV_FREQUENCY;
	return cassette_image::error::SUCCESS;
}


static cassette_image::error oric_tap_load(cassette_image *cassette)
{
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> bytes(file_size);
	cassette->image_read(&bytes[0], 0, file_size);
	std::vector<int16_t> samples;

	oric_cassette_fill_wave(samples, bytes);

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / ORIC_WAV_FREQUENCY, samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


static const cassette_image::Format oric_tap_format =
{
	"tap",
	oric_tap_identify,
	oric_tap_load,
	nullptr
};


CASSETTE_FORMATLIST_START(oric_cassette_formats)
	CASSETTE_FORMAT(oric_tap_format)
CASSETTE_FORMATLIST_END
