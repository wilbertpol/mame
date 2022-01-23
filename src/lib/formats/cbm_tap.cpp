// license:BSD-3-Clause
// copyright-holders:Fabio Priuli
/*

    Tape support for C16 / C64 / VIC20 TAP format


    Credits to:

        - Peter Schepers for the informations on C64 formats
        - Vice Team for the source of their very complete emulator
        - Attila G. for tap2wav (both the source and the actual tool)


    TODO:
    * to verify and fix (if needed) support for .TAP v2
    * to implement reading (and logging) the remaining part of the header
    * to verify if it is more accurate to use a different HIGH_WAVE value
      when the pulse corresponds to a 0x00 in the .TAP file
    (...)
    * (far away in the future) can this code be merged with TZX code?

*/

/* Info based on http://ist.uwaterloo.ca/~schepers/formats/TAP.TXT      */
/* Please refer to the webpage for the latest version and for a very
   complete listing of various cart types and their bankswitch tricks   */
/*

  Designed by Per Hakan Sundell (author of the CCS64 C64 emulator) in 1997,
this format attempts to duplicate the data stored on a C64  cassette  tape,
bit for bit. Since it is simply a representation of  the  raw  serial  data
from a tape, it should handle *any* custom tape loaders that exist.

  The TAP images are generally very large, being a minimum of eight  times,
and up to sixteen times as large as what a raw PRG file would be.  This  is
due to the way the data is stored, with each bit of the original  file  now
being one byte large in the TAP file. The layout is fairly simple,  with  a
small 14-byte header followed by file data.

    Bytes: $0000-000B: File signature "C64-TAPE-RAW"
                 000C: TAP version (see below for description)
                        $00 - Original layout
                         01 - Updated
            000D-000F: Future expansion
            0010-0013: File  data  size  (not  including  this  header,  in
                       LOW/HIGH format).
            0014-xxxx: File data

  In TAP version $00 files, each data byte in the file data area represents
the length of a pulse, when the C64's hardware  will  trigger  again.  This
pulse length is determined by the following formula:

    pulse length (in seconds) = (8 * data byte) / (clock cycles)

  Therefore, a data value of $2F (47 in decimal) would be:

    (47 * 8) / 985248 = .00038975 seconds.

  A data value of $00 represents an "overflow" condition, any pulse  length
which is more that 255 * 8 in length.

  The value of "clock cycles" from above  (985248)  is  based  on  the  PAL
value.  Since  this  file  format  was  developed  in  Europe,   which   is
predominantly PAL video, this is only logical.  The  NTSC  value  would  be
1022730, which is very close to  the  PAL,  and  therefore  won't  cause  a
compatibility problem converting European and NTSC tapes. I would stick  to
using the PAL value just in case.


  In TAP version $01 files, the data value of  $00  has  been  re-coded  to
represent values greater than 255 * 8. When a  $00  is  encountered,  three
bytes will follow which are the actual time (in cycles) of a pulse, and the
above formula does not apply.  The  three  bytes  are  stored  in  LOW/HIGH
format.


  The actual interpretation of the serial data takes a little more work  to
explain.  The  typical  ROM  tape  loader  (and  the  turbo  loaders)  will
initialize a timer with a specified value and start it  counting  down.  If
either the tape data changes or the timer runs out, an IRQ will occur.  The
loader will determine which condition caused the  IRQ.  If  the  tape  data
changed before the timer ran out, we have a short pulse, or a "0"  bit.  If
the timer ran out first, we have a long pulse, or a  "1"  bit.  Doing  this
continuously and we decode the entire file.


[ Additional notes on v.2:

  I found no documents about this around, but it seems an expansion of the
format specifically thought for C16 tapes. In a .TAP version 2, each byte
only stores informations on half of the wave...
Unfortunately, I have no such a .tap file to test, so my implementation
below could be not working.  FP ]
*/

#include "cbm_tap.h"
#include "imageutl.h"


#define CBM_WAV_FREQUENCY   44100

/* Systems */
#define C64     0
#define VIC20   1
#define C16     2

/* Video standards */
#define PAL     0
#define NTSC    1

/* Frequencies in [Hz] to determine the length of each pulse */
#define C64_PAL     123156      /*  985248 / 8 */
#define C64_NTSC    127841      /* 1022727 / 8 */
#define VIC20_PAL   138551      /* 1108405 / 8 */
#define VIC20_NTSC  127841      /* 1022727 / 8 */
#define C16_PAL     110840      /*  886724 / 8 */
#define C16_NTSC    111860      /*  894886 / 8 */

#define PAUSE (CBM_WAV_FREQUENCY / 50)      /* tap2wav uses this value for 0x00 in .TAP v0, instead of 0x100 */

/* These values do not really matter, as long as the produced pulses
  go above & below 0. However, for documentation purpose it would be
  nice to find out which values were used by Commodore tapes. I was
  not able to find any reference on the subject. */
#define WAVE_HIGH       (0x5a9e >> 1)
#define WAVE_LOW        -(0x5a9e >> 1)
#define WAVE_PAUSE      0x80

#define CBM_HEADER_SIZE 20


/* This in fact gives the number of samples for half of the pulse */
static inline int tap_data_to_samplecount(int data, int frequency)
{
//  return (int) (0.5 * (0.5 + (((double)CBM_WAV_FREQUENCY / frequency) * (double)data)));      // MAME TZX formula
	return (int) (0.5 * (((double)CBM_WAV_FREQUENCY / frequency) * (double)((data) + 0.5)));    // tap2wav formula
}

/* The version with parameters could be handy if we decide to implement a
different values for byte == 0x00 below (the WAVE_PAUSE) as tap2wav does. */
#if 0
static void toggle_wave_data(int low, int high)
{
	wave_data = (wave_data == low) ? high : low;
}
#endif

static void toggle_wave_data(int16_t &wave_data)
{
	wave_data = (wave_data == WAVE_HIGH) ? WAVE_LOW : WAVE_HIGH;
}


static void cbm_output_wave(std::vector<int16_t> &samples, int16_t wave_data, int length)
{
	for( ; length > 0; length-- )
	{
		samples.push_back(wave_data);
	}
}


static cassette_image::error cbm_tap_do_work(std::vector<int16_t> &samples, std::vector<uint8_t> &bytes)
{
	int j = 0;
	int16_t wave_data = 0;

	int tap_frequency = 0;

	int byte_samples = 0;
	uint8_t over_pulse_bytes[3] = { 0, 0, 0 };
	int over_pulse_length = 0;
	/* These waveamp_* values are currently stored but not used.
	  Further investigations are needed to find real pulse amplitude
	  in Commodore tapes. Implementation here would follow */
	/* int waveamp_high, waveamp_low; */

	/* is the .tap file corrupted? */
	if (bytes.size() <= CBM_HEADER_SIZE)
		return cassette_image::error::INVALID_IMAGE;

	uint8_t version = bytes[0x0c];
	uint8_t system = bytes[0x0d];
	uint8_t video_standard = bytes[0x0e];

	/* Log .TAP info */
	LOG_FORMATS("TAP version    : %d\n", version);
	LOG_FORMATS("Machine type   : %d\n", system);
	LOG_FORMATS("Video standard : %d\n", video_standard);
	LOG_FORMATS("Tape frequency : %d\n", (tap_frequency) << 3);


	/* is this a supported version? */
	if ((version < 0) || (version > 2))
	{
		LOG_FORMATS("Unsupported .tap version: %d \n", version);
		return cassette_image::error::UNSUPPORTED;
	}


	/* read the frequency from the .tap header */
	switch (system)
	{
		case VIC20:
			tap_frequency = (video_standard == NTSC) ? VIC20_NTSC : VIC20_PAL;
			break;

		case C16:
			tap_frequency = (video_standard == NTSC) ? C16_NTSC : C16_PAL;
			break;

		case C64:
		default:
			tap_frequency = (video_standard == NTSC) ? C64_NTSC : C64_PAL;
			break;
	}


	for (int i = CBM_HEADER_SIZE; i < bytes.size(); i++)
	{
		uint8_t byte = bytes[i];

		/* .TAP v0 */
		/* Here is simple:
		  if byte is != 0 -> length = byte
		  otherwise -> length =  0x100 (i.e. 0xff + 1) */
		if (!version)
		{
			if (byte != 0x00)
			{
				byte_samples = tap_data_to_samplecount(byte, tap_frequency);
				/* waveamp_high = WAVE_HIGH; */
			}
			else
			{
				byte_samples = tap_data_to_samplecount(PAUSE, tap_frequency);   // tap2wav value
//              byte_samples = tap_data_to_samplecount(0x100, tap_frequency);   // vice value
				/* waveamp_high = WAVE_PAUSE; */
			}
			/* waveamp_low = WAVE_LOW; */
		}

		/* .TAP v1 & v2 */
		/* Here is a bit more complicate:
		  if byte is != 0 -> length = byte
		  otherwise -> the length of the pulse is stored as a 24bit value in the 3 bytes after the 0.
		  See below for comments on the implementation of this mechanism */
		if (version)
		{
			if ((byte != 0x00) && !j)
			{
				byte_samples = tap_data_to_samplecount(byte, tap_frequency);
				/* waveamp_high = WAVE_HIGH; */
			}
			else
			{
				/* If we have a long pulse close to the end of the .TAP, check that bytes still
				  to be read are enough to complete it. */
				if (bytes.size() - i + j >= 4)
				{
					/* Here we read the 3 following bytes, using an index j
					  j = 0 -> The 0x00 byte: we simply skip everything and go on
					  j = 1,2 -> The 1st and 2nd bytes after 0x00: we store them and go on
					  j = 3 -> The final byte of the pulse length: we store it,
					  and then we pass to finally output the wave */
					if (j > 0)
					{
						over_pulse_bytes[j-1] = byte;
						j += 1;

						if (j >= 4)
						{
							over_pulse_length = ((over_pulse_bytes[2] << 16) | (over_pulse_bytes[1] << 8) | over_pulse_bytes[0]) >> 3;
							byte_samples = tap_data_to_samplecount(over_pulse_length, tap_frequency);
							/* waveamp_high = WAVE_PAUSE; */
							j = 0;
						}
					}
					else
					{
						j += 1;
						LOG_FORMATS("Found a 00 byte close to the end of the .tap file.\n");
						LOG_FORMATS("This is not allowed by the format specs. \n");
						LOG_FORMATS("Check if your .tap file got corrupted when you created it!\n");
					}
				}
				else j = 1;
			}
			/* waveamp_low = WAVE_LOW; */
		}

		if (j == 0)
		{
			cbm_output_wave(samples, wave_data, byte_samples);
//          toggle_wave_data(waveamp_low, waveamp_high);
			toggle_wave_data(wave_data);
			if (version < 2)
			{
				cbm_output_wave(samples, wave_data, byte_samples);
//              toggle_wave_data(waveamp_low, waveamp_high);
				toggle_wave_data(wave_data);
			}
		}
	}

	return cassette_image::error::SUCCESS;
}


static cassette_image::error cbm_cassette_identify(cassette_image *cassette, cassette_image::Options *opts)
{
	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = CBM_WAV_FREQUENCY;
	return cassette_image::error::SUCCESS;
}


static cassette_image::error cbm_cassette_load(cassette_image *cassette)
{
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> bytes(file_size);
	cassette->image_read(&bytes[0], 0, file_size);
	std::vector<int16_t> samples;

	cassette_image::error err = cbm_tap_do_work(samples, bytes);
	if (err != cassette_image::error::SUCCESS)
		return err;

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / CBM_WAV_FREQUENCY, samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


static const cassette_image::Format cbm_tap_cassette_format = {
	"tap",
	cbm_cassette_identify,
	cbm_cassette_load,
	nullptr
};


CASSETTE_FORMATLIST_START(cbm_cassette_formats)
	CASSETTE_FORMAT(cbm_tap_cassette_format)
CASSETTE_FORMATLIST_END
