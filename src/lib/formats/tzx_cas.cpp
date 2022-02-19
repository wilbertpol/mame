// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/*

TZX (currently spectrum only) and spectrum TAP cassette format support by Wilbert Pol

TODO:
    Add support for the remaining block types:
        case 0x15:  Direct Recording
        case 0x18:  CSW Recording
        case 0x21:  Group Start
        case 0x22:  Group End
        case 0x23:  Jump To Block
        case 0x24:  Loop Start
        case 0x25:  Loop End
        case 0x26:  Call Sequence
        case 0x27:  Return From Sequence
        case 0x28:  Select Block
        case 0x2A:  Stop Tape if in 48K Mode
        case 0x2B:  Set signal level
        case 0x5A:  Merge Block
    Add support for the deprecated block types? Only if there is some image which need them:
        case 0x16:  C64 ROM type data block
        case 0x17:  C64 turbo tape data block
        case 0x34:  Emulation info
        case 0x40:  Snapshot block

*/

#include "tzx_cas.h"
#include "imageutl.h"

#include <cmath>


#define TZX_WAV_FREQUENCY   44100
#define WAVE_LOW        -0x5a9e
#define WAVE_HIGH       0x5a9e
#define WAVE_NULL       0

#define SUPPORTED_VERSION_MAJOR 0x01

#define INITIAL_MAX_BLOCK_COUNT 256
#define BLOCK_COUNT_INCREMENTS  256

#define TZX_STANDARD_DATA   0x10
#define TZX_TURBO_DATA      0x11
#define TZX_PURE_TONE       0x12
#define TZX_SEQUENCE        0x13
#define TZX_PURE_DATA       0x14
#define TZX_DIRECT          0x15
#define TZX_CSW             0x18
#define TZX_GENERALIZED     0x19
#define TZX_PAUSE           0x20
#define TZX_GROUP_START     0x21
#define TZX_GROUP_END       0x22
#define TZX_JUMP            0x23
#define TZX_LOOP_START      0x24
#define TZX_LOOP_END        0x25
#define TZX_CALL_SEQUENCE   0x26
#define TZX_RETURN_SEQUENCE 0x27
#define TZX_SELECT          0x28
#define TZX_STOP_48K        0x2a
#define TZX_SET_LEVEL       0x2b
#define TZX_DESCRIPTION     0x30
#define TZX_MESSAGE         0x31
#define TZX_ARCHIVE_INFO    0x32
#define TZX_HARDWARE_TYPE   0x33
#define TZX_CUSTOM_INFO     0x35
#define TZX_GLUE            0x5a

static const uint8_t TZX_HEADER[8] = { 'Z','X','T','a','p','e','!',0x1a };


static void toggle_wave_data(int16_t &wave_data)
{
	if (wave_data == WAVE_LOW)
	{
		wave_data = WAVE_HIGH;
	}
	else
	{
		wave_data = WAVE_LOW;
	}
}


static inline int millisec_to_samplecount(int millisec)
{
	return (int) (millisec * ((double)TZX_WAV_FREQUENCY / 1000.0));
}


static inline int tcycles_to_samplecount(int tcycles, float t_scale)
{
	return (int) (0.5 + ((((double)TZX_WAV_FREQUENCY * tcycles * t_scale) / 3500000)));
}


static void tzx_output_wave(std::vector<int16_t> &samples, int16_t wave_data, int length)
{
	for ( ; length > 0; length--)
	{
		samples.push_back(wave_data);
	}
}


static void pause_one_millisec(std::vector<int16_t> &samples, int16_t wave_data)
{
	int pause_samples = millisec_to_samplecount(1);
	tzx_output_wave(samples, wave_data, pause_samples);
}


static void tzx_cas_handle_block(std::vector<int16_t> &samples, int16_t &wave_data, const uint8_t *bytes, int pause, int data_size, int pilot, int pilot_length, int sync1, int sync2, int bit0, int bit1, int bits_in_last_byte, float t_scale)
{
	int pilot_samples = tcycles_to_samplecount(pilot, t_scale);
	int sync1_samples = tcycles_to_samplecount(sync1, t_scale);
	int sync2_samples = tcycles_to_samplecount(sync2, t_scale);
	int bit0_samples = tcycles_to_samplecount(bit0, t_scale);
	int bit1_samples = tcycles_to_samplecount(bit1, t_scale);
	int data_index;

	/* Uncomment this to include into error.log a fully detailed analysis of each block */
//  LOG_FORMATS("tzx_cas_block_size: pilot_length = %d, pilot_samples = %d, sync1_samples = %d, sync2_samples = %d, bit0_samples = %d, bit1_samples = %d\n", pilot_length, pilot_samples, sync1_samples, sync2_samples, bit0_samples, bit1_samples);

	// PILOT
	for ( ; pilot_length > 0; pilot_length--)
	{
		tzx_output_wave(samples, wave_data, pilot_samples);
		toggle_wave_data(wave_data);
	}
	// SYNC1
	if (sync1_samples > 0)
	{
		tzx_output_wave(samples, wave_data, sync1_samples);
		toggle_wave_data(wave_data);
	}
	// SYNC2
	if (sync2_samples > 0)
	{
		tzx_output_wave(samples, wave_data, sync2_samples);
		toggle_wave_data(wave_data);
	}
	// data
	for (data_index = 0; data_index < data_size; data_index++)
	{
		uint8_t byte = bytes[data_index];
		int bits_to_go = (data_index == (data_size - 1)) ? bits_in_last_byte : 8;

		for ( ; bits_to_go > 0; byte <<= 1, bits_to_go--)
		{
			int bit_samples = (byte & 0x80) ? bit1_samples : bit0_samples;
			tzx_output_wave(samples, wave_data, bit_samples);
			toggle_wave_data(wave_data);
			tzx_output_wave(samples, wave_data, bit_samples);
			toggle_wave_data(wave_data);
		}
	}
	// pause
	if (pause > 0)
	{
		pause_one_millisec(samples, wave_data);

		int rest_pause_samples = millisec_to_samplecount(pause - 1);

		wave_data = WAVE_LOW;
		tzx_output_wave(samples, wave_data, rest_pause_samples);
	}
}


static void tzx_handle_direct(std::vector<int16_t> &samples, int16_t &wave_data, const uint8_t *bytes, int pause, int data_size, int tstates, int bits_in_last_byte, float t_scale)
{
	int samplecount = tcycles_to_samplecount(tstates, t_scale);

	// data
	for (int data_index = 0; data_index < data_size; data_index++)
	{
		uint8_t byte = bytes[data_index];
		int bits_to_go = (data_index == (data_size - 1)) ? bits_in_last_byte : 8;

		for ( ; bits_to_go > 0; byte <<= 1, bits_to_go--)
		{
			if (byte & 0x80) wave_data = WAVE_HIGH;
			else wave_data = WAVE_LOW;

			tzx_output_wave(samples, wave_data, samplecount);
		}
	}

	// pause
	if (pause > 0)
	{
		pause_one_millisec(samples, wave_data);

		int rest_pause_samples = millisec_to_samplecount(pause - 1);

		wave_data = WAVE_LOW;
		tzx_output_wave(samples, wave_data, rest_pause_samples);
	}
}


static inline void tzx_handle_symbol(std::vector<int16_t> &samples, int16_t &wave_data, const uint8_t *symtable, uint8_t symbol, int maxp, float t_scale)
{
	const uint8_t *cursymb = symtable + (2 * maxp + 1)*symbol;

	uint8_t starttype = cursymb[0];

	switch (starttype)
	{
	case 0x00:
		// pulse level has already been toggled so don't change
		break;

	case 0x01:
		// pulse level has already been toggled so revert
		toggle_wave_data(wave_data);
		break;

	case 0x02:
		// force low
		wave_data = WAVE_LOW;
		break;

	case 0x03:
		// force high
		wave_data = WAVE_HIGH;
		break;

	default:
		printf("SYMDEF invalid - bad starting polarity");
	}

	for (int i = 0; i < maxp; i++)
	{
		uint16_t pulse_length = cursymb[1 + (i*2)] | (cursymb[2 + (i*2)] << 8);

		// shorter lists can be terminated with a pulse_length of 0
		if (pulse_length != 0)
		{
			int samplecount = tcycles_to_samplecount(pulse_length, t_scale);
			tzx_output_wave(samples, wave_data, samplecount);
			toggle_wave_data(wave_data);
		}
		else
		{
			break;
		}
	}
}


static inline int stream_get_bit(const uint8_t *bytes, uint8_t &stream_bit, uint32_t &stream_byte)
{
	// get bit here
	uint8_t retbit = 0;

	uint8_t byte = bytes[stream_byte];
	byte = byte << stream_bit;

	if (byte & 0x80) retbit = 1;

	stream_bit++;

	if (stream_bit == 8)
	{
		stream_bit = 0;
		stream_byte++;
	}

	return retbit;
}


static void tzx_handle_generalized(std::vector<int16_t> &samples, int16_t &wave_data, const uint8_t *bytes, int pause, int data_size, uint32_t totp, int npp, int asp, uint32_t totd, int npd, int asd, float t_scale)
{
	if (totp > 0)
	{
	//  printf("pilot block table %04x\n", totp);

		const uint8_t *symtable = bytes;
		const uint8_t *table2 = symtable + (2 * npp + 1)*asp;

		// the Pilot and sync data stream has an RLE encoding
		for (int i = 0; i < totp*3; i+=3)
		{
			uint8_t symbol = table2[i + 0];
			uint16_t repetitions = table2[i + 1] + (table2[i + 2] << 8);
			//printf("new symbol %02x repetitions %04x\n", symbol, repetitions); // does 1 mean repeat once, or that it only occurs once?

			for (int j = 0; j < repetitions; j++)
			{
				tzx_handle_symbol(samples, wave_data, symtable, symbol, npp, t_scale);
			}
		}

		// advance to after this data
		bytes += ((2 * npp + 1)*asp) + totp * 3;
	}

	if (totd > 0)
	{
		//printf("new data block table %04x (has %0d symbols, max symbol length is %d)\n", totd, asd, npd);

		const uint8_t *symtable = bytes;
		const uint8_t *table2 = bytes + (2 * npd + 1)*asd;

		int NB = std::ceil(compute_log2(asd)); // number of bits needed to represent each symbol

		uint8_t stream_bit = 0;
		uint32_t stream_byte = 0;

		for (int i = 0; i < totd; i++)
		{
			uint8_t symbol = 0;

			for (int j = 0; j < NB; j++)
			{
				symbol |= stream_get_bit(table2, stream_bit, stream_byte) << j;
			}

			tzx_handle_symbol(samples, wave_data, symtable, symbol, npd, t_scale);
		}
	}

	/* pause */
	if (pause > 0)
	{
		pause_one_millisec(samples, wave_data);

		int rest_pause_samples = millisec_to_samplecount(pause - 1);

		wave_data = WAVE_LOW;
		tzx_output_wave(samples, wave_data, rest_pause_samples);
	}
}


static void ascii_block_common_log(const char *block_type_string, uint8_t block_type)
{
	LOG_FORMATS("%s (type %02x) encountered:\n", block_type_string, block_type);
}


static const char *const archive_ident[] =
{
	"Full title",
	"Software house/publisher",
	"Author(s)",
	"Year of publication",
	"Language",
	"Game/utility type",
	"Price",
	"Protection scheme/loader",
	"Origin",
};


static const char *const hw_info[] =
{
	"Tape runs on this machine / this hardware",
	"Tape needs this machine / this hardware",
	"Tape runs on this machine / this hardware, but does not require its special features",
	"Tape does not run on this machine / this hardware",
};


static uint16_t tzx_read16(std::vector<uint8_t> &bytes, size_t pos)
{
	return bytes[pos] + (bytes[pos + 1] << 8);
}


static uint32_t tzx_read24(std::vector<uint8_t> &bytes, size_t pos)
{
	return bytes[pos] + (bytes[pos + 1] << 8) + (bytes[pos + 2] << 16);
}


static uint32_t tzx_read32(std::vector<uint8_t> &bytes, size_t pos)
{
	return bytes[pos] + (bytes[pos + 1] << 8) + (bytes[pos + 2] << 16) + (bytes[pos + 3] << 24);
}


static cassette_image::error tzx_find_blocks(std::vector<size_t> blocks, std::vector<uint8_t> &bytes)
{
	const size_t bytes_length = bytes.size();
	size_t pos = sizeof(TZX_HEADER) + 2;

	while (pos < bytes_length)
	{
		uint8_t blocktype = bytes[pos];

		blocks.push_back(pos);
		pos += 1;

		switch (blocktype)
		{
		case TZX_STANDARD_DATA:  // Standard Speed Data Block (.TAP block)
			if (pos + 4 > bytes_length)
				return cassette_image::error::INVALID_IMAGE;
			pos += 2;   // skip pause time
			pos += 2 + tzx_read16(bytes, pos);
			break;
		case TZX_TURBO_DATA:  // Turbo Loading Data Block
			if (pos + 0x12 > bytes_length)
				return cassette_image::error::INVALID_IMAGE;
			pos += 0x0f;
			pos += 3 + tzx_read24(bytes, pos);
			break;
		case TZX_PURE_TONE:
			pos += 4;
			break;
		case TZX_SEQUENCE:
			if (pos + 1 > bytes_length)
				return cassette_image::error::INVALID_IMAGE;
			pos += 1 + 2 * bytes[pos];
			break;
		case TZX_PURE_DATA:
			if (pos + 10 > bytes_length)
				return cassette_image::error::INVALID_IMAGE;
			pos += 7;
			pos += 3 + tzx_read24(bytes, pos);
			break;
		case TZX_DIRECT:
			if (pos + 8 > bytes_length)
				return cassette_image::error::INVALID_IMAGE;
			pos += 5;
			pos += 3 + tzx_read24(bytes, pos);
			break;
		case TZX_PAUSE:
		case TZX_JUMP:
		case TZX_LOOP_START:
			pos += 2;
			break;
		case TZX_GROUP_START:
		case TZX_DESCRIPTION:
			if (pos + 1 > bytes_length)
				return cassette_image::error::INVALID_IMAGE;
			pos += 1 + bytes[pos];
			break;
		case TZX_GROUP_END:
		case TZX_LOOP_END:
		case TZX_RETURN_SEQUENCE:
			break;
		case TZX_CALL_SEQUENCE:
			if (pos + 2 > bytes_length)
				return cassette_image::error::INVALID_IMAGE;
			pos += 2 + 2 * tzx_read16(bytes, pos);
			break;
		case TZX_SELECT:
		case TZX_ARCHIVE_INFO:
			if (pos + 2 > bytes_length)
				return cassette_image::error::INVALID_IMAGE;
			pos += 2 + tzx_read16(bytes, pos);
			break;
		case TZX_MESSAGE:
			if (pos + 2 > bytes_length)
				return cassette_image::error::INVALID_IMAGE;
			pos += 1;
			pos += 1 + bytes[pos];
			break;
		case TZX_HARDWARE_TYPE:
			if (pos + 1 > bytes_length)
				return cassette_image::error::INVALID_IMAGE;
			pos += 1 + 3 * bytes[pos];
			break;
		case TZX_CUSTOM_INFO:
			if (pos + 0x14 > bytes_length)
				return cassette_image::error::INVALID_IMAGE;
			pos += 0x10;
			pos += 4 + tzx_read32(bytes, pos);
			break;
		case TZX_GLUE:
			pos += 9;
			break;
		case TZX_CSW:
		case TZX_GENERALIZED:
		case TZX_STOP_48K:
		case TZX_SET_LEVEL:
			if (pos + 4 > bytes_length)
				return cassette_image::error::INVALID_IMAGE;
			pos += 4 + tzx_read32(bytes, pos);
			break;

		// Deprecated types
		case 0x34:
			pos += 8;
			break;
		case 0x40:
			if (pos + 4 > bytes_length)
				return cassette_image::error::INVALID_IMAGE;
			pos += 1;
			pos += 3 + tzx_read24(bytes, pos);
			break;

		default:
			if (pos + 4 > bytes_length)
				return cassette_image::error::INVALID_IMAGE;
			pos += 4 + tzx_read32(bytes, pos);
			break;
		}

		if (pos > bytes_length)
			return cassette_image::error::INVALID_IMAGE;
	}
	return cassette_image::error::SUCCESS;
}


static cassette_image::error tzx_cas_do_work(std::vector<int16_t> &samples, float t_scale, std::vector<uint8_t> &bytes)
{
	size_t pos = sizeof(TZX_HEADER) + 2;
	int current_block = 0;
	int16_t wave_data = WAVE_LOW;
	std::vector<size_t> blocks;

	int loopcount = 0, loopoffset = 0;

	cassette_image::error err = tzx_find_blocks(blocks, bytes);
	if (err != cassette_image::error::SUCCESS) return err;

	while (pos < bytes.size())
	{
		int pause_time;
		uint32_t data_size;
		int text_size, total_size, i;
		int pilot, pilot_length, sync1, sync2;
		int bit0, bit1, bits_in_last_byte;
		uint16_t tstates = 0;

		if (current_block > blocks.size())
		{
			// This should be impossible
			printf("TODO current_block > blocks.size()\n");
			return cassette_image::error::INVALID_IMAGE;
		}
		if (current_block == blocks.size())
		{
			blocks.push_back(pos);
		}

		pos = blocks[current_block];
		uint8_t block_type = bytes[pos++];
		LOG_FORMATS("tzx_cas_fill_wave: block %d, block_type %02x\n", current_block, block_type);
		printf("new current_block = %d, block_type = %02x, size = %lu\n", current_block, block_type, samples.size());

		switch (block_type)
		{
		case TZX_STANDARD_DATA:  // Standard Speed Data Block (.TAP block)
			pause_time = bytes[pos] + (bytes[pos + 1] << 8);
			data_size = bytes[pos + 2] + (bytes[pos + 3] << 8);
			pos += 4;
			pilot_length = (bytes[pos] < 128) ?  8063 : 3223;
			tzx_cas_handle_block(samples, wave_data, &bytes[pos], pause_time, data_size, 2168, pilot_length, 667, 735, 855, 1710, 8, t_scale);
			pos += data_size;
			current_block++;
			break;
		case TZX_TURBO_DATA:  // Turbo Loading Data Block
			pilot = bytes[pos] + (bytes[pos + 1] << 8);
			sync1 = bytes[pos + 2] + (bytes[pos + 3] << 8);
			sync2 = bytes[pos + 4] + (bytes[pos + 5] << 8);
			bit0 = bytes[pos + 6] + (bytes[pos + 7] << 8);
			bit1 = bytes[pos + 8] + (bytes[pos + 9] << 8);
			pilot_length = bytes[pos + 10] + (bytes[pos + 11] << 8);
			bits_in_last_byte = bytes[pos + 12];
			pause_time = bytes[pos + 13] + (bytes[pos + 14] << 8);
			data_size = bytes[pos + 15] + (bytes[pos + 16] << 8) + (bytes[pos + 17] << 16);
			tzx_cas_handle_block(samples, wave_data, &bytes[pos + 18], pause_time, data_size, pilot, pilot_length, sync1, sync2, bit0, bit1, bits_in_last_byte, t_scale);
			pos += 18 + data_size;
			current_block++;
			break;
		case TZX_PURE_TONE:  // Pure Tone
			pilot = bytes[pos] + (bytes[pos + 1] << 8);
			pilot_length = bytes[pos + 2] + (bytes[pos + 3] << 8);
			tzx_cas_handle_block(samples, wave_data, nullptr, 0, 0, pilot, pilot_length, 0, 0, 0, 0, 0, t_scale);
			pos += 4;
			current_block++;
			break;
		case TZX_SEQUENCE:  // Sequence of Pulses of Different Lengths
			{
				int pulses = bytes[pos];
				pos++;
				for (data_size = 0; data_size < pulses; data_size++)
				{
					pilot = bytes[pos] + (bytes[pos + 1] << 8);
					pos += 2;
					tzx_cas_handle_block(samples, wave_data, nullptr, 0, 0, pilot, 1, 0, 0, 0, 0, 0, t_scale);
				}
				current_block++;
			}
			break;
		case TZX_PURE_DATA:  // Pure Data Block
			bit0 = bytes[pos] + (bytes[pos + 1] << 8);
			bit1 = bytes[pos + 2] + (bytes[pos + 3] << 8);
			bits_in_last_byte = bytes[pos + 4];
			pause_time = bytes[pos + 5] + (bytes[pos + 6] << 8);
			data_size = bytes[pos + 7] + (bytes[pos + 8] << 8) + (bytes[pos + 9] << 16);
			pos += 10;
			tzx_cas_handle_block(samples, wave_data, &bytes[pos], pause_time, data_size, 0, 0, 0, 0, bit0, bit1, bits_in_last_byte, t_scale);
			pos += data_size;
			current_block++;
			break;

		case TZX_DIRECT:  // Direct Recording
			// used on 'bombscar' in the cpc_cass list
			// having this missing is fatal
			tstates = bytes[pos] + (bytes[pos + 1] << 8);
			pause_time= bytes[pos + 2] + (bytes[pos + 3] << 8);
			bits_in_last_byte = bytes[pos + 4];
			data_size = bytes[pos + 5] + (bytes[pos + 6] << 8) + (bytes[pos + 7] << 16);
			tzx_handle_direct(samples, wave_data, &bytes[pos + 8], pause_time, data_size, tstates, bits_in_last_byte, t_scale);
			pos += 8 + data_size;
			current_block++;
			break;

		case TZX_CSW:  // CSW Recording
			// having this missing is fatal
			printf("Unsupported block type (0x15 - CSW Recording) encountered.\n");
			data_size = bytes[pos] + (bytes[pos + 1] << 8) + (bytes[pos + 2] << 16) + (bytes[pos + 3] << 24);
			pos += 4 + data_size;
			current_block++;
			break;

		case TZX_GENERALIZED:  // Generalized Data Block
			{
				// having this missing is fatal
				// used crudely by batmanc in spectrum_cass list (which is just a redundant encoding of batmane ?)
				data_size = bytes[pos] + (bytes[pos + 1] << 8) + (bytes[pos + 2] << 16) + (bytes[pos + 3] << 24);
				pause_time= bytes[pos + 4] + (bytes[pos + 5] << 8);

				uint32_t totp = bytes[pos + 6] + (bytes[pos + 7] << 8) + (bytes[pos + 8] << 16) + (bytes[pos + 9] << 24);
				int npp = bytes[pos + 10];
				int asp = bytes[pos + 11];
				if (asp == 0 && totp > 0) asp = 256;

				uint32_t totd = bytes[pos + 12] + (bytes[pos + 13] << 8) + (bytes[pos + 14] << 16) + (bytes[pos + 15] << 24);
				int npd = bytes[pos + 16];
				int asd = bytes[pos + 17];
				if (asd == 0 && totd > 0) asd = 256;

				tzx_handle_generalized(samples, wave_data, &bytes[pos + 18], pause_time, data_size, totp, npp, asp, totd, npd, asd, t_scale);

				pos += 4 + data_size;
				current_block++;
			}
			break;

		case TZX_PAUSE:  // Pause (Silence) or 'Stop the Tape' Command
			pause_time = bytes[pos] + (bytes[pos + 1] << 8);
			if (pause_time == 0)
			{
				/* pause = 0 is used to let an emulator automagically stop the tape
				   in MAME we do not do that, so we insert a 5 second pause. */
				pause_time = 5000;
			}
			tzx_cas_handle_block(samples, wave_data, nullptr, pause_time, 0, 0, 0, 0, 0, 0, 0, 0, t_scale);
			pos += 2;
			current_block++;
			break;
		case TZX_DESCRIPTION:  // Text Description
			ascii_block_common_log("Text Description Block", block_type);
			for (data_size = 0; data_size < bytes[pos]; data_size++)
				LOG_FORMATS("%c", bytes[pos + 1 + data_size]);
			LOG_FORMATS("\n");
			pos += 1 + bytes[pos];
			current_block++;
			break;
		case TZX_MESSAGE:  // Message Block
			ascii_block_common_log("Message Block", block_type);
			LOG_FORMATS("Expected duration of the message display: %02x\n", bytes[pos]);
			LOG_FORMATS("Message: \n");
			for (data_size = 0; data_size < bytes[pos + 1]; data_size++)
			{
				LOG_FORMATS("%c", bytes[pos + 2 + data_size]);
				if (bytes[pos + 2 + data_size] == 0x0d)
					LOG_FORMATS("\n");
			}
			LOG_FORMATS("\n");
			pos += 2 + bytes[pos + 1];
			current_block++;
			break;
		case TZX_ARCHIVE_INFO:  // Archive Info
			ascii_block_common_log("Archive Info Block", block_type);
			total_size = bytes[pos] + (bytes[pos + 1] << 8);
			text_size = 0;
			for (data_size = 0; data_size < bytes[pos + 2]; data_size++)  // data_size = number of text blocks, in this case
			{
				if (bytes[pos + 3 + text_size] < 0x09) {
					LOG_FORMATS("%s: \n", archive_ident[bytes[pos + 3 + text_size]]);
				}
				else {
					LOG_FORMATS("Comment(s): \n");
				}

				for (i = 0; i < bytes[pos + 3 + text_size + 1]; i++)
				{
					LOG_FORMATS("%c", bytes[pos + 3 + text_size + 2 + i]);
				}
				text_size += 2 + i;
			}
			LOG_FORMATS("\n");
			if (text_size != total_size)
				LOG_FORMATS("Malformed Archive Info Block (Text length different from the declared one).\n Please verify your tape image.\n");
			pos += 2 + total_size;
			current_block++;
			break;
		case TZX_HARDWARE_TYPE:  // Hardware Type
			ascii_block_common_log("Hardware Type Block", block_type);
			for (data_size = 0; data_size < bytes[pos]; data_size++)  // data_size = number of hardware blocks, in this case
			{
				LOG_FORMATS("Hardware Type %02x - Hardware ID %02x - ", bytes[pos + 1 + data_size * 3], bytes[pos + 1 + data_size * 3 + 1]);
				LOG_FORMATS("%s \n ", hw_info[bytes[pos + 1 + data_size * 3 + 2]]);
			}
			pos += 1 + bytes[pos] * 3;
			current_block++;
			break;
		case TZX_CUSTOM_INFO:  // Custom Info Block
			ascii_block_common_log("Custom Info Block", block_type);
			for (data_size = 0; data_size < 16; data_size++)
			{
				LOG_FORMATS("%c", bytes[pos + data_size]);
			}
			LOG_FORMATS(":\n");
			text_size = bytes[pos + 16] + (bytes[pos + 17] << 8) + (bytes[pos + 18] << 16) + (bytes[pos + 19] << 24);
			for (data_size = 0; data_size < text_size; data_size++)
				LOG_FORMATS("%c", bytes[pos + 20 + data_size]);
			LOG_FORMATS("\n");
			pos += 20 + text_size;
			current_block++;
			break;
		case TZX_GLUE:  // "Glue" Block
			LOG_FORMATS("Glue Block (type %02x) encountered.\n", block_type);
			LOG_FORMATS("Please use a .tzx handling utility to split the merged tape files.\n");
			pos += 9;
			current_block++;
			break;
		case TZX_LOOP_START:  // Loop Start
			loopcount = bytes[pos] + (bytes[pos + 1] << 8);
			pos += 2;
			current_block++;
			loopoffset = current_block;

			LOG_FORMATS("loop start %d %d\n",  loopcount, current_block);
			break;
		case TZX_LOOP_END:  // Loop End
			if (loopcount > 0)
			{
				current_block = loopoffset;
				loopcount--;
				LOG_FORMATS("do loop\n");
			}
			else
			{
				current_block++;
			}
			break;

		case TZX_GROUP_START:  // Group Start
			LOG_FORMATS("Unsupported block type (%02x) encountered.\n", block_type);
			data_size = bytes[pos];
			pos += 1 + data_size;
			current_block++;
			break;
		case TZX_GROUP_END:  // Group End
			LOG_FORMATS("Unsupported block type (%02x) encountered.\n", block_type);
			current_block++;
			break;
		case TZX_JUMP:  // Jump To Block
			LOG_FORMATS("Unsupported block type (%02x) encountered.\n", block_type);
			pos += 2;
			current_block++;
			break;
		case TZX_CALL_SEQUENCE:  // Call Sequence
			LOG_FORMATS("Unsupported block type (%02x) encountered.\n", block_type);
			data_size = bytes[pos] + (bytes[pos + 1] << 8);
			pos += 2 + 2 * data_size;
			current_block++;
			break;
		case TZX_RETURN_SEQUENCE:  // Return From Sequence
			LOG_FORMATS("Unsupported block type (%02x) encountered.\n", block_type);
			current_block++;
			break;
		case TZX_SELECT:  // Select Block
			LOG_FORMATS("Unsupported block type (%02x) encountered.\n", block_type);
			data_size = bytes[pos] + (bytes[pos + 1] << 8);
			pos += 2 + data_size;
			current_block++;
			break;
		case TZX_STOP_48K:  // Stop Tape if in 48K Mode
			LOG_FORMATS("Unsupported block type (%02x) encountered.\n", block_type);
			data_size = bytes[pos] + (bytes[pos + 1] << 8) + (bytes[pos + 2] << 16) + (bytes[pos + 3] << 24);
			pos += 4 + data_size;
			current_block++;
			break;
		case TZX_SET_LEVEL:  // Set signal level
			LOG_FORMATS("Unsupported block type (%02x) encountered.\n", block_type);
			data_size = bytes[pos] + (bytes[pos + 1] << 8) + (bytes[pos + 2] << 16) + (bytes[pos + 3] << 24);
			pos += 4 + data_size;
			current_block++;
			break;

		// Deprecated types
		case 0x16:  // C64 ROM Type Data Block, deprecated in TZX 1.20
			LOG_FORMATS("Deprecated block type (%02x) encountered.\n", block_type);
			LOG_FORMATS("Please look for an updated .tzx file.\n");
			data_size = bytes[pos] + (bytes[pos + 1] << 8) + (bytes[pos + 2] << 16) + (bytes[pos + 3] << 24);
			pos += 4 + data_size;
			current_block++;
			break;
		case 0x17:  // C64 Turbo Tape Data Block, deprecated in TZX 1.20
			LOG_FORMATS("Deprecated block type (%02x) encountered.\n", block_type);
			LOG_FORMATS("Please look for an updated .tzx file.\n");
			data_size = bytes[pos] + (bytes[pos + 1] << 8) + (bytes[pos + 2] << 16) + (bytes[pos + 3] << 24);
			pos += 4 + data_size;
			current_block++;
			break;
		case 0x34:  // Emulation Info, deprecated in TZX 1.20
			LOG_FORMATS("Deprecated block type (%02x) encountered.\n", block_type);
			LOG_FORMATS("Please look for an updated .tzx file.\n");
			pos += 8;
			current_block++;
			break;
		case 0x40:  // Snapshot Block, deprecated in TZX 1.20
			LOG_FORMATS("Deprecated block type (%02x) encountered.\n", block_type);
			LOG_FORMATS("Please look for an updated .tzx file.\n");
			pos += 1;
			data_size = bytes[pos] + (bytes[pos + 1] << 8) + (bytes[pos + 2] << 16);
			pos += 3 + data_size;
			current_block++;
			break;

		default:
			LOG_FORMATS("Unsupported block type (%02x) encountered.\n", block_type);
			data_size = bytes[pos] + (bytes[pos + 1] << 8) + (bytes[pos + 2] << 16) + (bytes[pos + 3] << 24);
			pos += 4 + data_size;
			current_block++;
			return cassette_image::error::UNSUPPORTED;
			break;
		}
	}
	// Adding 1 ms. pause to ensure that the last edge is properly finished at the end of tape
	pause_one_millisec(samples, wave_data);

	return cassette_image::error::SUCCESS;
}


static cassette_image::error tzx_cassette_identify( cassette_image *cassette, cassette_image::Options *opts )
{
	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = TZX_WAV_FREQUENCY;
	return cassette_image::error::SUCCESS;
}


static cassette_image::error tap_cassette_identify( cassette_image *cassette, cassette_image::Options *opts )
{
	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = TZX_WAV_FREQUENCY;
	return cassette_image::error::SUCCESS;
}


static cassette_image::error cdt_cassette_identify( cassette_image *cassette, cassette_image::Options *opts )
{
	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = TZX_WAV_FREQUENCY;
	return cassette_image::error::SUCCESS;
}


static cassette_image::error tzx_cassette_load(cassette_image *cassette)
{
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> bytes(file_size);
	cassette->image_read(&bytes[0], 0, file_size);
	std::vector<int16_t> samples;

	// Header size plus major and minor version number
	if (file_size < 10)
	{
		LOG_FORMATS("tzx_cas_to_wav_size: cassette image too small\n");
		return cassette_image::error::INVALID_IMAGE;
	}

	// Check for correct header
	if (memcmp(&bytes[0], TZX_HEADER, sizeof(TZX_HEADER)))
	{
		LOG_FORMATS("tzx_cas_to_wav_size: cassette image has incompatible header\n");
		return cassette_image::error::INVALID_IMAGE;
	}

	// Check major version number in header */
	if (bytes[0x08] > SUPPORTED_VERSION_MAJOR)
	{
		LOG_FORMATS("tzx_cas_to_wav_size: unsupported version\n");
		return cassette_image::error::INVALID_IMAGE;
	}

	cassette_image::error err = tzx_cas_do_work(samples, 1.0f, bytes);
	if (err != cassette_image::error::SUCCESS) return err;

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / TZX_WAV_FREQUENCY, samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


static cassette_image::error tap_cassette_load(cassette_image *cassette)
{
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> bytes(file_size);
	cassette->image_read(&bytes[0], 0, file_size);
	std::vector<int16_t> samples;

	int pos = 0;
	int16_t wave_data = 0;
	while (pos < file_size)
	{
		int data_size = bytes[pos] + (bytes[pos + 1] << 8);
		int pilot_length = (bytes[pos + 2] == 0x00) ? 8063 : 3223;
		LOG_FORMATS("tap_cas_fill_wave: Handling TAP block containing 0x%X bytes\n", data_size);
		pos += 2;
		tzx_cas_handle_block(samples, wave_data, &bytes[pos], 1000, data_size, 2168, pilot_length, 667, 735, 855, 1710, 8, 1.0f);
		pos += data_size;
	}

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / TZX_WAV_FREQUENCY, samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


static cassette_image::error cdt_cassette_load( cassette_image *cassette )
{
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> bytes(file_size);
	cassette->image_read(&bytes[0], 0, file_size);
	std::vector<int16_t> samples;

	// Header size plus major and minor version number
	if (file_size < 10)
	{
		LOG_FORMATS("tzx_cas_to_wav_size: cassette image too small\n");
		return cassette_image::error::INVALID_IMAGE;
	}

	// Check for correct header
	if (memcmp(&bytes[0], TZX_HEADER, sizeof(TZX_HEADER)))
	{
		LOG_FORMATS("tzx_cas_to_wav_size: cassette image has incompatible header\n");
		return cassette_image::error::INVALID_IMAGE;
	}

	// Check major version number in header */
	if (bytes[0x08] > SUPPORTED_VERSION_MAJOR)
	{
		LOG_FORMATS("tzx_cas_to_wav_size: unsupported version\n");
		return cassette_image::error::INVALID_IMAGE;
	}

	tzx_cas_do_work(samples, (float)40 / 35, bytes); /* scale to 4MHz */

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / TZX_WAV_FREQUENCY, samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
}


const cassette_image::Format tzx_cassette_format =
{
	"tzx",
	tzx_cassette_identify,
	tzx_cassette_load,
	nullptr
};


static const cassette_image::Format tap_cassette_format =
{
	"tap,blk",
	tap_cassette_identify,
	tap_cassette_load,
	nullptr
};


static const cassette_image::Format cdt_cassette_format =
{
	"cdt",
	cdt_cassette_identify,
	cdt_cassette_load,
	nullptr
};


CASSETTE_FORMATLIST_START(tzx_cassette_formats)
	CASSETTE_FORMAT(tzx_cassette_format)
	CASSETTE_FORMAT(tap_cassette_format)
CASSETTE_FORMATLIST_END


CASSETTE_FORMATLIST_START(cdt_cassette_formats)
	CASSETTE_FORMAT(cdt_cassette_format)
CASSETTE_FORMATLIST_END
