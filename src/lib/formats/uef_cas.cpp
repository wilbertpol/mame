// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/*

The UEF file format is designed to store accurate images of the common media types associated
with the BBC Micro, Acorn Electron and Atom. Tape storage is compatible with the CUTS/BYTE/Kansas City
Format, and hence the format is also capable of storing software for non-Acorn systems such as the
Altair 8800, PT SOL-20, Ohio Scientific, Compukit UK101, Nascom 1/2/3, Motorola MEK D1 6800 and
SWTPC 6800 kit based computers.

UEF files are chunk based and optionally compressed.

*/
#include "uef_cas.h"
#include "imageutl.h"

#include <zlib.h>

#include <cmath>
#include <cstring>


#define UEF_BASE_FREQUENCY  1200
#define UEF_WAV_FREQUENCY   4800
#define INITAL_PHASE        180
#define WAVE_LOW    -32768
#define WAVE_HIGH   32767
#define WAVE_NULL   0

static const uint8_t UEF_HEADER[10] = { 0x55, 0x45, 0x46, 0x20, 0x46, 0x69, 0x6c, 0x65, 0x21, 0x00 };
static const uint8_t GZ_HEADER[2] = { 0x1f, 0x8b };

/*
    bytes are stored as
    start bit   1 * 0
    data bits   8 * X
    stop bit    1 * 1
*/

// gzip flag byte
#define ASCII_FLAG  0x01 // bit 0 set: file probably ascii text
#define HEAD_CRC    0x02 // bit 1 set: header CRC present
#define EXTRA_FIELD 0x04 // bit 2 set: extra field present
#define ORIG_NAME   0x08 // bit 3 set: original file name present
#define COMMENT     0x10 // bit 4 set: file comment present
#define RESERVED    0xe0 // bits 5..7: reserved


static const cassette_image::Modulation uef_cas_modulation =
{
	cassette_image::MODULATION_SINEWAVE,
	1200.0 - 300, 1200.0, 1200.0 + 300,
	2400.0 - 600, 2400.0, 2400.0 + 600
};


static const uint8_t* skip_gz_header(const uint8_t *p, int length)
{
	const uint8_t *max_p = p + length;
	if (p + 10 >= max_p)
		return nullptr;
	// ignore initial 1f 8b header
	// get method and flags
	uint8_t method = p[2];
	uint8_t flags = p[3];
	if (method != Z_DEFLATED || (flags & RESERVED) != 0)
	{
		return nullptr;
	}
	// Skip initial 1f 8b header, method, flags, time, xflags and OS code
	p += 10;

	// Skip the extra field
	if ((flags & EXTRA_FIELD) != 0)
	{
		int len = (p[1] << 8) | p[0];
		p += 2 + len;
	}
	if (p >= max_p)
		return nullptr;
	// Skip the original file name
	if ((flags & ORIG_NAME) != 0)
	{
		for ( ; *p && p < max_p; p++);
		p++;
	}
	// Skip the .gz file comment
	if ((flags & COMMENT) != 0)
	{
		for( ; *p && p < max_p; p++);
	}
	// Skip the header crc
	if ((flags & HEAD_CRC) != 0)
	{
		p += 2;
	}
	if (p >= max_p)
		return nullptr;
	return p;
}

static float get_uef_float(const uint8_t *bytes)
{
	float result;

	// decode mantissa
	int mantissa = bytes[0] | (bytes[1] << 8) | ((bytes[2] & 0x7f) | 0x80) << 16;

	result = (float)mantissa;
	result = (float)ldexp(result, -23);

	// decode exponent
	int exponent = ((bytes[2] & 0x80) >> 7) | (bytes[3] & 0x7f) << 1;
	exponent -= 127;
	result = (float)ldexp(result, exponent);

	// flip sign if necessary
	if (bytes[3] & 0x80)
		result = -result;

	return result;
}


static cassette_image::error uef_cas_fill_bit(uint8_t loops, cassette_image *cassette, double &time_index, bool bit, cassette_image::Modulation &modulation, uint16_t phase)
{
	for (uint8_t i = 0; i < loops * (bit ? 2 : 1); i++)
	{
		double time_displacement;
		cassette_image::error err = cassette->put_modulated_data_bit(0, time_index, bit, modulation, &time_displacement, phase);
		if (err != cassette_image::error::SUCCESS) return err;
		time_index += time_displacement;
	}
	return cassette_image::error::SUCCESS;
}


static cassette_image::error uef_cas_fill_short_wave(cassette_image *cassette, double &time_index, cassette_image::Modulation &modulation, uint16_t phase)
{
	double time_displacement;
	cassette_image::error err = cassette->put_modulated_data_bit(0, time_index, 1, modulation, &time_displacement, phase);
	if (err != cassette_image::error::SUCCESS) return err;
	time_index += time_displacement;
	return cassette_image::error::SUCCESS;
}


static cassette_image::error uef_cas_fill_half_bit(cassette_image *cassette, double &time_index, bool bit, cassette_image::Modulation &modulation, uint16_t phase)
{
	double time_displacement;
	cassette_image::error err = cassette->put_modulated_data_half_bit(0, time_index, bit, modulation, &time_displacement, phase);
	if (err != cassette_image::error::SUCCESS) return err;
	time_index += time_displacement;
	return cassette_image::error::SUCCESS;
}


static void update_modulation(uint32_t base_frequency, cassette_image::Modulation &modulation)
{
	modulation.flags = cassette_image::MODULATION_SINEWAVE;
	modulation.zero_frequency_canonical = base_frequency;
	modulation.zero_frequency_low = modulation.zero_frequency_canonical * 0.75;
	modulation.zero_frequency_high = modulation.zero_frequency_canonical * 1.25;
	modulation.one_frequency_canonical = 2 * base_frequency;
	modulation.one_frequency_low = modulation.one_frequency_canonical * 0.75;
	modulation.one_frequency_high = modulation.one_frequency_canonical * 1.25;
}


static cassette_image::error uef_cas_fill_wave(cassette_image *cassette, std::vector<uint8_t> &bytes)
{
	std::vector<uint8_t> gzip_area;
	int casdata_length = bytes.size();
	uint8_t *casdata = &bytes[0];
	uint32_t base_frequency = UEF_BASE_FREQUENCY;
	uint16_t phase = INITAL_PHASE;
	double time_index = 0.0;
	cassette_image::error err;
	cassette_image::Modulation modulation;

	update_modulation(base_frequency, modulation);

	if (bytes[0] == 0x1f && bytes[1] == 0x8b)
	{
		int gz_err;
		z_stream d_stream;
		unsigned long inflate_size = (casdata[casdata_length - 1] << 24) | (casdata[casdata_length - 2] << 16) | (casdata[casdata_length - 3] << 8) | casdata[casdata_length - 4];
		const uint8_t *in_ptr = skip_gz_header(casdata, casdata_length);

		if (in_ptr == nullptr)
		{
			return cassette_image::error::INVALID_IMAGE;
		}
		gzip_area.reserve(inflate_size);

		d_stream.zalloc = nullptr;
		d_stream.zfree = nullptr;
		d_stream.opaque = nullptr;
		d_stream.next_in = (unsigned char *)in_ptr;
		d_stream.avail_in = casdata_length - (in_ptr - casdata);
		d_stream.next_out = &gzip_area[0];
		d_stream.avail_out = inflate_size;

		gz_err =inflateInit2(&d_stream, -MAX_WBITS);
		if (gz_err != Z_OK)
		{
			LOG_FORMATS("inflateInit2 error: %d\n", gz_err);
			return cassette_image::error::INVALID_IMAGE;
		}
		gz_err = inflate(&d_stream, Z_NO_FLUSH);
		if (gz_err != Z_STREAM_END && gz_err != Z_OK)
		{
			LOG_FORMATS("inflate error: %d\n", gz_err);
			return cassette_image::error::INVALID_IMAGE;
		}
		gz_err = inflateEnd(&d_stream);
		if (gz_err != Z_OK)
		{
			LOG_FORMATS("inflateEnd error: %d\n", gz_err);
			return cassette_image::error::INVALID_IMAGE;
		}

		casdata_length = inflate_size;
		casdata = &gzip_area[0];
	}

	if (casdata_length < sizeof(UEF_HEADER) || memcmp(casdata, UEF_HEADER, sizeof(UEF_HEADER)))
	{
		return cassette_image::error::INVALID_IMAGE;
	}

	uint8_t loops = 1;
	uint32_t pos = sizeof(UEF_HEADER) + 2;
	while (pos < casdata_length)
	{
		if (pos + 6 > casdata_length)
		{
			return cassette_image::error::INVALID_IMAGE;
		}

		int chunk_type = (casdata[pos + 1] << 8) | casdata[pos];
		int chunk_length = (casdata[pos + 5] << 24) | (casdata[pos + 4] << 16) | (casdata[pos + 3] << 8) | casdata[pos + 2];
		pos += 6;

		if (pos + chunk_length > casdata_length)
		{
			return cassette_image::error::INVALID_IMAGE;
		}

		switch (chunk_type)
		{
		case 0x0000:    // empty
		case 0x0001:    // game instructions/manual/url
		case 0x0005:    // target machine
		case 0x0009:    // short title
			break;
		case 0x0100:    // implicit start/stop bit data block
			for (int j = 0; j < chunk_length; j++)
			{
				uint8_t byte = casdata[pos+j];
				err = uef_cas_fill_bit(loops, cassette, time_index, 0, modulation, phase);
				if (err != cassette_image::error::SUCCESS) return err;
				for (int i = 0; i < 8; i++)
				{
					err = uef_cas_fill_bit(loops, cassette, time_index, (byte >> i) & 1, modulation, phase);
					if (err != cassette_image::error::SUCCESS) return err;
				}
				err = uef_cas_fill_bit(loops, cassette, time_index, 1, modulation, phase);
				if (err != cassette_image::error::SUCCESS) return err;
			}
			break;
		case 0x0101:    // multiplexed data block
			LOG_FORMATS("Unsupported chunk type: %04x\n", chunk_type);
			return cassette_image::error::UNSUPPORTED;
		case 0x0102:    // explicit tape data block
			{
				int j = (chunk_length * 8) - casdata[pos];
				uint32_t temp_pos = pos + 1;
				uint8_t byte = 0;
				for (int i = 0; i < j; i++)
				{
					if ((i & 7) == 0)
						byte = casdata[temp_pos++];
					uint8_t bit = byte & 1;
					byte >>= 1;
					err = uef_cas_fill_bit(loops, cassette, time_index, bit, modulation, phase);
					if (err != cassette_image::error::SUCCESS) return err;
				}
			}
			break;
		case 0x0104:    // defined tape format data block
			{
				uint8_t num_bits = casdata[pos];
				uint8_t parity_type = casdata[pos + 1];
				int8_t num_stop_bits = casdata[pos + 2];
				bool extra_short_wave = num_stop_bits < 0;
				num_stop_bits = abs(num_stop_bits);
				for (int j = 3; j < chunk_length; j++)
				{
					uint8_t byte = casdata[pos+j];
					err = uef_cas_fill_bit(loops, cassette, time_index, 0, modulation, phase);
					if (err != cassette_image::error::SUCCESS) return err;
					uint8_t parity = 0;
					for (int i = 0; i < num_bits; i++)
					{
						uint8_t bit = (byte >> i) & 1;
						err = uef_cas_fill_bit(loops, cassette, time_index, bit, modulation, phase);
						if (err != cassette_image::error::SUCCESS) return err;
						parity ^= bit;
					}
					if (parity_type == 'O')
					{
						err = uef_cas_fill_bit(loops, cassette, time_index, parity ^ 1, modulation, phase);
						if (err != cassette_image::error::SUCCESS) return err;
					}
					else if (parity_type == 'E')
					{
						err = uef_cas_fill_bit(loops, cassette, time_index, parity, modulation, phase);
						if (err != cassette_image::error::SUCCESS) return err;
					}
					for (int i = num_stop_bits; i > 0; i--)
					{
						err = uef_cas_fill_bit(loops, cassette, time_index, 1, modulation, phase);
						if (err != cassette_image::error::SUCCESS) return err;
					}
					if (extra_short_wave)
					{
						err = uef_cas_fill_short_wave(cassette, time_index, modulation, phase);
						if (err != cassette_image::error::SUCCESS) return err;
					}
				}
			}
			break;
		case 0x0110:    // carrier tone (previously referred to as 'high tone')
			for (int i = ((casdata[pos + 1] << 8) | casdata[pos]); i > 0; i--)
			{
				err = uef_cas_fill_bit(loops, cassette, time_index, 1, modulation, phase);
				if (err != cassette_image::error::SUCCESS) return err;
			}
			break;
		case 0x0111:    // carrier tone with dummy byte
			for (int i = ((casdata[pos + 1] << 8) | casdata[pos]); i > 0; i--)
			{
				err = uef_cas_fill_bit(loops, cassette, time_index, 1, modulation, phase);
				if (err != cassette_image::error::SUCCESS) return err;
			}
			{
				const uint8_t byte = 0xaa;
				err = uef_cas_fill_bit(loops, cassette, time_index, 0, modulation, phase);
				if (err != cassette_image::error::SUCCESS) return err;
				for (int i = 0; i < 8; i++)
				{
					err = uef_cas_fill_bit(loops, cassette, time_index, (byte >> i) & 1, modulation, phase);
					if (err != cassette_image::error::SUCCESS) return err;
				}
				err = uef_cas_fill_bit(loops, cassette, time_index, 1, modulation, phase);
			}
			if (err != cassette_image::error::SUCCESS) return err;
			for (int i = ((casdata[pos + 3] << 8) | casdata[pos + 2]); i > 0; i--)
			{
				err = uef_cas_fill_bit(loops, cassette, time_index, 1, modulation, phase);
				if (err != cassette_image::error::SUCCESS) return err;
			}
			break;
		case 0x0112:    // integer gap
			{
				double gap = ((casdata[pos + 1] << 8) | casdata[pos]) / (base_frequency * 2);
				err = cassette->put_sample(0, time_index, gap, 0);
				if (err != cassette_image::error::SUCCESS) return err;
				time_index += gap;
			}
			break;
		case 0x0113:    // change of base frequency
			{
				base_frequency = get_uef_float(&casdata[pos]);
				update_modulation(base_frequency, modulation);
			}
			break;
			return cassette_image::error::UNSUPPORTED;
		case 0x0114:    // security cycles
			// Not fully working yet
			// Software that uses this:
			// bbc_cass:
			// - androida, applepie
			// electron_cass:
			// - cascad50, chipbust
			{
				uint32_t number_of_cycles = (casdata[pos + 2] << 16) | (casdata[pos + 1] << 8) | casdata[pos];
				bool first_high_pulse = casdata[pos + 3] == 'P';
				bool last_low_pulse = casdata[pos + 4] == 'P';
				uint8_t data = 0;
				uint32_t security_index = pos + 5;
				for (int i = 0; i < number_of_cycles; i++)
				{
					if ((i & 7) == 0)
						data = casdata[security_index++];
					uint8_t bit = data >> 7;
					data <<= 1;

					if (!i && first_high_pulse)
					{
						// Output high pulse, ie half cycle at phase 0
						err = uef_cas_fill_half_bit(cassette, time_index, bit, modulation, (phase + 180) % 360);
						if (err != cassette_image::error::SUCCESS) return err;
					}
					else if (i == number_of_cycles - 1 && last_low_pulse)
					{
						// Output low pulse, ie half cycle at phase 180
						err = uef_cas_fill_half_bit(cassette, time_index, bit, modulation, phase);
						if (err != cassette_image::error::SUCCESS) return err;
					}
					else
					{
						err = uef_cas_fill_bit(loops, cassette, time_index, bit, modulation, phase);
						if (err != cassette_image::error::SUCCESS) return err;
					}
				}
			}
			break;
		case 0x0115:    // phase change
			LOG_FORMATS("Unsupported chunk type: %04x\n", chunk_type);
			phase = (casdata[pos + 1] << 8) | casdata[pos];
			if (phase != 0 && phase != 180)
			{
				LOG_FORMATS("Unsupported phase: %d\n", phase);
				return cassette_image::error::UNSUPPORTED;
			}
			break;
		case 0x0116:    // floating point gap
			{
				double gap = get_uef_float(&casdata[pos]);
				err = cassette->put_sample(0, time_index, gap, 0);
				if (err != cassette_image::error::SUCCESS) return err;
				time_index += gap;
			}
			break;
		case 0x0117:    // data encoding format change
			{
				int baud_length = (casdata[pos + 1] << 8) | casdata[pos];
				// These are the only supported numbers
				if (baud_length == 300)
					loops = 4;
				else
				if (baud_length == 1200)
					loops = 1;
				else
				{
					LOG_FORMATS("Unsupported baud rate = %d\n",baud_length);
					return cassette_image::error::UNSUPPORTED;
				}
			}
			break;
		case 0x0120:    // position marker
		case 0x0130:    // tape set info
		case 0x0131:    // start of tape side
		default:
			LOG_FORMATS("Unsupported chunk type: %04x\n", chunk_type);
			return cassette_image::error::UNSUPPORTED;
		}
		pos += chunk_length;
	}
	return cassette_image::error::SUCCESS;
}


static cassette_image::error uef_cassette_identify(cassette_image *cassette, cassette_image::Options *opts)
{
	uint8_t header[10];

	cassette->image_read(header, 0, sizeof(header));
	if (memcmp(&header[0], GZ_HEADER, sizeof(GZ_HEADER)) && memcmp(&header[0], UEF_HEADER, sizeof(UEF_HEADER)))
	{
		return cassette_image::error::INVALID_IMAGE;
	}
	return cassette->modulation_identify(uef_cas_modulation, opts);
}


static cassette_image::error uef_cassette_load(cassette_image *cassette)
{
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> bytes(file_size);
	cassette->image_read(&bytes[0], 0, file_size);
	std::vector<int16_t> samples;

	return uef_cas_fill_wave(cassette, bytes);
}


const cassette_image::Format uef_cassette_format =
{
	"uef",
	uef_cassette_identify,
	uef_cassette_load,
	nullptr
};


CASSETTE_FORMATLIST_START(uef_cassette_formats)
	CASSETTE_FORMAT(uef_cassette_format)
CASSETTE_FORMATLIST_END
