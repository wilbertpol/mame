// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/*

The UEF file format is designed to store accurate images of the common media types associated
with the BBC Micro, Acorn Electron and Atom. Tape storage is compatible with the CUTS/BYTE/Kansas City
Format, and hence the format is also capable of storing software for non-Acorn systems such as the
Altair 8800, PT SOL-20, Ohio Scientific, Compukit UK101, Nascom 1/2/3, Motorola MEK D1 6800 and
SWTPC 6800 kit based computers.

UEF files are chunk based and optionally compressed.

The UEF format supports gzipped images, i'm doing the gunzip step during uef_cas_to_wav_size
because that is when the length of the original file is known. This is needed to determine
the size of memory to alloc for the decoding.

Not nice, but it works...

*/
#include "uef_cas.h"
#include "imageutl.h"

#include <zlib.h>

#include <cmath>
#include <cstring>


#define UEF_WAV_FREQUENCY   4800
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


static const uint8_t* skip_gz_header(const uint8_t *p, int length)
 {
	const uint8_t *max_p = p + length;
	if (p + 10 >= max_p)
		return nullptr;
	// skip initial 1f 8b header
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
	if (( flags & ORIG_NAME) != 0) 
	{
		for ( ; *p && p < max_p; p++);
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


static void uef_cas_fill_bit(uint8_t loops, std::vector<int16_t> &samples, bool bit)
{
	for (uint8_t i = 0; i < loops; i++)
	{
		if (bit)
		{
			samples.push_back(WAVE_LOW);
			samples.push_back(WAVE_HIGH);
			samples.push_back(WAVE_LOW);
			samples.push_back(WAVE_HIGH);
		}
		else
		{
			samples.push_back(WAVE_LOW);
			samples.push_back(WAVE_LOW);
			samples.push_back(WAVE_HIGH);
			samples.push_back(WAVE_HIGH);
		}
	}
}


static cassette_image::error uef_cas_fill_wave(std::vector<int16_t> &samples, std::vector<uint8_t> &bytes)
{
	std::vector<uint8_t> gzip_area;
	int casdata_length = bytes.size();
	uint8_t *casdata = &bytes[0];

	if (bytes[0] == 0x1f && bytes[1] == 0x8b)
	{
		int gz_err;
		z_stream d_stream;
		int inflate_size = (casdata[casdata_length - 1] << 24) | (casdata[casdata_length - 2] << 16) | (casdata[casdata_length - 3] << 8) | casdata[casdata_length - 4];
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

	uint8_t loops = 1;
	uint32_t pos = sizeof(UEF_HEADER) + 2;
	while (pos < casdata_length)
	{
		if (pos + 6 > casdata_length)
			return cassette_image::error::INVALID_IMAGE;

		int chunk_type = (casdata[pos + 1] << 8) | casdata[pos];
		int chunk_length = (casdata[pos + 5] << 24) | (casdata[pos + 4] << 16) | (casdata[pos + 3] << 8) | casdata[pos + 2];
		pos += 6;

		if (pos + chunk_length > casdata_length)
			return cassette_image::error::INVALID_IMAGE;

		switch (chunk_type)
		{
		case 0x0000:    // empty
			break;
		case 0x0100:    // implicit start/stop bit data block
		case 0x0104:    // used by atom dumps, looks like normal data
			for (int j = 0; j < chunk_length; j++)
			{
				uint8_t byte = casdata[pos+j];
				uef_cas_fill_bit(loops, samples, 0);
				for (int i = 0; i < 8; i++)
					uef_cas_fill_bit(loops, samples, (byte >> i) & 1);
				uef_cas_fill_bit(loops, samples, 1);
			}
			break;
		case 0x0101:    // multiplexed data block
		case 0x0103:
			LOG_FORMATS("Unsupported chunk type: %04x\n", chunk_type);
			printf("Unsupported chunk type: %04x\n", chunk_type);
			return cassette_image::error::UNSUPPORTED;
		case 0x0102:    // explicit tape data block
			printf("TODO Check chunk_type 0x0102\n");
			{
				// Weird; subtracting casdata[pos] from length and also outputting that byte
				int j = (chunk_length * 10) - casdata[pos];
				uint32_t temp_pos = pos;
				while (j)
				{
					uint8_t byte = casdata[temp_pos];
					for (int i = 0; i < 8 && i < j; i++)
					{
						uef_cas_fill_bit(loops, samples, (byte >> i) & 1);
						j--;
					}
					temp_pos++;
				}
			}
			break;
		case 0x0110:    // carrier tone (previously referred to as 'high tone')
			for (int i = ((casdata[pos + 1] << 8) | casdata[pos]); i; i--)
			{
				samples.push_back(WAVE_LOW);
				samples.push_back(WAVE_HIGH);
			}
			break;
		case 0x0112:    // integer gap
			for (int i = ((casdata[pos + 1] << 8) | casdata[pos]); i; i--)
			{
				samples.push_back(WAVE_NULL);
				samples.push_back(WAVE_NULL);
			}
			break;
		case 0x0116:    // floating point gap
			for (int i = (get_uef_float(&casdata[pos]) * UEF_WAV_FREQUENCY); i; i--)
			{
				samples.push_back(WAVE_NULL);
			}
			break;
		case 0x0117:    // change baud rate
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
					printf("Unsupported baud rate = %d\n",baud_length);
					return cassette_image::error::UNSUPPORTED;
				}
			}
			break;
		default:
			LOG_FORMATS("Unsupported chunk type: %04x\n", chunk_type);
			printf("Unsupported chunk type: %04x\n", chunk_type);
			return cassette_image::error::UNSUPPORTED;
		}
		pos += chunk_length;
	}
	return cassette_image::error::SUCCESS;
}


static cassette_image::error uef_cassette_identify( cassette_image *cassette, cassette_image::Options *opts )
{
	uint8_t header[10];

	cassette->image_read(header, 0, sizeof(header));
	if (memcmp(&header[0], GZ_HEADER, sizeof(GZ_HEADER)) && memcmp(&header[0], UEF_HEADER, sizeof(UEF_HEADER)))
	{
		return cassette_image::error::INVALID_IMAGE;
	}
	opts->channels = 1;
	opts->bits_per_sample = 16;
	opts->sample_frequency = UEF_WAV_FREQUENCY;
	return cassette_image::error::SUCCESS;
}


static cassette_image::error uef_cassette_load( cassette_image *cassette )
{
	uint64_t file_size = cassette->image_size();
	std::vector<uint8_t> bytes(file_size);
	cassette->image_read(&bytes[0], 0, file_size);
	std::vector<int16_t> samples;

	cassette_image::error err = uef_cas_fill_wave(samples, bytes);
	if (err != cassette_image::error::SUCCESS)
		return err;

	return cassette->put_samples(0, 0.0,
			(double)samples.size() / UEF_WAV_FREQUENCY, samples.size(), 2,
			&samples[0], cassette_image::WAVEFORM_16BIT);
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
