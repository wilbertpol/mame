/***********************************************************************************************************


 Sega AI card emulation

 ***********************************************************************************************************/


#include "emu.h"
#include "segaai_slot.h"

#define VERBOSE 0
#define LOG(x) do { if (VERBOSE) logerror x; } while (0)


/* PCB */
enum
{
	SEGAAI_CARD_ROM_128 = 0,
	SEGAAI_CARD_ROM_256 
};


DEFINE_DEVICE_TYPE(SEGAAI_CARD_SLOT, segaai_card_slot_device, "segaai_card_slot", "Sega AI Card Slot")


device_segaai_card_interface::device_segaai_card_interface(const machine_config &mconfig, device_t &device)
	: device_slot_card_interface(mconfig, device),
		m_rom(NULL),
		m_rom_size(0),
		m_rom_page_count(0)
{
}


device_segaai_card_interface::~device_segaai_card_interface()
{
}


void device_segaai_card_interface::rom_alloc(u32 size, const char *tag)
{
	if (m_rom == NULL)
	{
		m_rom = device().machine().memory().region_alloc(std::string(tag).append(SEGA_AI_SLOT_ROM_REGION_TAG).c_str(), size, 1, ENDIANNESS_LITTLE)->base();
		m_rom_size = size;
		m_rom_page_count = size / 0x4000;
		if (!m_rom_page_count)
			m_rom_page_count = 1;   // we compute rom pages through (XXX % m_rom_page_count)!
		late_bank_setup();
	}
}




segaai_card_slot_device::segaai_card_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
		: device_t(mconfig, SEGAAI_CARD_SLOT, tag, owner, clock)
		, device_image_interface(mconfig, *this)
		, device_slot_interface(mconfig, *this)
		, m_type(SEGAAI_CARD_ROM_128)
		, m_must_be_loaded(false)
		, m_interface("segaai_card")
		, m_extensions("aic,bin")
{
}


segaai_card_slot_device::~segaai_card_slot_device()
{
}


void segaai_card_slot_device::device_start()
{
	m_cart = dynamic_cast<device_segaai_card_interface *>(get_card_device());
}


struct segaai_slot
{
	int                     pcb_id;
	const char              *slot_option;
};

// Here, we take the feature attribute from .xml (i.e. the PCB name) and we assign a unique ID to it
static const segaai_slot slot_list[] =
{
	{ SEGAAI_CARD_ROM_128, "rom_128" },
	{ SEGAAI_CARD_ROM_256, "rom_256" }
};


static const char *segaai_get_slot(int type)
{
	for (int i = 0; i < ARRAY_LENGTH(slot_list); i++)
	{
		if (slot_list[i].pcb_id == type)
			return slot_list[i].slot_option;
	}

	return slot_list[0].slot_option;
}


image_init_result segaai_card_slot_device::call_load()
{
	if (m_cart)
	{
		u32 len = !loaded_through_softlist() ? length() : get_software_region_length("rom");
		u32 offset = 0;
		u8 *ROM;

		if (len != 0x20000 && len != 0x40000)
		{
			seterror(IMAGE_ERROR_UNSPECIFIED, "Invalid card size. Allowed sizes are: 128KB, 256KB");
			return image_init_result::FAIL;
		}

		m_cart->rom_alloc(len, tag());
		ROM = m_cart->get_rom_base();

		if (!loaded_through_softlist())
		{
			fseek(offset, SEEK_SET);
			fread(ROM, len);
		}
		else
			memcpy(ROM, get_software_region("rom"), len);

		m_type = get_cart_type(ROM, len);

		return image_init_result::PASS;
	}

	return image_init_result::PASS;
}


int segaai_card_slot_device::get_cart_type(const u8 *ROM, u32 len)
{
	int type = SEGAAI_CARD_ROM_128;

	if (len == 0x40000)
	{
		type = SEGAAI_CARD_ROM_256;
	}

	return type;
}


std::string segaai_card_slot_device::get_default_card_software(get_default_card_software_hook &hook) const
{
	if (hook.image_file())
	{
		const char *slot_string = "rom_128";
		u32 len = hook.image_file()->size();
		std::vector<u8> rom(len);
		int type;

		hook.image_file()->read(&rom[0], len);

		type = get_cart_type(&rom[0], len);
		slot_string = segaai_get_slot(type);

		return std::string(slot_string);
	}

	return software_get_default_slot("rom_128");
}


READ8_MEMBER(segaai_card_slot_device::read_cart)
{
	if (m_cart)
		return m_cart->read_cart(space, offset);
	else
		return 0xff;
}


WRITE8_MEMBER(segaai_card_slot_device::write_cart)
{
	if (m_cart)
		m_cart->write_cart(space, offset, data);
}



// slot interfaces
#include "rom.h"


SLOT_INTERFACE_START(segaai_card)
	SLOT_INTERFACE_INTERNAL("rom_128", SEGAAI_ROM_128)
	SLOT_INTERFACE_INTERNAL("rom_256", SEGAAI_ROM_256)
SLOT_INTERFACE_END

