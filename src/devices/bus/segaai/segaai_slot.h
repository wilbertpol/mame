#ifndef MAME_BUS_SEGAAI_SLOT_H
#define MAME_BUS_SEGAAI_SLOT_H

#pragma once

#include "softlist_dev.h"


DECLARE_DEVICE_TYPE(SEGAAI_CARD_SLOT, segaai_card_slot_device);


class device_segaai_card_interface : public device_slot_card_interface
{
public:
	// construction/destruction
	device_segaai_card_interface(const machine_config &mconfig, device_t &device);
	virtual ~device_segaai_card_interface();

	// reading and writing
	virtual DECLARE_READ8_MEMBER(read_cart) { return 0xff; }
	virtual DECLARE_WRITE8_MEMBER(write_cart) {}

	void rom_alloc(u32 size, const char *tag);

	virtual void late_bank_setup() {}

	u8* get_rom_base() { return m_rom; }
	u32 get_rom_size() { return m_rom_size; }

protected:
	void rom_map_setup(u32 size);

	// internal state
	u8 *m_rom;
	u32 m_rom_size;
	int m_rom_page_count;
};


class segaai_card_slot_device : public device_t,
								public device_image_interface,
								public device_slot_interface
{
public:
	// construction/destruction
	segaai_card_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);
	virtual ~segaai_card_slot_device();

	// device-level overrides
	virtual void device_start() override;
//	virtual void device_config_complete() { update_names(SEGAAI_CARD_SLOT, "cartridge", "cart"); }

	// image-level overrides
	virtual image_init_result call_load() override ;
	virtual void call_unload() override {}

	int get_type() { return m_type; }
	static int get_cart_type(const u8 *ROM, u32 len);

	void set_mandatory(bool val) { m_must_be_loaded = val; }
	void set_intf(const char * interface) { m_interface = interface; }
	void set_ext(const char * extensions) { m_extensions = extensions; }

	virtual iodevice_t image_type() const override { return IO_CARTSLOT; }
	virtual bool is_readable()  const override { return 1; }
	virtual bool is_writeable() const override { return 0; }
	virtual bool is_creatable() const override { return 0; }
	virtual bool must_be_loaded() const override { return m_must_be_loaded; }
	virtual bool is_reset_on_load() const override { return 1; }
	virtual const char *image_interface() const override { return m_interface; }
	virtual const char *file_extensions() const override { return m_extensions; }

	// slot interface overrides
	virtual std::string get_default_card_software(get_default_card_software_hook &hook) const override ;

	// reading and writing
	virtual DECLARE_READ8_MEMBER(read_cart);
	virtual DECLARE_WRITE8_MEMBER(write_cart);

protected:
	int m_type;
	bool m_must_be_loaded, m_is_card;
	const char *m_interface;
	const char *m_extensions;
	device_segaai_card_interface*       m_cart;
};


#define SEGA_AI_SLOT_ROM_REGION_TAG ":cart:rom"


#define MCFG_SEGAAI_CARD_ADD(_tag,_slot_intf,_def_slot) \
	MCFG_DEVICE_ADD(_tag, SEGAAI_CARD_SLOT, 0) \
	MCFG_DEVICE_SLOT_INTERFACE(_slot_intf, _def_slot, false) \
	static_cast<segaai_card_slot_device *>(device)->set_mandatory(false); \
	static_cast<segaai_card_slot_device *>(device)->set_intf("segaai_card"); \
	static_cast<segaai_card_slot_device *>(device)->set_ext("aic,bin");

SLOT_INTERFACE_EXTERN(segaai_card);

#endif
