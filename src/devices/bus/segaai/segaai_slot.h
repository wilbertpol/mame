// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
#ifndef MAME_BUS_SEGAAI_SLOT_H
#define MAME_BUS_SEGAAI_SLOT_H

#pragma once

#include "softlist_dev.h"


DECLARE_DEVICE_TYPE(SEGAAI_CARD_SLOT, segaai_card_slot_device);


class device_segaai_card_interface : public device_interface
{
public:
	device_segaai_card_interface(const machine_config &mconfig, device_t &device);
	virtual ~device_segaai_card_interface();

	virtual u8 read_cart(offs_t offset) { return 0xff; }
	virtual void write_cart(offs_t offset, u8 data) {}

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
								public device_single_card_slot_interface<device_segaai_card_interface>
{
public:
	template <typename T>
	segaai_card_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, T &&opts, const char *dflt)
		: segaai_card_slot_device(mconfig, tag, owner, u32(0))
	{
		option_reset();
		opts(*this);
		set_default_option(dflt);
		set_fixed(false);
	}
	segaai_card_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);
	virtual ~segaai_card_slot_device();

	virtual void device_start() override;

	virtual image_init_result call_load() override ;
	virtual void call_unload() override {}
	virtual const software_list_loader &get_software_list_loader() const override { return rom_software_list_loader::instance(); }
<<<<<<< HEAD
	virtual const char *image_type_name() const noexcept override { return "card"; }
	virtual const char *image_brief_type_name() const noexcept override { return "card"; }
=======
	virtual const char *custom_instance_name() const noexcept override { return "card"; }
	virtual const char *custom_brief_instance_name() const noexcept override { return "card"; }
>>>>>>> b94ded99dbc (checkpoint)

	int get_type() { return m_type; }
	static int get_cart_type(const u8 *ROM, u32 len);

	void set_mandatory(bool val) { m_must_be_loaded = val; }
	void set_intf(const char * interface) { m_interface = interface; }
	void set_ext(const char * extensions) { m_extensions = extensions; }

<<<<<<< HEAD
	virtual bool is_readable()  const noexcept override { return 1; }
	virtual bool is_writeable() const noexcept override { return 0; }
	virtual bool is_creatable() const noexcept override { return 0; }
	virtual bool is_reset_on_load() const noexcept override { return true; }
=======
	virtual iodevice_t image_type() const noexcept override { return IO_CARTSLOT; }
	virtual bool is_readable()  const noexcept override { return 1; }
	virtual bool is_writeable() const noexcept override { return 0; }
	virtual bool is_creatable() const noexcept override { return 0; }
	virtual bool must_be_loaded() const noexcept override { return m_must_be_loaded; }
	virtual bool is_reset_on_load() const noexcept override { return 1; }
>>>>>>> b94ded99dbc (checkpoint)
	virtual const char *image_interface() const noexcept override { return m_interface; }
	virtual const char *file_extensions() const noexcept override { return m_extensions; }

	virtual std::string get_default_card_software(get_default_card_software_hook &hook) const override ;

	virtual u8 read_cart(offs_t offset);
	virtual void write_cart(offs_t offset, u8 data);

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
	static_cast<segaai_card_slot_device *>(device)->set_ext("bin,aic");

void segaai_card(device_slot_interface &device);

#endif
