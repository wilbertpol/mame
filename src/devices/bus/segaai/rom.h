#ifndef MAME_BUS_SEGAAI_ROM_H
#define MAME_BUS_SEGAAI_ROM_H

#include "segaai_slot.h"

// ======================> segaai_rom_device

class segaai_rom_128_device : public device_t,
						public device_segaai_card_interface
{
public:
	// construction/destruction
	segaai_rom_128_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	// device-level overrides
	virtual void device_start();
	virtual void device_reset();

	// reading and writing
	virtual DECLARE_READ8_MEMBER(read_cart);
	virtual DECLARE_WRITE8_MEMBER(write_cart);

protected:
	segaai_rom_128_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, u32 clock);
};


class segaai_rom_256_device : public segaai_rom_128_device
{
public:
	segaai_rom_256_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	// device-level overrides
	virtual void device_start();
	virtual void device_reset();

    // reading and writing
	virtual DECLARE_READ8_MEMBER(read_cart);
	virtual DECLARE_WRITE8_MEMBER(write_cart);

protected:
	u8 m_bank_reg[4];
};


// device type definition
DECLARE_DEVICE_TYPE(SEGAAI_ROM_128, segaai_rom_128_device);
DECLARE_DEVICE_TYPE(SEGAAI_ROM_256, segaai_rom_256_device);

#endif
