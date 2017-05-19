#ifndef MAME_BUS_SEGAAI_EXP_H
#define MAME_BUS_SEGAAI_EXP_H

/***************************************************************************
 TYPE DEFINITIONS
 ***************************************************************************/


DECLARE_DEVICE_TYPE(SEGAAI_EXP_SLOT, segaai_exp_slot_device);


// ======================> device_segaai_exp_interface

class device_segaai_exp_interface : public device_slot_card_interface
{
public:
	// construction/destruction
	device_segaai_exp_interface(const machine_config &mconfig, device_t &device);
	virtual ~device_segaai_exp_interface();

	// reading and writing
	// 0x20000 - 0x3ffff
	virtual DECLARE_READ8_MEMBER(read_lo) { return 0xff; }
	virtual DECLARE_WRITE8_MEMBER(write_lo) {}
	// 0x80000 - 0x9ffff
	virtual DECLARE_READ8_MEMBER(read_hi) { return 0xff; }
	virtual DECLARE_WRITE8_MEMBER(write_hi) {}
	// I/O 0x20 - 0x3f
	virtual DECLARE_READ8_MEMBER(read_io) { return 0xff; }
	virtual DECLARE_WRITE8_MEMBER(write_io) {}
};


// ======================> segaai_exp_slot_device

class segaai_exp_slot_device : public device_t,
								public device_slot_interface
{
public:
	// construction/destruction
	segaai_exp_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);
	virtual ~segaai_exp_slot_device();

	// device-level overrides
	virtual void device_start();

	// reading and writing
	virtual DECLARE_READ8_MEMBER(read_lo);
	virtual DECLARE_WRITE8_MEMBER(write_lo);
	virtual DECLARE_READ8_MEMBER(read_hi);
	virtual DECLARE_WRITE8_MEMBER(write_hi);
	virtual DECLARE_READ8_MEMBER(read_io);
	virtual DECLARE_WRITE8_MEMBER(write_io);

protected:
	device_segaai_exp_interface*       m_exp;
};


/***************************************************************************
 DEVICE CONFIGURATION MACROS
 ***************************************************************************/

#define MCFG_SEGAAI_EXP_ADD(_tag,_slot_intf,_def_slot) \
	MCFG_DEVICE_ADD(_tag, SEGAAI_EXP_SLOT, 0) \
	MCFG_DEVICE_SLOT_INTERFACE(_slot_intf, _def_slot, false)

// slot interfaces
SLOT_INTERFACE_EXTERN( segaai_exp );

#endif
