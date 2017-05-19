/***********************************************************************************************************


 Sega AI Expansion slot emulation

 ***********************************************************************************************************/


#include "emu.h"
#include "segaai_exp.h"

#define VERBOSE 0
#define LOG(x) do { if (VERBOSE) logerror x; } while (0)


DEFINE_DEVICE_TYPE(SEGAAI_EXP_SLOT, segaai_exp_slot_device, "segaai_exp_slot", "Sega AI Expansion Slot")


device_segaai_exp_interface::device_segaai_exp_interface(const machine_config &mconfig, device_t &device)
	: device_slot_card_interface(mconfig, device)
{
}


device_segaai_exp_interface::~device_segaai_exp_interface()
{
}


segaai_exp_slot_device::segaai_exp_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, SEGAAI_EXP_SLOT, tag, owner, clock)
	, device_slot_interface(mconfig, *this)
{
}


segaai_exp_slot_device::~segaai_exp_slot_device()
{
}


void segaai_exp_slot_device::device_start()
{
	m_exp = dynamic_cast<device_segaai_exp_interface *>(get_card_device());
}


READ8_MEMBER(segaai_exp_slot_device::read_lo)
{
	if (m_exp)
	{
		return m_exp->read_lo(space, offset);
	}
	else
	{
		return 0xff;
	}
}


WRITE8_MEMBER(segaai_exp_slot_device::write_lo)
{
	if (m_exp)
	{
		m_exp->write_lo(space, offset, data);
	}
}


READ8_MEMBER(segaai_exp_slot_device::read_hi)
{
	if (m_exp)
	{
		return m_exp->read_hi(space, offset);
	}
	else
	{
		return 0xff;
	}
}


WRITE8_MEMBER(segaai_exp_slot_device::write_hi)
{
	if (m_exp)
	{
		m_exp->write_hi(space, offset, data);
	}
}


READ8_MEMBER(segaai_exp_slot_device::read_io)
{
	if (m_exp)
	{   
		return m_exp->read_io(space, offset);
	}
	else
	{   
		return 0xff;
	}
}


WRITE8_MEMBER(segaai_exp_slot_device::write_io)
{
	if (m_exp)
	{
		m_exp->write_io(space, offset, data);
	}
}


// slot interfaces
#include "soundbox.h"

SLOT_INTERFACE_START(segaai_exp)
	SLOT_INTERFACE("soundbox", SEGAAI_SOUNDBOX)
SLOT_INTERFACE_END

