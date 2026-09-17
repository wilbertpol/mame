// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/**********************************************************************

    TI Explorer CPU board.

    Board part number 2243895-0001, board type "CPU", vendor "TIAU".

    It is a NuBus card like every other board in the machine. What
	makes it unusual is that the NuBus and the local bus *are* this
	board's own address spaces: the raven's AS_DATA is the NuBus and
	AS_LOCAL_BUS is the local bus, which is why the backplane
	(ti_nubus_device) is pointed at this board's CPU rather than the
	other way round.

**********************************************************************/

#ifndef MAME_TI_EXPLORER_CPU_H
#define MAME_TI_EXPLORER_CPU_H

#pragma once

#include "ti_nubus.h"

#include "cpu/raven/raven.h"


class explorer_cpu_device : public device_t, public device_ti_nubus_card_interface
{
public:
	explorer_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;

	// device_ti_nubus_card_interface implementation
	virtual void assert_bus_error() override;

private:
	required_device<raven_cpu_device> m_cpu;
	required_memory_region m_microcode_proms;
	required_memory_region m_control_store;
	// The lamps along the front edge of the board, numbered as Field
	// Maintenance Figure 1-13 numbers them. Lamps 1-6 are the yellow
	// "internal states" code the microcode writes into MCR(05:00); 7 and 8 are
	// yellow too but are hardware conditions, see the TODO in the source. The
	// red fault LED is the one below the column.
	output_finder<6> m_state_led;
	output_finder<> m_fault_led;

	void nubus_map(address_map &map) ATTR_COLD;

	void state_leds_w(u8 data);
	void fault_led_w(int state);
};

DECLARE_DEVICE_TYPE(EXPLORER_CPU, explorer_cpu_device)

#endif // MAME_TI_EXPLORER_CPU_H
