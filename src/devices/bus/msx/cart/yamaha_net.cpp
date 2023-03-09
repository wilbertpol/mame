// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/**************************************************************************

Yamaha networking as found in the Russian YIS-503iir, YIS-503iiir, and
YIS-805/128R2 machines.

Information found from:
- https://web.archive.org/web/20040416072133/http://www.betuwe.net:80/~mellemab/homecomputers/specials/index.html
- https://web.archive.org/web/20050204104155/http://members.chello.nl/h.otten/fronthw.htm
- https://web.archive.org/web/20090503205741/http://milliways.chance.ru/~tnt23/msx/
- https://www.msx.org/forum/msx-talk/hardware/yamaha-yis-805128r2-hardware-problems
- https://sysadminmosaic.ru/msx/yamaha_local_network/yamaha_local_network

TODO:
- Everything

**************************************************************************/

#include "emu.h"
#include "yamaha_net.h"

#include "bus/midi/midi.h"
#include "bus/rs232/rs232.h"
#include "machine/i8251.h"
#include "machine/pit8253.h"

namespace {

// Yamaha XA586A0
// Uses 8253 and 8251 like in RS-232
// SW1 - Teacher/Student switch
// DIP switch SW1 (computer id?)
// Student mode
// - TXD connected to CN2/3 pin 4
// - RXD connected to CN2/3 pin 5
// Teacher mode
// - TXD connected to CN2/3 pin 5
// - RXD connected to CN2/3 pin 4
class msx_cart_yamaha_netv1_device : public device_t, public msx_cart_interface
{
public:
	msx_cart_yamaha_netv1_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
		: device_t(mconfig, MSX_CART_YAMAHA_NETV1, tag, owner, clock)
		, msx_cart_interface(mconfig, *this)
		, m_region_net(*this, "net")
		, m_i8251(*this, "i8251")
		, m_i8253(*this, "i8253")
		, m_cn2(*this, "cn2")
		, m_cn3(*this, "cn3")
		, m_sw(*this, "SW%u", 1L)
		, m_out2(0)
	{ }

protected:
	virtual const tiny_rom_entry *device_rom_region() const override;
	virtual ioport_constructor device_input_ports() const override;
	virtual void device_add_mconfig(machine_config &config) override;
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	void out2_w(int state);
	u8 status_r();
	void control_w(u8 data);

	required_memory_region m_region_net;
	required_device<i8251_device> m_i8251;
	required_device<pit8253_device> m_i8253;
	required_device<rs232_port_device> m_cn2;
	required_device<rs232_port_device> m_cn3;
	required_ioport_array<2> m_sw;
	u8 m_out2;
};

ROM_START(msx_netv1)
	ROM_REGION(0x2000, "net", 0)
	ROM_LOAD("yis503iirnet.rom.ic2",  0x0000, 0x2000, CRC(0731db3f) SHA1(264fbb2de69fdb03f87dc5413428f6aa19511a7f))
ROM_END

const tiny_rom_entry *msx_cart_yamaha_netv1_device::device_rom_region() const
{
	return ROM_NAME(msx_netv1);
}

static INPUT_PORTS_START(msx_netv1_ports)
	PORT_START("SW1")
	PORT_CONFNAME(0x01, 0x01, "Mode")
	PORT_CONFSETTING(0x00, "Teacher")
	PORT_CONFSETTING(0x01, "Student")

	PORT_START("SW2")
    PORT_DIPNAME(0x01, 0x00, "Network ID bit 0")
    PORT_DIPSETTING(0x00, DEF_STR(Off))
    PORT_DIPSETTING(0x01, DEF_STR(On))
    PORT_DIPNAME(0x02, 0x00, "Network ID bit 1")
    PORT_DIPSETTING(0x00, DEF_STR(Off))
    PORT_DIPSETTING(0x02, DEF_STR(On))
    PORT_DIPNAME(0x04, 0x00, "Network ID bit 2")
    PORT_DIPSETTING(0x00, DEF_STR(Off))
    PORT_DIPSETTING(0x04, DEF_STR(On))
    PORT_DIPNAME(0x08, 0x00, "Network ID bit 3")
    PORT_DIPSETTING(0x00, DEF_STR(Off))
    PORT_DIPSETTING(0x08, DEF_STR(On))
    PORT_DIPNAME(0x10, 0x00, "Network ID bit 4")
    PORT_DIPSETTING(0x00, DEF_STR(Off))
    PORT_DIPSETTING(0x10, DEF_STR(On))
    PORT_DIPNAME(0x20, 0x00, "Network ID bit 5")
    PORT_DIPSETTING(0x00, DEF_STR(Off))
    PORT_DIPSETTING(0x20, DEF_STR(On))
INPUT_PORTS_END

ioport_constructor msx_cart_yamaha_netv1_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(msx_netv1_ports);
}

void msx_cart_yamaha_netv1_device::device_add_mconfig(machine_config &config)
{
	// Config based on svi738 schematics, are they the same for other machines?

	I8251(config, m_i8251, 1.8432_MHz_XTAL);
//	m_i8251->txd_handler().set(m_rs232, FUNC(rs232_port_device::write_txd));
//	m_i8251->rxrdy_handler().set(*this, FUNC(msx_slot_rs232_base_device::rxrdy_w)); // Linked to IRQ?

	PIT8253(config, m_i8253);
	m_i8253->set_clk<0>(1.8432_MHz_XTAL);
	m_i8253->set_clk<1>(1.8432_MHz_XTAL);
	m_i8253->set_clk<2>(1.8432_MHz_XTAL);
	m_i8253->out_handler<0>().set(m_i8251, FUNC(i8251_device::write_rxc));
	m_i8253->out_handler<1>().set(m_i8251, FUNC(i8251_device::write_txc));
	m_i8253->out_handler<2>().set(*this, FUNC(msx_cart_yamaha_netv1_device::out2_w));

	RS232_PORT(config, m_cn2, default_rs232_devices, nullptr);

	RS232_PORT(config, m_cn3, default_rs232_devices, nullptr);
//	m_rs232->rxd_handler().set(m_i8251, FUNC(i8251_device::write_rxd));
//	m_rs232->dcd_handler().set(*this, FUNC(msx_slot_rs232_base_device::dcd_w));
//	m_rs232->ri_handler().set(*this, FUNC(msx_slot_rs232_base_device::ri_w));
//	m_rs232->cts_handler().set(*this, FUNC(msx_slot_rs232_base_device::cts_w));
//	m_rs232->dsr_handler().set(m_i8251, FUNC(i8251_device::write_dsr));
}

void msx_cart_yamaha_netv1_device::device_start()
{
	save_item(NAME(m_out2));

	page(1)->install_rom(0x4000, 0x5fff, m_region_net->base());

	io_space().install_readwrite_handler(0x00, 0x01, read8sm_delegate(*m_i8251, FUNC(i8251_device::read)), write8sm_delegate(*m_i8251, FUNC(i8251_device::write)));
	io_space().install_readwrite_handler(0x02, 0x02, read8smo_delegate(*this, FUNC(msx_cart_yamaha_netv1_device::status_r)), write8smo_delegate(*this, FUNC(msx_cart_yamaha_netv1_device::control_w)));
	io_space().install_readwrite_handler(0x04, 0x07, read8sm_delegate(*m_i8253, FUNC(pit8253_device::read)), write8sm_delegate(*m_i8253, FUNC(pit8253_device::write)));
}

void msx_cart_yamaha_netv1_device::device_reset()
{
	m_out2 = 0;
	// DSR and CTS are pulled low
	m_i8251->write_cts(0);
	m_i8251->write_dsr(0);
}

void msx_cart_yamaha_netv1_device::out2_w(int state)
{
	m_out2 = state;
}

u8 msx_cart_yamaha_netv1_device::status_r()
{
	return (m_sw[1]->read() & 0x3f) | (m_out2 ? 0x40 : 0x00);
}

void msx_cart_yamaha_netv1_device::control_w(u8 data)
{
	// Bit 0 enables/disables IRQs from 8251?
}




// Yamaha XC466B - Serial I/O Mark II
// Built around ym3802 midi chip
//
// There is actually no distinction between teacher and student units, The only difference is the network id.
//
// SW1 - 4bit dip switch - computer id/address (0 = teacher, 1-15 = student 1-15)
// IC2 - Yamaha YM3802 - CLKF - 614 kHz; CLK - system clock 3.57MHz
// IC3 - Toshiba TMM24256BP-20 - 32KB ROM
// IC4 - Sanyo LC3517BL-15 - 2K RAM
// Networks had terminators on the first and last machines.
class msx_cart_yamaha_netv2_device : public device_t, public msx_cart_interface
{
public:
	msx_cart_yamaha_netv2_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
		: device_t(mconfig, MSX_CART_YAMAHA_NETV2, tag, owner, clock)
		, msx_cart_interface(mconfig, *this)
	{ }

protected:
	virtual const tiny_rom_entry *device_rom_region() const override;
	virtual void device_start() override;
};

ROM_START(msx_netv2)
	ROM_REGION(0x8000, "net", 0)
	ROM_LOAD("yis503iiirnet.rom.ic3", 0x0000, 0x8000, CRC(75331cac) SHA1(307a7be064442feb4ab2e1a2bc971b138c1a1169)) // From student machine
	ROM_LOAD("yis805128r2net.rom.ic3", 0x0000, 0x8000, CRC(0e345b43) SHA1(e8fd2bbc1bdab12c73a0fec178a190f9063547bb)) // From teacher machine
ROM_END

const tiny_rom_entry *msx_cart_yamaha_netv2_device::device_rom_region() const
{
	return ROM_NAME(msx_netv2);
}

void msx_cart_yamaha_netv2_device::device_start()
{
}

} // anonymous namespace

DEFINE_DEVICE_TYPE_PRIVATE(MSX_CART_YAMAHA_NETV1, msx_cart_interface, msx_cart_yamaha_netv1_device, "msx_cart_yamaha_netv1", "MSX Yamaha networking module v1")
DEFINE_DEVICE_TYPE_PRIVATE(MSX_CART_YAMAHA_NETV2, msx_cart_interface, msx_cart_yamaha_netv2_device, "msx_cart_yamaha_netv2", "MSX Yamaha networking module v2")
