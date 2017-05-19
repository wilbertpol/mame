// license:BSD-3-Clause
// copyright-holders:Wilbert Pol, Fabio Priuli
// thanks-to:Chris Covell
/*
 
 Sega AI driver

 
 Not much is known at this stage, except that the system was intended to be
 used for educational purposes in schools. Yet the audio chips seem much more
 powerful than what an educational computer requires...

 CPU : 16bit V20 @ 5MHz
 ROM : 128KB OS.with SEGA PROLOG
 RAM : 128KB
 VRAM : 64KB
 Video : V9938 Resolution 256x212
 Sound : SN76489
 Cassette Drive : 9600bps
 TV Output : RGB, Video, RF
 Keyboard : new JIS arrangement (Japanese input mapping)

 
TODO:
- The artwork system has no support for a real touchpad device with
  selectable artowrk, so the touchpad is emulated as 24x20 matrix
  of clickable buttons. This is currently good enough to make most
  games playable.
- Add on the fly switching of the upd7759 between stand alone and
  slave modes.
- IRQ enable/disable register
- Proper hooking of uPD7759 DRQ signals in slave mode.
- Cassette
- Keyboard (there is probably an mcu on it)

===========================================================================

 Sega AI Computer quick PCB overview by Chris Covell
 
 Major ICs 
 
 IC 1    D701080-5     (86/09?)  NEC V20 CPU       DIP40 
 IC 2    315-5200      (86/25)   SEGA          QFP100 
 IC 3    27C512-25     (86/15)   64K EPROM "E000  8/24" 
 IC 4    27C512-25     (86/06)   64K EPROM "F000  7/21" 
 IC 5    MPR-7689      (86/22)   SEGA "264 AA E79" (ROM) DIP28
 IC 10   V9938                   Yamaha MSX2 VDP 
 IC 13   D7759C        (86/12)   NEC Speech Synthesizer   DIP40 
 IC 14   MPR-7619      (86/23)   SEGA (ROM)      DIP28
 IC 15   MPR-7620      (86/23)   SEGA (ROM)      DIP28
 IC 16   SN76489AN               TI PSG         DIP16 
 IC 17   D8251AFC      (86/09)   NEC Communications Interface DIP28 
 IC 18   315-5201      (86/25)   SEGA (bodge wire on pins 9,10) DIP20 
 IC 19   M5204A        (87?/01)  OKI 
 IC 20   D8255AC-2     (86/08)   NEC Peripheral Interface DIP40 
 
 IC 6,7,8,9,11,12   D41464C-12   NEC 32K DRAMs - 128K RAM, 64K VRAM 
 
 Crystals, etc 
 
 X1   20.000           "KDS 6D" 
 X2   21.47727         "KDS" 
 X3   640kHz           "CSB 640 P" 
 
 Connectors 
 
 CN1   6-pin DIN Power connector 
 CN2   8-pin DIN "AUX" connector 
 CN3   Video phono jack 
 CN4   Audio phono jack 
 CN5   35-pin Sega MyCard connector 
 CN6   60-pin expansion connector A1..A30 Bottom, B1..B30 Top 
 CN7   9-pin header connector to "Joy, Button, LED" unit 
 CN8   13(?) pin flat flex connector to pressure pad 
 CN9   9-pin header connector to tape drive motor, etc. 
 CN10   13-pin header connector to tape heads 
 JP2   2-wire header to SW2 button board 
 PJ1   7-wire header to Keyboard / Mic connector board 
 MIC   2-wire header to mic on KB/Mic board 
 SW1   Reset Switch 
 
 Power switch is on the AC Adaptor 
 
 Joypad unit (by Mitsumi) has U/D/L/R, "PL" and "PR" buttons, and a power LED. 
 
Power Connector Pinout (Seen from AC Adaptor plug):                
   1     5        1  12V COM    5   5V COM 
      6           2  12V OUT    6   5V OUT 
   2     4        3   5V COM        
      3           4   5V OUT 

AUX Connector Pinout: 
   7   6          1 +5V(?)      5 csync 
  3  8  1         2 GND         6 green 
   5   4          3 blue        7 Audio out 
     2            4 +5V(?)      8 red 

New JIS Keyboard Connector Pinout: 
    1 2           1,2,3 data lines 
  3 4   5         4 ??          5,8 data lines 
   6 7 8          6 GND         7 +5V
 

*/ 

#include "emu.h"
#include "cpu/nec/nec.h"
#include "cpu/z80/z80.h"
#include "sound/sn76496.h"
#include "sound/upd7759.h"
#include "video/v9938.h"
#include "bus/segaai/segaai_slot.h"
#include "bus/segaai/segaai_exp.h"
#include "machine/i8255.h"
#include "machine/i8251.h"
#include "speaker.h"
#include "softlist.h"

// Layout
#include "segaai.lh"


#define TOUCHPAD_ROWS 20

class segaai_state : public driver_device
{
public:
	segaai_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_sound(*this, "sn76489a")
		, m_v9938(*this, "v9938")
		, m_upd7759(*this, "upd7759")
		, m_port4(*this, "PORT4")
		, m_port5(*this, "PORT5")
		, m_port_tp(*this, "TP.%u", 0)
	{ }
	
	DECLARE_WRITE_LINE_MEMBER(vdp_interrupt);
	DECLARE_WRITE_LINE_MEMBER(upd7759_drq_w);
	DECLARE_WRITE_LINE_MEMBER(upd7759_busy_w);
	IRQ_CALLBACK_MEMBER(irq_callback);
	DECLARE_READ8_MEMBER(i8255_porta_r);
	DECLARE_READ8_MEMBER(i8255_portb_r);
	DECLARE_READ8_MEMBER(i8255_portc_r);
	DECLARE_WRITE8_MEMBER(i8255_portc_w);
	DECLARE_WRITE8_MEMBER(upd7759_ctrl_w);
	DECLARE_WRITE8_MEMBER(port1c_w);
	DECLARE_WRITE8_MEMBER(port1d_w);
	DECLARE_WRITE8_MEMBER(port1e_w);
	DECLARE_READ8_MEMBER(port1e_r);

	// unknown device writes
	DECLARE_READ8_MEMBER(unk16_r);
	DECLARE_WRITE8_MEMBER(unk17_w);

protected:
	virtual void machine_start();

private:
	void update_irq_state();
	bool get_touchpad_pressed();

	required_device<cpu_device> m_maincpu;
	required_device<sn76489a_device> m_sound;
	required_device<v9938_device> m_v9938;
	required_device<upd7759_device> m_upd7759;
	required_ioport m_port4;
	required_ioport m_port5;
	required_ioport_array<TOUCHPAD_ROWS> m_port_tp;

	u8 m_i8255_portb;
	u8 m_upd7759_ctrl;
	u8 m_port_1c;
	u8 m_port_1d;
	u8 m_port_1e;
	int m_v9938_irq;
	int m_0xfb_irq;
	int m_prev_v9938_irq;
	int m_prev_0xfb_irq;
	bool m_v9938_irq_triggered;
	bool m_0xfb_irq_triggered;
	u8 m_touchpad_x;
	u8 m_touchpad_y;
	u8 m_unk17[8];
};


static ADDRESS_MAP_START(mem_map, AS_PROGRAM, 8, segaai_state)
	AM_RANGE(0x00000, 0x1ffff) AM_RAM
	AM_RANGE(0x20000, 0x3ffff) AM_DEVREADWRITE("exp", segaai_exp_slot_device, read_lo, write_lo)
	AM_RANGE(0x80000, 0x8ffff) AM_DEVREADWRITE("exp", segaai_exp_slot_device, read_hi, write_hi)
	AM_RANGE(0xa0000, 0xbffff) AM_DEVREADWRITE("cardslot", segaai_card_slot_device, read_cart, write_cart)
	AM_RANGE(0xc0000, 0xdffff) AM_ROM
	AM_RANGE(0xe0000, 0xeffff) AM_ROM
	AM_RANGE(0xf0000, 0xfffff) AM_ROM
ADDRESS_MAP_END


static ADDRESS_MAP_START(io_map, AS_IO, 8, segaai_state)
	AM_RANGE(0x00, 0x03) AM_DEVREADWRITE("v9938", v9938_device, read, write)
	AM_RANGE(0x04, 0x07) AM_DEVREADWRITE("tmp8255", i8255_device, read, write)

	AM_RANGE(0x08, 0x08) AM_DEVREADWRITE("i8251", i8251_device, data_r, data_w)
	AM_RANGE(0x09, 0x09) AM_DEVREADWRITE("i8251", i8251_device, status_r, control_w)

	// 0x0a (w) - ??
	AM_RANGE(0x0b, 0x0b) AM_WRITE(upd7759_ctrl_w)    // 315-5201

	AM_RANGE(0x0c, 0x0c) AM_DEVWRITE("sn76489a", sn76489a_device, write)

	// 0x0e (w) - ??
	// 0x0f (w) - ??

	AM_RANGE(0x14, 0x14) AM_MIRROR(0x01) AM_DEVWRITE("upd7759", upd7759_device, port_w)

	// 0x16 (w) - ??  irq enable/disable??
	AM_RANGE(0x16, 0x16) AM_READ(unk16_r)
	// 0x17 (w) - ??
	AM_RANGE(0x17, 0x17) AM_WRITE(unk17_w)  // ??

	// Touchpad
	AM_RANGE(0x1c, 0x1c) AM_WRITE(port1c_w)
	AM_RANGE(0x1d, 0x1d) AM_WRITE(port1d_w)
	AM_RANGE(0x1e, 0x1e) AM_READWRITE(port1e_r, port1e_w)

	// 0x1f (w) - ??

	// Expansion I/O
	AM_RANGE(0x20, 0x3f) AM_DEVREADWRITE("exp", segaai_exp_slot_device, read_io, write_io)
ADDRESS_MAP_END


#define INPUT_TP_ROW(row) \
	PORT_START(row) \
	PORT_BIT(0x000001, IP_ACTIVE_HIGH, IPT_OTHER) \
	PORT_BIT(0x000002, IP_ACTIVE_HIGH, IPT_OTHER) \
	PORT_BIT(0x000004, IP_ACTIVE_HIGH, IPT_OTHER) \
	PORT_BIT(0x000008, IP_ACTIVE_HIGH, IPT_OTHER) \
	PORT_BIT(0x000010, IP_ACTIVE_HIGH, IPT_OTHER) \
	PORT_BIT(0x000020, IP_ACTIVE_HIGH, IPT_OTHER) \
	PORT_BIT(0x000040, IP_ACTIVE_HIGH, IPT_OTHER) \
	PORT_BIT(0x000080, IP_ACTIVE_HIGH, IPT_OTHER) \
	PORT_BIT(0x000100, IP_ACTIVE_HIGH, IPT_OTHER) \
	PORT_BIT(0x000200, IP_ACTIVE_HIGH, IPT_OTHER) \
	PORT_BIT(0x000400, IP_ACTIVE_HIGH, IPT_OTHER) \
	PORT_BIT(0x000800, IP_ACTIVE_HIGH, IPT_OTHER) \
	PORT_BIT(0x001000, IP_ACTIVE_HIGH, IPT_OTHER) \
	PORT_BIT(0x002000, IP_ACTIVE_HIGH, IPT_OTHER) \
	PORT_BIT(0x004000, IP_ACTIVE_HIGH, IPT_OTHER) \
	PORT_BIT(0x008000, IP_ACTIVE_HIGH, IPT_OTHER) \
	PORT_BIT(0x010000, IP_ACTIVE_HIGH, IPT_OTHER) \
	PORT_BIT(0x020000, IP_ACTIVE_HIGH, IPT_OTHER) \
	PORT_BIT(0x040000, IP_ACTIVE_HIGH, IPT_OTHER) \
	PORT_BIT(0x080000, IP_ACTIVE_HIGH, IPT_OTHER) \
	PORT_BIT(0x100000, IP_ACTIVE_HIGH, IPT_OTHER) \
	PORT_BIT(0x200000, IP_ACTIVE_HIGH, IPT_OTHER) \
	PORT_BIT(0x400000, IP_ACTIVE_HIGH, IPT_OTHER) \
	PORT_BIT(0x800000, IP_ACTIVE_HIGH, IPT_OTHER)

static INPUT_PORTS_START(ai_kbd)
	PORT_START("PORT4")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_JOYSTICK_UP) PORT_8WAY
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN) PORT_8WAY
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT) PORT_8WAY
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT) PORT_8WAY
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_BUTTON2) PORT_NAME("PL")
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_BUTTON1) PORT_NAME("RL")
	PORT_BIT(0xc0, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("PORT5")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_BUTTON3) PORT_NAME("Grey Button")
	PORT_BIT(0xfe, IP_ACTIVE_LOW, IPT_UNUSED)

	// Touchpad
	INPUT_TP_ROW("TP.0")
	INPUT_TP_ROW("TP.1")
	INPUT_TP_ROW("TP.2")
	INPUT_TP_ROW("TP.3")
	INPUT_TP_ROW("TP.4")
	INPUT_TP_ROW("TP.5")
	INPUT_TP_ROW("TP.6")
	INPUT_TP_ROW("TP.7")
	INPUT_TP_ROW("TP.8")
	INPUT_TP_ROW("TP.9")
	INPUT_TP_ROW("TP.10")
	INPUT_TP_ROW("TP.11")
	INPUT_TP_ROW("TP.12")
	INPUT_TP_ROW("TP.13")
	INPUT_TP_ROW("TP.14")
	INPUT_TP_ROW("TP.15")
	INPUT_TP_ROW("TP.16")
	INPUT_TP_ROW("TP.17")
	INPUT_TP_ROW("TP.18")
	INPUT_TP_ROW("TP.19")
INPUT_PORTS_END


// Based on edge triggers, level triggers are created(?)
void segaai_state::update_irq_state()
{
	int state = CLEAR_LINE;

	if (m_v9938_irq != CLEAR_LINE)
	{
		if (m_prev_v9938_irq == CLEAR_LINE)
		{
			m_v9938_irq_triggered = true;
		}
	}
	m_prev_v9938_irq = m_v9938_irq;

	if (m_0xfb_irq != CLEAR_LINE)
	{
		if (m_prev_0xfb_irq == CLEAR_LINE)
		{
			m_0xfb_irq_triggered = true;
		}
	}
	m_prev_0xfb_irq = m_0xfb_irq;

	if (m_v9938_irq_triggered || m_0xfb_irq_triggered)
	{
		state = ASSERT_LINE;
	}

	m_maincpu->set_input_line(0, state);
}


WRITE_LINE_MEMBER(segaai_state::vdp_interrupt)
{
	m_v9938_irq = state;
	update_irq_state();
}


WRITE_LINE_MEMBER(segaai_state::upd7759_drq_w)
{
//	if ((m_upd7759_ctrl & 0x01))
//	{
		m_0xfb_irq = state ? CLEAR_LINE : ASSERT_LINE;
		update_irq_state();
//	}
}


WRITE_LINE_MEMBER(segaai_state::upd7759_busy_w)
{
	if (!(m_upd7759_ctrl & 0x01))
	{
//		m_0xfb_irq = state ? CLEAR_LINE : ASSERT_LINE;
//		update_irq_state();
	}
}


IRQ_CALLBACK_MEMBER(segaai_state::irq_callback)
{
	int vector = 0;	// default??

	if (m_v9938_irq_triggered)
	{
		vector = 0xf8;
		m_v9938_irq_triggered = false;
	}
	else if (m_0xfb_irq_triggered)
	{
		vector = 0xfb;
		m_0xfb_irq_triggered = false;
	}

	update_irq_state();

    return vector;
}


/*
Mainboard 8255 port A

 76543210
 +-------- Microphone sensor (1 = sound enabled)
  +------- Unknown (usually 1) // -BUSY output from the uPD7759?
   +------ PR trigger (active low)
    +----- PL trigger (active low)
     +---- Pad right (active low)
      +--- Pad lefta (active low)
       +-- Pad down (active low)
        +- Pad up (active low)
*/
READ8_MEMBER(segaai_state::i8255_porta_r)
{
	u8 data = (m_upd7759->busy_r() ? 0x40 : 0) | (m_port4->read() & ~0x40);

	return data;
}


/*
Mainboard 8255 port B

 76543210
 +-------- CN9 Pin 8
  +------- Tape head engaged
   +------ Tape insertion sensor
    +----- Tape write enable sensor
     +---- keyboard connector pin 3
      +--- 0 = Touch pad data available
       +-- 0 = Touch pad pressed
        +- Trigger button near touch panel (active low)
*/
READ8_MEMBER(segaai_state::i8255_portb_r)
{
	m_i8255_portb = (m_i8255_portb & 0xf8) | (m_port5->read() & 0x01);

	if (m_port_1d & 0x01)
	{
		if (!get_touchpad_pressed())
		{
			m_i8255_portb |= 0x02;
		}

		m_i8255_portb |= 0x04;
	}
	else
	{
		m_i8255_portb |= 0x02;
		// Bit 2 reset to indicate that touchpad data is available
	}

	return m_i8255_portb;
}


bool segaai_state::get_touchpad_pressed()
{
	static const u8 tp_x[24] =
	{
		  5,  15,  26,  37,  47,  58,  69,  79,  90, 101, 111, 122,
		133, 143, 154, 165, 175, 186, 197, 207, 218, 229, 239, 250
	};

	static const u8 tp_y[20] =
	{
		  6,  18,  31,  44,  57,  70,  82,  95, 108, 121,
		134, 146, 159, 172, 185, 198, 210, 223, 236, 249
	};

	for (int row = 0; row < TOUCHPAD_ROWS; row++)
	{
		u32 port = m_port_tp[row]->read();

		if (port)
		{
			int bit = -1;
			while (port)
			{
				bit++;
				port >>= 1;
			}

			if (bit >= 0 && bit < ARRAY_LENGTH(tp_x))
			{
				m_touchpad_x = tp_x[bit];
				m_touchpad_y = tp_y[row];
				return true;
			}
		}
	}

	return false;
}


/*
Mainboard 8255 port C

 76543210
 +-------- keyboard connector pin 5
  +------- keyboard connector pin 8
   +------ keyboard connector pin 2
    +----- keyboard connector pin 1
     +---- Output
      +--- Output
       +-- Output
        +- Output
*/
READ8_MEMBER(segaai_state::i8255_portc_r)
{
	u8 data = 0xf0;

	return data;
}


WRITE8_MEMBER(segaai_state::i8255_portc_w)
{
	logerror("i8255 port c write: %02x\n", data);
}


WRITE8_MEMBER(segaai_state::upd7759_ctrl_w)
{
	logerror("I/O Port $0b write: $%02x\n", data);

	m_upd7759_ctrl = data;

	// bit0 is connected to /md line of the uPD7759?
	//m_upd7759->md_w((m_upd7759_ctrl & 0x01) ? 0 : 1);
	m_upd7759->reset_w((m_upd7759_ctrl & 0x01) ? 1 : 0);

	// bit1 selects which ROM should be used?
	m_upd7759->set_bank_base((m_upd7759_ctrl & 2) ? 0x00000 : 0x20000);
}


READ8_MEMBER(segaai_state::unk16_r)
{
	u8 data = (m_unk17[7] ? 0x80 : 0)
	           | (m_unk17[6] ? 0x40 : 0)
	           | (m_unk17[5] ? 0x20 : 0)
	           | (m_unk17[4] ? 0x10 : 0)
	           | (m_unk17[3] ? 0x08 : 0)
	           | (m_unk17[2] ? 0x04 : 0)
	           | (m_unk17[1] ? 0x02 : 0)
	           | (m_unk17[0] ? 0x01 : 0)
	;

	return data;
}

/*
Port 16 and 17 are closely related

Some config can be written through port 17, and the current combined
settings can be read through port 16.

See these snippets from eigogam2:
A9EC5: FA                        di
A9EC6: E4 16                     in      al,16h
A9EC8: A2 82 12                  mov     [1282h],al
A9ECB: B0 00                     mov     al,0h
A9ECD: E6 17                     out     17h,al
A9ECF: B0 02                     mov     al,2h
A9ED1: E6 17                     out     17h,al
A9ED3: B0 04                     mov     al,4h
A9ED5: E6 17                     out     17h,al
A9ED7: B0 07                     mov     al,7h
A9ED9: E6 17                     out     17h,al
A9EDB: B0 0D                     mov     al,0Dh
A9EDD: E6 17                     out     17h,al
A9EDF: B0 0E                     mov     al,0Eh
A9EE1: E6 17                     out     17h,al
A9EE3: FB                        ei
...
A9F05: B0 06                     mov     al,6h
A9F07: E6 17                     out     17h,al
A9F09: B0 0D                     mov     al,0Dh
A9F0B: E6 17                     out     17h,al
A9F0D: A0 82 12                  mov     al,[1282h]
A9F10: D0 C0                     rol     al,1
A9F12: 24 01                     and     al,1h
A9F14: 04 0E                     add     al,0Eh
A9F16: E6 17                     out     17h,al
A9F18: A0 82 12                  mov     al,[1282h]
A9F1B: D0 C0                     rol     al,1
A9F1D: D0 C0                     rol     al,1
A9F1F: 24 01                     and     al,1h
A9F21: 04 0C                     add     al,0Ch
A9F23: E6 17                     out     17h,al
A9F25: 8A 26 82 12               mov     ah,[1282h]
A9F29: 32 DB                     xor     bl,bl
A9F2B: B9 03 00                  mov     cw,3h
A9F2E: 8A C4                     mov     al,ah
A9F30: 24 01                     and     al,1h
A9F32: 02 C3                     add     al,bl
A9F34: E6 17                     out     17h,al
A9F36: D0 EC                     shr     ah,1
A9F38: 80 C3 02                  add     bl,2h
A9F3B: E2 F1                     dbnz    0A9F2Eh
*/
WRITE8_MEMBER(segaai_state::unk17_w)
{
	// Possibly mode pins on the 7759
	logerror("I/O Port $17 write: $%02x\n", data);

	int pin = (data >> 1) & 0x07;
	u8 state = data & 1;
	u8 old_state = m_unk17[pin];

	m_unk17[pin] = state;

	if (old_state != state)
	{
		switch (pin)
		{
			case 3:		// -MD pin on upd7759?
				// TODO
				m_upd7759->md_w(state ? 0 : 1);
				m_upd7759->reset_w(state ? 1 : 0);
				break;
		}
	}
}


WRITE8_MEMBER(segaai_state::port1c_w)
{
	m_port_1c = data;
}


WRITE8_MEMBER(segaai_state::port1d_w)
{
	m_port_1d = data;
}


WRITE8_MEMBER(segaai_state::port1e_w)
{
	m_port_1e = data;
}


READ8_MEMBER(segaai_state::port1e_r)
{
	if (m_port_1c & 0x01)
	{
		return m_touchpad_y;
	}
	else
	{
		return m_touchpad_x;
	}
}


void segaai_state::machine_start()
{
	m_i8255_portb = 0x7f;
	m_upd7759_ctrl = 0;
	m_port_1c = 0;
	m_port_1d = 0;
	m_port_1e = 0;
	m_v9938_irq = CLEAR_LINE;
	m_0xfb_irq = CLEAR_LINE;
	m_prev_v9938_irq = CLEAR_LINE;
	m_prev_0xfb_irq = CLEAR_LINE;
	m_v9938_irq_triggered = false;
	m_0xfb_irq_triggered = false;
	m_touchpad_x = 0;
	m_touchpad_y = 0;
	for (int i = 0; i < 8; i++)
	{
		m_unk17[i] = 0xff;
	}
}


static MACHINE_CONFIG_START(segaai)
	MCFG_CPU_ADD("maincpu", V20, XTAL_20MHz/4)
	MCFG_CPU_PROGRAM_MAP(mem_map)
	MCFG_CPU_IO_MAP(io_map)
	MCFG_CPU_IRQ_ACKNOWLEDGE_DRIVER(segaai_state, irq_callback)

	MCFG_V9938_ADD("v9938", "screen", 0x10000, XTAL_21_4772MHz)   // 64KB VRAM, clocked at 21477270
	MCFG_V99X8_INTERRUPT_CALLBACK(WRITELINE(segaai_state, vdp_interrupt))
	MCFG_V99X8_SCREEN_ADD_NTSC("screen", "v9938", XTAL_21_4772MHz)

	MCFG_DEVICE_ADD("tmp8255", I8255, 0)
	MCFG_I8255_IN_PORTA_CB(READ8(segaai_state, i8255_porta_r))
	MCFG_I8255_IN_PORTB_CB(READ8(segaai_state, i8255_portb_r))
	MCFG_I8255_IN_PORTC_CB(READ8(segaai_state, i8255_portc_r))
	MCFG_I8255_OUT_PORTC_CB(WRITE8(segaai_state, i8255_portc_w))

	MCFG_DEVICE_ADD("i8251", I8251, 0)
//	MCFG_I8251_TXD_HANDLER()
//	MCFG_I8251_DTR_HANDLER()
//	MCFG_I8251_RTS_HANDLER()
//	MCFG_I8251_RXRDY_HANDLER()
//	MCFG_I8251_TXRDY_HANDLER()
//	MCFG_I8251_TXEMPTY_HANDLER()
//	MCFG_I8251_SYNDET_HANDLER()

	MCFG_SPEAKER_STANDARD_MONO("mono")

	MCFG_SOUND_ADD("sn76489a", SN76489A, XTAL_21_4772MHz/6) // not verified, but sounds close to real hw recordings
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 1.00)

	MCFG_SOUND_ADD("upd7759", UPD7759, UPD7759_STANDARD_CLOCK)
	MCFG_UPD7759_DRQ_CALLBACK(WRITELINE(segaai_state, upd7759_drq_w))
	MCFG_UPD7759_BUSY_CALLBACK(WRITELINE(segaai_state, upd7759_busy_w))
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 1.00)

	// Card slot
	MCFG_SEGAAI_CARD_ADD("cardslot", segaai_card, nullptr)
	MCFG_SOFTWARE_LIST_ADD("software", "segaai")

	// Expansion slot
	MCFG_SEGAAI_EXP_ADD("exp", segaai_exp, "soundbox")

	MCFG_DEFAULT_LAYOUT(layout_segaai)
MACHINE_CONFIG_END


ROM_START(segaai)
	ROM_REGION(0x100000, "maincpu", 0)
	ROM_LOAD("mpr-7689.ic5",  0xc0000, 0x20000, CRC(62402ac9) SHA1(bf52d22b119d54410dad4949b0687bb0edf3e143))
	ROM_LOAD("e000 8_24.ic3", 0xe0000, 0x10000, CRC(c8b6a539) SHA1(cbf8473d1e3d8037ea98e9ca8b9aafdc8d16ff23))	// actual label was "e000 8/24"
	ROM_LOAD("f000 7_21.ic4", 0xf0000, 0x10000, CRC(64d6cd8c) SHA1(68c130048f16d6a0abe1978e84440931470222d9))	// actual label was "f000 7/21"

	ROM_REGION(0x40000, "upd7759", 0)
	ROM_LOAD("mpr-7619.ic14", 0x00000, 0x20000, CRC(d1aea002) SHA1(c8d5408bba65b17301f19cf9ebd2b635d642525a))
	ROM_LOAD("mpr-7620.ic15", 0x20000, 0x20000, CRC(e042754b) SHA1(02aede7a3e2fda9cbca621b530afa4520cf16610))
ROM_END


COMP(1986, segaai,     0,         0,      segaai,   ai_kbd, segaai_state,   0,    "Sega",   "AI", MACHINE_NOT_WORKING)
