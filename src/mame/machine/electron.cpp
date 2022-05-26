// license:BSD-3-Clause
// copyright-holders:Wilbert Pol, Nigel Barnes
/******************************************************************************

    Acorn Electron driver

******************************************************************************/

#include "emu.h"
#include "includes/electron.h"
#include "screen.h"
#include "netlist/devices/net_lib.h"


void electron_state::waitforramsync()
{
	int cycles = 0;

	if (!(m_ula.screen_mode & 4) && (m_screen->vpos() > m_screen->visible_area().top()) && (m_screen->vpos() < m_screen->visible_area().bottom()) && !m_screen->hblank())
	{
		cycles += (m_screen->visible_area().right() - m_screen->hpos()) / 16;
	}
	if (cycles & 1) cycles++;

	m_maincpu->adjust_icount(-cycles);
}


void electron_state::device_timer(emu_timer &timer, device_timer_id id, int param)
{
	switch (id)
	{
	case TIMER_TAPE_HANDLER:
		electron_tape_timer_handler(param);
		break;
	case TIMER_SETUP_BEEP:
		setup_beep(param);
		break;
	case TIMER_SCANLINE_INTERRUPT:
		electron_scanline_interrupt(param);
		break;
	default:
		throw emu_fatalerror("Unknown id in electron_state::device_timer");
	}
}


void electron_state::electron_tape_start()
{
	if (m_ula.tape_running )
	{
		return;
	}
	m_ula.tape_steps = 0;
	m_ula.tape_value = 0x80808080;
	m_ula.high_tone_set = 0;
	m_ula.bit_count = 0;
	m_ula.tape_running = 1;
	m_tape_timer->adjust(attotime::zero, 0, attotime::from_hz(44100));
}

void electron_state::electron_tape_stop()
{
	m_ula.tape_running = 0;
	m_tape_timer->reset();
}

#define TAPE_LOW    0x00;
#define TAPE_HIGH   0xFF;

TIMER_CALLBACK_MEMBER(electron_state::electron_tape_timer_handler)
{
	if (m_ula.cassette_motor_mode)
	{
		double tap_val = m_cassette->input();
		if ((tap_val >= 0.0 && m_ula.last_tap_val < 0.0) || (tap_val < 0.0 && m_ula.last_tap_val >= 0.0))
		{
			if (m_ula.tap_val_length > (9 * 3))
			{
				for (int i = 0; i <= 3; i++)
					m_ula.len[i] = 0;
				m_ula.tap_val_length = 0;
			}

			for (int i = 3; i > 0; i--)
				m_ula.len[i] = m_ula.len[i-1];
			m_ula.len[0] = m_ula.tap_val_length;

			m_ula.tap_val_length = 0;

			if ((m_ula.len[0] + m_ula.len[1]) >= (18 + 18 - 5))
			{
				cassette_bit_received(0);
				for (int i = 0; i <= 3; i++)
					m_ula.len[i] = 0;
			}

			if (((m_ula.len[0] + m_ula.len[1] + m_ula.len[2] + m_ula.len[3]) <= (18 + 18 + 5)) && (m_ula.len[3] != 0))
			{
				cassette_bit_received(1);
				for (int i = 0; i <= 3; i++)
					m_ula.len[i] = 0;
			}
		}
		m_ula.tap_val_length++;
		m_ula.last_tap_val = tap_val;
	}
}

NETLIST_START(electron_cass_input)
	// No idea what all this means
	NET_MODEL("LM324_5V OPAMP(TYPE=3 VLH=0.667 VLL=0.0667 FPF=5 UGF=500k SLEW=0.3M RI=1000k RO=50 DAB=0.00075)")

	SOLVER(Solver, 4800)
	PARAM(Solver.ACCURACY, 5e-4)
	PARAM(Solver.DYNAMIC_TS, 1)
	PARAM(Solver.DYNAMIC_LTE, 1e-2)
	PARAM(Solver.DYNAMIC_MIN_TIMESTEP, 1e-4)

	ANALOG_INPUT(V5, 5)
	ANALOG_INPUT(VM5, -5)

	ANALOG_INPUT(CASIN, 0)

	RES(R8, RES_K(3.3))
	RES(R9, RES_K(270))
	RES(R10, RES_K(4.7))
	RES(R11, RES_K(4.7))
	RES(R15, RES_K(5.6))
	RES(R18, RES_K(10))
	RES(R20, RES_K(39))
	RES(R25, RES_K(8.2))
	RES(R27, RES_K(8.2))
	RES(R35, RES_K(160))
	RES(R36, RES_K(220))
	RES(R43, RES_K(820))
	RES(R44, RES_K(150))
	RES(R64, RES_K(10))

	DIODE(D6, "1N914")  // type unknown
	DIODE(D7, "1N914")  // type unknown

	CAP(C7, CAP_U(10))
	CAP(C10, CAP_N(220))
	CAP(C14, CAP_N(4.7)) // 4n7
	CAP(C16, CAP_N(4.7)) // 4n7
	CAP(C20, CAP_P(820))
	CAP(C22, CAP_P(820))
	CAP(C26, CAP_N(1.5)) // 1n5

	OPAMP(IC13_A, "LM324_5V")  // pins 5,6,7
	OPAMP(IC13_B, "LM324_5V")  // pins 8,9,10
	OPAMP(IC13_C, "LM324_5V")  // pins 11,12,13,14

	NET_C(V5, IC13_A.VCC, IC13_B.VCC, IC13_C.VCC)
	NET_C(VM5, IC13_A.GND, IC13_B.GND, IC13_C.GND)

	QBJT_SW(Q1, "BC237B")  // Actually BC239
	QBJT_SW(Q2, "BC237B")  // Actually BC239

	NET_C(CASIN, R64.1)
	NET_C(V5, D7.K)
	NET_C(VM5, D6.A)
	NET_C(GND, C26.1, R43.1, R44.1, C14.1, R20.1, R15.1, R10.1, R8.1)
	NET_C(R64.2, D6.K, D7.A, C26.2, C22.1)
	NET_C(C22.2, C20.1, R35.1)
	NET_C(C20.2, R44.2, IC13_A.PLUS)
	NET_C(R36.2, R43.2, IC13_A.MINUS)
	NET_C(R35.2, R36.1, R27.1, IC13_A.OUT)
	NET_C(R27.2, C16.1, R25.1)
	NET_C(R25.2, C14.2, IC13_B.PLUS)
	NET_C(R20.2, IC13_B.MINUS, R18.2)
	NET_C(C16.2, R18.1, C10.1, R15.2, IC13_B.OUT)
	NET_C(C10.2, R11.1)
	NET_C(R11.2, Q2.E, Q1.B, Q1.C, R9.1, IC13_C.MINUS)
	NET_C(R10.2, IC13_C.PLUS)
	NET_C(IC13_C.OUT, R8.2, C7.1, R9.2, Q1.E, Q2.B, Q2.C)

	ALIAS(OUTPUT, C7.2)
NETLIST_END()


NETDEV_ANALOG_CALLBACK_MEMBER(electron_state::casin_cb)
{
	logerror("received %d\n", data);
}

void electron_state::cassette_bit_received(int bit)
{
	switch (m_ula.bit_count)
	{
	case 0: // start bit
		m_ula.start_bit = bit;
		if (m_ula.start_bit)
		{
			if (m_ula.high_tone_set)
			{
				return;
			}
		}
		else
		{
			m_ula.high_tone_set = 0;
		}
		break;
	case 1: case 2: case 3: case 4:
	case 5: case 6: case 7: case 8:
		//logerror("++ Read regular bit: %d\n", bit ? 0 : 1);
		m_ula.tape_byte = (m_ula.tape_byte >> 1) | (bit ? 0x80 : 0);
		break;
	case 9: // stop bit
		m_ula.stop_bit = bit;
		//logerror("++ Read stop bit: %d\n", m_ula.stop_bit);
		if (m_ula.start_bit && m_ula.stop_bit && m_ula.tape_byte == 0xFF && !m_ula.high_tone_set)
		{
			electron_interrupt_handler(INT_SET, INT_HIGH_TONE);
			m_ula.high_tone_set = 1;
		}
		else if (!m_ula.start_bit && m_ula.stop_bit)
		{
			//logerror("-- Byte read from tape: %02x\n", m_ula.tape_byte);
			electron_interrupt_handler(INT_SET, INT_RECEIVE_FULL);
		}
		else
		{
			logerror("Invalid start/stop bit combination detected: %d,%d\n", m_ula.start_bit, m_ula.stop_bit);
		}
		break;
	}
	m_ula.bit_count = (m_ula.bit_count + 1) % 10;
}


uint8_t electron_state::electron64_fetch_r(offs_t offset)
{
	m_vdu_drivers = (offset & 0xe000) == 0xc000 ? true : false;

	return m_maincpu->space(AS_PROGRAM).read_byte(offset);
}

uint8_t electron_state::electron_mem_r(offs_t offset)
{
	uint8_t data = 0xff;

	data &= m_exp->expbus_r(offset);

	switch (m_mrb.read_safe(0))
	{
	case 0x00: /* Normal */
		/* The processor will run at 1MHz during an access cycle to the RAM */
		m_maincpu->set_clock_scale(0.5f);
		waitforramsync();
		break;

	case 0x01: /* Turbo */
		if (m_mrb_mapped && offset < 0x3000) offset += 0x8000;
		break;

	case 0x02: /* Shadow */
		if (m_mrb_mapped && (offset < 0x3000 || !m_vdu_drivers)) offset += 0x8000;
		break;
	}
	data &= m_ram->read(offset);

	return data;
}

void electron_state::electron_mem_w(offs_t offset, uint8_t data)
{
	m_exp->expbus_w(offset, data);

	switch (m_mrb.read_safe(0))
	{
	case 0x00: /* Normal */
		/* The processor will run at 1MHz during an access cycle to the RAM */
		m_maincpu->set_clock_scale(0.5f);
		waitforramsync();
		break;

	case 0x01: /* Turbo */
		if (m_mrb_mapped && offset < 0x3000) offset += 0x8000;
		break;

	case 0x02: /* Shadow */
		if (m_mrb_mapped && (offset < 0x3000 || !m_vdu_drivers)) offset += 0x8000;
		break;
	}
	m_ram->write(offset, data);
}

uint8_t electron_state::electron_paged_r(offs_t offset)
{
	/*  0 Second external socket on the expansion module (SK2) */
	/*  1 Second external socket on the expansion module (SK2) */
	/*  2 First external socket on the expansion module (SK1)  */
	/*  3 First external socket on the expansion module (SK1)  */
	/*  4 Disc                                                 */
	/*  5 USER applications                                    */
	/*  6 USER applications                                    */
	/*  7 Modem interface ROM                                  */
	/*  8 Keyboard                                             */
	/*  9 Keyboard mirror                                      */
	/* 10 BASIC rom                                            */
	/* 11 BASIC rom mirror                                     */
	/* 12 Expansion module operating system                    */
	/* 13 High priority slot in expansion module               */
	/* 14 ECONET                                               */
	/* 15 Reserved                                             */

	uint8_t data = 0;

	/* The processor will run at 2MHz during an access cycle to the ROM */
	m_maincpu->set_clock_scale(1.0f);

	switch (m_ula.rompage)
	{
	case 8:
	case 9:
		/* Keyboard */
		for (int i = 0; i < 14; i++)
		{
			if (!(offset & 1))
				data |= m_keybd[i]->read() & 0x0f;

			offset = offset >> 1;
		}
		break;

	case 10:
	case 11:
		/* BASIC */
		data = m_region_mos->base()[offset & 0x3fff];
		break;

	default:
		/* ROM in extension devices */
		data = m_exp->expbus_r(0x8000 + offset);
		break;
	}
	return data;
}

void electron_state::electron_paged_w(offs_t offset, uint8_t data)
{
	/* The processor will run at 2MHz during an access cycle to the ROM */
	m_maincpu->set_clock_scale(1.0f);

	m_exp->expbus_w(0x8000 + offset, data);
}

uint8_t electronsp_state::electron_paged_r(offs_t offset)
{
	uint8_t data = 0;

	/* The processor will run at 2MHz during an access cycle to the ROM */
	m_maincpu->set_clock_scale(1.0f);

	if ((m_ula.rompage & 0x0e) == m_rompages->read())
	{
		data = m_romi[m_ula.rompage & 0x01]->read_rom(offset);
	}
	else
	{
		switch (m_ula.rompage)
		{
		case 10:
			/* SP64 ROM utilises the spare BASIC ROM page */
			if (BIT(m_sp64_bank, 7) && (offset & 0x2000))
			{
				data = m_sp64_ram[offset & 0x1fff];
			}
			else
			{
				data = m_region_sp64->base()[(!BIT(m_sp64_bank, 0) << 14) | offset];
			}
			break;

		default:
			data = electron_state::electron_paged_r(offset);
			break;
		}
	}
	return data;
}

void electronsp_state::electron_paged_w(offs_t offset, uint8_t data)
{
	/* The processor will run at 2MHz during an access cycle to the ROM */
	m_maincpu->set_clock_scale(1.0f);

	if ((m_ula.rompage & 0x0e) == m_rompages->read())
	{
		/* TODO: sockets are writeable if RAM */
	}
	else
	{
		switch (m_ula.rompage)
		{
		case 10:
			/* SP64 ROM utilises the spare BASIC ROM page */
			if (BIT(m_sp64_bank, 7) && (offset & 0x2000))
			{
				m_sp64_ram[offset & 0x1fff] = data;
			}
			break;

		default:
			electronsp_state::electron_paged_w(offset, data);
			break;
		}
	}
}

uint8_t electron_state::electron_mos_r(offs_t offset)
{
	/* The processor will run at 2MHz during an access cycle to the ROM */
	m_maincpu->set_clock_scale(1.0f);

	return m_region_mos->base()[0x4000 | offset];
}

void electron_state::electron_mos_w(offs_t offset, uint8_t data)
{
	/* The processor will run at 2MHz during an access cycle to the ROM */
	m_maincpu->set_clock_scale(1.0f);

	logerror("MOS: write %04x %02x\n", offset + 0xc000, data);
	m_exp->expbus_w(0xc000 + offset, data);
}

uint8_t electron_state::electron_fred_r(offs_t offset)
{
	/* The processor will run at 2MHz during an access cycle to the ROM */
	m_maincpu->set_clock_scale(1.0f);

	/* The Issue 4 ULA returns data from OS ROM, whereas Issue 6 ULA will return 0xff */
	//logerror("FRED: read fc%02x\n", offset);
	return m_exp->expbus_r(0xfc00 + offset);
}

void electron_state::electron_fred_w(offs_t offset, uint8_t data)
{
	/* The processor will run at 2MHz during an access cycle to the ROM */
	m_maincpu->set_clock_scale(1.0f);

	/* Master RAM Board */
	if (offset == 0x7f) m_mrb_mapped = !(data & 0x80);

	//logerror("FRED: write fc%02x\n", offset);
	m_exp->expbus_w(0xfc00 + offset, data);
}

uint8_t electronsp_state::electron_fred_r(offs_t offset)
{
	uint8_t data = 0;

	/* The processor will run at 2MHz during an access cycle to the ROM */
	m_maincpu->set_clock_scale(1.0f);

	if ((offset & 0xf0) == 0xb0)
	{
		data = m_via->read(offset & 0x0f);
	}
	else
	{
		data = electron_state::electron_fred_r(offset);
	}
	return data;;
}

void electronsp_state::electron_fred_w(offs_t offset, uint8_t data)
{
	/* The processor will run at 2MHz during an access cycle to the ROM */
	m_maincpu->set_clock_scale(1.0f);

	electron_state::electron_fred_w(offset, data);

	if ((offset & 0xf0) == 0xb0)
	{
		m_via->write(offset & 0x0f, data);
	}
	else if (offset == 0xfa)
	{
		m_sp64_bank = data;
	}
}

uint8_t electron_state::electron_jim_r(offs_t offset)
{
	/* The processor will run at 2MHz during an access cycle to the ROM */
	m_maincpu->set_clock_scale(1.0f);

	/* The Issue 4 ULA returns data from OS ROM, whereas Issue 6 ULA will return 0xff */
	//logerror("JIM: read fd%02x\n", offset);
	return m_exp->expbus_r(0xfd00 + offset);
}

void electron_state::electron_jim_w(offs_t offset, uint8_t data)
{
	/* The processor will run at 2MHz during an access cycle to the ROM */
	m_maincpu->set_clock_scale(1.0f);

	//logerror("JIM: write fd%02x\n", offset);
	m_exp->expbus_w(0xfd00 + offset, data);
}

uint8_t electron_state::electron_sheila_r(offs_t offset)
{
	/* The processor will run at 2MHz during an access cycle to the ROM */
	m_maincpu->set_clock_scale(1.0f);

	/* The Issue 4 ULA returns data from OS ROM, whereas Issue 6 ULA will return 0xfe */
	uint8_t data = 0xff;

	switch ( offset & 0x0f )
	{
	case 0x00:  /* Interrupt status */
		data = m_ula.interrupt_status;
		m_ula.interrupt_status &= ~0x02;
		break;
	case 0x01:  /* Unknown */
		break;
	case 0x04:  /* Cassette data shift register */
		electron_interrupt_handler(INT_CLEAR, INT_RECEIVE_FULL );
		data = m_ula.tape_byte;
		break;
	}
	//logerror( "ULA: read fe%02x: %02x\n", offset, data );
	return data;
}

static const int electron_palette_offset[4] = { 0, 4, 5, 1 };
static const uint16_t electron_screen_base[8] = { 0x3000, 0x3000, 0x3000, 0x4000, 0x5800, 0x5800, 0x6000, 0x6000 };
static const int electron_mode_end[8] = { 255, 255, 255 ,249 ,255, 255, 249, 249 };

void electron_state::electron_sheila_w(offs_t offset, uint8_t data)
{
	/* The processor will run at 2MHz during an access cycle to the ROM */
	m_maincpu->set_clock_scale(1.0f);

	m_exp->expbus_w(0xfe00 + offset, data);

	int i = electron_palette_offset[(( offset >> 1 ) & 0x03)];
	//logerror( "ULA: write fe%02x <- %02x\n", offset & 0x0f, data );
	switch( offset & 0x0f )
	{
	case 0x00:  /* Interrupt control */
		m_ula.interrupt_control = data;
		break;
	case 0x01:  /* Unknown */
		break;
	case 0x02:  /* Screen start address #1 */
		m_ula.screen_start = ( m_ula.screen_start & 0x7e00 ) | ( ( data & 0xe0 ) << 1 );
		logerror( "screen_start changed to %04x\n", m_ula.screen_start );
		break;
	case 0x03:  /* Screen start address #2 */
		m_ula.screen_start = ( m_ula.screen_start & 0x1ff ) | ( ( data & 0x3f ) << 9 );
		logerror( "screen_start changed to %04x\n", m_ula.screen_start );
		break;
	case 0x04:  /* Cassette data shift register */
		break;
	case 0x05:  /* Interrupt clear and paging */
		/* rom page requests are honoured when currently bank 0-7 or 12-15 is switched in,
		 * or when 8-11 is currently switched in only switching to bank 8-15 is allowed.
		 *
		 * Rompages 10 and 11 both select the Basic ROM.
		 * Rompages 8 and 9 both select the keyboard.
		 */
		if ( ( ( m_ula.rompage & 0x0C ) != 0x08 ) || ( data & 0x08 ) )
		{
			m_ula.rompage = data & 0x0f;
			if ( m_ula.rompage == 8 || m_ula.rompage == 9 )
			{
				m_ula.rompage = 8;
			}
		}
		if ( data & 0x10 )
		{
			electron_interrupt_handler( INT_CLEAR, INT_DISPLAY_END );
		}
		if ( data & 0x20 )
		{
			electron_interrupt_handler( INT_CLEAR, INT_RTC );
		}
		if ( data & 0x40 )
		{
			electron_interrupt_handler( INT_CLEAR, INT_HIGH_TONE );
		}
		if ( data & 0x80 )
		{
		}
		break;
	case 0x06:  /* Counter divider */
		if ( m_ula.communication_mode == 0x01)
		{
		/* GUESS
		 * the Advanced Users manual says this is the correct algorithm
		 * but the divider is wrong(?), says 16 but results in high pitch,
		 * 32 is more close
		 */
			m_beeper->set_clock( 1000000 / ( 32 * ( data + 1 ) ) );
		}
		break;
	case 0x07:  /* Misc. */
		m_ula.communication_mode = ( data >> 1 ) & 0x03;
		switch( m_ula.communication_mode )
		{
		case 0x00:  /* cassette input */
			m_beeper->set_state( 0 );
			electron_tape_start();
			break;
		case 0x01:  /* sound generation */
			m_beeper->set_state( 1 );
			electron_tape_stop();
			break;
		case 0x02:  /* cassette output */
			m_beeper->set_state( 0 );
			electron_tape_stop();
			break;
		case 0x03:  /* not used */
			m_beeper->set_state( 0 );
			electron_tape_stop();
			break;
		}
		m_ula.screen_mode = ( data >> 3 ) & 0x07;
		m_ula.screen_base = electron_screen_base[ m_ula.screen_mode ];
		m_ula.screen_size = 0x8000 - m_ula.screen_base;
		m_ula.screen_dispend = electron_mode_end[ m_ula.screen_mode ];
		logerror( "ULA: screen mode set to %d\n", m_ula.screen_mode );
		m_ula.cassette_motor_mode = ( data >> 6 ) & 0x01;
		m_cassette->change_state(m_ula.cassette_motor_mode ? CASSETTE_MOTOR_ENABLED : CASSETTE_MOTOR_DISABLED, CASSETTE_MOTOR_DISABLED );
		m_ula.capslock_mode = ( data >> 7 ) & 0x01;
		m_capslock_led = m_ula.capslock_mode;
		break;
	case 0x08: case 0x0a: case 0x0c: case 0x0e:
		/* colour palette */
		m_ula.current_pal[i+10] = (m_ula.current_pal[i+10] & 0x01) | (((data & 0x80) >> 5) | ((data & 0x08) >> 2));
		m_ula.current_pal[i+8] = (m_ula.current_pal[i+8] & 0x01) | (((data & 0x40) >> 4) | ((data & 0x04) >> 1));
		m_ula.current_pal[i+2] = (m_ula.current_pal[i+2] & 0x03) | ((data & 0x20) >> 3);
		m_ula.current_pal[i] = (m_ula.current_pal[i] & 0x03) | ((data & 0x10) >> 2);
		break;
	case 0x09: case 0x0b: case 0x0d: case 0x0f:
		/* colour palette */
		m_ula.current_pal[i+10] = (m_ula.current_pal[i+10] & 0x06) | ((data & 0x08) >> 3);
		m_ula.current_pal[i+8] = (m_ula.current_pal[i+8] & 0x06) | ((data & 0x04) >> 2);
		m_ula.current_pal[i+2] = (m_ula.current_pal[i+2] & 0x04) | (((data & 0x20) >> 4) | ((data & 0x02) >> 1));
		m_ula.current_pal[i] = (m_ula.current_pal[i] & 0x04) | (((data & 0x10) >> 3) | ((data & 0x01)));
		break;
	}
}

void electron_state::electron_interrupt_handler(int mode, int interrupt)
{
	if ( mode == INT_SET )
	{
		m_ula.interrupt_status |= interrupt;
	}
	else
	{
		m_ula.interrupt_status &= ~interrupt;
	}
	if ( m_ula.interrupt_status & m_ula.interrupt_control & ~0x83 )
	{
		m_ula.interrupt_status |= 0x01;
		m_irqs->in_w<0>(ASSERT_LINE);
	}
	else
	{
		m_ula.interrupt_status &= ~0x01;
		m_irqs->in_w<0>(CLEAR_LINE);
	}
}

/**************************************
   Machine Initialisation functions
***************************************/

TIMER_CALLBACK_MEMBER(electron_state::setup_beep)
{
	m_beeper->set_state( 0 );
	m_beeper->set_clock( 300 );
}

void electron_state::machine_start()
{
	m_capslock_led.resolve();

	m_ula.interrupt_status = 0x82;
	m_ula.interrupt_control = 0x00;
	timer_set(attotime::zero, TIMER_SETUP_BEEP);
	m_tape_timer = timer_alloc(TIMER_TAPE_HANDLER);

	/* register save states */
	save_item(STRUCT_MEMBER(m_ula, interrupt_status));
	save_item(STRUCT_MEMBER(m_ula, interrupt_control));
	save_item(STRUCT_MEMBER(m_ula, rompage));
	save_item(STRUCT_MEMBER(m_ula, screen_start));
	save_item(STRUCT_MEMBER(m_ula, screen_base));
	save_item(STRUCT_MEMBER(m_ula, screen_size));
	save_item(STRUCT_MEMBER(m_ula, screen_addr));
	save_item(STRUCT_MEMBER(m_ula, screen_dispend));
	save_item(STRUCT_MEMBER(m_ula, current_pal));
	save_item(STRUCT_MEMBER(m_ula, communication_mode));
	save_item(STRUCT_MEMBER(m_ula, screen_mode));
	save_item(STRUCT_MEMBER(m_ula, cassette_motor_mode));
	save_item(STRUCT_MEMBER(m_ula, capslock_mode));
	save_item(STRUCT_MEMBER(m_ula, last_tap_val));
	save_item(STRUCT_MEMBER(m_ula, tap_val_length));
	save_item(STRUCT_MEMBER(m_ula, len));
	save_item(NAME(m_mrb_mapped));
	save_item(NAME(m_vdu_drivers));
}

void electron_state::machine_reset()
{
	m_ula.communication_mode = 0x04;
	m_ula.screen_mode = 0;
	m_ula.cassette_motor_mode = 0;
	m_ula.capslock_mode = 0;
	m_ula.screen_start = 0x3000;
	m_ula.screen_base = 0x3000;
	m_ula.screen_size = 0x8000 - 0x3000;
	m_ula.screen_addr = 0x3000;
	m_ula.tape_running = 0;

	m_mrb_mapped = true;
	m_vdu_drivers = false;
}

void electronsp_state::machine_start()
{
	electron_state::machine_start();

	m_sp64_ram = std::make_unique<uint8_t[]>(0x2000);

	/* register save states */
	save_item(NAME(m_sp64_bank));
	save_pointer(NAME(m_sp64_ram), 0x2000);
}


image_init_result electronsp_state::load_rom(device_image_interface &image, generic_slot_device *slot)
{
	uint32_t size = slot->common_get_size("rom");

	// socket accepts 8K and 16K ROM only
	if (size != 0x2000 && size != 0x4000)
	{
		image.seterror(image_error::INVALIDIMAGE, "Invalid size: Only 8K/16K is supported");
		return image_init_result::FAIL;
	}

	slot->rom_alloc(0x4000, GENERIC_ROM8_WIDTH, ENDIANNESS_LITTLE);
	slot->common_load_rom(slot->get_rom_base(), size, "rom");

	// mirror 8K ROMs
	uint8_t *crt = slot->get_rom_base();
	if (size <= 0x2000) memcpy(crt + 0x2000, crt, 0x2000);

	return image_init_result::PASS;
}
