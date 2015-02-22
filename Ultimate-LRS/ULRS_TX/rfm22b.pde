/*****************************************************************************************************
***         Ultimate LRS - Long Range RC & Data Link using 2 x 1W 433 MHz OrangeRX modules         ***
***                                                                                                ***
*** Copyright 2014-2015 Benoit Joassart    benoit@joassart.com                                     ***
***                                        rcgroups.com : flipflap                                 ***
***                     Project website : http://www.itluxembourg.lu/site/                         ***
***                                                                                                ***
*** Contains code from OpenLRS ( http://code.google.com/p/openlrs/ )                               ***
***   Copyright 2010-2012 Melih Karakelle ( http://www.flytron.com ) (forum nick name: Flytron)    ***
***                     Jan-Dirk Schuitemaker ( http://www.schuitemaker.org/ ) (CrashingDutchman)  ***
***                     Etienne Saint-Paul ( http://www.gameseed.fr ) (forum nick name: Etienne)   ***
***                     thUndead (forum nick name: thUndead)                                       ***
***                                                                                                ***
******************************************************************************************************

					This file is part of Ultimate LRS.

					Ultimate LRS is free software: you can redistribute it and/or modify
					it under the terms of the GNU General Public License as published by
					the Free Software Foundation, either version 3 of the License, or
					(at your option) any later version.

					Ultimate LRS is distributed in the hope that it will be useful,
					but WITHOUT ANY WARRANTY; without even the implied warranty of
					MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
					GNU General Public License for more details.

					You should have received a copy of the GNU General Public License
					along with Ultimate LRS.  If not, see <http://www.gnu.org/licenses/>.

					This project must be compiled with Arduino 1.5.8.
*/

#define NOP() __asm__ __volatile__("nop")

#define RF22B_PWRSTATE_READY    01
#define RF22B_PWRSTATE_TX        0x09

#define RF22B_PWRSTATE_RX       05

#define RF22B_Rx_packet_received_interrupt   0b00000010
#define RF22B_PACKET_SENT_INTERRUPT  0b00000100

#define InterruptStatus1 0x03
#define iferr 		0b10000000
#define itxffafull 	0b01000000
#define itxffaem 	0b00100000
#define irxffafull 	0b00010000
#define iext 		0b00001000
#define ipksent 	0b00000100
#define ipkvalid 	0b00000010
#define icrcerror 	0b00000001

#define RF22B_PWRSTATE_POWERDOWN  00

unsigned char ItStatus1, ItStatus2;

// not used
typedef struct
{
	unsigned char reach_1s    : 1;
} FlagType;
FlagType               Flag;

unsigned char read_8bit_data(void);
void to_tx_mode(void);
void to_ready_mode(void);
void send_8bit_data(unsigned char i);
void send_read_address(unsigned char i);
void _spi_write(unsigned char address, unsigned char data);
void RF22B_init_parameter(void);
void port_init(void);
unsigned char _spi_read(unsigned char address);
void Write0(void);
void Write1(void);
void timer2_init(void);
void Write8bitcommand(unsigned char command);
void to_sleep_mode(void);

//*****************************************************************************

void clear_fifos()
{
	_spi_write(0x08, 0x03);
	_spi_write(0x08, 0x00);
}

//--------------------------------------------------------------
void Write0(void)
{
	SCK_off;
	NOP();
	SDI_off;
	NOP();
	SCK_on;
	NOP();
}
//--------------------------------------------------------------
void Write1(void)
{
	SCK_off;
	NOP();
	SDI_on;
	NOP();
	SCK_on;
	NOP();
}
//--------------------------------------------------------------
void Write8bitcommand(unsigned char command)
{
	unsigned char n = 8;
	nSEL_on;
	SCK_off;
	nSEL_off;

	while (n--)
	{
		if (command & 0x80)
		{
			Write1();
		}

		else
		{
			Write0();
		}

		command = command << 1;
	}

	SCK_off;
}

//--------------------------------------------------------------
unsigned char _spi_read(unsigned char address)
{
	unsigned char result;
	send_read_address(address);
	result = read_8bit_data();
	nSEL_on;
	return(result);
}

//--------------------------------------------------------------
void _spi_write(unsigned char address, unsigned char data)
{
	address |= 0x80;
	Write8bitcommand(address);
	send_8bit_data(data);
	nSEL_on;
}


void RF22B_init_parameter(void)
{
	ItStatus1 = _spi_read(0x03);
	ItStatus2 = _spi_read(0x04);
	_spi_write(0x06, 0x00);
	_spi_write(0x07, RF22B_PWRSTATE_READY);
	_spi_write(0x09, 0x7f);
	_spi_write(0x0a, 0x05);
	_spi_write(0x0b, 0x12);
	_spi_write(0x0c, 0x15);
	_spi_write(0x0d, 0xfd);
	_spi_write(0x0e, 0x00);
	_spi_write(0x70, 0x00);
	_spi_write(0x1d, 0b01000100);

	switch (rf_rate)
	{
		case rate_57K :
			_spi_write(0x1c, 0x05);
			_spi_write(0x20, 0x45);
			_spi_write(0x21, 0x01);
			_spi_write(0x22, 0xD7);
			_spi_write(0x23, 0xDC);
			_spi_write(0x24, 0x03);
			_spi_write(0x25, 0xB8);
			_spi_write(0x2a, 0x1e);
			_spi_write(0x6e, 0x0E);
			_spi_write(0x6f, 0xBF);
			_spi_write(0x71, 0x23);
			_spi_write(0x72, 0x2E);
			_spi_write(0x58, 0x80);
			break;

		case rate_125K :
			_spi_write(0x1c, 0x8a);
			_spi_write(0x20, 0x60);
			_spi_write(0x21, 0x01);
			_spi_write(0x22, 0x55);
			_spi_write(0x23, 0x55);
			_spi_write(0x24, 0x02);
			_spi_write(0x25, 0xad);
			_spi_write(0x2a, 0x1e);
			_spi_write(0x6e, 0x20);
			_spi_write(0x6f, 0x00);
			_spi_write(0x71, 0x23);
			_spi_write(0x72, 0xc8);
			_spi_write(0x1f, 0x03);
			_spi_write(0x58, 0xc0);
			break;

		case rate_125KOOK :
			_spi_write(0x1c, 0x81);
			_spi_write(0x20, 0x60);
			_spi_write(0x21, 0x01);
			_spi_write(0x22, 0x55);
			_spi_write(0x23, 0x55);
			_spi_write(0x24, 0x11);
			_spi_write(0x25, 0x57);
			_spi_write(0x2a, 0x44);
			_spi_write(0x2c, 0x28);
			_spi_write(0x2d, 0x0a);
			_spi_write(0x2e, 0x28);
			_spi_write(0x6e, 0x20);
			_spi_write(0x6f, 0x00);
			_spi_write(0x1f, 0x00);
			_spi_write(0x58, 0xc0);
			_spi_write(0x70, 0x0c);
			_spi_write(0x71, 0x21);
			_spi_write(0x72, 0x2e);
			break;
	}

	_spi_write(0x30, 0x8c);
	_spi_write(0x32, 0xf3);
	_spi_write(0x33, 0x42);
	_spi_write(0x34, 0x07);
	_spi_write(0x3a, RF_Header[0]);
	_spi_write(0x3b, RF_Header[1]);
	_spi_write(0x3c, RF_Header[2]);
	_spi_write(0x3d, RF_Header[3]);
	_spi_write(0x3e, RF_PACK_SIZE_UP);
	_spi_write(0x3f, RF_Header[0]);
	_spi_write(0x40, RF_Header[1]);
	_spi_write(0x41, RF_Header[2]);
	_spi_write(0x42, RF_Header[3]);
	_spi_write(0x69, 0b01110101);
	_spi_write(0x73, 0x00);
	_spi_write(0x74, 0x00);
	_spi_write(0x75, 0x53);
	_spi_write(0x76, 0x7D);
	_spi_write(0x77, 0x00);
	_spi_write(0x79, 0x00);
	_spi_write(0x7a, HOPPING_STEP_SIZE);
}

//--------------------------------------------------------------
void send_read_address(unsigned char i)
{
	i &= 0x7f;
	Write8bitcommand(i);
}
//--------------------------------------------------------------
void send_8bit_data(unsigned char i)
{
	unsigned char n = 8;
	SCK_off;

	while (n--)
	{
		if (i & 0x80)
		{
			Write1();
		}

		else
		{
			Write0();
		}

		i = i << 1;
	}

	SCK_off;
}
//--------------------------------------------------------------

unsigned char read_8bit_data(void)
{
	unsigned char Result, i;
	SCK_off;
	Result = 0;

	for (i = 0; i < 8; i++)
	{
		//read fifo data byte
		Result = Result << 1;
		SCK_on;
		NOP();

		if (SDO_1)
		{
			Result |= 1;
		}

		SCK_off;
		NOP();
	}

	return(Result);
}

//--------------------------------------------------------------
void rx_reset(void)
{
	_spi_write(0x07, RF22B_PWRSTATE_READY);
	_spi_write(0x7e, 36);
	_spi_write(0x08, 0x03);
	_spi_write(0x08, 0x00);
	_spi_write(0x07, RF22B_PWRSTATE_RX);
	_spi_write(0x05, RF22B_Rx_packet_received_interrupt);
	ItStatus1 = _spi_read(0x03);
	ItStatus2 = _spi_read(0x04);
}
//-----------------------------------------------------------------------

void to_rx_mode(void)
{
	to_ready_mode();
	delay(50);
	rx_reset();
	NOP();
}

void to_tx_mode(void)
{
	unsigned char i;
	blue_led(true);
	to_ready_mode();
	_spi_write(0x08, 0x03);
	_spi_write(0x08, 0x00);
	_spi_write(0x34, 7);
	_spi_write(0x3e, RF_PACK_SIZE_UP);

	for (i = 0; i < RF_PACK_SIZE_UP; i++)
	{
		_spi_write(0x7f, RF_Tx_Buffer[i]);
	}

	_spi_write(0x05, RF22B_PACKET_SENT_INTERRUPT);
	ItStatus1 = _spi_read(0x03);
	ItStatus2 = _spi_read(0x04);
	_spi_write(0x07, RF22B_PWRSTATE_TX);

	while (nIRQ_1);

	to_ready_mode();
	blue_led(false);
}

void to_ready_mode(void)
{
	ItStatus1 = _spi_read(0x03);
	ItStatus2 = _spi_read(0x04);
	_spi_write(0x07, RF22B_PWRSTATE_READY);
}

// not used
void to_sleep_mode(void)
{
	_spi_write(0x07, RF22B_PWRSTATE_READY);
	ItStatus1 = _spi_read(0x03);
	ItStatus2 = _spi_read(0x04);
	_spi_write(0x07, RF22B_PWRSTATE_POWERDOWN);
}

void frequency_configurator(long frequency)
{
	unsigned int frequency_constant = (frequency / 10000) - 24;
	frequency = frequency / 10;
	frequency = frequency - 24000;
	frequency =  frequency - (frequency_constant * 1000);
	frequency = frequency * 64;
	byte byte0 = (byte) frequency;
	byte byte1 = (byte)(frequency >> 8);
	_spi_write(0x75, 0x40 + frequency_constant);
	_spi_write(0x76, byte1);
	_spi_write(0x77, byte0);
}

bool crc_valid()
{
	return ((_spi_read(0x31) & 0b0000100) > 0);
}

void set_tx_almost_full_threshold(int threshold)
{
	_spi_write(0x7C, threshold & 0b00111111);
}

bool tx_buffer_almost_full()
{
	return((_spi_read(InterruptStatus1) & itxffafull) > 0);
}

void set_tx_almost_empty_threshold(int threshold)
{
	_spi_write(0x7D, threshold & 0b00111111);
}

bool tx_buffer_almost_empty()
{
	return((_spi_read(InterruptStatus1) & itxffaem) > 0);
}

void set_rx_almost_full_threshold(int threshold)
{
	_spi_write(0x7E, threshold & 0b00111111);
}

bool rx_buffer_almost_full()
{
	return((_spi_read(InterruptStatus1) & irxffafull) > 0);
}