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

void RF22B_init_parameter(void);
void _spi_write(unsigned char address, unsigned char data);
unsigned char _spi_read(unsigned char address);
unsigned char _spi_read(unsigned char address);
void send_read_address(unsigned char i);
unsigned char read_8bit_data(void);
void to_tx_mode(void);

#define POWER 7
bool debug = false;
bool IsBuzzerEnabled = true;

#define FIFO_SIZE 1000 // secret ingredient
unsigned char Input_Fifo[FIFO_SIZE];
int writepos = 0;
int readpos = 0;
int fifo_len = 0;

#define TX_BOARD_TYPE 2 // 1W TX only is supported
#define CONTROL_TYPE 0

unsigned long CARRIER_FREQUENCY = 432000;
unsigned char HOPPING_STEP_SIZE = 6;

#define FREQUENCY_HOPPING 1

static unsigned char hop_list[3] = { 45, 50, 55};

enum board_type_enum
{
	RX_V1_BOARD,
	RX_V2_BOARD,
	TX_BOARD
};

board_type_enum rx_board = TX_BOARD;

enum rf_rate_enum
{
	rate_57K,
	rate_125K,
	rate_125KOOK
};

rf_rate_enum rf_rate = rate_57K;

static unsigned char RF_Header[4] = {'F', 'L', 'I', 'P'};
#define SERIAL_BAUD_RATE 115200

#define Lost_Package_Alert 3

#define RF_PACK_SIZE 64
#define RF_PACK_SIZE_UP 40
#define RC_CHANNEL_COUNT 18

unsigned char RF_Rx_Buffer[RF_PACK_SIZE];
unsigned char RF_Tx_Buffer[RF_PACK_SIZE];

unsigned char Telemetry_Buffer[8];

volatile unsigned int Servo_Buffer[RC_CHANNEL_COUNT];

volatile unsigned char channel_no = 0;
volatile unsigned int transmitted = 1;
volatile unsigned char channel_count = 0;

static unsigned char hopping_channel = 1;
unsigned long time, old_time = 0, old_rssi = 0;

unsigned char Rx_Pack_Received = 0;
unsigned char Rx_RSSI = 110;
unsigned char Tx_RSSI = 110;

#define PPM_IN 3
#define RF_OUT_INDICATOR A0
#define BUZZER 10
#define BTN 11
#define Red_LED 13
#define Green_LED 12
#define Red_LED_ON  PORTB |= _BV(5);
#define Red_LED_OFF  PORTB &= ~_BV(5);
#define Green_LED_ON   PORTB |= _BV(4);
#define Green_LED_OFF  PORTB &= ~_BV(4);
#define PPM_Pin_Interrupt_Setup  PCMSK2 = 0b00001000;PCICR|=(1<<PCIE2);
#define PPM_Signal_Interrupt PCINT2_vect
#define PPM_Signal_Edge_Check (PIND & 0x08)==0x08
#define RSSI_out_pin 5
#define  nIRQ_1 (PIND & 0x04)==0x04
#define  nIRQ_0 (PIND & 0x04)==0x00
#define  nSEL_on PORTD |= (1<<4)
#define  nSEL_off PORTD &= 0xEF
#define  SCK_on PORTD |= (1<<7)
#define  SCK_off PORTD &= 0x7F
#define  SDI_on PORTB |= (1<<0)
#define  SDI_off PORTB &= 0xFE
#define  SDO_1 (PINB & 0x02) == 0x02
#define  SDO_0 (PINB & 0x02) == 0x00
#define SDO_pin 9
#define SDI_pin 8
#define SCLK_pin 7
#define IRQ_pin 2
#define nSel_pin 4

long rssi_timer = 0;