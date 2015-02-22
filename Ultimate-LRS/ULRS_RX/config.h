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
void  Hopping(void);

#define POWER 7
bool debug = false;
long rssi_timer = 0;

enum board_type_enum
{
	RX_V1_BOARD,
	RX_V2_BOARD,
	TX_BOARD
};

board_type_enum rx_board = TX_BOARD; // TX_BOARD and RX_V2 are supported, to compile 

enum rf_rate_enum
{
	rate_57K,
	rate_125K,
	rate_125KOOK
};

rf_rate_enum rf_rate = rate_57K;

enum serial_ppm_type_enum
{
	CH8,
	MIXED
};

const int low_pulse_width = 300;
serial_ppm_type_enum ppm_type = MIXED;

#define PC_CONFIGURATION_ENABLED 0
#define DEBUG_MODE 0

unsigned long CARRIER_FREQUENCY = 432000;
unsigned char HOPPING_STEP_SIZE = 6;
static unsigned char hop_list[3] = { 45, 50, 55};
static unsigned char RF_Header[4] = {'F', 'L', 'I', 'P'};

#define SERIAL_BAUD_RATE 19200

#define AILERON 0
#define ELEVATOR 1
#define THROTTLE 2
#define RUDDER 3
#define RETRACTS 4
#define FLAPS 5
#define AUX1 6
#define AUX2 7

// not used
#define DUAL_AILERON_SERVO 4
#define DUAL_AILERON_DIRECTION -1

// not used
#define Gyro_Roll_Gain 5
#define Gyro_Pitch_Gain 5
#define Gyro_Yaw_Gain 5

#define RF_PACK_SIZE 64
#define RF_PACK_SIZE_UP 40
#define RC_CHANNEL_COUNT 18

// fifo serial

#define FIFO_SIZE 1000 // secret ingredient
unsigned char Input_Fifo[FIFO_SIZE];
int writepos = 0;
int readpos = 0;
int fifo_len = 0;

unsigned char RF_Rx_Buffer[RF_PACK_SIZE];
unsigned char RF_Tx_Buffer[RF_PACK_SIZE];
unsigned char RS232_Tx_Buffer[RF_PACK_SIZE] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};	
unsigned int Servo_Buffer[RC_CHANNEL_COUNT] = {3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000};	
unsigned int Servo_Position[RC_CHANNEL_COUNT] = {3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000};	
static unsigned char Servo_Number = 0;
unsigned int total_ppm_time = 0;
unsigned short Rx_RSSI, vbat = 0;

static unsigned char receiver_mode = 0;
static unsigned char hopping_channel = 1;

unsigned char temp_char;
unsigned int temp_int;

unsigned long time;
unsigned long last_pack_time ;
unsigned long last_hopping_time;
unsigned char failsafe_mode = 0;

volatile unsigned char RF_Mode = 0;
#define Available 0
#define Transmit 1
#define Transmitted 2
#define Receive 3
#define Received 4

volatile bool Preamble = false;

unsigned char loop_counter = 0;

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//### OrangeRx RX 100mW --> uncomment to compile for 100 mW RX (not recommended, as it makes an asymetrical link 1W - 100mW)
/*

#define SDO_pin A0
#define SDI_pin A1
#define SCLK_pin A2
#define IRQ_pin 2
#define nSel_pin 4
#define IRQ_interrupt 0

#define  nIRQ_1 (PIND & 0x04)==0x04 //D2
#define  nIRQ_0 (PIND & 0x04)==0x00 //D2

#define  nSEL_on PORTD |= 0x10 //D4
#define  nSEL_off PORTD &= 0xEF //D4

#define  SCK_on PORTC |= 0x04 //C2
#define  SCK_off PORTC &= 0xFB //C2

#define  SDI_on PORTC |= 0x02 //C1
#define  SDI_off PORTC &= 0xFD //C1

#define  SDO_1 (PINC & 0x01) == 0x01 //C0
#define  SDO_0 (PINC & 0x01) == 0x00 //C0

#define GREEN_LED_pin 13
#define RED_LED_pin A3

#define Red_LED_ON  PORTC |= _BV(3);
#define Red_LED_OFF  PORTC &= ~_BV(3);

#define Green_LED_ON  PORTB |= _BV(5);
#define Green_LED_OFF  PORTB &= ~_BV(5);

#define PPM_out 7 
#define Servo_Ports_LOW digitalWrite(PPM_out,LOW);

#define Serial_PPM_OUT_HIGH digitalWrite(PPM_out,HIGH);

#define RSSI_out_pin 3
*/

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//### 1W Tx Board --> comment this block to compile for 100 mW RX (not recommended, as it makes an asymetrical link 1W - 100mW)

#define SDO_pin 9
#define SDI_pin 8
#define SCLK_pin 7

#define RSSI_out_pin 5

#define PPM_out 3

#define IRQ_pin 2
#define nSel_pin 4
#define IRQ_interrupt 0  

#define  nIRQ_1 (PIND & 0x04)==0x04 
#define  nIRQ_0 (PIND & 0x04)==0x00 

#define  nSEL_on PORTD |= (1<<4) 
#define  nSEL_off PORTD &= 0xEF 

#define  SCK_on PORTD |= 0b10000000 
#define  SCK_off PORTD &= 0b01111111 

#define  SDI_on PORTB |= 0b00000001 
#define  SDI_off PORTB &= 0b11111110 

#define  SDO_1 (PINB & 0b00000010) == 0b00000010 
#define  SDO_0 (PINB & 0b00000010) == 0b00000000 
#define GREEN_LED_pin 12
#define RED_LED_pin 13
#define Red_LED_ON  digitalWrite(RED_LED_pin, HIGH);
#define Red_LED_OFF  digitalWrite(RED_LED_pin, LOW);
#define Green_LED_ON  digitalWrite(GREEN_LED_pin, HIGH)
#define Green_LED_OFF  digitalWrite(GREEN_LED_pin, LOW)
#define Servo_Ports_LOW digitalWrite(PPM_out,LOW);
#define Serial_PPM_OUT_HIGH digitalWrite(PPM_out,HIGH);

//---------------------------------------------------------------------

// not used
unsigned char raw_data[6];
int Gyro_YAW, Gyro_PITCH, Gyro_ROLL;
int Gyro_YAW_Zero, Gyro_PITCH_Zero, Gyro_ROLL_Zero;
int MagX, MagY, MagZ;
float AccX, AccY, AccZ;
int JoyX, JoyY;
int acc_offx = 0;
int acc_offy = 0;
int acc_offz = 0;
unsigned char GPS_data_status = 0;
char *parts[25];


#define Servo1_OUT 3 //Servo1
#define Servo3_OUT 6 //Servo3