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

#include "config.h"
#include <EEPROM.h>
#include <Wire.h>

// not used
String GPS_Latitude = "";
String GPS_Longitude = "";
String GPS_Altitude = "";
String GPS_Speed = "";
String GPS_Heading = "";
String GPS_Time = "";
String GPS_Date = "";
String GPS_Status = "";
String GPS_Mag_Variation = "";

// not used
float filtered_servo = 0;

void setup()
{
	pinMode(GREEN_LED_pin, OUTPUT);
	pinMode(RED_LED_pin, OUTPUT);
	pinMode(SDO_pin, INPUT);
	pinMode(SDI_pin, OUTPUT);
	pinMode(SCLK_pin, OUTPUT);
	pinMode(IRQ_pin, INPUT);
	pinMode(nSel_pin, OUTPUT);
	pinMode(0, INPUT);
	pinMode(1, OUTPUT);
	pinMode(PPM_out, OUTPUT);
	pinMode(RSSI_out_pin, OUTPUT);

	if (debug)
	{
		Serial.begin(115200);
	}

	else
	{
		Serial.begin(SERIAL_BAUD_RATE);
	}

	INIT_SERVO_DRIVER();
	attachInterrupt(IRQ_interrupt, RFM22B_Int, FALLING);
}

volatile unsigned char current = 0;

void InterruptInMicros(int m)
{
	TCNT1 = 40000 - (2 * m  - 2 * 13);
}

unsigned char i, tx_data_length;

void loop()
{
	receiver_mode = check_modes();
	load_failsafe_values();
	Power_Set(POWER);
	RF22B_init_parameter();
	frequency_configurator(CARRIER_FREQUENCY);
	_spi_write(0x79, hop_list[hopping_channel]);
	to_rx_mode();
	sei();
	Hopping();
	RF_Mode = Receive;
	time = millis();
	last_pack_time = time;
	last_hopping_time = time;

	while (1)
	{
		if (debug && false)
		{
			unsigned char ch = 50;

			while (1)
			{
				_spi_write(0x79, ch);
				Serial.print(" CH ");
				Serial.print(ch, DEC);
				Serial.print(" RSSI ");
				int rssi = (int) _spi_read(0x26);
				Serial.print(rssi, DEC);
				Serial.print(" AFC ");
				Serial.println((signed char)_spi_read(0x2B), DEC);
			}
		}

		time = millis();

		if (debug)
		{
			Serial.print(time, DEC);
			Serial.print(" CH ");
			Serial.print(hopping_channel, DEC);
			Serial.print(" RSSI ");
			int rssi = (int) _spi_read(0x26);
			Serial.print("   ");

			for (int i = 0; i < 3; i++)
			{
				Serial.print(i, DEC);
				Serial.print(" ");

				if (hopping_channel == i)
				{
					Serial.print(rssi, 1);
				}

				else
				{
					Serial.print("-");
				}

				Serial.print(" ");
			}

			Serial.print(" AFC ");
			Serial.println((signed char)_spi_read(0x2B), DEC);
			Serial.print(" CH ");
			Serial.print(hop_list[hopping_channel], DEC);
			Serial.print(" AGC ");
			Serial.print(_spi_read(0x69) & 0b00001111, DEC);
			Serial.println();
		}

		if (_spi_read(0x0C) == 0)
		{
			RF22B_init_parameter();
			frequency_configurator(CARRIER_FREQUENCY);
			_spi_write(0x79, hop_list[hopping_channel]);
			to_rx_mode();

			if (debug)
			{
				Serial.println("locked");
			}
		}

		if ((time - last_pack_time > 5000))
		{
			RF22B_init_parameter();
			frequency_configurator(CARRIER_FREQUENCY);
			_spi_write(0x79, hop_list[hopping_channel]);
			to_rx_mode();
			last_pack_time = time;

			if (debug)
			{
				Serial.println("reboot");
			}
		}

		if ((time - last_pack_time > 1000) && (failsafe_mode == 0))
		{
			failsafe_mode = 1;
			INIT_SERVO_DRIVER();
			load_failsafe_values();
			Direct_Servo_Drive();
			analogWrite(RSSI_out_pin, 0);

			if (debug)
			{
				Serial.println("failsafe");
			}
		}

		rssi();

		if ((time - last_hopping_time > 30 + 17))
		{
			last_hopping_time = time;
			analogWrite(RSSI_out_pin, 0);
			Hopping();
		}

		if (RF_Mode == Received)
		{
			received();
			Hopping();
			write();
		}
	}
}

void received()
{
	red_led(true);

	if (debug && false)
	{
		info_time();
		info('R');

		for (int i = 0; i < 3; i++)
		{
			Serial.print(Servo_Buffer[i] * (0.5f), 2);
			Serial.print(" ");
		}
	}

	if ((_spi_read(0x31) & 0b0000100) > 0)
	{
		info('!');
	}

	else
	{
		info('K');
	}

	failsafe_mode = 0;
	last_pack_time = time;
	send_read_address(0x7f);

	for (i = 0; i < RF_PACK_SIZE_UP; i++)
	{
		RF_Rx_Buffer[i] = read_8bit_data();
	}

	rx_reset();

	if (RF_Rx_Buffer[0] == 'S')
	{
		for (i = 0; i < RC_CHANNEL_COUNT; i++)
		{
			temp_int = (256 * RF_Rx_Buffer[1 + (2 * i)]) + RF_Rx_Buffer[2 + (2 * i)];

			if ((temp_int > 1500) && (temp_int < 4500))
			{
				Servo_Buffer[i] = temp_int ;
			}
		}
	}

	if (RF_Rx_Buffer[0] == 'C')
	{
		if (RF_Rx_Buffer[1] == 'F')
		{
			save_failsafe_values();
		}
	}

	if (RF_Rx_Buffer[0] == 'T')
	{
		tx_data_length = RF_Rx_Buffer[1];

		for (i = 0; i < tx_data_length; i++)
		{
			RS232_Tx_Buffer[i] = RF_Rx_Buffer[i + 2];
		}
	}

	if (!debug)
	{
		if (RF_Rx_Buffer[0] == 'B')
		{
			for (i = 2; i < RF_Rx_Buffer[1] + 2; i++)
			{
				Serial.write(RF_Rx_Buffer[i]);
			}
		}
	}

	Direct_Servo_Drive();
	red_led(false);
}

void write()
{
	info('W');
	delay(1);
	Telemetry_Bridge_Write();
	RF_Mode = Receive;
	last_hopping_time = time;
}

void rssi()
{
	if (((_spi_read(0x31) & 0b00010000) > 0) && (time - rssi_timer > 100))
	{
		Rx_RSSI = _spi_read(0x26);
		rssi_timer = time;
		Rx_RSSI = constrain(Rx_RSSI, 80, 210);
		analogWrite(RSSI_out_pin, map(Rx_RSSI, 80, 210, 0, 255));

		if (debug && false)
		{
			Serial.print("RSSI:");
			Serial.print(Rx_RSSI);
			Serial.print(" ");
			Serial.print((float) Rx_RSSI / 1.9 - 130.0);
			Serial.println(" dBm");
		}
	}
}