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

void setup()
{
	pinMode(Red_LED, OUTPUT);
	pinMode(Green_LED, OUTPUT);
	pinMode(BUZZER, OUTPUT);
	pinMode(BTN, INPUT);
	pinMode(SDO_pin, INPUT);
	pinMode(SDI_pin, OUTPUT);
	pinMode(SCLK_pin, OUTPUT);
	pinMode(IRQ_pin, INPUT);
	pinMode(nSel_pin, OUTPUT);
	pinMode(PPM_IN, INPUT);
	pinMode(RF_OUT_INDICATOR, OUTPUT);
	pinMode(RSSI_out_pin, OUTPUT);
	Serial.begin(SERIAL_BAUD_RATE);
	PPM_Pin_Interrupt_Setup

	for (unsigned char i = 0; i < 8; i++)
	{
		SetServoPos(i, 3000);
	}

	TCCR1B   =   0x00;
	TCNT1H   =   0x00;
	TCNT1L   =   0x00;
	ICR1     =   60005;
	TCCR1A   =   0x02;
	TCCR1B   =   0x1A;
}

void requestEvent()
{
	Wire.write(Rx_RSSI);
}

ISR(PPM_Signal_Interrupt)
{
	unsigned int time_temp;
	unsigned int servo_temp;
	unsigned int servo_temp2;

	if (PPM_Signal_Edge_Check)
	{
		time_temp = TCNT1;
		TCNT1 = 0;

		if (channel_no < 19)
		{
			channel_no++;
		}

		if (time_temp > 8000)
		{
			channel_count = channel_no;
			channel_no = 0;
			transmitted = 0;
		}

		else
		{
			if ((time_temp > 1500) && (time_temp < 4500))
			{
				Servo_Buffer[channel_no] =  time_temp;
			}
		}
	}
}

void loop()
{
	BuzzerHello();
	unsigned char i;
	RF22B_init_parameter();
	frequency_configurator(CARRIER_FREQUENCY);
	Power_Set(POWER);
	sei();
	digitalWrite(BUZZER, LOW);
	digitalWrite(BTN, HIGH);
	delay(100);
	Check_Button();
	digitalWrite(BUZZER, LOW);
	digitalWrite(RF_OUT_INDICATOR, LOW);
	digitalWrite(PPM_IN, HIGH);
	transmitted = 0;
	rx_reset();
	time = millis();
	old_time = time;

	while (1)
	{
		time = millis();

		if (_spi_read(0x0C) == 0)
		{
			RF22B_init_parameter();
			frequency_configurator(CARRIER_FREQUENCY);
			rx_reset();
		}

		rssi();

		if (nIRQ_0)
		{
			red_led(true);

			if (debug)
			{
				Serial.print(" AFC ");
				Serial.println((signed char)_spi_read(0x2B), DEC);
			}

			send_read_address(0x7f);

			for (i = 0; i < RF_PACK_SIZE; i++)
			{
				RF_Rx_Buffer[i] = read_8bit_data();
			}

			rx_reset();

			if (RF_Rx_Buffer[0] == 'B')
			{
				if (!debug)
				{
					for (i = 2; i < RF_Rx_Buffer[1] + 2; i++)
					{
						Serial.write(RF_Rx_Buffer[i]);
					}
				}
			}

			Rx_Pack_Received = 0;
			LostSignal = false; // Reset LostSignal to false (has signal)

			red_led(false);
		}

		if (time > old_time + 30)
		{
			old_time = time;
			fill_fifo();
			byte total_rx_byte = fifo_available();

			if (total_rx_byte > 0)
			{
				RF_Tx_Buffer[0] = 'B';

				if (total_rx_byte > RF_PACK_SIZE_UP - 2)
				{
					total_rx_byte = RF_PACK_SIZE_UP - 2;
				}

				RF_Tx_Buffer[1] = total_rx_byte;

				for (byte i = 0; i < total_rx_byte; i++)
				{
					RF_Tx_Buffer[2 + i] = fifo_read();
				}
			}

			else
			{
				RF_Tx_Buffer[0] = 'S';

				for (i = 0; i < RC_CHANNEL_COUNT; i++)
				{
					RF_Tx_Buffer[(i * 2) + 1] = Servo_Buffer[i + 1] / 256;
					RF_Tx_Buffer[(i * 2) + 2] = Servo_Buffer[i + 1] % 256;

					if (debug)
					{
						Serial.print("s");
						Serial.print(i);
						Serial.print("=");
						Serial.print((float) Servo_Buffer[i + 1] / 2.0);
						Serial.print(" ");
					}
				}

				if (debug)
				{
					Serial.println();
				}
			}

			if (digitalRead(BTN) == 0)
			{
				RF_Tx_Buffer[0] = 'C';
				RF_Tx_Buffer[1] = 'F';
			}

			to_tx_mode();
			transmitted = 1;

			if (debug && false)
			{
				for (int i = 0; i < 8; i++)
				{
					Serial.print(" ");
					Serial.print(i);
					Serial.print("=");
					Serial.print(Servo_Buffer[i]);
				}

				Serial.println("");
			}

			Hopping();
			rx_reset();

			if (Rx_Pack_Received >= Lost_Package_Alert + 50) // if lost +50 more packets than Lost_Package_Alert
			{
				LostSignal = true; // Set LostSignal to true (Lost Signal)
			}

			if (Rx_Pack_Received >= Lost_Package_Alert)
			{
				Rx_RSSI = 0;
				analogWrite(RSSI_out_pin, 0);

				if (IsBuzzerEnabled && LinkInit)
				{
					unsigned long currentMillis = millis();

				if (LostSignal) // if lost +50 more packets than Lost_Package_Alert
				{
				if (currentMillis - buzpreviousMillis >= buzinterval) // Buz at buzinterval rate
				{
				buzpreviousMillis = currentMillis;
				for (int i = 0; i < 3; i++)
				{
					digitalWrite(BUZZER, HIGH);
					delay(20);
					digitalWrite(BUZZER, LOW);
					delay(5);
				}
					}
				}

				else // Lost packets
				{
					digitalWrite(BUZZER, HIGH); // Less than Lost_Package_Alert + 50, so Buzzer full on
				}
				}
			}

			else
			{
				digitalWrite(BUZZER, LOW); // Buzzer disalbed
			}

			Rx_Pack_Received++;
		}
	}
}

// not used
void I2C()
{
	Wire.beginTransmission(12);
	Wire.write('A');
	Wire.endTransmission();
}

// bip bip bip...
void BuzzerHello()
{
	for (int i = 0; i < 4; i++)
	{
		digitalWrite(BUZZER, HIGH);
		delay(150);
		digitalWrite(BUZZER, LOW);
		delay(30);
	}
}

void rssi()
{
	if (((_spi_read(0x31) & 0b00010000) > 0) && (time - rssi_timer > 100))
	{
		Rx_RSSI = _spi_read(0x26);
		rssi_timer = time;
		Rx_RSSI = constrain(Rx_RSSI, 80, 210);
		analogWrite(RSSI_out_pin, map(Rx_RSSI, 80, 210, 0, 255));
	}
}