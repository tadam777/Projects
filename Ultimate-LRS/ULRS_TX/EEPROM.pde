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

#define eeprom_start_address 0

unsigned int read_eeprom_uint(int address)
{
	return (EEPROM.read(eeprom_start_address + address) * 256) + EEPROM.read(eeprom_start_address + address + 1);
}

unsigned char read_eeprom_uchar(int address)
{
	return  EEPROM.read(eeprom_start_address + address);
}

void write_eeprom_uint(int address, unsigned int value)
{
	EEPROM.write(eeprom_start_address + address, value / 256);
	EEPROM.write(eeprom_start_address + address + 1, value % 256);
}

void write_eeprom_uchar(int address, unsigned char value)
{
	return  EEPROM.write(eeprom_start_address + address, value);
}

void load_failsafe_values()
{
	for (int i = 0; i < 8; i++)
	{
		Servo_Buffer[i] = (EEPROM.read(eeprom_start_address + 100 + (2 * i)) * 256) + EEPROM.read(eeprom_start_address + 101 + (2 * i));
	}
}

void save_failsafe_values(void)
{
	for (int i = 0; i < 8; i++)
	{
		EEPROM.write(eeprom_start_address + 100 + (2 * i), Servo_Buffer[i] / 256);
		EEPROM.write(eeprom_start_address + 101 + (2 * i), Servo_Buffer[i] % 256);
	}
}

// not used
void read_eeprom(void)
{
	CARRIER_FREQUENCY = 400000 + read_eeprom_uint(0);
	HOPPING_STEP_SIZE = read_eeprom_uchar(2);
	hop_list[0] = read_eeprom_uchar(3);
	hop_list[1] = read_eeprom_uchar(4);
	hop_list[2] = read_eeprom_uchar(5);
	RF_Header[0] = read_eeprom_uchar(6);
	RF_Header[1] = read_eeprom_uchar(7);
	RF_Header[2] = read_eeprom_uchar(8);
	RF_Header[3] = read_eeprom_uchar(9);
}


void write_eeprom(void)
{
	write_eeprom_uint(0, CARRIER_FREQUENCY - 400000) ;
}

// not used
void eeprom_check(void)
{
	byte temp1, temp2, temp3;
	Serial.begin(9600);
	Serial.print("W4E"); 
	delay(100);

	if (Serial.available() > 2) 
	{
		if ((Serial.read() == 'R') && (Serial.read() == '4') && (Serial.read() == 'T'))
		{
			while (1)
			{
				if (Serial.available() > 2)
				{
					temp1 = Serial.read();

					if (temp1 == 'W')
					{
						temp2 = Serial.read();
						temp3 = Serial.read();
						temp1 = 0;

						while (1)
						{
							if (Serial.available() > 0)
							{
								temp3 = Serial.read();
								write_eeprom_uchar(temp1++, temp3); 
							}
						}
					}

					if (temp1 == 'R')
					{
						temp2 = Serial.read();
						temp3 = Serial.read();

						for (int i = 0; i < 100; i++)
						{
							Serial.write(read_eeprom_uchar(i));
						}

						Serial.write('0'); 
						Serial.write('0');
					}
				}
			}
		}

		delay(100);
	}

	read_eeprom();
	Serial.print(CARRIER_FREQUENCY);
	Serial.end();
}
