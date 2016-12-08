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

// not used
void Red_LED_Blink(unsigned short blink_count)
{
	unsigned char i;

	for (i = 0; i < blink_count; i++)
	{
		delay(100);
		Red_LED_ON;
		delay(100);
		Red_LED_OFF;
	}
}

// not used
void Green_LED_Blink(unsigned short blink_count)
{
	unsigned char i;

	for (i = 0; i < blink_count; i++)
	{
		delay(100);
		Green_LED_ON;
		delay(100);
		Green_LED_OFF;
	}
}

void Hopping(void)
{
	hopping_channel++;

	if (hopping_channel > 2)
	{
		hopping_channel = 0;
	}

	_spi_write(0x79, hop_list[hopping_channel]);

	if (debug)
	{
	}
}

void Power_Set(unsigned short level)
{
	if (level < 8)
	{
		_spi_write(0x6d, level | 0b00001000);
	}
}

void Check_Button(void)
{
	unsigned long loop_time;
	byte btn_mode = 0;

	if (digitalRead(BTN) == 0)
	{
		delay(1000);
		digitalWrite(BUZZER, LOW);
		time = millis();
		loop_time = time;

		while (digitalRead(BTN) == 0)
		{
			loop_time = millis();

			if (loop_time > time + 1000)
			{
				time = millis();
				btn_mode++;
				digitalWrite(BUZZER, HIGH);
				delay(100);
				digitalWrite(BUZZER, LOW);
			}
		}

		if (btn_mode == 0)
		{
			Power_Set(6);
		}

		if (btn_mode == 3)
		{
			Power_Set(0);
		}

		delay(500);
		digitalWrite(BUZZER, HIGH);
		delay(500);
		digitalWrite(BUZZER, LOW);
	}
}

// not used
void Binding_Mode(unsigned int btn_press_time)
{
}

void SetServoPos(unsigned char channel, int value)
{
	unsigned char ch = channel * 2;
	Servo_Buffer[ch] = value / 256;
	Servo_Buffer[ch + 1] = value % 256;
}

// not used
void march_beep(int frequencyInHertz, long timeInMilliseconds)
{
	digitalWrite(Red_LED, HIGH);
	int x;
	long delayAmount = (long)(1000000 / frequencyInHertz);
	long loopTime = (long)((timeInMilliseconds * 1000) / (delayAmount * 2));

	for (x = 0; x < loopTime; x++)
	{
		digitalWrite(BUZZER, HIGH);
		delayMicroseconds(delayAmount);
		digitalWrite(BUZZER, LOW);
		delayMicroseconds(delayAmount);
	}

	digitalWrite(Red_LED, LOW);
	delay(20);
}

// not used
void march()
{
#define c 261
#define d 294
#define e 329
#define f 349
#define g 391
#define gS 415
#define a 440
#define aS 455
#define b 466
#define cH 523
#define cSH 554
#define dH 587
#define dSH 622
#define eH 659
#define fH 698
#define fSH 740
#define gH 784
#define gSH 830
#define aH 880
	march_beep(a, 500);
	march_beep(a, 500);
	march_beep(a, 500);
	march_beep(f, 350);
	march_beep(cH, 150);
	march_beep(a, 500);
	march_beep(f, 350);
	march_beep(cH, 150);
	march_beep(a, 1000);
	march_beep(eH, 500);
	march_beep(eH, 500);
	march_beep(eH, 500);
	march_beep(fH, 350);
	march_beep(cH, 150);
	march_beep(gS, 500);
	march_beep(f, 350);
	march_beep(cH, 150);
	march_beep(a, 1000);
	march_beep(aH, 500);
	march_beep(a, 350);
	march_beep(a, 150);
	march_beep(aH, 500);
	march_beep(gSH, 250);
	march_beep(gH, 250);
	march_beep(fSH, 125);
	march_beep(fH, 125);
	march_beep(fSH, 250);
	delay(250);
	march_beep(aS, 250);
	march_beep(dSH, 500);
	march_beep(dH, 250);
	march_beep(cSH, 250);
	march_beep(cH, 125);
	march_beep(b, 125);
	march_beep(cH, 250);
	delay(250);
	march_beep(f, 125);
	march_beep(gS, 500);
	march_beep(f, 375);
	march_beep(a, 125);
	march_beep(cH, 500);
	march_beep(a, 375);
	march_beep(cH, 125);
	march_beep(eH, 1000);
	march_beep(aH, 500);
	march_beep(a, 350);
	march_beep(a, 150);
	march_beep(aH, 500);
	march_beep(gSH, 250);
	march_beep(gH, 250);
	march_beep(fSH, 125);
	march_beep(fH, 125);
	march_beep(fSH, 250);
	delay(250);
	march_beep(aS, 250);
	march_beep(dSH, 500);
	march_beep(dH, 250);
	march_beep(cSH, 250);
	march_beep(cH, 125);
	march_beep(b, 125);
	march_beep(cH, 250);
	delay(250);
	march_beep(f, 250);
	march_beep(gS, 500);
	march_beep(f, 375);
	march_beep(cH, 125);
	march_beep(a, 500);
	march_beep(f, 375);
	march_beep(c, 125);
	march_beep(a, 1000);
}

// not used
void Plane_Hit(void)
{
	digitalWrite(BUZZER, HIGH);
}

void red_led(bool status)
{
	if (status)
	{
		Red_LED_ON;
	}

	else
	{
		Red_LED_OFF;
	}
}

void blue_led(bool status)
{
	if (status)
	{
		Green_LED_ON;
	}

	else
	{
		Green_LED_OFF;
	}
}

unsigned char fifo_read()
{
	if (fifo_len > 0)
	{
		readpos++;
		readpos %= FIFO_SIZE;
		fifo_len--;
		return(Input_Fifo[readpos]);
	}

	else
	{
		return 0;
	}
}

void fifo_write(unsigned char data)
{
	if (fifo_len < FIFO_SIZE)
	{
		writepos++;
		writepos %= FIFO_SIZE;
		fifo_len++;
		Input_Fifo[writepos] = data;
	}

	else
	{
	}
}

int fifo_available()
{
	return fifo_len;
}

void fill_fifo()
{
	int av = Serial.available();

	while (av > 0)
	{
		int ch = Serial.read();

		if (ch >= 0)
		{
			fifo_write((unsigned char) ch);
		}

		else
		{
			break;
		}

		av--;
	}
}
