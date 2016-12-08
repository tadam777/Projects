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

ISR(TIMER1_OVF_vect)
{
	unsigned int us;

	if (current == 1)
	{
		Servo_Ports_LOW;
		InterruptInMicros(low_pulse_width);
		current = 0;
	}

	else
	{
		Servo_Number++;

		if (Servo_Number > 8)
		{
			total_ppm_time = 0;
			Servo_Number = 0;
		}

		if (Servo_Number == 8)
		{
			us = 40000 - total_ppm_time;
		}

		else
		{
			us = Servo_Position[Servo_Number];
		}

		total_ppm_time += us;

		if (receiver_mode == 0)
		{
			if (rx_board != TX_BOARD)
			{
			}
		}
		else
		{
			Serial_PPM_OUT_HIGH;
			TCNT1 = 40000 - (us - 2 * low_pulse_width - 2 * (10 + 34));
			current = 1;
		}
	}
}