/**
 * @file learner_bot.c
 * @author <b>Group 20:</b>
 <ul>
	<li>Pradyumna Kumar
	<li>Jayanth Tadinada
	<li>Sharjeel Imam
 </ul>
 * @version 1.1
 * @section DESCRIPTION
 * The following is C code for the learner robot to recieve the encoded task, decode it and then perform the task. This code runs on Firebid V robot with Zigbee module for wireless communication.
 *
 * @section LICENSE

   Copyright (c) 2010, ERTS Lab IIT Bombay erts@cse.iitb.ac.in              
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.

   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

   * Source code can be used for academic purpose. 
	 For commercial use permission form the author needs to be taken.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. 

  Software released under Creative Commence cc by-nc-sa licence.
  For legal information refer to: 
  http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode

********************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/signal.h>
#include <util/delay.h>
#include "lcd.h"
#include "movement.h"

#define LEFT_MOTOR_SPEED 170		//!< Left motor speed
#define RIGHT_MOTOR_SPEED 170		//!< Right motor speed 

char data;							//!< Stores recieved charater
char task[40];						//!< The complete encoded task recieved
int size = 0;						//!< Counter to store characters into the task string

/**
	Function To Initialize USART0
 	desired baud rate: 9600
 	actual baud rate: 9600 (error 0.0%)
 	char size: 8 bit
	parity: Disabled
*/
void init_usart0()
{
	UCSR0B = 0x00;	//disable while setting baud rate
	UCSR0A = 0x00;
	UCSR0C = 0x06;
	UBRR0L = 0x47;	//set baud rate lo
	UBRR0H = 0x00;	//set baud rate hi
	UCSR0B = 0x98;
}

/**
	Right interrupt handler. Increase right shaft count
*/
ISR(INT5_vect)
{
	ShaftCountRight++;	//Increase shaft count after interrupt
}

/**
	Left interrupt handler. Increase left shaft count
*/
ISR(INT4_vect)
{
	ShaftCountLeft++;	//increase shaft count after interrupt
}

/**
	Signal to denote that data has been recieved by USART0 (Zigbee).
	Store the characters in the string till the termination character 'x' is recieved.
	Once 'x' is recieved, print the string on LCD and then call the function to perform the task
*/
SIGNAL(SIG_USART0_RECV)
{
	data = UDR0;		//recieved character
	if(data!=10)		//MATLAB sends character 10 after every transmission. So ignore this character
	{
		if(data=='x')	//termination character recieved
		{
			task[size] = '\0';
			lcd_string(task);	//display task on LCD
			perform_task(task);	//function to perform the task
			size = 0;			//set size to zero for next transmission
		}
		else
		{
			task[size] = data;	//store the character
			size++;
		}
	}
}

/**
	Main function.
	Initialize devices, ports, interrupts and set motor speed.
*/
int main()
{
	cli();					//disable global interrupts
	init_movement_ports();	//initialize movement ports
	init_timers();			//initialize timers
	init_devices();
	lcd_set_4bit();
	lcd_init();				//initialize lcd
	init_usart0();			//initialize USART0
	sei();					//enable global interrupts

	lcd_home();				//set lcd display to 1,1

	set_motor_speed(LEFT_MOTOR_SPEED,RIGHT_MOTOR_SPEED);	//set motor speed
	
	left_pos_interrupt_init();	//initialize left position interrupts
	right_pos_interrupt_init();	//initialize right position interrupts
	while (1);
}
