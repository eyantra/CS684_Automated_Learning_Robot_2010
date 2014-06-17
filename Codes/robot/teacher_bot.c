/**
 * @mainpage Automated Learning Robot
 * @author <b>Group 20:</b>
 <ul>
	<li>Pradyumna Kumar
	<li>Jayanth Tadinada
	<li>Sharjeel Imam
 </ul>
 * @version 1.1
 * @section DESCRIPTION
 * The teacher robot performs a task which is captured by an overhead camera. A computer encodes the task and transmits it to the learner robot which decodes the task and performs it. This code runs on Firebird V robot and requires Zigbee module for wireless communication.
 *
 ******************************************************************/
/**
 * @file teacher_bot.c
 * @author <b>Group 20:</b>
 <ul>
	<li>Pradyumna Kumar
	<li>Jayanth Tadinada
	<li>Sharjeel Imam
 </ul>
 * @version 1.1
 * @section DESCRIPTION
 * The following is C code for the teacher robot to perform the task. There are some pre-programmed tasks which can be selected by transferring 1, 2 or 3. The robot can also be controlled manually using the keys A, W, S, D, X. This code runs on Firebid V robot with Zigbee module for wireless communication.
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
#include "movement.h"

#define LEFT_MOTOR_SPEED 170				//!< Left motor speed
#define RIGHT_MOTOR_SPEED 170				//!< Right motor speed 

char data;									//!< Stores recieved charater
char task1[30] = "40f90r40f90r40f90r40f";	//!< Task1: Square with right turns
char task2[30] = "35f120r40f120l35f";		//!< Task2: 'Z' motion starting from top left
char task3[30] = "35f120r35f60l35b";		//!< Task3: Equilateral triangle starting from bottom left

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
	'1'. Perform task 1.
	'2'. Perform task 2.
	'3'. Perform task 3.
	'w'. Move forward.
	's'. Move backward.
	'a'. Left turn.
	'd'. Right turn.
	'x'. Stop.
	Note: The letters are in lower case
*/
SIGNAL(SIG_USART0_RECV)
{
	data = UDR0;	//recieved character
	switch (data)
	{
		case 'w': FORWARD(); break;
		case 's': REVERSE(); break; 
		case 'a': LEFT_TURN(); break;
		case 'd': RIGHT_TURN(); break;
		case 'x': STOP(); break;
		case '1': perform_task(task1); break;
		case '2': perform_task(task2); break;
		case '3': perform_task(task3); break;
		default: STOP(); break;
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
	init_usart0();			//initialize USART0
	sei();					//enable global interrupts
	
	set_motor_speed(LEFT_MOTOR_SPEED,RIGHT_MOTOR_SPEED);	//set motor speed
	
	left_pos_interrupt_init();	//initialize left position interrupts
	right_pos_interrupt_init();	//initialize right position interrupts
	while (1);
}
