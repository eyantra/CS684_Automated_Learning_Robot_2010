/**
 * @file movement.h
 * @author <b>Group 20:</b>
 <ul>
	<li>Pradyumna Kumar
	<li>Jayanth Tadinada
	<li>Sharjeel Imam
 </ul>
 * @section DESCRIPTION
 This file defines various functions for Firebird V movement, interrupts, motor speed and pin configurations
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

unsigned long int ShaftCountRight;		//!< Right shaft count
unsigned long int ShaftCountLeft;		//!< Left shaft count

/**
	Initialize pins for Motors and PWM
*/
void init_movement_ports()
{
	DDRA = 0x0F;
	PORTA = 0x00;	//stop condition initially
	DDRL = 0x18;	//Setting PL3 and PL4 pins as output for PWM generation
	PORTL = 0x18;	//PL3 and PL4 pins are for velocity control using PWM
	DDRE = 0x00;
	PORTE = 0xCF;
}

/**
	Initialize pins for timer
*/
void init_timers()
{
	TCCR5B = 0x00;
	TCCR5A = 0xA9;
	TCCR5B = 0x0B;
}

/**
	Initialize left position interrupt
*/
void left_pos_interrupt_init()
{
	cli();  				//clear global interrupt
	EICRB = EICRB | 0x02;  	//INT4 set to falling edge 
	EIMSK = EIMSK | 0x10;	//enable interrupt INT4 for left position encoder
	sei();					//enable global interrupt

}

/**
	Initialize right position interrupt
*/
void right_pos_interrupt_init()
{
	cli();					//clear global inetrrupt
	EICRB = EICRB | 0x08;	//INT5 set to falling edge
	EIMSK = EIMSK | 0x20;	//enable interrupt INT5 for right position encoder
	sei();					//enable global interrupt

}

/**
	Assign speed to left and right motors
	@param left_motor Speed of left motor
	@param right_motor Speed of right motor
*/
void set_motor_speed(unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = left_motor;
	OCR5BL = right_motor;
}

/**
	Hard right turn
*/
void RIGHT_TURN()
{
	PORTA = 0x0A; //hard right
}

/**
	Hard left turn
*/
void LEFT_TURN()
{
	PORTA = 0x05; //hard left
}

/**
	Move forward
*/
void FORWARD()
{
	PORTA = 0x06; //forward
}

/**
	Move backward
*/
void REVERSE()
{
	PORTA = 0x09; //reverse
}

/**
	Stop movement
*/
void STOP()
{
	PORTA = 0x00; //stop
}

/**
	Function to move the robot in a straight line for a given distance
	@param dist Distance to move the robot (in millimeters)
	@param dir The direction to move the robot. 0 means forward and 1 means backward
*/
void linear_distance_mm(unsigned int dist, short dir)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = dist/5.338;	//division by resolution to get shaft count 
	ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
	
	ShaftCountRight = 0;	//initialize shaft count to zero
	
	if (dir == 0)
		FORWARD();	//move forward
	else
		REVERSE();	//move reverse

	while (1)
	{
		if (ShaftCountRight > ReqdShaftCountInt)
			break;	//target distance moved
	}
	STOP();		//stop
	
	_delay_ms(500);
}

/**
	Function to turn the robot for a specified angle and direction
	@param angle Angle to turn the robot (in degrees)
	@param dir The turn direction. 0 means right turn and 1 means left turn
*/
void rotate_angle_deg(unsigned int angle, short dir)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = (float)angle/4.090;	//division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int)ReqdShaftCount;
	
	ShaftCountRight = 0;	//initialize left shaft count to zero
	ShaftCountLeft = 0;		//initialize right shaft count to zero

	if (dir == 0)
		RIGHT_TURN();	//right turn
	else 
		LEFT_TURN();	//left turn

	while (1)
	{
		if ((ShaftCountRight >= ReqdShaftCountInt) || (ShaftCountLeft >= ReqdShaftCountInt))
			break;	//right or left turn completed
	}

	STOP();		//stop
	_delay_ms(500);
}

/**
	Function to decode the task and perform it
	@param task The task string which is to be decoded and performed
*/
void perform_task(char* task)
{	
	left_pos_interrupt_init();	//initialize left position interrupts
	right_pos_interrupt_init();	//initialize right position interrupts
	
	int i=0,j=0; char temp[4];
	
	while (task[i] != '\0')		//not end of string
	{
		if (!((task[i]<=90 && task[i]>=65) || (task[i]<=122 && task[i] >=97)))	//numbers
		{
			temp[j++] = task[i++];
			continue;
		}
		else
		{
			temp[j++] = '\0';
			int k = atoi(temp);		//convert string to integer
			i++; j=0; 
			switch(task[i-1])
			{
				case 'f': linear_distance_mm(k*10,0); STOP(); break;	//forward
				case 'b': linear_distance_mm(k*10,1); STOP(); break;	//reverse
				case 'r': rotate_angle_deg(k,0); STOP(); break;			//right turn
				case 'l': rotate_angle_deg(k,1); STOP(); break;			//left turn
				default: STOP(); break;									//stop
			}
		}
	}
	STOP();
}