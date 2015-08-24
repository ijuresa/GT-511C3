/*
 * project_2_3.c
 *
 * Created: 28/05/2015 18:49:02
 *  Author: Ivan
 *
 *Atmega32 with touch sensor GT-511C3
 *****************************************************
 *Fuse bits set:
 *				High : 0xC9
 *				Low  : 0xFF
 *
 *CKSEL3,2,1,0	= 1
 *SUT2,1		= 1
 *CKOPT			= 0
 *****************************************************
*/

/* Clock */
#ifndef F_CPU
#define F_CPU 8000000UL							
#endif


/* Includes */
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "lcd.h"
#include "uart.h"
#include "GT_511C3_lib.h"


/* Operating BAUDRATE */
#define baudRate 9600

/* Calculation used to calculate UBBR */
#define UBRR F_CPU/16/baudRate-1


/*  Variables  */
uint16_t checksum;						//Variable for checksum
uint8_t outgoing_packet[12];			//Sending data
uint8_t incoming_buffer[12];			//Response packet
uint8_t parameter[4], command[2];
uint8_t ON_OFF_BACKLIGHT = 0;			//Always off on begining
static volatile uint8_t key = 0;		//Used for keys
uint8_t status = 0;						// Used for menu status 



//------------------------------------------------------------------------------
//    Name:        UART_send_packet
//    Description: Sending packet via UART to sensor.
//    Input:       outgoing_packet--> array which is send via uart
//    Output:      -
//    Misc:		   -
//------------------------------------------------------------------------------
void UART_send_packet(uint8_t outgoing_packet[])
{
	uint8_t i;

	/* Actually sending array via UART to sensor */
	for(i = 0; i < 12; i++)
	{
		uart0_putc(outgoing_packet[i]);
	}
 	_delay_ms(500);
}

//------------------------------------------------------------------------------
//    Name:        UART_response_packet
//    Description: Receive UART packet from GT-511C3 sensor.
//    Input:       incoming_buffer--> array for incoming data
//    Output:      -
//    Misc:		   -
//------------------------------------------------------------------------------
void UART_response_packet(uint8_t incoming_buffer[])
{
	/* 16 bit data array for data + error response */
	uint16_t n_data_plus_error[12]; 
	uint8_t i, n_data[12], n_error_code;
	
	do 
	{
		/* Input Array | dividing low and high bits and putting data into input array */
		for( i=0; i<12; i++ )   
		{
			n_data_plus_error[i] = uart0_getc();
		
			/* extract/cast "data" from error+data array -> Lower 8 bits */
			n_data[i] = (uint8_t)(n_data_plus_error[i] & 0x00FF);
		
			/* extract/cast "error" from error+data array -> higher 8 bits */
			n_error_code = (uint8_t)((n_data_plus_error[i] & 0xFF00) >> 8);
		
			/* Putting response data into our array */
			incoming_buffer[i] = n_data[i];	
		}
		
	} while ( uart0_available() != 0);
	_delay_ms(500);
}

//------------------------------------------------------------------------------
//    Name:        parameter_from_int
//    Description: Integer to hex for sending packages.
//    Input:       -
//    Output:      -
//    Misc:		   -
//------------------------------------------------------------------------------
void parameter_from_int(uint8_t i)
{
	parameter[0] = ( i & 0x000000FF );
	parameter[1] = ( i & 0x0000FF00 ) >> 8;
	parameter[2] = ( i & 0x00FF0000 ) >> 16;
	parameter[3] = ( i & 0xFF000000 ) >> 24;
}

//------------------------------------------------------------------------------
//    Name:        int_from_parameter
//    Description: Hex to integer for receive packages.
//    Input:       -
//    Output:      Returns parameter
//    Misc:		   -
//------------------------------------------------------------------------------
int int_from_parameter()
{
	uint8_t return_parameter = 0;
	
	return_parameter = ( return_parameter << 8) + parameter[3];
	return_parameter = ( return_parameter << 8) + parameter[2];
	return_parameter = ( return_parameter << 8) + parameter[1];
	return_parameter = ( return_parameter << 8) + parameter[0];
	
	return return_parameter;
	
}

//------------------------------------------------------------------------------
//    Name:        calculate_checksum
//    Description: Summing first 10 bytes of package which is to be send via UART to sensor as lower and higher checksum
//    Input:       Parameter of given command and command
//    Output:      Returns checksum
//    Misc:		   -
//------------------------------------------------------------------------------
int calculate_checksum(uint8_t parameter[], uint8_t command[])
{
	uint16_t checksum = 0;
	checksum += COMMAND_START_CODE_1;
	checksum += COMMAND_START_CODE_2;
	checksum += COMMAND_DEVICE_ID_1;
	checksum += COMMAND_DEVICE_ID_2;
	checksum += parameter[0];
	checksum += parameter[1];
	checksum += parameter[2];
	checksum += parameter[3];
	checksum += command[0];
	checksum += command[1];

	return checksum;
}

//------------------------------------------------------------------------------
//    Name:        assemble_data_packet
//    Description: Assembling data packet to be sent.
//    Input:       -
//    Output:      -
//    Misc:		   -
//------------------------------------------------------------------------------
void assemble_data_packet()
{
	
	/* Checksum calculation -> outgoing_packet[0] + .... + outgoing_packet[9] = Checksum */
	checksum = calculate_checksum(parameter, command);
	
	/* Data packet which is to be sent by UART to sensor */
	outgoing_packet[0] = COMMAND_START_CODE_1;
	outgoing_packet[1] = COMMAND_START_CODE_2;
	outgoing_packet[2] = COMMAND_DEVICE_ID_1;
	outgoing_packet[3] = COMMAND_DEVICE_ID_2;
	outgoing_packet[4] = parameter[0];
	outgoing_packet[5] = parameter[1];
	outgoing_packet[6] = parameter[2];
	outgoing_packet[7] = parameter[3];
	outgoing_packet[8] = command[0];
	outgoing_packet[9] = command[1];
	outgoing_packet[10] = get_low_byte(checksum);
	outgoing_packet[11] = get_hight_byte(checksum);	
}

//------------------------------------------------------------------------------
//    Name:        delete_one_ID
//    Description: Deletes only 1 ID which is send by parameter
//    Input:       -
//    Output:      NACK_INVALID_POS
//    Misc:		   -
//------------------------------------------------------------------------------
// void delete_one_ID(uint8_t ID)
// {
// 	
// }



//------------------------------------------------------------------------------
//    Name:        check_enrolled
//    Description: Check whether the specified ID is already enrolled if it is then we search for empty one.
//    Input:       ID--> of user to be enrolled
//    Output:      ACK( Already used ) or NACK( Not used or ID is not between 0-199 )
//    Misc:		   -
//------------------------------------------------------------------------------
int check_enrolled(uint8_t ID)
{
	uint8_t buffer1[20];
	
	if( ON_OFF_BACKLIGHT == 0 )
	{
		lcd_clrscr();
		lcd_puts("Turn Backlight \nON with key 3!");
		return -1;
	}
	else
	{
		/* Dividing lower byte and higher byte from 16bit command to fit into 1 UART message */
		command[0] = get_low_byte(CHECK_ENROLLED);
		command[1] = get_hight_byte(CHECK_ENROLLED);
	
		/* Sending current ID to function which transforms integer to hex and puts it into parameter[3,2,1,0] package */
		parameter_from_int(ID);

		assemble_data_packet();
	
		/* UART sending */
		UART_send_packet(outgoing_packet);
		_delay_ms(500);
	
		/* UART receiving */
		UART_response_packet(incoming_buffer);
		
		/* Checking if my incoming buffer is 0x30 which is ACK_low defined. Means that ID is already used */ 
		if( incoming_buffer[8] == ACK_low)
		{
			/* Error--> Message output */
			case_error(incoming_buffer);
			return 0;
		}
		/* Else we are returning on which ID we are enrolling */
		else 
		{	
			lcd_clrscr();
			lcd_puts("Enrolling on ID =");
			lcd_gotoxy(0, 1);
			lcd_puts( itoa( outgoing_packet[4], buffer1, 10 ));
				
			return 1;
		}
	}
}

//------------------------------------------------------------------------------
//    Name:        enroll_start
//    Description: Start an enrollment
//    Input:       ID--> number of next enrolled user
//    Output:      NACK_DB_IS_FULL, NACK_INVALID_POS, NACK_ALREADY_USED
//    Misc:		   -
//------------------------------------------------------------------------------
void enroll_start(uint8_t ID)
{
	/* Dividing lower byte and higher byte from 16bit command to fit into 1 UART message */
	command[0] = get_low_byte(ENROLL_START);
	command[1] = get_hight_byte(ENROLL_START);
	
	/* Sending current ID to function which transforms integer to hex and puts it into parameter[3,2,1,0] package */
	parameter_from_int(ID);
	
	assemble_data_packet();
	
	/* UART sending */
	UART_send_packet(outgoing_packet);
	_delay_ms(500);
	
	/* UART receiving */
	UART_response_packet(incoming_buffer);
	
	/* Error--> Message output */
	case_error(incoming_buffer);
}

//------------------------------------------------------------------------------
//    Name:        enroll_1
//    Description: Make 1st template for an enrollment
//    Input:       -
//    Output:      NACK_ENROLL_FAILED, NACK_BAD_FINGERPRINT
//    Misc:		   -
//------------------------------------------------------------------------------
void enroll_1()
{
	/* Dividing lower byte and higher byte from 16bit command to fit into 1 UART message */
	command[0] = get_low_byte(ENROLL_1);
	command[1] = get_hight_byte(ENROLL_1);
	
	parameter[0] = 0x00;
	parameter[1] = 0x00;
	parameter[2] = 0x00;
	parameter[3] = 0x00;
	
	assemble_data_packet();
	
	/* UART sending */
	UART_send_packet(outgoing_packet);
	_delay_ms(500);
	
	/* UART receiving */
	UART_response_packet(incoming_buffer);
	
	/* Error--> Message output */
	case_error(incoming_buffer);
}

//------------------------------------------------------------------------------
//    Name:        enroll_2
//    Description: Make 2nd template for an enrollment
//    Input:       -
//    Output:      NACK_ENROLL_FAILED, NACK_BAD_FINGERPRINT
//    Misc:		   -
//------------------------------------------------------------------------------
void enroll_2()
{
	/* Dividing lower byte and higher byte from 16bit command to fit into 1 UART message */
	command[0] = get_low_byte(ENROLL_2);
	command[1] = get_hight_byte(ENROLL_2);
	
	parameter[0] = 0x00;
	parameter[1] = 0x00;
	parameter[2] = 0x00;
	parameter[3] = 0x00;
	
	assemble_data_packet();
	
	/* UART sending */
	UART_send_packet(outgoing_packet);
	_delay_ms(500);
	
	/* UART receiving */
	UART_response_packet(incoming_buffer);
	
	/* Error--> Message output */
	case_error(incoming_buffer);
}

//------------------------------------------------------------------------------
//    Name:        enroll_3
//    Description: Make 3rd template for an enrollment,
//				   Merge them into one template and save to the database.
//    Input:       -
//    Output:      NACK_ENROLL_FAILED, NACK_BAD_FINGERPRINT
//    Misc:		   -
//------------------------------------------------------------------------------
void enroll_3()
{
	/* Dividing lower byte and higher byte from 16bit command to fit into 1 UART message */
	command[0] = get_low_byte(ENROLL_3);
	command[1] = get_hight_byte(ENROLL_3);
	
	parameter[0] = 0x00;
	parameter[1] = 0x00;
	parameter[2] = 0x00;
	parameter[3] = 0x00;
	
	assemble_data_packet();
	
	/* UART sending */
	UART_send_packet(outgoing_packet);
	_delay_ms(500);
	
	/* UART receiving */
	UART_response_packet(incoming_buffer);
	
	/* When enrollment is successful response packet is ACK (0x0030) */
	if( incoming_buffer[8] == ACK_low )
	{
		lcd_clrscr();
		lcd_puts("Enrollment was \nSuccessful");
	}
	
	/* Error--> Message output */
	else case_error(incoming_buffer);
}

//------------------------------------------------------------------------------
//    Name:			is_press_finger
//    Description:	Check is finger is placed on the sensor
//    Input:       -
//    Output:      -1-->If backlight is OFF
//					0-->If finger is not yet placed
//					1-->If finger is placed/read and needs to be removed
//    Misc:		   -
//------------------------------------------------------------------------------
int is_press_finger()
{
	if( ON_OFF_BACKLIGHT == 0 )
	{
		lcd_clrscr();
		lcd_puts("Turn Backlight \nON with key 3!");
		return -1;
	}
	else
	{
		/* Dividing lower byte and higher byte from 16bit command to fit into 1 UART message */
		command[0] = get_low_byte(IS_PRESS_FINGER);
		command[1] = get_hight_byte(IS_PRESS_FINGER);
	
		parameter[0] = 0x00;
		parameter[1] = 0x00;
		parameter[2] = 0x00;
		parameter[3] = 0x00;
	
		assemble_data_packet();
	
		/* UART sending */
		UART_send_packet(outgoing_packet);
		_delay_ms(500);
	
		/* UART receiving */
		UART_response_packet(incoming_buffer);
		
		/* Error--> Message output */
		case_error(incoming_buffer);
	
		/* If 4th (parameter[0]) place in incoming package is 0x00 then finger is pressed */ 
		if(incoming_buffer[4] == 0x00)
		{
			lcd_clrscr();
			lcd_puts("Remove finger");
			return 1;
		}
		 
		else return 0;
	}
}

//------------------------------------------------------------------------------
//    Name:        capture_finger
//    Description: Captures finger which is pressed onto sensor ( ATM set to capture best quality image )		
//    Input:       -
//    Output:      Finger is not pressed ( Press finger )
//    Misc:		   -
//------------------------------------------------------------------------------
void capture_finger()
{
	/* Dividing lower byte and higher byte from 16bit command to fit into 1 UART message */
	command[0] = get_low_byte(CAPTURE_FINGER);
	command[1] = get_hight_byte(CAPTURE_FINGER);
	
	parameter_from_int(1); //For best image, slow
	
	assemble_data_packet();

	/* UART sending */
	UART_send_packet(outgoing_packet);
	_delay_ms(500);
			
	/* UART receiving */
	UART_response_packet(incoming_buffer);	
	
	/* Error--> Message output */
	case_error(incoming_buffer);
}

//------------------------------------------------------------------------------
//    Name:        ID_identify
//    Description: Checks if fingerprint is already in database
//    Input:       ID
//    Output:      Verified ID ( if fingerprint is verified ) and "Finger not found"
//    Misc:		   -
//------------------------------------------------------------------------------
void ID_identify(uint8_t ID)
{
	uint8_t i;
	/* Dividing lower byte and higher byte from 16bit command to fit into 1 UART message */
	command[0] = get_low_byte(ID_IDENTIFY);
	command[1] = get_hight_byte(ID_IDENTIFY);
	
	parameter[0] = 0x00;
	parameter[1] = 0x00;
	parameter[2] = 0x00;
	parameter[3] = 0x00;
	
	assemble_data_packet();

	/* UART sending */
	UART_send_packet(outgoing_packet);
	_delay_ms(500);
	
	/* UART receiving */
	UART_response_packet(incoming_buffer);
	
	/* Error--> Message output */
	case_error(incoming_buffer);
	
	lcd_clrscr();
	
	/* If parameter is 0 then fingerprint is found in database*/ 
	if(incoming_buffer[5] == 0x00)
	{
		lcd_puts("Welcome!!");
		/* Servo motor LEFT */
		//DDRD = (1 << PD5);
		//OCR1A = ICR1 - 53;
		//_delay_ms(100);
		//DDRD |= (0 << PD5);	
	}
	else
	{
		lcd_puts("Access Denied!!");
		for(i = 0; i < 200; i++)
		{
			/* Speaker , will be used when fingerprint is not found in database.*/
 			PORTA = 0 << PA5;
 			_delay_ms(1);
 			PORTA = 1 << PA5;
 			_delay_ms(1);
		}

		 /* Servo motor RIGHT */
// 		 DDRD = (1 << PD5);
// 		 OCR1A = ICR1 - 25;
// 		 _delay_ms(100);
// 		 DDRD |= (0 << PD5);	
	}
}

//------------------------------------------------------------------------------
//    Name:        delete_all
//    Description: Deletes all fingerprints from database
//    Input:       -
//    Output:      ACK =  OK, NACK = NACK_DB_IS_EMPTY
//    Misc:		   -
//------------------------------------------------------------------------------
void delete_all()
{
	/* Dividing lower byte and higher byte from 16bit command to fit into 1 UART message */
	command[0] = get_low_byte(DELETE_ALL);
	command[1] = get_hight_byte(DELETE_ALL);
	
	parameter[0] = 0x00;
	parameter[1] = 0x00;
	parameter[2] = 0x00;
	parameter[3] = 0x00;
	
	assemble_data_packet();
	
	/* UART sending */
	UART_send_packet(outgoing_packet);
	_delay_ms(500);
	
	/* UART receiving */
	UART_response_packet(incoming_buffer);
	
	/* Error--> Message output */
	case_error(incoming_buffer);	
}

//------------------------------------------------------------------------------
//    Name:        get_enroll_count
//    Description: Returns number of fingerprints in database ( still need to finish )
//    Input:       ID
//    Output:      Number of enrolled fingerprints or "The database is empty"
//    Misc:		   -
//------------------------------------------------------------------------------
void get_enroll_count(uint8_t ID)
{
	/* Number of enrolled IDs */
	uint8_t buffer11[20];
	
	/* Dividing lower byte and higher byte from 16bit command to fit into 1 UART message */
	command[0] = get_low_byte(GET_ENROLL_COUNT);
	command[1] = get_hight_byte(GET_ENROLL_COUNT);
	
	parameter[0] = 0x00;
	parameter[1] = 0x00;
	parameter[2] = 0x00;
	parameter[3] = 0x00;
	
	assemble_data_packet();

	/* UART sending */
	UART_send_packet(outgoing_packet);
	_delay_ms(500);
	
	/* UART receiving */
	UART_response_packet(incoming_buffer);
		
	/* Error--> Message output */
	case_error(incoming_buffer);
	
	/* Number of enrolled IDs - output */
	lcd_clrscr();
	lcd_puts("Enrolled IDs = \n");
	lcd_puts(itoa(incoming_buffer[4], buffer11, 10));
	
// 	if(new_ID == 0) { ID = 1; return ID;}
// 	else ID = new_ID + 1; return ID;
	 	
}

//------------------------------------------------------------------------------
//    Name:        open
//    Description: Used to open connection with GT-511C3
//    Input:       -
//    Output:      
//    Misc:		   It's used only once. In main program.
//------------------------------------------------------------------------------
void open()
{
	/* Dividing lower byte and higher byte from 16bit command to fit into 1 UART message */
	command[0] = get_low_byte(OPEN);
	command[1] = get_hight_byte(OPEN);
	
	parameter[0] = 0x00;
	parameter[1] = 0x00;
	parameter[2] = 0x00;
	parameter[3] = 0x00;
	
	assemble_data_packet();

	/* UART sending */
	UART_send_packet(outgoing_packet);
	_delay_ms(500);
			
	/* UART receiving */
	UART_response_packet(incoming_buffer);	
}

//------------------------------------------------------------------------------
//    Name:			cmosled       
//    Description:	Turns ON or OFF backlight. Initially it's always OFF
//    Input:       -
//    Output:       Backlight is ON/OFF!
//    Misc:		   -
//------------------------------------------------------------------------------
void cmosled()
{
	/* Dividing lower byte and higher byte from 16bit command to fit into 1 UART message */
	command[0] = get_low_byte(CMOSLED);
	command[1] = get_hight_byte(CMOSLED);
	
	/* If LED is OFF then we turn it ON */ 
	if(ON_OFF_BACKLIGHT == 0)
	{
		parameter[0] = 0x01;
		ON_OFF_BACKLIGHT = 1;	
		lcd_clrscr();
		lcd_puts("Backlight is ON!");	
	}
	/* IF LED is ON we turn it OFF */
	else
	{
		parameter[0] = 0x00;
		ON_OFF_BACKLIGHT = 0;
		lcd_clrscr();
		lcd_puts("Backlight is OFF!");		
	}
	/* Rest of parameters are same for ON and OFF state */
	parameter[1] = 0x00;
	parameter[2] = 0x00;	
	parameter[3] = 0x00;	
	
	/* Assembling data packet */
	assemble_data_packet();
	
	/* UART sending */
	 UART_send_packet(outgoing_packet);
	_delay_ms(500);
	 
	/* UART receiving */
	 UART_response_packet(incoming_buffer);
} 

//------------------------------------------------------------------------------
//    Name:        hex_polje_sum
//    Description: Assembling array of data to be send  
//    Input:       Key input ( ATM 6 keys ), ID
//    Output:      -
//    Misc:		   -
//------------------------------------------------------------------------------
void hex_polje_sum(uint8_t outgoing_packet[], uint8_t ID)
{
	
	/* Depends which button we press */
	switch (key)
	{		
		/* Enrolling new fingerprint into database */
		case 2:
			key = 0;
			/* Counting number of fingerprints in database */
			//get_enroll_count(ID);
			
			/* Check if LED is OFF */
			if( check_enrolled(ID) == -1) break;
			/* Check for an ID slot that is unused until u find one:	*/											
			while( check_enrolled(ID) == 0 )ID ++; _delay_ms(100);
			//Start and enrollment with that ID							
			enroll_start(ID);
			/* Wait while finger is pressed */																
 			while( is_press_finger() == 0 ) _delay_ms(100);	
			/* Capture fingerprint in high quality */ 								
			capture_finger();
			
			/* 1st Enrollment */																
			enroll_1();	
			/* Wait while finger is removed */																	
			while( is_press_finger() == 1 ) lcd_puts("Remove finger!"); _delay_ms(100);
			/* Wait while finger is pressed again for 2nd enrollment */	
			while( is_press_finger() == 0 ) _delay_ms(100);	
			/* Capture fingerprint in high quality */								
			capture_finger();	
			
			/* 2nd Enrollment */															
			enroll_2();
			/* Wait while finger is removed */																		
			while( is_press_finger() == 1 ) lcd_puts("Remove finger!"); _delay_ms(100);
			/* Wait while finger is pressed again for 3nd enrollment */	
			while( is_press_finger() == 0 ) _delay_ms(100);	
			/* Capture fingerprint in high quality */								
			capture_finger();
			
			/* 3rd Enrollment */																
			enroll_3();									
			break;
	
		case 3:
			key = 0;
			/* Turns backlight ON or OFF */
			cmosled();		
			break;	
		
		case 4:
			key = 0;
			/* Returns number of enrolled fingerprints */
			get_enroll_count(ID);
			break;
			
		/* When key 4 is pressed identification of fingerprint will begin */	
		case 5:
			key = 0;
			/* Check if LED is OFF */
			if( is_press_finger() == -1) break;
			/* Checking if finger is pressed */
			while( is_press_finger() == 0) _delay_ms(100);
			/* Capturing fingerprint so we can match it with fingerprints in database */
			capture_finger();
			/* Verification of our fingerprint, returns 1 or 0. */
			ID_identify(ID);
			break;
			
		case 6:
			
			key = 0;
			/* Testing */
			if(status == 6)
			{
				lcd_puts("Uso sam");
				break;
			}
			
			else 
			{
				/* Deletes all records from database */
				delete_all();
				break;
			}
	}
}

int main(void)
{
	/* Used for buttons */
	/* Clear the bit---> 1 u 0, without touching other bits */
	DDRB &= ~(1 << PB0 | 1 << PB1 | 1 << PB2 | 1 <<PB3 | 1 << PB4); 
	/* Set the bit---> 0 u 1, without touching other bits */
	PORTB |= (1<< PB0 | 1 << PB1 | 1 << PB2 | 1 <<PB3 | 1 << PB4);
	
	/* Key 6 */
	DDRC &= ~(1 << PC0);
	PORTC |= (1 << PC0);
	
	/* LED for testing purpose and speaker */
	DDRA |= (1 << PA4 | 1 << PA5);
	/* Servo is initially OFF */
	DDRD |= (0 << PD5);	

	/* Inverted Mode */ 
	TCCR1A |= 1<<WGM11 | 1<<COM1A1 | 1<<COM1A0;
	/* Fast PWM mode 14, Prescaler = 256 */ 
	TCCR1B |= 1<<WGM13 | 1<<WGM12 | 1<<CS12;
	
	/* Top of the counter count */
	ICR1 = 625;
	OCR1A = ICR1 - 65;

	/* UART initialization */ 
	uart0_init(UBRR); 
	
	/* Getting LCD ready */
	lcd_init(LCD_DISP_ON); 
	lcd_clrscr();
	lcd_puts("Sensor is ready!");

	/* ID number for fingerprint count */
	uint8_t ID = 0;
	/* Used for menu status */
	uint8_t status = 0;						

	/* Enabling global interrupts */
    sei();
	
	/* On startup command OPEN is executed once */
	open(); 
	
	while(1)
    {
		
		if(bit_is_clear(PINC, PC0))
		{
			key = 1;
			/* Used for 1st key (Menu) */
			status++;
			status = status%7;		
		}
		
		if(bit_is_clear(PINB, PB0))
		{
			key = 2;	
		}
		
		if(bit_is_clear(PINB, PB1))
		{
			key = 3;		
		}
		
		if(bit_is_clear(PINB, PB2))
		{
			key = 4;		
		}
		
		if(bit_is_clear(PINB, PB3))
		{
			key = 5;
		}
		
		if(bit_is_clear(PINB, PB4))
		{
			key = 6;
		}
		
		/* Using key 1 for switching */
        if(key == 1)
		{
			if (status == 1)
			{
				lcd_clrscr();
				lcd_puts("Welcome to Menu \nfor GT-511C3");
				_delay_ms(250);
			}
			
			else if(status == 2)
			{
				lcd_clrscr();
				lcd_puts("Press key 2 for \nEnrollment");
				_delay_ms(250);	
			}
			
			else if(status == 3)
			{
				lcd_clrscr();
				lcd_puts("Press key 3 for \nBacklight ON/OFF");
				_delay_ms(250);
			}
			
			else if(status == 4)
			{
				lcd_clrscr();
				lcd_puts("Press key 4 for \nEnroll Count");
				_delay_ms(250);
			}
			
			else if(status == 5)
			{
				lcd_clrscr();
				lcd_puts("Press key 5 for \nIdentification");
				_delay_ms(250);
			}
			
			else if(status == 6)
			{
				lcd_clrscr();
				lcd_puts("Press key 6 for \nDeleting Database");
				_delay_ms(250);
			}
		}
		 
		if(key == 2)
		{
			lcd_clrscr();
			lcd_puts("Enrolling in \nProcess");
			hex_polje_sum(outgoing_packet, ID);
		}
		 
		if(key == 3)
		{
			lcd_clrscr();
			hex_polje_sum(outgoing_packet, ID);	
		}
		
		if(key == 4)
		{
			lcd_clrscr();
			lcd_puts("Enroll count");
			hex_polje_sum(outgoing_packet, ID);
		}
		
		if(key == 5)
		{
			lcd_clrscr();
			lcd_puts("Identification \nIn Process");
			hex_polje_sum(outgoing_packet, ID);
		}
		/* Used for testing purpose */
		if((key == 6) && (status != 6))
		{
			lcd_clrscr();
			lcd_puts("Deleting \nDatabase");
			hex_polje_sum(outgoing_packet, ID);
		}
		/* Used for testing purpose */
		if((status == 6) && (key == 6))
		{
			lcd_clrscr();
			hex_polje_sum(outgoing_packet, ID);
		}
	}
}