/*
 * project_2_3.c
 *
 * Created: 28/05/2015 18:49:02
 *  Author: Ivan
 *
 *FUSE bits set: High : 0xC9
 *				 Low  : 0xFF
 *
 *CKSEL3,2,1,0	= 1
 *SUT2,1		= 1
 *CKOPT			= 0
 *
 *
 *
 *
 *To DO: Return UART send and receive to main file. Divide function for sensor to lib.h and lib.c
 *		 Implement Delete 1 ID from database and make menu with LCD. (Probably will be 6 keys)	
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
uint16_t checksum;					//Variable for checksum
uint8_t outgoing_packet[12];		//Sending data
uint8_t incoming_buffer[12];		//Response packet
uint8_t parameter[4], command[2];
uint8_t ON_OFF_BACKLIGHT = 0;		//Always off on begining
static volatile uint8_t stanje = 0;	//Used for keys


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
//    Output:      -
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
//					Need to implement receive calculate_checksum!!!!
//    Input:       -
//    Output:      -
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
	
	//Checksum calculation -> outgoing_packet[0] + .... + outgoing_packet[9] = Checksum
	checksum = calculate_checksum(parameter, command);
	
	//Data packet which is to be sent by UART to sensor
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
//    Name:        check_enrolled
//    Description: Check whether the specified ID is already enrolled if it is then we search for empty one.
//    Input:       -
//    Output:      ACK( Already used ) or NACK( Not used or ID is not between 0-199 )
//    Misc:		   -
//------------------------------------------------------------------------------
int check_enrolled(uint8_t ID)
{
	if( ON_OFF_BACKLIGHT == 0 )
	{
		lcd_clrscr();
		lcd_puts("Press key 3\nFor Backlight");
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
		_delay_ms(1000);
	
		/* UART receiving */
		UART_response_packet(incoming_buffer);
	
		/* Checking if my incoming buffer if 0x30 which is ACK_low defined. Need to finish this but it WORKS!*/ 
		if( incoming_buffer[8] == ACK_low)
		{
			/* Error--> Message output */
			case_error(incoming_buffer);
			return 0;
		}
		/* Else we are returning which error is received */
		else 
		{
			/* Error--> Message output */
			case_error(incoming_buffer);
			return 1;
		}
	}
}

//------------------------------------------------------------------------------
//    Name:        enroll_start
//    Description: Start an enrollment
//    Input:       -
//    Output:      Error message ( Still need to check which ones )
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
	_delay_ms(1000);
	
	/* UART receiving */
	UART_response_packet(incoming_buffer);
	
	/* Error--> Message output */
	case_error(incoming_buffer);
}

//------------------------------------------------------------------------------
//    Name:        enroll_1
//    Description: Make 1st template for an enrollment
//    Input:       -
//    Output:      Error message ( Still need to check which ones )
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
	_delay_ms(1000);
	
	/* UART receiving */
	UART_response_packet(incoming_buffer);
	
	/* Error--> Message output */
	case_error(incoming_buffer);
}

//------------------------------------------------------------------------------
//    Name:        enroll_2
//    Description: Make 2nd template for an enrollment
//    Input:       -
//    Output:      Error message ( Still need to check which ones )
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
	_delay_ms(1000);
	
	/* UART receiving */
	UART_response_packet(incoming_buffer);
	
	/* Error--> Message output */
	case_error(incoming_buffer);
}

//------------------------------------------------------------------------------
//    Name:        enroll_3
//    Description: Make 3rd template for an enrollment, merge three templates into one template, save merged template to the database
//    Input:       -
//    Output:       Error message ( Still need to check which ones )
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
	_delay_ms(1000);
	
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
//    Output:       Error message ( Still need to check which ones )
//    Misc:		   -
//------------------------------------------------------------------------------
int is_press_finger()
{
	if( ON_OFF_BACKLIGHT == 0 )
	{
		lcd_clrscr();
		lcd_puts("Press key 3\nFor Backlight");
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
		_delay_ms(1000);
	
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
//    Output:      Error message ( Still need to check which ones )
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
	_delay_ms(1000);
			
	/* UART receiving */
	UART_response_packet(incoming_buffer);	
	
	/* Error--> Message output */
	case_error(incoming_buffer);
}

//------------------------------------------------------------------------------
//    Name:        ID_identify
//    Description: Checks if fingerprint is already in database
//    Input:       -
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
	_delay_ms(1000);
	
	/* UART receiving */
	UART_response_packet(incoming_buffer);
	
	/* Error--> Message output */
	case_error(incoming_buffer);
	
	lcd_clrscr();
	
	/* If parameter is 0 then fingerprint is found in database*/ 
	if(incoming_buffer[5] == 0x00)
	{
		lcd_puts("Verified ID");
		/* Servo motor LEFT */
// 		DDRD = (1 << PD5);
// 		OCR1A = ICR1 - 53;
// 		_delay_ms(100);
// 		DDRD |= (0 << PD5);	
	}
	else
	{
		lcd_puts("Finger not found");
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
//    Description: Deletes all fingerprints from database---> Need to implement some sort of security check
//    Input:       -
//    Output:      Error message ( Still need to check which ones )
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
	_delay_ms(1000);
	
	/* UART receiving */
	UART_response_packet(incoming_buffer);
	
	/* Error--> Message output */
	case_error(incoming_buffer);
		
}


//------------------------------------------------------------------------------
//    Name:        get_enroll_count
//    Description: Returns number of fingerprints in database ( still need to finish )
//    Input:       -
//    Output:      Number of enrolled fingerprints or "The database is empty"
//    Misc:		   -
//------------------------------------------------------------------------------
int get_enroll_count(uint8_t ID)
{
	/* Buffer11[20] is here only for testing purpose */
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
	_delay_ms(1000);
	
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
//    Description: Parameters and command for OPEN function. Used to open connection to sensor.
//    Input:       -
//    Output:      Error message ( Still need to check which ones )
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
	_delay_ms(1000);
			
	/* UART receiving */
	UART_response_packet(incoming_buffer);	
}

//------------------------------------------------------------------------------
//    Name:			cmosled       
//    Description:	Parameters and commands for cmosled command
//    Input:       -
//    Output:       LED ON or OFF, Initially it's always OFF
//    Misc:		   -
//------------------------------------------------------------------------------
void cmosled()
{
	/* Dividing lower byte and higher byte from 16bit command to fit into 1 UART message */
	command[0] = get_low_byte(CMOSLED);
	command[1] = get_hight_byte(CMOSLED);
	
	/* If LED is OFF then we turn her ON --> Need to implement this code better!!*/ 
	if(ON_OFF_BACKLIGHT == 0)
	{
		parameter[0] = 0x01;
		ON_OFF_BACKLIGHT = 1;	
		lcd_clrscr();
		lcd_puts("Backlight is ON!");	
	}
	/* IF LED is ON we turn her OFF */
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
	_delay_ms(1000);
	 
	/* UART receiving */
	 UART_response_packet(incoming_buffer);
} 


//------------------------------------------------------------------------------
//    Name:        hex_polje_sum
//    Description: Assembling array of data to be send  
//    Input:       Key input ( ATM 4 keys )
//    Output:      1->Enrollment
//				   2->Delete all fingerprints from database
//				   3->Returning number of fingerprinsts in database
//				   4->Checking if fingerprint is in database ( Verification )
//    Misc:		   -
//------------------------------------------------------------------------------
void hex_polje_sum(uint8_t outgoing_packet[], uint8_t ID)
{
	
	/* Depends which button we press */
	switch (stanje)
	{
		/* Enrolling new fingerprint into database */
		case 1:
			stanje = 0;
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
			while( is_press_finger() == 1 ) lcd_puts("Remove finger"); _delay_ms(100);
			/* Wait while finger is pressed again for 2nd enrollment */	
			while( is_press_finger() == 0 ) _delay_ms(100);	
			/* Capture fingerprint in high quality */								
			capture_finger();	
			
			/* 2nd Enrollment */															
			enroll_2();
			/* Wait while finger is removed */																		
			while( is_press_finger() == 1 ) lcd_puts("Remove finger"); _delay_ms(100);
			/* Wait while finger is pressed again for 3nd enrollment */	
			while( is_press_finger() == 0 ) _delay_ms(100);	
			/* Capture fingerprint in high quality */								
			capture_finger();
			
			/* 3rd Enrollment */																
			enroll_3();									
			break;
		
		case 2:
			stanje = 0;
			get_enroll_count(ID);
			break;
			
		
		case 3:
			stanje = 0;
			/*  Only here for testing purpose will be removed later */ 
			cmosled();		
			break;	
			
		/* When key 4 is pressed identification of fingerprint will begin */	
		case 4:
			stanje = 0;
			/* Check if LED is OFF */
			if( is_press_finger() == -1) break;
			/* Checking if finger is pressed */
			while( is_press_finger() == 0) _delay_ms(100);
			/* Capturing fingerprint so we can match it with fingerprints in database */
			capture_finger();
			/* Verification of our fingerprint, returns 1 or 0. */
			ID_identify(ID);
			break;
			
		case 5:
			stanje = 0;
			delete_all();
			break;
			
	}
}


int main(void)
{
	/* Used for buttons */
	/* Clear the bit---> 1 u 0, without touching other bits */
	DDRB &= ~(1 << PB0 | 1 << PB1 | 1 << PB2 | 1 <<PB3 | 1 << PB4); 
	/* Set the bit---> 0 u 1, without touching other bits */
	PORTB |= (1<< PB0 | 1 << PB1 | 1 << PB2 | 1 <<PB3 | 1 << PB4);
	/* LED for testing purpose (light up when UART is sending or receiving) */
	DDRA |= (1 << PA4 | 1 << PA5);
	/* Servo is initially OFF */
	DDRD |= (0 << PD5);	

	/* Inverted Mode */ 
	TCCR1A |= 1<<WGM11 | 1<<COM1A1 | 1<<COM1A0;
	
	/* Fast PWM mode 14, Prescaler = 256 */ 
	TCCR1B |= 1<<WGM13 | 1<<WGM12 | 1<<CS12;
	
	/* Top of the counter count */
	ICR1 = 625;

	/**/ 
	OCR1A = ICR1 - 65;

	/* UART initialization */ 
	uart0_init(UBRR); 
	
	/* Getting LCD ready */
	lcd_init(LCD_DISP_ON); 
	lcd_clrscr();
	lcd_puts("LCD is ready!!");
	

	/* ID number for fingerprint count */
	uint8_t ID = 0;						

	
	/* Enabling global interrupts */
    sei();
	
	/* On startup command OPEN is executed once */
	open(); 
	
	while(1)
    {
		
		if(bit_is_clear(PINB, PB0))
		{
			stanje = 1;		
		}
		
		if(bit_is_clear(PINB, PB1))
		{
			stanje = 2;	
		}
		
		if(bit_is_clear(PINB, PB2))
		{
			stanje = 3;		
		}
		
		if(bit_is_clear(PINB, PB3))
		{
			stanje = 4;		
		}
		
		if(bit_is_clear(PINB, PB4))
		{
			stanje = 5;
		}
               
        if(stanje == 1)
		{
			
			lcd_clrscr();
			lcd_puts("Prvo stanje");
			hex_polje_sum(outgoing_packet, ID);
		 }
		 
		 if(stanje == 2)
		 {
			
			lcd_clrscr();
			lcd_puts("Drugo stanje");
			hex_polje_sum(outgoing_packet, ID);
			
		}
		 
		if(stanje == 3)
		{
			lcd_clrscr();
			lcd_puts("Trece stanje");
			
			hex_polje_sum(outgoing_packet, ID);	
		}
		
		if(stanje == 4)
		{
			lcd_clrscr();
			lcd_puts("Cetvrto stanje");
			hex_polje_sum(outgoing_packet, ID);
		}
		
		if(stanje == 5)
		{
			lcd_clrscr();
			lcd_puts("Peto stanje");
			
			hex_polje_sum(outgoing_packet, ID);
		}
	}

}