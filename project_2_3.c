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
 *To DO:
 *		
 */ 



#ifndef F_CPU
#define F_CPU 8000000UL							//Clock
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

#define baudRate 9600
#define UBRR F_CPU/16/baudRate-1




/*  Variables  */
uint16_t checksum;					//Variable for checksum
uint8_t outgoing_packet[12];		//Sending data
uint8_t incoming_buffer[12];		//Response packet
uint8_t parameter[4], command[2];
uint8_t ON_OFF_BACKLIGHT = 0;		//Always off on begining
static volatile uint8_t stanje = 0;	//Used for keys




//------------------------------------------------------------------------------
//    Name:        -
//    Description: Functions to be implemented
//    Input:       -
//    Output:      -
//    Misc:		   -
//------------------------------------------------------------------------------
/*
	
			Here goes what needs to be implemented

*/



//------------------------------------------------------------------------------
//    Name:        parameter_from_int
//    Description: Integer to hex
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
//    Description: Assembling data packet to be sent
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
//    Description: Check whether the specified ID is already enrolled, we search for the empty one so we can enroll there
//    Input:       -
//    Output:      -
//    Misc:		   -
//------------------------------------------------------------------------------
void check_enrolled(uint8_t ID)
{
	parameter_from_int(ID);
	
					
	command[0] = get_low_byte(CHECK_ENROLLED);
	command[1] = get_hight_byte(CHECK_ENROLLED);
	
	assemble_data_packet();
	
	//UART sending
	UART_send_packet(outgoing_packet);
	_delay_ms(1000);
	
	//UART receiving
	UART_response_packet(incoming_buffer);
	lcd_clrscr();
	//ACK, which means that this ID is already used so we need to search for new one
	if(incoming_buffer[8] == 0x30)
	{
		lcd_puts("This ID is already\nEnrolled");
		ID++;
		check_enrolled(ID);
	}
	
	else case_error(incoming_buffer);

}

//------------------------------------------------------------------------------
//    Name:        enroll_start
//    Description: Start an enrollment
//    Input:       -
//    Output:      -
//    Misc:		   -
//------------------------------------------------------------------------------
void enroll_start(uint8_t ID)
{
	parameter_from_int(ID);
	
	command[0] = get_low_byte(ENROLL_START);
	command[1] = get_hight_byte(ENROLL_START);
	
	assemble_data_packet();
	
	//UART sending
	UART_send_packet(outgoing_packet);
	_delay_ms(1000);
	
	//UART receiving
	UART_response_packet(incoming_buffer);
	
	//Error--> Message output
	case_error(incoming_buffer);
		
}

//------------------------------------------------------------------------------
//    Name:        enroll_1
//    Description: Make 1st template for an enrollment
//    Input:       -
//    Output:      -
//    Misc:		   -
//------------------------------------------------------------------------------
void enroll_1()
{
	parameter[0] = 0x00;
	parameter[1] = 0x00;
	parameter[2] = 0x00;
	parameter[3] = 0x00;
	
	command[0] = get_low_byte(ENROLL_1);
	command[1] = get_hight_byte(ENROLL_1);
	
	assemble_data_packet();
	
	//UART sending
	UART_send_packet(outgoing_packet);
	_delay_ms(1000);
	
	//UART receiving
	UART_response_packet(incoming_buffer);
	
	//Error--> Message output
	case_error(incoming_buffer);
}


//------------------------------------------------------------------------------
//    Name:        enroll_2
//    Description: Make 2nd template for an enrollment
//    Input:       -
//    Output:      -
//    Misc:		   -
//------------------------------------------------------------------------------
void enroll_2()
{
	parameter[0] = 0x00;
	parameter[1] = 0x00;
	parameter[2] = 0x00;
	parameter[3] = 0x00;
	
	command[0] = get_low_byte(ENROLL_2);
	command[1] = get_hight_byte(ENROLL_2);
	
	assemble_data_packet();
	
	//UART sending
	UART_send_packet(outgoing_packet);
	_delay_ms(1000);
	
	//UART receiving
	UART_response_packet(incoming_buffer);
	
	//Error--> Message output
	case_error(incoming_buffer);
}

//------------------------------------------------------------------------------
//    Name:        enroll_3
//    Description: Make 3rd template for an enrollment, merge three templates into one template, save merged template to the database
//    Input:       -
//    Output:      -
//    Misc:		   -
//------------------------------------------------------------------------------
void enroll_3()
{
	parameter[0] = 0x00;
	parameter[1] = 0x00;
	parameter[2] = 0x00;
	parameter[3] = 0x00;
	
	command[0] = get_low_byte(ENROLL_3);
	command[1] = get_hight_byte(ENROLL_3);
	
	assemble_data_packet();
	
	//UART sending
	UART_send_packet(outgoing_packet);
	_delay_ms(1000);
	
	//UART receiving
	UART_response_packet(incoming_buffer);
	
	//Error--> Message output
	case_error(incoming_buffer);
}

//------------------------------------------------------------------------------
//    Name:			is_press_finger
//    Description:	Check is finger is placed on the sensor
//    Input:       -
//    Output:      -
//    Misc:		   -
//------------------------------------------------------------------------------
int is_press_finger()
{
	
	parameter[0] = 0x00;
	parameter[1] = 0x00;
	parameter[2] = 0x00;
	parameter[3] = 0x00;
	
	command[0] = get_low_byte(IS_PRESS_FINGER);
	command[1] = get_hight_byte(IS_PRESS_FINGER);
	
	assemble_data_packet();
	
	//UART sending
	UART_send_packet(outgoing_packet);
	_delay_ms(1000);
	
	//UART receiving
	UART_response_packet(incoming_buffer);
	
	//Error--> Message output
	case_error(incoming_buffer);
	
	if(incoming_buffer[4] == 0x00) return 1;
	else return 0;
}

//------------------------------------------------------------------------------
//    Name:        capture_finger
//    Description: 		
//    Input:       -
//    Output:      -
//    Misc:		   -
//------------------------------------------------------------------------------
void capture_finger()
{
	command[0] = get_low_byte(CAPTURE_FINGER);
	command[1] = get_hight_byte(CAPTURE_FINGER);
	
	parameter_from_int(1); //For best image, slow
	
	assemble_data_packet();

	//UART sending
	UART_send_packet(outgoing_packet);
	_delay_ms(1000);
			
	//UART receiving
	UART_response_packet(incoming_buffer);	
	
	//Error--> Message output
	case_error(incoming_buffer);
	
}

//------------------------------------------------------------------------------
//    Name:        ID_identify
//    Description: 
//    Input:       -
//    Output:      -
//    Misc:		   -
//------------------------------------------------------------------------------
void ID_identify(uint8_t ID)
{
	command[0] = get_low_byte(ID_IDENTIFY);
	command[1] = get_hight_byte(ID_IDENTIFY);
	
	parameter[0] = 0x00;
	parameter[1] = 0x00;
	parameter[2] = 0x00;
	parameter[3] = 0x00;
	
	assemble_data_packet();

	//UART sending
	UART_send_packet(outgoing_packet);
	_delay_ms(1000);
	
	//UART receiving
	UART_response_packet(incoming_buffer);
	
	//Error--> Message output
	case_error(incoming_buffer);
	
	lcd_clrscr();
	if(incoming_buffer[5] == 0x00)
	{
		lcd_puts("Verified ID");
	}
	else lcd_puts("Finger not found");
}

//------------------------------------------------------------------------------
//    Name:        delete_all
//    Description: 
//    Input:       -
//    Output:      -
//    Misc:		   -
//------------------------------------------------------------------------------
void delete_all()
{
	parameter[0] = 0x00;
	parameter[1] = 0x00;
	parameter[2] = 0x00;
	parameter[3] = 0x00;
	
	command[0] = get_low_byte(DELETE_ALL);
	command[1] = get_hight_byte(DELETE_ALL);
	
	assemble_data_packet();
	
	//UART sending
	UART_send_packet(outgoing_packet);
	_delay_ms(1000);
	
	//UART receiving
	UART_response_packet(incoming_buffer);
	
	//Error--> Message output
	case_error(incoming_buffer);
		
}


//------------------------------------------------------------------------------
//    Name:        open
//    Description: Parameters for OPEN function. Will be removed to different file or be more clearly defined.
//					Only here for testing purpose.
//    Input:       -
//    Output:      -
//    Misc:		   -
//------------------------------------------------------------------------------
void open()
{
	command[0] = get_low_byte(OPEN);
	command[1] = get_hight_byte(OPEN);
	
	parameter[0] = 0x00;
	parameter[1] = 0x00;
	parameter[2] = 0x00;
	parameter[3] = 0x00;
	
	assemble_data_packet();

	//UART sending
	UART_send_packet(outgoing_packet);
	_delay_ms(1000);
			
	//UART receiving
	UART_response_packet(incoming_buffer);	
	
}

//------------------------------------------------------------------------------
//    Name:			cmosled       
//    Description:	Parameters and commands for cmosled command
//    Input:       -
//    Output:      -
//    Misc:		   -
//------------------------------------------------------------------------------
void cmosled()
{
	command[0] = get_low_byte(CMOSLED);
	command[1] = get_hight_byte(CMOSLED);
	
	if(ON_OFF_BACKLIGHT == 0)
	{
		parameter[0] = 0x01;
		ON_OFF_BACKLIGHT = 1;		
	}
	else
	{
		parameter[0] = 0x00;
		ON_OFF_BACKLIGHT = 0;		
	}
	parameter[1] = 0x00;
	parameter[2] = 0x00;	
	parameter[3] = 0x00;	
	
	assemble_data_packet();
	
	 //UART sending
	 UART_send_packet(outgoing_packet);
	 _delay_ms(1000);
	 
	 //UART receiving
	 UART_response_packet(incoming_buffer);
} 


//------------------------------------------------------------------------------
//    Name:        hex_polje_sum
//    Description: Assembling array of data to be send  
//    Input:       -
//    Output:      -
//    Misc:		   -
//------------------------------------------------------------------------------
void hex_polje_sum(uint8_t outgoing_packet[], uint8_t ID)
{
	
	
	//Depends which button we press
	switch (stanje)
	{
		//Trying to enroll finger
		case 1:
			stanje = 0;
			check_enrolled(ID);																//Check for an ID slot that is unused until u find one:
			enroll_start(ID);																	//Start and enrollment with that ID
 			while( is_press_finger() == 0 ) _delay_ms(100);									//Wait while finger is pressed
			capture_finger();																//Capture fingerprint in high quality
			enroll_1();																		//1st Enrollment
			while( is_press_finger() == 1 ) lcd_puts("\nRemove finger"); _delay_ms(100);	//Wait while finger is removed
			while( is_press_finger() == 0 ) _delay_ms(100);									//Wait while finger is pressed again for 2nd enrollment
			capture_finger();																//Capture fingerprint in high quality
			enroll_2();																		//2nd Enrollment
			while( is_press_finger() == 1 ) lcd_puts("\nRemove finger"); _delay_ms(100);	//Wait while finger is removed
			while( is_press_finger() == 0 ) _delay_ms(100);									//Wait while finger is pressed again for 2nd enrollment
			capture_finger();																//Capture fingerprint in high quality
			enroll_3();																		//3rd Enrollment
			lcd_puts("\nEnrollment was successful");											

			break;
		
		case 2:
			stanje = 0;
			delete_all();
			break;
			
		//When we press key3 sensor turns ON/OFF backlight ( CMOSLED ) 
		case 3:
			stanje = 0;
			cmosled();		
			break;	
			
		case 4:
			stanje = 0;
			while( is_press_finger() == 0) _delay_ms(100);
			capture_finger();
			ID_identify(ID);
			
	}
}

int main(void)
{
	//Used for buttons
	DDRB &= ~(1 << PB0 | 1 << PB1 | 1 << PB2 | 1 <<PB3); //Clear the bit---> 1 u 0, without touching other bits
	PORTB |= (1<< PB0 | 1 << PB1 | 1 << PB2 | 1 <<PB3); //Set the bit---> 0 u 1, without touching other bits
	DDRA |= (1 << PA4 | 1 << PA5);	//LED for testing purpose (light up when UART is sending)
	
	//UART initialization 
	uart0_init(UBRR); 
	
	//Getting LCD ready
	lcd_init(LCD_DISP_ON); 
	lcd_clrscr();
	lcd_puts("LCD is ready!!");
	
	uint8_t i;
	uint8_t ID = 0;						//ID number for fingerprint count
	

    char buffer[50], buffer2[10];	//Used for itoa() function, to check if sending/response packet is good
	
	//Enabling global interrupts
    sei();
	
	open(); // On startup command OPEN is executed once
	
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
			
			
			// Output to LCD for testing purpose
// 			 for( i = 0; i < 12; i++)
// 			 {			 
// 				 lcd_clrscr();
// 				 // itoa(outgoing_packet[i],buffer,16);  //COMMAND packet check (outgoing ) in HEX
// 				 itoa(incoming_buffer[i],buffer,16);  //RESPONSE packet check (incoming ) in HEX
// 				 lcd_gotoxy(1,0);
// 				 lcd_puts(buffer); 
// 				 
// 				
// 				itoa(i,buffer2,10);  //dec
// 				lcd_gotoxy(0,1);
// 				lcd_puts(buffer2);  
// 				_delay_ms(1000);  
// 			}	
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
			
// 			   PORTA = 0 << PA5;
// 			   _delay_ms(1);
// 			   PORTA = 1 << PA5;
// 			   _delay_ms(1);
		}
	}

}