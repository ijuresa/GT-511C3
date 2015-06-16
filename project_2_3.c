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
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "lcd.h"
#include "uart.h"



#define baudRate 9600
#define UBRR F_CPU/16/baudRate-1

#define COMMAND_START_CODE_1 0x55	//Static byte to mark the beginning of a command packet
#define COMMAND_START_CODE_2 0xAA	//Static byte to mark the beginning of a command packet
#define COMMAND_DEVICE_ID_1	0x01	//Device ID byte - lower	
#define COMMAND_DEVICE_ID_2	0x00	//Device ID byte - higher	

//
//	COMMAND PACKET (COMMAND)
//

#define COMMAND_OPEN 0x0001					//Open Initialization
#define COMMAND_CMOSLED 0x0012				//CMOSLED control (backlight)
#define GET_ENROLL_COUNT 0x0020				//Get enrolled fingerprint count
#define CHECK_ENROLLED 0x0021				//Check whether the specified ID is already enrolled
#define ENROLL_START 0x0022					//Start and enrollment
#define ENROLL_1 0x0023						//Make 1st template for an enrollment
#define ENROLL_2 0x0024						//Make 2nd template for an enrollment
#define ENROLL_3 0x0025						//Make 3rd template for an enrollment, merge three templates into one template, save merged template to the database
#define IS_PRESS_FINGER 0x0026				//Check is finger is placed on the sensor
#define DELETE_ID 0x0040					//Delete the fingerprint with the specified ID
#define DELETE_ALL 0x0041					//Delete all fingerprints from the database
#define VERIFY 0x0051						//1:1 Verification of the capture fingerprint image with the specified ID
#define ID_IDENTIFY 0x0051					//1:N Identification of the capture fingerprint image with the database
#define VERIFY_TEMPLATE 0x0052				//1:1 Verification of a fingerprint template with the database
#define IDENTIFY_TEMPLATE 0x0053			//1:N Identification of a fingerprint template with the database
#define CAPTURE_FINGER 0x0060				//Capture a fingerprint image(256x256) from the sensor
#define MAKE_TEMPLATE 0x0061				//Make template for transmission

//
//	RESPONSE PACKET (ACKNOWLEDGE) ->8 and 9th byte
//		
#define ACK_low 0x30
#define NACK_low 0x31 
#define ACK_NACK_high 0x00

//
//ERROR CODES - When response packet is NACK
//	
#define NACK_TIMEOUT 0x1001					//Obsolete, Capture timeout
#define NACK_INVALID_BAUDRATE 0x1002		//Obsolete, Invalid serial baud rate
#define NACK_INVALID_POS 0x1003				//The specified ID is not between 0-199
#define NACK_IS_NOT_USED 0x1004				//The specified ID is not used
#define NACK_IS_ALREADY_USED 0x1005			//The specified ID is already used
#define NACK_COMM_ERR 0x1006				//Communication error
#define NACK_VERIFY_FAILED 0x1007			//1:1 Verification failure
#define NACK_IDENTIFY_FAILED 0x1008			//1:N Identification failure
#define NACK_DB_IS_FULL 0x1009				//The database is full
#define NACK_DB_IS_EMPTY 0x100A				//The database is empty
#define NACK_TURN_ERR 0x100B				//Obsolete, Invalid order of enrollment (The order was not as: ENROLL_START->ENROLL_1->ENROLL_2->ENROLL_3)
#define NACK_BAD_FINGER 0x100C				//Too bad fingerprint
#define NACK_ENROLL_FAILED 0x100D			//Enrollment failure
#define NACK_IS_NOT_SUPPORTED 0x100E		//The specified command is not supported
#define NACK_DEV_ERR 0x100F					//Device error, especially if Crypto_chip is trouble
#define NACK_CAPTURE_CANCELED 0x1010		//Obsolete, The capturing is canceled
#define NACK_INVALID_PARAM 0x1011			//Invalid parameter
#define NACK_FINGER_IS_NOT_PRESSED 0x1012	//Finger is not pressed
//#define DUPLICATED_ID	0-199				//There is duplicated fingerprint (While enrollment or setting template), this error describes just duplicated ID




/*  Variables  */
uint16_t checksum;					//Variable for checksum
uint8_t outgoing_packet[12];		//Sending data
uint8_t incoming_buffer[12];		//Response packet
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
//    Name:        UART_send_packet
//    Description: Sending packet via UART to sensor.
//    Input:       -
//    Output:      LED blinks when data is sent
//    Misc:		   -
//------------------------------------------------------------------------------

void UART_send_packet(uint8_t outgoing_packet[])
{
	uint8_t i;
	PORTA = 1 << PA4;
	_delay_ms(250);
	
	//Actually sending array via UART to sensor
	for(i = 0; i < 12; i++)
	{
		uart0_putc(outgoing_packet[i]);
	}
	
	PORTA = 0 << PA4;
	_delay_ms(250);
}


//------------------------------------------------------------------------------
//    Name:        UART_response_packet
//    Description: Receive UART packet from GT-511C3 sensor.
//    Input:       -
//    Output:      -
//    Misc:		   -
//------------------------------------------------------------------------------
void UART_response_packet(uint8_t incoming_buffer[])
{	
	uint16_t n_data_plus_error[12]; //16 bit data array for data + error response
    uint8_t i, n_data[12], n_error_code;
        
    for( i=0; i<12; i++ )    //Input Array | dividing low and high bits and putting data into input array
    {
		
		n_data_plus_error[i] = uart0_getc();
		
        // extract/cast "data" from error+data array -> Lower 8 bits
        n_data[i] = (uint8_t)(n_data_plus_error[i] & 0x00FF);
		
        // extract/cast "error" from error+data array -> higher 8 bits  
        n_error_code = (uint8_t)((n_data_plus_error[i] & 0xFF00) >> 8);  
		 
		PORTA = 1 << PA4;
		_delay_ms(250);
		
		//Putting response data into our array
		incoming_buffer[i] = n_data[i];
		
		PORTA = 0 << PA4;
		_delay_ms(250); 
	}

}

//------------------------------------------------------------------------------
//    Name:        parameter_OPEN
//    Description: Parameters for OPEN function. Will be removed to seperate file or be more clearly defined.
//					Only here for testing purpose.
//    Input:       -
//    Output:      -
//    Misc:		   -
//------------------------------------------------------------------------------
void parameter_OPEN(uint8_t parameter[])
{
	parameter[0] = 0x01;
	parameter[1] = 0x00;
	parameter[2] = 0x00;
	parameter[3] = 0x00;
}


//------------------------------------------------------------------------------
//    Name:        parameter_calculate_OPEN
//    Description: Parameters for CMOSLED function.
//    Input:       -
//    Output:      -
//    Misc:		   -
//------------------------------------------------------------------------------
void parameter_CMOSLED(uint8_t parameter[])
{
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
	
} 

//------------------------------------------------------------------------------
//    Name:        lower_checksum
//    Description: Dividing 16_bit variable (cmd) to it's lower 8 bit value
//					0x01_13--->lower_byte = 0x13
//    Input:       -
//    Output:      -
//    Misc:		   -
//------------------------------------------------------------------------------
int lower_checksum(uint16_t cmd)
{
	 uint8_t lower_byte = 0;
	 return lower_byte = cmd & 0x00FF;
}

//------------------------------------------------------------------------------
//    Name:        lower_checksum
//    Description: Dividing 16_bit variable (cmd) to it's higher 8 bit value
//					0x01_13--->higher_byte = 0x01
//    Input:       -
//    Output:      -
//    Misc:		   -
//------------------------------------------------------------------------------
int higher_checksum(uint16_t cmd)
{
	uint8_t higher_byte = 0;
	return higher_byte = (cmd >> 8) & 0x00FF;
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
//    Name:        hex_polje_sum
//    Description: Assembling array of data to be send  
//    Input:       -
//    Output:      -
//    Misc:		   -
//------------------------------------------------------------------------------
void hex_polje_sum(uint8_t outgoing_packet[])
{
	uint8_t parameter[4], command[2];
	
	//Ovisno koji gumb pritisnemo baca nas u drugi case
	switch (stanje)
	{
		//When we press key1 sensor opens communication
		case 1:
			stanje = 0;
			parameter_OPEN(parameter);
			command[0] = lower_checksum(COMMAND_OPEN);
			command[1] = higher_checksum(COMMAND_OPEN);
			
			break;
			
		case 3:
		//When we press key3 sensor turns ON/OFF backlight ( CMOSLED )
			stanje = 0;
			parameter_CMOSLED(parameter);
			command[0] = lower_checksum(COMMAND_CMOSLED);
			command[1] = higher_checksum(COMMAND_CMOSLED);
					
			break;	
	}
		
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
	outgoing_packet[10] = lower_checksum(checksum);
	outgoing_packet[11] = higher_checksum(checksum);
	
}

int main(void)
{
	//Used for buttons
	DDRB &= ~(1 << PB0 | 1 << PB1 | 1 << PB2 | 1 <<PB3); //Clear the bit---> 1 u 0, without touching other bits
	PORTB |= (1<< PB0 | 1 << PB1 | 1 << PB2 | 1 <<PB3); //Set the bit---> 0 u 1, without touching other bits
	DDRA |= (1<<PA4);	//LED for testing purpose (light up when UART is sending)
	
	//UART initialization 
	uart0_init(UBRR); 
	
	//Getting LCD ready
	lcd_init(LCD_DISP_ON); 
	lcd_clrscr();
	lcd_puts("LCD is ready!!");
	
	uint8_t stanje2 = 0;  //Used for testing purpose with KEY2
	
	int i;
    char buffer[50], buffer2[10];	//Used for itoa() function, to check if sending/response packet is good
	
	//Enabling global interrupts
    sei();
	
	while(1)
    {
		if(bit_is_clear(PINB, PB0))
		{
			stanje = 1;		
		}
		
		if(bit_is_clear(PINB, PB1))
		{
			//stanje = 2;	
			stanje2 = 2;	
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
			hex_polje_sum(outgoing_packet);
			
			//UART sending
			UART_send_packet(outgoing_packet);
			_delay_ms(1000);
			
			//UART receiving
			UART_response_packet(incoming_buffer);
	
		 }
		 
		 if(stanje2 == 2)
		 {
			 	
			 //Output to LCD for testing purpose
			 for( i = 0; i < 12; i++)
			 {
				 
				 lcd_clrscr();
				 itoa(outgoing_packet[i],buffer,16);  //COMMAND packet check (outgoing ) in HEX
				 //itoa(incoming_buffer[i],buffer,16);  //RESPONSE packet check (incoming ) in HEX
				 lcd_gotoxy(1,0);
				 lcd_puts(buffer); 
				 
				 itoa(i,buffer2,10);  //dec
				 lcd_gotoxy(0,1);
				 lcd_puts(buffer2);  
				 _delay_ms(1000);  

			}	
		}
		 
		if(stanje == 3)
		{
			
			
			lcd_clrscr();
			lcd_puts("Trece stanje");
			
			hex_polje_sum(outgoing_packet);
			
			
			 //UART sending
			 UART_send_packet(outgoing_packet);
			 _delay_ms(1000);
			 
			//UART receiving
			UART_response_packet(incoming_buffer);
			
		}
		  
		if(stanje == 4)
		{
			   
		}
	}

}