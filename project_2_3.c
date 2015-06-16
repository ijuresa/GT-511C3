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
#include "GT-511-lib.c"


#define baudRate 9600
#define UBRR F_CPU/16/baudRate-1




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