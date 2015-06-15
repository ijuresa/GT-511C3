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
 *		Finish receive packet and try to send complete array to sensor not seperate bytes
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
//
//	COMMAND PACKET (COMMAND)
//
#define COMMAND_START_CODE_1 0x55
#define COMMAND_START_CODE_2 0xAA
#define COMMAND_DEVICE_ID_1	0x01
#define COMMAND_DEVICE_ID_2	0x00
#define COMMAND_OPEN 0x0001
#define COMMAND_CMOSLED 0x0012

//
//	RESPONSE PACKET (ACKNOWLEDGE)
//		
#define ACK_low 0x30
#define NACK_low 0x31 
#define ACK_NACK_high 0x00



uint16_t checksum; 

uint8_t outgoing_packet[12];		//Sending data
uint8_t incoming_buffer[12];		//Response packet
uint8_t flag = 0;
uint8_t ON_OFF_BACKLIGHT = 0;
static volatile uint8_t stanje = 0;


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
//    Name:        response_packet
//    Description: Receive UART packet from GT-511C3 sensor.
//    Input:       -
//    Output:      -
//    Misc:		   -
//------------------------------------------------------------------------------
void response_packet(uint8_t incoming_buffer[])
{	
	uint16_t m_primljena_rijec[12];
    uint8_t i, n_podatak[12], n_error_code;
        
    for( i=0; i<12; i++ )    //Input Array | dividing low and high bits and putting data into input array
    {
		
		m_primljena_rijec[i] = uart0_getc();
		
        // extract/cast "data" iz error+data polja    
        n_podatak[i] = (uint8_t)(m_primljena_rijec[i] & 0x00FF);
		
        // extract/cast "error" iz error+data polja   
        n_error_code = (uint8_t)((m_primljena_rijec[i] & 0xFF00) >> 8);   
		PORTA = 1 << PA4;
		_delay_ms(250);
		incoming_buffer[i] = n_podatak[i];
		
//         switch( n_podatak )
// 		{
// 			case COMMAND_START_CODE_1:
// 				incoming_buffer[0] = COMMAND_START_CODE_1;
// 				break;
// 				   
//             case COMMAND_START_CODE_2:
//                 incoming_buffer[1] = COMMAND_START_CODE_2;
//                 break;
// 					
//             case COMMAND_DEVICE_ID_1:
//                 incoming_buffer[2] = COMMAND_DEVICE_ID_1;
//                 break;
// 					
//             case COMMAND_DEVICE_ID_2:
//                 incoming_buffer[3] = COMMAND_DEVICE_ID_2;
//                 break;
// 				
// 			case ACK_low:
// 				incoming_buffer[8] = ACK_low;
// 				break;
// 			
// 			case NACK_low:
// 				incoming_buffer[8] = NACK_low;
// 				break;
// 		}   
		
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
		case 1:
			stanje = 0;
			parameter_OPEN(parameter);
			command[0] = lower_checksum(COMMAND_OPEN);
			command[1] = higher_checksum(COMMAND_OPEN);
			
			break;
			
		case 3:
			stanje = 0;
			parameter_CMOSLED(parameter);
			command[0] = lower_checksum(COMMAND_CMOSLED);
			command[1] = higher_checksum(COMMAND_CMOSLED);
					
			break;	
	}
		
	//Racunanje checksum-a
	checksum = calculate_checksum(parameter, command);
	
	
	//Paket koji saljem UART-om prema senzoru
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
	
	//Rucno saljem polje jer ne radi na uart0_putc(outgoing_packet[i] u FOR petlji
	if(flag == 1)
	{
		PORTA = 1 << PA4;
		_delay_ms(250);
		uart0_putc(outgoing_packet[0]);
		uart0_putc(outgoing_packet[1]);
		uart0_putc(outgoing_packet[2]);
		uart0_putc(outgoing_packet[3]);
		uart0_putc(outgoing_packet[4]);
		uart0_putc(outgoing_packet[5]);
		uart0_putc(outgoing_packet[6]);
		uart0_putc(outgoing_packet[7]);
		uart0_putc(outgoing_packet[8]);
		uart0_putc(outgoing_packet[9]);
		uart0_putc(outgoing_packet[10]);
		uart0_putc(outgoing_packet[11]);
		
		PORTA = 0 << PA4;
		_delay_ms(250);
		flag = 0;
	}
}

int main(void)
{
	DDRB &= ~(1 << PB0 | 1 << PB1 | 1 << PB2 | 1 <<PB3); //Clear the bit---> 1 u 0, bez diranja ostalih bitova
	PORTB |= (1<< PB0 | 1 << PB1 | 1 << PB2 | 1 <<PB3); //Set the bit---> 0 u 1, bez diranja ostalih bitova
	DDRA |= (1<<PA4);	//LED for testing purpose (light up when UART is sending)

	uart0_init(UBRR); 
	
	lcd_init(LCD_DISP_ON); 
	lcd_clrscr();
	lcd_puts("LCD is ready!!");
	
	uint8_t stanje2 = 0;  //Ovo je samo za key2, da mi se ne mijesa sa "stanjem" sa kojim vrtim switch case
	
	int i;
    char buffer[50], buffer2[10];	//Sluzi za for petlju ( itoa ) da provjerim da li je dobro sastavio sve
	
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
		
			flag = 1;
			lcd_clrscr();
			lcd_puts("Prvo stanje");
			hex_polje_sum(outgoing_packet);
			
			
			_delay_ms(1000);
			//UART receiving
			response_packet(incoming_buffer);
			_delay_ms(1000);
		 }
		 
		 if(stanje2 == 2)
		 {
			 	
			 //Jednostavan ispit na lcd-u da provjerim jer je paket dobar, atm se provjerava response koji nevalja
			 for( i = 0; i < 12; i++)
			 {
				 
				 lcd_clrscr();
				 itoa(outgoing_packet[i],buffer,16);  //Provjera COMMAND paketa hex
				 //itoa(incoming_buffer[i],buffer,16);  //Provjera RECEIVE paketa hex
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
			
			flag = 1;
			lcd_clrscr();
			lcd_puts("Trece stanje");
			
			hex_polje_sum(outgoing_packet);
			 _delay_ms(1000);
			
			 
			//UART receiving
			response_packet(incoming_buffer);
			_delay_ms(1000);
		}
		  
		if(stanje == 4)
		{
			   
		}
	}

}