/*
 * project_2_3.c
 *
 * Created: 28/05/2015 18:49:02
 *  Author: Ivan
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

//COMMAND PACKET (COMMAND)
#define COMMAND_START_CODE_1 0x55
#define COMMAND_START_CODE_2 0xAA
#define COMMAND_DEVICE_ID_1	0x01
#define COMMAND_DEVICE_ID_2	0x00
#define COMMAND_OPEN 0x0001
#define COMMAND_CMOSLED 0x0012

//RESPONSE PACKET (ACKNOWLEDGE)


static volatile uint16_t checksum; 
static volatile uint8_t	stanje = 0;
static volatile uint8_t outgoing_packet[12];		//Sending data
static volatile uint16_t incoming_buffer[12];	//Response packet


/*##########################################################################
UBBR->	Baud Rate Register, konektiran na down-counter -> funkcioniraju kao programmable prescaler or baud rate generator.
		down-counter -> running at system clock (F_CPU ili fosc) is loaded with the UBRR value each time the counter 
						has counted to zero
						or when the UBRRL Register is written.
		Clock ->	generiran svaki put kada counter dode do 0
									
Koristimo Asynchronous Normal Mode (U2X = 0)
 
###########################################################################*/

//#define UART_BAUD_SELECT(baudRate, F_CPU)

/*##########################################################################
parameter_to_int(int i)---> U i-u saljemo Input Parameter, ovisno o komandi koju izvodimo.(Vecinom je 0)

##########################################################################*/

//RESPONSE PAKET JOS NIJE GOTOV
/*##########################################################################
Cilj response paketa bi bio da vecinom usporeduje da li je senzor dobro odgovorio.
	RESPONSE_PACKET izgleda skoro isti kao i COMMAND_PACKET samo je razlika u Parametru i Response ( bivsi Command ) sve ostalo je isto.
	Znaci on bi meni trebao vratiti:
	0x55
	0xAA
	0x01
	0x00
	etc
		Ova prva 4 se provjeravaju dolje. ATM vraca sve 0.
		
############################################################################*/
uint8_t response_packet(uint8_t incoming_buffer[])
{	
	uint16_t m_primljena_rijec, buffer19[20];
    uint8_t i, n_podatak, n_error_code;
        
    for( i=0; i<4; i++ )    // evaluiramo sadržaj prvih 4 primljenih 16-bit word-ova
    {
		m_primljena_rijec = uart0_getc();
        // extract/cast "data" iz error+data polja    
        n_podatak = (uint8_t)(m_primljena_rijec & 0x00FF);
        // extract/cast "error" iz error+data polja
        // ...nek' se nadje, mozda se pojavi     
        n_error_code = (uint8_t)((m_primljena_rijec & 0xFF00) >> 8);   
	
        switch( n_podatak )
		{
			case COMMAND_START_CODE_1:
				incoming_buffer[0] = COMMAND_START_CODE_1;
				break;
				   
            case COMMAND_START_CODE_2:
                incoming_buffer[1] = COMMAND_START_CODE_2;
                break;
					
            case COMMAND_DEVICE_ID_1:
                incoming_buffer[2] = COMMAND_DEVICE_ID_1;
                break;
					
            case COMMAND_DEVICE_ID_2:
                incoming_buffer[3] = COMMAND_DEVICE_ID_2;
                break;
		}

        // ukoliko n_error_code, koji je povratni "error" podatak iz uart0_getc(), i ako sadrži više stanja
        // onda bi se mogla napraviti evaluacija stanja korištenjem switch izraza, 
        // ina?e taj error kod se može, ali i nemora, hendlati u pozivaju?oj funkciji
        
	}
	return n_error_code;    // 0x00 is OK/NoERROR	
}


//Parametri za funkcjiu OPEN--> Otvara konekciju prema senzoru
void parameter_calculate_OPEN(uint8_t parameter[])
{
	parameter[0] = 0x00;
	parameter[1] = 0x00;
	parameter[2] = 0x00;
	parameter[3] = 0x00;
}


/*############################################################################
Parametri za funkciju CMOSLED--> Trebalo bi upaliti backlight

	Znaci prvotno je LED uvijek ugasena, zato prvo provjeravam da li je upaljena if(parameter[0] == 0x01).
	
############################################################################*/
void parameter_CMOSLED(uint8_t parameter[])
{
	if(parameter[0] == 0x01) //Provjerava da li je upaljena
	{
		parameter[0] = 0x00;		//Ugasit ce je	
		parameter[1] = 0x00;
		parameter[2] = 0x00;
		parameter[3] = 0x00;
	}
	else						
	{
		parameter[0] = 0x01;		//Upalit ce je	
		parameter[1] = 0x00;
		parameter[2] = 0x00;
		parameter[3] = 0x00;
	}	
} 

//Izdvaja lower_checksum (Npr; 0x01_13--->lower_byte = 0x13)
int lower_checksum(uint16_t cmd)
{
	 uint8_t lower_byte = 0;
	 return lower_byte = cmd & 0x00FF;
}

//Izdvaja higher_checksum (Npr; 0x01_13--->higher_byte = 0x01)
int higher_checksum(uint16_t cmd)
{
	uint8_t higher_byte = 0;
	return higher_byte = (cmd >> 8) & 0x00FF;
}

//Racuna checksum, zbraja sve hex brojeve ( provjera )
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
/*########################################################################################
Ovdje se skace odmah iz maina, te se dalje grana-> racunaju se parametri, command, checksum
	te se sastavlja izlazni paket

#############################################################################################*/
void hex_polje_zbroj(uint8_t outgoing_packet[])
{
	uint8_t parameter[4], command[2], i;
	
	//Ovisno koji gumb pritisnemo baca nas u drugi case
	switch (stanje)
	{
		case 1:
			parameter_calculate_OPEN(parameter);
			stanje = 0;
			break;
			
		case 3:
			parameter_CMOSLED(parameter);
			stanje = 0;		
			break;	
	}
	
	//Rastavljanje command
	command[0] = lower_checksum(COMMAND_OPEN);
	command[1] = higher_checksum(COMMAND_OPEN);
	
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
}

int main(void)
{
	DDRB &= ~(1 << PB0 | 1 << PB1 | 1 << PB2 | 1 <<PB3); //Clear the bit---> 1 u 0, bez diranja ostalih bitova
	PORTB |= (1<< PB0 | 1 << PB1 | 1 << PB2 | 1 <<PB3); //Set the bit---> 0 u 1, bez diranja ostalih bitova
	//DDRA |= (1<<PA4);	//Ledica
	/*
	 MCUCR = 0x0f;
     GICR = 0xC0;

     sei();
	 
	 TCCR1B = 1<<CS10 | 1<<WGM13 | 1<<WGM12;
     TCCR1A = 1<<WGM10 | 1<<WGM11 | 1<<COM1B1 | 1<<COM1B0;

     OCR1A = 7372;
     OCR1B = 4500;
	 */
	 /***************************************************************************
	Inicijalizacija UARTA ---> UBBR je 51
	
	****************************************************************************/
	uart0_init(UBRR); 
	
	lcd_init(LCD_DISP_ON); 
	lcd_clrscr();
	lcd_puts("LCD is ready!!");
	
	uint8_t stanje2 = 0;  //Ovo je samo za key2, da mi se ne mijesa sa "stanjem" sa kojim vrtim switch case
	int i;
    char buffer[50], buffer2[10];	//Sluzi za for petlju ( itoa ) da provjerim da li je dobro sastavio sve
   
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
			hex_polje_zbroj(outgoing_packet);
			
			//UART sending
			for( i = 0; i > 12; i++)
			{
				uart0_putc(	outgoing_packet[i] );
			}
			
			//UART receiving
			response_packet(incoming_buffer);
		 }
		 
		 if(stanje2 == 2)
		 {
			 //Jednostavan ispit na lcd-u da provjerim jer je paket dobar, atm se provjerava response koji nevalja
			 for( i = 0; i < 12; i++)
			 {
				 lcd_clrscr();
				 itoa(outgoing_packet[i],buffer,16);  //Provjera COMMAND paketa hex
				 //itoa(incoming_buffer[i],buffer,16);  //hex
				 lcd_gotoxy(1,0);
				 lcd_puts(buffer); 
				 
				 itoa(i,buffer2,10);  //dec
				 lcd_gotoxy(0,1);
				 lcd_puts(buffer2);  
				 _delay_ms(250);  
			}	
		}
		 
		if(stanje == 3)
		{
			lcd_clrscr();
			lcd_puts("Trece stanje");
			hex_polje_zbroj(outgoing_packet);
			 
			//UART sending
			for( i = 0; i > 12; i++)
			{
				uart0_putc(	outgoing_packet[i] );
			}
		}
		  
		if(stanje == 4)
		{
			   
		}
	}

}