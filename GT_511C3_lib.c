/*
 * GT_511_lib.c
 *
 * Created: 16/06/2015 13:50:09
 *  Author: Ivan
 */ 

#ifndef F_CPU
#define F_CPU 8000000UL							//Clock
#endif

#include "GT_511C3_lib.h"


//------------------------------------------------------------------------------
//    Name:        get_low_byte
//    Description: Dividing 16_bit variable (cmd) to it's lower 8 bit value
//					0x01_13--->lower_byte = 0x13
//    Input:       -
//    Output:      -
//    Misc:		   -
//------------------------------------------------------------------------------
int get_low_byte(uint16_t cmd)
{
	uint8_t lower_byte = 0;
	return lower_byte = cmd & 0x00FF;
}

//------------------------------------------------------------------------------
//    Name:        get_high_byte
//    Description: Dividing 16_bit variable (cmd) to it's higher 8 bit value
//					0x01_13--->higher_byte = 0x01
//    Input:       -
//    Output:      -
//    Misc:		   -
//------------------------------------------------------------------------------

int get_high_byte(uint16_t cmd)
{
	uint8_t higher_byte = 0;
	return higher_byte = (cmd >> 8) & 0x00FF;
}


//------------------------------------------------------------------------------
//    Name:        case_error
//    Description: Comparing lower byte of message errors with incoming data of same lower data			
//    Input:       -
//    Output:      Message error is written on LCD
//    Misc:		   -
//------------------------------------------------------------------------------
void case_error(uint8_t incoming_buffer[])
{
	/* Need to change output message depending on what's going on */
	if(incoming_buffer[5] == ACK_NACK_high)
	{
		/*lcd_puts("No error");*/
	}
	
	/* Output of errors when parameter[0] is NACK --> 0x31 */
	else
	{
		lcd_clrscr();
		switch(incoming_buffer[4])
		{
			case 0x01:  lcd_puts("Capture timeout"); break;
			case 0x02:  lcd_puts("Invalid serial \nbaud rate"); break;
			case 0x03:  lcd_puts("The specified ID \nis not between 0-199"); break;
			case 0x04:  lcd_puts("The specified ID \nis not used"); break;
			case 0x05:  lcd_puts("The specified ID \nis already used"); break;
			case 0x06:  lcd_puts("Communication \nerror"); break;
			case 0x07:  lcd_puts("1:1 Verification \nfailure"); break;
			case 0x08:  lcd_puts("1:N Identification \nfailure"); break;
			case 0x09:  lcd_puts("The database is \nfull"); break;
			case 0x0A:  lcd_puts("The database is \nempty"); break;
			case 0x0B:  lcd_puts("Invalid order of \nenrollment"); break;
			case 0x0C:  lcd_puts("Too bad \nfingerprint"); break;
			case 0x0D:  lcd_puts("Enrollment \nfailure"); break;
			case 0x0E:  lcd_puts("The specified \ncommand is not supported"); break;
			case 0x0F:  lcd_puts("Device error"); break;
			case 0x10:  lcd_puts("The capturing \nis canceled"); break;
			case 0x11:  lcd_puts("Invalid parameter"); break;
			case 0x12:  lcd_puts("Press finger!"); break;
		}
	}
	
	
	
}


