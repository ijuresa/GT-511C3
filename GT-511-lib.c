/*
 * GT_511_lib.c
 *
 * Created: 16/06/2015 13:50:09
 *  Author: Ivan
 */ 


#include <stdint.h>
#include <avr/io.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>


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

