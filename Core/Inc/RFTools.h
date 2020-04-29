/*
 * RFTools.h
 *
 *  Created on: Apr 10, 2020
 *      Author: maxim
 */

#ifndef INC_RFTOOLS_H_
#define INC_RFTOOLS_H_
#include "types.h"

//serialization parameters
//0x93|0xC4|NUM_OF_ADDR_ELEM|ADDR[0]|..|ADDR[NUM_OF_ADDR_ELEM-1]|COMMAND|0xC4|NUM_OF_DATA_ELEM|DATA[]
//where 0x93 - fixarray; 0xC4 - bin8; - type of Messagepack serializer
//#define EOF 					{0xAA,0xBB,0xCC}
#define NUM_OF_ELEM				3
#define NUM_OF_ADDR_ELEM		2
#define	INDEX_NUM_OF_DATA_ELEM	(3 + NUM_OF_ADDR_ELEM + 2)

//not involve in de/serialization, just allocate memory;
#define __SIZE_OF_DATA_ARRAY		256


#define PREAMBLE_LENGTH 5
#define PACKET_LENGTH 32
#define COUNT_OF_PACKET_LENGTH 2
#define CURRENT_PACKET_LENGTH 2
#define DATA_CRC_LENGTH 1
#define DATA_LENGTH (PACKET_LENGTH - (PREAMBLE_LENGTH + COUNT_OF_PACKET_LENGTH + CURRENT_PACKET_LENGTH + DATA_CRC_LENGTH))

const uint8_t  ASK[4];

typedef struct {
	uint8_t CountOfPacket[COUNT_OF_PACKET_LENGTH];
	uint8_t CurrentPacket[CURRENT_PACKET_LENGTH];
	uint8_t DataCRC;
	uint8_t Data[DATA_LENGTH];
} rftoolsPacket_t;

typedef struct {
	uint8_t command;
	uint8_t address[2];
	uint8_t data[__SIZE_OF_DATA_ARRAY ];
	uint32_t data_length;
} message_t;

enum messageCommand_e {
	RFTOOLS_COMMAND_STROBE = 1,
	RFTOOLS_COMMAND_REGISTER = 2,
	RFTOOLS_COMMAND_DATA = 3,
	RFTOOLS_COMMAND_RESPONSE = 4
};
//typedef struct buf_s {
//  size_t capacity;
//  size_t size;
//  size_t cursor;
//  char *data;
//} buf_t;

uint8_t serialize(message_t p_message, uint8_t **ptr_buf);
uint8_t getPacket(const uint8_t *buffer, rftoolsPacket_t *packet);
uint8_t deserialize(const uint8_t *buffer, message_t *pMessage);
#endif /* INC_RFTOOLS_H_ */
