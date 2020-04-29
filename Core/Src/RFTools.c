/*
 * RFTools.c
 *
 *  Created on: Apr 10, 2020
 *      Author: maxim
 */

#include "RFTools.h"
#include "types.h"
#include "cmp.h"
#include "CRC.h"

uint16_t curpos = 0;

const uint8_t PREAMBLE[PREAMBLE_LENGTH] = { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE };
const uint8_t ASK[] = { 0xBB, 0xBB, 0xDD, 0xAB };

static bool read_bytes(void *data, size_t count, uint8_t *buf_to_read) {
	for (uint16_t i = 0; i < count; i++) {
		((uint8_t*) data)[i] = ((uint8_t*) buf_to_read)[i + curpos];
	}
	curpos += count;
	return true;
	/*fread(data, sizeof(uint8_t), count, buf_to_read)
	 == (count * sizeof(uint8_t));*/
}

static bool reader(cmp_ctx_t *ctx, void *data, size_t limit) {
	return read_bytes(data, limit, (uint8_t*) ctx->buf);
}

//static bool file_skipper(cmp_ctx_t *ctx, size_t count) {
//	return fseek((uint8_t*) ctx->buf, count, SEEK_CUR);
//}

static size_t writer(cmp_ctx_t *ctx, const void *data, size_t count) {
	for (uint16_t i = 0; i < count; i++) {
		((uint8_t*) ctx->buf)[i + curpos] = ((uint8_t*) data)[i];
	}
	curpos += count;
	return count;
}

//void error_and_exit(const char *msg) {
//   fprintf(stderr, "%s\n\n", msg);
//	exit(EXIT_FAILURE);
//}

cmp_ctx_t cmp;
uint8_t serialize(message_t p_message, uint8_t **ptr_buffer) {
	curpos = 0;
//	for (; curpos < sizeof(START_SEQ); curpos++) {
//		*ptr_buffer[curpos] = START_SEQ[curpos];
//	}

	cmp_init(&cmp, (*ptr_buffer), NULL, NULL, writer);

	cmp_write_array(&cmp, NUM_OF_ELEM);
	cmp_write_bin8(&cmp, p_message.address, NUM_OF_ADDR_ELEM);
	cmp_write_integer(&cmp, p_message.command);
	cmp_write_bin8(&cmp, p_message.data, p_message.data_length);

//	for (uint8_t i = 0; i < sizeof(END_SEQ); i++) {
//		*ptr_buffer[curpos + i] = END_SEQ[i];
//		curpos++;
//	}

	return (curpos);
}

uint8_t getPacket(const uint8_t *buffer, rftoolsPacket_t *packet) {

	uint8_t buffcur = 0;
	for (; buffcur < sizeof(PREAMBLE); buffcur++) {
		if (buffer[buffcur] != PREAMBLE[buffcur])
			return 0;
	}
	for (int i = 0; i < COUNT_OF_PACKET_LENGTH; buffcur++) {
		packet->CountOfPacket[i] = buffer[buffcur];
		i++;
	}
	for (int i = 0; i < CURRENT_PACKET_LENGTH; buffcur++) {
		packet->CurrentPacket[i] = buffer[buffcur];
		i++;
	}

	packet->DataCRC = buffer[buffcur];
	buffcur++;

	for (int i = 0; i < (DATA_LENGTH); buffcur++) {
		packet->Data[i] = buffer[buffcur];
		i++;
	}
	uint8_t crc = getCRC(packet->Data, DATA_LENGTH);
	if (packet->DataCRC == crc)
		return 1;
	return 0;

}

//uint8_t messageToPacket(message_t message, rftoolsPacket_t *packet){
//
//	serialize(message);
//}

uint8_t setPacket(uint8_t *buffer, const rftoolsPacket_t packet) {
	uint8_t buffcur = 0;
	for (; buffcur < sizeof(PREAMBLE); buffcur++) {
		buffer[buffcur] = PREAMBLE[buffcur];
	}

	for (int i = 0; i < COUNT_OF_PACKET_LENGTH; i++) {
		buffer[buffcur] = packet.CountOfPacket[i];
		buffcur++;
	}
	for (int i = 0; i < CURRENT_PACKET_LENGTH; i++) {
		buffer[buffcur] = packet.CurrentPacket[i];
		buffcur++;
	}

	buffer[buffcur] = packet.DataCRC;
	buffcur++;

	for (int i = 0; i < (DATA_LENGTH); buffcur++) {
		buffer[buffcur] = packet.Data[i];
		i++;
	}
//	buffer[] = getCRC(packet.Data, DATA_LENGTH);

	return 1;
}

uint8_t deserialize(const uint8_t *buffer, message_t *pMessage) {
	curpos = 0;
	message_t *ptrMessage = pMessage;

	uint8_t num_of_elements = 0;
	cmp_init(&cmp, (uint8_t*) buffer, reader, NULL, NULL);

	cmp_read_array(&cmp, (uint32_t*) &num_of_elements);
	if (num_of_elements != NUM_OF_ELEM)
		return 0;

	uint8_t num_of_addr_elem = NUM_OF_ADDR_ELEM;
	ptrMessage->data_length = (buffer[INDEX_NUM_OF_DATA_ELEM]);

	cmp_read_bin(&cmp, (ptrMessage->address), ((uint32_t*) &num_of_addr_elem));
	ptrMessage->command = buffer[curpos];
	reader(&cmp, &(ptrMessage->command), 1);
//	cmp_read_int(&cmp, &message.command);
	cmp_read_bin(&cmp, (ptrMessage->data),
			(uint32_t*) &(ptrMessage->data_length));

	*pMessage = *ptrMessage;
	return (curpos);
}

