/*
 * RFTools.c
 *
 *  Created on: Apr 10, 2020
 *      Author: maxim
 */

#include "RFTools.h"
#include "types.h"
#include "cmp.h"

uint16_t curpos = 0;

const uint8_t START_SEQ[3] = { 0xAA, 0xFF, 0xBB };
const uint8_t END_SEQ[3] = { 0xBB, 0xCC, 0xDD };

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

void error_and_exit(const char *msg) {
//    fprintf(stderr, "%s\n\n", msg);
//	exit(EXIT_FAILURE);
}

cmp_ctx_t cmp;
uint8_t serialize(message_t p_message, uint8_t **ptr_buffer) {
	curpos = 0;
	for (; curpos < sizeof(START_SEQ); curpos++) {
		*ptr_buffer[curpos] = START_SEQ[curpos];
	}

	cmp_init(&cmp, (*ptr_buffer), NULL, NULL, writer);

	cmp_write_array(&cmp, NUM_OF_ELEM);
	cmp_write_bin8(&cmp, p_message.address, NUM_OF_ADDR_ELEM);
	cmp_write_integer(&cmp, p_message.command);
	cmp_write_bin8(&cmp, p_message.data, p_message.data_length);

	for (uint8_t i = 0; i < sizeof(END_SEQ); i++) {
		*ptr_buffer[curpos + i] = END_SEQ[i];
		curpos++;
	}

	return (curpos);
}

uint8_t deserialize(const uint8_t *buffer, message_t *pMessage) {
	curpos = 0;
	uint8_t num_of_elements = 0;
	cmp_init(&cmp, (uint8_t *)buffer, reader, NULL, NULL);

	cmp_read_array(&cmp, (uint32_t*) &num_of_elements);
	if (num_of_elements != NUM_OF_ELEM)
		return 0;

	uint8_t num_of_addr_elem = NUM_OF_ADDR_ELEM;
	pMessage->data_length = (buffer[INDEX_NUM_OF_DATA_ELEM]);

	cmp_read_bin(&cmp, (pMessage->address), ((uint32_t*) &num_of_addr_elem));
	pMessage->command = buffer[curpos];
	reader(&cmp, &(pMessage->command), 1);
//	cmp_read_int(&cmp, &message.command);
	cmp_read_bin(&cmp, (pMessage->data), (uint32_t*) &(pMessage->data_length));
	return (curpos);
}

