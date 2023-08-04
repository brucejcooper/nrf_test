#ifndef DALI_H_
#define DALI_H_

#include <stdint.h>
#include <zephyr/kernel.h>

#define DALI_CMD_QUERY_ACTUAL_LEVEL 0x1A0
#define DALI_CMD_GOTO_LAST_ACTIVE_LEVEL 0x10a
#define DALI_CMD_OFF 0x100


#define DALI_GEAR_ADDR(x) ((x) << 9)
#define DALI_GROUP_ADDR(x) (1 << 15 | (x) << 9)


#define DALI_RESPONSE_NAK -1
#define DALI_RESPONSE_COLLISION -2
#define DALI_RESPONSE_TIMEOUT -3
#define DALI_RESPONSE_BUS_BUSY -4
#define DALI_RESPONSE_QUEUED -4
#define DALI_RESPONSE_PROCESSING -5

struct dali_cmd_t;

typedef void (*dali_cmd_callback_t)(struct dali_cmd_t *cmd);

typedef struct dali_cmd_t {
    void *fifo_reserved;   /* 1st word reserved for use by FIFO */
	uint32_t cmd;
    int32_t result;
	dali_cmd_callback_t callback;
	void *ctx;
} dali_cmd_t;



void dali_transmit_cmd_async(dali_cmd_t *cmd);
int32_t dali_transmit_cmd_sync(uint16_t data, struct k_sem *sem, k_timeout_t timeout);


#endif