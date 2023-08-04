/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <helpers/nrfx_gppi.h>
#include <zephyr/init.h>
#include <stdlib.h>
#include "./dali.h"
#include "./buttons.h"

#define SLEEP_TIME_MS   500
#define LED0_NODE DT_ALIAS(led0)



typedef enum {
	BUTTON_ACTION_TOGGLE,
	BUTTON_ACTION_DIM,
	BUTTON_ACTION_BRIGHTEN,
} button_action_t;

typedef enum {
	DEVICE_TYPE_NOT_PRESENT,
	DEVICE_TYPE_DIMMABLE_LAMP,
	DEVICE_TYPE_ONOFF,
} device_type_t;


typedef struct {
	device_type_t type;
	uint32_t address;
	uint8_t level;
} device_t;


typedef struct {
	uint32_t index;
	button_action_t action;
	device_t target;
} button_state_t;

static button_state_t buttons[5] = {
	{ .index = 0, .target = { .type = DEVICE_TYPE_DIMMABLE_LAMP, .address = DALI_GEAR_ADDR(0)},  .action = BUTTON_ACTION_TOGGLE },
	{ .index = 1, .target = { .type = DEVICE_TYPE_ONOFF, .address = DALI_GEAR_ADDR(1)},  .action = BUTTON_ACTION_TOGGLE },
	{ .index = 2, .target = { .type = DEVICE_TYPE_NOT_PRESENT }, .action = BUTTON_ACTION_TOGGLE },
	{ .index = 3, .target = { .type = DEVICE_TYPE_NOT_PRESENT }, .action = BUTTON_ACTION_TOGGLE },
	{ .index = 4, .target = { .type = DEVICE_TYPE_NOT_PRESENT }, .action = BUTTON_ACTION_TOGGLE },
};

/**
 * @brief Semaphore used to make asynchronous calls more synchronous.
 */
K_SEM_DEFINE(scan_sem, 0, 1);

static void update_level(dali_cmd_t *cmd) {
	button_state_t *button = (button_state_t *) cmd->ctx;
	if (cmd->result >= 0) {
		button->target.level = cmd->result;
		printk("Button %d level now %d from %d\r\n", button->index, button->target.level, cmd->result);
	} else {
		printk("Error when fetching level for button %d: %d\r\n", button->index, cmd->result);
	}
	k_free(cmd);
}


static void fetch_device_level(button_state_t *button) {
	dali_cmd_t *cmd = (dali_cmd_t *) k_malloc(sizeof(dali_cmd_t));
	if (cmd) {
		cmd->cmd = button->target.address | DALI_CMD_QUERY_ACTUAL_LEVEL; 
		cmd->ctx = button;
		cmd->callback = update_level;
		dali_transmit_cmd_async(cmd);
	} else {
		printk("Malloc failed");
	}
}


static void free_cmd(dali_cmd_t *cmd) {
	button_state_t *button = (button_state_t *) cmd->ctx;
	k_free(cmd);
	fetch_device_level(button);
}

static void send_cmd(button_state_t *button, uint32_t operation) {
	dali_cmd_t *cmd = (dali_cmd_t *) k_malloc(sizeof(dali_cmd_t));
	if (cmd) {
		cmd->cmd = button->target.address | operation; 
		cmd->ctx = button;
		cmd->callback = free_cmd;
		dali_transmit_cmd_async(cmd);
	} else {
		printk("Malloc failed");
	}
}



static void button_handler(int btn, button_evt_t evt, uint32_t repeat_count) {
	button_state_t *button = buttons + btn;
	switch (evt) {
		case BUTTON_EVT_PRESS:
			button->action = BUTTON_ACTION_TOGGLE;
			if (button->target.type != DEVICE_TYPE_NOT_PRESENT) {
				fetch_device_level(button);
				printk("Request status of %d\r\n", btn);
			}
			break;

		case BUTTON_EVT_RELEASE:
			if (button->action == BUTTON_ACTION_TOGGLE) {
				if (button->target.level == 0) {
					send_cmd(button, DALI_CMD_GOTO_LAST_ACTIVE_LEVEL);
				} else {
					send_cmd(button, DALI_CMD_OFF);
				}
				// k_msleep(20);
				// fetch_device_level(button);
				printk("toggle %d\r\n", btn);
			}
			break;

		case BUTTON_EVT_REPEAT:
			switch (button->action) {
				case BUTTON_ACTION_TOGGLE:
					// First dim/brighten action depends on current level. 
					button->action = button->target.level == 0 ? BUTTON_ACTION_BRIGHTEN : BUTTON_ACTION_DIM;
					break;
				case BUTTON_ACTION_DIM:
					printk("Dim %d\r\n", btn);
					break;
				case BUTTON_ACTION_BRIGHTEN:
					printk("Brighten %d\r\n", btn);
					break;
			}
			break;


		case BUTTON_EVT_REPRESS:
			button->action = button->action == BUTTON_ACTION_DIM ? BUTTON_ACTION_BRIGHTEN :  BUTTON_ACTION_DIM;
			if (button->action == BUTTON_ACTION_DIM) {
				printk("Dim %d\r\n", btn);
			} else {
				printk("Brighten %d\r\n", btn);
			}
			break;
	}
}
	

void scanDevices() {
	for (int addr = 0; addr < 64; addr++) {
		int32_t response = dali_transmit_cmd_sync(DALI_GEAR_ADDR(addr) | DALI_CMD_QUERY_ACTUAL_LEVEL, &scan_sem, K_FOREVER);
		// Wait for task to complete (callback will give to semaphore)
		k_sem_take(&scan_sem, K_FOREVER);
		// anything NAK (-1) and above are valid responses. All other negative responses are errors.
		if (response >= DALI_RESPONSE_NAK) {
			// printk("Gear %d level = %d\r\n", addr, response);
		} else {
			printk("Error fetching data: %d\r\n", response);
		}
	}
}


int main(void)
{
	buttons_init(button_handler);
	printk("Hello World\r\n");


	while (1) {
		// scanDevices();
		k_msleep(5000);
	}
	return 0;
}
