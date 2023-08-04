
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include "./buttons.h"

/*
 * Get button configuration from the devicetree sw0 alias. This is mandatory.
 */
#define SW1_NODE	DT_ALIAS(sw1)
#define SW2_NODE	DT_ALIAS(sw2)
#define SW3_NODE	DT_ALIAS(sw3)
#define SW4_NODE	DT_ALIAS(sw4)
#define SW5_NODE	DT_ALIAS(sw5)

#define gpio0_NODE	DT_ALIAS(gpio0)
#define gpio1_NODE	DT_ALIAS(gpio1)


#if !DT_NODE_HAS_STATUS(SW1_NODE, okay)
#error "Unsupported board: sw1 devicetree alias is not defined"
#endif

#if !DT_NODE_HAS_STATUS(SW2_NODE, okay)
#error "Unsupported board: sw2 devicetree alias is not defined"
#endif

#if !DT_NODE_HAS_STATUS(SW3_NODE, okay)
#error "Unsupported board: sw3 devicetree alias is not defined"
#endif



#if !DT_NODE_HAS_STATUS(SW4_NODE, okay)
#error "Unsupported board: sw4 devicetree alias is not defined"
#endif

#if !DT_NODE_HAS_STATUS(SW5_NODE, okay)
#error "Unsupported board: sw5 devicetree alias is not defined"
#endif


#define NUM_BUTTONS 5
#define DEBOUNCE_INTERVAL 25
#define INITIAL_REPEAT_INTERVAL 750
#define REPEAT_INTERVAL 250


button_event_handler_t event_handler = NULL;


typedef struct {
	unsigned int index;
	uint32_t level;
	uint64_t last_release_ts;
    uint32_t repeat_count;
	struct gpio_dt_spec button;
	struct gpio_callback callback;
	struct k_work_delayable debounce_task;
	struct k_work_delayable repeat_task;
} t_button_state;

static t_button_state buttons[NUM_BUTTONS] = {
	{
		.index = 0,
		.button = GPIO_DT_SPEC_GET_OR(SW1_NODE, gpios, {0}),
		.last_release_ts = 0,
	},
	{
		.index = 1,
		.button = GPIO_DT_SPEC_GET_OR(SW2_NODE, gpios, {0}),
		.last_release_ts = 0,
	},
	{
		.index = 2,
		.button = GPIO_DT_SPEC_GET_OR(SW3_NODE, gpios, {0}),
		.last_release_ts = 0,
	},
	{
		.index = 3,
		.button = GPIO_DT_SPEC_GET_OR(SW4_NODE, gpios, {0}),
		.last_release_ts = 0,
	},
	{
		.index = 4,
		.button = GPIO_DT_SPEC_GET_OR(SW5_NODE, gpios, {0}),
		.last_release_ts = 0,
	},
};



void button_change(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	t_button_state *btn = buttons;
	for (int i = 0; i < NUM_BUTTONS; i++, btn++) {
		if (btn->button.port == dev && pins & BIT(btn->button.pin)) {
			k_work_reschedule(&(btn->debounce_task), K_MSEC(DEBOUNCE_INTERVAL));	
		}
	}
}



static void debounce(struct k_work *task) {
	t_button_state *button = CONTAINER_OF(task, t_button_state, debounce_task);

	int val = gpio_pin_get_dt(&(button->button));
	if (val != button->level) {
		button->level = val;
		if (val) {
			k_work_reschedule(&(button->repeat_task), K_MSEC(INITIAL_REPEAT_INTERVAL));	
			uint64_t now = k_uptime_get();
			if ((now - button->last_release_ts < 500) && button->repeat_count > 0) {
                if (event_handler) {
                    event_handler(button->index, BUTTON_EVT_REPRESS, button->repeat_count);
                }
			} else {
                button->repeat_count = 0;
                if (event_handler) {
                    event_handler(button->index, BUTTON_EVT_PRESS, button->repeat_count);
                }
			}
		} else {
			button->last_release_ts = k_uptime_get();
            // button->repeat_count = 0;
			k_work_cancel_delayable(&(button->repeat_task));	
            if (event_handler) {
                event_handler(button->index, BUTTON_EVT_RELEASE, button->repeat_count);
            }
		}

	}

}

static void repeat(struct k_work *task) {
	t_button_state *button = CONTAINER_OF(task, t_button_state, repeat_task);
	if (button->level) {
		k_work_reschedule(&(button->repeat_task), K_MSEC(REPEAT_INTERVAL));	
        button->repeat_count++;
        if (event_handler) {
            event_handler(button->index, BUTTON_EVT_REPEAT, button->repeat_count);
        }
	}
}

static int setup_button_gpio(struct gpio_dt_spec *button) {
	if (!gpio_is_ready_dt(button)) {
		printk("Error: button device %s is not ready\r\n", button->port->name);
		return 1;
	}

	int ret = gpio_pin_configure_dt(button, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\r\n", ret, button->port->name, button->pin);
		return 1;
	}

	ret = gpio_pin_interrupt_configure_dt(button, GPIO_INT_EDGE_BOTH);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\r\n",
			ret, button->port->name, button->pin);
		return 1;
	}
	printk("Set up button at %s pin %d\r\n", button->port->name, button->pin);
	return 0;
}


void buttons_init(button_event_handler_t handler) {
	t_button_state *button;
	int i; 

    event_handler = handler;

	for (i = 0, button = buttons; i < NUM_BUTTONS; i++, button++) {
		k_work_init_delayable(&button->debounce_task, debounce);
		k_work_init_delayable(&button->repeat_task, repeat);

		if (setup_button_gpio(&button->button)) {
			printk("Button setup fail!\r\n");
		}
		button->level = gpio_pin_get_dt(&button->button);
		button->last_release_ts = k_uptime_get_32();
        // Set up GPIO to call button_change as its callback. 
		gpio_init_callback(&(button->callback), button_change, BIT(button->button.pin));
		gpio_add_callback(button->button.port, &(button->callback));
	}
}
