#include "./dali.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <nrfx_gpiote.h>
#include <nrfx_timer.h>
#include <nrfx_log.h>
#include <helpers/nrfx_gppi.h>
#include <nrfx_ppi.h>


#define DALI_STACK_SIZE 1024
#define DALI_PRIORITY 0

#define NRF_ERRCHK(call) { nrfx_err_t err = (call); if (err != NRFX_SUCCESS) { printk("(%s:%d) nrfx call failed: %x\r\n", __FILE__, __LINE__, err); return 1;}} 

// TX pin is p1.12
#define TRANSMIT_PIN ((1 << 5) | 12)
// RX pin is p1.13
#define RECEIVE_PIN ((1 << 5) | 13)





#define DALI_TE_US (833/2)



// We allow 15% of one TE as the Grace period for timing.
#define DALI_TIMING_GRACE 62
#define SHORT_PULSE_MIN_US (DALI_TE_US - DALI_TIMING_GRACE)
#define SHORT_PULSE_MAX_US (DALI_TE_US + DALI_TIMING_GRACE)
#define LONG_PULSE_MIN_US (2*DALI_TE_US - DALI_TIMING_GRACE)
#define LONG_PULSE_MAX_US (2*DALI_TE_US + DALI_TIMING_GRACE)
#define DALI_BACK_FRAME_WAIT_US (22*DALI_TE_US)
#define DALI_DEAD_TIME_US (6*DALI_TE_US)

static const nrfx_timer_t io_timer = NRFX_TIMER_INSTANCE(1);
static nrf_ppi_channel_t mid_bit_toggle_ppi_channel;
static nrf_ppi_channel_t start_bit_one_ppi_channel;
static nrf_ppi_channel_t start_bit_zero_ppi_channel;
static nrf_ppi_channel_t input_toggle_ppi_channel;
static uint8_t transmit_gpiote_channel;
static uint8_t receive_gpiote_channel;

static volatile uint32_t shift_reg = 0;
static volatile size_t num_bits = 0;

K_SEM_DEFINE(line_idle, 0, 1);


typedef void (*state_callback_t)();

typedef struct {
	char *name;
	state_callback_t entry;
	state_callback_t exit;
	state_callback_t timer;
	state_callback_t short_pulse;
	state_callback_t long_pulse;
	state_callback_t invalid_pulse;
} dali_state_t;

K_FIFO_DEFINE(command_queue);

static void dali_task(void *, void *, void *);

K_THREAD_DEFINE(dali_tid, DALI_STACK_SIZE,
                dali_task, NULL, NULL, NULL,
                DALI_PRIORITY, 0, 1200);



static void bus_error_occurred();


static void tx_state_entry();
static void tx_exit();
static void tx_state_bit_transmitted();

static void post_tx_dead_state_entry();
static void post_tx_dead_state_timeout();

static void wait_for_rx_start_entry();
static void wait_for_rx_timeout();
static void wait_for_rx_transition_occurred();

static void rx_start_bit_received_entry();
static void rx_start_bit_received_short_pulse();

static void rx_mid_bit_short_pulse_received();
static void rx_mid_bit_long_pulse_received();
static void check_for_transmission_end();

static void rx_bit_end_state_short_pulse_received();

static void bus_error_state_entry();
static void bus_error_state_pulse_handler();
static void bus_error_state_timeout();

static void idle_state_entry();

static const dali_state_t transmitting_state = {
	.name = "tx",
	.entry = tx_state_entry,
	.exit = tx_exit,
	.timer = tx_state_bit_transmitted,
	.short_pulse = NULL,
	.long_pulse = NULL,
	.invalid_pulse = NULL,
};


static const dali_state_t post_tx_dead_state = {
	.name = "dead",
	.entry = post_tx_dead_state_entry,
	.exit = NULL,
	.timer = post_tx_dead_state_timeout,
	.short_pulse = NULL,
	.long_pulse = NULL,
	.invalid_pulse = NULL,
};


static const dali_state_t rx_wait_for_start_state = {
	.name = "wfs",
	.entry = wait_for_rx_start_entry,
	.exit = NULL,
	.timer = wait_for_rx_timeout,
	// Pulse duration doesn't make sense for the wait_for_start state, so we react the same for all durations.
	.short_pulse = wait_for_rx_transition_occurred,
	.long_pulse = wait_for_rx_transition_occurred,
	.invalid_pulse = wait_for_rx_transition_occurred,
};

static const dali_state_t rx_start_bit_received_state = {
	.name = "Start Bit Received",
	.entry = rx_start_bit_received_entry,
	.exit = NULL,
	.timer = bus_error_occurred,
	.short_pulse = rx_start_bit_received_short_pulse,
	.long_pulse = bus_error_occurred,
	.invalid_pulse = bus_error_occurred,
};

static const dali_state_t rx_mid_bit_state = {
	.name = "M",
	.entry = NULL,
	.exit = NULL,
	.timer = check_for_transmission_end,
	.short_pulse = rx_mid_bit_short_pulse_received,
	.long_pulse = rx_mid_bit_long_pulse_received,
	.invalid_pulse = bus_error_occurred,
};

static const dali_state_t rx_bit_end_state = {
	.name = "E",
	.entry = NULL,
	.exit = NULL,
	.timer = check_for_transmission_end,
	.short_pulse = rx_bit_end_state_short_pulse_received,
	.long_pulse = bus_error_occurred,
	.invalid_pulse = bus_error_occurred,
};

static const dali_state_t bus_error_state = {
	.name = "Bus Error",
	.entry = bus_error_state_entry,
	.exit = NULL,
	.timer = bus_error_state_timeout,
	.short_pulse = bus_error_state_pulse_handler,
	.long_pulse = bus_error_state_pulse_handler,
	.invalid_pulse = bus_error_state_pulse_handler,
};

static const dali_state_t idle_state = {
	.name = "I",
	.entry = idle_state_entry,
	.exit = NULL,
	.timer = NULL,
	.short_pulse = NULL,
	.long_pulse = NULL,
	.invalid_pulse = NULL,
};

dali_state_t *current_state = (dali_state_t *) &idle_state;



/**
 * @brief Sets up the appropriate PPI channels to send the next bit correctly.
 */
static inline void configure_tx_next_bit() {
	if (shift_reg & BIT(31)) {
		nrfx_ppi_channel_enable(start_bit_one_ppi_channel);
		nrfx_ppi_channel_disable(start_bit_zero_ppi_channel);
	} else {
		nrfx_ppi_channel_disable(start_bit_one_ppi_channel);
		nrfx_ppi_channel_enable(start_bit_zero_ppi_channel);
	}
	shift_reg <<= 1;
}



// =================== Common state handlers ===========================
static void set_state(const dali_state_t *new_state) {
	if (current_state->exit) {
		current_state->exit();
	}
	// printk("%s->%s\r\n", current_state->name, new_state->name);
	current_state = (dali_state_t *) new_state;
	if (current_state->entry) {
		current_state->entry();
	}
}

/**
 * @brief Shared by many states, this indicates that some timing error occurred.  
 * We go into a stat that waits for the bus to be idle for a period of time, then
 * returns to active duty.
 */
static void bus_error_occurred() {
	printk("Bus error in state %s\r\n", current_state->name);
	set_state(&bus_error_state);
}

static void check_for_transmission_end() {
	// This can be called from either the half bit or the end of a bit.

	// if pin is high, there is a problem, as we must always return to idle.
	if (nrfx_gpiote_in_is_set(RECEIVE_PIN)) {
		set_state(&bus_error_state);
	} else {
		// Strip off the extra high 1 bit that represented the start bit.
		shift_reg &= (1 << num_bits)-1;
		// We're done with receiving.
		set_state(&idle_state);
	}
}




// =================== Transmit State. ===========================
static void tx_state_entry() {
	// The timer should already be off, but lets make sure.
	nrfx_timer_disable(&io_timer);
	// Enable GPIOTE capture on input toggle. 
	nrfx_gpiote_in_event_disable(RECEIVE_PIN);

	// Set timer to have fixed 2TE period.
	nrf_timer_cc_set(io_timer.p_reg, NRF_TIMER_CC_CHANNEL0, nrfx_timer_us_to_ticks(&io_timer, DALI_TE_US*2));
	// Make it so that it will toggle at one TE
	nrfx_ppi_channel_enable(mid_bit_toggle_ppi_channel);
	// Set up the next bit
	configure_tx_next_bit();
	// We start with driving the start bit (a logical 1)
	nrfx_timer_clear(&io_timer);
	nrfx_gpiote_set_task_trigger(TRANSMIT_PIN);	
	nrfx_timer_enable(&io_timer);

}


static void tx_state_bit_transmitted() {
	if (--num_bits > 0) {
		configure_tx_next_bit();
	} else {
		// We're done transmitting. Because we stay one bit ahead, an extra 0 will have
		// been set up, which will drive the line back low, which is what we want.

		// If we were to start reading straight away, we might read the final bit toggle reflected back
		// after propagation delay through the read line.  Instead, we introduce a dead time period where
		// we ignore the bus for a few TE.
		set_state(&post_tx_dead_state);
	}
}

static void tx_exit() {
	// Disable the PPI channels that make TX work. 
	nrfx_ppi_channel_disable(mid_bit_toggle_ppi_channel);
	nrfx_ppi_channel_disable(start_bit_one_ppi_channel);
	nrfx_ppi_channel_disable(start_bit_zero_ppi_channel);

	// Timer is left running as this state exits.
	// nrfx_timer_disable(&io_timer);
}


// =================== Post TX dead state ===========================

static void post_tx_dead_state_entry() {
	// Wait for 6 TEs (one less than the required 7) before we start receiving.
	nrf_timer_cc_set(io_timer.p_reg, NRF_TIMER_CC_CHANNEL0, nrfx_timer_us_to_ticks(&io_timer, DALI_DEAD_TIME_US));
	// nrfx_timer_clear(&io_timer);
	// Re-enable the timer. 
	// nrfx_timer_enable(&io_timer);
}

static void post_tx_dead_state_timeout() {
	set_state(&rx_wait_for_start_state);
}


// =================== RX Wait for Start State ===========================

static void wait_for_rx_start_entry() {
	shift_reg = 0;
	num_bits = 0;

	// Transmission must start between 7 and 22 TEs after TX finished.  We've already waited 6, so we set timeout to 16
	nrf_timer_cc_set(io_timer.p_reg, NRF_TIMER_CC_CHANNEL0, nrfx_timer_us_to_ticks(&io_timer, DALI_BACK_FRAME_WAIT_US-DALI_DEAD_TIME_US));
	// TODO Ensure bus is still low.
	// Listen for Bit Transitions
	nrfx_gpiote_in_event_enable(RECEIVE_PIN, true);
}

static void wait_for_rx_timeout() {
	// No start bit detected during detection period.
	set_state(&idle_state);
}

static void wait_for_rx_transition_occurred() {
	// A start bit has happened
	set_state(&rx_start_bit_received_state);
}


// =================== RX Start bit received state ===========================

/**
 * @brief We've started reading, meaning a transition has occurred.
 */
static void rx_start_bit_received_entry() {
	// Reset timeout to be the maximum long pulse width
	nrf_timer_cc_set(io_timer.p_reg, NRF_TIMER_CC_CHANNEL0, nrfx_timer_us_to_ticks(&io_timer, LONG_PULSE_MAX_US));
}

/**
 * @brief We've received the first half of the start bit. Set ourselves up for normal reads from here on in. 
 */
static void rx_start_bit_received_short_pulse() {
	shift_reg = 1; // We store the last bit in the LSB of the shift reg.  We start with 1 to represent the start it. This will be masked out later.
	set_state(&rx_mid_bit_state);
}


// =================== RX Mid bit state ===========================

static void rx_mid_bit_short_pulse_received() {
	set_state(&rx_bit_end_state);
}

static void rx_mid_bit_long_pulse_received() {
	// Its a bit flip.
	shift_reg = shift_reg << 1 | (shift_reg & 0x01 ? 0 : 1);
	num_bits++;
	// Stay in mid-bit state.
}



// =================== RX bit end state ===========================

static void rx_bit_end_state_short_pulse_received() {
	// The only way we get here is if the previous state was a half bit,
	// and it received a short pulse.  A second short pulse indicates another
	// bit of the same polarity.
	shift_reg = shift_reg << 1 | (shift_reg & 0x01);
	num_bits++;
	set_state(&rx_mid_bit_state);

}

// =================== Bus Error State ===========================

static void bus_error_state_entry() {
	// We want the bus to be low for a period of time before we exit this state.
	// TODO make this do something other than just wait for a bit.
	nrf_timer_cc_set(io_timer.p_reg, NRF_TIMER_CC_CHANNEL0, nrfx_timer_ms_to_ticks(&io_timer, 5));

}

static void bus_error_state_pulse_handler() {
}

static void bus_error_state_timeout() {
	set_state(&idle_state);
}

// =================== Idle State  ===========================

static void idle_state_entry() {
	nrfx_timer_disable(&io_timer);
	// nrfx_ppi_channel_disable(input_toggle_ppi_channel);
	nrfx_gpiote_in_event_disable(RECEIVE_PIN);
	k_sem_give(&line_idle);
}


// =================== End of state handlers.  ===========================



/**
 * @brief Called for compare interrupts coming from the I/O Timer. 
 * This will handle two types of events
 *  - CC0 - This is used by both transmit and receive
 *  - CC2 - This is used by Receive each time there is a polarity toggle. It records the pulse width in us 
 *          and uses this to determine what has been read. 
 * 
 * @param event_type 
 * @param p_context 
 */
void timer1_event_handler(nrf_timer_event_t event_type, void * p_context)
{
	switch(event_type) {
		case NRF_TIMER_EVENT_COMPARE0:
			if (current_state && current_state->timer) {
				current_state->timer();
			}
			break;
		default:
			break;
	}
}


/**
 * @brief Called each time the input toggles (when in the various RX modes)
 * 
 * @param pin 
 * @param action 
 */
void gpiote_receive_event_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
	if (current_state) {
		uint32_t ticks = nrfx_timer_capture_get(&io_timer, NRF_TIMER_CC_CHANNEL2);
		// printk("P%d\r\n", ticks);

		if (ticks >= SHORT_PULSE_MIN_US && ticks <= SHORT_PULSE_MAX_US) {
			// Short Pulse
			if (current_state->short_pulse) {
				current_state->short_pulse();
			}
		} else if (ticks >= LONG_PULSE_MIN_US) {
			// Long pulse - Anything greater than LONG_PULSE_MAX_US will cause a timeout.
			if (current_state->long_pulse) {
				current_state->long_pulse();
			}
		} else {
			if (current_state->invalid_pulse) {
				current_state->invalid_pulse();
			}
		}
	}
}



void dali_transmit_cmd_async(dali_cmd_t *cmd) {
	cmd->result = DALI_RESPONSE_QUEUED;
	k_fifo_put(&command_queue, cmd);
}


struct sync_callback_ctx {
	int response;
	struct k_sem *sem;
};


static void syncCallback(dali_cmd_t *cmd) {
	k_sem_give((struct k_sem *) cmd->ctx);
}

/**
 * @brief Uses the asynchronous call, but incorporates a semaphore to make it more synchronous.
 * 
 * @param data 
 * @param sem 
 * @param timeout 
 * @return int32_t 
 */
int32_t dali_transmit_cmd_sync(uint16_t data, struct k_sem *sem, k_timeout_t timeout) {
	dali_cmd_t cmd = {
		.cmd = data,
		.ctx = sem,
		.callback = syncCallback,
	};
	dali_transmit_cmd_async(&cmd);
	// Wait for task to complete (callback will give to semaphore)
	int res = k_sem_take(sem, timeout);
	if (res) {
		return -res;
	}
	return cmd.result;
}


static int dali_init() {
	nrfx_gpiote_out_config_t toggleConfig = {
		.action = NRF_GPIOTE_POLARITY_TOGGLE,
		.init_state = NRF_GPIOTE_INITIAL_VALUE_LOW,
		.task_pin = true,
	};

  	if (!nrfx_gpiote_is_init()) {
		NRF_ERRCHK(nrfx_gpiote_init(1));
	}

	// Create a GPIO Task that will manipulate the output of the transmit pin.
	NRF_ERRCHK(nrfx_gpiote_channel_alloc(&transmit_gpiote_channel));
	NRF_ERRCHK(nrfx_gpiote_out_prealloc_init(TRANSMIT_PIN, &toggleConfig, transmit_gpiote_channel));
	nrfx_gpiote_out_task_enable(TRANSMIT_PIN);


	// Set up Timer
	nrfx_timer_config_t timer_config = NRFX_TIMER_DEFAULT_CONFIG;
	timer_config.bit_width = NRF_TIMER_BIT_WIDTH_32;
	timer_config.frequency = NRF_TIMER_FREQ_1MHz;
	NRF_ERRCHK(nrfx_timer_init(&io_timer, &timer_config, timer1_event_handler));

	IRQ_DIRECT_CONNECT(TIMER1_IRQn, 0, nrfx_timer_1_irq_handler, 0);
	irq_enable(TIMER1_IRQn);
	nrfx_timer_extended_compare(&io_timer, 
                                NRF_TIMER_CC_CHANNEL0, 
                                nrfx_timer_us_to_ticks(&io_timer, DALI_TE_US*2), 
                                NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);


	// For transmit, set up a compare channel to trigger the toggle via PPI
	nrfx_timer_compare(&io_timer, 
						NRF_TIMER_CC_CHANNEL1, 
						nrfx_timer_us_to_ticks(&io_timer, DALI_TE_US), 
						false);

	// Set up PPI to wire together timer Compare1 (Mid Bit) to the OUT (toggle) task for the output pin
    NRF_ERRCHK(nrfx_ppi_channel_alloc(&mid_bit_toggle_ppi_channel));
    NRF_ERRCHK(nrfx_ppi_channel_assign(mid_bit_toggle_ppi_channel, 
			nrfx_timer_event_address_get(&io_timer, NRF_TIMER_EVENT_COMPARE1), 
			nrfx_gpiote_out_task_addr_get(TRANSMIT_PIN)
	));

	// Also set up channels for set (1) or clear (0) on CC0.  
	// These will be enabled or disabled based on the next bit to be transmitted.
    NRF_ERRCHK(nrfx_ppi_channel_alloc(&start_bit_one_ppi_channel));
    NRF_ERRCHK(nrfx_ppi_channel_assign(start_bit_one_ppi_channel, 
			nrfx_timer_event_address_get(&io_timer, NRF_TIMER_EVENT_COMPARE0), 
			nrfx_gpiote_set_task_addr_get(TRANSMIT_PIN)
	));
    NRF_ERRCHK(nrfx_ppi_channel_alloc(&start_bit_zero_ppi_channel));
    NRF_ERRCHK(nrfx_ppi_channel_assign(start_bit_zero_ppi_channel, 
			nrfx_timer_event_address_get(&io_timer, NRF_TIMER_EVENT_COMPARE0), 
			nrfx_gpiote_clr_task_addr_get(TRANSMIT_PIN)
	));

	// For Receive
	// Create a timer capture event upon any change of the input
	nrfx_gpiote_in_config_t readToggleConfig = {
		.sense = NRF_GPIOTE_POLARITY_TOGGLE,
		.pull =  NRF_GPIO_PIN_PULLUP, // CLICK board already has pullup, and will be driven low.
		.is_watcher = false,
		.hi_accuracy = true,
		.skip_gpio_setup = false,
	};
	NRF_ERRCHK(nrfx_gpiote_channel_alloc(&receive_gpiote_channel));
	NRF_ERRCHK(nrfx_gpiote_in_prealloc_init(RECEIVE_PIN, &readToggleConfig, receive_gpiote_channel, gpiote_receive_event_handler));



	// Create a PPI channel that binds the toggle event to a capture event.
	NRF_ERRCHK(nrfx_ppi_channel_alloc(&input_toggle_ppi_channel));
    NRF_ERRCHK(nrfx_ppi_channel_assign(input_toggle_ppi_channel, 
		nrfx_gpiote_in_event_addr_get(RECEIVE_PIN),
		nrfx_timer_capture_task_address_get(&io_timer, 2) // Capture into CC2
	));
	// Also clear the counter upon receiving the toggle event.
	NRF_ERRCHK(nrfx_ppi_channel_fork_assign(input_toggle_ppi_channel, 
		nrf_timer_task_address_get(io_timer.p_reg, NRF_TIMER_TASK_CLEAR)
	));
	nrfx_ppi_channel_enable(input_toggle_ppi_channel);

	// Do not enable the capture PPI channel yet, as we are not receiving.
	return 0;
}




static void dali_task(void *_a1, void *_a2, void *_a3) {
    dali_cmd_t *cmd;

    if (dali_init()) {
        printk("Error initialising DALI\r\n");
        return;
    }

    while (true) {
        if ((cmd = (dali_cmd_t *) k_fifo_get(&command_queue, K_FOREVER))) {
			if (current_state != &idle_state) {
				printk("Bus is not idle\r\n");
				cmd->result = DALI_RESPONSE_BUS_BUSY;
			} else {
				cmd->result = DALI_RESPONSE_PROCESSING;
				shift_reg = cmd->cmd << 16;
				num_bits = 17; // extra one for the start bit.
				set_state(&transmitting_state);

				// Wait for the timers to finish transmitting and receiving.
				if (k_sem_take(&line_idle, K_MSEC(50))) {
					cmd->result = DALI_RESPONSE_TIMEOUT;
				} else {
					if (num_bits == 0) {
						// TODO techically a bus error could have also caused 0 bits to be read.
						cmd->result = DALI_RESPONSE_NAK;
					} else  if (num_bits == 8) {
						// Copy the received result into the output.
						cmd->result = (int32_t) shift_reg;
					} else {
						printk("Error reading respone\r\n");
						cmd->result = DALI_RESPONSE_COLLISION;
					}
				}
			}
			// Mark message as deleted, and send info back to the caller.
			if (cmd->callback) {
				cmd->callback(cmd);
			}
		} else {
			printk("Error getting msg\r\n");
		}
    }
}


