#ifndef BUTTONS_H_
#define BUTTONS_H_

typedef enum {
    BUTTON_EVT_PRESS,
    BUTTON_EVT_RELEASE,
    BUTTON_EVT_REPEAT,
    BUTTON_EVT_REPRESS,
} button_evt_t;

typedef void (*button_event_handler_t)(int btn, button_evt_t evt, uint32_t repeat_count);

void buttons_init(button_event_handler_t handler);

#endif