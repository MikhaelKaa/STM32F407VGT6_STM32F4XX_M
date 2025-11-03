// Green led on board stm32f407
// Michael Kaa
// 03.11.2025

#ifndef BOARD_GREEN_LED
#define BOARD_GREEN_LED

typedef enum {
  led_off = 0,
  led_on
} led_state_t;

// Init board led pin (PA1)
int green_led_init(void);

// Set led state
int green_led_set(led_state_t state);

#endif // BOARD_GREEN_LED
