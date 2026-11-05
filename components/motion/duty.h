/* Duty-cycle ledger: rolling window of motor-on time, kept in RTC memory. */
#pragma once
#include <stdint.h>

void duty_init(void);
/* Add motor-on time. Call from the motion task only. */
void duty_add_ms(uint32_t ms);
uint32_t duty_used_ms(void);
uint32_t duty_remaining_ms(void);
/* Seconds until at least `want_ms` of budget is available again (0 if already). */
uint32_t duty_wait_s_for(uint32_t want_ms);
/* Debug only: forget all recorded on-time. */
void duty_clear(void);
