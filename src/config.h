#pragma once
#include <stdint.h>

extern volatile uint32_t g_sample_interval_ms; // GNSS, ping data sampling
extern volatile uint32_t g_log_interval_ms;  // sd, lora batching