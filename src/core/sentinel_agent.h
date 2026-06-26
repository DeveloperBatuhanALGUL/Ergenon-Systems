#ifndef ERGENON_SENTINEL_AGENT_H
#define ERGENON_SENTINEL_AGENT_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    SENTINEL_STATE_NOMINAL = 0,
    SENTINEL_STATE_WARNING = 1,
    SENTINEL_STATE_CRITICAL = 2,
    SENTINEL_STATE_UNRESPONSIVE = 3
} sentinel_state_t;

typedef struct {
    uint32_t subsystem_id;
    float min_bound;
    float max_bound;
    uint32_t max_silence_ticks;
} sentinel_bounds_t;

typedef struct {
    uint32_t subsystem_id;
    sentinel_state_t state;
    float last_value;
    uint32_t last_update_tick;
    uint32_t violation_count;
} sentinel_agent_t;

void sentinel_agent_init(sentinel_agent_t *agent, uint32_t subsystem_id);
sentinel_state_t sentinel_agent_check(sentinel_agent_t *agent, const sentinel_bounds_t *bounds, float observed_value, uint32_t current_tick);
bool sentinel_agent_is_healthy(const sentinel_agent_t *agent);

#endif
