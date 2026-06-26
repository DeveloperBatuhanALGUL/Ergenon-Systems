#include "sentinel_agent.h"

void sentinel_agent_init(sentinel_agent_t *agent, uint32_t subsystem_id)
{
    agent->subsystem_id = subsystem_id;
    agent->state = SENTINEL_STATE_NOMINAL;
    agent->last_value = 0.0f;
    agent->last_update_tick = 0;
    agent->violation_count = 0;
}

sentinel_state_t sentinel_agent_check(sentinel_agent_t *agent, const sentinel_bounds_t *bounds, float observed_value, uint32_t current_tick)
{
    uint32_t elapsed_ticks = current_tick - agent->last_update_tick;

    if (elapsed_ticks > bounds->max_silence_ticks) {
        agent->state = SENTINEL_STATE_UNRESPONSIVE;
        agent->violation_count++;
        return agent->state;
    }

    agent->last_value = observed_value;
    agent->last_update_tick = current_tick;

    if (observed_value < bounds->min_bound || observed_value > bounds->max_bound) {
        agent->violation_count++;
        agent->state = (agent->violation_count >= 3) ? SENTINEL_STATE_CRITICAL : SENTINEL_STATE_WARNING;
    } else {
        agent->violation_count = 0;
        agent->state = SENTINEL_STATE_NOMINAL;
    }

    return agent->state;
}

bool sentinel_agent_is_healthy(const sentinel_agent_t *agent)
{
    return agent->state == SENTINEL_STATE_NOMINAL;
}
