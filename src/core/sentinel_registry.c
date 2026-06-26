#include "sentinel_registry.h"

void sentinel_registry_init(sentinel_registry_t *registry)
{
    registry->agent_count = 0;
}

bool sentinel_registry_register(sentinel_registry_t *registry, uint32_t subsystem_id, sentinel_bounds_t bounds)
{
    if (registry->agent_count >= SENTINEL_REGISTRY_MAX_AGENTS) {
        return false;
    }

    uint32_t index = registry->agent_count;
    sentinel_agent_init(&registry->agents[index], subsystem_id);
    registry->bounds[index] = bounds;
    registry->agent_count++;

    return true;
}

static int32_t sentinel_registry_find(const sentinel_registry_t *registry, uint32_t subsystem_id)
{
    for (uint32_t i = 0; i < registry->agent_count; i++) {
        if (registry->agents[i].subsystem_id == subsystem_id) {
            return (int32_t)i;
        }
    }
    return -1;
}

sentinel_state_t sentinel_registry_report(sentinel_registry_t *registry, uint32_t subsystem_id, float observed_value, uint32_t current_tick)
{
    int32_t index = sentinel_registry_find(registry, subsystem_id);
    if (index < 0) {
        return SENTINEL_STATE_UNRESPONSIVE;
    }

    return sentinel_agent_check(&registry->agents[index], &registry->bounds[index], observed_value, current_tick);
}

sentinel_state_t sentinel_registry_worst_state(const sentinel_registry_t *registry)
{
    sentinel_state_t worst = SENTINEL_STATE_NOMINAL;

    for (uint32_t i = 0; i < registry->agent_count; i++) {
        if (registry->agents[i].state > worst) {
            worst = registry->agents[i].state;
        }
    }

    return worst;
}

uint32_t sentinel_registry_count_in_state(const sentinel_registry_t *registry, sentinel_state_t state)
{
    uint32_t count = 0;

    for (uint32_t i = 0; i < registry->agent_count; i++) {
        if (registry->agents[i].state == state) {
            count++;
        }
    }

    return count;
}
