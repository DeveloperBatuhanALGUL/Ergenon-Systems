#ifndef ERGENON_SENTINEL_REGISTRY_H
#define ERGENON_SENTINEL_REGISTRY_H

#include "sentinel_agent.h"

#define SENTINEL_REGISTRY_MAX_AGENTS 256

typedef struct {
    sentinel_agent_t agents[SENTINEL_REGISTRY_MAX_AGENTS];
    sentinel_bounds_t bounds[SENTINEL_REGISTRY_MAX_AGENTS];
    uint32_t agent_count;
} sentinel_registry_t;

void sentinel_registry_init(sentinel_registry_t *registry);
bool sentinel_registry_register(sentinel_registry_t *registry, uint32_t subsystem_id, sentinel_bounds_t bounds);
sentinel_state_t sentinel_registry_report(sentinel_registry_t *registry, uint32_t subsystem_id, float observed_value, uint32_t current_tick);
sentinel_state_t sentinel_registry_worst_state(const sentinel_registry_t *registry);
uint32_t sentinel_registry_count_in_state(const sentinel_registry_t *registry, sentinel_state_t state);

#endif
