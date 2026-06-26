#include <stdio.h>
#include <assert.h>
#include "../src/core/sentinel_agent.h"

static void test_nominal_within_bounds(void)
{
    sentinel_agent_t agent;
    sentinel_bounds_t bounds = { .subsystem_id = 1, .min_bound = 0.0f, .max_bound = 100.0f, .max_silence_ticks = 10 };
    sentinel_agent_init(&agent, 1);

    sentinel_state_t state = sentinel_agent_check(&agent, &bounds, 50.0f, 1);
    assert(state == SENTINEL_STATE_NOMINAL);
    assert(sentinel_agent_is_healthy(&agent));
    printf("PASS: test_nominal_within_bounds\n");
}

static void test_escalates_to_critical_after_three_violations(void)
{
    sentinel_agent_t agent;
    sentinel_bounds_t bounds = { .subsystem_id = 2, .min_bound = 0.0f, .max_bound = 100.0f, .max_silence_ticks = 10 };
    sentinel_agent_init(&agent, 2);

    sentinel_agent_check(&agent, &bounds, 150.0f, 1);
    sentinel_agent_check(&agent, &bounds, 150.0f, 2);
    sentinel_state_t state = sentinel_agent_check(&agent, &bounds, 150.0f, 3);

    assert(state == SENTINEL_STATE_CRITICAL);
    assert(!sentinel_agent_is_healthy(&agent));
    printf("PASS: test_escalates_to_critical_after_three_violations\n");
}

static void test_unresponsive_after_silence(void)
{
    sentinel_agent_t agent;
    sentinel_bounds_t bounds = { .subsystem_id = 3, .min_bound = 0.0f, .max_bound = 100.0f, .max_silence_ticks = 5 };
    sentinel_agent_init(&agent, 3);

    sentinel_agent_check(&agent, &bounds, 50.0f, 1);
    sentinel_state_t state = sentinel_agent_check(&agent, &bounds, 50.0f, 20);

    assert(state == SENTINEL_STATE_UNRESPONSIVE);
    printf("PASS: test_unresponsive_after_silence\n");
}

static void test_recovers_to_nominal_after_violation(void)
{
    sentinel_agent_t agent;
    sentinel_bounds_t bounds = { .subsystem_id = 4, .min_bound = 0.0f, .max_bound = 100.0f, .max_silence_ticks = 10 };
    sentinel_agent_init(&agent, 4);

    sentinel_agent_check(&agent, &bounds, 150.0f, 1);
    sentinel_state_t state = sentinel_agent_check(&agent, &bounds, 50.0f, 2);

    assert(state == SENTINEL_STATE_NOMINAL);
    printf("PASS: test_recovers_to_nominal_after_violation\n");
}

int main(void)
{
    test_nominal_within_bounds();
    test_escalates_to_critical_after_three_violations();
    test_unresponsive_after_silence();
    test_recovers_to_nominal_after_violation();
    printf("ALL TESTS PASSED\n");
    return 0;
}
