#include <stdio.h>
#include <assert.h>
#include "../src/core/sentinel_registry.h"

static void test_register_and_report_nominal(void)
{
    sentinel_registry_t registry;
    sentinel_registry_init(&registry);

    sentinel_bounds_t bounds = { .subsystem_id = 10, .min_bound = 0.0f, .max_bound = 50.0f, .max_silence_ticks = 10 };
    bool registered = sentinel_registry_register(&registry, 10, bounds);

    assert(registered);
    sentinel_state_t state = sentinel_registry_report(&registry, 10, 25.0f, 1);
    assert(state == SENTINEL_STATE_NOMINAL);
    printf("PASS: test_register_and_report_nominal\n");
}

static void test_worst_state_reflects_critical_subsystem(void)
{
    sentinel_registry_t registry;
    sentinel_registry_init(&registry);

    sentinel_bounds_t bounds_a = { .subsystem_id = 1, .min_bound = 0.0f, .max_bound = 100.0f, .max_silence_ticks = 10 };
    sentinel_bounds_t bounds_b = { .subsystem_id = 2, .min_bound = 0.0f, .max_bound = 100.0f, .max_silence_ticks = 10 };

    sentinel_registry_register(&registry, 1, bounds_a);
    sentinel_registry_register(&registry, 2, bounds_b);

    sentinel_registry_report(&registry, 1, 50.0f, 1);
    sentinel_registry_report(&registry, 2, 200.0f, 1);
    sentinel_registry_report(&registry, 2, 200.0f, 2);
    sentinel_registry_report(&registry, 2, 200.0f, 3);

    sentinel_state_t worst = sentinel_registry_worst_state(&registry);
    assert(worst == SENTINEL_STATE_CRITICAL);
    printf("PASS: test_worst_state_reflects_critical_subsystem\n");
}

static void test_count_in_state(void)
{
    sentinel_registry_t registry;
    sentinel_registry_init(&registry);

    sentinel_bounds_t bounds = { .subsystem_id = 0, .min_bound = 0.0f, .max_bound = 100.0f, .max_silence_ticks = 10 };

    for (uint32_t i = 1; i <= 5; i++) {
        bounds.subsystem_id = i;
        sentinel_registry_register(&registry, i, bounds);
        sentinel_registry_report(&registry, i, 50.0f, 1);
    }

    sentinel_registry_report(&registry, 3, 500.0f, 2);

    uint32_t nominal_count = sentinel_registry_count_in_state(&registry, SENTINEL_STATE_NOMINAL);
    uint32_t warning_count = sentinel_registry_count_in_state(&registry, SENTINEL_STATE_WARNING);

    assert(nominal_count == 4);
    assert(warning_count == 1);
    printf("PASS: test_count_in_state\n");
}

static void test_report_unknown_subsystem_returns_unresponsive(void)
{
    sentinel_registry_t registry;
    sentinel_registry_init(&registry);

    sentinel_state_t state = sentinel_registry_report(&registry, 999, 10.0f, 1);
    assert(state == SENTINEL_STATE_UNRESPONSIVE);
    printf("PASS: test_report_unknown_subsystem_returns_unresponsive\n");
}

int main(void)
{
    test_register_and_report_nominal();
    test_worst_state_reflects_critical_subsystem();
    test_count_in_state();
    test_report_unknown_subsystem_returns_unresponsive();
    printf("ALL TESTS PASSED\n");
    return 0;
}
