-- =============================================================================
-- ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
-- Module:      electronic_warfare_manager.c
-- Description: Electronic Warfare Management Subsystem. Handles RWR data ingestion,
--              Threat Evaluation & Ordering (TEO), Radar Cross Section (RCS) 
--              optimization logic, and jamming coordination.
-- Author:      Batuhan ALGÜL
-- Copyright:   © 2026 Batuhan ALGÜL. All Rights Reserved.
-- License:     Proprietary & Confidential
-- Standard:    DO-178C Level A Ready
-- Signature:   Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) ➔ "Batuhan ALGÜL"  ➔ 0xE84F1A2B9C0D
-- =============================================================================

#include "types.h"
#include "interrupt_manager.h"
#include "neural_engine.h"
#include "ahrs_engine.h"
#include "math.h"

#define MAX_THREAT_SLOTS 16
#define RWR_REFRESH_HZ 50
#define RCS_THRESHOLD_DB 0.05
#define THREAT_PRIORITY_DEADLY 100
#define THREAT_PRIORITY_HIGH 75
#define THREAT_PRIORITY_MEDIUM 50
#define THREAT_PRIORITY_LOW 25

typedef struct {
    float bearing_deg;
    float elevation_deg;
    float signal_strength_dbm;
    float doppler_shift_hz;
    threat_type_t type;
    bool is_locked;
    uint64_t first_seen_tick;
    uint64_t last_update_tick;
    uint8_t confidence_level;
} rwr_contact_t;

typedef struct {
    uint8_t slot_index;
    rwr_contact_t contact;
    uint8_t priority_score;
    bool is_active;
} threat_slot_t;

static threat_slot_t threat_db[MAX_THREAT_SLOTS];
static stealth_mode_t current_stealth_mode = STEALTH_FULL;
static uint8_t active_threat_count = 0;

static float calculate_radar_cross_section(float aspect_angle_deg, float freq_ghz) {
    float sigma = 0.0f;
    float angle_rad = aspect_angle_deg * M_PI / 180.0f;
    float lambda = 0.299792458f / freq_ghz;
    
    sigma = (4.0f * M_PI * powf(aspect_angle_deg / 10.0f, 4.0f)) / (lambda * lambda);
    
    if (current_stealth_mode == STEALTH_FULL) {
        sigma *= 0.1f;
    } else if (current_stealth_mode == STEALTH_WEAPON_BAY_OPEN) {
        sigma *= 1.5f;
    }
    
    return sigma;
}

static uint8_t calculate_threat_priority(rwr_contact_t *contact, ahrs_state_t *state) {
    uint8_t score = 0;
    float range_km = 50.0f * powf(10.0f, (contact->signal_strength_dbm + 100.0f) / 20.0f);
    float closing_speed_ms = contact->doppler_shift_hz * 0.03f; 
    
    if (contact->is_locked) {
        score += THREAT_PRIORITY_DEADLY;
    } else if (contact->type == THREAT_SAM) {
        score += THREAT_PRIORITY_HIGH;
    } else if (contact->type == THREAT_FCR) {
        score += THREAT_PRIORITY_MEDIUM;
    } else {
        score += THREAT_PRIORITY_LOW;
    }
    
    if (range_km < 10.0f) {
        score += 20;
    } else if (range_km < 50.0f) {
        score += 10;
    }
    
    if (closing_speed_ms > 300.0f) {
        score += 15;
    }
    
    return (score > 100) ? 100 : score;
}

static void update_threat_database(rwr_raw_data_t *raw_data, ahrs_state_t *state) {
    uint8_t best_empty_slot = MAX_THREAT_SLOTS;
    bool found_existing = false;
    
    for (uint8_t i = 0; i < MAX_THREAT_SLOTS; i++) {
        if (threat_db[i].is_active) {
            float angle_diff = fabsf(raw_data->bearing_deg - threat_db[i].contact.bearing_deg);
            
            if (angle_diff < 5.0f && raw_data->pulse_type == threat_db[i].contact.type) {
                threat_db[i].contact.last_update_tick = get_system_tick();
                threat_db[i].contact.signal_strength_dbm = raw_data->signal_strength_dbm;
                threat_db[i].contact.doppler_shift_hz = raw_data->doppler_shift_hz;
                
                threat_db[i].priority_score = calculate_threat_priority(&threat_db[i].contact, state);
                found_existing = true;
                break;
            }
        } else {
            if (best_empty_slot == MAX_THREAT_SLOTS) {
                best_empty_slot = i;
            }
        }
    }
    
    if (!found_existing && best_empty_slot != MAX_THREAT_SLOTS) {
        threat_db[best_empty_slot].is_active = true;
        threat_db[best_empty_slot].slot_index = best_empty_slot;
        threat_db[best_empty_slot].contact.bearing_deg = raw_data->bearing_deg;
        threat_db[best_empty_slot].contact.elevation_deg = raw_data->elevation_deg;
        threat_db[best_empty_slot].contact.signal_strength_dbm = raw_data->signal_strength_dbm;
        threat_db[best_empty_slot].contact.doppler_shift_hz = raw_data->doppler_shift_hz;
        threat_db[best_empty_slot].contact.type = raw_data->pulse_type;
        threat_db[best_empty_slot].contact.is_locked = raw_data->is_locked;
        threat_db[best_empty_slot].contact.first_seen_tick = get_system_tick();
        threat_db[best_empty_slot].contact.last_update_tick = get_system_tick();
        threat_db[best_empty_slot].contact.confidence_level = 80;
        
        threat_db[best_empty_slot].priority_score = calculate_threat_priority(&threat_db[best_empty_slot].contact, state);
        active_threat_count++;
    }
}

static void purge_stale_threats(void) {
    uint64_t current_tick = get_system_tick();
    uint64_t stale_threshold = (RWR_REFRESH_HZ * 5); 
    
    for (uint8_t i = 0; i < MAX_THREAT_SLOTS; i++) {
        if (threat_db[i].is_active) {
            if ((current_tick - threat_db[i].contact.last_update_tick) > stale_threshold) {
                threat_db[i].is_active = false;
                active_threat_count--;
            }
        }
    }
}

void ew_manager_init(void) {
    for (uint8_t i = 0; i < MAX_THREAT_SLOTS; i++) {
        threat_db[i].is_active = false;
        threat_db[i].slot_index = i;
        threat_db[i].priority_score = 0;
    }
    active_threat_count = 0;
    current_stealth_mode = STEALTH_FULL;
}

void ew_manager_update_cycle(ahrs_state_t *state) {
    rwr_raw_data_t rwr_buffer;
    
    if (hal_rwr_read(&rwr_buffer) == HAL_OK) {
        update_threat_database(&rwr_buffer, state);
    }
    
    purge_stale_threats();
    
    threat_slot_t sorted_threats[MAX_THREAT_SLOTS];
    uint8_t active_idx = 0;
    
    for (uint8_t i = 0; i < MAX_THREAT_SLOTS; i++) {
        if (threat_db[i].is_active) {
            sorted_threats[active_idx++] = threat_db[i];
        }
    }
    
    for (uint8_t i = 0; i < active_idx - 1; i++) {
        for (uint8_t j = 0; j < active_idx - i - 1; j++) {
            if (sorted_threats[j].priority_score < sorted_threats[j + 1].priority_score) {
                threat_slot_t temp = sorted_threats[j];
                sorted_threats[j] = sorted_threats[j + 1];
                sorted_threats[j + 1] = temp;
            }
        }
    }
    
    if (active_idx > 0) {
        if (sorted_threats[0].contact.is_locked && sorted_threats[0].contact.type == THREAT_SAM) {
            if (current_stealth_mode != STEALTH_EMERGENCY) {
                current_stealth_mode = STEALTH_EMERGENCY;
                hal_radar_transmitter_set_power(POWER_OFF);
                hal_ir_suppressor_enable(true);
            }
        } else if (sorted_threats[0].priority_score < THREAT_PRIORITY_MEDIUM) {
             if (current_stealth_mode == STEALTH_EMERGENCY) {
                 current_stealth_mode = STEALTH_FULL;
                 hal_ir_suppressor_enable(false);
             }
        }
    }
    
    hal_displays_update_threat_list(sorted_threats, active_idx);
}

stealth_mode_t ew_get_stealth_mode(void) {
    return current_stealth_mode;
}

void ew_force_stealth_mode(stealth_mode_t mode) {
    current_stealth_mode = mode;
    if (mode == STEALTH_FULL) {
        hal_radar_transmitter_set_power(POWER_LOW);
    } else {
        hal_radar_transmitter_set_power(POWER_OFF);
    }
}
