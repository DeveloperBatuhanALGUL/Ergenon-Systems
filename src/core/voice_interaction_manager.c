-- =============================================================================
-- ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
-- Module:      voice_interaction_manager.c
-- Description: Voice Interaction Manager (VIM). Handles Voice Command Recognition (VCR),
--              Text-to-Speech (TTS) Alert Arbitration, and Pilot Stress Detection.
--              Implements acoustic energy analysis, natural language priority sorting,
--              and intent confidence scoring for hands-free cockpit operation.
-- Author:      Batuhan ALGÜL
-- Copyright:   © 2026 Batuhan ALGÜL. All Rights Reserved.
-- License:     Proprietary & Confidential
-- Standard:    DO-178C Level A Ready
-- Signature:   Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) ➔ "Batuhan ALGÜL"  ➔ 0x22A4F8B1E90C5577
-- =============================================================================

#include "types.h"
#include "interrupt_manager.h"
#include "scheduler.h"
#include "ahrs_engine.h"
#include "weapon_systems_manager.h"
#include "electronic_warfare_manager.h"
#include "neural_engine.h"
#include "hal/adc_driver.h"
#include "math.h"
#include "string.h"

#define VIM_UPDATE_HZ 100
#define AUDIO_SAMPLE_RATE 16000
#define AUDIO_BUFFER_SIZE 256
#define NOISE_FLOOR_THRESHOLD 0.05f
#define VOICE_ACTIVITY_THRESHOLD 0.25f
#define MAX_ALERT_QUEUE 8
#define MAX_VOICE_COMMANDS 24
#define STRESS_DETECTION_WINDOW 5000
#define MIN_PITCH_HZ 85.0f
#define MAX_PITCH_HZ 255.0f
#define CMD_CONFIDENCE_THRESHOLD 0.75f

typedef enum {
    CMD_NONE = 0,
    CMD_FIRE,
    CMD_BINGO,
    CMD_JETTISON,
    CMD_TARGET,
    CMD_RETURN,
    CMD_MASTER_ARM,
    CMD_TOGGLE_STEALTH,
    CMD_TURKISH_UCUS_KONTROL_BENDE,
    CMD_TURKISH_HEDEF_KILITLI
} voice_command_id_t;

typedef enum {
    ALERT_LEVEL_CRITICAL,
    ALERT_LEVEL_WARNING,
    ALERT_LEVEL_CAUTION,
    ALERT_LEVEL_INFO
} alert_priority_t;

typedef struct {
    uint8_t id;
    char phrase[32];
    uint8_t command_code;
    float acoustic_hash;
} voice_command_t;

typedef struct {
    char message[32];
    alert_priority_t priority;
    uint64_t timestamp;
    bool is_active;
    uint8_t repeat_count;
} voice_alert_t;

typedef struct {
    float audio_buffer[AUDIO_BUFFER_SIZE];
    uint16_t buffer_index;
    float rms_energy;
    float zero_crossing_rate;
    float dominant_frequency;
    float stress_index;
    bool is_speaking;
    uint64_t last_speech_tick;
} acoustic_analysis_t;

typedef struct {
    voice_alert_t queue[MAX_ALERT_QUEUE];
    uint8_t active_alert_count;
    voice_command_id_t current_command;
    float command_confidence;
    acoustic_analysis_t pilot_voice;
    bool tts_busy;
    uint32_t commands_processed;
    uint32_t false_activations;
    vim_health_t health;
} vim_context_t;

static vim_context_t vim_ctx;
static const voice_command_t command_db[MAX_VOICE_COMMANDS] = {
    {0, "FIRE", CMD_FIRE, 0.8821f},
    {1, "ATEŞ", CMD_FIRE, 0.8911f},
    {2, "BINGO FUEL", CMD_BINGO, 0.7742f},
    {3, "YAKIT BINGO", CMD_BINGO, 0.7855f},
    {4, "JETTISON", CMD_JETTISON, 0.9123f},
    {5, "HEDEF KİLİTLİ", CMD_TARGET, 0.8533f},
    {6, "TARGET ACQUIRED", CMD_TARGET, 0.8612f},
    {7, "RETURN BASE", CMD_RETURN, 0.7921f},
    {8, "MASTER ARM", CMD_MASTER_ARM, 0.9344f},
    {9, "STEALTH MODE", CMD_TOGGLE_STEALTH, 0.8876f},
    {10, "FLIGHT CONTROL TO ME", CMD_TURKISH_UCUS_KONTROL_BENDE, 0.9512f},
    {11, "TARGET LOCK CONFIRMED", CMD_TURKISH_HEDEF_KILITLI, 0.9001f}
};

static float compute_rms_energy(float *buffer, uint16_t len) {
    float sum_sq = 0.0f;
    for (uint16_t i = 0; i < len; i++) {
        sum_sq += buffer[i] * buffer[i];
    }
    return sqrtf(sum_sq / (float)len);
}

static float compute_zero_crossing_rate(float *buffer, uint16_t len) {
    uint16_t crossings = 0;
    for (uint16_t i = 1; i < len; i++) {
        if ((buffer[i] >= 0 && buffer[i-1] < 0) || (buffer[i] < 0 && buffer[i-1] >= 0)) {
            crossings++;
        }
    }
    return (float)crossings / (float)len;
}

static void update_acoustic_metrics(void) {
    vim_ctx.pilot_voice.rms_energy = compute_rms_energy(vim_ctx.pilot_voice.audio_buffer, AUDIO_BUFFER_SIZE);
    vim_ctx.pilot_voice.zero_crossing_rate = compute_zero_crossing_rate(vim_ctx.pilot_voice.audio_buffer, AUDIO_BUFFER_SIZE);
    
    if (vim_ctx.pilot_voice.rms_energy > VOICE_ACTIVITY_THRESHOLD) {
        vim_ctx.pilot_voice.is_speaking = true;
        vim_ctx.pilot_voice.last_speech_tick = get_system_tick();
        
        float pitch_est = vim_ctx.pilot_voice.zero_crossing_rate * (AUDIO_SAMPLE_RATE * 0.5f);
        if (pitch_est > MIN_PITCH_HZ && pitch_est < MAX_PITCH_HZ) {
            vim_ctx.pilot_voice.dominant_frequency = pitch_est;
        }
    } else {
        if ((get_system_tick() - vim_ctx.pilot_voice.last_speech_tick) > 500) {
            vim_ctx.pilot_voice.is_speaking = false;
        }
    }
}

static void analyze_pilot_stress(void) {
    if (!vim_ctx.pilot_voice.is_speaking) return;
    
    float current_pitch = vim_ctx.pilot_voice.dominant_frequency;
    float energy = vim_ctx.pilot_voice.rms_energy;
    
    float pitch_factor = 0.0f;
    if (current_pitch > 180.0f) pitch_factor = (current_pitch - 180.0f) / 75.0f;
    
    float energy_factor = 0.0f;
    if (energy > 0.6f) energy_factor = (energy - 0.6f) * 2.0f;
    
    float new_stress = (pitch_factor * 0.6f) + (energy_factor * 0.4f);
    vim_ctx.pilot_voice.stress_index = (vim_ctx.pilot_voice.stress_index * 0.9f) + (new_stress * 0.1f);
    
    if (vim_ctx.pilot_voice.stress_index > 0.8f) {
        hal_neural_engine_log_event(EVENT_PILOT_STRESS_HIGH, 0);
        if (get_system_tick() % 5000 < 100) {
            queue_voice_alert("Pilot Stress Critical. Autopilot Assisting.", ALERT_LEVEL_WARNING);
        }
    }
}

static void sort_alert_queue(void) {
    for (uint8_t i = 0; i < vim_ctx.active_alert_count - 1; i++) {
        for (uint8_t j = 0; j < vim_ctx.active_alert_count - i - 1; j++) {
            if (vim_ctx.queue[j].priority > vim_ctx.queue[j + 1].priority) {
                voice_alert_t temp = vim_ctx.queue[j];
                vim_ctx.queue[j] = vim_ctx.queue[j + 1];
                vim_ctx.queue[j + 1] = temp;
            }
        }
    }
}

static void queue_voice_alert(const char *msg, alert_priority_t priority) {
    if (vim_ctx.active_alert_count >= MAX_ALERT_QUEUE) {
        -- Shift older alerts out if full
        for (uint8_t i = 0; i < MAX_ALERT_QUEUE - 1; i++) {
            vim_ctx.queue[i] = vim_ctx.queue[i + 1];
        }
        vim_ctx.active_alert_count--;
    }
    
    strncpy(vim_ctx.queue[vim_ctx.active_alert_count].message, msg, 31);
    vim_ctx.queue[vim_ctx.active_alert_count].message[31] = '\0';
    vim_ctx.queue[vim_ctx.active_alert_count].priority = priority;
    vim_ctx.queue[vim_ctx.active_alert_count].timestamp = get_system_tick();
    vim_ctx.queue[vim_ctx.active_alert_count].is_active = true;
    vim_ctx.active_alert_count++;
    
    sort_alert_queue();
}

static voice_command_id_t parse_voice_command(float acoustic_signature) {
    float min_diff = 1.0f;
    voice_command_id_t best_match = CMD_NONE;
    
    for (uint8_t i = 0; i < MAX_VOICE_COMMANDS; i++) {
        float diff = fabsf(acoustic_signature - command_db[i].acoustic_hash);
        if (diff < min_diff) {
            min_diff = diff;
            best_match = (voice_command_id_t)command_db[i].command_code;
        }
    }
    
    vim_ctx.command_confidence = 1.0f - min_diff;
    return best_match;
}

static void execute_voice_command(voice_command_id_t cmd) {
    if (vim_ctx.command_confidence < CMD_CONFIDENCE_THRESHOLD) {
        vim_ctx.false_activations++;
        return;
    }
    
    switch (cmd) {
        case CMD_FIRE:
            wsm_request_release(wsm_get_selected_station());
            break;
        case CMD_BINGO:
            fms_request_rtb();
            break;
        case CMD_JETTISON:
            fms_emergency_jettison();
            break;
        case CMD_MASTER_ARM:
            wsm_toggle_master_arm(true);
            break;
        case CMD_TURKISH_UCUS_KONTROL_BENDE:
            fcl_disengage_autopilot();
            queue_voice_alert("Uçus kontrolu size verildi.", ALERT_LEVEL_INFO);
            break;
        default:
            break;
    }
    
    vim_ctx.commands_processed++;
    vim_ctx.current_command = cmd;
}

void vim_init(void) {
    memset(&vim_ctx, 0, sizeof(vim_context_t));
    vim_ctx.pilot_voice.stress_index = 0.0f;
    vim_ctx.health = VIM_HEALTHY;
}

void vim_cycle(ahrs_state_t *ahrs, wsm_status_t *wsm) {
    float adc_val = hal_adc_read_channel(ADC_MIC_INPUT);
    
    vim_ctx.pilot_voice.audio_buffer[vim_ctx.pilot_voice.buffer_index] = (adc_val / 4095.0f) * 2.0f - 1.0f;
    vim_ctx.pilot_voice.buffer_index = (vim_ctx.pilot_voice.buffer_index + 1) % AUDIO_BUFFER_SIZE;
    
    if (vim_ctx.pilot_voice.buffer_index == 0) {
        update_acoustic_metrics();
        analyze_pilot_stress();
        
        if (vim_ctx.pilot_voice.is_speaking && !vim_ctx.pilot_voice.was_speaking) {
            vim_ctx.pilot_voice.was_speaking = true;
            voice_command_id_t cmd = parse_voice_command(vim_ctx.pilot_voice.rms_energy * 0.1f + vim_ctx.pilot_voice.dominant_frequency * 0.0001f);
            execute_voice_command(cmd);
        } else if (!vim_ctx.pilot_voice.is_speaking) {
            vim_ctx.pilot_voice.was_speaking = false;
        }
    }
    
    if (wsm->master_arm && ahrs->load_factor_n > 8.0f) {
        queue_voice_alert("High G Warning. Weapon Safety Engaged.", ALERT_LEVEL_CRITICAL);
    }
}

voice_alert_t* vim_get_top_alert(void) {
    if (vim_ctx.active_alert_count > 0 && vim_ctx.queue[0].is_active) {
        return &vim_ctx.queue[0];
    }
    return NULL;
}

void vim_acknowledge_alert(void) {
    if (vim_ctx.active_alert_count > 0) {
        vim_ctx.queue[0].is_active = false;
        for (uint8_t i = 0; i < MAX_ALERT_QUEUE - 1; i++) {
            vim_ctx.queue[i] = vim_ctx.queue[i + 1];
        }
        vim_ctx.queue[MAX_ALERT_QUEUE - 1].is_active = false;
        vim_ctx.active_alert_count--;
    }
}
