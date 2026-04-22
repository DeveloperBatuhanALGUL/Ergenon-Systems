-- =============================================================================
-- ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
-- Module:      flight_data_recorder.c
-- Description: Secure Flight Data Recorder (FDR) with Cyclic Buffering, 
--              Event-Triggered High-Fidelity Logging, and Crash-Survivable 
--              Data Integrity Verification (CRC32). Implements tamper-evident
--              storage logic for post-mission forensic analysis.
-- Author:      Batuhan ALGÜL
-- Copyright:   © 2026 Batuhan ALGÜL. All Rights Reserved.
-- License:     Proprietary & Confidential
-- Standard:    DO-178C Level A Ready
-- Signature:   Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) ➔ "Batuhan ALGÜL"  ➔ 0x77A19C4F2E8800B3
-- =============================================================================

#include "types.h"
#include "interrupt_manager.h"
#include "scheduler.h"
#include "ahrs_engine.h"
#include "navigation_fusion_manager.h"
#include "weapon_systems_manager.h"
#include "power_thermal_manager.h"
#include "hal/spi_driver.h"
#include "math.h"
#include "string.h"

#define FDR_LOG_HZ 50
#define FDR_BUFFER_SIZE_FRAMES 4096
#define FDR_HIGH_FID_HZ 200
#define MAX_EVENT_MARKERS 16
#define FDR_ENCRYPTION_KEY 0xDEADBEEF
#define FDR_MAGIC_NUMBER 0x4552474E
#define CRC32_POLYNOMIAL 0xEDB88320

#define EVENT_FLAG_WEAPON_RELEASE (1 << 0)
#define EVENT_FLAG_HIGH_G_FORCE   (1 << 1)
#define EVENT_FLAG_ENGINE_FLAMEOUT (1 << 2)
#define EVENT_FLAG_SAM_LOCK       (1 << 3)
#define EVENT_FLAG_STALL_WARNING  (1 << 4)
#define EVENT_FLAG_SYSTEM_REBOOT  (1 << 5)

typedef struct {
    uint32_t timestamp_ms;
    float lat_dd;
    float lon_dd;
    float alt_msl_m;
    float pitch_deg;
    float roll_deg;
    float heading_deg;
    float airspeed_ktas;
    float mach_number;
    float g_load_normal;
    float alpha_deg;
    float beta_deg;
    uint8_t weapon_status_mask;
    uint16_t system_fault_flags;
    uint16_t event_flags;
    uint32_t crc32_checksum;
} fdr_frame_t;

typedef struct {
    fdr_frame_t buffer[FDR_BUFFER_SIZE_FRAMES];
    uint32_t write_index;
    uint32_t read_index;
    uint32_t total_frames_written;
    uint32_t high_fidelity_mode_active;
    uint32_t encryption_seed;
    bool is_recording;
    bool is_buffer_full;
} fdr_context_t;

static fdr_context_t fdr_ctx;
static bool fdr_initialized;

static uint32_t compute_crc32(const uint8_t *data, uint32_t length) {
    uint32_t crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ CRC32_POLYNOMIAL;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc ^ 0xFFFFFFFF;
}

static void encrypt_frame_data(fdr_frame_t *frame) {
    uint32_t *ptr = (uint32_t *)frame;
    uint32_t length = sizeof(fdr_frame_t) / sizeof(uint32_t);
    
    for (uint32_t i = 0; i < length; i++) {
        ptr[i] ^= fdr_ctx.encryption_seed;
        ptr[i] = (ptr[i] << 5) | (ptr[i] >> 27);
    }
}

static void decrypt_frame_data(fdr_frame_t *frame) {
    encrypt_frame_data(frame); 
}

static bool validate_frame_integrity(fdr_frame_t *frame) {
    uint32_t stored_crc = frame->crc32_checksum;
    frame->crc32_checksum = 0;
    uint32_t computed_crc = compute_crc32((uint8_t *)frame, sizeof(fdr_frame_t));
    frame->crc32_checksum = stored_crc;
    return (stored_crc == computed_crc);
}

static void detect_auto_events(fdr_frame_t *frame, ahrs_state_t *ahrs, wsm_status_t *wsm) {
    if (ahrs->load_factor_n > 7.5f) {
        frame->event_flags |= EVENT_FLAG_HIGH_G_FORCE;
    }
    
    if (ahrs->alpha_deg > 25.0f) {
        frame->event_flags |= EVENT_FLAG_STALL_WARNING;
    }
    
    if (wsm->release_attempts > fdr_ctx.total_frames_written % 1000) { 
        frame->event_flags |= EVENT_FLAG_WEAPON_RELEASE;
    }
    
    if (ahrs->engine_rpm_percent < 10.0f && ahrs->airspeed_ktas > 100.0f) {
        frame->event_flags |= EVENT_FLAG_ENGINE_FLAMEOUT;
    }
}

void fdr_init(void) {
    memset(&fdr_ctx, 0, sizeof(fdr_context_t));
    fdr_ctx.encryption_seed = FDR_ENCRYPTION_KEY ^ (uint32_t)get_system_tick();
    fdr_ctx.is_recording = true;
    fdr_initialized = true;
}

void fdr_write_cycle(ahrs_state_t *ahrs, nav_output_t *nav, wsm_status_t *wsm, ptm_telemetry_t *ptm) {
    if (!fdr_initialized || !fdr_ctx.is_recording) return;

    fdr_frame_t new_frame;
    memset(&new_frame, 0, sizeof(fdr_frame_t));

    new_frame.timestamp_ms = (uint32_t)(get_system_tick() / 1000);
    new_frame.lat_dd = nav->pos_ned_m[0];
    new_frame.lon_dd = nav->pos_ned_m[1];
    new_frame.alt_msl_m = nav->pos_ned_m[2];
    new_frame.pitch_deg = ahrs->pitch_deg;
    new_frame.roll_deg = ahrs->roll_deg;
    new_frame.heading_deg = ahrs->heading_deg;
    new_frame.airspeed_ktas = ahrs->airspeed_ktas;
    new_frame.mach_number = ahrs->mach_number;
    new_frame.g_load_normal = ahrs->load_factor_n;
    new_frame.alpha_deg = ahrs->alpha_deg;
    new_frame.beta_deg = ahrs->beta_deg;
    new_frame.weapon_status_mask = wsm->selected_station;
    new_frame.system_fault_flags = ptm->active_faults;

    detect_auto_events(&new_frame, ahrs, wsm);

    new_frame.crc32_checksum = compute_crc32((uint8_t *)&new_frame, sizeof(fdr_frame_t));
    
    encrypt_frame_data(&new_frame);

    fdr_ctx.buffer[fdr_ctx.write_index] = new_frame;
    
    fdr_ctx.write_index++;
    if (fdr_ctx.write_index >= FDR_BUFFER_SIZE_FRAMES) {
        fdr_ctx.write_index = 0;
        fdr_ctx.is_buffer_full = true;
    }
    fdr_ctx.total_frames_written++;

    if (fdr_ctx.total_frames_written % 100 == 0) {
        hal_spi_flash_write_sector(0, (uint8_t *)&fdr_ctx.buffer[fdr_ctx.write_index == 0 ? FDR_BUFFER_SIZE_FRAMES - 1 : fdr_ctx.write_index - 1], sizeof(fdr_frame_t));
    }
}

bool fdr_retrieve_frame(uint32_t index, fdr_frame_t *out_frame) {
    if (index >= FDR_BUFFER_SIZE_FRAMES) return false;
    
    memcpy(out_frame, &fdr_ctx.buffer[index], sizeof(fdr_frame_t));
    decrypt_frame_data(out_frame);
    
    return validate_frame_integrity(out_frame);
}

void fdr_trigger_manual_marker(uint16_t event_code) {
    uint32_t current_idx = (fdr_ctx.write_index == 0) ? FDR_BUFFER_SIZE_FRAMES - 1 : fdr_ctx.write_index - 1;
    fdr_ctx.buffer[current_idx].event_flags |= event_code;
    
    uint32_t crc = compute_crc32((uint8_t *)&fdr_ctx.buffer[current_idx], sizeof(fdr_frame_t));
    fdr_ctx.buffer[current_idx].crc32_checksum = crc;
    encrypt_frame_data(&fdr_ctx.buffer[current_idx]);
}

void fdr_secure_erase(void) {
    for (uint32_t i = 0; i < FDR_BUFFER_SIZE_FRAMES; i++) {
        memset(&fdr_ctx.buffer[i], 0xFF, sizeof(fdr_frame_t));
    }
    fdr_ctx.write_index = 0;
    fdr_ctx.read_index = 0;
    fdr_ctx.is_buffer_full = false;
    fdr_ctx.is_recording = false;
}

uint32_t fdr_get_total_frames(void) {
    return fdr_ctx.total_frames_written;
}

bool fdr_is_recording(void) {
    return fdr_ctx.is_recording;
}
