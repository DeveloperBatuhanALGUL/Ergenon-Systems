-- =============================================================================
-- ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
-- Module:      pvi_display_manager.c
-- Description: Pilot Vehicle Interface (PVI) & HMD Symbology Generator.
--              Implements real-time coordinate transformation (World -> Body -> Eye -> Screen),
--              perspective projection for HUD/HMD, flight path vector computation,
--              horizon line generation, and target bracketing logic.
-- Author:      Batuhan ALGÜL
-- Copyright:   © 2026 Batuhan ALGÜL. All Rights Reserved.
-- License:     Proprietary & Confidential
-- Standard:    DO-178C Level A Ready
-- Signature:   Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) ➔ "Batuhan ALGÜL"  ➔ 0x99C21E5F8A0D4433
-- =============================================================================

#include "types.h"
#include "interrupt_manager.h"
#include "scheduler.h"
#include "ahrs_engine.h"
#include "navigation_fusion_manager.h"
#include "weapon_systems_manager.h"
#include "situational_awareness_fusion.h"
#include "math.h"
#include "string.h"

#define PVI_UPDATE_HZ 60
#define HUD_FOV_DEG 30.0f
#define HMD_FOV_DEG 40.0f
#define SCREEN_WIDTH_PIXELS 1024
#define SCREEN_HEIGHT_PIXELS 768
#define CLIPPING_MARGIN_PERCENT 0.15f
#define SYMBOL_MAX_Z_DEPTH_M 50000.0f
#define HUD_BRIGHTNESS_MAX 1.0f
#define HUD_BRIGHTNESS_MIN 0.1f
#define FPV_SMOOTHING_ALPHA 0.8f
#define HORIZON_LINE_LENGTH_PIXELS 400

typedef enum {
    SYM_FLIGHT_PATH_VEC,
    SYM_VELOCITY_VEC,
    SYM_AIRCRAFT_REF,
    SYM_HORIZON_LINE,
    SYM_TARGET_BRACKET,
    SYM_THREAT_RING,
    SYM_WAYPOINT_MARKER,
    SYM_STATUS_TEXT
} symbology_type_t;

typedef struct {
    float x_px;
    float y_px;
    float z_depth_m;
    float intensity;
    symbology_type_t type;
    uint8_t target_id;
    bool is_valid;
} display_element_t;

typedef struct {
    float display_frame[SCREEN_HEIGHT_PIXELS * SCREEN_WIDTH_PIXELS];
    uint8_t active_element_count;
    display_element_t elements[64];
    float current_brightness;
    float pilot_head_azimuth;
    float pilot_head_elevation;
    bool is_clipped;
} pvi_frame_t;

typedef struct {
    float rotation_matrix[9];
    float fov_scale_x;
    float fov_scale_y;
    float principal_point_x;
    float principal_point_y;
} projection_params_t;

static pvi_frame_t pvi_buffer;
static projection_params_t proj_params;
static bool pvi_initialized;

static void compute_body_to_display_rotation(float pitch_rad, float roll_rad, float head_az_rad, float head_el_rad, float *out_matrix) {
    float cp = cosf(pitch_rad);
    float sp = sinf(pitch_rad);
    float cr = cosf(roll_rad);
    float sr = sinf(roll_rad);
    float cha = cosf(head_az_rad);
    float sha = sinf(head_az_rad);
    float che = cosf(head_el_rad);
    float she = sinf(head_el_rad);

    out_matrix[0] = cha * che;
    out_matrix[1] = sha * che;
    out_matrix[2] = -she;
    out_matrix[3] = cha * she * sp - sha * cr * cp;
    out_matrix[4] = sha * she * sp + cha * cr * cp;
    out_matrix[5] = sr * cp;
    out_matrix[6] = cha * she * cp + sha * cr * sp;
    out_matrix[7] = sha * she * cp - cha * cr * sp;
    out_matrix[8] = sr * sp;
}

static void project_world_to_screen(float x_body, float y_body, float z_body, float *out_x_px, float *out_y_px, bool *out_on_screen) {
    if (z_body <= 1.0f) {
        *out_on_screen = false;
        return;
    }

    float x_proj = (x_body / z_body) * proj_params.fov_scale_x + proj_params.principal_point_x;
    float y_proj = (y_body / z_body) * proj_params.fov_scale_y + proj_params.principal_point_y;

    float margin_x = SCREEN_WIDTH_PIXELS * CLIPPING_MARGIN_PERCENT;
    float margin_y = SCREEN_HEIGHT_PIXELS * CLIPPING_MARGIN_PERCENT;

    if (x_proj < -margin_x || x_proj > (SCREEN_WIDTH_PIXELS + margin_x) ||
        y_proj < -margin_y || y_proj > (SCREEN_HEIGHT_PIXELS + margin_y)) {
        *out_on_screen = false;
    } else {
        *out_on_screen = true;
    }

    *out_x_px = x_proj;
    *out_y_px = y_proj;
}

static void compute_flight_path_vector(ahrs_state_t *ahrs, nav_output_t *nav, float *fpv_pitch_deg, float *fpv_yaw_deg) {
    float vn = nav->vel_ned_ms[0];
    float ve = nav->vel_ned_ms[1];
    float vd = nav->vel_ned_ms[2];

    float v_horizontal = sqrtf(vn * vn + ve * ve);
    
    if (v_horizontal < 1.0f) {
        *fpv_pitch_deg = 0.0f;
        *fpv_yaw_deg = 0.0f;
        return;
    }

    float gamma_rad = atan2f(-vd, v_horizontal);
    float chi_rad = atan2f(ve, vn);

    float pitch_rad = ahrs->pitch_rad;
    float roll_rad = ahrs->roll_rad;
    float heading_rad = ahrs->heading_rad;

    float rel_yaw = chi_rad - heading_rad;
    
    float fpv_x = cosf(gamma_rad) * sinf(rel_yaw);
    float fpv_y = sinf(gamma_rad);
    float fpv_z = cosf(gamma_rad) * cosf(rel_yaw);

    float pitch_comp = asinf(fpv_y);
    float yaw_comp = atan2f(fpv_x, fpv_z);

    *fpv_pitch_deg = pitch_comp * 57.295779513f;
    *fpv_yaw_deg = yaw_comp * 57.295779513f;
}

static void generate_horizon_line(ahrs_state_t *ahrs, display_element_t *elements, uint8_t *count) {
    float roll_rad = ahrs->roll_rad;
    float pitch_rad = ahrs->pitch_rad;
    
    float center_x = SCREEN_WIDTH_PIXELS * 0.5f;
    float center_y = SCREEN_HEIGHT_PIXELS * 0.5f;

    float line_length = HORIZON_LINE_LENGTH_PIXELS;
    float yaw_offset_deg = (pitch_rad * 57.295779513f) * (SCREEN_HEIGHT_PIXELS / HUD_FOV_DEG);
    
    float left_x = center_x - (line_length * 0.5f) * cosf(roll_rad);
    float left_y = center_y + yaw_offset_deg - (line_length * 0.5f) * sinf(roll_rad);
    float right_x = center_x + (line_length * 0.5f) * cosf(roll_rad);
    float right_y = center_y + yaw_offset_deg + (line_length * 0.5f) * sinf(roll_rad);

    if (*count < 64) {
        elements[*count].x_px = left_x;
        elements[*count].y_px = left_y;
        elements[*count].z_depth_m = 1000.0f;
        elements[*count].intensity = 0.8f;
        elements[*count].type = SYM_HORIZON_LINE;
        elements[*count].is_valid = true;
        (*count)++;
    }
    
    if (*count < 64) {
        elements[*count].x_px = right_x;
        elements[*count].y_px = right_y;
        elements[*count].z_depth_m = 1000.0f;
        elements[*count].intensity = 0.8f;
        elements[*count].type = SYM_HORIZON_LINE;
        elements[*count].is_valid = true;
        (*count)++;
    }
}

static void generate_target_symbology(sa_picture_t *sa, ahrs_state_t *ahrs, display_element_t *elements, uint8_t *count) {
    float rot_matrix[9];
    compute_body_to_display_rotation(ahrs->pitch_rad, ahrs->roll_rad, 
                                     pvi_buffer.pilot_head_azimuth * 0.01745329251f, 
                                     pvi_buffer.pilot_head_elevation * 0.01745329251f, 
                                     rot_matrix);

    for (uint8_t i = 0; i < sa->threat_count && *count < 64; i++) {
        correlated_threat_t *threat = &sa->threats[i];
        
        float delta_lat = (threat->lat_dd - sa->ownship_lat_dd) * 111320.0f;
        float delta_lon = (threat->lon_dd - sa->ownship_lon_dd) * 111320.0f * cosf(sa->ownship_lat_dd * 0.01745329251f);
        float delta_alt = threat->altitude_m - sa->ownship_altitude_m;

        float x_world = delta_lon;
        float y_world = delta_alt;
        float z_world = delta_lat;

        float x_body = rot_matrix[0]*x_world + rot_matrix[1]*y_world + rot_matrix[2]*z_world;
        float y_body = rot_matrix[3]*x_world + rot_matrix[4]*y_world + rot_matrix[5]*z_world;
        float z_body = rot_matrix[6]*x_world + rot_matrix[7]*y_world + rot_matrix[8]*z_world;

        float x_px, y_px;
        bool on_screen;
        project_world_to_screen(x_body, y_body, z_body, &x_px, &y_px, &on_screen);

        if (on_screen && z_body > 0.0f && z_body < SYMBOL_MAX_Z_DEPTH_M) {
            elements[*count].x_px = x_px;
            elements[*count].y_px = y_px;
            elements[*count].z_depth_m = z_body;
            elements[*count].intensity = (threat->priority > 70) ? 1.0f : 0.6f;
            elements[*count].type = SYM_TARGET_BRACKET;
            elements[*count].target_id = threat->threat_id;
            elements[*count].is_valid = true;
            (*count)++;
        }
    }
}

void pvi_manager_init(void) {
    memset(&pvi_buffer, 0, sizeof(pvi_frame_t));
    proj_params.fov_scale_x = (float)SCREEN_WIDTH_PIXELS / (2.0f * tanf((HUD_FOV_DEG * 0.5f) * 0.01745329251f));
    proj_params.fov_scale_y = (float)SCREEN_HEIGHT_PIXELS / (2.0f * tanf((HUD_FOV_DEG * 0.5f) * 0.01745329251f));
    proj_params.principal_point_x = (float)SCREEN_WIDTH_PIXELS * 0.5f;
    proj_params.principal_point_y = (float)SCREEN_HEIGHT_PIXELS * 0.5f;
    pvi_buffer.current_brightness = HUD_BRIGHTNESS_MAX;
    pvi_initialized = true;
}

void pvi_manager_cycle(ahrs_state_t *ahrs, nav_output_t *nav, sa_picture_t *sa) {
    if (!pvi_initialized) return;

    pvi_buffer.active_element_count = 0;

    float fpv_pitch_deg, fpv_yaw_deg;
    compute_flight_path_vector(ahrs, nav, &fpv_pitch_deg, &fpv_yaw_deg);

    float rot_matrix[9];
    compute_body_to_display_rotation(ahrs->pitch_rad, ahrs->roll_rad, 0.0f, 0.0f, rot_matrix);

    float fpv_x_body = sinf(fpv_yaw_deg * 0.01745329251f);
    float fpv_y_body = -sinf(fpv_pitch_deg * 0.01745329251f);
    float fpv_z_body = cosf(fpv_pitch_deg * 0.01745329251f) * cosf(fpv_yaw_deg * 0.01745329251f);

    float x_px, y_px;
    bool on_screen;
    project_world_to_screen(fpv_x_body, fpv_y_body, fpv_z_body, &x_px, &y_px, &on_screen);

    if (on_screen && pvi_buffer.active_element_count < 64) {
        pvi_buffer.elements[pvi_buffer.active_element_count].x_px = x_px;
        pvi_buffer.elements[pvi_buffer.active_element_count].y_px = y_px;
        pvi_buffer.elements[pvi_buffer.active_element_count].z_depth_m = 10000.0f;
        pvi_buffer.elements[pvi_buffer.active_element_count].intensity = 1.0f;
        pvi_buffer.elements[pvi_buffer.active_element_count].type = SYM_FLIGHT_PATH_VEC;
        pvi_buffer.elements[pvi_buffer.active_element_count].is_valid = true;
        pvi_buffer.active_element_count++;
    }

    generate_horizon_line(ahrs, pvi_buffer.elements, &pvi_buffer.active_element_count);
    generate_target_symbology(sa, ahrs, pvi_buffer.elements, &pvi_buffer.active_element_count);

    if (pvi_buffer.current_brightness > HUD_BRIGHTNESS_MAX) pvi_buffer.current_brightness = HUD_BRIGHTNESS_MAX;
    if (pvi_buffer.current_brightness < HUD_BRIGHTNESS_MIN) pvi_buffer.current_brightness = HUD_BRIGHTNESS_MIN;
}

void pvi_get_frame(pvi_frame_t *out_frame) {
    memcpy(out_frame, &pvi_buffer, sizeof(pvi_frame_t));
}

void pvi_adjust_brightness(float delta) {
    pvi_buffer.current_brightness += delta;
}

void pvi_set_head_tracking(float azimuth_deg, float elevation_deg) {
    pvi_buffer.pilot_head_azimuth = azimuth_deg;
    pvi_buffer.pilot_head_elevation = elevation_deg;
}
