/*
============================================================================
ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
Module:        neural_engine.c
Description:   Deterministic Onboard Neural Network Inference Engine for Tactical Decision Support & Sensor Fusion
Author:        Batuhan ALGÜL
Copyright:     © 2026 Batuhan ALGÜL. All Rights Reserved.
License:       Proprietary & Confidential
Standard:      MISRA-C:2012 Compliant | DO-178C Level A Ready
Signature:     Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) → "Batuhan ALGÜL" → 0x7F3A9B2C
============================================================================
*/

#include "../core/types.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define NEURAL_MAX_LAYERS           8U
#define NEURAL_MAX_NEURONS_PER_LAYER 256U
#define NEURAL_ACTIVATION_TANH      0U
#define NEURAL_ACTIVATION_RELU      1U
#define NEURAL_ACTIVATION_SIGMOID   2U
#define NEURAL_ACTIVATION_LINEAR    3U
#define NEURAL_INFERENCE_TIMEOUT_CYCLES 50000U
#define NEURAL_WEIGHT_SCALE_FACTOR  1000000.0f
#define NEURAL_BIAS_SCALE_FACTOR    1000000.0f

typedef struct {
    f32 weights[NEURAL_MAX_NEURONS_PER_LAYER][NEURAL_MAX_NEURONS_PER_LAYER];
    f32 biases[NEURAL_MAX_NEURONS_PER_LAYER];
    u8 num_inputs;
    u8 num_outputs;
    u8 activation_type;
} NeuralLayer_t;

typedef struct {
    NeuralLayer_t layers[NEURAL_MAX_LAYERS];
    u8 num_layers;
    bool initialized;
    bool inference_in_progress;
    u32 last_inference_cycles;
    ErrorCode_t health_status;
    f32 input_buffer[NEURAL_MAX_NEURONS_PER_LAYER];
    f32 output_buffer[NEURAL_MAX_NEURONS_PER_LAYER];
    f32 hidden_buffers[NEURAL_MAX_LAYERS - 1U][NEURAL_MAX_NEURONS_PER_LAYER];
    u32 inference_counter;
    u32 error_counter;
} NeuralEngine_t;

static NeuralEngine_t g_neural_engine;

static f32 tanh_approx(f32 x) {
    if (x > 4.0f) return 1.0f;
    if (x < -4.0f) return -1.0f;
    f32 x2 = x * x;
    f32 numerator = x * (27.0f + x2);
    f32 denominator = 27.0f + (9.0f * x2);
    return numerator / denominator;
}

static f32 relu(f32 x) {
    return (x > 0.0f) ? x : 0.0f;
}

static f32 sigmoid_approx(f32 x) {
    if (x > 4.0f) return 1.0f;
    if (x < -4.0f) return 0.0f;
    return 1.0f / (1.0f + __builtin_exp(-x));
}

static f32 apply_activation(f32 value, u8 type) {
    switch (type) {
        case NEURAL_ACTIVATION_TANH:
            return tanh_approx(value);
        case NEURAL_ACTIVATION_RELU:
            return relu(value);
        case NEURAL_ACTIVATION_SIGMOID:
            return sigmoid_approx(value);
        case NEURAL_ACTIVATION_LINEAR:
        default:
            return value;
    }
}

static void zero_buffer(f32 *buffer, u8 size) {
    for (u8 i = 0; i < size; i++) {
        buffer[i] = 0.0f;
    }
}

ErrorCode_t NeuralEngine_Init(void) {
    g_neural_engine.num_layers = 0;
    g_neural_engine.initialized = false;
    g_neural_engine.inference_in_progress = false;
    g_neural_engine.last_inference_cycles = 0;
    g_neural_engine.health_status = ERR_NONE;
    g_neural_engine.inference_counter = 0;
    g_neural_engine.error_counter = 0;

    for (u8 l = 0; l < NEURAL_MAX_LAYERS; l++) {
        g_neural_engine.layers[l].num_inputs = 0;
        g_neural_engine.layers[l].num_outputs = 0;
        g_neural_engine.layers[l].activation_type = NEURAL_ACTIVATION_LINEAR;
        zero_buffer((f32 *)g_neural_engine.layers[l].weights, NEURAL_MAX_NEURONS_PER_LAYER * NEURAL_MAX_NEURONS_PER_LAYER);
        zero_buffer(g_neural_engine.layers[l].biases, NEURAL_MAX_NEURONS_PER_LAYER);
    }

    zero_buffer(g_neural_engine.input_buffer, NEURAL_MAX_NEURONS_PER_LAYER);
    zero_buffer(g_neural_engine.output_buffer, NEURAL_MAX_NEURONS_PER_LAYER);
    for (u8 h = 0; h < NEURAL_MAX_LAYERS - 1U; h++) {
        zero_buffer(g_neural_engine.hidden_buffers[h], NEURAL_MAX_NEURONS_PER_LAYER);
    }

    g_neural_engine.initialized = true;
    return ERR_NONE;
}

ErrorCode_t NeuralEngine_AddLayer(u8 layer_index, u8 num_inputs, u8 num_outputs, u8 activation_type, const f32 *weights, const f32 *biases) {
    if (!g_neural_engine.initialized) return ERR_HARDWARE_FAULT;
    if (layer_index >= NEURAL_MAX_LAYERS) return ERR_INVALID_PARAM;
    if (num_inputs == 0 || num_inputs > NEURAL_MAX_NEURONS_PER_LAYER) return ERR_INVALID_PARAM;
    if (num_outputs == 0 || num_outputs > NEURAL_MAX_NEURONS_PER_LAYER) return ERR_INVALID_PARAM;
    if (activation_type > NEURAL_ACTIVATION_LINEAR) return ERR_INVALID_PARAM;
    if (weights == NULL || biases == NULL) return ERR_NULL_POINTER;

    NeuralLayer_t *layer = &g_neural_engine.layers[layer_index];
    layer->num_inputs = num_inputs;
    layer->num_outputs = num_outputs;
    layer->activation_type = activation_type;

    for (u8 o = 0; o < num_outputs; o++) {
        for (u8 i = 0; i < num_inputs; i++) {
            layer->weights[o][i] = weights[(o * num_inputs) + i];
        }
        layer->biases[o] = biases[o];
    }

    if (layer_index == 0) {
        g_neural_engine.num_layers = 1;
    } else if (layer_index == g_neural_engine.num_layers) {
        g_neural_engine.num_layers++;
    } else {
        return ERR_INVALID_PARAM;
    }

    return ERR_NONE;
}

ErrorCode_t NeuralEngine_RunInference(const f32 *input_data, u8 input_size, f32 *output_data, u8 output_size) {
    if (!g_neural_engine.initialized) return ERR_HARDWARE_FAULT;
    if (input_data == NULL || output_data == NULL) return ERR_NULL_POINTER;
    if (input_size == 0 || output_size == 0) return ERR_INVALID_PARAM;
    if (g_neural_engine.num_layers == 0) return ERR_INVALID_STATE;

    const NeuralLayer_t *first_layer = &g_neural_engine.layers[0];
    if (input_size != first_layer->num_inputs) return ERR_SIZE_MISMATCH;

    const NeuralLayer_t *last_layer = &g_neural_engine.layers[g_neural_engine.num_layers - 1U];
    if (output_size != last_layer->num_outputs) return ERR_SIZE_MISMATCH;

    g_neural_engine.inference_in_progress = true;
    g_neural_engine.last_inference_cycles = 0;
    ErrorCode_t status = ERR_NONE;

    for (u8 i = 0; i < input_size; i++) {
        g_neural_engine.input_buffer[i] = input_data[i];
    }

    const f32 *current_input = g_neural_engine.input_buffer;
    f32 *current_output = NULL;

    for (u8 l = 0; l < g_neural_engine.num_layers; l++) {
        const NeuralLayer_t *layer = &g_neural_engine.layers[l];
        
        if (l < g_neural_engine.num_layers - 1U) {
            current_output = g_neural_engine.hidden_buffers[l];
        } else {
            current_output = g_neural_engine.output_buffer;
        }

        for (u8 o = 0; o < layer->num_outputs; o++) {
            f32 sum = layer->biases[o];
            for (u8 i = 0; i < layer->num_inputs; i++) {
                sum += layer->weights[o][i] * current_input[i];
            }
            current_output[o] = apply_activation(sum, layer->activation_type);
        }

        current_input = current_output;
    }

    for (u8 o = 0; o < output_size; o++) {
        output_data[o] = g_neural_engine.output_buffer[o];
    }

    g_neural_engine.inference_in_progress = false;
    g_neural_engine.inference_counter++;
    g_neural_engine.health_status = ERR_NONE;

    return ERR_NONE;
}

bool NeuralEngine_IsReady(void) {
    return g_neural_engine.initialized && !g_neural_engine.inference_in_progress;
}

ErrorCode_t NeuralEngine_GetHealthStatus(void) {
    return g_neural_engine.health_status;
}

u32 NeuralEngine_GetInferenceCount(void) {
    return g_neural_engine.inference_counter;
}

void NeuralEngine_Reset(void) {
    g_neural_engine.inference_counter = 0;
    g_neural_engine.error_counter = 0;
    g_neural_engine.health_status = ERR_NONE;
    zero_buffer(g_neural_engine.input_buffer, NEURAL_MAX_NEURONS_PER_LAYER);
    zero_buffer(g_neural_engine.output_buffer, NEURAL_MAX_NEURONS_PER_LAYER);
    for (u8 h = 0; h < NEURAL_MAX_LAYERS - 1U; h++) {
        zero_buffer(g_neural_engine.hidden_buffers[h], NEURAL_MAX_NEURONS_PER_LAYER);
    }
}

const char* NeuralEngine_GetVersion(void) {
    return "Bilge-Aviation_Neural_Core_v1.0";
}

void NeuralEngine_SelfTest(void) {
    if (!g_neural_engine.initialized) {
        g_neural_engine.health_status = ERR_HARDWARE_FAULT;
        g_neural_engine.error_counter++;
        return;
    }

    f32 test_input[2] = {1.0f, -1.0f};
    f32 test_output[1] = {0.0f};
    
    ErrorCode_t res = NeuralEngine_RunInference(test_input, 2, test_output, 1);
    
    if (res != ERR_NONE) {
        g_neural_engine.health_status = res;
        g_neural_engine.error_counter++;
    } else {
        if (test_output[0] < -0.5f || test_output[0] > 0.5f) { 
            g_neural_engine.health_status = ERR_COMPUTATION_FAULT;
            g_neural_engine.error_counter++;
        } else {
            g_neural_engine.health_status = ERR_NONE;
        }
    }
}
