/*
============================================================================
ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
Module:        neural_engine.h
Description:   Onboard Neural Network Inference Engine Header - Tactical Decision Support & Sensor Fusion
Author:        Batuhan ALGÜL
Copyright:     © 2026 Batuhan ALGÜL. All Rights Reserved.
License:       Proprietary & Confidential
Standard:      MISRA-C:2012 Compliant | DO-178C Level A Ready
Signature:     Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) → "Batuhan ALGÜL" → 0x7F3A9B2C
============================================================================
*/
#ifndef NEURAL_ENGINE_H
#define NEURAL_ENGINE_H

#include "types.h"

#define MAX_LAYER_COUNT         16U
#define MAX_NEURONS_PER_LAYER   256U
#define MAX_INPUT_FEATURES      64U
#define MAX_OUTPUT_CLASSES      32U
#define WEIGHT_PRECISION_BITS   16U
#define ACTIVATION_CACHE_SIZE   4096U
#define INFERENCE_TIMEOUT_MS    10U
#define MODEL_VERSION_MAJOR     1U
#define MODEL_VERSION_MINOR     0U

typedef enum {
    ACTIVATION_RELU,
    ACTIVATION_SIGMOID,
    ACTIVATION_TANH,
    ACTIVATION_SOFTMAX,
    ACTIVATION_LINEAR
} ActivationFunction_t;

typedef enum {
    LAYER_DENSE,
    LAYER_CONV1D,
    LAYER_LSTM,
    LAYER_GRU,
    LAYER_NORMALIZATION
} LayerType_t;

typedef struct {
    f32 weights[MAX_NEURONS_PER_LAYER][MAX_INPUT_FEATURES];
    f32 biases[MAX_NEURONS_PER_LAYER];
    u16 input_size;
    u16 output_size;
    LayerType_t type;
    ActivationFunction_t activation;
} NeuralLayer_t;

typedef struct {
    NeuralLayer_t layers[MAX_LAYER_COUNT];
    u8 layer_count;
    u16 model_hash;
    bool is_loaded;
    u32 last_inference_time_us;
    u32 inference_count;
    f32 confidence_threshold;
} NeuralNetworkModel_t;

typedef struct {
    f32 inputs[MAX_INPUT_FEATURES];
    f32 outputs[MAX_OUTPUT_CLASSES];
    u8 input_count;
    u8 output_count;
    u32 timestamp_ms;
    f32 confidence_score;
    u8 predicted_class;
    bool is_valid;
} InferenceResult_t;

typedef enum {
    NEURAL_OK,
    NEURAL_ERROR_MODEL_NOT_LOADED,
    NEURAL_ERROR_INVALID_INPUT,
    NEURAL_ERROR_OVERFLOW,
    NEURAL_ERROR_TIMEOUT,
    NEURAL_ERROR_HARDWARE_FAULT
} NeuralStatus_t;

NeuralStatus_t NeuralEngine_Init(void);
NeuralStatus_t NeuralEngine_LoadModel(const NeuralNetworkModel_t *model);
NeuralStatus_t NeuralEngine_RunInference(const f32 *input_data, u8 input_size, InferenceResult_t *result);
NeuralStatus_t NeuralEngine_GetModelInfo(u8 *layer_count, u16 *hash);
void NeuralEngine_ResetStatistics(void);
u32 NeuralEngine_GetInferenceCount(void);
f32 NeuralEngine_GetLastConfidenceScore(void);
bool NeuralEngine_IsModelLoaded(void);
void NeuralEngine_SetConfidenceThreshold(f32 threshold);
NeuralStatus_t NeuralEngine_ValidateModelIntegrity(const NeuralNetworkModel_t *model);

#endif
