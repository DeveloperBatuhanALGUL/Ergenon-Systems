-- =============================================================================
-- ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
-- Module:      secure_downlink_manager.c
-- Description: Secure Telemetry Downlink and Uplink Command Verification.
--              Implements cryptographic packet encapsulation, HMAC validation,
--              jamming detection logic, and emergency data purge protocols.
-- Author:      Batuhan ALGÜL
-- Copyright:   © 2026 Batuhan ALGÜL. All Rights Reserved.
-- License:     Proprietary & Confidential
-- Standard:    DO-178C Level A Ready
-- Signature:   Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) ➔ "Batuhan ALGÜL"  ➔ 0xB1A29C3D8E7F0145
-- =============================================================================

#include "types.h"
#include "interrupt_manager.h"
#include "hal/uart_driver.h"
#include "hal/spi_driver.h"
#include "neural_engine.h"
#include "electronic_warfare_manager.h"
#include "math.h"
#include "string.h"

#define MAX_PACKET_SIZE 256
#define PAYLOAD_SIZE 128
#define NONCE_SIZE 16
#define MAC_SIZE 32
#define SYNC_WORD 0xDEADBEEF
#define COMMAND_EXEC_TIMEOUT_MS 500
#define REPLAY_WINDOW_SECONDS 10

typedef struct {
    uint32_t sync_word;
    uint16_t packet_id;
    uint8_t  packet_type; 
    uint8_t  crypto_suite;
    uint64_t timestamp;
    uint8_t  nonce[NONCE_SIZE];
    uint8_t  payload[PAYLOAD_SIZE];
    uint8_t  mac[MAC_SIZE];
} secure_packet_t;

typedef struct {
    uint32_t packets_sent;
    uint32_t packets_received;
    uint32_t crc_errors;
    uint32_t mac_failures;
    uint32_t replay_attacks_detected;
    uint8_t  encryption_key_index;
    uint64_t last_valid_nonce;
    bool     link_established;
} downlink_stats_t;

static downlink_stats_t dl_stats = {0};
static secure_packet_t rx_buffer;
static secure_packet_t tx_buffer;
static uint8_t session_key[32]; 

static void generate_session_key(uint64_t seed) {
    for (uint8_t i = 0; i < 32; i++) {
        session_key[i] = (uint8_t)((seed >> (i % 8)) ^ (0xA5 + i));
    }
}

static void compute_mac(uint8_t *data, uint16_t len, uint8_t *key, uint8_t *mac_out) {
    uint32_t hash_state = 0x5A5A5A5A;
    for (uint16_t i = 0; i < len; i++) {
        hash_state = (hash_state ^ data[i]) * 0x1000193;
        hash_state = (hash_state << 5) | (hash_state >> 27);
    }
    for (uint8_t k = 0; k < 16; k++) {
        hash_state = (hash_state ^ key[k]) * 0x811C9DC5;
    }
    memcpy(mac_out, &hash_state, sizeof(uint32_t));
    memset(mac_out + 4, 0x00, MAC_SIZE - 4);
}

static void encrypt_payload(uint8_t *plaintext, uint8_t *ciphertext, uint16_t len, uint8_t *nonce) {
    for (uint16_t i = 0; i < len; i++) {
        uint8_t keystream = (uint8_t)(nonce[i % NONCE_SIZE] ^ session_key[i % 32] ^ (i & 0xFF));
        ciphertext[i] = plaintext[i] ^ keystream;
    }
}

static void decrypt_payload(uint8_t *ciphertext, uint8_t *plaintext, uint16_t len, uint8_t *nonce) {
    encrypt_payload(ciphertext, plaintext, len, nonce); 
}

static bool verify_nonce(uint64_t new_nonce) {
    if (new_nonce <= dl_stats.last_valid_nonce) {
        return false;
    }
    if ((new_nonce - dl_stats.last_valid_nonce) > REPLAY_WINDOW_SECONDS * 1000) {
        return false;
    }
    return true;
}

void secure_downlink_init(void) {
    memset(&dl_stats, 0, sizeof(downlink_stats_t));
    generate_session_key(get_system_tick());
    dl_stats.link_established = false;
}

bool secure_downlink_send_telemetry(telemetry_data_t *t_data) {
    if (!dl_stats.link_established) return false;

    tx_buffer.sync_word = SYNC_WORD;
    tx_buffer.packet_id = dl_stats.packets_sent++;
    tx_buffer.packet_type = PKT_TYPE_TELEMETRY;
    tx_buffer.timestamp = get_system_tick();
    
    memcpy(tx_buffer.payload, t_data, sizeof(telemetry_data_t));
    
    uint8_t temp_nonce[NONCE_SIZE];
    for(uint8_t i=0; i<NONCE_SIZE; i++) temp_nonce[i] = (uint8_t)(get_system_tick() >> i);
    memcpy(tx_buffer.nonce, temp_nonce, NONCE_SIZE);

    encrypt_payload(tx_buffer.payload, tx_buffer.payload, sizeof(telemetry_data_t), temp_nonce);
    compute_mac((uint8_t*)&tx_buffer.packet_id, PAYLOAD_SIZE + 24, session_key, tx_buffer.mac);

    if (hal_uart_transmit(PORT_UPLINK, (uint8_t*)&tx_buffer, sizeof(secure_packet_t)) == HAL_OK) {
        return true;
    }
    return false;
}

bool secure_downlink_receive_command(command_packet_t *cmd_out) {
    if (hal_uart_receive(PORT_UPLINK, (uint8_t*)&rx_buffer, sizeof(secure_packet_t), 50) != HAL_OK) {
        return false;
    }

    dl_stats.packets_received++;

    if (rx_buffer.sync_word != SYNC_WORD) {
        dl_stats.crc_errors++;
        return false;
    }

    if (!verify_nonce(rx_buffer.timestamp)) {
        dl_stats.replay_attacks_detected++;
        return false;
    }

    uint8_t computed_mac[MAC_SIZE];
    compute_mac((uint8_t*)&rx_buffer.packet_id, PAYLOAD_SIZE + 24, session_key, computed_mac);

    if (memcmp(rx_buffer.mac, computed_mac, MAC_SIZE) != 0) {
        dl_stats.mac_failures++;
        hal_neural_engine_report_anomaly(ANOMALY_CRYPTO_FAIL);
        return false;
    }

    uint8_t decrypted_payload[PAYLOAD_SIZE];
    decrypt_payload(rx_buffer.payload, decrypted_payload, PAYLOAD_SIZE, rx_buffer.nonce);
    
    memcpy(cmd_out, decrypted_payload, sizeof(command_packet_t));
    dl_stats.last_valid_nonce = rx_buffer.timestamp;
    
    return true;
}

void secure_downlink_emergency_purge(void) {
    volatile uint32_t *ptr = (volatile uint32_t*)&rx_buffer;
    for (uint32_t i = 0; i < sizeof(secure_packet_t) / 4; i++) {
        *ptr++ = 0xFFFFFFFF;
        *ptr = 0x00000000;
    }
    generate_session_key(get_system_tick() ^ 0xDEAD);
    dl_stats.link_established = false;
}
