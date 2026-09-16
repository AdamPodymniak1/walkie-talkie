#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <driver/i2s.h>
#include <mbedtls/aes.h>
#include <math.h>
#include <LittleFS.h>  // Added filesystem library

// =====================================================
// PIN DEFINITIONS
// =====================================================
#define SWITCH_PIN 14
#define LED_PIN 2
#define CE_PIN 4
#define CSN_PIN 5

// ADC Microphone Pin
#define MIC_ADC_CHAN ADC1_CHANNEL_7 

// I2S PCM5102 DAC Pins
#define I2S_BCLK_PIN 26
#define I2S_LRCK_PIN 25
#define I2S_DOUT_PIN 22

// =====================================================
// HIGH FIDELITY AUDIO SETTINGS
// =====================================================
#define SAMPLE_RATE 24000     
#define AUDIO_FRAME_SIZE 64   
#define PACKET_SIZE 32        

RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "AUDIO";

const uint8_t CRYPTO_KEY[32] = {
    0x4F, 0x8A, 0x13, 0x7C, 0xD2, 0x91, 0xEA, 0xB5, 
    0x36, 0xFC, 0x08, 0x22, 0xBB, 0x64, 0x97, 0x50, 
    0x11, 0xAA, 0xDF, 0x88, 0x43, 0x75, 0x2C, 0x9E, 
    0x6B, 0x01, 0xDD, 0x3F, 0x54, 0x89, 0xCE, 0x77
};

mbedtls_aes_context aes_ctx_tx;
mbedtls_aes_context aes_ctx_rx;

volatile bool sendMode = false;
bool lastMode = false;
float dc_offset = 2048.0f;

// =====================================================
// AUDIO STREAMING ENGINE (MIXED TONE & FILE)
// =====================================================

// Keeps your loved TX/RX tones running perfectly
void playToneAlert(float frequency, int duration_ms) {
    int16_t tone_buf[AUDIO_FRAME_SIZE * 2];
    size_t bytesWritten;
    float phase = 0;
    float phase_inc = 2.0f * PI * frequency / SAMPLE_RATE;
    
    int total_frames = (SAMPLE_RATE * duration_ms) / (AUDIO_FRAME_SIZE * 1000);
    if (total_frames < 1) total_frames = 1;

    for (int i = 0; i < total_frames; i++) {
        int buf_idx = 0;
        for (int j = 0; j < AUDIO_FRAME_SIZE; j++) {
            int16_t sample = (int16_t)(sin(phase) * 7000.0f); 
            phase += phase_inc;
            if (phase >= 2.0f * PI) phase -= 2.0f * PI;
            
            tone_buf[buf_idx++] = sample; // Left
            tone_buf[buf_idx++] = sample; // Right
        }
        i2s_write(I2S_NUM_1, tone_buf, sizeof(tone_buf), &bytesWritten, portMAX_DELAY);
    }
}

// Low-RAM Streamer for reading raw files out of the data folder
void playAudioFile(const char* path) {
    File file = LittleFS.open(path, "r");
    if (!file) {
        Serial.print("❌ File system error: Cannot open ");
        Serial.println(path);
        return;
    }

    int16_t mono_buf[AUDIO_FRAME_SIZE];
    int16_t stereo_buf[AUDIO_FRAME_SIZE * 2];
    size_t bytes_written;

    // Read file in small fragments to protect RAM stability
    while (file.available()) {
        size_t bytesRead = file.read((uint8_t*)mono_buf, sizeof(mono_buf));
        int samples_read = bytesRead / 2;

        int buf_idx = 0;
        for (int i = 0; i < samples_read; i++) {
            stereo_buf[buf_idx++] = mono_buf[i]; // Convert Mono to Stereo Left
            stereo_buf[buf_idx++] = mono_buf[i]; // Convert Mono to Stereo Right
        }
        i2s_write(I2S_NUM_1, stereo_buf, samples_read * 4, &bytes_written, portMAX_DELAY);
    }
    file.close();
}

// Unified Context Router
void playAudioCommunicate(const char* type) {
    if (strcmp(type, "BOOT") == 0) {
        Serial.println("[AUDIO] Playing boot.raw from data folder...");
        playAudioFile("/boot.raw");
    } 
    else if (strcmp(type, "PASS") == 0) {
        Serial.println("[AUDIO] Playing pass.raw from data folder...");
        playAudioFile("/pass.raw");
    } 
    else if (strcmp(type, "FAIL") == 0) {
        Serial.println("[AUDIO] Playing fail.raw from data folder...");
        playAudioFile("/fail.raw");
    } 
    else if (strcmp(type, "TX_START") == 0) {
        // Kept as original custom tone alert
        playToneAlert(880.0f, 80);
        playToneAlert(1174.66f, 120);
    } 
    else if (strcmp(type, "RX_START") == 0) {
        // Kept as original custom tone alert
        playToneAlert(587.33f, 80);
        playToneAlert(440.0f, 120);
    }
}

// =====================================================
// IMA ADPCM CODEC TABLES & STATE
// =====================================================
const int8_t IndexTable[16] = { -1, -1, -1, -1, 2, 4, 6, 8, -1, -1, -1, -1, 2, 4, 6, 8 };

const uint16_t StepSizeTable[89] = {
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 23, 25, 28, 31, 34, 37, 41, 45,
    50, 55, 60, 66, 73, 80, 88, 97, 107, 118, 130, 143, 157, 173, 190, 209, 230,
    253, 279, 307, 337, 371, 408, 449, 494, 544, 598, 658, 724, 796, 876, 963,
    1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066, 2272, 2499, 2749, 3024, 3327,
    3660, 4026, 4428, 4871, 5358, 5894, 6484, 7132, 7845, 8630, 9493, 10442, 11487,
    12635, 13899, 15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767
};

struct ADPCMState { int32_t prev_val; int8_t index; };
ADPCMState tx_state = {0, 0};
ADPCMState rx_state = {0, 0};

uint8_t adpcm_encode(int16_t sample, ADPCMState* state) {
    int32_t diff = sample - state->prev_val;
    uint8_t nibble = 0;
    int32_t step = StepSizeTable[state->index];
    int32_t vpdiff = step >> 3;

    if (diff < 0) { nibble = 8; diff = -diff; }
    if (diff >= step) { nibble |= 4; diff -= step; vpdiff += step; }
    step >>= 1;
    if (diff >= step) { nibble |= 2; diff -= step; vpdiff += step; }
    step >>= 1;
    if (diff >= step) { nibble |= 1; vpdiff += step; }

    if (nibble & 8) state->prev_val -= vpdiff; else state->prev_val += vpdiff;

    if (state->prev_val > 32767) state->prev_val = 32767;
    else if (state->prev_val < -32768) state->prev_val = -32768;

    state->index += IndexTable[nibble];
    if (state->index < 0) state->index = 0;
    else if (state->index > 88) state->index = 88;

    return nibble;
}

int16_t adpcm_decode(uint8_t nibble, ADPCMState* state) {
    int32_t step = StepSizeTable[state->index];
    int32_t vpdiff = step >> 3;

    if (nibble & 4) vpdiff += step;
    if (nibble & 2) vpdiff += step >> 1;
    if (nibble & 1) vpdiff += step >> 2;

    if (nibble & 8) state->prev_val -= vpdiff; else state->prev_val += vpdiff;
    state->prev_val -= (state->prev_val >> 6);

    if (state->prev_val > 32767) state->prev_val = 32767;
    else if (state->prev_val < -32768) state->prev_val = -32768;

    state->index += IndexTable[nibble];
    if (state->index < 0) state->index = 0;
    else if (state->index > 88) state->index = 88;

    return (int16_t)state->prev_val;
}

// =====================================================
// AUDIO HARDWARE CONFIGURATION
// =====================================================
void setupI2S() {
    i2s_config_t i2s_config_adc = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = AUDIO_FRAME_SIZE,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };
    i2s_driver_install(I2S_NUM_0, &i2s_config_adc, 0, NULL);
    i2s_set_adc_mode(ADC_UNIT_1, MIC_ADC_CHAN);
    analogReadResolution(12);
    analogSetPinAttenuation(35, ADC_11db);

    i2s_config_t i2s_config_dac = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, 
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,  
        .dma_buf_len = AUDIO_FRAME_SIZE,
        .use_apll = true,    
        .tx_desc_auto_clear = true, 
        .fixed_mclk = 0
    };
    i2s_pin_config_t pin_config_dac = {
        .bck_io_num = I2S_BCLK_PIN, .ws_io_num = I2S_LRCK_PIN,
        .data_out_num = I2S_DOUT_PIN, .data_in_num = I2S_PIN_NO_CHANGE 
    };
    i2s_driver_install(I2S_NUM_1, &i2s_config_dac, 0, NULL);
    i2s_set_pin(I2S_NUM_1, &pin_config_dac);
}

void printLine() { Serial.println("--------------------------------------------------"); }

// =====================================================
// SETUP & HARDWARE INTERROGATION
// =====================================================
void setup() {
    Serial.begin(115200);
    delay(2000); 
    pinMode(SWITCH_PIN, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);
    
    printLine();
    Serial.println("SYSTEM DIAGNOSTICS & COMMS CHECK RUNNING...");
    printLine();

    // Initialize LittleFS Partition
    Serial.println("[0] Initializing LittleFS Flash Partition...");
    if (!LittleFS.begin(true)) {
        Serial.println("❌ CRITICAL ERROR: LittleFS Mount Failed!");
        while(1);
    }
    Serial.println("✅ LittleFS Storage Mounted Successfully.");

    Serial.println("[1] Deploying I2S Audio Drivers...");
    setupI2S();
    Serial.println("✅ I2S Driver Channels Online.");

    // ---------------------------------------------------
    // 1. NRF24L01 TRANSCEIVER VERIFICATION
    // ---------------------------------------------------
    SPI.begin(18, 19, 23, CSN_PIN);
    Serial.println("[2] Initiating NRF24 System Core...");

    if (!radio.begin()) {
        Serial.println("❌ CRITICAL ERROR: RADIO HARDWARE OFFLINE");
        playAudioCommunicate("FAIL");
        while (1); 
    }
    Serial.println("✅ SPI Bus Connection Validated.");

    radio.setAutoAck(false); radio.disableCRC(); radio.setRetries(0, 0);
    radio.setPALevel(RF24_PA_MAX); radio.setDataRate(RF24_250KBPS);

    Serial.println("[3] Performing RF Channel Register Interrogation...");
    bool registerOK = true;
    for (int ch = 0; ch <= 125; ch += 25) {
        radio.setChannel(ch); delay(10);
        if (radio.getChannel() != ch) registerOK = false;
    }
    
    if (registerOK) {
        Serial.println("✅ Hex-Register Read/Write Stability Passed.");
    } else {
        Serial.println("❌ ERROR: Hex-Register Mismatch!");
        playAudioCommunicate("FAIL");
    }

    Serial.println("[4] Analyzing Ambient RF Energy Environments...");
    int detections = 0;
    for (int ch = 0; ch <= 125; ch++) {
        radio.setChannel(ch);
        for (int i = 0; i < 20; i++) {
            radio.startListening(); delayMicroseconds(200);
            if (radio.testRPD()) detections++;
            radio.stopListening();
        }
    }
    if (detections == 0) {
        Serial.println("⚠ WARNING: Zero RF Ambient Energy Detected.");
    } else {
        Serial.println("✅ RF Receiver Array Operational.");
    }

    // ---------------------------------------------------
    // 2. ANALOG MICROPHONE SAMPLING TEST
    // ---------------------------------------------------
    Serial.println("[5] Interrogating ADC Built-In Microphone...");
    uint16_t test_buf[AUDIO_FRAME_SIZE];
    size_t bytesRead = 0;
    uint16_t min_val = 4096; uint16_t max_val = 0;
    
    i2s_adc_enable(I2S_NUM_0);
    for(int i = 0; i < 40; i++) {
        i2s_read(I2S_NUM_0, test_buf, AUDIO_FRAME_SIZE * 2, &bytesRead, portMAX_DELAY);
        for(int j = 0; j < AUDIO_FRAME_SIZE; j++) {
            uint16_t val = test_buf[j] & 0x0FFF;
            if(val < min_val) min_val = val;
            if(val > max_val) max_val = val;
        }
    }
    i2s_adc_disable(I2S_NUM_0);
    
    int mic_spread = max_val - min_val;
    if (mic_spread < 20) {
        Serial.println("❌ ERROR: Microphone Waveform is Flatlined.");
        playAudioCommunicate("FAIL");
    } else {
        Serial.println("✅ Microphone Waveform Detected.");
        playAudioCommunicate("PASS");
    }

    printLine();
    Serial.println(">>> HARDWARE OK. SYSTEM READY FOR INTERCOM OPERATIONAL MODE <<<");
    printLine();
    
    playAudioCommunicate("BOOT"); // Success Boot Sound File Played

    // ---------------------------------------------------
    // 3. FINAL COMPRESSION & CRYPTO PARAMETER PACKS
    // ---------------------------------------------------
    mbedtls_aes_init(&aes_ctx_tx); mbedtls_aes_init(&aes_ctx_rx);
    mbedtls_aes_setkey_enc(&aes_ctx_tx, CRYPTO_KEY, 256);
    mbedtls_aes_setkey_dec(&aes_ctx_rx, CRYPTO_KEY, 256);

    radio.setChannel(120); radio.setDataRate(RF24_2MBPS);
    radio.setPALevel(RF24_PA_MAX); radio.setAutoAck(false);
    radio.setPayloadSize(PACKET_SIZE);
    radio.openWritingPipe(address); radio.openReadingPipe(1, address);
    radio.startListening();
}

// =====================================================
// LOOP (Normal Operation)
// =====================================================
void loop() {
    sendMode = (digitalRead(SWITCH_PIN) == LOW);

    // Context Change Trigger Engine
    if (sendMode != lastMode) {
        if (sendMode) {
            Serial.println("\r\n[COMM] >> TRANSMIT REQUEST DETECTED. CHANGING STATE...");
            playAudioCommunicate("TX_START"); // Stays as tone alert
            
            radio.stopListening();
            i2s_zero_dma_buffer(I2S_NUM_1); 
            i2s_adc_enable(I2S_NUM_0);
            tx_state.prev_val = 0; tx_state.index = 0;
            dc_offset = 2048.0f; 
            Serial.println("[SYSTEM] Channel Open. Audio Stream Broadcasting Encrypted...");
        } else {
            Serial.println("\r\n[COMM] << TRANSMIT LEAVE REQUEST DETECTED. CHANGING STATE...");
            i2s_adc_disable(I2S_NUM_0);
            
            playAudioCommunicate("RX_START"); // Stays as tone alert

            radio.txStandBy(); delay(5); 
            rx_state.prev_val = 0; rx_state.index = 0;
            radio.startListening();
            Serial.println("[SYSTEM] RX Channel Open. Passive Monitor Listening Mode Active...");
        }
        digitalWrite(LED_PIN, sendMode);
        lastMode = sendMode;
    }

    if (sendMode) {
        // --- TRANSMIT PROCESSING ---
        uint16_t raw[AUDIO_FRAME_SIZE];
        size_t bytesRead = 0;
        i2s_read(I2S_NUM_0, raw, AUDIO_FRAME_SIZE * 2, &bytesRead, portMAX_DELAY);

        uint8_t pt[PACKET_SIZE];
        uint8_t tx[PACKET_SIZE];

        for (int i = 0; i < PACKET_SIZE; i++) {
            uint16_t raw1 = raw[i * 2] & 0x0FFF;
            uint16_t raw2 = raw[i * 2 + 1] & 0x0FFF;

            dc_offset = 0.99f * dc_offset + 0.01f * raw1; 
            int32_t s1 = ((int32_t)raw1 - (int32_t)dc_offset) * 8;
            if (s1 > 32767) s1 = 32767; else if (s1 < -32768) s1 = -32768; 

            dc_offset = 0.99f * dc_offset + 0.01f * raw2;
            int32_t s2 = ((int32_t)raw2 - (int32_t)dc_offset) * 8;
            if (s2 > 32767) s2 = 32767; else if (s2 < -32768) s2 = -32768; 

            uint8_t nibble1 = adpcm_encode((int16_t)s1, &tx_state);
            uint8_t nibble2 = adpcm_encode((int16_t)s2, &tx_state);

            pt[i] = nibble1 | (nibble2 << 4);
        }

        mbedtls_aes_crypt_ecb(&aes_ctx_tx, MBEDTLS_AES_ENCRYPT, pt, tx);
        mbedtls_aes_crypt_ecb(&aes_ctx_tx, MBEDTLS_AES_ENCRYPT, pt + 16, tx + 16);

        radio.write(&tx, PACKET_SIZE); 

    } else {
        // --- RECEIVE PROCESSING ---
        uint8_t incoming[PACKET_SIZE];
        uint8_t decrypted[PACKET_SIZE];
        int16_t out_buffer[AUDIO_FRAME_SIZE * 2]; 

        while (radio.available()) {
            radio.read(&incoming, PACKET_SIZE);
            
            mbedtls_aes_crypt_ecb(&aes_ctx_rx, MBEDTLS_AES_DECRYPT, incoming, decrypted);
            mbedtls_aes_crypt_ecb(&aes_ctx_rx, MBEDTLS_AES_DECRYPT, incoming + 16, decrypted + 16);

            int buf_idx = 0;
            for(int i = 0; i < PACKET_SIZE; i++) {
                uint8_t byteVal = decrypted[i];
                uint8_t nibble1 = byteVal & 0x0F;
                uint8_t nibble2 = (byteVal >> 4) & 0x0F;

                int16_t samples[2];
                samples[0] = adpcm_decode(nibble1, &rx_state);
                samples[1] = adpcm_decode(nibble2, &rx_state);

                for(int s = 0; s < 2; s++) {
                    out_buffer[buf_idx++] = samples[s]; // Left
                    out_buffer[buf_idx++] = samples[s]; // Right
                }
            }

            size_t bytes_written = 0;
            i2s_write(I2S_NUM_1, out_buffer, sizeof(out_buffer), &bytes_written, portMAX_DELAY);
        }
    }
}