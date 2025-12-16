/**
 * Embedded Challenge Fall 2025 - "Shake, Rattle, and Roll..and Freeze"
 * Parkinson's Disease Symptom Detection System
 * 
 * Platform: B-L475E-IOT01A Discovery Kit
 * Framework: Mbed OS (PlatformIO)
 * 
 * Direct I2C communication with LSM6DSL sensor
 */

#include "mbed.h"
#include <cmath>
#include <cstring>
#include "ble/BLE.h"
#include "ble/Gap.h"
#include "ble/GattServer.h"

#define LSM6DSL_ADDR            (0x6A << 1)
#define LSM6DSL_WHO_AM_I        0x0F
#define LSM6DSL_WHO_AM_I_VALUE  0x6A
#define LSM6DSL_CTRL1_XL        0x10
#define LSM6DSL_CTRL2_G         0x11
#define LSM6DSL_CTRL3_C         0x12
#define LSM6DSL_OUTX_L_XL       0x28
#define LSM6DSL_OUTX_L_G        0x22

#define SAMPLE_RATE_HZ      52
#define SAMPLE_PERIOD_MS    (1000 / SAMPLE_RATE_HZ)
#define WINDOW_SECONDS      3
#define NUM_SAMPLES         (SAMPLE_RATE_HZ * WINDOW_SECONDS)
#define FFT_SIZE            256

#define TREMOR_LOW_HZ       3.0f
#define TREMOR_HIGH_HZ      5.0f  
#define DYSKINESIA_LOW_HZ   5.0f     
#define DYSKINESIA_HIGH_HZ  7.0f 

#define TREMOR_THRESHOLD        12.0f
#define DYSKINESIA_THRESHOLD    15.0f
#define WALKING_THRESHOLD       0.01f
#define FOG_STILLNESS_THRESHOLD 0.003f
#define WALKING_DURATION_MS     2000

I2C i2c(PB_11, PB_10);

DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);

InterruptIn userButton(BUTTON1);

BufferedSerial pc(USBTX, USBRX, 115200);

struct DetectionResult {
    uint8_t detected;
    uint8_t intensity;
    float dominantFreq;
    float power;
};

#pragma pack(push, 1)
struct TremorCharData {
    uint8_t detected;
    uint8_t intensity;
    uint16_t freqX100;
    uint16_t power;
};

struct DyskinesiaCharData {
    uint8_t detected;
    uint8_t intensity;
    uint16_t freqX100;
    uint16_t power;
};

struct FOGCharData {
    uint8_t detected;
    uint8_t intensity;
    uint16_t reserved;
};
#pragma pack(pop)

static float accMagnitude[NUM_SAMPLES];
static float fftInput[FFT_SIZE];
static float fftMagnitude[FFT_SIZE / 2];

static DetectionResult tremorResult = {0, 0, 0.0f, 0.0f};
static DetectionResult dyskinesiaResult = {0, 0, 0.0f, 0.0f};
static DetectionResult fogResult = {0, 0, 0.0f, 0.0f};

static volatile bool walkingActive = false;

static Timer walkingTimer;
static uint32_t walkingDuration = 0;

static volatile bool systemRunning = true;
static volatile bool buttonPressed = false;
static volatile bool bleResetRequested = false;

static Timer buttonPressTimer;
static volatile bool buttonPressActive = false;

static const char DEVICE_NAME[] = "PD-Detector";

static const UUID PD_SERVICE_UUID(0x1810);
static const UUID TREMOR_CHAR_UUID(0x1910);
static const UUID DYSKINESIA_CHAR_UUID(0x1911);
static const UUID FOG_CHAR_UUID(0x1912);

static BLE &bleInstance = BLE::Instance();
static bool bleConnected = false;
static bool bleInitialized = false;
static GattCharacteristic *tremorCharPtr = nullptr;
static GattCharacteristic *dyskinesiaCharPtr = nullptr;
static GattCharacteristic *fogCharPtr = nullptr;
static events::EventQueue eventQueue(16 * EVENTS_EVENT_SIZE);

static TremorCharData tremorCharData = {0, 0, 0, 0};
static DyskinesiaCharData dyskinesiaCharData = {0, 0, 0, 0};
static FOGCharData fogCharData = {0, 0, 0};

FileHandle *mbed::mbed_override_console(int fd) {
    return &pc;
}

void onBleInitComplete(BLE::InitializationCompleteCallbackContext *context) {
    if (context->error != BLE_ERROR_NONE) {
        printf("BLE init failed with error %d\r\n", context->error);
        return;
    }
    bleInitialized = true;
    printf("BLE initialized successfully\r\n");
}

void onConnectionComplete(const ble::ConnectionCompleteEvent &event) {
    if (event.getStatus() == BLE_ERROR_NONE) {
        bleConnected = true;
        printf("BLE: Device connected\r\n");
    }
}

void onDisconnection(const ble::DisconnectionCompleteEvent &event) {
    bleConnected = false;
    printf("BLE: Device disconnected, restarting advertising\r\n");
    bleInstance.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);
}

class GapEventHandler : public ble::Gap::EventHandler {
public:
    void onConnectionComplete(const ble::ConnectionCompleteEvent &event) override {
        ::onConnectionComplete(event);
    }
    
    void onDisconnectionComplete(const ble::DisconnectionCompleteEvent &event) override {
        ::onDisconnection(event);
    }
};

static GapEventHandler gapEventHandler;

void updateBLECharacteristics() {
    if (!bleConnected) return;
    
    tremorCharData.detected = tremorResult.detected;
    tremorCharData.intensity = tremorResult.intensity;
    tremorCharData.freqX100 = (uint16_t)(tremorResult.dominantFreq * 100.0f);
    tremorCharData.power = (uint16_t)fminf(65535.0f, tremorResult.power);
    
    bleInstance.gattServer().write(
        tremorCharPtr->getValueHandle(),
        (const uint8_t*)&tremorCharData,
        sizeof(tremorCharData)
    );
    
    dyskinesiaCharData.detected = dyskinesiaResult.detected;
    dyskinesiaCharData.intensity = dyskinesiaResult.intensity;
    dyskinesiaCharData.freqX100 = (uint16_t)(dyskinesiaResult.dominantFreq * 100.0f);
    dyskinesiaCharData.power = (uint16_t)fminf(65535.0f, dyskinesiaResult.power);
    
    bleInstance.gattServer().write(
        dyskinesiaCharPtr->getValueHandle(),
        (const uint8_t*)&dyskinesiaCharData,
        sizeof(dyskinesiaCharData)
    );
    
    fogCharData.detected = fogResult.detected;
    fogCharData.intensity = fogResult.intensity;
    fogCharData.reserved = walkingActive ? 0x0001 : 0x0000;
    
    bleInstance.gattServer().write(
        fogCharPtr->getValueHandle(),
        (const uint8_t*)&fogCharData,
        sizeof(fogCharData)
    );
}

void scheduleBleEvents(BLE::OnEventsToProcessCallbackContext *context) {
    eventQueue.call(mbed::Callback<void()>(&context->ble, &BLE::processEvents));
}

bool initBLE() {
    printf("Initializing BLE...\r\n");
    
    bleInstance.onEventsToProcess(scheduleBleEvents);
    
    ble_error_t error = bleInstance.init(onBleInitComplete);
    if (error != BLE_ERROR_NONE) {
        printf("BLE init call failed: %d\r\n", error);
        return false;
    }
    
    while (!bleInitialized) {
        eventQueue.dispatch(100);
    }
    
    bleInstance.gap().setEventHandler(&gapEventHandler);
    
    static GattCharacteristic tremorChar(
        TREMOR_CHAR_UUID,
        (uint8_t*)&tremorCharData,
        sizeof(tremorCharData),
        sizeof(tremorCharData),
        GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ |
        GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY
    );
    tremorCharPtr = &tremorChar;
    
    static GattCharacteristic dyskinesiaChar(
        DYSKINESIA_CHAR_UUID,
        (uint8_t*)&dyskinesiaCharData,
        sizeof(dyskinesiaCharData),
        sizeof(dyskinesiaCharData),
        GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ |
        GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY
    );
    dyskinesiaCharPtr = &dyskinesiaChar;
    
    static GattCharacteristic fogChar(
        FOG_CHAR_UUID,
        (uint8_t*)&fogCharData,
        sizeof(fogCharData),
        sizeof(fogCharData),
        GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ |
        GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY
    );
    fogCharPtr = &fogChar;
    
    static GattCharacteristic *characteristics[] = {
        &tremorChar,
        &dyskinesiaChar,
        &fogChar
    };
    
    static GattService pdService(
        PD_SERVICE_UUID,
        characteristics,
        sizeof(characteristics) / sizeof(characteristics[0])
    );
    
    error = bleInstance.gattServer().addService(pdService);
    if (error != BLE_ERROR_NONE) {
        printf("Failed to add GATT service: %d\r\n", error);
        return false;
    }
    
    ble::AdvertisingParameters advParams(
        ble::advertising_type_t::CONNECTABLE_UNDIRECTED,
        ble::adv_interval_t(ble::millisecond_t(100))
    );
    
    error = bleInstance.gap().setAdvertisingParameters(ble::LEGACY_ADVERTISING_HANDLE, advParams);
    if (error != BLE_ERROR_NONE) {
        printf("Failed to set advertising params: %d\r\n", error);
        return false;
    }
    
    uint8_t advData[31];
    ble::AdvertisingDataBuilder advBuilder(advData, sizeof(advData));
    advBuilder.setFlags();
    advBuilder.setLocalServiceList(mbed::make_Span(&PD_SERVICE_UUID, 1));
    advBuilder.setName(DEVICE_NAME);
    
    error = bleInstance.gap().setAdvertisingPayload(
        ble::LEGACY_ADVERTISING_HANDLE,
        advBuilder.getAdvertisingData()
    );
    if (error != BLE_ERROR_NONE) {
        printf("Failed to set advertising payload: %d\r\n", error);
        return false;
    }
    
    error = bleInstance.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);
    if (error != BLE_ERROR_NONE) {
        printf("Failed to start advertising: %d\r\n", error);
        return false;
    }
    
    printf("BLE advertising as \"%s\"\r\n", DEVICE_NAME);
    return true;
}

bool i2cWriteReg(uint8_t reg, uint8_t value) {
    char data[2] = {(char)reg, (char)value};
    return (i2c.write(LSM6DSL_ADDR, data, 2) == 0);
}

bool i2cReadReg(uint8_t reg, uint8_t *value) {
    char cmd = reg;
    if (i2c.write(LSM6DSL_ADDR, &cmd, 1, true) != 0) return false;
    char result;
    if (i2c.read(LSM6DSL_ADDR, &result, 1) != 0) return false;
    *value = result;
    return true;
}

bool i2cReadMulti(uint8_t reg, uint8_t *buffer, int len) {
    char cmd = reg;
    if (i2c.write(LSM6DSL_ADDR, &cmd, 1, true) != 0) return false;
    return (i2c.read(LSM6DSL_ADDR, (char*)buffer, len) == 0);
}

bool initLSM6DSL() {
    printf("Initializing LSM6DSL...\r\n");
    
    i2c.frequency(400000);
    
    uint8_t whoami;
    if (!i2cReadReg(LSM6DSL_WHO_AM_I, &whoami)) {
        printf("Failed to read WHO_AM_I\r\n");
        return false;
    }
    
    printf("WHO_AM_I: 0x%02X (expected 0x6A)\r\n", whoami);
    if (whoami != LSM6DSL_WHO_AM_I_VALUE) {
        printf("Wrong device!\r\n");
        return false;
    }
    
    i2cWriteReg(LSM6DSL_CTRL3_C, 0x01);
    ThisThread::sleep_for(50ms);
    
    if (!i2cWriteReg(LSM6DSL_CTRL1_XL, 0x30)) {
        printf("Failed to configure accelerometer\r\n");
        return false;
    }
    
    if (!i2cWriteReg(LSM6DSL_CTRL2_G, 0x30)) {
        printf("Failed to configure gyroscope\r\n");
        return false;
    }
    
    i2cWriteReg(LSM6DSL_CTRL3_C, 0x44);
    
    printf("LSM6DSL initialized: 52Hz, ±2g\r\n");
    return true;
}

float readAccelMagnitude() {
    uint8_t data[6];
    
    if (!i2cReadMulti(LSM6DSL_OUTX_L_XL, data, 6)) {
        return 1.0f;
    }
    
    int16_t ax = (int16_t)(data[1] << 8 | data[0]);
    int16_t ay = (int16_t)(data[3] << 8 | data[2]);
    int16_t az = (int16_t)(data[5] << 8 | data[4]);
    
    float x = ax * 0.000061f;
    float y = ay * 0.000061f;
    float z = az * 0.000061f;
    
    return sqrtf(x*x + y*y + z*z);
}

void computeDFT(float *input, int N, float *magOutput) {
    const float PI = 3.14159265358979f;
    float freqResolution = (float)SAMPLE_RATE_HZ / FFT_SIZE;
    
    int maxBin = (int)(10.0f / freqResolution) + 1;
    if (maxBin > FFT_SIZE / 2) maxBin = FFT_SIZE / 2;
    
    for (int k = 0; k < maxBin; k++) {
        float real = 0.0f;
        float imag = 0.0f;
        float omega = 2.0f * PI * k / FFT_SIZE;
        
        for (int n = 0; n < N; n++) {
            real += input[n] * cosf(omega * n);
            imag -= input[n] * sinf(omega * n);
        }
        
        magOutput[k] = sqrtf(real * real + imag * imag);
    }
    
    for (int k = maxBin; k < FFT_SIZE / 2; k++) {
        magOutput[k] = 0.0f;
    }
}

float getFrequencyBinHz(int bin) {
    return (float)bin * SAMPLE_RATE_HZ / FFT_SIZE;
}

int getFrequencyBin(float freqHz) {
    return (int)(freqHz * FFT_SIZE / SAMPLE_RATE_HZ);
}

void performFFT(float *input, int numSamples) {
    const float PI = 3.14159265358979f;
    
    memset(fftInput, 0, sizeof(fftInput));
    
    float mean = 0.0f;
    for (int i = 0; i < numSamples; i++) {
        mean += input[i];
    }
    mean /= numSamples;
    
    for (int i = 0; i < numSamples; i++) {
        float window = 0.5f * (1.0f - cosf(2.0f * PI * i / (numSamples - 1)));
        fftInput[i] = (input[i] - mean) * window;
    }
    
    computeDFT(fftInput, numSamples, fftMagnitude);
}

DetectionResult analyzeFrequencyBand(float lowFreq, float highFreq, float threshold) {
    DetectionResult result = {0, 0, 0.0f, 0.0f};
    
    int lowBin = getFrequencyBin(lowFreq);
    int highBin = getFrequencyBin(highFreq);
    
    float maxPower = 0.0f; 
    int maxBin = lowBin;
    float totalPower = 0.0f;
    
    for (int i = lowBin; i <= highBin && i < FFT_SIZE / 2; i++) {
        totalPower += fftMagnitude[i];
        if (fftMagnitude[i] > maxPower) {
            maxPower = fftMagnitude[i];
            maxBin = i;
        }
    }
    
    result.dominantFreq = getFrequencyBinHz(maxBin);
    result.power = totalPower;
    
    if (totalPower > threshold) {
        result.detected = 1;
        result.intensity = (uint8_t)fminf(100.0f, (totalPower / threshold) * 50.0f);
    }
    
    return result;
}

void detectTremorAndDyskinesia(float currentVariance, bool isFrozen) {
    performFFT(accMagnitude, NUM_SAMPLES);
    
    DetectionResult rawTremor = analyzeFrequencyBand(TREMOR_LOW_HZ, TREMOR_HIGH_HZ, TREMOR_THRESHOLD);
    DetectionResult rawDyskinesia = analyzeFrequencyBand(DYSKINESIA_LOW_HZ, DYSKINESIA_HIGH_HZ, DYSKINESIA_THRESHOLD);
    
    bool isWalking = (currentVariance > WALKING_THRESHOLD);
    
    if (isFrozen) {
        tremorResult = {0, 0, rawTremor.dominantFreq, rawTremor.power};
        dyskinesiaResult = {0, 0, rawDyskinesia.dominantFreq, rawDyskinesia.power};
    } else if (isWalking) {
        tremorResult = {0, 0, rawTremor.dominantFreq, rawTremor.power};
        dyskinesiaResult = rawDyskinesia;
    } else if (rawTremor.detected && rawDyskinesia.detected) {
        if (rawTremor.power >= rawDyskinesia.power) {
            tremorResult = rawTremor;
            dyskinesiaResult = {0, 0, rawDyskinesia.dominantFreq, rawDyskinesia.power};
        } else {
            tremorResult = {0, 0, rawTremor.dominantFreq, rawTremor.power};
            dyskinesiaResult = rawDyskinesia;
        }
    } else {
        tremorResult = rawTremor;
        dyskinesiaResult = rawDyskinesia;
    }
    
    led2 = tremorResult.detected;
    led3 = dyskinesiaResult.detected;
}

float calculateVariance(float *data, int n) {
    float mean = 0.0f;
    for (int i = 0; i < n; i++) {
        mean += data[i];
    }
    mean /= n;
    
    float variance = 0.0f;
    for (int i = 0; i < n; i++) {
        float diff = data[i] - mean;
        variance += diff * diff;
    }
    return variance / n;
}

void detectFOG(float currentVariance) {
    static enum { IDLE, WALKING, FROZEN } fogState = IDLE;
    static Timer freezeTimer;
    
    switch (fogState) {
        case IDLE:
            if (currentVariance > WALKING_THRESHOLD) {
                fogState = WALKING;
                walkingTimer.reset();
                walkingTimer.start();
                printf("Walking detected\r\n");
            }
            walkingActive = false;
            fogResult.detected = 0;
            break;
            
        case WALKING:
            walkingActive = true;
            if (currentVariance < FOG_STILLNESS_THRESHOLD) {
                walkingDuration = std::chrono::duration_cast<std::chrono::milliseconds>(
                    walkingTimer.elapsed_time()).count();
                if (walkingDuration >= WALKING_DURATION_MS) {
                    fogState = FROZEN;
                    freezeTimer.reset();
                    freezeTimer.start();
                    fogResult.detected = 1;
                    fogResult.intensity = 100;
                    printf("*** FOG DETECTED ***\r\n");
                } else {
                    fogState = IDLE;
                }
            }
            break;
            
        case FROZEN:
            walkingActive = false;
            if (currentVariance > WALKING_THRESHOLD) {
                printf("FOG ended\r\n");
                fogState = IDLE;
                fogResult.detected = 0;
                fogResult.intensity = 0;
            } else {
                uint32_t freezeDuration = std::chrono::duration_cast<std::chrono::milliseconds>(
                    freezeTimer.elapsed_time()).count();
                fogResult.intensity = (uint8_t)fminf(100.0f, 50.0f + freezeDuration / 100.0f);
            }
            break;
    }
}

void onButtonFall() {
    buttonPressTimer.reset();
    buttonPressTimer.start();
    buttonPressActive = true;
}

void onButtonRise() {
    if (!buttonPressActive) return;
    buttonPressActive = false;

    buttonPressTimer.stop();
    const uint32_t pressMs = std::chrono::duration_cast<std::chrono::milliseconds>(
        buttonPressTimer.elapsed_time()).count();

    if (pressMs >= 2000) {
        bleResetRequested = true;
    } else {
        buttonPressed = true;
    }
}

void printStatus() {
    printf("\r\n===== Detection Status =====\r\n");
    printf("Tremor:     %s (Int:%3d%%, Freq:%d.%dHz, Pwr:%d)\r\n", 
           tremorResult.detected ? "YES" : "No ",
           (int)tremorResult.intensity,
           (int)tremorResult.dominantFreq,
           (int)(tremorResult.dominantFreq * 10) % 10,
           (int)tremorResult.power);
    
    printf("Dyskinesia: %s (Int:%3d%%, Freq:%d.%dHz, Pwr:%d)\r\n",
           dyskinesiaResult.detected ? "YES" : "No ",
           (int)dyskinesiaResult.intensity,
           (int)dyskinesiaResult.dominantFreq,
           (int)(dyskinesiaResult.dominantFreq * 10) % 10,
           (int)dyskinesiaResult.power);
    
    printf("FOG:        %s (Int:%3d%%)\r\n",
           fogResult.detected ? "YES" : "No ",
           (int)fogResult.intensity);
    printf("============================\r\n");
}

int main() {
    printf("\r\n========================================\r\n");
    printf("  Parkinson's Symptom Detector\r\n");
    printf("  B-L475E-IOT01A Discovery Kit\r\n");
    printf("  Embedded Challenge Fall 2025\r\n");
    printf("========================================\r\n\r\n");
    
    led1 = 1;
    led2 = 0;
    led3 = 0;
    
    userButton.fall(&onButtonFall);
    userButton.rise(&onButtonRise);
    
    if (!initLSM6DSL()) {
        printf("Sensor init failed! Check connections.\r\n");
        while (1) {
            led1 = !led1;
            ThisThread::sleep_for(100ms);
        }
    }
    
    if (!initBLE()) {
        printf("BLE init failed!\r\n");
    }
    
    printf("\r\nSystem ready!\r\n");
    printf("LED1: System status (fast=running, slow=paused)\r\n");
    printf("LED2: Tremor detected (3-5Hz)\r\n");
    printf("LED3: Dyskinesia detected (5-7Hz)\r\n");
    printf("Button: Short press = Pause/Resume\r\n");
    printf("Button: Long press  = Reset device (BLE reset)\r\n");
    printf("BLE: Connect to \"%s\" for live data\r\n\r\n", DEVICE_NAME);
    
    int sampleIndex = 0;
    int blinkCounter = 0;
    
    while (true) {
        if (bleResetRequested) {
            bleResetRequested = false;
            printf("\r\n*** Reset requested (BLE reset) ***\r\n");
            ThisThread::sleep_for(50ms);
            NVIC_SystemReset();
        }

        if (buttonPressed) {
            buttonPressed = false;
            systemRunning = !systemRunning;
            printf("\r\n*** System %s ***\r\n", systemRunning ? "RUNNING" : "PAUSED");
            
            if (!systemRunning) {
                led2 = 0;
                led3 = 0;
                sampleIndex = 0;
                tremorResult = {0, 0, 0.0f, 0.0f};
                dyskinesiaResult = {0, 0, 0.0f, 0.0f};
                fogResult = {0, 0, 0.0f, 0.0f};
            }
        }
        
        blinkCounter++;
        if (systemRunning) {
            led1 = (blinkCounter / 10) % 2;
        } else {
            led1 = (blinkCounter / 50) % 2;
        }
        
        if (systemRunning) {
            float mag = readAccelMagnitude();
            accMagnitude[sampleIndex++] = mag;
            
            if (sampleIndex >= NUM_SAMPLES) {
                float variance = calculateVariance(accMagnitude, NUM_SAMPLES);
                
                detectFOG(variance);
                
                detectTremorAndDyskinesia(variance, fogResult.detected);
                
                printStatus();
                
                updateBLECharacteristics();
                
                memmove(accMagnitude, &accMagnitude[NUM_SAMPLES/2], 
                        (NUM_SAMPLES/2) * sizeof(float));
                sampleIndex = NUM_SAMPLES / 2;
            }
        }
        
        eventQueue.dispatch(0);
        
        ThisThread::sleep_for(std::chrono::milliseconds(SAMPLE_PERIOD_MS));
    }
    
    return 0;
}