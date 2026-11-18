#include <Arduino.h>
#include <math.h>
#include <stdint.h>

namespace {

// Pin & ADC configuration for Arduino Nano 33 IoT (SAMD21G18A, 3.3V logic).
// Note: Avoid A4 and A5 for analog inputs, as they share the I2C bus with
// onboard pull-up resistors.
constexpr uint8_t kFlexPin = A0;
constexpr uint8_t kAdcBits = 12;
constexpr uint16_t kAdcMax = (1u << kAdcBits) - 1u;  // 4095 for 12-bit ADC
constexpr float kVccVolts = 3.3f;

// Voltage divider configuration:
//   Option A (kFlexConnectedToGnd = true):
//     3V3 --- [ 10kΩ Resistor ] --- (kFlexPin) --- [ SpectraFlex ] --- GND
//     Voltage increases as the flex sensor bends.
//
//   Option B (kFlexConnectedToGnd = false):
//     3V3 --- [ SpectraFlex ] --- (kFlexPin) --- [ 10kΩ Resistor ] --- GND
//     Voltage decreases as the flex sensor bends.
constexpr bool kFlexConnectedToGnd = true;
constexpr float kFixedResistorOhms = 10000.0f;

// Sampling & serial streaming configuration.
constexpr uint32_t kSerialBaud = 115200;
constexpr uint32_t kSampleIntervalMs = 20;  // 50 Hz
constexpr uint8_t kOversampleCount = 16;    // Averaged reads per output sample

uint32_t lastSampleMs = 0;
bool wasSerialConnected = false;

float readAveragedAdc(uint8_t pin, uint8_t samples) {
  // Discard first reading after mux switch/idle to allow S&H capacitor to settle.
  (void)analogRead(pin);

  uint32_t sum = 0;
  for (uint8_t i = 0; i < samples; ++i) {
    sum += static_cast<uint32_t>(analogRead(pin));
  }
  return static_cast<float>(sum) / static_cast<float>(samples);
}

float computeFlexResistanceOhms(float adcCounts) {
  if (kFlexConnectedToGnd) {
    const float denom = static_cast<float>(kAdcMax) - adcCounts;
    if (denom <= 0.5f) {
      return INFINITY;
    }
    return kFixedResistorOhms * (adcCounts / denom);
  } else {
    if (adcCounts <= 0.5f) {
      return INFINITY;
    }
    return kFixedResistorOhms * ((static_cast<float>(kAdcMax) - adcCounts) / adcCounts);
  }
}

void printCsvHeader() {
  Serial.println(F("timestamp_ms,raw_adc,voltage_v,resistance_ohm"));
}

}  // namespace

void setup() {
  Serial.begin(kSerialBaud);

  // Briefly wait for native USB CDC connection so initial header isn't missed,
  // without blocking indefinitely if powered without a host monitor attached.
  const uint32_t waitStartMs = millis();
  while (!Serial && (millis() - waitStartMs < 2000)) {
    delay(10);
  }

  analogReadResolution(kAdcBits);
  analogReference(AR_DEFAULT);  // 3.3V ratiometric reference (VDDANA)
  pinMode(kFlexPin, INPUT);

  if (Serial) {
    wasSerialConnected = true;
    printCsvHeader();
  }

  lastSampleMs = millis();
}

void loop() {
  const bool serialConnected = static_cast<bool>(Serial);
  if (serialConnected && !wasSerialConnected) {
    wasSerialConnected = true;
    printCsvHeader();
  } else if (!serialConnected) {
    wasSerialConnected = false;
  }

  const uint32_t nowMs = millis();
  if (nowMs - lastSampleMs < kSampleIntervalMs) {
    return;
  }
  lastSampleMs += kSampleIntervalMs;
  // Re-sync if loop was stalled by >1 interval (e.g., USB host backpressure).
  if (nowMs - lastSampleMs >= kSampleIntervalMs) {
    lastSampleMs = nowMs;
  }

  const float rawAdc = readAveragedAdc(kFlexPin, kOversampleCount);
  const float voltageV = (rawAdc / static_cast<float>(kAdcMax)) * kVccVolts;
  const float resistanceOhm = computeFlexResistanceOhms(rawAdc);

  if (serialConnected) {
    Serial.print(nowMs);
    Serial.print(',');
    Serial.print(rawAdc, 1);
    Serial.print(',');
    Serial.print(voltageV, 4);
    Serial.print(',');
    if (isfinite(resistanceOhm)) {
      Serial.println(resistanceOhm, 1);
    } else {
      Serial.println(F("inf"));
    }
  }
}
