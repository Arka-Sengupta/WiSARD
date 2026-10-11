#include "RF24.h"
#include <SPI.h>
#include <SD.h>
#include "esp_bt.h"
#include "esp_wifi.h"

// --- Pin Configuration ---
const int CE_PIN = 26;
const int CSN_PIN = 15;
const int SCK_PIN = 14;
const int MOSI_PIN = 13;
const int MISO_PIN = 12;
const int IRQ_PIN = 27;

const int SD_CS_PIN = 5;
const int SD_MOSI_PIN = 23;
const int SD_MISO_PIN = 19;
const int SD_SCK_PIN = 18;

// --- Configuration ---
const int NUM_CHANNELS = 26;   // 2.4 GHz band
const int WINDOW_SIZE = 50;    // 50 samples per window
const int SAMPLE_RATE_MS = 10; // Sample every 10ms
const int STREAM_RATE_MS = 100; // Send data every 100ms
const unsigned long SD_FLUSH_MS = 2000; // Flush SD log every 2s

// --- Global Objects ---
RF24 radio(CE_PIN, CSN_PIN, 16000000);
SPIClass *hp = nullptr;

// --- State ---
bool radio_ok = false;
bool sd_ok = false;
File logFile;

// Packet Counting
volatile bool irqFlag = false;   // set by ISR, handled in loop()
uint32_t packetCount = 0;
uint32_t packetCountPrev = 0;

// Feature Buffer
float ppsBuffer[WINDOW_SIZE] = {0};
int ppsIdx = 0;
int ppsCount = 0;

// Channel State
int currentChannel = 1;
unsigned long lastStream = 0;
unsigned long lastChannelHop = 0;
unsigned long lastFlush = 0;

// Bandwidth Tracking
bool channelActivity[NUM_CHANNELS] = {false};
int activeChannelCount = 0;

// --- Feature Structure ---
struct Features {
  float mean;
  float variance;
  float occupancy;
  float entropy;
  int bandwidth;
};

// --- ISR: ONLY set a flag. No SPI / radio calls inside ISR! ---
void IRAM_ATTR onIRQ() {
  irqFlag = true;
}

// --- Drain RX FIFO and count packets (runs in loop, not ISR) ---
void processRadio() {
  uint8_t dummy[32];
  while (radio.available()) {
    radio.read(dummy, 32);
    packetCount++;
  }
  // Clear IRQ flags on the nRF24 so the IRQ pin goes back HIGH
  bool tx_ok, tx_fail, rx_ready;
  radio.whatHappened(tx_ok, tx_fail, rx_ready);
}

// --- Functions ---
void initRadio() {
  hp = new SPIClass(HSPI);
  hp->begin(SCK_PIN, MISO_PIN, MOSI_PIN, CSN_PIN);

  for (int i = 0; i < 5; i++) {
    delay(100);
    if (radio.begin(hp)) {
      radio.setAutoAck(false);
      radio.setRetries(0, 0);
      radio.setPALevel(RF24_PA_MAX, true);
      radio.setDataRate(RF24_2MBPS);
      radio.setCRCLength(RF24_CRC_DISABLED);
      radio.setPayloadSize(32);
      radio.setAddressWidth(3);
      radio.maskIRQ(true, true, false); // IRQ pin only on RX data ready
      radio.openReadingPipe(1, 0xE7E7E7LL);
      radio.startListening();
      radio_ok = true;
      Serial.println("[JDCS] Radio Ready (Packet Counting Mode).");
      return;
    }
    Serial.print("[JDCS] Radio Init Attempt ");
    Serial.print(i + 1);
    Serial.println(" failed.");
  }
}

void initSD() {
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);

  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);

  if (SD.begin(SD_CS_PIN, SPI)) {
    sd_ok = true;
    logFile = SD.open("/jdc_log.csv", FILE_WRITE);
    if (logFile) {
      logFile.println("Timestamp,Channel,PPS_Mean,PPS_Var,Occupancy,Entropy,Bandwidth,Class");
      logFile.flush();
      Serial.println("[JDCS] SD Card Initialized.");
    }
  } else {
    Serial.println("[JDCS] SD Card FAILED.");
  }
}

// --- Feature Extraction ---
Features extractFeatures() {
  Features f;
  float sum = 0;
  for (int i = 0; i < ppsCount; i++) {
    sum += ppsBuffer[i];
  }
  f.mean = sum / ppsCount;

  float sumSq = 0;
  for (int i = 0; i < ppsCount; i++) {
    float diff = ppsBuffer[i] - f.mean;
    sumSq += diff * diff;
  }
  f.variance = sumSq / ppsCount;

  int occupied = 0;
  for (int i = 0; i < ppsCount; i++) {
    if (ppsBuffer[i] > 0) occupied++;
  }
  f.occupancy = (100.0 * occupied) / ppsCount;

  int bins[8] = {0};
  for (int i = 0; i < ppsCount; i++) {
    int bin = min(7, (int)(ppsBuffer[i] / 10.0));
    bins[bin]++;
  }
  float entropy = 0;
  for (int i = 0; i < 8; i++) {
    if (bins[i] > 0) {
      float p = (float)bins[i] / ppsCount;
      entropy -= p * log2(p);
    }
  }
  f.entropy = entropy;

  f.bandwidth = activeChannelCount;

  return f;
}

// --- Rule-Based Classifier ---
int classify(Features &f) {
  if (f.occupancy > 80 && f.variance < 10) {
    return 3; // Full Band
  }
  if (f.variance > 10 && f.occupancy > 50) {
    return 4; // Pulse
  }
  if (f.entropy > 1.5 && f.occupancy > 50) {
    return 1; // Wi-Fi
  }
  if (f.occupancy > 50) {
    return 2; // BLE
  }
  return 0;
}

// --- Data Streaming ---
void streamData(Features &f, int cls) {
  // Format: CH,PPS_MEAN,PPS_VAR,OCCUPANCY,ENTROPY,BANDWIDTH,CLASS
  Serial.print(currentChannel);
  Serial.print(",");
  Serial.print(f.mean, 1);
  Serial.print(",");
  Serial.print(f.variance, 1);
  Serial.print(",");
  Serial.print(f.occupancy, 1);
  Serial.print(",");
  Serial.print(f.entropy, 2);
  Serial.print(",");
  Serial.print(f.bandwidth);
  Serial.print(",");
  Serial.println(cls);
}

// --- Setup ---
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n================================");
  Serial.println("  JAMMER DETECTION & CLASSIFICATION");
  Serial.println("  JDCS v2.2 (Bandwidth Tracking)");
  Serial.println("================================");

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);
  esp_wifi_stop();
  esp_wifi_deinit();
  esp_bt_controller_deinit();

  initRadio();
  initSD();

  if (radio_ok) {
    pinMode(IRQ_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(IRQ_PIN), onIRQ, FALLING);
    Serial.println("[JDCS] System Ready.");
    Serial.println("Features: PPS, Var, Occ, Ent, Bandwidth");
    Serial.println("Format: CH,PPS_MEAN,PPS_VAR,OCCUPANCY,ENTROPY,BANDWIDTH,CLASS");
  }
}

// --- Main Loop ---
void loop() {
  if (!radio_ok) return;

  unsigned long now = millis();

  // 0. Handle packets (IRQ flag, plus poll as backup if IRQ missed)
  if (irqFlag) {
    irqFlag = false;
    processRadio();
  } else {
    processRadio();
  }

  // 1. Channel Hopping & Bandwidth Update
  if (now - lastChannelHop >= (WINDOW_SIZE * SAMPLE_RATE_MS)) {
    lastChannelHop = now;

    float currentPPS = (packetCount - packetCountPrev) * 10.0;
    packetCountPrev = packetCount;

    ppsBuffer[ppsIdx] = currentPPS;
    ppsIdx = (ppsIdx + 1) % WINDOW_SIZE;
    ppsCount = (ppsCount < WINDOW_SIZE) ? ppsCount + 1 : WINDOW_SIZE;

    // Update Bandwidth: Check if current channel is active
    int chIndex = currentChannel - 1; // 0-25
    if (currentPPS > 5.0) {
      if (!channelActivity[chIndex]) {
        channelActivity[chIndex] = true;
        activeChannelCount++;
      }
    } else {
      if (channelActivity[chIndex]) {
        channelActivity[chIndex] = false;
        activeChannelCount--;
      }
    }

    // Hop to next channel
    currentChannel = (currentChannel % NUM_CHANNELS) + 1;
    radio.setChannel(currentChannel);
  }

  // 2. Analysis & Classification
  if (now - lastStream >= STREAM_RATE_MS && ppsCount >= WINDOW_SIZE) {
    lastStream = now;

    Features f = extractFeatures();
    int cls = classify(f);

    if (sd_ok && logFile) {
      logFile.print(now);
      logFile.print(",");
      logFile.print(currentChannel);
      logFile.print(",");
      logFile.print(f.mean, 2);
      logFile.print(",");
      logFile.print(f.variance, 2);
      logFile.print(",");
      logFile.print(f.occupancy, 2);
      logFile.print(",");
      logFile.print(f.entropy, 2);
      logFile.print(",");
      logFile.print(f.bandwidth);
      logFile.print(",");
      logFile.println(cls);

      if (now - lastFlush >= SD_FLUSH_MS) {
        lastFlush = now;
        logFile.flush();
      }
    }

    streamData(f, cls);
  }
}