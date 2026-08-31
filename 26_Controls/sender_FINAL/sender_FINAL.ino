/*
  IMU Sender Module (Updated to match receiver)

  Sends:
  senderId, ax, ay, az, gx, gy, gz, t_ms
*/

#include <esp_now.h>
#include <WiFi.h>
#include <SPI.h>
#include <mpu6500.h>

// ==============================
// CONFIG
// ==============================

#define BOARD_ID 1

// Receiver MAC (YOUR CYD)
uint8_t receiverMAC[] = {0xC0, 0xCD, 0xD6, 0x84, 0xC5, 0x80};

// SPI pins
#define SCK 4
#define MISO 5
#define MOSI 6
#define CS 7

// Timing
#define SAMPLE_INTERVAL_MS 5     // IMU sampling (~200 Hz)
#define SEND_INTERVAL_MS   20    // ESP-NOW send rate (50 Hz)

// ==============================
// STRUCT (MUST MATCH RECEIVER)
// ==============================

typedef struct struct_message {
  uint8_t senderId;
  float ax, ay, az;
  float gx, gy, gz;
  uint32_t t_ms;
} struct_message;

struct_message data;

// ==============================
// IMU OBJECT
// ==============================

Mpu6500 imu(SPI, CS);

// ==============================
// TIMERS
// ==============================

unsigned long lastSampleTime = 0;
unsigned long lastSendTime = 0;

// ==============================
// SETUP
// ==============================

void setup() {
  Serial.begin(115200);

  // WiFi setup
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(1);

  Serial.print("Sender MAC: ");
  Serial.println(WiFi.macAddress());

  // ESP-NOW init
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  // Register peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // SPI + IMU init
  SPI.begin(SCK, MISO, MOSI, CS);

  if (imu.begin() < 0) {
    Serial.println("IMU init failed");
    while (1);
  }

  // Set sample rate (~200 Hz)
  imu.setSrd(4);

  Serial.println("IMU initialized");

  delay(2000); // match receiver delay
}

// ==============================
// LOOP
// ==============================

void loop() {
  unsigned long now = millis();

  // ==========================
  // SAMPLE IMU
  // ==========================
  if (now - lastSampleTime >= SAMPLE_INTERVAL_MS) {
    lastSampleTime = now;

    imu.readSensor();

    data.senderId = BOARD_ID;

    data.ax = imu.getAccelX_mss();
    data.ay = imu.getAccelY_mss();
    data.az = imu.getAccelZ_mss();

    data.gx = imu.getGyroX_rads();
    data.gy = imu.getGyroY_rads();
    data.gz = imu.getGyroZ_rads();

    data.t_ms = now;
  }

  // ==========================
  // SEND DATA
  // ==========================
  if (now - lastSendTime >= SEND_INTERVAL_MS) {
    lastSendTime = now;

    esp_err_t result = esp_now_send(
      receiverMAC,
      (uint8_t *)&data,
      sizeof(data)
    );

    if (result != ESP_OK) {
      Serial.println("Send error");
    }
  }
}