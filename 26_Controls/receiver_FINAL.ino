/*
  Receiver module

  TODO:
  1. Internal timer + global timer for time stamp comparison?
  2. Don't rewrite file every time
  3. Expand to multiple esp modules
    - File save process
      - Save to one file, then use Python Pandas to separate into files
  4. Delay for all ESPs to connect
  5. Constantly check for SD card so that it will start writing whenever the SD card is inserted

*/

// esp-now
#include <esp_now.h>
#include <WiFi.h>

// sd card libraries
#include "FS.h"
#include "SD.h"
#include "SPI.h"

// pins
#define SCK 4
#define MISO 5
#define MOSI 6
#define CS 7

File logFile;

typedef struct struct_message {
  uint8_t senderId;
  float ax, ay, az;
  float gx, gy, gz;
  uint32_t t_ms;
} struct_message;

struct_message systemStruct[4];

void initSDCard() {
  SPI.begin(SCK, MISO, MOSI, CS);
  if (!SD.begin(CS)) {
    Serial.println("ERROR: SD card mount failed");
    return;
  }
  Serial.println("SD card initialized");

  if (!SD.exists("/imu_data.txt")) {
    File tmp = SD.open("/imu_data.txt", FILE_WRITE);
    if (tmp) {
      tmp.print("t(ms),id,ax,ay,az,gx,gy,gz\n");
      tmp.close();
      Serial.println("Log file created");
    }
  } else {
    Serial.println("Log file exists, appending...");
  }

  logFile = SD.open("/imu_data.txt", FILE_APPEND);
  if (!logFile) {
    Serial.println("ERROR: Failed to open log file for appending");
  }
}

void OnDataRecv(const esp_now_recv_info *info,
                const uint8_t *incomingData,
                int len)
{
  struct_message temp;
  memcpy(&temp, incomingData, sizeof(temp));

  if (temp.senderId >= 1 && temp.senderId <= 4) {
    systemStruct[temp.senderId - 1] = temp;
  }

  if (logFile) {
    logFile.printf("%lu,%u,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n",
      (unsigned long)temp.t_ms,
      (unsigned int)temp.senderId,
      temp.ax, temp.ay, temp.az,
      temp.gx, temp.gy, temp.gz
    );
    logFile.flush();
  }
  /*Serial.printf("%lu,%u,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
    (unsigned long)temp.t_ms,
    (unsigned int)temp.senderId,
    temp.ax, temp.ay, temp.az,
    temp.gx, temp.gy, temp.gz
  );*/
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(1);

  Serial.print("Receiver MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
  initSDCard();

  delay(2000); // match with sender delay
}

void loop() {
}
