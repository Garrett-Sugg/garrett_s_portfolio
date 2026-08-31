/*
  IMU module

  TODO:
  1. Setup protocol on setup for checking imu/esp settings
  2. Expand to multiple esp modules
    - Channel identifiers for each modules (done?)
  3. Add timing constraints for separate suspension systems
    - Would this mess up time sync for overall impacts?

*/

//*****************************//
//  ESP to IMU pin connection  //
//   --ESP--         --IMU--   //
//  GPIO  NAME      NAME GPIO  //
//  3.3   VCC <---> VCC   1    //
//  GND   GND <---> GND   2    //
//   4    SCK <---> SCL   3    //
//   5    MISO <--> AD0   7    //
//   6    MOSI <--> SDA   4    //
//   7    SS/CS <-> NCS   9    //
//*****************************//

#include <WiFi.h>

#include <Arduino.h>
#include <SPI.h>
#include <mpu6500.h>
#include <esp_now.h>
#include <cstdint> 
#include <cstddef> 

// configuration 
uint8_t receiver_address[] = {0x88, 0x56, 0xA6, 0x6E, 0x75, 0x40};

uint8_t senderId = 2; // change sender_id when flashing new sender modules\

// 1 = left front
// 2 = right front
// 3 = left rear
// 4 = right rear

/* implement later
uint8_t channel = 1; // every module should be on the same channel
uint8_t imuSrd = 4; // match sampling rate;1000/(srd+1)=200Hz (srd=4)
*/

int PIN_SCK = 4; int PIN_MISO = 5; int PIN_MOSI = 6; int PIN_SS_CS = 7;

typedef struct struct_message {
  uint8_t senderId;
  float ax, ay, az;
  float gx, gy, gz;
  uint32_t t_ms;
} struct_message;

struct_message imu_data;

// correct declaration of Mpu6500 object, SPI bus, SS_CS on pin 7
// DO NOT CHANGE SS_CS VALUE, the imu will throw imu.begin error
bfs::Mpu6500 imu(&SPI, 7); 

esp_now_peer_info_t peerInfo; 
// #############################################################################
bool imu_init() {
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_SS_CS);

  if (!imu.Begin()) {
    Serial.println("ERROR: imu.Begin failed");
   return false;
  }
  Serial.println("IMU init successful");
  delay(2000);
  return true;
}
// #############################################################################
// callback function for debugging
void OnDataSent(const wifi_tx_info_t* mac_addr, esp_now_send_status_t status) {
  Serial.print("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}
// #############################################################################
void checkData(){
  Serial.print("ax: "); Serial.print(imu_data.ax);
  Serial.print(" ay: "); Serial.print(imu_data.ay);
  Serial.print(" az: "); Serial.println(imu_data.az);

  Serial.print("gx: "); Serial.print(imu_data.gx);
  Serial.print(" gy: "); Serial.print(imu_data.gy);
  Serial.print(" gz: "); Serial.println(imu_data.gz);

  Serial.print("Id: "); Serial.println(imu_data.senderId);

  Serial.print("t(ms): "); Serial.println(imu_data.t_ms);

  Serial.println("");
}
// #############################################################################
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(1);

  // imu init check
  if (!imu_init()) { 
    while (true) delay(1000);
  }

  // esp-now init check
  if (esp_now_init() != ESP_OK) { 
    Serial.println("ERROR: ESP-NOW init failed");
    while (true) delay(1000);
  }

  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, receiver_address, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("ERROR: Failed to add peer");
    while (true) delay(1000);
  }

  Serial.println("IMU Sender Ready");
  delay(2000);
  // should add setup display (ie. what each ESP hz is set at, address its 
  // talking to, etc.) for general check and making sure everything is synced
}
// #############################################################################
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 5000; // 10 seconds

void loop() {
  if (!imu.Read()) return; 
  if (!imu.new_imu_data()) return;

  unsigned long now = millis(); // timing gate
  if(now - lastSendTime >= sendInterval) {
    lastSendTime = now;

    imu_data.ax = imu.accel_x_mps2();
    imu_data.ay = imu.accel_y_mps2(); 
    imu_data.az = imu.accel_z_mps2();

    imu_data.gx = imu.gyro_x_radps();
    imu_data.gy = imu.gyro_y_radps(); 
    imu_data.gz = imu.gyro_z_radps();

    imu_data.senderId = senderId;

    imu_data.t_ms = now; 

    esp_now_send(receiver_address, (uint8_t*)&imu_data, sizeof(imu_data));
  }
}

