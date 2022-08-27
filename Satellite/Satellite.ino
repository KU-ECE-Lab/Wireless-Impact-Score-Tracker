// #define ETL_NO_STL
#include <WiFi.h>
#include <esp_now.h>
#include <ArduinoJson.h>
#include <Embedded_Template_Library.h>
#include <etl/array.h>
#include <etl/atomic.h>
#include <Wire.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <Arduino_DebugUtils.h>

Adafruit_LIS3DH lis;

#define CLICKTHRESHHOLD 180

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#define OTHER_CORE 1
#else
#define ARDUINO_RUNNING_CORE 1
#define OTHER_CORE 0
#endif

TaskHandle_t task_check_taps;
TaskHandle_t task_send_data;

using mac_t = etl::array<uint8_t, 6>;

struct Hub {
  mac_t mac_address;
  esp_now_peer_info_t peer_info;
  bool peer_connected;
};

Hub hub = { { 0xF4, 0x12, 0xFA, 0x59, 0x6A, 0x64 } };

etl::atomic<uint16_t> number_of_taps;

void TaskCheckTaps(void* pvParameters) {
  (void) pvParameters;
  for (;;) {
    auto click = lis.getClick();
    if (click & 0x10 && !(click == 0) && (click & 0x30)) ++number_of_taps;
    delay(20);
  }
}

void TaskSendData(void* pvParameters) {
  (void) pvParameters;
  for (;;) {
  StaticJsonDocument<100> testDocument;

  testDocument["globalReset"] = false;
  testDocument["localReset"] = true;
  testDocument["threshold"] = number_of_taps.load();

  uint8_t buffer[100];

  int bytesWritten = serializeMsgPack(testDocument, buffer);
  // for(int i = 0; i<bytesWritten; i++){
  //   Serial.printf("%02X ",buffer[i]);
  // }
  Serial.println("Finished serializing");
  esp_err_t result = esp_now_send(hub.mac_address.data(), buffer, bytesWritten);
  if (result == ESP_OK)
    Serial.println("Sent data successfully!");
  else {
    Serial.print("Error sending the data: ");
    Serial.println(result);
  }
  delay(200);
  }
}

void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  //Initialize Serial Monitor
  Serial.begin(115200);
  Debug.setDebugOutputStream(&Serial);
  Debug.setDebugLevel(DBG_VERBOSE);
  Debug.newlineOff();

  //Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  // esp_now_register_recv_cb(OnDataRecv);

  memcpy(hub.peer_info.peer_addr, hub.mac_address.data(), hub.mac_address.size());
  hub.peer_info.channel = 0;
  hub.peer_info.encrypt = false;

  if (esp_now_add_peer(&hub.peer_info) == ESP_OK) {  // Add the peer and check if there were any issues
    hub.peer_connected = true;
    Debug.print(DBG_DEBUG, "Added hub: %02x:%02x:%02x:%02x:%02x:%02x\n", hub.mac_address[0], hub.mac_address[1], hub.mac_address[2], hub.mac_address[3], hub.mac_address[4], hub.mac_address[5]);
  } else {
    hub.peer_connected = false;
    Debug.print(DBG_ERROR, "Failed to add hub: %02x:%02x:%02x:%02x:%02x:%02x\n", hub.mac_address[0], hub.mac_address[1], hub.mac_address[2], hub.mac_address[3], hub.mac_address[4], hub.mac_address[5]);
  }
  Debug.print(DBG_VERBOSE, "Finished adding hub\n");

  lis = Adafruit_LIS3DH();
  lis.begin(0x18);
  lis.setRange(LIS3DH_RANGE_4_G);  // 2, 4, 8 or 16 G!
  lis.setClick(1, CLICKTHRESHHOLD);

  xTaskCreatePinnedToCore(
    TaskCheckTaps,
    "TaskCheckTaps",
    10000,
    NULL,
    2,
    &task_check_taps,
    OTHER_CORE);

  xTaskCreatePinnedToCore(
    TaskSendData,
    "TaskSendData",
    10000,
    NULL,
    3,
    &task_send_data,
    ARDUINO_RUNNING_CORE);
}

void loop() {
}