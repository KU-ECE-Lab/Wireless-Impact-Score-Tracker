// #define ETL_NO_STL
#include <WiFi.h>
#include <esp_now.h>
#include <ArduinoJson.h>
#include <Embedded_Template_Library.h>
#include <etl/array.h>
#include <Wire.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <Arduino_DebugUtils.h>

Adafruit_LIS3DH lis;

#define CLICKTHRESHHOLD 180


using mac_t = etl::array<uint8_t, 6>;

struct Hub {
  mac_t mac_address;
  esp_now_peer_info_t peer_info;
  bool peer_connected;
};

Hub hub = { { 0xF4, 0x12, 0xFA, 0x59, 0x6A, 0x64 } };

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  //Initialize Serial Monitor
  Serial.begin(115200);
  while (!Serial)
    ;
  Debug.setDebugOutputStream(&Serial);
  Debug.setDebugLevel(DBG_VERBOSE);
  Debug.newlineOff();
  Serial.println("Test");
  Debug.print(DBG_VERBOSE, "DBG Test\n");

  //Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);

  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
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
}

int thres = 0;

void loop() {
  StaticJsonDocument<100> testDocument;

  testDocument["globalReset"] = false;
  testDocument["localReset"] = true;
  testDocument["threshold"] = thres;

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

  uint8_t click = lis.getClick();
  if (click & 0x10 && !(click == 0) && (click & 0x30)) {
    ++thres;
  }
  delay(200);
}