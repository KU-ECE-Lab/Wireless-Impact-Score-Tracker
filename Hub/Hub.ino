#include <Arduino_DebugUtils.h>
// #define ETL_NO_STL
#include <ArduinoJson.h>
#include <Embedded_Template_Library.h>
#include <etl/array.h>
#include <Arduino.h>
#include <sdkconfig.h>


// Sender Code
#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_GFX.h>     // Core graphics library
#include <Adafruit_ST7789.h>  // Hardware-specific library for ST7789
#include <SPI.h>
#include <etl/atomic.h>

// #define DEBUG

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

// Use dedicated hardware SPI pins
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

using mac_t = etl::array<uint8_t, 6>;

struct Satellite {
  mac_t mac_address;
  esp_now_peer_info_t peer_info;
  bool peer_exists;
  bool peer_connected;
};

// REPLACE WITH THE RECEIVER'S MAC Address
// mac_t broadcastAddress = { 0xF4, 0x12, 0xFA, 0x5A, 0x1B, 0xE0 };

constexpr Satellite satellite_0 = { { 0xF4, 0x12, 0xFA, 0x5A, 0x1B, 0xE0 } };

etl::array<Satellite, 1> satellites = { satellite_0 };

// Create peer interface
esp_now_peer_info_t peerInfo;

StaticJsonDocument<512> json_doc;

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  Debug.print(DBG_INFO, "Packet Received\n");
  Debug.print(DBG_DEBUG, "From: %02X:%02X:%02X:%02X:%02X:%02X\n", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

  DeserializationError error = deserializeMsgPack(json_doc, incomingData);
  if (error) Debug.print(DBG_ERROR, "deserializeMsgPack() failed: %s\n", error.f_str());
}

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}



void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
  Debug.setDebugOutputStream(&Serial);
  Debug.setDebugLevel(DBG_ERROR);
  Debug.newlineOff();

  // turn on backlite
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);

  // turn on the TFT / I2C power supply
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
  delay(10);

  // initialize TFT
  tft.init(135, 240);  // Init ST7789 240x135
  tft.setRotation(3);
  tft.fillScreen(ST77XX_BLACK);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_recv_cb(OnDataRecv);
  // esp_now_register_send_cb(OnDataSent);

  for (auto &peer : satellites) {
    memcpy(peer.peer_info.peer_addr, peer.mac_address.data(), peer.mac_address.size());
    peer.peer_info.channel = 0;
    peer.peer_info.encrypt = false;
    peer.peer_exists = esp_now_is_peer_exist(peer.mac_address.data());

    Debug.print(DBG_DEBUG, "Found peer: %02x:%02x:%02x:%02x:%02x:%02x\n", peer.mac_address[0], peer.mac_address[1], peer.mac_address[2], peer.mac_address[3], peer.mac_address[4], peer.mac_address[5]);
  }
  Debug.print(DBG_VERBOSE, "Finished finding peers\n");

  for (auto &peer : satellites) {
    if (peer.peer_exists) {
      if (esp_now_add_peer(&peer.peer_info) == ESP_OK) {
        Debug.print(DBG_DEBUG, "Added peer: %02x:%02x:%02x:%02x:%02x:%02x\n", peer.mac_address[0], peer.mac_address[1], peer.mac_address[2], peer.mac_address[3], peer.mac_address[4], peer.mac_address[5]);
      } else {
        Debug.print(DBG_ERROR, "Failed to add peer: %02x:%02x:%02x:%02x:%02x:%02x\n", peer.mac_address[0], peer.mac_address[1], peer.mac_address[2], peer.mac_address[3], peer.mac_address[4], peer.mac_address[5]);
      }
    }
  }
  Debug.print(DBG_VERBOSE, "Finished adding peers\n");
}

void loop() {
  bool global_reset = json_doc["globalReset"];
  bool local_reset = json_doc["localReset"];
  int threshold = json_doc["threshold"];

  Debug.print(DBG_VERBOSE, "JSON Parsing Successful");

  char parsed[200];
  sprintf(parsed, "Global Reset: %s Local Reset: %s Threshold: %d\n\n", (global_reset) ? "True" : "False", (local_reset) ? "True" : "False", threshold);
  tft.setCursor(0, 0);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE);
  tft.fillScreen(ST77XX_BLACK);
  tft.print(parsed);
  delay(200);
}