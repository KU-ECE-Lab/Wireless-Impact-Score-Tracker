// #define ETL_NO_STL
#include <Embedded_Template_Library.h>
#include <etl/array.h>
#include <etl/atomic.h>
#include <etl/utility.h>
#include <etl/algorithm.h>
#include <esp_now.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <Adafruit_GFX.h>     // Core graphics library
#include <Adafruit_ST7789.h>  // Hardware-specific library for ST7789
#include <Arduino_DebugUtils.h>

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
  bool peer_connected;
  float battery_voltage;
  float battery_percent;
  StaticJsonDocument<512> json_doc;
};

bool operator==(const Satellite& lhs, const Satellite& rhs) {
  return lhs.mac_address == rhs.mac_address;
}

Satellite satellite_0 = { { 0xF4, 0x12, 0xFA, 0x5A, 0x1B, 0xE0 } };

etl::array<Satellite, 1> satellites = { satellite_0 };

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incoming_data, int len) {
  Debug.print(DBG_INFO, "Packet Received\n");
  Debug.print(DBG_DEBUG, "From: %02X:%02X:%02X:%02X:%02X:%02X\n", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

  mac_t address = {mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]};
  auto it = etl::find_if(satellites.begin(), satellites.end(), [&](const Satellite& sat){return sat.mac_address == address;});

  if (it == satellites.end()) {
    Debug.print(DBG_ERROR, "Failed to find registered satellite with the MAC address received\n");
    return;
  }
  DeserializationError error = deserializeMsgPack(it->json_doc, incoming_data); // Deserialize the MessagePack data into the JSON doc
  if (error) Debug.print(DBG_ERROR, "deserializeMsgPack() failed: %s\n", error.f_str()); // Print any error if one occurs
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
  Debug.setDebugLevel(DBG_INFO);
  Debug.newlineOff();

  Debug.print(DBG_INFO, "MAC: %s", WiFi.macAddress());

  // initialize TFT
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);
  tft.init(135, 240);  // Init ST7789 240x135
  tft.setRotation(3);
  tft.fillScreen(ST77XX_BLACK);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) { // Make sure ESP Now is initialized correctly
    Debug.print(DBG_ERROR, "Error initializing ESP-NOW");
    return;
  } else {
    Debug.print(DBG_VERBOSE, "Successfully initialized ESP-NOW");
  }

  esp_now_register_recv_cb(OnDataRecv); // Register the function to call when data is received via ESP Now
  // esp_now_register_send_cb(OnDataSent);

  for (auto &peer : satellites) { // Go through each satellite in the satellite list
    memcpy(peer.peer_info.peer_addr, peer.mac_address.data(), peer.mac_address.size()); // Populate its peer_info mac address
    peer.peer_info.channel = 0; // Set the channel
    peer.peer_info.encrypt = false; // Do not encrypt the messages

      if (esp_now_add_peer(&peer.peer_info) == ESP_OK) { // Add the peer and check if there were any issues
        peer.peer_connected = true;
        Debug.print(DBG_DEBUG, "Added peer: %02x:%02x:%02x:%02x:%02x:%02x\n", peer.mac_address[0], peer.mac_address[1], peer.mac_address[2], peer.mac_address[3], peer.mac_address[4], peer.mac_address[5]);
      } else {
        peer.peer_connected = false;
        Debug.print(DBG_ERROR, "Failed to add peer: %02x:%02x:%02x:%02x:%02x:%02x\n", peer.mac_address[0], peer.mac_address[1], peer.mac_address[2], peer.mac_address[3], peer.mac_address[4], peer.mac_address[5]);
      }
  }
  Debug.print(DBG_VERBOSE, "Finished adding peers\n");
}

void loop() {
  // bool global_reset = json_doc.second["globalReset"];
  // bool local_reset = json_doc.second["localReset"];
  // int threshold = json_doc.second["threshold"];
  // Debug.print(DBG_VERBOSE, "JSON Parsing Successful\n");

  // char parsed[200];
  // sprintf(parsed, "Global Reset: %s Local Reset: %s Threshold: %d\n\n", (global_reset) ? "True" : "False", (local_reset) ? "True" : "False", threshold);
  // tft.setCursor(0, 0);
  // tft.setTextSize(2);
  // tft.setTextColor(ST77XX_WHITE);
  // tft.fillScreen(ST77XX_BLACK);
  // tft.print(parsed);
  delay(200);
}