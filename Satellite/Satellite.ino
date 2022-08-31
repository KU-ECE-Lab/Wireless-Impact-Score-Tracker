// #define ETL_NO_STL
#include <WiFi.h>
#include <esp_now.h>
#include <ArduinoJson.h>
#include <Embedded_Template_Library.h>
#include <etl/array.h>
#include <etl/atomic.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
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

Adafruit_NeoPixel pixel(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// etl::atomic<uint16_t> number_of_taps;
uint16_t number_of_taps;
uint16_t threshold = 20;
uint16_t poll_delay = 20;

void TaskCheckTaps(void* pvParameters) {
  (void) pvParameters;
  for (;;) {
    auto click = lis.getClick();
    if (click & 0x10 && !(click == 0) && (click & 0x30)){
      digitalWrite(LED_BUILTIN, HIGH);
      ++number_of_taps;
    }
    delay(poll_delay / 2);
    digitalWrite(LED_BUILTIN, LOW);
    delay(poll_delay / 2);
  }
}

void TaskSendData(void* pvParameters) {
  (void) pvParameters;
  for (;;) {
  StaticJsonDocument<200> testDocument;

Serial.print(WiFi.macAddress());

  testDocument["tapCount"] = number_of_taps;
  Debug.print(DBG_INFO, "Count: %d\n", number_of_taps);

  uint8_t buffer[200];

  int bytesWritten = serializeMsgPack(testDocument, buffer);
  for(int i = 0; i<bytesWritten; i++){
    Serial.printf("%02X ",buffer[i]);
  }
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

StaticJsonDocument<300> incoming_json;

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incoming_data, int len) {
  Debug.print(DBG_INFO, "Packet Received\n");
  Debug.print(DBG_DEBUG, "From: %02X:%02X:%02X:%02X:%02X:%02X\n", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

  Serial.println("Data received: ");
  for(int i = 0; i<len; i++){
    Serial.printf("%02X ",incoming_data[i]);
  }

  mac_t address = {mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]};
  if (address != hub.mac_address) Debug.print(DBG_ERROR, "Packet received from MAC different than hub\n");

  DeserializationError error = deserializeMsgPack(incoming_json, incoming_data); // Deserialize the MessagePack data into the JSON doc
  if (error) Debug.print(DBG_ERROR, "deserializeMsgPack() failed: %s\n", error.f_str()); // Print any error if one occurs

  if (incoming_json["localReset"] || incoming_json["globalReset"]) number_of_taps = 0;
  if (incoming_json["identify"]) {
    for (uint16_t hsv = 0; hsv <= UINT16_MAX - 35; hsv+=35) {
      auto color = pixel.ColorHSV(hsv);
      pixel.fill(color);
      pixel.show();
    }
    pixel.clear();
    pixel.show();
  }
  threshold = incoming_json["threshold"];
  poll_delay = incoming_json["pollDelay"];
  lis.setClick(1, threshold);
}

void setup() {
  //Initialize Serial Monitor
  Serial.begin(115200);
  Debug.setDebugOutputStream(&Serial);
  Debug.setDebugLevel(DBG_VERBOSE);
  Debug.newlineOff();

  pinMode(LED_BUILTIN, OUTPUT);

  pixel.setBrightness(50);

  //Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

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