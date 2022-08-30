// #define ETL_NO_STL
#include <WiFi.h>
#include <ArduinoJson.h>
#include <Adafruit_GFX.h>     // Core graphics library
#include <Adafruit_ST7789.h>  // Hardware-specific library for ST7789
#include <Arduino_DebugUtils.h>
#include "ESPSetup.h"

#include <GEM_adafruit_gfx.h>
#include "Adafruit_seesaw.h"
#include <seesaw_neopixel.h>

#define SS_SWITCH 24
#define SS_NEOPIX 6

#define SEESAW_ADDR 0x36

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

Adafruit_seesaw ss;
seesaw_NeoPixel sspixel = seesaw_NeoPixel(1, SS_NEOPIX, NEO_GRB + NEO_KHZ800);

int32_t encoder_position;

// Use dedicated hardware SPI pins
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

Satellite satellite_0 = { { 0xF4, 0x12, 0xFA, 0x5A, 0x1B, 0xE0 } };
etl::array<Satellite, 1> satellites = { satellite_0 };

uint8_t range_threshold = 1;
GEMItem menuItemRange("Range:", range_threshold);
uint8_t course_threshold = 4;
GEMItem menuItemCourse("Course:", course_threshold);
uint8_t fine_threshold = 0;
GEMItem menuItemFine("Fine:", fine_threshold);

int poll_delay = 10;
GEMItem menuItemDelay("Poll Delay:", poll_delay);

int sat_count;
GEMItem menuItemCount("Count:", sat_count);


bool identify = false;
bool local_reset = false;
bool modified = false;

void Identify() {
  identify = true;
  modified = true;
}
GEMItem menuItemIdentify("Identify", Identify);

void ResetCount() {
  local_reset = true;
  modified = true;
}
GEMItem menuItemReset("Reset Count", ResetCount);

GEMPage menuPageMain("Main Menu");  // Main page
GEMPage menuPageSettings("Settings");   // Settings submenu

// Create menu item linked to Settings menu page
GEMItem menuItemMainSettings("Settings", menuPageSettings);

inline void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incoming_data, int len) {
  Debug.print(DBG_INFO, "Packet Received\n");
  Debug.print(DBG_DEBUG, "From: %02X:%02X:%02X:%02X:%02X:%02X\n", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

  mac_t address = { mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5] };
  auto it = etl::find_if(satellites.begin(), satellites.end(), [&](const Satellite &sat) {
    return sat.mac_address == address;
  });

  for(int i = 0; i<len; i++){
    Serial.printf("%02X ",incoming_data[i]);
  }

  if (it == satellites.end()) {
    Debug.print(DBG_ERROR, "Failed to find registered satellite with the MAC address received\n");
    return;
  }
  // DeserializationError error = deserializeMsgPack(it->json_doc, incoming_data);           // Deserialize the MessagePack data into the JSON doc
  DeserializationError error = deserializeMsgPack(satellite_0.json_doc, incoming_data);           // Deserialize the MessagePack data into the JSON doc
  if (error) Debug.print(DBG_ERROR, "deserializeMsgPack() failed: %s\n", error.f_str());  // Print any error if one occurs
  int count = satellite_0.json_doc["tapCount"];
  menuItemCount.setReadonly(false);
  sat_count = count;
  menuItemCount.setReadonly(true);
  Debug.print(DBG_INFO, "Tap count: %d\n", count);
}

// callback when data is sent
inline void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}



GEM_adafruit_gfx menu(tft, GEM_POINTER_ROW, GEM_ITEMS_COUNT_AUTO);

void setupMenu() {
  // Add menu items to Settings menu page
  menuPageSettings.addMenuItem(menuItemCount);
  menuPageSettings.addMenuItem(menuItemIdentify);
  menuPageSettings.addMenuItem(menuItemReset);
  menuPageSettings.addMenuItem(menuItemRange);
  menuPageSettings.addMenuItem(menuItemCourse);
  menuPageSettings.addMenuItem(menuItemFine);
  menuPageSettings.addMenuItem(menuItemDelay);

  menuItemCount.setReadonly(true);

  // Add menu items to Main Menu page
  menuPageMain.addMenuItem(menuItemMainSettings);

  // Specify parent menu page for the Settings menu page
  menuPageSettings.setParentMenuPage(menuPageMain);

  // Add Main Menu page to menu and set it as current
  menu.setMenuPageCurrent(menuPageMain);
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

  PeerInit(satellites);

  esp_now_register_recv_cb(OnDataRecv);  // Register the function to call when data is received via ESP Now

  if (!ss.begin(SEESAW_ADDR) || !sspixel.begin(SEESAW_ADDR)) {
    Serial.println("Couldn't find seesaw on default address");
    while (1) delay(10);
  }
  Serial.println("seesaw started");

  uint32_t version = ((ss.getVersion() >> 16) & 0xFFFF);
  if (version != 4991) {
    Serial.print("Wrong firmware loaded? ");
    Serial.println(version);
    while (1) delay(10);
  }
  Serial.println("Found Product 4991");

  // set not so bright!
  sspixel.setBrightness(20);
  sspixel.show();

  // use a pin for the built in encoder switch
  ss.pinMode(SS_SWITCH, INPUT_PULLUP);

  // get starting position
  encoder_position = ss.getEncoderPosition();

  Serial.println("Turning on interrupts");
  delay(10);
  ss.setGPIOInterrupts((uint32_t)1 << SS_SWITCH, 1);
  ss.enableEncoderInterrupt();



  menu.init();
  setupMenu();
  menu.drawMenu();
}

void loop() {
  static bool last_button = false;
  if (menu.readyForKey()) {
    bool button = !ss.digitalRead(SS_SWITCH);
    if (button != last_button) {
      if (button) menu.registerKeyPress(GEM_KEY_OK);
      last_button = button;
            menu.drawMenu();
    }

    int32_t new_position = ss.getEncoderPosition();
    // did we move arounde?
    if (encoder_position != new_position) {
      if (new_position - encoder_position < 0) {
        menu.registerKeyPress(GEM_KEY_UP);
      } else {
        menu.registerKeyPress(GEM_KEY_DOWN);
      }
      encoder_position = new_position;  // and save for next round
    }
    
  }

  if (modified) {
    auto threshold = static_cast<int>(range_threshold) * 100 + static_cast<int>(course_threshold) * 10 + static_cast<int>(fine_threshold);

    satellite_0.json_doc["threshold"] = threshold;
    satellite_0.json_doc["localReset"] = local_reset;
    satellite_0.json_doc["globalReset"] = false;
    satellite_0.json_doc["identify"] = identify;
    satellite_0.json_doc["pollDelay"] = poll_delay;

uint8_t buffer[300];
    Debug.print(DBG_VERBOSE, "JSON Parsing Successful\n");

      int bytesWritten = serializeMsgPack(satellite_0.json_doc, buffer);
  for(int i = 0; i<bytesWritten; i++){
    Serial.printf("%02X ",buffer[i]);
  }
  Serial.println();
  esp_err_t result = esp_now_send(satellite_0.mac_address.data(), buffer, bytesWritten);
  if (result == ESP_OK)
    Serial.println("Sent data successfully!");
  else {
    Serial.print("Error sending the data: ");
    Serial.println(result);
  }
    modified = false;
    identify = false;
    local_reset = false;
  }

  delay(10);
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
}