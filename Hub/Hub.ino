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

#include "SerialTransfer.h"

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

SerialTransfer my_transfer;

// Use dedicated hardware SPI pins
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

Satellite satellite_0 = { { 0xF4, 0x12, 0xFA, 0x5A, 0x1B, 0xE0 } };
Satellite satellite_1 = { { 0xF4, 0x12, 0xFA, 0x5A, 0x10, 0x90 } };
Satellite satellite_2 = { { 0xF4, 0x12, 0xFA, 0x5A, 0x11, 0xC4 } };
Satellite satellite_3 = { { 0xF4, 0x12, 0xFA, 0x59, 0x96, 0x70 } };
etl::array<Satellite, 4> satellites = { satellite_0, satellite_1, satellite_2, satellite_3 };

#define VAR_INIT(num) \
  uint8_t range_threshold_##num = 1; \
  GEMItem menu_item_range_##num("Range:", range_threshold_##num); \
  uint8_t course_threshold_##num = 4; \
  GEMItem menu_item_course_##num("Course:", course_threshold_##num); \
  uint8_t fine_threshold_##num = 0; \
  GEMItem menu_item_fine_##num("Fine:", fine_threshold_##num); \
  int poll_delay_##num = 90; \
  GEMItem menu_item_delay_##num("Poll Delay:", poll_delay_##num); \
  int tap_count_##num; \
  GEMItem menu_item_count_##num("Count:", tap_count_##num); \
  GEMItem menu_item_count_main_##num("Count M:", tap_count_##num); \
  bool modified_##num = false; \
  bool identify_##num = false; \
  void Identify##num() { \
    identify_##num = true; \
    modified_##num = true; \
  } \
  GEMItem menu_item_identify_##num("Identify", Identify##num); \
  bool local_reset_##num = false; \
  void ResetCount##num() { \
    local_reset_##num = true; \
    modified_##num = true; \
  } \
  GEMItem menu_item_reset_##num("Reset Count", ResetCount##num); \
  GEMPage menu_page_satellite_settings_##num("Settings"); \
  GEMItem menu_item_main_settings_##num("Settings", menu_page_satellite_settings_##num);

//*******************************************************************************************************************
VAR_INIT(0)
VAR_INIT(1)
VAR_INIT(2)
VAR_INIT(3)

void GlobalReset() {
  ResetCount0();
  ResetCount1();
  ResetCount2();
  ResetCount3();
}
GEMItem menu_item_global_reset("RESET ALL", GlobalReset);

GEMPage menuPageMain("Main Menu");  // Main page

inline void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incoming_data, int len) {
  Debug.print(DBG_INFO, "Packet Received\n");
  Debug.print(DBG_DEBUG, "From: %02X:%02X:%02X:%02X:%02X:%02X\n", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

  mac_t address = { mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5] };
  auto it = etl::find_if(satellites.begin(), satellites.end(), [&](const Satellite &sat) {
    return sat.mac_address == address;
  });

  auto index = it - satellites.begin();

  for (int i = 0; i < len; i++) {
    Debug.print(DBG_VERBOSE, "%02X ", incoming_data[i]);
  }

  if (it == satellites.end()) {
    Debug.print(DBG_ERROR, "Failed to find registered satellite with the MAC address received\n");
    return;
  }
  // DeserializationError error = deserializeMsgPack(it->json_doc, incoming_data);           // Deserialize the MessagePack data into the JSON doc
  DeserializationError error = deserializeMsgPack(satellites[index].json_doc, incoming_data);  // Deserialize the MessagePack data into the JSON doc
  if (error) Debug.print(DBG_ERROR, "deserializeMsgPack() failed: %s\n", error.f_str());       // Print any error if one occurs
  int count = satellites[index].json_doc["tapCount"];
  static int count_0 = 0;
  static int count_1 = 0;
  static int count_2 = 0;
  static int count_3 = 0;

      uint16_t send_size = 0;
    send_size = my_transfer.txObj(count_0, send_size);
    send_size = my_transfer.txObj(count_1, send_size);
    send_size = my_transfer.txObj(count_2, send_size);
    send_size = my_transfer.txObj(count_3, send_size);
    my_transfer.sendData(send_size);


#define TAP_COUNT_SET(num) \
  case num: \
    menu_item_count_##num.setReadonly(false); \
    menu_item_count_main_##num.setReadonly(false); \
    tap_count_##num = count; \
    count_##num = count;\
    menu_item_count_##num.setReadonly(true); \
    menu_item_count_main_##num.setReadonly(true); \
    break;

  switch (index) {
    //*******************************************************************************************************************
    TAP_COUNT_SET(0)
    TAP_COUNT_SET(1)
    TAP_COUNT_SET(2)
    TAP_COUNT_SET(3)
  }
  Debug.print(DBG_INFO, "Tap count: %d\n", count);
}

// callback when data is sent
inline void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Debug.print(DBG_INFO, "\r\nLast Packet Send Status:\t");
  Debug.print(DBG_INFO, status == ESP_NOW_SEND_SUCCESS ? "Delivery Success\n" : "Delivery Fail\n");
}

GEM_adafruit_gfx menu(tft, GEM_POINTER_ROW, GEM_ITEMS_COUNT_AUTO);

void setupMenu() {
// Add menu items to Settings menu page
#define ADD_MENUS(num) \
  menu_page_satellite_settings_##num.addMenuItem(menu_item_count_##num); \
  menu_page_satellite_settings_##num.addMenuItem(menu_item_identify_##num); \
  menu_page_satellite_settings_##num.addMenuItem(menu_item_reset_##num); \
  menu_page_satellite_settings_##num.addMenuItem(menu_item_range_##num); \
  menu_page_satellite_settings_##num.addMenuItem(menu_item_course_##num); \
  menu_page_satellite_settings_##num.addMenuItem(menu_item_fine_##num); \
  menu_page_satellite_settings_##num.addMenuItem(menu_item_delay_##num); \
  menu_item_count_##num.setReadonly(true); \
  menuPageMain.addMenuItem(menu_item_main_settings_##num); \
  menuPageMain.addMenuItem(menu_item_count_main_##num); \
  menu_page_satellite_settings_##num.setParentMenuPage(menuPageMain);

  menuPageMain.addMenuItem(menu_item_global_reset);

  //*******************************************************************************************************************
  ADD_MENUS(0)
  ADD_MENUS(1)
  ADD_MENUS(2)
  ADD_MENUS(3)

  // Add Main Menu page to menu and set it as current
  menu.setMenuPageCurrent(menuPageMain);
}

struct __attribute__((packed)) STRUCT {
  char z;
  double y;
} testStruct;


void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
  my_transfer.begin(Serial);
  Debug.setDebugOutputStream(&Serial);
  Debug.setDebugLevel(DBG_NONE);
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
    // Serial.println("Couldn't find seesaw on default address");
    while (1) delay(10);
  }
  // Serial.println("seesaw started");

  uint32_t version = ((ss.getVersion() >> 16) & 0xFFFF);
  if (version != 4991) {
    // Serial.print("Wrong firmware loaded? ");
    // Serial.println(version);
    while (1) delay(10);
  }
  // Serial.println("Found Product 4991");

  // set not so bright!
  sspixel.setBrightness(20);
  sspixel.show();

  // use a pin for the built in encoder switch
  ss.pinMode(SS_SWITCH, INPUT_PULLUP);

  // get starting position
  encoder_position = ss.getEncoderPosition();

  // Serial.println("Turning on interrupts");
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
      // menu.drawMenu();
    }
  }

#define CHECK_AND_SEND(num) \
  if (modified_##num) { \
    auto threshold = static_cast<int>(range_threshold_##num) * 100 + static_cast<int>(range_threshold_##num) * 10 + static_cast<int>(fine_threshold_##num); \
    satellites[num].json_doc["threshold"] = threshold; \
    satellites[num].json_doc["localReset"] = local_reset_##num; \
    satellites[num].json_doc["globalReset"] = false; \
    satellites[num].json_doc["identify"] = identify_##num; \
    satellites[num].json_doc["pollDelay"] = poll_delay_##num; \
\
    uint8_t buffer[300]; \
    Debug.print(DBG_VERBOSE, "JSON Parsing Successful\n"); \
\
    int bytesWritten = serializeMsgPack(satellites[num].json_doc, buffer); \
    for (int i = 0; i < bytesWritten; i++) { \
      Debug.print(DBG_VERBOSE, "%02X ", buffer[i]); \
    } \
    Debug.print(DBG_VERBOSE, "\n"); \
    esp_err_t result = esp_now_send(satellites[num].mac_address.data(), buffer, bytesWritten); \
    if (result == ESP_OK) { \
      Debug.print(DBG_VERBOSE, "Sent data successfully!: "); \
      Debug.print(DBG_VERBOSE, "%d\n", num); \
    } else { \
    Debug.print(DBG_VERBOSE, "Error sending the data: "); \
      Debug.print(DBG_VERBOSE, "%d\n", result); \
    } \
    modified_##num = false; \
    identify_##num = false; \
    local_reset_##num = false; \
  }

  //*******************************************************************************************************************
  CHECK_AND_SEND(0)
  CHECK_AND_SEND(1)
  CHECK_AND_SEND(2)
  CHECK_AND_SEND(3)

  delay(10);
}