#define ETL_NO_STL

#include <esp_now.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <Embedded_Template_Library.h>
#include <etl/array.h>

// using mac_t = etl::array<uint8_t, 6>;

// struct Satellite {
//   mac_t mac_address;
//   esp_now_peer_info_t peer_info;
//   bool peer_exists;
//   bool peer_connected;
// };

//  void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
//    Serial.println("Recieved!");
//    char macStr[18];
//    Serial.print("Packet received from: ");
//    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
//    Serial.println(macStr);

//    StaticJsonDocument<100> doc;
// //   memcpy(buf, incomingData, len);
//    DeserializationError error = deserializeMsgPack(doc, incomingData);

//   if (error) {
//     Serial.print("deserializeMsgPack() failed: ");
//     Serial.println(error.f_str());
//     return;
//   }

//   bool global_reset = doc["globalReset"];
//   bool local_reset = doc["localReset"];
//   int threshold = doc["threshold"];

//   Serial.printf("Global Reset: %s Local Reset: %s Threshold: %d\n", global_reset, local_reset, threshold);
   
// //   Serial.printf("Board ID %u: %u bytes\n", myData.id, len);
//    // Update the structures with the new incoming data
// //   boardsStruct[myData.id-1].x = myData.x;
// //   boardsStruct[myData.id-1].y = myData.y;
// //   Serial.printf("x value: %d \n", boardsStruct[myData.id-1].x);
// //   Serial.printf("y value: %d \n", boardsStruct[myData.id-1].y);
//    Serial.println();
//  }

// constexpr Satellite satellite_0 = {{0x4C, 0xEB, 0xD6, 0x7B, 0x02, 0x90}};

// etl::array<Satellite, 1> satellites = {satellite_0};

// void setup() {
//   Serial.begin(115200);
//   while(!Serial);
  
//   //Set device as a Wi-Fi Station
//   WiFi.mode(WIFI_STA);

//   if (esp_now_init() != ESP_OK) {
//     Serial.println("Error initializing ESP-NOW");
//     while(1) continue;
//   } else {
//     Serial.println("ESP Now initialized");
//   }
  
//   // esp_now_register_send_cb(OnDataSent);
//    esp_now_register_recv_cb(OnDataRecv);
//    Serial.println("Registered callback");

//   // for (auto& peer : satellites) {
//   //   memcpy(peer.peer_info.peer_addr, peer.mac_address.data(), peer.mac_address.size());
//   //   peer.peer_info.channel = 0;
//   //   peer.peer_info.encrypt = false;
//   //   peer.peer_exists = esp_now_is_peer_exist(peer.mac_address.data());

//   //   char str_buf[40];
//   //   snprintf(str_buf, sizeof(str_buf), "Found peer: %02x:%02x:%02x:%02x:%02x:%02x\n", peer.mac_address[0], peer.mac_address[1], peer.mac_address[2], peer.mac_address[3], peer.mac_address[4], peer.mac_address[5]);
//   //   Serial.print(str_buf);
//   // }
//   // Serial.println("Finished finding peers");

//   // for (auto& peer : satellites) {
//   //   char str_buf[60];
//   //   if (peer.peer_exists) {
//   //     if (esp_now_add_peer(&peer.peer_info) == ESP_OK) {
//   //       snprintf(str_buf, sizeof(str_buf), "Added peer: %02x:%02x:%02x:%02x:%02x:%02x", peer.mac_address[0], peer.mac_address[1], peer.mac_address[2], peer.mac_address[3], peer.mac_address[4], peer.mac_address[5]);
//   //     } else {
//   //       snprintf(str_buf, sizeof(str_buf), "FAILED TO ADD EXISTING PEER: %02x:%02x:%02x:%02x:%02x:%02x\n", peer.mac_address[0], peer.mac_address[1], peer.mac_address[2], peer.mac_address[3], peer.mac_address[4], peer.mac_address[5]);
//   //     }
//   //     Serial.print(str_buf);
//   //   }
//   // }
//   // Serial.println("Finished adding peers");
// }
 
// void loop() {
// }





// Sender Code

#include <esp_now.h>
#include <WiFi.h>


// REPLACE WITH THE RECEIVER'S MAC Address
uint8_t broadcastAddress[] = {0xF4, 0x12, 0xFA, 0x5A, 0x1B, 0xE0};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
    int id; // must be unique for each sender board
    int x;
    int y;
} struct_message;

struct_message board1;

struct_message boardsStruct[1] = {board1};

// Create a struct_message called myData
struct_message myData;
struct_message otherData;

// Create peer interface
esp_now_peer_info_t peerInfo;

// // callback function that will be executed when data is received
// void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
//   char macStr[18];
//   Serial.print("Packet received from: ");
//   snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
//            mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
//   Serial.println(macStr);
//   memcpy(&myData, incomingData, sizeof(myData));
//   Serial.printf("Board ID %u: %u bytes\n", myData.id, len);
//   // Update the structures with the new incoming data
//   boardsStruct[myData.id-1].x = myData.x;
//   boardsStruct[myData.id-1].y = myData.y;
//   Serial.printf("x value: %d \n", boardsStruct[myData.id-1].x);
//   Serial.printf("y value: %d \n", boardsStruct[myData.id-1].y);
//   Serial.println();
// }

 void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
   Serial.println("Recieved!");
   char macStr[18];
   Serial.print("Packet received from: ");
   snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
   Serial.println(macStr);

   StaticJsonDocument<200> doc;
//   memcpy(buf, incomingData, len);
   DeserializationError error = deserializeMsgPack(doc, incomingData);

  if (error) {
    Serial.print("deserializeMsgPack() failed: ");
    Serial.println(error.f_str());
    return;
  } else {
    Serial.println("Deserialization Successful");
  }

  bool global_reset = doc["globalReset"];
  bool local_reset = doc["localReset"];
  int threshold = doc["threshold"];

  Serial.println("Parsing Successful");

  Serial.printf("Global Reset: %s Local Reset: %s Threshold: %d\n\n", (global_reset) ? "True" : "False", (local_reset) ? "True" : "False", threshold);
 }

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

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
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}
 
void loop() {
  // // Set values to send
  // myData.id = 1;
  // myData.x = random(0,50);
  // myData.y = random(0,50);

  // // Send message via ESP-NOW
  // esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  // if (result == ESP_OK) {
  //   Serial.println("Sent with success");
  // }
  // else {
  //   Serial.println("Error sending the data");
  // }
  // delay(10000);
}
