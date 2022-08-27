#define ETL_NO_STL
#include <WiFi.h>
#include <esp_now.h>
#include <ArduinoJson.h>
#include <Embedded_Template_Library.h>
#include <etl/array.h>

// using mac_t = etl::array<uint8_t, 6>;

// struct Hub {
//   mac_t mac_address;
//   esp_now_peer_info_t peer_info;
//   bool peer_exists;
//   bool peer_connected;
// };

// Hub hub;

// void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
//   char macStr[18];
//   Serial.print("Packet to: ");
//   // Copies the sender mac address to a string
//   snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
//            mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
//   Serial.print(macStr);
//   Serial.print(" send status:\t");
//   Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
// }

// void setup() {
//   Serial.begin(115200);
//   while (!Serial)
//     ;
//   Serial.println("Booted");

//   hub.mac_address = { 0xF4, 0x12, 0xFA, 0x59, 0x6A, 0x64 };

//   //Set device as a Wi-Fi Station
//   WiFi.mode(WIFI_STA);

//   if (esp_now_init() != ESP_OK) {
//     Serial.println("Error initializing ESP-NOW");
//     while (1) continue;
//   } else {
//     Serial.println("ESP Now initialized successfully");
//   }

//   esp_now_register_send_cb(OnDataSent);
//   // esp_now_register_recv_cb(OnDataRecv);

//   {
//     memcpy(hub.peer_info.peer_addr, hub.mac_address.data(), hub.mac_address.size());
//     hub.peer_info.channel = 0;
//     hub.peer_info.encrypt = false;
//     hub.peer_exists = esp_now_is_peer_exist(hub.mac_address.data());
//     char str_buf[40];
//     snprintf(str_buf, sizeof(str_buf), "Found peer: %02x:%02x:%02x:%02x:%02x:%02x\n", hub.mac_address[0], hub.mac_address[1], hub.mac_address[2], hub.mac_address[3], hub.mac_address[4], hub.mac_address[5]);
//     Serial.print(str_buf);
//   }
//   Serial.println("Finished searching for peers");

//   {
//     char str_buf[60];
//     if (hub.peer_exists) {
//       if (esp_now_add_peer(&hub.peer_info) == ESP_OK) {
//         snprintf(str_buf, sizeof(str_buf), "Added peer: %02x:%02x:%02x:%02x:%02x:%02x", hub.mac_address[0], hub.mac_address[1], hub.mac_address[2], hub.mac_address[3], hub.mac_address[4], hub.mac_address[5]);
//       } else {
//         snprintf(str_buf, sizeof(str_buf), "FAILED TO ADD EXISTING PEER: %02x:%02x:%02x:%02x:%02x:%02x\n", hub.mac_address[0], hub.mac_address[1], hub.mac_address[2], hub.mac_address[3], hub.mac_address[4], hub.mac_address[5]);
//       }
//       Serial.print(str_buf);
//     }
//   }
//   Serial.println("Finished adding peers");
// }

// void loop() {
//   StaticJsonDocument<100> testDocument;

//   testDocument["globalReset"] = false;
//   testDocument["localReset"] = true;
//   testDocument["threshold"] = random(0, 120);

//   uint8_t buffer[100];

//   int bytesWritten = serializeMsgPack(testDocument, buffer);
//   for(int i = 0; i<bytesWritten; i++){
//     Serial.printf("%02X ",buffer[i]);
//   }
//   Serial.println("Finished serializing");
//   esp_err_t result = esp_now_send(hub.mac_address.data(), buffer, bytesWritten);
//   if (result == ESP_OK)
//     Serial.println("Sent data successfully!");
//   else {
//     Serial.print("Error sending the data: ");
//     Serial.print(result);
//   }
//   delay(5000);
// }




///*********
//  Rui Santos
//  Complete project details at https://RandomNerdTutorials.com/esp-now-many-to-one-esp32/
//  
//  Permission is hereby granted, free of charge, to any person obtaining a copy
//  of this software and associated documentation files.
//  
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//*********/
//
//// Reciever code
//
//#include <esp_now.h>
//#include <WiFi.h>
//
//// Structure example to receive data
//// Must match the sender structure
//typedef struct struct_message {
//  int id;
//  int x;
//  int y;
//}struct_message;
//
//// Create a struct_message called myData
//struct_message myData;
//
//// Create a structure to hold the readings from each board
//struct_message board1;
//struct_message board2;
//struct_message board3;
//
//// Create an array with all the structures
//struct_message boardsStruct[1] = {board1};
//
//// callback function that will be executed when data is received
//void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
//  char macStr[18];
//  Serial.print("Packet received from: ");
//  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
//           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
//  Serial.println(macStr);
//  memcpy(&myData, incomingData, sizeof(myData));
//  Serial.printf("Board ID %u: %u bytes\n", myData.id, len);
//  // Update the structures with the new incoming data
//  boardsStruct[myData.id-1].x = myData.x;
//  boardsStruct[myData.id-1].y = myData.y;
//  Serial.printf("x value: %d \n", boardsStruct[myData.id-1].x);
//  Serial.printf("y value: %d \n", boardsStruct[myData.id-1].y);
//  Serial.println();
//}
// 
//void setup() {
//  //Initialize Serial Monitor
//  Serial.begin(115200);
//  
//  //Set device as a Wi-Fi Station
//  WiFi.mode(WIFI_STA);
//
//  //Init ESP-NOW
//  if (esp_now_init() != ESP_OK) {
//    Serial.println("Error initializing ESP-NOW");
//    return;
//  }
//  
//  // Once ESPNow is successfully Init, we will register for recv CB to
//  // get recv packer info
//  esp_now_register_recv_cb(OnDataRecv);
//}
// 
//void loop() {
//  // Acess the variables for each board
//  /*int board1X = boardsStruct[0].x;
//  int board1Y = boardsStruct[0].y;
//  int board2X = boardsStruct[1].x;
//  int board2Y = boardsStruct[1].y;
//  int board3X = boardsStruct[2].x;
//  int board3Y = boardsStruct[2].y;*/
//
//  delay(10000);  
//}












#include <Wire.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

Adafruit_LIS3DH lis;

#define CLICKTHRESHHOLD 180

uint8_t broadcastAddress[] = {0xF4, 0x12, 0xFA, 0x59, 0x6A, 0x64};

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  int id;
  int x;
  int y;
}struct_message;

struct_message board1;

struct_message boardsStruct[1] = {board1};

// Create a struct_message called myData
struct_message myData;
struct_message otherData;

// Create peer interface
esp_now_peer_info_t peerInfo;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  char macStr[18];
  Serial.print("Packet received from: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.printf("Board ID %u: %u bytes\n", myData.id, len);
  // Update the structures with the new incoming data
  boardsStruct[myData.id-1].x = myData.x;
  boardsStruct[myData.id-1].y = myData.y;
  Serial.printf("x value: %d \n", boardsStruct[myData.id-1].x);
  Serial.printf("y value: %d \n", boardsStruct[myData.id-1].y);
  Serial.println();
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  //Initialize Serial Monitor
  Serial.begin(115200);
  
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
  esp_now_register_recv_cb(OnDataRecv);

    // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  lis = Adafruit_LIS3DH();
  lis.begin(0x18);
  lis.setRange(LIS3DH_RANGE_4_G);   // 2, 4, 8 or 16 G!
  lis.setClick(1, CLICKTHRESHHOLD);
}

int thres = 0;
 
void loop() {
  // // Set values to send
  // myData.id = 2;
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
  esp_err_t result = esp_now_send(broadcastAddress, buffer, bytesWritten);
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
  delay(1000);
}

