#ifndef ESPSETUP_H
#define ESPSETUP_H

#include <Embedded_Template_Library.h>
#include <etl/array.h>
#include <etl/atomic.h>
#include <etl/utility.h>
#include <etl/algorithm.h>
#include <esp_now.h>

using mac_t = etl::array<uint8_t, 6>;

struct Satellite {
  mac_t mac_address;
  esp_now_peer_info_t peer_info;
  bool peer_connected;
  float battery_voltage;
  float battery_percent;
  StaticJsonDocument<512> json_doc;
};

inline bool operator==(const Satellite& lhs, const Satellite& rhs) {
  return lhs.mac_address == rhs.mac_address;
}

template <typename T>
inline void PeerInit(T& satellites) {
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) { // Make sure ESP Now is initialized correctly
    Debug.print(DBG_ERROR, "Error initializing ESP-NOW");
    return;
  } else {
    Debug.print(DBG_VERBOSE, "Successfully initialized ESP-NOW");
  }

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

#endif // ESPSETUP_H