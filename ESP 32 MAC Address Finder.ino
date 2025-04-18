#include <WiFi.h>
#include <esp_wifi.h>

void setup() {
  Serial.begin(115200);
  delay(2000); // Wait for the board to catch up

  // Initialize WiFi
  WiFi.mode(WIFI_STA);

  // Retrieve and print the WiFi MAC address
  uint8_t macWiFi[6];
  esp_wifi_get_mac(WIFI_IF_STA, macWiFi);
  Serial.print("WiFi MAC Address: ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", macWiFi[i]);
    if (i < 5) {
      Serial.print(":");
    }
  }
  Serial.println();

  // Retrieve and print the Base MAC address
  uint8_t macBase[6];
  esp_wifi_get_mac(WIFI_IF_AP, macBase);
  Serial.print("Base MAC Address: ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", macBase[i]);
    if (i < 5) {
      Serial.print(":");
    }
  }
  Serial.println();

  // Retrieve and print the BLE MAC address
  uint8_t macBLE[6];
  esp_wifi_get_mac(WIFI_IF_AP, macBLE);
  Serial.print("BLE MAC Address: ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", macBLE[i]);
    if (i < 5) {
      Serial.print(":");
    }
  }
  Serial.println();
}

void loop() {
  // Do nothing here
}
