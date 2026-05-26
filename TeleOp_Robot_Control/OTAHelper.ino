// ===========================
// OTAHelper.ino
// WiFi + OTA setup
// ===========================

#include <WiFi.h>
#include <ArduinoOTA.h>

const char* WIFI_SSID     = "N300-EEF9";
const char* WIFI_PASSWORD = "123456789";
const char* OTA_PASSWORD  = "password";
const char* HOSTNAME      = "your-name-robot";

void setupOTA() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected! IP: " + WiFi.localIP().toString());

  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.begin();
  Serial.println("OTA ready");
}

void handleOTA() {
  ArduinoOTA.handle();
}
