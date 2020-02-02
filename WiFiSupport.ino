/*
 * WiFiSupport.ino
 *
 * Support for ESPxxx on-board WiFi
 *
 * (c) 2019 Lee Dowthwaite. All Rights Reserved.
 */

#include "WiFi.h"

// WiFi Credentials
static const char *_ssid = NULL;
static const char *_password = NULL;

// WiFi status
//static bool _wifiConnected = false;

// connect to wifi – returns true if successful or false if not
//
static boolean connectWifi(const char *ssid, const char *password) {
  boolean success = true;
  int i = 0;

  WiFi.disconnect(true);
  delay(500);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoConnect(true);
  WiFi.begin(ssid, password);
  Serial.println( "Connecting to WiFi...");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    if (i > 10) {
      success = false;
      break;
    }
    i++;
  }
  return success;
}

bool isWiFiConnected() {
  return WiFi.status() == WL_CONNECTED;
}


static void displayConnectionStatus() {
  //clearDisplay();
  if (isWiFiConnected()) {
    Serial.println( "Connected to:");
    Serial.println( _ssid);
    Serial.println( "IP address:");
    Serial.println( String(WiFi.localIP()).c_str());
  } else {
    Serial.println( "Connection failed.");
  }
}


void initWiFi(const char *ssid, const char *password) {
  if (connectWifi(ssid, password)) {
    _ssid = ssid;
    _password = password;
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Connection failed.");
  }
  displayConnectionStatus();
  delay(1000);
}

void checkWiFiStatus() {
  if (!isWiFiConnected() && _ssid && _password) {
    connectWifi(_ssid, _password);
    displayConnectionStatus();
    delay(1000);
  }
}
