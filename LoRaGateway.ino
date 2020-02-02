/*
 * LoRaGateway.ino
 *
 * Implements a minimal, single-channel LoRa to MQTT gateway
 *
 * (c) 2019 Lee Dowthwaite. All Rights Reserved.
 */

 /*
 * Lora msg tartalmaz ":"-t, ez választja el a topic-ot és a payload-ot
 * ez alapján küldiMQTT üzenetként tovább
 *
 *MQTT üzenetekből a lora/# (mind ami lora-val kezdődik) iratkozik fel
 * ezeket mind továbbítja lora msg-ként, ":"-tal elválasztva a topic-ot és a payload-ot
 * topic:payload
 */

//board: TTGO LoRa32-OLED V1

#include "Arduino.h"
#include <PubSubClient.h>

//Libraries for LoRa
#include <SPI.h>
#include <LoRa.h>

//#include "HAL.h"
#include "LoRaInterface.h"

// WiFi credentials
#define WIFI_SSID     "379ffe"
#define WIFI_PASSWORD "247400171"
#define MQTT_SERVER   "192.168.0.200"

//define the pins used by the LoRa transceiver module
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DIO0 26

//866E6 for Europe
#define BAND 866E6

// checkAndForwardPackets()
// This is the core function that checks for received LoRa packets and forwards the contents on to MQTT
//
unsigned long prev_ms = 0;
static void checkAndForwardPackets() {
  const char *_topic;
  const char *_payload;

  if(millis() - prev_ms > 5000){
    Serial.println("checkAndForwardPackets");
    prev_ms = millis();
  }
  
  // check for received data
  String *rxPacketString = checkRxBuffer();
  if (rxPacketString) {
    Serial.print("rx packet: msg: ");
    
    // forward packet content to MQTT
    const char *msg = rxPacketString->c_str();
    
    char *position_ptr = strchr(msg, ':');
    if(position_ptr != NULL){
      (*position_ptr) = '\0';
      _topic = msg;
      _payload = position_ptr+1;
      publishMQTT(_topic, _payload);
      Serial.print(_topic); Serial.print(":lora:"); Serial.println(_payload);
    } else {
      publishMQTT(msg);
      Serial.println(msg);
    }

    Serial.println(rssi());
  }
}

void callback(char* topic_, byte* payload_, unsigned int length) {
  String inTopic = String( topic_ );
  char buf[200];

  for (int i = 0; i < length; i++) {
    buf[i] = (char)payload_[i];
  }
  buf[length] = '\0';
  String inPayload = String( buf );
  Serial.print( inTopic ); Serial.print(":>:");
  Serial.println( inPayload );

  //Ami lora/ kezdetű, azt mind továbbítjuk a LoRa rádióval
  //++++++++++++++++++++++++++++++
  LoRa.beginPacket();
  LoRa.print(inTopic);
  LoRa.print(":");
  LoRa.print(inPayload);
  LoRa.endPacket();
  LoRa.receive();         // go back into receive mode
  //sendIfReady();
  //++++++++++++++++++++++++++++++
  Serial.println("sent with lora");
  publishMQTT("env/node1/temp", inTopic.c_str() );
}

// Arduino main hooks

void setup() {
  // initialise the board
  //configureBoard();

  Serial.begin(115200);
  Serial.println("setup()");

  pinMode(LED_BUILTIN, OUTPUT);

  //clearDisplay();
  //displayString(0, 0, "Initialising Gateway...");
  Serial.println("Initialising Gateway...");

  // Initialise wifi connection
  initWiFi(WIFI_SSID, WIFI_PASSWORD);

  /*-------------------------------------------------*/
  //SPI LoRa pins
  SPI.begin(SCK, MISO, MOSI, SS);
  //setup LoRa transceiver module
  LoRa.setPins(SS, RST, DIO0);

  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  Serial.println("LoRa Initializing OK!");
  delay(2000);
  /*-------------------------------------------------*/
  // Configure LoRa interface
  configureLoRa();

  if (isWiFiConnected()) {
    connectToMQTTServer(MQTT_SERVER, 1883);
  }

  Serial.println("setup() done");
}


void loop() {

  // ensure WiFi stays connected
  checkWiFiStatus();

  // Perform packet forwarding
  checkAndForwardPackets();

  // MQTT housekeeping
  updateMQTT();

  //delay(100);
}
