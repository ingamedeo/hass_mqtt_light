#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <EEPROM.h>
#include <WiFiUdp.h>

#define EEPROM_ADDR 0
#define EEPROM_ADDR_1 1
#define MQTT_VERSION MQTT_VERSION_3_1_1
byte eeprom_light = 255;
byte eeprom_wifi = 255;

const PROGMEM uint8_t RELAY_PIN = 14;
const PROGMEM uint8_t BUTTON_PIN = 4;
const PROGMEM uint8_t SWITCH_PIN = 13; //Requires external pull-up
const PROGMEM uint8_t WIFI_B_PIN = 3;
const PROGMEM uint8_t WIFI_LED = 12;

// Wifi: SSID and password
const char* WIFI_SSID = "SSID";
const char* WIFI_PASSWORD = "WPA2_PASSWORD";

// MQTT: ID, server IP, port, username and password
const PROGMEM char* MQTT_CLIENT_ID = "bedroom_light";
const PROGMEM char* MQTT_SERVER_IP = "192.168.X.X";
const PROGMEM uint16_t MQTT_SERVER_PORT = 1883;
const PROGMEM char* MQTT_USER = "MQTT_USER";
const PROGMEM char* MQTT_PASSWORD = "MQTT_PASSWORD";

// MQTT: topics
const char* MQTT_LIGHT_STATE_TOPIC = "/home/xxxx/light/status";
const char* MQTT_LIGHT_COMMAND_TOPIC = "/home/xxxx/light/switch";
const char* MQTT_LOG_TOPIC = "/home/xxxx/light/log";
const char* MQTT_PING_TOPIC = "/home/xxxx/light/ping";
//const char* MQTT_LIGHT_ISALIVE_TOPIC = "/home/xxxx/light/isalive";
//const char* MQTT_LIGHT_LASTBOOT_TOPIC = "/home/xxxx/light/lastboot";
//const char* MQTT_LIGHT_LASTEVENT_TOPIC = "/home/xxxx/light/lastevent";

// payloads by default (on/off)
const char* LIGHT_ON = "ON";
const char* LIGHT_OFF = "OFF";

#define SHORT_PRESS_BOUNCE 100 //in milliseconds

int ledState = LOW;
int actLedState = HIGH;
//ACT LED
unsigned long previousMillis = 0;
unsigned long actpreviousMillis = 0;
long actinterval = 200;
long interval = 50;

volatile bool wifiStatus = false;
volatile bool relayStatus = LOW;
volatile bool prevSwitch = HIGH;

//volatile bool timeUpdated = false;
volatile bool pong = false;

WiFiClient wifiClient;
PubSubClient client(wifiClient);

//unsigned int localPort = 2390;

//IPAddress timeServerIP; // time.nist.gov NTP server address
//const char* ntpServerName = "time.nist.gov";

//const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message

//byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets

// A UDP instance to let us send and receive packets over UDP
//WiFiUDP udp;

//char currentTimeData[50];

// function called to publish the state of the light (on/off)
void publishLightState() {
  if (relayStatus) {
    client.publish(MQTT_LIGHT_STATE_TOPIC, LIGHT_ON, true);
  } else {
    client.publish(MQTT_LIGHT_STATE_TOPIC, LIGHT_OFF, true);
  }
}

// function called to turn on/off the light
void setLightState() {
  if (relayStatus) {
    digitalWrite(RELAY_PIN, HIGH);
    write_to_EEPROM(EEPROM_ADDR, 1); //Update light status on EEPROM
    //Serial.println("INFO: Turn light on...");
  } else {
    digitalWrite(RELAY_PIN, LOW);
    write_to_EEPROM(EEPROM_ADDR, 0); //Update light status on EEPROM
    //Serial.println("INFO: Turn light off...");
  }
  EEPROM.commit();
}

// function called when a MQTT message arrived
void callback(char* p_topic, byte* p_payload, unsigned int p_length) {
  // concat the payload into a string
  String payload;
  for (uint8_t i = 0; i < p_length; i++) {
    payload.concat((char)p_payload[i]);
  }

  // handle message topic
  if (String(MQTT_LIGHT_COMMAND_TOPIC).equals(p_topic)) {
    // test if the payload is equal to "ON" or "OFF"
    if (payload.equals(String(LIGHT_ON))) {
      if (relayStatus != true) {
        relayStatus = true;
        setLightState();
        publishLightState();
      }
    } else if (payload.equals(String(LIGHT_OFF))) {
      if (relayStatus != false) {
        relayStatus = false;
        setLightState();
        publishLightState();
      }
    }

    LogOverMQTT("MQTT message: Switching light on/off!");

  } else if (String(MQTT_PING_TOPIC).equals(p_topic)) {
    pong = true;
  }
}

void reconnect() {
  //Serial.print("INFO: Attempting MQTT connection...");
  // Attempt to connect
  if (client.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD)) {
    //Serial.println("INFO: connected");
    // Once connected, publish an announcement...
    publishLightState();
    // ... and resubscribe
    client.subscribe(MQTT_LIGHT_COMMAND_TOPIC);
    client.subscribe(MQTT_PING_TOPIC);

  } else {
    //Serial.print("ERROR: failed, rc=");
    //Serial.print(client.state());
    //Serial.println("DEBUG: try again in 5 seconds");
    // Wait 5 seconds before retrying
  }
}

void setup() {
  // init the //Serial
  //Serial.begin(115200);

  //Serial.println("Ready");

  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, actLedState);
  pinMode(WIFI_LED, OUTPUT);
  digitalWrite(WIFI_LED, LOW);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(WIFI_B_PIN, INPUT_PULLUP);
  pinMode(SWITCH_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  analogWriteRange(255);

  //Set initial switch value
  prevSwitch = digitalRead(SWITCH_PIN);

  /* Reload status from EEPROM after reboot */
  EEPROM.begin(8);
  eeprom_light = EEPROM.read(EEPROM_ADDR);
  eeprom_wifi = EEPROM.read(EEPROM_ADDR_1);

  if (eeprom_light == 0) {
    relayStatus = LOW;
  } else if (eeprom_light == 1) {
    relayStatus = HIGH;
  }
  setLightState();
  /* Reload status from EEPROM after reboot */

  /* Reload WiFi status from EEPROM */
  if (eeprom_wifi == 0) {
    switchWiFi(false);
  } else if (eeprom_wifi == 1) {
    switchWiFi(true);
  } else {
    switchWiFi(true);
  }

  // init the MQTT connection
  client.setServer(MQTT_SERVER_IP, MQTT_SERVER_PORT);
  client.setCallback(callback);
}

void loop() {

  //Blink when running
  if (non_blocking_interval_act()) {
    actLED();

    //if (!timeUpdated && WiFi.status() == WL_CONNECTED && wifiStatus && client.connected()) {
    //  updateStartTime();
    //}

  }

  //Blink LED when not connected, but wifi is on
  if (WiFi.status() != WL_CONNECTED && wifiStatus && non_blocking_interval()) {
    wifiLED();
  } else if (WiFi.status() == WL_CONNECTED && wifiStatus && client.connected()) {
    //Turn LED On (We are connected)

    if (!pong) {
      digitalWrite(WIFI_LED, HIGH);
    } else {
      pongWifiLED();
    }

    //udp.begin(localPort);
    client.loop();

  } else if (WiFi.status() == WL_CONNECTED && wifiStatus && non_blocking_interval() && !client.connected()) {
    wifiLED();
    reconnect(); //Reconnect MQTT if it fails
  } else if (!wifiStatus) {
    //Turn LED Off
    digitalWrite(WIFI_LED, LOW);
  }

   if (isButtonPressed(WIFI_B_PIN)) {
      LogOverMQTT("WiFi button pressed!");
      switchWiFi(!wifiStatus);
  }

   if (isButtonPressed(BUTTON_PIN)) {
      LogOverMQTT("Button pressed!");
      relayStatus = !relayStatus;
      setLightState();
      publishLightState();
  }

//  //WiFi button handler
//  if (digitalRead(WIFI_B_PIN) == LOW) {
//
//    if (isButtonPressed(WIFI_B_PIN)) {
//      switchWiFi(!wifiStatus); //Switch WiFi interface
//    }
//
//    //Debounce Button
//    //delay(SHORT_PRESS_BOUNCE);
//    //if (!(digitalRead(WIFI_B_PIN) == LOW)) {
//    //  return;
//    //}
//  }

  //
  //  //Relay button handler
  //  if (digitalRead(BUTTON_PIN) == LOW) {
  //    //Debounce Button
  //    //delay(SHORT_PRESS_BOUNCE);
  //    //if (!(digitalRead(BUTTON_PIN) == LOW)) {
  //     // LogOverMQTT("Button debouncing failed :(");
  //      //return;
  //    //}
  //
  //    if (buttonDebounce(BUTTON_PIN)) {
  //
  //      LogOverMQTT("Button pressed!");
  //
  //    relayStatus = !relayStatus;
  //    setLightState();
  //    publishLightState();
  //    }
  //  }

  //Light Switch handler
  if (digitalRead(SWITCH_PIN) != prevSwitch) {
    //Debounce Button
    delay(SHORT_PRESS_BOUNCE);
    if (!(digitalRead(SWITCH_PIN) != prevSwitch)) {
      LogOverMQTT("Switch debouncing failed: ");
      return;
    }

    LogOverMQTT("Switch changed!");

    prevSwitch = !prevSwitch;
    relayStatus = !relayStatus;
    setLightState();
    publishLightState();
    delay(50);
  }
}

void wifiLED() {
  if (ledState == LOW)
    ledState = HIGH;
  else
    ledState = LOW;
  digitalWrite(WIFI_LED, ledState);
}

void actLED() {
  if (actLedState == LOW)
    actLedState = HIGH;
  else
    actLedState = LOW;
  digitalWrite(BUILTIN_LED, actLedState);
}

bool non_blocking_interval() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    return true;
  }
  return false;
}

bool non_blocking_interval_act() {
  unsigned long currentMillis = millis();
  if (currentMillis - actpreviousMillis >= actinterval) {
    actpreviousMillis = currentMillis;
    return true;
  }
  return false;
}

void switchWiFi(bool b) {
  if (b) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    wifiStatus = true;
    write_to_EEPROM(EEPROM_ADDR_1, 1);
  } else {
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    wifiStatus = false;
    write_to_EEPROM(EEPROM_ADDR_1, 0);
  }
}

void write_to_EEPROM(int address, byte value) {
  if (EEPROM.read(address) != value) {
    EEPROM.write(address, value);
    EEPROM.commit();
  }
}

// send an NTP request to the time server at the given address
//unsigned long sendNTPpacket(IPAddress& address)
//{
//  //Serial.println("sending NTP packet...");
//  // set all bytes in the buffer to 0
//  memset(packetBuffer, 0, NTP_PACKET_SIZE);
//  // Initialize values needed to form NTP request
//  // (see URL above for details on the packets)
//  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
//  packetBuffer[1] = 0;     // Stratum, or type of clock
//  packetBuffer[2] = 6;     // Polling Interval
//  packetBuffer[3] = 0xEC;  // Peer Clock Precision
//  // 8 bytes of zero for Root Delay & Root Dispersion
//  packetBuffer[12]  = 49;
//  packetBuffer[13]  = 0x4E;
//  packetBuffer[14]  = 49;
//  packetBuffer[15]  = 52;
//
//  // all NTP fields have been given values, now
//  // you can send a packet requesting a timestamp:
//  udp.beginPacket(address, 123); //NTP requests are to port 123
//  udp.write(packetBuffer, NTP_PACKET_SIZE);
//  udp.endPacket();
//}

//void updateStartTime() {
//
// //get a random server from the pool
//  WiFi.hostByName(ntpServerName, timeServerIP);
//
//  sendNTPpacket(timeServerIP); // send an NTP packet to a time server
//
//  int cb = udp.parsePacket();
//  if (cb) {
//    timeUpdated = true;
//
//    udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer
//
//    //the timestamp starts at byte 40 of the received packet and is four bytes,
//    // or two words, long. First, esxtract the two words:
//
//    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
//    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
//    // combine the four bytes (two words) into a long integer
//    // this is NTP time (seconds since Jan 1 1900):
//    unsigned long secsSince1900 = highWord << 16 | lowWord;
//
//    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
//    const unsigned long seventyYears = 2208988800UL;
//    // subtract seventy years:
//    unsigned long epoch = secsSince1900 - seventyYears;
//
//    String readableTime = String(((epoch  % 86400L) / 3600))+":";
//    if ( ((epoch % 3600) / 60) < 10 ) {
//      readableTime+="0";
//    }
//    readableTime+=String(((epoch  % 3600) / 60))+":";
//    if ( (epoch % 60) < 10 ) {
//      readableTime+="0";
//    }
//    readableTime+=String((epoch % 60));
//    readableTime+=" GMT+0";
//
//    if (client.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD)) {
//      //readableTime.toCharArray(currentTimeData, 50);
//      //client.publish(MQTT_LIGHT_LASTBOOT_TOPIC, currentTimeData, true);
//    }
//  }
//}

void LogOverMQTT(char* msg) {

  if (client.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD)) {
    client.publish(MQTT_LOG_TOPIC, msg, true);
  }

}

void pongWifiLED() {
  digitalWrite(WIFI_LED, LOW);

  //delay 500 ms
  for (int i = 0; i < 10; i++) {
    delay(50);
    client.loop();
  }

  for (int i = 0; i < 3; i++) {
    digitalWrite(WIFI_LED, HIGH);
    delay(50);
    digitalWrite(WIFI_LED, LOW);
    delay(50);
  }

  //delay 500 ms
  for (int i = 0; i < 10; i++) {
    delay(50);
    client.loop();
  }

  digitalWrite(WIFI_LED, HIGH);
  pong = false;
}

//Better debounce routine, non-blocking

#define NHB 2
unsigned long lastDebounceTime[NHB] = {0, 0};
int lastButtonState[NHB] = {HIGH, HIGH};
bool buttonStep[NHB] = {false, false};
uint8_t buttonsArray[NHB] = {BUTTON_PIN, WIFI_B_PIN};

//BUTTON -> Array pos
int buttonToPos(uint8_t BUTTON) {
  for (int i=0; i<NHB; i++) {
    if (buttonsArray[i] == BUTTON) {
          return i;
    }
  }

  return -1; //Not found
}

bool isButtonPressed(uint8_t BUTTON) {

  int value = digitalRead(BUTTON);
  int pos = buttonToPos(BUTTON);

  //Start counting when button is first pressed
  if (value != lastButtonState[pos]) {
    lastDebounceTime[pos] = millis();
  }

  //Set current state
  lastButtonState[pos] = value;

  //If true, at least SHORT_PRESS_BOUNCE ms have passed since the button was first pressed
  if ((millis() - lastDebounceTime[pos]) > SHORT_PRESS_BOUNCE) {

    if (value == lastButtonState[pos] && value == LOW) {
      buttonStep[pos] = true;
    } else if (value == lastButtonState[pos] && value == HIGH) {
      if (buttonStep[pos]==true) {
        buttonStep[pos] = false;
        return true;
      }
    }

  }

  return false;
}

/*

  // init the WiFi connection
  //Serial.println();
  //Serial.println();
  //Serial.print("INFO: Connecting to ");
  WiFi.mode(WIFI_STA);
  //Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    //Serial.print(".");
  }

  //Serial.println("");
  //Serial.println("INFO: WiFi connected");
  //Serial.print("INFO: IP address: ");
  //Serial.println(WiFi.localIP());
*/
