/*
  Wifi-Ethernet-NAT (network address [port] translation)

  The YB-ESP32-S3-ETH board has both Wifi and Ethernet connectivity. This example
  demonstrates how to bridge the two interfaces via NA(P)T. It creates a Wifi access
  point with Ethernet connection, allowing connected WiFi clients to access the Ethernet
  network and the Internet if the board is connected to an internet router via Ethernet.

  Only a single Wifi connection is supported.
  Make sure solder bridges "INT" and "RST" on the bottom of the board are closed.

  Last updated 2026-06-10, ThJ <yellobyte@bluewin.ch>
*/

#include <Arduino.h>
#include <WiFi.h>
#include <ETH.h>
#include <SPI.h>

#define ETH_TYPE     ETH_PHY_W5500
#define ETH_ADDR     1

#define AP_SSID "YB-ESP32-S3-ETH_Soft-AP"
#define AP_PASS "12345678"

IPAddress apIp(192, 168, 2, 1);
IPAddress apMask(255, 255, 255, 0);
IPAddress apLeaseStart(192, 168, 2, 2);
IPAddress apDns(9, 9, 9, 9);

void eventHandler(arduino_event_id_t event, arduino_event_info_t info) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("ETH Started");
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("ETH Connected");
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.println("ETH Got IP ");
      Serial.println(ETH);
      WiFi.AP.enableNAPT(true);    // enables NAT between Ethernet and WiFi interfaces
      break;
    case ARDUINO_EVENT_ETH_LOST_IP:
      Serial.println("ETH Lost IP");
      WiFi.AP.enableNAPT(false);   // NAT is disabled when Ethernet connection is lost, preventing
                                   // WiFi clients from accessing the Ethernet network
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH Disconnected");
      WiFi.AP.enableNAPT(false);   // NAT is disabled when Ethernet connection is lost, preventing
                                   // WiFi clients from accessing the Ethernet network
      break;
    case ARDUINO_EVENT_ETH_STOP:
      Serial.println("ETH Stopped");
      break;
    case ARDUINO_EVENT_WIFI_AP_START:
      Serial.println("AP Started ");
      Serial.println(WiFi.AP);
      break;
    case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
      Serial.println("AP STA Connected");
      digitalWrite(LED_BUILTIN, HIGH);
      break;
    case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
      Serial.println("AP STA Disconnected");
      digitalWrite(LED_BUILTIN, LOW);
      break;
    case ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED:
      Serial.print("AP STA IP Assigned: ");
      Serial.println(IPAddress(info.wifi_ap_staipassigned.ip.addr));
      break;
    case ARDUINO_EVENT_WIFI_AP_PROBEREQRECVED:
      Serial.println("AP Probe Request Received");
      break;
    case ARDUINO_EVENT_WIFI_AP_STOP:
      Serial.println("AP Stopped");
      digitalWrite(LED_BUILTIN, LOW);
      break;
    case ARDUINO_EVENT_WIFI_READY:
      Serial.println("WiFi Ready");
      break;
    default:
      Serial.printf("Event ID: %d\n", event);
      break;
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);  // status LED off

  Serial.begin(115200);
  Serial.setDebugOutput(true);     // ESP-IDF log messages will be sent to this Serial port

  btStop();                        // Bluetooth not needed  
  Network.onEvent(eventHandler);   // install event handler for network events

  WiFi.AP.begin();
  WiFi.AP.config(apIp, apIp, apMask, apLeaseStart, apDns);
  WiFi.AP.create(AP_SSID, AP_PASS);
  if (!WiFi.AP.waitStatusBits(ESP_NETIF_STARTED_BIT, 1000)) {
    Serial.println("Failed to start AP!");
    return;
  }
  delay(100);

  SPI.begin(SCK, MISO, MOSI);
  SPI.setFrequency(40000000);      // 40MHz
  ETH.begin(ETH_TYPE, ETH_ADDR, W5500_SS, W5500_INT, W5500_RST, SPI);
}

void loop() {
  delay(20000);
}

