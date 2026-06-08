/*
  Webserver_lwIP

  This example just gets an Ethernet IP address from DHCP server and starts a simple HTTP server.
  Instead of using a typical Arduino Ethernet lib (e.g. Ethernet.h) it uses the lwIP stack
  integrated in the ESP32 Arduino core.

  You have to make sure solder bridges "INT" and "RST" on the bottom of the board are closed.

  The LED will: blink very fast - with no Ethernet cable attached/no link to a switch
                blink normal    - with Ethernet cable attached to a switch and the link is up
                blink very slow - local IP has been obtained from DHPC service

  The device is reachable at http://esp32-eth0.local from any device in the same LAN.
  The HTTP server will respond with "Hello from YB-ESP32-S3-ETH over ethernet!" to requests
  on the root URI "/", otherwise with a "File Not Found" message.

  The software is based on Sebastian Hirnschall's example on
    https://blog.hirnschall.net/esp32-w5500-ethernet/

  Last updated 2026-06-07, ThJ <yellobyte@bluewin.ch>
*/

#include <Arduino.h>
#include <WiFi.h>
#include <ETH.h>
#include <WebServer.h>
#include <ESPmDNS.h>

SPIClass ethSPI(FSPI);                      // SPI instance for W5500
WebServer server(80);
bool bEthConnected = false;                 // initial Ethernet connection status
uint32_t tdelay = 100;                      // initial blink delay in ms

void blinkTask (void *parameter) {
  Serial.println("blinkTask has started.");
  while (true) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(*((uint32_t*)parameter));
  }
  // will never get here
  Serial.println("blinkTask has finished.");
  vTaskDelete(NULL);                        // finishes the task/thread
}

void onEvent(arduino_event_id_t event, arduino_event_info_t info) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("ETH Started");
      ETH.setHostname("esp32-eth0");
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("ETH Connected");
      tdelay = 500;                         // blink slow
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.print("ETH Got IP: ");
      Serial.println(ETH.localIP());
      bEthConnected = true;
      tdelay = 2000;                        // blink very slow
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH Disconnected");
      bEthConnected = false;
      tdelay = 100;                         // blink very fast
      break;
    case ARDUINO_EVENT_ETH_STOP:
      Serial.println("ETH Stopped");
      bEthConnected = false;
      tdelay = 100;                         // blink very fast
      break;
    default:
      Serial.print("Event, id=");
      Serial.println(event);
      break;
  }
}

void handleRoot() {
  server.send(200, "text/plain", "Hello from YB-ESP32-S3-ETH over Ethernet!");
}

void handleNotFound() {
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);           // status LED off

  // start the backgound task responsible for letting the status LED blink
  xTaskCreatePinnedToCore(blinkTask,        // task function
                          "blinkTask",      // name of task
                          2048,             // stack size in words
                          (void *)&tdelay,  // task input parameter is a pointer to variable holding the delay time
                          0,                // priority of task
                          NULL,             // task handle (not used)
                          1);               // core the task should run on

  WiFi.mode(WIFI_OFF);
  btStop();

  Serial.begin(115200);
  delay(500);

  ethSPI.begin(SCK, MISO, MOSI, W5500_SS);
  Network.onEvent(onEvent);
  ETH.begin(ETH_PHY_W5500, 0, W5500_SS, W5500_INT, W5500_RST, ethSPI);

  if (MDNS.begin("esp32-eth0")) {
    Serial.println("mDNS responder started");
  }

  server.on("/", handleRoot);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  if (bEthConnected) {
    server.handleClient();
  }
  delay(10);
}
