/*
  ESP32-WiFi-Access-Point

  This example creates an ESP32-S3 WiFi Access Point (AP) on WiFi channel 3 and starts
  a server on port 80 which lets you switch the onboard status LED (GPIO47) on/off.

  The AP lets you connect to the ESP32 directly without the need for a WiFi router.

  The software is based on the example from
     https://randomnerdtutorials.com/esp32-access-point-ap-web-server.

  Last updated 2026-06-01, ThJ <yellobyte@bluewin.ch>
*/

#include <Arduino.h>
#include <WiFi.h>

// WiFi definitions, feel free to change
#define WIFI_AP_CHANNEL 3
const char *ssid     = "ESP32-Access-Point";
const char *password = "123456789";

String led47State = "off",  // initial LED state is off
       header;

// set web server port number to 80
WiFiServer server(80);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200);

  // create Wi-Fi Access Point (AP)
  Serial.print("Create AP (Access Point)…");
  WiFi.softAP(ssid, password, WIFI_AP_CHANNEL);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("on IP address: ");
  Serial.println(IP);

  server.begin();
}

void loop(){
  WiFiClient client = server.accept();      // Listen for incoming clients

  if (client) {                             // If a new client connects,
    Serial.println("New Client.");          // print a message out on the serial port
    String currentLine = "";                // String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row,
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();

            // turns the GPIOs on and off
            if (header.indexOf("GET /47/on") >= 0) {
              Serial.println("GPIO47 on");
              led47State = "on";
              digitalWrite(LED_BUILTIN, HIGH);
            }
            else if (header.indexOf("GET /47/off") >= 0) {
              Serial.println("GPIO47 off");
              led47State = "off";
              digitalWrite(LED_BUILTIN, LOW);
            }

            // display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");

            // CSS to style the on/off buttons,
            // feel free to change the background-color and/or font-size attributes
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #555555;}</style></head>");

            // web page heading
            client.println("<body><h1>YB-ESP32-S3-ETH - WiFi Web Server on port 80</h1>");

            // display current state, and ON/OFF buttons for GPIO47
            client.println("<p>GPIO 47 - State " + led47State + "</p>");

            // if the led47State is off, it displays the ON button and vice versa
            if (led47State == "off") {
              client.println("<p><a href=\"/47/on\"><button class=\"button\">ON</button></a></p>");
            }
            else {
              client.println("<p><a href=\"/47/off\"><button class=\"button button2\">OFF</button></a></p>");
            }
            client.println("</body></html>");

            // the HTTP response ends with another blank line
            client.println();
            // break out of the while loop
            break;
          }
          else {   // if we got a newline, clear currentLine
            currentLine = "";
          }
        }
        else if (c != '\r') {  // if we got anything else but a carriage return character,
          currentLine += c;    // add it to the end of the currentLine
        }
      }
    }
    // clear header variable
    header = "";
    // close connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
}
