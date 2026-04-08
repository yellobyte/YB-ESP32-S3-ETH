/*
  Link-Status

  The sketch checks the current link status of the W5500 ethernet port integrated on YB-ESP32-S3-ETH board. 
  The status LED should be "ON" when an ethernet cable is properly connected between board and a switch.
  Getting an IP address via DHCP is not tested.

  Last updated 2026-04-07, ThJ <yellobyte@bluewin.ch>
*/

#include <Arduino.h>
#include <Ethernet.h>

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);           // status LED off
	
#ifdef W5500_HARD_RESET
  pinMode(W5500_RST, OUTPUT);
  digitalWrite(W5500_RST, LOW);             // needs RST solder bridge closed
  delay(10);
  pinMode(W5500_RST, INPUT);
  delay(250);
#endif 	
  
  Serial.begin(115200);

  Ethernet.init(W5500_SS);
}

void loop() {
  EthernetLinkStatus status = Ethernet.linkStatus();
  Serial.print("Link status: ");
  switch (status) {
    case Unknown:
      Serial.println("Unknown");
      digitalWrite(LED_BUILTIN, LOW);       // status LED off  
      break;
    case LinkON:
      Serial.println("ON");
      digitalWrite(LED_BUILTIN, HIGH);      // status LED on  
      break;
    case LinkOFF:
      Serial.println("OFF");
      digitalWrite(LED_BUILTIN, LOW);       // status LED off  
      break;
  }
  delay(1000);
}
