/*
  Test-W5500-MACRAW

  Sniffing the Ethernet LAN with W5500 in promisucous mode.

  The sketch is based on Nicholas Humfrey's "W5500MacRaw" example.
    https://github.com/njh/W5500MacRaw

  Last updated 2026-04-09, ThJ <yellobyte@bluewin.ch>
*/

#include <Arduino.h>
#include "w5500.h"

uint8_t mac_addr[6] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
Wiznet5500 w5500(W5500_SS);              // setting the Chip Select pin

uint8_t buffer[2048];

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

#if 1 // set to 0 if not wanted
  pinMode(W5500_RST, OUTPUT);            // needs RST solder bridge closed
  digitalWrite(W5500_RST, LOW);
  delay(10);
  pinMode(W5500_RST, INPUT);
  delay(250);
#endif

  Serial.begin(115200);
  Serial.print("W5500 MACRAW mode. W5500 MAC address: ");
  Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
                mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

  w5500.begin(mac_addr);
}

void loop() {
  uint16_t len = w5500.readFrame(buffer, sizeof(buffer));
  if (len > 0) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    Serial.printf("Len=%04dd Dst=%02x:%02x:%02x:%02x:%02x:%02x Src=%02x:%02x:%02x:%02x:%02x:%02x" \
                  " Type=0x%02x%02x (%s%s)\n",
                  len, buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5],
                  buffer[6], buffer[7], buffer[8], buffer[9], buffer[10], buffer[11],
                  buffer[12], buffer[13],
                  (buffer[12] == 0x08 && buffer[13] == 0x00) ? "IPv4" :
                  ((buffer[12] == 0x08 && buffer[13] == 0x06) ? "ARP" :
                  ((buffer[12] == 0x86 && buffer[13] == 0xDD) ? "IPv6" :
                  ((buffer[12] == 0x88 && buffer[13] == 0x74) ? "BRDCM switch mngmt" :
                  ((buffer[12] == 0x88 && buffer[13] == 0x99) ? "Realtek RCP" :
                  ((buffer[12] == 0x89 && buffer[13] == 0x3A) ? "IEEE 1905.1" :
                  ((buffer[12] == 0x88 && buffer[13] == 0x7B) ? "Homeplug 1.0" :
                  ((buffer[12] == 0x88 && buffer[13] == 0xE1) ? "Homeplug Green PHY" :
                  ((buffer[12] == 0x88 && buffer[13] == 0xCC) ? "LLDP" :
                  "?")))))))),
                  (buffer[0] == 0x00 && buffer[1] == 0x00 && buffer[2] == 0x5E) ? ", IANA Unicast" :
                  ((buffer[0] == 0x01 && buffer[1] == 0x00 && buffer[2] == 0x5E) ? ", IANA Multicast" :
                  ((buffer[0] == 0x33 && buffer[1] == 0x33) ? ", multicast" :
                  ((buffer[0] == 0xFF && buffer[1] == 0xFF && buffer[2] == 0xFF) ? ", broadcast" :
                  (", ?")))));
  }
}

