/*
  Test Ping

  The sketch pings another network device via Ethernet/LAN connection.     
  Status LED flashes when ping reply is received.

  Last updated 2023-06-16, ThJ <yellobyte@bluewin.ch>
*/

#include <Arduino.h>
#include <Ethernet2.h>
#include <ICMPPing.h>

#define GPIO_STATUS_LED  47                 // onboard status LED connected to GPIO47

// Ethernet settings
#define GPIO_W5500_CS  14                   // onboard W5500 CS pin is connected to GPIO14
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

// Ping settings
char buffer [128];
//IPAddress pingAddr(192, 168, 1, 1);         // local IP address to ping
IPAddress pingAddr(8, 8, 8, 8);             // external IP address to ping (Google DNS server)
SOCKET pingSocket = 0;
ICMPPing ping(pingSocket, (uint16_t)random(0, 255));

void setup() {
  pinMode(GPIO_STATUS_LED, OUTPUT);
  digitalWrite(GPIO_STATUS_LED, LOW);       // status LED off  
	
#ifdef W5500_HARD_RESET
  pinMode(W5500_RST, OUTPUT);
  digitalWrite(W5500_RST, LOW);             // needs RST solder bridge closed
  delay(10);
  pinMode(W5500_RST, INPUT);
  delay(250);
#endif 

  Serial.begin(115200);
  Serial.println();
  Serial.println("Please make sure Ethernet cable is connected between board and switch and DHCP service is available in your LAN.");

  // connect to local network via Ethernet
  Serial.print("Initializing Ethernet...");
  Ethernet.init(GPIO_W5500_CS);
  while (true) {
    if (Ethernet.begin(mac)) {
      Serial.println("DHCP on Ethernet ok.");
      break;
    }
    Serial.println("Ethernet DHCP error, couldn't get IP !");
    delay(2000);
  }
  Serial.print("Local Ethernet IP: ");
  Serial.println(Ethernet.localIP());
}

void loop() {
  ICMPEchoReply echoReply = ping(pingAddr, 4);

  if (echoReply.status == SUCCESS) {
    sprintf(buffer, "Reply[%d] from: %d.%d.%d.%d: bytes=%d time=%ldms TTL=%d",
            echoReply.data.seq, echoReply.addr[0], echoReply.addr[1], echoReply.addr[2], echoReply.addr[3],
            REQ_DATASIZE, millis() - echoReply.data.time, echoReply.ttl);
    digitalWrite(GPIO_STATUS_LED, HIGH);    // status LED on  
  }
  else {
    sprintf(buffer, "Echo request failed: %d", echoReply.status);
  }
  Serial.println(buffer);
  delay(1000);
  digitalWrite(GPIO_STATUS_LED, LOW);       // status LED off  
  delay(100);
}