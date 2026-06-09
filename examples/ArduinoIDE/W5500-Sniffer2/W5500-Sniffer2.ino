/*
  W5500-Sniffer2

  Sniffing the LAN with W5500 Ethernet controller in promiscuous mode on the YB-ESP32-S3-ETH board.

  We use Espressif's Ethernet driver for W5500, which is included in the ESP-IDF framework.
  The driver is configured to receive all Ethernet frames (promiscuous mode) and print their basic
  information (length, source/destination MAC addresses, EtherType) to the serial console.

  Make sure solder bridges "INT" and "RST" on the bottom of the board are closed.

  Last updated 2026-06-09, ThJ <yellobyte@bluewin.ch>
*/

#include <Arduino.h>
#include <WiFi.h>
#include <ETH.h>

esp_eth_handle_t hEth = NULL;

esp_err_t printPacket(esp_eth_handle_t handle, uint8_t *buffer, uint32_t len, void *priv)
{
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));  // toggle LED to indicate packet reception
  Serial.printf("Eth packet: len=% 4d Dst=%02x:%02x:%02x:%02x:%02x:%02x Src=%02x:%02x:%02x:%02x:%02x:%02x" \
                " Type=0x%02x%02x (%s%s)\n",
                len, buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5],
                buffer[6], buffer[7], buffer[8], buffer[9], buffer[10], buffer[11],
                buffer[12], buffer[13],
                (buffer[12] == 0x08 && buffer[13] == 0x00) ? "IPv4" :
                ((buffer[12] == 0x08 && buffer[13] == 0x06) ? "ARP" :
                ((buffer[12] == 0x81 && buffer[13] == 0x00) ? "802.1Q VLAN tagging" :
                ((buffer[12] == 0x86 && buffer[13] == 0xDD) ? "IPv6" :
                ((buffer[12] == 0x88 && buffer[13] == 0x74) ? "BRDCM switch mngmt" :
                ((buffer[12] == 0x88 && buffer[13] == 0x99) ? "Realtek RCP" :
                ((buffer[12] == 0x89 && buffer[13] == 0x3A) ? "IEEE 1905.1" :
                ((buffer[12] == 0x88 && buffer[13] == 0x7B) ? "Homeplug 1.0" :
                ((buffer[12] == 0x88 && buffer[13] == 0xE1) ? "Homeplug Green PHY" :
                ((buffer[12] == 0x88 && buffer[13] == 0xCC) ? "LLDP" :
                "?"))))))))),
                (buffer[0] == 0x00 && buffer[1] == 0x00 && buffer[2] == 0x5E) ? ", IANA Unicast" :
                ((buffer[0] == 0x01 && buffer[1] == 0x00 && buffer[2] == 0x5E) ? ", IANA Multicast" :
                ((buffer[0] == 0x33 && buffer[1] == 0x33) ? ", multicast" :
                ((buffer[0] == 0xFF && buffer[1] == 0xFF && buffer[2] == 0xFF) ? ", broadcast" :
                (", ?")))));
  free(buffer);

  return ESP_OK;
}

// Event handler for Ethernet
void ethEventHandler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
  uint8_t mac[6];
	
  switch (event_id) {
  case ETHERNET_EVENT_CONNECTED:
    log_i("Ethernet Link Up");
    esp_eth_ioctl(hEth, ETH_CMD_G_MAC_ADDR, mac);
    log_i("Ethernet MAC Address: %02X:%02X:%02X:%02X:%02X:%02X",
          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);		
    break;
  case ETHERNET_EVENT_DISCONNECTED:
    log_i("Ethernet Link Down");
    break;
  case ETHERNET_EVENT_START:
    log_i("Ethernet Started");
    break;
  case ETHERNET_EVENT_STOP:
    log_i("Ethernet Stopped");
    break;
  default:
    log_i("Event %d", event_id);
    break;
  }
}

void initializeEthernet(void)
{
  ESP_ERROR_CHECK(gpio_install_isr_service(0));
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  esp_eth_handle_t hEth = NULL;
  spi_bus_config_t busCfg = {
    .mosi_io_num = MOSI,
    .miso_io_num = MISO,
    .sclk_io_num = SCK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
  };
  spi_device_handle_t hSpi = NULL;
  spi_device_interface_config_t spiDevCfg = {
    // .command_bits = 16, // Actually it's the address phase in W5500 SPI frame
    // .address_bits = 8,  // Actually it's the control phase in W5500 SPI frame
    .mode = 0,
    .clock_speed_hz = 40 * 1000 * 1000,
    .spics_io_num = W5500_SS,
    .queue_size = 20
  };
  ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &busCfg, SPI_DMA_CH_AUTO));
  ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &spiDevCfg, &hSpi));

  eth_w5500_config_t w5500_config = ETH_W5500_DEFAULT_CONFIG(SPI3_HOST, &spiDevCfg);
  w5500_config.int_gpio_num = W5500_INT;
  esp_eth_mac_t *eth_mac = NULL;
  esp_eth_phy_t *eth_phy = NULL;
  eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
  //mac_config.rx_task_stack_size = 8192;                 // doubles the stack size
  eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
  phy_config.reset_gpio_num = W5500_RST;

  eth_mac = esp_eth_mac_new_w5500(&w5500_config, &mac_config);
  ESP_ERROR_CHECK((eth_mac == NULL) ? ESP_FAIL : ESP_OK);
  eth_phy = esp_eth_phy_new_w5500(&phy_config);
  ESP_ERROR_CHECK((eth_phy == NULL) ? ESP_FAIL : ESP_OK);

  esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(eth_mac, eth_phy);
  ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config, &hEth));

  uint8_t mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
  ESP_ERROR_CHECK(esp_eth_ioctl(eth_handle, ETH_CMD_S_MAC_ADDR, mac));

  ESP_ERROR_CHECK(esp_eth_update_input_path(hEth, printPacket, NULL));

  bool bEthPromiscuous = true;
  ESP_ERROR_CHECK(esp_eth_ioctl(hEth, ETH_CMD_S_PROMISCUOUS, &bEthPromiscuous));
  ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, ethEventHandler, NULL));
  ESP_ERROR_CHECK(esp_eth_start(hEth));
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);  // status LED off

  WiFi.mode(WIFI_OFF);             // WiFi not needed
  btStop();                        // Bluetooth not needed

  Serial.begin(115200);
  Serial.println("W5500-Sniffer running...");

  initializeEthernet();
}

void loop() {
  ;
}
