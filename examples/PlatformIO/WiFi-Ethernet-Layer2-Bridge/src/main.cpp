/*
  Wifi-Ethernet-Layer2-Bridge (Wifi access point with Ethernet connection)

  The YB-ESP32-S3-ETH board has both Wifi and Ethernet connectivity. This example demonstrates
  how to bridge the two interfaces at Layer 2, allowing devices connected to the Wifi Soft-AP to
  communicate with devices on the Ethernet network and the internet if the board is connected
  to an internet router via Ethernet.

  IMPORTANT:
  Make sure solder bridges "INT" and "RST" on the bottom of the board are closed.

  Last updated 2026-06-18, ThJ <yellobyte@bluewin.ch>
*/

#include <Arduino.h>
#include <WiFi.h>
#include <ETH.h>
#include <freertos/queue.h>
#include <esp_eth_driver.h>
#include <esp_wifi.h>
#include <esp_private/wifi.h>

#define CONFIG_WIFI_AP_SSID     "YB-ESP32-S3-ETH_Soft-AP"
#define CONFIG_WIFI_AP_PASSWORD "12345678"

#ifdef CONFIG_WIFI_AP_CHANNEL
#undef CONFIG_WIFI_AP_CHANNEL
#endif
#define CONFIG_WIFI_AP_CHANNEL        3   // choose a less crowded channel in your environment
                                          // to improve performance
#define CONFIG_WIFI_AP_MAX_STA_CONN   2   // two simultaneous WiFi connections are supported
#define CONFIG_WIFI_AP_IS_SSID_HIDDEN 0   // set to 1 to hide the Soft-AP SSID, but then you need to
                                          // manually connect to the AP by specifying the SSID on the client side

#define FLOW_CONTROL_QUEUE_TIMEOUT_MS     (100)
#define FLOW_CONTROL_QUEUE_LENGTH         (250)
#define FLOW_CONTROL_WIFI_SEND_TIMEOUT_MS (100)

esp_eth_handle_t hEth = NULL;
QueueHandle_t hFlowControlQueue = NULL;
bool bStaConnected = false;
bool bEthernetConnected = false;
uint8_t conCnt = 0;
uint32_t pktCntEthOk = 0,
         pktCntEthErr = 0,
         pktCntWifiOk = 0,
         pktCntWifiErr = 0;
uint8_t mac[6] = { 0 };

typedef struct {
  void *packet;
  uint16_t length;
} flow_control_msg_t;

// Event handler for Ethernet
void ethEventHandler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
  switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
      log_i("Ethernet Link Up");
      bEthernetConnected = true;
      esp_eth_ioctl(hEth, ETH_CMD_G_MAC_ADDR, mac);
      log_i("Ethernet MAC Address: %02X:%02X:%02X:%02X:%02X:%02X",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
      esp_wifi_set_mac(WIFI_IF_AP, mac);
      ESP_ERROR_CHECK(esp_wifi_start());
      break;
    case ETHERNET_EVENT_DISCONNECTED:
      log_i("Ethernet Link Down");
      bEthernetConnected = false;
      ESP_ERROR_CHECK(esp_wifi_stop());
      break;
    case ETHERNET_EVENT_START:
      log_i("Ethernet Started");
      break;
    case ETHERNET_EVENT_STOP:
      log_i("Ethernet Stopped");
      break;
    default:
      log_i("Ethernet Event %d", event_id);
      break;
  }
}

// Forwards packets directly from Wifi to Ethernet.
esp_err_t pktWifi2eth(void *buffer, uint16_t len, void *eb)
{
  if (bEthernetConnected) {
    if (esp_eth_transmit(hEth, buffer, len) != ESP_OK) {
      log_e("sending packet to ethernet failed");
      pktCntEthErr++;
    }
    else {
      // toggle status LED to indicate packet successfully transmitted to Ethernet
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      pktCntEthOk++;
    }
  }
  esp_wifi_internal_free_rx_buffer(eb);
  return ESP_OK;
}

// Event handler for Wi-Fi
void wifiEventHandler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
  switch (event_id) {
    case WIFI_EVENT_AP_STACONNECTED:
      log_i("WiFi Soft-AP got a station connected");
      if (!conCnt) {
        bStaConnected = true;
        esp_wifi_internal_reg_rxcb(WIFI_IF_AP, pktWifi2eth);
      }
      conCnt++;
      break;
    case WIFI_EVENT_AP_STADISCONNECTED:
      log_i("WiFi Soft-AP got a station disconnected");
      conCnt--;
      if (!conCnt) {
        bStaConnected = false;
        esp_wifi_internal_reg_rxcb(WIFI_IF_AP, NULL);
      }
      break;
    case WIFI_EVENT_AP_START:
      log_i("WiFi Soft-AP started");
      break;
    case WIFI_EVENT_AP_STOP:
      log_i("WiFi Soft-AP stopped");
      break;
    case WIFI_EVENT_HOME_CHANNEL_CHANGE:
      log_i("WiFi home channel changed");
      break;
    default:
      log_i("WiFi Event %d", event_id);
      break;
  }
}

// Takes the packets from Ethernet and queues them.
// This is necessary because Wifi handles packets slower than Ethernet.
esp_err_t queueEth2wifi(esp_eth_handle_t eth_handle, uint8_t *buffer, uint32_t len, void *priv)
{
  flow_control_msg_t msg = { .packet = buffer, .length = (uint16_t)len };

  if (xQueueSend(hFlowControlQueue, &msg, pdMS_TO_TICKS(FLOW_CONTROL_QUEUE_TIMEOUT_MS)) != pdTRUE) {
    log_e("queueing eth->wifi failed or timeout");
    free(buffer);
    pktCntWifiErr++;
    return ESP_FAIL;
  }

  return ESP_OK;
}

// This task fetches packets from the queue and then sends them out through Wifi.
void pktEth2wifiTask(void *args)
{
  flow_control_msg_t msg;
  int res = 0;
  uint32_t timeout = 0;

  while (1) {
    if (xQueueReceive(hFlowControlQueue, &msg, pdMS_TO_TICKS(FLOW_CONTROL_QUEUE_TIMEOUT_MS)) == pdTRUE) {
      timeout = 0;
      if (bStaConnected && msg.length) {
        do {
          vTaskDelay(pdMS_TO_TICKS(timeout));
          timeout += 2;
          res = esp_wifi_internal_tx(WIFI_IF_AP, msg.packet, msg.length);
        }
        while (res && timeout < FLOW_CONTROL_WIFI_SEND_TIMEOUT_MS);
        if (res != ESP_OK) {
          log_e("sending packet to wifi failed, error=%d(0x%x)", res, res);
          pktCntWifiErr++;
        }
        else {
          // toggle status LED to indicate successfull packet transmission to Wifi
          digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
          pktCntWifiOk++;
        }
      }
      free(msg.packet);
    }
  }
  vTaskDelete(NULL);  // will never get here
}

esp_err_t initializeFlowControl(void)
{
  // Create the queue for flow control between Ethernet and Wifi and the task
  // to forward packets from queue to Wifi.
  hFlowControlQueue = xQueueCreate(FLOW_CONTROL_QUEUE_LENGTH, sizeof(flow_control_msg_t));
  if (!hFlowControlQueue) {
    log_e("create eth->wifi queue failed");
    return ESP_FAIL;
  }
  BaseType_t ret = xTaskCreate(pktEth2wifiTask, "pktEth2wifiTask", 2048, NULL, (tskIDLE_PRIORITY + 2), NULL);
  if (ret != pdTRUE) {
    log_e("create eth->wifi task failed");
    return ESP_FAIL;
  }
  return ESP_OK;
}

void initializeEthernet(void)
{
  spi_bus_config_t busCfg = {
    .mosi_io_num = MOSI,
    .miso_io_num = MISO,
    .sclk_io_num = SCK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
  };
  spi_device_handle_t hSpi = NULL;
  spi_device_interface_config_t spiDevCfg = {
    // .command_bits = 16,
    // .address_bits = 8,
    .mode = 0,
    .clock_speed_hz = 40 * 1000 * 1000,  // 40MHz
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
  eth_config.stack_input = queueEth2wifi;
  ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config, &hEth));

  uint8_t mac[6] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
  ESP_ERROR_CHECK(esp_eth_ioctl(hEth, ETH_CMD_S_MAC_ADDR, mac));

  bool bEthPromiscuous = true;
  ESP_ERROR_CHECK(esp_eth_ioctl(hEth, ETH_CMD_S_PROMISCUOUS, &bEthPromiscuous));
  ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, ethEventHandler, NULL));
  ESP_ERROR_CHECK(esp_eth_start(hEth));
}

void initializeWifi(void)
{
  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifiEventHandler, NULL));
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  wifi_config_t wifiConfig = {
    .ap = {
      .ssid = CONFIG_WIFI_AP_SSID,
      .password = CONFIG_WIFI_AP_PASSWORD,
      .ssid_len = strlen(CONFIG_WIFI_AP_SSID),
      .channel = CONFIG_WIFI_AP_CHANNEL,
      .authmode = WIFI_AUTH_WPA_WPA2_PSK,
      .ssid_hidden = CONFIG_WIFI_AP_IS_SSID_HIDDEN,
      .max_connection = CONFIG_WIFI_AP_MAX_STA_CONN
    },
  };
  if (strlen(CONFIG_WIFI_AP_PASSWORD) == 0) {
      wifiConfig.ap.authmode = WIFI_AUTH_OPEN;
  }
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifiConfig));
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);  // status LED off

  btStop();                        // Bluetooth not needed

  Serial.begin(115200);
  Serial.println("ESP32-S3 Wifi-Ethernet L2 Bridge running...");

  ESP_ERROR_CHECK(gpio_install_isr_service(0));
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  ESP_ERROR_CHECK(initializeFlowControl());
  initializeWifi();
  initializeEthernet();
}

void loop() {
  delay(60000);
  Serial.printf("Stats: RAM Used/Free=%lu/%lu Bytes, Eth->Wifi=%lu/%lu, Wifi->Eth=%lu/%lu, AP-Connections=%d\n",
                ESP.getHeapSize() - ESP.getFreeHeap(), ESP.getFreeHeap(),
                pktCntWifiOk, pktCntWifiErr, pktCntEthOk, pktCntEthErr, conCnt);
}
