## YB-ESP32-S3-ETH Development Board Overview:
The **YB-ESP32-S3-ETH** is a general purpose development board based on Espressif's ESP32-S3 MCU. Version 2.0 has been released recently and succeeds the earlier board versions [1.x](https://github.com/yellobyte/YB-ESP32-S3-ETH/tree/main/doc/retired_board_versions_V1.x). It presently comes in two different variants:  
- with **ESP32-S3-WROOM-1U-N8R2 module** (8MB Flash/2MB PSRAM),
- without **ESP32-S3-WROOM-1(U) module** which allows you to solder on any ESP32-S3-WROOM module of your choice.  

The board provides a **RJ45 Ethernet connector**, an Ethernet PHY bridge chip **Wiznet W5500**, CH334 USB-Hub chip, CH343 USB-UART bridge chip, two status LEDs and a **USB-C connector** for software upload, serial output, JTAG debugging and feeding power to the board. The boards are currently available on sales platforms [eBay](https://www.ebay.de/sch/i.html?_nkw=yb-esp32-s3) and [Ricardo.ch](https://www.ricardo.ch/en/s/YB-ESP32-S3). 

![](https://github.com/yellobyte/YB-ESP32-S3-ETH/raw/main/doc/YB-ESP32-S3-ETH_board_top.jpg)

Arduino libraries for the Wiznet W5500 are widely available (e.g. [**Ethernet**](https://www.arduino.cc/reference/en/libraries/ethernet/) or [**EthernetESP32**](https://docs.arduino.cc/libraries/ethernetesp32/)), enabling you to easily realise ESP32 projects around this board with **Ethernet or combined Ethernet/WLAN** support.

The densly populated YB-ESP32-S3-ETH board provides multiple GPIO pins (as shown below) and is still [**highly breadboard compatible**](https://github.com/yellobyte/YB-ESP32-S3-ETH/raw/main/doc/YB-ESP32-S3-ETH_on_breadboard.jpg) for it leaves one row of accessible breadboard contacts on either side of the board. All I/O ports (GPIOx) are labeled on both sides of the board. 

If WiFi/BT is needed instead of or additionally to an Ethernet connection then the external 2.4GHz WLAN antenna can be connected to the onboard WROOM-1U module. The connector is compatible with the following standards: U.FL (Hirose), MHF-I (I-PEX) and AMC (Amphen). 

A collection of software examples (for PlatformIO and/or ArduinoIDE) are available in folder [examples](https://github.com/yellobyte/YB-ESP32-S3-ETH/tree/main/examples). They will help you getting used to the board and exploring all hardware features.

## YB-ESP32-S3-ETH board features:
 - **ESP32-S3-WROOM-1U-N8R2** module (8MB Flash, 2MB PSRAM) or no module
 - **WiFi/BT IPEX** antenna connector (on ESP32-S3-WROOM-1U module)
 - **RJ45 10M/100M Ethernet** connector driven by onboard PHY controller chip Wiznet W5500
 - the **W5500** pins required to control the chip are hardwired to the ESP32-S3 GPIOs as follows:
   - *MOSI - GPIO11, MISO - GPIO13, SCK - GPIO12* (SPI bus data communication)
   - *SCS - GPIO14* (chip select, required for SPI bus control)
   - *RST - GPIO21* (if solder bridge is closed, for chip reset without resetting the whole board)  
   - *INT - GPIO18* (if solder bridge is closed, only needed by certain libraries)  
 - **control LEDs**. One LED labeled 'P' is connected to the 3.3V rail to indicate board power and the other LED labeled 'IO47' is connected to GPIO47 which can be used as status LED, for debugging purposes etc.
 - **USB-C** port connected to the onboard CH334 USB-Hub chip. This allows for simultaneous JTAG debugging and serial output as well as software upload (e.g. via ArduinoIDE, VSCode/PlatformIO etc). The ESP32-S3 contains an inbuild JTAG adapter hence [**debugging**](https://github.com/yellobyte/ESP32-DevBoards-Getting-Started/tree/main/debugging) becomes fairly easy.
 - **hardware logic** for *automatic* software uploads (supported by most Development IDEs) via USB-C port using the onboard CH343 USB-UART bridge chip.  
 - **pushbuttons**. One is labeled 'R' and resets the ESP32-S3 (shorts EN pin to ground) and the other one is labeled 'B' and shorts GPIO0 to ground when pressed. The latter is sometimes needed to force the board into boot mode.
 - **lots of available GPIOs** next to the ones already mentioned above.  

The board will spare you the trouble of cabling two different Arduino boards and saves a lot of room on the breadboard as well:  

 ![](https://github.com/yellobyte/YB-ESP32-S3-ETH/raw/main/doc/YB-ESP32-S3-ETH_modules_replacement.jpg)

Just for information purposes: The ESP32-S3-WROOM-1(U) module family comprises several [**versions**](https://github.com/yellobyte/ESP32-DevBoards-Getting-Started/raw/main/ESP32_specs_and_manuals/ESP32-S3-WROOM-1(U)_Variants.jpg). The **-1** versions come with embedded PCB antenna, the **-1U** versions with IPEX antenna socket instead. The extension -Nx(Ry) defines the amount of available FLASH/PSRAM, e.g. -N4 (4MB Flash, no PSRAM), -N8 (8MB Flash, no PSRAM), -N8R2 (8 MB Flash, 2MB PSRAM), -N8R8 (8 MB Flash, 8MB PSRAM) etc.  

## Board Pin Layout:
 ![](https://github.com/yellobyte/YB-ESP32-S3-ETH/raw/main/doc/YB-ESP32-S3-ETH_pinlayout.jpg)

The boards **outline**, **block diagram** and **schematic** files are all located in folder [doc](https://github.com/yellobyte/YB-ESP32-S3-ETH/tree/main/doc), together with some data sheets for Espressif's MCU ESP32-S3 and the ESP32-S3-WROOM-1(U) modules.

## Powering the board:
The board uses a LDO to drop the external supply voltage (5VDC min.) and internally operates on 3.3Volt. There are three different ways to provide power to the board.
  - through the USB-C port
  - 5...9VDC applied to the VIN pin
  - 3.3VDC applied to the 3V3 pin(s)  

Normal operating current of the idle board (all GPIOs unconnected, Ethernet Link down, WiFi disabled) is about 100mA (-N4) resp. 110mA (-N8R2). With Ethernet cable attached and Link up the current rises to about 165mA resp 180mA. With both Ethernet and WiFi active the board draws about 200...260mA (mainly depending on WiFi link).

## Application hints:
The board uses the popular WCH chips CH334P (USB-Hub) and CH343P (USB-UART bridge). If you haven't done yet then you need to install the CH343 Driver on your Laptop/PC. For Windows go [here](https://www.wch-ic.com/search?t=all&q=ch343) and download and install the newest version of the driver. Linux provides CH34x drivers by default. 

### Arduino IDE:
As of Arduino ESP32 Core V3.1.1 you open the board list, enter "yb" and then select "**Yellobyte YB-ESP32-S3-ETH**". Now choose the proper settings for COM port, debug level, flash size, PSRAM, etc. as shown below. Be aware, since the ESP32-S3 MCU is very versatile there are a lot of build options to play with. Espressif's homepage offers some help.

Settings that apply to the standard **-N8R2** board (8MB Flash/2MB PSRAM):  
- Board: *Yellobyte YB-ESP32-S3-ETH*
- Flash Size: *8MB*
- Partition Scheme: *8MB with spiffs (...)*
- PSRAM: *QSPI PSRAM*  

 ![](https://github.com/yellobyte/YB-ESP32-S3-ETH/raw/main/doc/YB-ESP32-S3-ETH-N8R2_ArduinoIDE-Settings.jpg)  

For **-N8R8** modules (8MB Flash/8MB PSRAM) the following settings apply: 
- Board: *Yellobyte YB-ESP32-S3-ETH*
- Flash Size: *8MB*
- Partition Scheme: *8MB with spiffs (...)*
- PSRAM: *OPI PSRAM*.

For **-N4** modules (4MB Flash/no PSRAM) the following settings apply: 
- Board: *Yellobyte YB-ESP32-S3-ETH*
- Flash Size: *4MB*
- Partition Scheme: *Default 4MB*
- PSRAM: *Disabled*.

### PlatformIO:
Building with **PlatformIO** is easy as well. Starting with Arduino ESP32 Core v3.1.1 the VSCode/PlatformIO IDE provides all the necessary board definition files. These *.json files provide the correct board definitions & settings.  

Just create a new project and give it a name, then go to board selection, enter "yb-" and choose your YB-ESP32-S3-*** board from the list thats popping up.

 ![](https://github.com/yellobyte/YB-ESP32-S3-ETH/raw/main/doc/YB-ESP32-S3-ETH_PlatformIO_board_selection.jpg)

Examples that need to be build with an older framework still come with a folder "boards" which keeps the necessary *.json board definition files. 

### Using the USB-C port:

With the board connected to your PC/Laptop you will see 3 additional devices. Two COM ports "Serial USB device" and "USB-Enhanced-Serial CH343" and a device "USB JTAG/serial debug unit" (the naming applies to Windows). 

You can connect a serial monitor program to device "USB-Enhanced-Serial CH343" and watch the serial output generated with Serial.print(). Please note: This connection is **not** affected by any board reset. 

Device "USB JTAG/serial debug unit" lets you simultaneously debug the board via the ESP32-S3 integrated JTAG debug circuitry. How to use the latter for debugging is explained in detail [**here**](https://github.com/yellobyte/ESP32-DevBoards-Getting-Started/tree/main/debugging).  
After a board reset a debugger tool (e.g. OpenOCD) will temporarily lose the JTAG debug connection since "USB JTAG/serial debug unit" is provided by the ESP32-S3 MCU (connected to the USB hub chip on GPIO19/20).

**Remark:**  

The PlatformIO builder scripts (*.json) for modules containing ESP32-S3/C3 already define the build flag _ARDUINO_USB_MODE=1_. This enables the USB-JTAG mode and disables USB-OTG. If not disabled or you want to override it you can (re-)define it in your platformio.ini control file. Normally you don't have to worry about it.
   
### Software Upload to the board:

The board contains the usual ESP32 reset and upload circuitry which makes automatic uploading new software to the board with your IDE a breeze. 

Below the PlatformIO log of flashing the dev board with provided software example [ESP32-S3-ETH-DHCP](https://github.com/yellobyte/YB-ESP32-S3-ETH/tree/main/examples/ArduinoIDE/ESP32-S3-ETH-DHCP). Please note: the example has been build with option `CORE_DEBUG_LEVEL=4` on PlatformIO (is equivalent to `Core Debug Level: "Debug"` on ArduinoIDE) and therefore produces lots of additional output (chip info, memory info, etc.) at startup.
```
Executing task: C:\Users\tj\.platformio\penv\Scripts\platformio.exe run --target upload --target monitor --upload-port COM3 --monitor-port COM3 

Processing release (platform: https://github.com/pioarduino/platform-espressif32/releases/download/stable/platform-espressif32.zip; framework: arduino; board: yb_esp32s3_eth)
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
Verbose mode can be enabled via `-v, --verbose` option
PYTHONEXE updated to penv environment: C:\Users\tj\.platformio\penv\Scripts\python.exe
CONFIGURATION: https://docs.platformio.org/page/boards/espressif32/yb_esp32s3_eth.html
PLATFORM: Espressif 32 (55.3.30) > YelloByte YB-ESP32-S3-ETH
HARDWARE: ESP32S3 240MHz, 320KB RAM, 8MB Flash
DEBUG: Current (esp-builtin) On-board (esp-builtin) External (cmsis-dap, esp-bridge, esp-prog, iot-bus-jtag, jlink, minimodule, olimex-arm-usb-ocd, olimex-arm-usb-ocd-h, olimex-arm-usb-tiny-h, olimex-jtag-tiny, tumpa)
PACKAGES: 
 - contrib-piohome @ 3.4.4 
 - framework-arduinoespressif32 @ 3.3.0 
 - framework-arduinoespressif32-libs @ 5.5.0+sha.b66b5448e0 
 - tool-dfuutil-arduino @ 1.11.0 
 - tool-esptoolpy @ 5.0.2 
 - tool-mkfatfs @ 2.0.1 
 - tool-mklittlefs @ 3.2.0 
 - tool-mklittlefs4 @ 4.0.2 
 - tool-mkspiffs @ 2.230.0 (2.30) 
 - toolchain-xtensa-esp-elf @ 14.2.0+20241119
*** Applied include path shortening for 325 framework paths ***
*** Path length reduced from 37477 to ~13517 characters ***
*** Estimated savings: 23400 characters ***
LDF: Library Dependency Finder -> https://bit.ly/configure-pio-ldf
LDF Modes: Finder ~ chain, Compatibility ~ soft
Found 42 compatible libraries
Scanning dependencies...
Dependency Graph
|-- Ethernet @ 2.0.2
Building in release mode
Retrieving maximum program size .pio\build\release\firmware.elf
Checking size .pio\build\release\firmware.elf
Advanced Memory Usage is available via "PlatformIO Home > Project Inspect"
RAM:   [=         ]   6.3% (used 20700 bytes from 327680 bytes)
Flash: [=         ]  10.1% (used 337139 bytes from 3342336 bytes)
Configuring upload protocol...
AVAILABLE: cmsis-dap, esp-bridge, esp-builtin, esp-prog, espota, esptool, iot-bus-jtag, jlink, minimodule, olimex-arm-usb-ocd, olimex-arm-usb-ocd-h, olimex-arm-usb-tiny-h, olimex-jtag-tiny, tumpa
CURRENT: upload_protocol = esptool
Looking for upload port...
Uploading .pio\build\release\firmware.bin
esptool v5.0.2
Serial port COM3:
Connecting...
Connected to ESP32-S3 on COM3:
Chip type:          ESP32-S3 (QFN56) (revision v0.2)
Features:           Wi-Fi, BT 5 (LE), Dual Core + LP Core, 240MHz, Embedded PSRAM 2MB (AP_3v3)
Crystal frequency:  40MHz
MAC:                20:6e:f1:e0:3c:a8

Uploading stub flasher...
Running stub flasher...
Stub flasher running.
Changing baud rate to 921600...
Changed.

Configuring flash size...
Auto-detected flash size: 8MB
Flash will be erased from 0x00000000 to 0x00004fff...
Flash will be erased from 0x00008000 to 0x00008fff...
Flash will be erased from 0x0000e000 to 0x0000ffff...
Flash will be erased from 0x00010000 to 0x00062fff...
SHA digest in image updated.
Compressed 20256 bytes to 13096...

Writing at 0x00000000 [░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░]   0.0% 0/13096 bytes...
...
...
...
Writing at 0x00062680 [██████████████████████████████] 100.0% 180240/180240 bytes... 
Wrote 337536 bytes (180240 compressed) at 0x00010000 in 2.4 seconds (1148.1 kbit/s).
Hash of data verified.

Hard resetting via RTS pin...

Please build project in debug configuration to get more details about an exception.
See https://docs.platformio.org/page/projectconf/build_configurations.html


--- Terminal on COM3 | 115200 8-N-1
--- Available filters and text transformations: colorize, debug, default, direct, esp32_exception_decoder, hexlify, log2file, nocontrol, printable, send_on_enter, time
--- More details at https://bit.ly/pio-monitor-filters
--- Quit: Ctrl+C | Menu: Ctrl+T | Help: Ctrl+T followed by Ctrl+H
00:51:10.583 > ESP-ROM:esp32s3-20210327
00:51:10.589 > Build:Mar 27 2021
00:51:10.589 > rst:0x1 (POWERON),boot:0x8 (SPI_FAST_FLASH_BOOT)
00:51:10.589 > SPIWP:0xee
00:51:10.589 > mode:DIO, clock div:1
00:51:10.589 > load:0x3fce2820,len:0x1180
00:51:10.674 > load:0x403c8700,len:0xc2c
00:51:10.674 > load:0x403cb700,len:0x311c
00:51:10.674 > entry 0x403c88b8
00:51:10.899 > [     2][I][esp32-hal-psram.c:104] psramAddToHeap(): PSRAM added to the heap.
00:51:10.914 > =========== Before Setup Start ===========
00:51:10.914 > Chip Info:
00:51:10.914 > ------------------------------------------
00:51:10.919 >   Model             : ESP32-S3
00:51:10.922 >   Package           : 0
00:51:10.922 >   Revision          : 0.02
00:51:10.922 >   Cores             : 2
00:51:10.929 >   CPU Frequency     : 240 MHz
00:51:10.929 >   XTAL Frequency    : 40 MHz
00:51:10.953 >   Features Bitfield : 0x00000012
00:51:10.953 >   Embedded Flash    : No
00:51:10.953 >   Embedded PSRAM    : No
00:51:10.953 >   2.4GHz WiFi       : Yes
00:51:10.953 >   Classic BT        : No
00:51:10.953 >   BT Low Energy     : Yes
00:51:10.953 >   IEEE 802.15.4     : No
00:51:10.953 > ------------------------------------------
00:51:10.953 > INTERNAL Memory Info:
00:51:10.959 > ------------------------------------------
00:51:10.959 >   Total Size        :   394764 B ( 385.5 KB)
00:51:10.965 >   Free Bytes        :   361544 B ( 353.1 KB)
00:51:10.969 >   Allocated Bytes   :    28316 B (  27.7 KB)
00:51:10.972 >   Minimum Free Bytes:   356396 B ( 348.0 KB)
00:51:10.972 >   Largest Free Block:   303092 B ( 296.0 KB)
00:51:10.979 > ------------------------------------------
00:51:10.986 > SPIRAM Memory Info:
00:51:10.986 > ------------------------------------------
00:51:10.989 >   Total Size        :  2097152 B (2048.0 KB)
00:51:10.989 >   Free Bytes        :  2095104 B (2046.0 KB)
00:51:10.999 >   Allocated Bytes   :        0 B (   0.0 KB)
00:51:11.003 >   Minimum Free Bytes:  2095104 B (2046.0 KB)
00:51:11.009 >   Largest Free Block:  2064372 B (2016.0 KB)
00:51:11.009 >   Bus Mode          : QSPI
00:51:11.009 > ------------------------------------------
00:51:11.009 > Flash Info:
00:51:11.019 > ------------------------------------------
00:51:11.019 >   Chip Size         :  8388608 B (8 MB)
00:51:11.019 >   Block Size        :    65536 B (  64.0 KB)
00:51:11.029 >   Sector Size       :     4096 B (   4.0 KB)
00:51:11.029 >   Page Size         :      256 B (   0.2 KB)
00:51:11.035 >   Bus Speed         : 80 MHz
00:51:11.039 >   Bus Mode          : QIO
00:51:11.039 > ------------------------------------------
00:51:11.049 > Partitions Info:
00:51:11.049 > ------------------------------------------
00:51:11.052 >                 nvs : addr: 0x00009000, size:    20.0 KB, type: DATA, subtype: NVS
00:51:11.072 >             otadata : addr: 0x0000E000, size:     8.0 KB, type: DATA, subtype: OTA
00:51:11.085 >                app0 : addr: 0x00010000, size:  3264.0 KB, type:  APP, subtype: OTA_0
00:51:11.106 >                app1 : addr: 0x00340000, size:  3264.0 KB, type:  APP, subtype: OTA_1
00:51:11.106 >              spiffs : addr: 0x00670000, size:  1536.0 KB, type: DATA, subtype: SPIFFS
00:51:11.129 >            coredump : addr: 0x007F0000, size:    64.0 KB, type: DATA, subtype: COREDUMP
00:51:11.129 > ------------------------------------------
00:51:11.129 > Software Info:
00:51:11.129 > ------------------------------------------
00:51:11.129 >   Compile Date/Time : Nov 25 2025 23:05:14
00:51:11.129 >   ESP-IDF Version   : v5.5-1-gb66b5448e0
00:51:11.129 >   Arduino Version   : 3.3.0
00:51:11.129 > ------------------------------------------
00:51:11.129 > Board Info:
00:51:11.129 > ------------------------------------------
00:51:11.129 >   Arduino Board     : YelloByte YB-ESP32-S3-ETH
00:51:11.134 >   Arduino Variant   : yb_esp32s3_eth
00:51:11.134 >   Core Debug Level  : 4
00:51:11.139 >   Arduino Runs Core : 1
00:51:11.139 >   Arduino Events on : 1
00:51:11.139 >   Arduino USB Mode  : 1
00:51:11.139 >   CDC On Boot       : 0
00:51:11.139 > ============ Before Setup End ============
00:51:12.253 > 
00:51:12.253 > Please make sure Ethernet cable is connected between board and switch and DHCP service is available in your LAN.
00:51:12.253 > =========== After Setup Start ============
00:51:12.262 > INTERNAL Memory Info:
00:51:12.262 > ------------------------------------------
00:51:12.270 >   Total Size        :   394764 B ( 385.5 KB)
00:51:12.272 >   Free Bytes        :   358764 B ( 350.4 KB)
00:51:12.272 >   Allocated Bytes   :    30968 B (  30.2 KB)
00:51:12.272 >   Minimum Free Bytes:   353616 B ( 345.3 KB)
00:51:12.288 >   Largest Free Block:   303092 B ( 296.0 KB)
00:51:12.289 > ------------------------------------------
00:51:12.289 > SPIRAM Memory Info:
00:51:12.289 > ------------------------------------------
00:51:12.289 >   Total Size        :  2097152 B (2048.0 KB)
00:51:12.304 >   Free Bytes        :  2093068 B (2044.0 KB)
00:51:12.305 >   Allocated Bytes   :     1844 B (   1.8 KB)
00:51:12.314 >   Minimum Free Bytes:  2093068 B (2044.0 KB)
00:51:12.314 >   Largest Free Block:  2064372 B (2016.0 KB)
00:51:12.320 > ------------------------------------------
00:51:12.322 > GPIO Info:
00:51:12.322 > ------------------------------------------
00:51:12.329 >   GPIO : BUS_TYPE[bus/unit][chan]
00:51:12.329 >   --------------------------------------
00:51:12.337 >     43 : UART_TX[0]
00:51:12.337 >     44 : UART_RX[0]
00:51:12.337 >     47 : GPIO
00:51:12.338 > ============ After Setup End =============
00:51:12.338 > blinkTask has started.
00:51:22.922 > Ethernet link is up.
00:51:23.021 > DHCP successful. Local IP: 192.168.1.37
00:51:33.026 > Ethernet link is up.
00:51:33.026 > Local IP: 192.168.1.37
....
```
### Integrating this board into your own PCB design projects:

Its easy. Folder [doc](https://github.com/yellobyte/YB-ESP32-S3-ETH/tree/main/doc) provides the Eagle library file **_yb-esp32-s3-eth.lbr_** containing the board. Most other PCB design software (e.g. KiCad) are able to import and use Eagle lib files. 

<p align="center"><img src="https://github.com/yellobyte/YB-ESP32-S3-ETH/raw/main/doc/Eagle_project_with_yb-esp32-s3-eth.jpg" height="250"/>&nbsp;&nbsp;&nbsp;&nbsp;<img src="https://github.com/yellobyte/YB-ESP32-S3-ETH/raw/main/doc/Eagle_project_with_yb-esp32-s3-eth2.jpg" height="250"/></p> 


## Final Remark for first usage: 
**All YB-ESP32-S3-ETH boards delivered have already been flashed with software example 'ESP32-S3-ETH-DHCP'.** This means the status LED 'IO47' blinks fast whith power applied and blinks slow with obtained IP address (DHCP via Ethernet).
