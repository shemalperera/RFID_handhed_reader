# RFID_handhed_reader  

## Table of Contents
- [Overview](#overview)
- [Hardware Components](#hardware-components)
- [Software Components](#software-components)
- [System Architecture](#system-architecture)
- [Schematics](#schematics)
- [Contributors](#contributors)
## Overview

This project involves designing and implementing a comprehensive system integrating various hardware components and communication protocols. The core elements include an RFID reader, an OLED display, a buzzer, and an ESP8266 microcontroller for Wi-Fi and MQTT communication.

<p align="center">
  <img src="https://github.com/NethmalWDI/RFID_handhed_reader/blob/main/Images/enclosure/final%20assembly.jpg" alt="Final Assembly">
</p>



## Hardware Components
- **ESP8266 Microcontroller**
- **RFID Reader (MFRC522)**
- **OLED Display (128x64)**
- **Buzzer**

## Software Components
- **ESP-IDF**: Development framework for ESP8266.
- **Arduino Core for ESP32**: Used for programming the ESP8266.
- **MQTT Library**: For MQTT communication.
- **SPI Library**: For communication with the RFID reader.
- **I2C Library**: For communication with the OLED display.

## System Architecture

The system consists of the following modules:
1. **RFID Custom PCB**: Reads RFID tags and sends the tag information to the ESP8266.
2. **ESP8266 MCU based PCB**: Processes the RFID tag information, displays it on the OLED, and publishes the data to an MQTT broker.
3. **Display Module**: Shows the RFID tag information on the OLED display.
4. **Buzzer Module**: Provides an audible indication when an RFID tag is detected.
5. **Wi-Fi integration**: Handles Wi-Fi connectivity and ensures the ESP8266 is connected to the local network.
6. **MQTT Module**: Manages communication with the MQTT broker and publishes the RFID tag data.

## Schematics
### Main pcb
<p align="center">
  <img src="https://github.com/NethmalWDI/RFID_handhed_reader/blob/main/Images/pcb/schematic_main.png" alt="Final Assembly">
</p>

### Reader PCB
<p align="center">
  <img src="https://github.com/NethmalWDI/RFID_handhed_reader/blob/main/Images/pcb/schematic_rfid.png" alt="Final Assembly">
</p>

## Contributors  

[Shemal Perera](https://github.com/shemalperera)   
[Imasha Nethmal](https://github.com/NethmalWDI)

