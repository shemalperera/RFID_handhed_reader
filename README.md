# RFID_handhed_reader  



# Final Project: Design Documentation v2.0

## Overview

This project involves the design and implementation of a comprehensive system integrating various hardware components and communication protocols. The core elements include an RFID reader, an OLED display, a buzzer, and an ESP32 microcontroller for Wi-Fi and MQTT communication.

## Table of Contents
- [Overview](#overview)
- [Hardware Components](#hardware-components)
- [Software Components](#software-components)
- [System Architecture](#system-architecture)
- [Circuit Diagram](#circuit-diagram)
- [Contribution](#contributing)


## Hardware Components
- **ESP32 Microcontroller**
- **RFID Reader (MFRC522)**
- **OLED Display (128x64)**
- **Buzzer**
- **Miscellaneous**: Resistors, capacitors, connectors, etc.

## Software Components
- **ESP-IDF**: Development framework for ESP32.
- **Arduino Core for ESP32**: Used for programming the ESP32.
- **MQTT Library**: For MQTT communication.
- **SPI Library**: For communication with the RFID reader.
- **I2C Library**: For communication with the OLED display.

## System Architecture

The system consists of the following modules:
1. **RFID custom pcb**: Reads RFID tags and sends the tag information to the ESP32.
2. **ESP32 Module-based pcb**: Processes the RFID tag information, displays it on the OLED, and publishes the data to an MQTT broker.
3. **Display Module**: Shows the RFID tag information on the OLED display.
4. **Buzzer Module**: Provides an audible indication when an RFID tag is detected.
5. **Wi-Fi integration**: Handles Wi-Fi connectivity and ensures the ESP32 is connected to the local network.
6. **MQTT Module**: Manages communication with the MQTT broker and publishes the RFID tag data.

## Schematics
### main pcb

![](RFID_handhed_reader/images/pcb/schematic_main.png)
### antenna PCB
![](images/pcb/schematic_rfid.png)

## Final results
![](images/pcb/soldered_pcb_main.png)

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/yourusername/your-repo-name.git
   cd your-repo-name
