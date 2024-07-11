#include "esp8266_peri.h"
#include "esp8266_gpio.h"
#include "esp8266_spi.h"
#include "esp8266_wifi.h"
#include "esp8266_mqtt.h"
#include "esp8266_i2c.h"

// Define SPI pin registers and bit masks for ESP8266
#define MOSI_PIN 13
#define MISO_PIN 12
#define SCK_PIN 14
#define SS_PIN 15

#define OLED_SDA 4
#define OLED_SCL 5
#define OLED_ADDRESS 0x3C

#define BUZZER_PIN 16

// MFRC522 Registers
#define PCD_IDLE          0x00
#define PCD_AUTHENT       0x0E
#define PCD_RECEIVE       0x08
#define PCD_TRANSMIT      0x04
#define PCD_TRANSCEIVE    0x0C
#define PCD_RESETPHASE    0x0F
#define PCD_CALCCRC       0x03
#define PICC_REQIDL       0x26
#define PICC_ANTICOLL     0x93
#define FIFODataReg       0x09
#define FIFOLevelReg      0x0A
#define CommandReg        0x01
#define BitFramingReg     0x0D
#define CommIrqReg        0x04
#define ErrorReg          0x06
#define Status2Reg        0x08
#define ControlReg        0x0C

const char* ssid = "your_ssid";
const char* password = "your_password";
const char* mqtt_server = "broker.hivemq.com";
const char* mqtt_topic = "rfid/updates";

// Function prototypes
void SPI_Init(void);
void SPI_WriteByte(uint8_t byte);
uint8_t SPI_ReadByte(void);

void I2C_Init(void);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_WriteByte(uint8_t byte);
void OLED_Command(uint8_t cmd);
void OLED_Data(uint8_t data);
void OLED_Init(void);

void GPIO_Init(void);
void Buzzer_On(void);
void Buzzer_Off(void);

void WiFi_Init(void);
void MQTT_Connect(void);
void MQTT_Publish(const char *topic, const char *payload);

void RFID_Init(void);
uint8_t RFID_ReadRegister(uint8_t reg);
void RFID_WriteRegister(uint8_t reg, uint8_t value);
uint8_t RFID_Request(uint8_t reqMode, uint8_t *TagType);

void delay(uint32_t ms);

// MQTT client instance
esp_mqtt_client_handle_t mqtt_client;

void setup(void) {
    // Initialize hardware components
    SPI_Init();
    OLED_Init();
    GPIO_Init();
    WiFi_Init();
    MQTT_Connect();
    RFID_Init();
}

void loop(void) {
    // Ensure the MQTT client is connected
    if (!esp_mqtt_client_is_connected(mqtt_client)) {
        MQTT_Connect();
    }

    // Look for new cards
    uint8_t status, str[MAX_LEN];
    status = RFID_Request(PICC_REQIDL, str);

    if (status == MI_OK) {
        uint8_t uid[10];
        // Assuming RFID_ReadCardSerial() fills uid and returns length
        uint8_t uid_len = RFID_ReadCardSerial(uid);

        // Show UID on serial monitor and OLED display
        char uidString[30] = {0};
        for (uint8_t i = 0; i < uid_len; i++) {
            sprintf(uidString + strlen(uidString), " %02X", uid[i]);
        }

        OLED_Command(0xAE); // Display OFF
        for (uint8_t i = 0; i < strlen(uidString); i++) {
            OLED_Data(uidString[i]);
        }
        OLED_Command(0xAF); // Display ON

        // Sound the buzzer
        Buzzer_On();
        delay(100);
        Buzzer_Off();

        // Publish UID to MQTT broker
        if (esp_mqtt_client_is_connected(mqtt_client)) {
            MQTT_Publish(mqtt_topic, uidString);
        }
    }
}

void SPI_Init(void) {
    // Configure MOSI, SCK, SS as output
    GPIO_ENABLE |= (1 << MOSI_PIN) | (1 << SCK_PIN) | (1 << SS_PIN);

    // Configure MISO as input
    GPIO_ENABLE &= ~(1 << MISO_PIN);

    // Initialize SPI
    SPI1C |= SPI1CPOL | SPI1CPHA | SPI1MSB; // CPOL=1, CPHA=1, MSB first
    SPI1U = SPIUMOSI | SPIUDUPLEX; // Enable full-duplex
}

void SPI_WriteByte(uint8_t byte) {
    SPI1W0 = byte; // Load byte into SPI data register
    SPI1CMD |= SPIBUSY; // Start transmission
    while (SPI1CMD & SPIBUSY); // Wait for transmission to complete
}

uint8_t SPI_ReadByte(void) {
    SPI1CMD |= SPIBUSY; // Start transmission
    while (SPI1CMD & SPIBUSY); // Wait for transmission to complete
    return SPI1W0 & 0xFF; // Return received byte
}

void I2C_Init(void) {
    // Configure SDA and SCL as output
    GPIO_ENABLE |= (1 << OLED_SDA) | (1 << OLED_SCL);
}

void I2C_Start(void) {
    GPIO_OUTPUT_SET(OLED_SDA, 1);
    GPIO_OUTPUT_SET(OLED_SCL, 1);
    GPIO_OUTPUT_SET(OLED_SDA, 0);
    GPIO_OUTPUT_SET(OLED_SCL, 0);
}

void I2C_Stop(void) {
    GPIO_OUTPUT_SET(OLED_SDA, 0);
    GPIO_OUTPUT_SET(OLED_SCL, 1);
    GPIO_OUTPUT_SET(OLED_SDA, 1);
}

void I2C_WriteByte(uint8_t byte) {
    for (uint8_t i = 0; i < 8; i++) {
        GPIO_OUTPUT_SET(OLED_SDA, (byte & 0x80) != 0);
        GPIO_OUTPUT_SET(OLED_SCL, 1);
        GPIO_OUTPUT_SET(OLED_SCL, 0);
        byte <<= 1;
    }
    // Ack bit
    GPIO_OUTPUT_SET(OLED_SDA, 1);
    GPIO_OUTPUT_SET(OLED_SCL, 1);
    GPIO_OUTPUT_SET(OLED_SCL, 0);
}

void OLED_Command(uint8_t cmd) {
    I2C_Start();
    I2C_WriteByte((OLED_ADDRESS << 1) | 0);
    I2C_WriteByte(0x00); // Command mode
    I2C_WriteByte(cmd);
    I2C_Stop();
}

void OLED_Data(uint8_t data) {
    I2C_Start();
    I2C_WriteByte((OLED_ADDRESS << 1) | 0);
    I2C_WriteByte(0x40); // Data mode
    I2C_WriteByte(data);
    I2C_Stop();
}

void OLED_Init(void) {
    I2C_Init();

    OLED_Command(0xAE); // Display OFF
    OLED_Command(0xD5); // Set Display Clock Divide Ratio / Oscillator Frequency
    OLED_Command(0x80); // Display Clock Divide Ratio / Oscillator Frequency
    OLED_Command(0xA8); // Set Multiplex Ratio
    OLED_Command(0x3F); // Multiplex Ratio for 128x64 (64-1)
    OLED_Command(0xD3); // Set Display Offset
    OLED_Command(0x00); // Display Offset
    OLED_Command(0x40); // Set Display Start Line
    OLED_Command(0x8D); // Charge Pump Setting
    OLED_Command(0x14); // Enable Charge Pump
    OLED_Command(0x20); // Memory Addressing Mode
    OLED_Command(0x00); // Horizontal Addressing Mode
    OLED_Command(0xA1); // Set Segment Re-map
    OLED_Command(0xC8); // Set COM Output Scan Direction
    OLED_Command(0xDA); // Set COM Pins Hardware Configuration
    OLED_Command(0x12); // COM Pins Hardware Configuration
    OLED_Command(0x81); // Set Contrast Control
    OLED_Command(0xCF); // Contrast Control
    OLED_Command(0xD9); // Set Pre-charge Period
    OLED_Command(0xF1); // Set Pre-charge Period (0x22 for SSD1306)
    OLED_Command(0xDB); // Set VCOMH Deselect Level
    OLED_Command(0x40); // VCOMH Deselect Level
    OLED_Command(0xA4); // Entire Display ON
    OLED_Command(0xA6); // Set Normal Display
    OLED_Command(0xAF); // Display ON
}

void GPIO_Init(void) {
    // Configure buzzer pin as output
    GPIO_ENABLE |= (1 << BUZZER_PIN);
    GPIO_OUTPUT_SET(BUZZER_PIN, 0);
}

void Buzzer_On(void) {
    GPIO_OUTPUT_SET(BUZZER_PIN, 1)
}

void Buzzer_Off(void) {
    GPIO_OUTPUT_SET(BUZZER_PIN, 0);
}

void WiFi_Init(void) {
    wifi_set_opmode(STATION_MODE);
    struct station_config stationConf;
    strcpy((char*)stationConf.ssid, ssid);
    strcpy((char*)stationConf.password, password);
    wifi_station_set_config(&stationConf);
    wifi_station_connect();
    while (wifi_station_get_connect_status() != STATION_GOT_IP) {
        delay(100);
    }
}

void MQTT_Connect(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = mqtt_server,
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(mqtt_client);
}

void MQTT_Publish(const char *topic, const char *payload) {
    esp_mqtt_client_publish(mqtt_client, topic, payload, 0, 1, 0);
}

void RFID_Init(void) {
    // Initialize RFID reader
    RFID_WriteRegister(CommandReg, PCD_RESETPHASE);
    RFID_WriteRegister(TModeReg, 0x8D);
    RFID_WriteRegister(TPrescalerReg, 0x3E);
    RFID_WriteRegister(TReloadRegL, 30);
    RFID_WriteRegister(TReloadRegH, 0);
    RFID_WriteRegister(TxASKReg, 0x40);
    RFID_WriteRegister(ModeReg, 0x3D);
    RFID_AntennaOn();
}

uint8_t RFID_ReadRegister(uint8_t reg) {
    SPI_WriteByte((reg << 1) & 0x7E);
    return SPI_ReadByte();
}

void RFID_WriteRegister(uint8_t reg, uint8_t value) {
    SPI_WriteByte(((reg << 1) & 0x7E) | 0x80);
    SPI_WriteByte(value);
}

uint8_t RFID_Request(uint8_t reqMode, uint8_t *TagType) {
    uint8_t status;
    uint32_t backBits;

    RFID_WriteRegister(BitFramingReg, 0x07);

    TagType[0] = reqMode;
    status = RFID_ToCard(PCD_TRANSCEIVE, TagType, 1, TagType, &backBits);

    if ((status != MI_OK) || (backBits != 0x10)) {
        status = MI_ERR;
    }

    return status;
}

uint8_t RFID_ToCard(uint8_t command, uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint32_t *backLen) {
    uint8_t status = MI_ERR;
    uint8_t irqEn = 0x00;
    uint8_t waitIRq = 0x00;
    uint8_t lastBits;
    uint8_t n;
    uint32_t i;

    switch (command) {
        case PCD_AUTHENT:
            irqEn = 0x12;
            waitIRq = 0x10;
            break;
        case PCD_TRANSCEIVE:
            irqEn = 0x77;
            waitIRq = 0x30;
            break;
        default:
            break;
    }

    RFID_WriteRegister(CommIEnReg, irqEn | 0x80);
    RFID_WriteRegister(CommIrqReg, 0x7F);
    RFID_WriteRegister(FIFOLevelReg, 0x80);
    RFID_WriteRegister(CommandReg, PCD_IDLE);

    for (i = 0; i < sendLen; i++) {
        RFID_WriteRegister(FIFODataReg, sendData[i]);
    }

    RFID_WriteRegister(CommandReg, command);
    if (command == PCD_TRANSCEIVE) {
        RFID_WriteRegister(BitFramingReg, 0x80);
    }

    i = 2000;
    do {
        n = RFID_ReadRegister(CommIrqReg);
        i--;
    } while ((i != 0) && !(n & 0x01) && !(n & waitIRq));

    RFID_WriteRegister(BitFramingReg, 0x00);

    if (i != 0) {
        if (!(RFID_ReadRegister(ErrorReg) & 0x1B)) {
            status = MI_OK;
            if (n & irqEn & 0x01) {
                status = MI_NOTAGERR;
            }

            if (command == PCD_TRANSCEIVE) {
                n = RFID_ReadRegister(FIFOLevelReg);
                lastBits = RFID_ReadRegister(ControlReg) & 0x07;
                if (lastBits) {
                    *backLen = (n - 1) * 8 + lastBits;
                } else {
                    *backLen = n * 8;
                }

                if (n == 0) {
                    n = 1;
                }
                if (n > MAX_LEN) {
                    n = MAX_LEN;
                }

                for (i = 0; i < n; i++) {
                    backData[i] = RFID_ReadRegister(FIFODataReg);
                }
            }
        } else {
            status = MI_ERR;
        }
    }

    return status;
}

uint8_t RFID_ReadCardSerial(uint8_t *serial) {
    uint8_t status;
    uint8_t i;
    uint8_t serNumCheck = 0;
    uint8_t serNum[5];

    RFID_WriteRegister(BitFramingReg, 0x00);

    serNum[0] = PICC_ANTICOLL;
    serNum[1] = 0x20;
    status = RFID_ToCard(PCD_TRANSCEIVE, serNum, 2, serNum, &backBits);

    if (status == MI_OK) {
        for (i = 0; i < 4; i++) {
            serial[i] = serNum[i];
            serNumCheck ^= serNum[i];
        }
        if (serNumCheck != serNum[4]) {
            status = MI_ERR;
        }
    }

    return status;
}

void delay(uint32_t ms) {
    os_delay_us(ms * 1000);
}

void RFID_AntennaOn(void) {
    uint8_t temp = RFID_ReadRegister(TxControlReg);
    if (!(temp & 0x03)) {
        RFID_WriteRegister(TxControlReg, temp | 0x03);
    }
}