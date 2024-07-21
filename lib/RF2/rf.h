#ifndef RF_H
#define RF_H

#include <Arduino.h>

#include "debugprinter.h"

/*
------------------------
 Pin Ayarlama Makroları
    Pin bağlantılarının yapıldığı makrolardır.

        E32     |   MCU
        RF_TX  ---  MCU_RX   (4.7k Pull-Up)
        RF_RX  ---  MCU_TX   (4.7k Pull-Up)
        RF_AUX ---  MCU_GPIO (Input) (4.7k Pull-Up)
        RF_M0  ---  MCU_GPIO (Output)
        RF_M1  ---  MCU_GPIO (Output)
------------------------
*/
/* Blackpill Transmitter
#define RF_TX PB6
#define RF_RX PB7
#define RF_AUX PA7
#define RF_M0 PA5
#define RF_M1 PA6*/

/* Blackpill Receiver*/
#define RF_TX PA9
#define RF_RX PA10
#define RF_AUX PB2
#define RF_M0 PB0
#define RF_M1 PB1


extern HardwareSerial SerialRF;

/*
----------------
 Temel Makrolar
    MAX_TX_BUFFER_SIZE --> RF modülün maksimum sub-byte miktarı
    TIMEOUT_AUX_RESPOND --> AUX pininin müsait duruma gelmesini beklenecek maksimum süre
----------------
*/
#define MAX_TX_BUFFER_SIZE 58
#define TIMEOUT_AUX_RESPOND 1500

/*
---------------------------------------------------------------------
 Error Durum Tanimlari
    Aşağıdaki durum tanimlari çalışma süresince çalışan
    fonksiyonlarda karşılabilecek tüm durumları kapsamaktadir.

    Fonksiyonlar hata durumlarına göre bu mesajlari dönecektir.

        E32_Success --> Başarılı
        E32_Timeout --> Fonksiyon Zaman Aşımına Uğradı
        E32_CrcBroken --> Paketlerin CRC8 şifreleri uyuşmuyor
        E32_FailureMode --> Mod ayarlamasinda beklenmeyen input
        E32_NoMessage --> Mesaj gelmemesi
---------------------------------------------------------------------
*/
typedef enum Error_Status{
    E32_Success = 1,
    E32_Timeout,
    E32_CrcBroken,
    E32_FailureMode,
    E32_NoMessage,
} Status;

/*
------------------------
 Ayar Öntanim Makrolari
------------------------
*/
// SPED 7, 6 bit
typedef enum RF_UART_PARITY{
    UARTPARITY_8N1 = 0b00,
    UARTPARITY_8O1 = 0b01,
    UARTPARITY_8E1 = 0b10,
}RF_UART_PARITY;

// SPED 5, 4, 3 bit
typedef enum RF_UART_BAUD{
    UARTBAUDRATE_1200 = 0b000,
    UARTBAUDRATE_2400 = 0b001,
    UARTBAUDRATE_4800 = 0b010,
    UARTBAUDRATE_9600 = 0b011,
    UARTBAUDRATE_19200 = 0b100,
    UARTBAUDRATE_38400 = 0b101,
    UARTBAUDRATE_57600 = 0b110,
    UARTBAUDRATE_115200 = 0b111,
}RF_UART_BAUD;

// SPED 2, 1, 0
typedef enum RF_AIR_DATA{
    AIRDATARATE_03k = 0b000,
    AIRDATARATE_12k = 0b001,
    AIRDATARATE_24k = 0b010,
    AIRDATARATE_48k = 0b011,
    AIRDATARATE_96k = 0b100,
    AIRDATARATE_192k = 0b101,
}RF_AIR_DATA;

// OPTION 7 bit
typedef enum RF_TRANS_MODE{
    TRANSPARENTMODE = 0b0,
    FIXEDMODE = 0b1,
    BROADCASTMODE = 0b11111,
}RF_TRANS_MODE;

// OPTION 6 bit
typedef enum RF_IO_MODE{
    IO_PUSHPULL = 0b0,
    IO_OPENDRAIN = 0b1,
}RF_IO_MODE;

// OPTION 5, 4, 3 bit
typedef enum RF_WIRELESS{
    WIRELESSWAKEUP_250 = 0b000,
    WIRELESSWAKEUP_500 = 0b001,
    WIRELESSWAKEUP_750 = 0b010,
    WIRELESSWAKEUP_1000 = 0b011,
    WIRELESSWAKEUP_1250 = 0b100,
    WIRELESSWAKEUP_1500 = 0b101,
    WIRELESSWAKEUP_1750 = 0b110,
    WIRELESSWAKEUP_2000 = 0b111,
}RF_WIRELESS;

// OPTION 2 bit
typedef enum RF_FEC{
    FEC_OFF = 0b0,
    FEC_ON = 0b1,
}RF_FEC;

// OPTION 1, 0 bit
typedef enum RF_TRANS_POWER{
    TRANSMISSIONPOWER_30 = 0b00,
    TRANSMISSIONPOWER_27 = 0b01,
    TRANSMISSIONPOWER_24 = 0b10,
    TRANSMISSIONPOWER_21 = 0b11,
}RF_TRANS_POWER;

/* 
--------------------
 Struct Tanimlari
--------------------
*/
struct Sped{
    uint8_t UARTParity = UARTPARITY_8N1;
    uint8_t UARTBaud = UARTBAUDRATE_9600;
    uint8_t AirDataRate = AIRDATARATE_03k;
};

struct Option{
    uint8_t TransmissionMode = TRANSPARENTMODE;
    uint8_t IODriver = IO_PUSHPULL;
    uint8_t WirelessWakeUp = WIRELESSWAKEUP_250;
    uint8_t FECset = FEC_ON;
    uint8_t TransmissionPower = TRANSMISSIONPOWER_30;
};

struct ConfigRF{
    uint8_t AddressHigh;
    uint8_t AddressLow;
    struct Sped RFSped;
    struct Option RFOption;
    uint8_t Channel = 0x17;
};


/* 
-----------------------
 Ana Fonksiyon Prototipleri
-----------------------
*/
uint8_t calculateCRC8(const uint8_t *data, size_t length);
void clearSerialBuffer();
Status waitAUX(unsigned long timeout);
Status RFBegin(struct ConfigRF *getConfs, uint8_t HighAddress, uint8_t LowAddress, uint8_t channel, RF_UART_PARITY parity, RF_UART_BAUD baud, RF_AIR_DATA airdata, RF_TRANS_MODE transmode, RF_IO_MODE IOmode, RF_WIRELESS wirelesswake, RF_FEC fecmode, RF_TRANS_POWER transpower);
Status setSettings(struct ConfigRF confs);
Status getSettings(struct ConfigRF *confs);
Status receiveSingleData(uint8_t *data);
Status receiveDataPacket(uint8_t *data, size_t size);
Status sendFixedSingleData(uint8_t AddressHigh, uint8_t AddressLow, uint8_t Channel, uint8_t data);
Status sendTransparentSingleData(uint8_t data);
Status sendFixedDataPacket(uint8_t AddressHigh, uint8_t AddressLow, uint8_t Channel, uint8_t *data, size_t size);
Status sendBroadcastDataPacket(uint8_t Channel, uint8_t *data, size_t size);
Status sendTransparentDataPacket(uint8_t *data, size_t size);
Status setTransmissionMode(struct ConfigRF *config, uint8_t Mode);
Status setAddresses(struct ConfigRF *config, uint8_t AddHigh, uint8_t AddLow);
Status setChannel(struct ConfigRF *config, uint8_t channel);
Status setTransmissionPower(struct ConfigRF *config, uint8_t power);
Status setIODriver(struct ConfigRF *config, uint8_t driver);
Status setFECSettings(struct ConfigRF *config, uint8_t mode);
Status setWirelesWakeup(struct ConfigRF *config, uint8_t time);
Status setUARTParity(struct ConfigRF *config, uint8_t paritybyte);
Status setUARTBaudRate(struct ConfigRF *config, uint8_t baudrate);
Status setAirDataRate(struct ConfigRF *config, uint8_t airdatarate);
Status setSerialBaudRateBegin(struct ConfigRF confs);
int8_t setSerialParityBegin(struct ConfigRF confs);
String getTranmissionPower(byte transmissionpower);
String getFECFilter(byte fecbyte);
String getWirelessWakeup(byte wireless);
String getIOMode(byte iotype);
String getTransmissionType(byte transmissiontype);
String getAirData(byte airdata);
String getUARTParity(byte uartparity);
String getUARTBaudRate(byte uartbaud);
void managedDelay(unsigned long timeout);

#endif