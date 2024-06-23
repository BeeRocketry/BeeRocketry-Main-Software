#include <Arduino.h>

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
#define RF_TX PA3
#define RF_RX PA2
#define RF_AUX PB1
#define RF_M0 PB3
#define RF_M1 PB2

HardwareSerial SerialRF(RF_TX, RF_RX);

/*
----------------
 Temel Makrolar
    MAX_TX_BUFFER_SIZE --> RF modülün maksimum sub-byte miktarı
    TIMEOUT_AUX_RESPOND --> AUX pininin müsait duruma gelmesini beklenecek maksimum süre
----------------
*/
#define MAX_TX_BUFFER_SIZE 58
#define TIMEOUT_AUX_RESPOND 500

/*
--------------------------------------------------------------------
 Debug Printer Makrosu
    DEBUG_MODE makrosu tanimlandığında debug mesajlarının
    seri porta yazdırılmasını sağlayan makro tanımıdır.

        DEBUG_PRINTER --> Yazdırılacak olan seri port aygıtını seçer
--------------------------------------------------------------------
*/

#define DEBUG_PRINTER Serial1

#ifdef DEBUG_MODE
    #define DEBUG_PRINT(...) {DEBUG_PRINTER.print(__VA_ARGS__); }
    #define DEBUG_PRINTLN(...) {DEBUG_PRINTER.println(__VA_ARGS__); }
#else
    #define DEBUG_PRINT(...) {}
    #define DEBUG_PRINTLN(...) {}
#endif

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
Status waitAUX(unsigned long timeout);
Status RFBegin(RF_UART_PARITY parity = UARTPARITY_8N1, RF_UART_BAUD baud = UARTBAUDRATE_9600, RF_AIR_DATA airdata = AIRDATARATE_03k, RF_TRANS_MODE transmode = FIXEDMODE, RF_IO_MODE IOmode = IO_PUSHPULL, RF_WIRELESS wirelesswake = WIRELESSWAKEUP_250, RF_FEC fecmode = FEC_ON, RF_TRANS_POWER transpower = TRANSMISSIONPOWER_30);
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


/*
------------------------------
 Yardimci Fonksiyon Tanimlari
    Ana fonksiyonlarda kullanılacak olan yardimci
    fonksiyonlardir.

        managedDelay --> Interruptları kesmeden delay sağlar
        Diğer fonksiyonlar --> Config ayarlarını yazdırırken stringleri döndürecek olan fonksiyonlar
------------------------------
*/
void managedDelay(unsigned long timeout){
    unsigned long t = millis();

    if((unsigned long) (t + timeout) == 0){
        t = 0;
    }

    while((millis()-t) < timeout){
        yield();
    }
}

String getUARTBaudRate(byte uartbaud){
    switch (uartbaud)
    {
    case UARTBAUDRATE_1200:
        return F("1200 bps");
        break;
    
    case UARTBAUDRATE_2400:
        return F("2400 bps");
        break;

    case UARTBAUDRATE_4800:
        return F("4800 bps");
        break;

    case UARTBAUDRATE_9600:
        return F("9600 bps (Varsayilan)");
        break;

    case UARTBAUDRATE_19200:
        return F("19200 bps");
        break;

    case UARTBAUDRATE_38400:
        return F("38400 bps");
        break;

    case UARTBAUDRATE_57600:
        return F("57600 bps");
        break;

    case UARTBAUDRATE_115200:
        return F("115200 bps");
        break;
    }
}

String getUARTParity(byte uartparity){
    switch (uartparity)
    {
    case UARTPARITY_8N1:
        return F("8 Bit, Parity Yok, 1 Durdurma Biti");
        break;
    
    case UARTPARITY_8E1:
        return F("8 Bit, Çift Parity, 1 Durdurma Biti");
        break;

    case UARTPARITY_8O1:
        return F("8 Bit, Tek Parity, 1 Durdurma Biti");
        break;
    }
}

String getAirData(byte airdata){
    switch (airdata)
    {
    case AIRDATARATE_03k:
        return F("0.3k bps");
        break;
    
    case AIRDATARATE_12k:
        return F("1.2k bps");
        break;

    case AIRDATARATE_24k:
        return F("2.4k bps (Varsayilan)");
        break;

    case AIRDATARATE_48k:
        return F("4.8k bps");
        break;

    case AIRDATARATE_96k:
        return F("9.6k bps");
        break;

    case AIRDATARATE_192k:
        return F("19.2k bps");
        break;
    }
}

String getTransmissionType(byte transmissiontype){
    switch (transmissiontype)
    {
    case TRANSPARENTMODE:
        return F("Seffaf Mod");
        break;
    
    case FIXEDMODE:
        return F("Sabit Kanal Modu");
        break;
    }
}

String getIOMode(byte iotype){
    switch (iotype)
    {
    case IO_OPENDRAIN:
        return F("IO Open Drain Modu");
        break;
    
    case IO_PUSHPULL:
        return F("IO Push Pull Modu");
        break;
    }  
}

String getWirelessWakeup(byte wireless){
    switch (wireless)
    {
    case WIRELESSWAKEUP_250:
        return F("250 ms");
        break;
    
    case WIRELESSWAKEUP_500:
        return F("500 ms");
        break;

    case WIRELESSWAKEUP_750:
        return F("750 ms");
        break;
    
    case WIRELESSWAKEUP_1000:
        return F("1000 ms");
        break;
    
    case WIRELESSWAKEUP_1250:
        return F("1250 ms");
        break;
    
    case WIRELESSWAKEUP_1500:
        return F("1500 ms");
        break;

    case WIRELESSWAKEUP_1750:
        return F("1750 ms");
        break;

    case WIRELESSWAKEUP_2000:
        return F("2000 ms");
        break;
    } 
}

String getFECFilter(byte fecbyte){
    switch (fecbyte)
    {
    case FEC_ON:
        return F("Aktif");
        break;
    
    case FEC_OFF:
        return F("Devre Disi");
        break;
    }  
}

String getTranmissionPower(byte transmissionpower){
    switch (transmissionpower)
    {
    case TRANSMISSIONPOWER_21:
        return F("21 dBm");
        break;
    
    case TRANSMISSIONPOWER_24:
        return F("24 dBm");
        break;
    
    case TRANSMISSIONPOWER_27:
        return F("27 dBm");
        break;
    
    case TRANSMISSIONPOWER_30:
        return F("30 dBm");
        break;
    }
}