#include <Arduino.h>

#define RF_TX PA3
#define RF_RX PA2
#define RF_AUX PB1
#define RF_M0 PB3
#define RF_M1 PB2

HardwareSerial SerialRF (RF_RX, RF_TX);

#define MAX_TX_BUFFER_SIZE 58

// SPED 7, 6 bit
#define UARTPARITY_8N1 0b00
#define UARTPARITY_8O1 0b01
#define UARTPARITY_8E1 0b10

// SPED 5, 4, 3 bit
#define UARTBAUDRATE_1200 0b000
#define UARTBAUDRATE_2400 0b001
#define UARTBAUDRATE_4800 0b010
#define UARTBAUDRATE_9600 0b011
#define UARTBAUDRATE_19200 0b100
#define UARTBAUDRATE_38400 0b101
#define UARTBAUDRATE_57600 0b110
#define UARTBAUDRATE_115200 0b111

// SPED 2, 1, 0
#define AIRDATARATE_03k 0b000
#define AIRDATARATE_12k 0b001
#define AIRDATARATE_24k 0b010
#define AIRDATARATE_48k 0b011
#define AIRDATARATE_96k 0b100
#define AIRDATARATE_192k 0b101

// OPTION 7 bit
#define TRANSPARENTMODE 0b0
#define FIXEDMODE 0b1
#define BROADCASTMODE 0b11111

// OPTION 6 bit
#define IO_PUSHPULL 0b0
#define IO_OPENDRAIN 0b1

// OPTION 5, 4, 3 bit
#define WIRELESSWAKEUP_250 0b000
#define WIRELESSWAKEUP_500 0b001
#define WIRELESSWAKEUP_750 0b010
#define WIRELESSWAKEUP_1000 0b011
#define WIRELESSWAKEUP_1250 0b100
#define WIRELESSWAKEUP_1500 0b101
#define WIRELESSWAKEUP_1750 0b110
#define WIRELESSWAKEUP_2000 0b111

// OPTION 2 bit
#define FEC_OFF 0b0
#define FEC_ON 0b1

// OPTION 1, 0 bit
#define TRANSMISSIONPOWER_30 0b00
#define TRANSMISSIONPOWER_27 0b01
#define TRANSMISSIONPOWER_24 0b10
#define TRANSMISSIONPOWER_21 0b11

struct Sped{
    uint8_t UARTParity = UARTPARITY_8N1;
    uint8_t UARTBaud = UARTBAUDRATE_9600;
    uint8_t AirDataRate = AIRDATARATE_24k;
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