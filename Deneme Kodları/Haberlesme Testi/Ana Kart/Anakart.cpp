#include <Arduino.h>

#define DEBUG_MODE

#undef Serial
#define Serial SeriPort

#include "debugprinter.h"
#include <HardwareSerial.h>
#include "gps.h"
#include "main.h"
#include "rf.h"

#define UartRXPini PA3
#define UartTXPini PA2
#define UartBaudRate 115200

#define GPSUartRXPini PA7
#define GPSUartTXPini PA6
#define GPSBaudRate 9600

#define RFLowAdresi 0x02
#define RFHighAdresi 0x03
#define RFKanal 23U

#define RFGondericiLowAdresi 0x01
#define RFGondericiHighAdresi 0x03
#define RFGondericiKanal 23U

#define BeklemeSuresi 1000

HardwareSerial SeriPort(UartRXPini, UartTXPini);
HardwareSerial GPSPort(GPSUartRXPini, GPSUartTXPini);

typedef union{
    float floatvar;
    uint8_t bytevar[4];
}Float2ByteConverter;

typedef struct{
    float enlem = 0;
    float boylam = 0;
    float irtifa = 0;
}GPSVeri;

GPSVeri AnaVeri;

uint8_t MessageBuffer[14];
struct ConfigRF rfayarlari;
long sure;

void converter(GPSVeri kayityeri);

void setup(){
    SeriPort.begin(UartBaudRate);
    DEBUG_PRINTLN(F("Seri Port Baslatildi..."));
    GPSPort.begin(GPSBaudRate);
    DEBUG_PRINTLN(F("GPS Port Baslatildi..."));
    RFBegin(&rfayarlari, RFHighAdresi, RFLowAdresi, RFKanal, UARTPARITY_8N1, UARTBAUDRATE_9600,
            AIRDATARATE_03k, FIXEDMODE, IO_PUSHPULL, WIRELESSWAKEUP_250, FEC_ON, TRANSMISSIONPOWER_30);
}

void loop(){
    getGPSData(&AnaVeri.enlem, &AnaVeri.boylam, &AnaVeri.irtifa);

    MessageBuffer[0] = 'a';
    converter(AnaVeri);

    sendFixedDataPacket(RFGondericiHighAdresi, RFGondericiLowAdresi, RFGondericiKanal, MessageBuffer, 14);

    delay(BeklemeSuresi);
}

void converter(GPSVeri kayityeri){
    Float2ByteConverter convert;
    
    convert.floatvar = kayityeri.enlem;
    for(int i = 0; i < 4; i++){
        MessageBuffer[i+1] = convert.bytevar[i];
    }

    convert.floatvar = kayityeri.boylam;
    for(int i = 0; i < 4; i++){
        MessageBuffer[i+5] = convert.bytevar[i];
    }

    convert.floatvar = kayityeri.irtifa;
    for(int i = 0; i < 4; i++){
        MessageBuffer[i+9] = convert.bytevar[i];
    }
}