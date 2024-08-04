#include <Arduino.h>

#define DEBUG_MODE

#undef Serial
#define Serial SeriPort

#include "debugprinter.h"
#include <HardwareSerial.h>
#include "main.h"
#include "rf.h"
#include "BNO055.h"

#define UartRXPini PA3
#define UartTXPini PA2
#define UartBaudRate 115200

#define RFUartRXPini PA13
#define RFUartTXPini PA12
#define RFBaudRate 9600

#define RFLowAdresi 0x01
#define RFHighAdresi 0x03
#define RFKanal 23U

#define BeklemeSuresi 20

HardwareSerial SerialRF(RFUartRXPini, RFUartTXPini);
HardwareSerial SeriPort(UartRXPini, UartTXPini);

typedef union{
    float floatvar;
    uint8_t bytevar[4];
}Float2ByteConverter;

BNO_DOF3_Float ivmeData, gyroData;
float pitch;

float irtifaData;

float gpsEnlem, gpsBoylam, gpsIrtifa;

struct ConfigRF rfayarlari;

uint8_t messageBuffer[45];

void setup(){
    SeriPort.begin(UartBaudRate);
    DEBUG_PRINTLN(F("Seri Port Baslatildi..."));

    RFBegin(&rfayarlari, RFHighAdresi, RFLowAdresi, RFKanal, UARTPARITY_8N1, UARTBAUDRATE_9600,
            AIRDATARATE_03k, FIXEDMODE, IO_PUSHPULL, WIRELESSWAKEUP_250, FEC_ON, TRANSMISSIONPOWER_30);
}

void loop(){
    if(receiveDataPacket(messageBuffer, sizeof(messageBuffer)) == E32_Success){
        if(calculateCRC8(messageBuffer, 44) == messageBuffer[44]){
            converter();
            DEBUG_PRINTLN(F("---------------------------------------------------------------"));
            DEBUG_PRINT(F("Ivme -- X: "));DEBUG_PRINT(ivmeData.x);
            DEBUG_PRINT(F(" Y: "));DEBUG_PRINT(ivmeData.y);
            DEBUG_PRINT(F(" Z: "));DEBUG_PRINTLN(ivmeData.z);
            DEBUG_PRINT(F("Gyro -- X: "));DEBUG_PRINT(gyroData.x);
            DEBUG_PRINT(F(" Y: "));DEBUG_PRINT(gyroData.y);
            DEBUG_PRINT(F(" Z: "));DEBUG_PRINTLN(gyroData.z);
            DEBUG_PRINT(F("GPS -- Enlem: "));DEBUG_PRINT(gpsEnlem);
            DEBUG_PRINT(F(" Boylam: "));DEBUG_PRINT(gpsBoylam);
            DEBUG_PRINT(F(" Irtifa: "));DEBUG_PRINTLN(gpsIrtifa);
            DEBUG_PRINT(F("BMP -- Irtifa: "));DEBUG_PRINTLN(irtifaData);
            DEBUG_PRINT(F("AHRS -- Pitch: "));DEBUG_PRINTLN(pitch);
            DEBUG_PRINTLN(F("---------------------------------------------------------------"));
            DEBUG_PRINTLN();
            DEBUG_PRINTLN();
        }
        else{
            DEBUG_PRINTLN(F("CRC Uymadi..."));
            DEBUG_PRINTLN();
            DEBUG_PRINTLN();
        }
    }

    delay(BeklemeSuresi);
}

void converter(){
    Float2ByteConverter convert;

    for(int i = 0; i < 4; i++)convert.bytevar[i] = messageBuffer[i];
    ivmeData.x = convert.floatvar;

    for(int i = 0; i < 4; i++)convert.bytevar[i] = messageBuffer[i+4];
    ivmeData.y = convert.floatvar;

    for(int i = 0; i < 4; i++)convert.bytevar[i] = messageBuffer[i+8];
    ivmeData.z = convert.floatvar;

    for(int i = 0; i < 4; i++)convert.bytevar[i] = messageBuffer[i+12];
    gyroData.x = convert.floatvar;

    for(int i = 0; i < 4; i++)convert.bytevar[i] = messageBuffer[i+16];
    gyroData.y = convert.floatvar;

    for(int i = 0; i < 4; i++)convert.bytevar[i] = messageBuffer[i+20];
    gyroData.z = convert.floatvar;

    for(int i = 0; i < 4; i++)convert.bytevar[i] = messageBuffer[i+24];
    gpsEnlem = convert.floatvar;

    for(int i = 0; i < 4; i++)convert.bytevar[i] = messageBuffer[i+28];
    gpsBoylam = convert.floatvar;

    for(int i = 0; i < 4; i++)convert.bytevar[i] = messageBuffer[i+32];
    gpsIrtifa = convert.floatvar;

    for(int i = 0; i < 4; i++)convert.bytevar[i] = messageBuffer[i+36];
    irtifaData = convert.floatvar;

    for(int i = 0; i < 4; i++)convert.bytevar[i] = messageBuffer[i+40];
    pitch = convert.floatvar;
}