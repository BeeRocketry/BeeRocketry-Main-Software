/*
*************************************************************************************************************************

Fonksiyonellik Testi - Alıcı Kod Bloğu

FONKSİYONELLİK TESTİ NEDİR?
---------------------------
    Fonksiyonellik testi ana aviyonik sistemin üzerinde bulundurduğu tüm modüllerin çalışırlığının kanıtlanmasını isteyen
testtir. Bu testi gerçekleştirmek üzere 2 adet devremiz bulunmaktadır. Bunlardan biri  tüm  sensör  ve  modül  testlerini
gerçekleştirecek olan ana aviyonik iken diğeri RF modülünü kontrol etmek üzere alıcı olarak görev alacak devredir.

ALICI GENEL ÖZET
----------------
    Alıcı devre kendi sistem kartımızı kullanmaktadır. Sürekli olarak verici devreden veri paketi bekler ve  veri  paketi
geldiğinde CRC-8 ile veri paketini kontrol eder. Eğer paket doğru ise verileri seri porta yazdırır.

KULLANILAN MODÜLLER
-------------------
RF Modülü         -- EByte E32-433T30D
Mikro Denetleyici -- STM32 Blackpill (STM32F411RE)

RF MODÜLÜ ÇİP AYARLARI
----------------------
İletişim Frekansı           --  433 MHz
Cihaz Adresi (High / Low)   --  0x05 0x04
UART BaudRate               --  115200 Bps
UART Parity                 --  8 Bit None Parity 1 Stop Bit
Data Aktarım Hızı           --  4.8k Bps
İletişim Modu               --  Fixed
Aktarım Gücü                --  30 Dbm
FEC                         --  Aktif


*************************************************************************************************************************
*/

#include <Arduino.h>

#define DEBUG_MODE

#undef Serial
#define Serial SeriPort

#include "debugprinter.h"
#include <HardwareSerial.h>
#include "rf.h"
#include "BNO055.h"
#include <SPI.h>

#define UartRXPini PA10
#define UartTXPini PA9
#define UartBaudRate 115200

#define RFUartRXPini PC7
#define RFUartTXPini PC6
#define RFBaudRate 9600

#define RFLowAdresi 0x04
#define RFHighAdresi 0x05
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
void converter();

void setup(){
    SeriPort.begin(UartBaudRate);
    DEBUG_PRINTLN(F("Seri Port Baslatildi..."));

    RFBegin(&rfayarlari, RFHighAdresi, RFLowAdresi, RFKanal, UARTPARITY_8N1, UARTBAUDRATE_115200,
            AIRDATARATE_48k, FIXEDMODE, IO_PUSHPULL, WIRELESSWAKEUP_250, FEC_ON, TRANSMISSIONPOWER_30);
}

void loop(){
    if(receiveDataPacket(messageBuffer, sizeof(messageBuffer)) == E32_Success){
        if(calculateCRC8(messageBuffer, 44) == messageBuffer[44]){
            if(messageBuffer[0] == 'z' && messageBuffer[1] == 'e'){
                DEBUG_PRINTLN(F("Ana Aviyonikten Baslatilma Sinyali Geldi..."));
                DEBUG_PRINTLN(F("Ana Aviyonik Basari Ile Baslatildi..."));
                DEBUG_PRINTLN();
                DEBUG_PRINTLN();
                delay(4000);
            }
            else{
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