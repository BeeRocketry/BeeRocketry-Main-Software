/*
*************************************************************************************************************************

Arayüz Testi - Yer İstasyonu Kod Bloğu

ARAYÜZ TESTİ NEDİR?
-------------------
    Arayüz testi Teknofest Roket yarışması için gereken yer istasyonu arayüzünün çalışırlığını kanıtlayacak olan testtir.

UYARI : Bu repoda bulunan arayüz test kod blokları arayüzün kod blokları değildir. Arayüzün  reposuna  aşağıdaki  linkten
ulaşabilirsin. Bu repoda bulunan kodlar arayüzün test edilmesi için veri  üretecek  ve  gönderecek  devrelerin  kodlarını
içermektedir.

Arayüz Reposu : https://github.com/btnrv/BeeRocketry_RocketGroundStation


YER İSTASYONU GENEL ÖZET
------------------------
    Yer istasyonu arayüz testini gerçekleştirmek üzere sürekli olarak RF modülüne gönderici devre tarafından gelecek veri
paketlerini CRC-8 algoritması ile kontrol eder. Paket sayacı gibi ek veriler ekleyerek arayüz yazılımının kontrol ettiği
formatta stringe dönüştürerek seri port ile arayüz yazılımını çalıştıran bilgisayara gönderir.


KULLANILAN MODÜLLER
-------------------
RF Modülü         -- EByte E32-433T30D
Mikro Denetleyici -- STM32 Blackpill (STM32F411CE)


RF MODÜLÜ ÇİP AYARLARI
----------------------
İletişim Frekansı           --  433 MHz
Cihaz Adresi (High / Low)   --  0x03 0x01
UART BaudRate               --  115200 Bps
UART Parity                 --  8 Bit None Parity 1 Stop Bit
Data Aktarım Hızı           --  0.3k Bps
İletişim Modu               --  Fixed
Aktarım Gücü                --  30 Dbm
FEC                         --  Aktif


ARAYÜZE GÖNDERİLEN VERİ PAKETİ
------------------------------

    1          2          3          4          5          6          7          8          9         10         11
  İrtifa   GPSİrtifa   GPSEnlem   GPSBoylam  GPSİrtifa  GPSEnlem  GPSBoylam   Gyro-X     Gyro-Y     Gyro-Z     İvme-X
             (Ana)      (Ana)      (Ana)     (Görev)    (Görev)    (Görev)

   12         13         14         15         16         17         18         19         20         21         22
  İvme-Y    İvme-Z      Açı        Yaw        Roll       Pitch       Nem     Sıcaklık     Durum      Sayaç   Total İvme


*************************************************************************************************************************
*/

#include <Arduino.h>

#define DEBUG_MODE

#undef Serial
#define Serial SeriPort

#include "debugprinter.h"
#include <HardwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include "rf.h"

#define UartRXPini PA3
#define UartTXPini PA2
#define UartBaudRate 115200

#define RFUartRXPini PA10
#define RFUartTxPini PA9

#define RFLowAdresi 0x01
#define RFHighAdresi 0x03
#define RFKanal 23U

#define BeklemeSuresi 60

HardwareSerial SeriPort(UartRXPini, UartTXPini);
HardwareSerial SerialRF(RFUartRXPini, RFUartTxPini);

void printArayuz();

int8_t paketSayaci = 0;

typedef union{
    float floatvar;
    uint8_t bytevar[4];
}Float2ByteConverter;

typedef struct{
    float enlem = 0;
    float boylam = 0;
    float irtifa = 0;
}GPSVeri;

GPSVeri GorevYukuVeri;
GPSVeri AnaVeri;

uint8_t MessageBuffer[55];

struct ConfigRF rfayarlari;
long sure;
bool data = false;

float pitch = 0, roll = 0, yaw = 0;
float ivmex = 0, ivmey = 0, ivmez = 0;
float gyrox = 0, gyroy = 0, gyroz = 0;
float anaIrtifa = 0, gorevIrtifa = 0;
float nem = 0, basinc = 0, sicaklik = 0;

void converterA();
float totalIvme = 0;
void converterB();

uint8_t durum = 0;

void setup(){
    SeriPort.begin(UartBaudRate);
    DEBUG_PRINTLN(F("Seri Port Baslatildi..."));
    RFBegin(&rfayarlari, RFHighAdresi, RFLowAdresi, RFKanal, UARTPARITY_8N1, UARTBAUDRATE_9600,
            AIRDATARATE_03k, FIXEDMODE, IO_PUSHPULL, WIRELESSWAKEUP_250, FEC_ON, TRANSMISSIONPOWER_30);
    sure = millis();
}

void loop(){
    if(receiveDataPacket(MessageBuffer, sizeof(MessageBuffer)) == E32_Success){
        if(MessageBuffer[0] == 'a'){
            if(calculateCRC8(MessageBuffer, 54) == MessageBuffer[54]){
                converterA();
            }
            else{
                DEBUG_PRINTLN(F("CRC Uyusmadi..."));
            }
        }
        if(MessageBuffer[0] == 'b'){
            if(calculateCRC8(MessageBuffer, 28) == MessageBuffer[28]){
                converterB();
            }
            else{
                DEBUG_PRINTLN(F("CRC Uyusmadi..."));
            }
        }
    }

    sicaklik = 29.6;
    basinc = 90050.3;
    gorevIrtifa = 956.2;
    nem = 44.2;

    if(millis() > sure + 150){
        printArayuz();
        /*DEBUG_PRINT(F("IvmeX: "));DEBUG_PRINT(ivmex);DEBUG_PRINT(F("   "));
        DEBUG_PRINT(F("IvmeY: "));DEBUG_PRINT(ivmey);DEBUG_PRINT(F("   "));
        DEBUG_PRINT(F("IvmeZ: "));DEBUG_PRINTLN(ivmez);

        DEBUG_PRINT(F("GyroX: "));DEBUG_PRINT(gyrox);DEBUG_PRINT(F("   "));
        DEBUG_PRINT(F("GyroY: "));DEBUG_PRINT(gyroy);DEBUG_PRINT(F("   "));
        DEBUG_PRINT(F("GyroZ: "));DEBUG_PRINTLN(gyroz);

        DEBUG_PRINT(F("GPSEnlem: "));DEBUG_PRINT(AnaVeri.enlem);DEBUG_PRINT(F("   "));
        DEBUG_PRINT(F("GPSBoylam: "));DEBUG_PRINT(AnaVeri.boylam);DEBUG_PRINT(F("   "));
        DEBUG_PRINT(F("GPSIrtifa: "));DEBUG_PRINTLN(AnaVeri.irtifa);

        DEBUG_PRINT(F("Pitch: "));DEBUG_PRINT(pitch);DEBUG_PRINT(F("   "));
        DEBUG_PRINT(F("Roll: "));DEBUG_PRINT(roll);DEBUG_PRINT(F("   "));
        DEBUG_PRINT(F("Yaw: "));DEBUG_PRINTLN(yaw);

        DEBUG_PRINT(F("Irtifa: "));DEBUG_PRINT(anaIrtifa);DEBUG_PRINT(F("   "));
        DEBUG_PRINT(F("Basinc: "));DEBUG_PRINT(basinc);DEBUG_PRINT(F("   "));
        DEBUG_PRINT(F("Gorev Irtifa: "));DEBUG_PRINT(gorevIrtifa);DEBUG_PRINT(F("   "));
        DEBUG_PRINT(F("Sicaklik: "));DEBUG_PRINT(sicaklik);DEBUG_PRINT(F("   "));
        DEBUG_PRINT(F("Durum: "));DEBUG_PRINTLN(durum);

        */
        sure =  millis();
    }
}

void converterA(){
    Float2ByteConverter convert;
    for(int i = 0; i < 4; i++)convert.bytevar[i] = MessageBuffer[i+1];
    ivmex = convert.floatvar;

    for(int i = 0; i < 4; i++)convert.bytevar[i] = MessageBuffer[i+5];
    ivmey = convert.floatvar;

    for(int i = 0; i < 4; i++)convert.bytevar[i] = MessageBuffer[i+9];
    ivmez = convert.floatvar;

    for(int i = 0; i < 4; i++)convert.bytevar[i] = MessageBuffer[i+13];
    gyrox = convert.floatvar;

    for(int i = 0; i < 4; i++)convert.bytevar[i] = MessageBuffer[i+17];
    gyroy = convert.floatvar;

    for(int i = 0; i < 4; i++)convert.bytevar[i] = MessageBuffer[i+21];
    gyroz = convert.floatvar;

    for(int i = 0; i < 4; i++)convert.bytevar[i] = MessageBuffer[i+25];
    AnaVeri.enlem = convert.floatvar;

    for(int i = 0; i < 4; i++)convert.bytevar[i] = MessageBuffer[i+29];
    AnaVeri.boylam = convert.floatvar;

    for(int i = 0; i < 4; i++)convert.bytevar[i] = MessageBuffer[i+33];
    AnaVeri.irtifa = convert.floatvar;

    for(int i = 0; i < 4; i++)convert.bytevar[i] = MessageBuffer[i+37];
    anaIrtifa = convert.floatvar;

    for(int i = 0; i < 4; i++)convert.bytevar[i] = MessageBuffer[i+41];
    pitch = convert.floatvar;

    for(int i = 0; i < 4; i++)convert.bytevar[i] = MessageBuffer[i+45];
    roll = convert.floatvar;

    for(int i = 0; i < 4; i++)convert.bytevar[i] = MessageBuffer[i+49];
    yaw = convert.floatvar;

    GorevYukuVeri.enlem = AnaVeri.enlem;
    GorevYukuVeri.boylam = AnaVeri.boylam;
    GorevYukuVeri.irtifa = AnaVeri.irtifa;
    totalIvme = sqrtf(powf(ivmex, 2) + powf(ivmey, 2) + powf(ivmez, 2));

    durum = MessageBuffer[53];
}

void converterB(){
    Float2ByteConverter convert;
    for(int i = 0; i < 4; i++)convert.bytevar[i] = MessageBuffer[i+1];
    sicaklik = convert.floatvar;

    for(int i = 0; i < 4; i++)convert.bytevar[i] = MessageBuffer[i+5];
    gorevIrtifa = convert.floatvar;

    for(int i = 0; i < 4; i++)convert.bytevar[i] = MessageBuffer[i+9];
    nem = convert.floatvar;

    for(int i = 0; i < 4; i++)convert.bytevar[i] = MessageBuffer[i+13];
    basinc = convert.floatvar;

    for(int i = 0; i < 4; i++)convert.bytevar[i] = MessageBuffer[i+17];
    GorevYukuVeri.enlem = convert.floatvar;

    for(int i = 0; i < 4; i++)convert.bytevar[i] = MessageBuffer[i+21];
    GorevYukuVeri.boylam = convert.floatvar;

    for(int i = 0; i < 4; i++)convert.bytevar[i] = MessageBuffer[i+25];
    GorevYukuVeri.irtifa = convert.floatvar;
}

void printArayuz(){
    paketSayaci %= 256;

    DEBUG_PRINT(anaIrtifa);DEBUG_PRINT(F(","));
    DEBUG_PRINT(AnaVeri.irtifa);DEBUG_PRINT(F(","));
    DEBUG_PRINT(AnaVeri.enlem);DEBUG_PRINT(F(","));
    DEBUG_PRINT(AnaVeri.boylam);DEBUG_PRINT(F(","));
    DEBUG_PRINT(GorevYukuVeri.irtifa);DEBUG_PRINT(F(","));
    DEBUG_PRINT(GorevYukuVeri.enlem);DEBUG_PRINT(F(","));
    DEBUG_PRINT(GorevYukuVeri.boylam);DEBUG_PRINT(F(","));
    DEBUG_PRINT(gyrox);DEBUG_PRINT(F(","));
    DEBUG_PRINT(gyroy);DEBUG_PRINT(F(","));
    DEBUG_PRINT(gyroz);DEBUG_PRINT(F(","));
    DEBUG_PRINT(ivmex);DEBUG_PRINT(F(","));
    DEBUG_PRINT(ivmey);DEBUG_PRINT(F(","));
    DEBUG_PRINT(ivmez);DEBUG_PRINT(F(","));
    DEBUG_PRINT(pitch - 90);DEBUG_PRINT(F(","));
    DEBUG_PRINT(yaw);DEBUG_PRINT(F(","));
    DEBUG_PRINT(roll);DEBUG_PRINT(F(","));
    DEBUG_PRINT(pitch);DEBUG_PRINT(F(","));
    DEBUG_PRINT(nem);DEBUG_PRINT(F(","));
    DEBUG_PRINT(sicaklik);DEBUG_PRINT(F(","));
    DEBUG_PRINT(durum);DEBUG_PRINT(F(","));
    DEBUG_PRINT(paketSayaci++);DEBUG_PRINT(F(","));
    DEBUG_PRINTLN(totalIvme);
}