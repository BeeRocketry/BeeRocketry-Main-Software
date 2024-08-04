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

#define RFLowAdresi 0x01
#define RFHighAdresi 0x03
#define RFKanal 23U

#define SeriPortBeklemeSuresi 2000
#define BeklemeSuresi 200

#define DunyaYariCapi 6372795.0

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

GPSVeri GorevYukuVeri;
GPSVeri AnaVeri;
GPSVeri YerVeri;

uint8_t MessageBuffer[14];
float GorevKusMesafe = 0;
float AnaKusMesafe = 0;
float GorevGercekMesafe = 0;
float AnaGercekMesafe = 0;
struct ConfigRF rfayarlari;
long sure;
bool GorevCheck = false;
bool AnaCheck = false;

void converter(GPSVeri *kayityeri);
void mesafeHesapla(GPSVeri ilkVeri, GPSVeri ikinciVeri, float *kusucumu, float *gercek);

void setup(){
    SeriPort.begin(UartBaudRate);
    DEBUG_PRINTLN(F("Seri Port Baslatildi..."));
    GPSPort.begin(GPSBaudRate);
    DEBUG_PRINTLN(F("GPS Port Baslatildi..."));
    RFBegin(&rfayarlari, RFHighAdresi, RFLowAdresi, RFKanal, UARTPARITY_8N1, UARTBAUDRATE_9600,
            AIRDATARATE_03k, FIXEDMODE, IO_PUSHPULL, WIRELESSWAKEUP_250, FEC_ON, TRANSMISSIONPOWER_30);
    sure = millis();
}

void loop(){
    if(receiveDataPacket(MessageBuffer, 14) == E32_Success){
        if(calculateCRC8(MessageBuffer, 14) == MessageBuffer[13]){
            if(MessageBuffer[0] == 'a'){
                converter(&AnaVeri);
                AnaCheck = true;
            }
            else if(MessageBuffer[0] == 'b'){
                converter(&GorevYukuVeri);
                GorevCheck = true;
            }
        }
        else{
            DEBUG_PRINTLN(F("CRC Uymadi..."));
        }
    }

    if(millis() > sure + SeriPortBeklemeSuresi){
        if(AnaCheck == true){
            AnaCheck = false;
            DEBUG_PRINTLN(F("Ana Veri Güncellendi..."));
        }
        if(GorevCheck == true){
            GorevCheck = false;
            DEBUG_PRINTLN(F("Gorev Yuku Veri Güncellendi..."));
        }

        getGPSData(&YerVeri.enlem, &YerVeri.boylam, &YerVeri.irtifa);

        mesafeHesapla(YerVeri, GorevYukuVeri, &GorevKusMesafe, &GorevGercekMesafe);
        mesafeHesapla(YerVeri, AnaVeri, &AnaKusMesafe, &AnaGercekMesafe);

        DEBUG_PRINTLN(F("------------------------------------------------------------------------------------------"));
        DEBUG_PRINT(F("Yer Istasyonu  --  Enlem: "));DEBUG_PRINT(YerVeri.enlem);
        DEBUG_PRINT(F(" Boylam: "));DEBUG_PRINT(YerVeri.boylam);
        DEBUG_PRINT(F(" Irtifa: "));DEBUG_PRINT(YerVeri.irtifa);

        DEBUG_PRINT(F("Ana Kart  --  Enlem: "));DEBUG_PRINT(AnaVeri.enlem);
        DEBUG_PRINT(F(" Boylam: "));DEBUG_PRINT(AnaVeri.boylam);
        DEBUG_PRINT(F(" Irtifa: "));DEBUG_PRINT(AnaVeri.irtifa);
        DEBUG_PRINT(F(" Kus Ucumu Mesafe: "));DEBUG_PRINT(AnaKusMesafe);DEBUG_PRINT(F(" m"));
        DEBUG_PRINT(F(" Gercek Mesafe: "));DEBUG_PRINT(AnaGercekMesafe);DEBUG_PRINTLN(F(" m"));

        DEBUG_PRINT(F("Gorev Yuku  --  Enlem: "));DEBUG_PRINT(GorevYukuVeri.enlem);
        DEBUG_PRINT(F(" Boylam: "));DEBUG_PRINT(GorevYukuVeri.boylam);
        DEBUG_PRINT(F(" Irtifa: "));DEBUG_PRINT(GorevYukuVeri.irtifa);
        DEBUG_PRINT(F(" Kus Ucumu Mesafe: "));DEBUG_PRINT(GorevKusMesafe);DEBUG_PRINT(F(" m"));
        DEBUG_PRINT(F(" Gercek Mesafe: "));DEBUG_PRINT(GorevGercekMesafe);DEBUG_PRINTLN(F(" m"));
        DEBUG_PRINTLN(F("------------------------------------------------------------------------------------------"));

        DEBUG_PRINTLN();
        DEBUG_PRINTLN();

        sure = millis();
    }

    delay(BeklemeSuresi);
}

void converter(GPSVeri *kayityeri){
    Float2ByteConverter convert;
    for(int i = 0; i < 4; i++){
        convert.bytevar[i] = MessageBuffer[i+1];
    }
    kayityeri->enlem = convert.floatvar;

    for(int i = 0; i < 4; i++){
        convert.bytevar[i] = MessageBuffer[i+5];
    }
    kayityeri->boylam = convert.floatvar;

    for(int i = 0; i < 4; i++){
        convert.bytevar[i] = MessageBuffer[i+9];
    }
    kayityeri->irtifa = convert.floatvar;
}

void mesafeHesapla(GPSVeri ilkVeri, GPSVeri ikinciVeri, float *kusucumu, float *gercek){
    float enlemFarki = 0;
    float boylamFarki = 0;
    float irtifaFarki = 0;
    double a;
    double c;

    enlemFarki = (ilkVeri.enlem * DEG_TO_RAD) - (ikinciVeri.enlem * DEG_TO_RAD);
    boylamFarki = (ilkVeri.boylam * DEG_TO_RAD) - (ikinciVeri.boylam * DEG_TO_RAD);

    a = pow(sin(enlemFarki / 2), 2) + cos(ilkVeri.enlem * DEG_TO_RAD) * cos(ikinciVeri.enlem * DEG_TO_RAD) * pow(sin(boylamFarki / 2), 2);
    c = 2 * atan2(sqrt(a), sqrt(1 - a));

    *kusucumu = DunyaYariCapi * c;

    irtifaFarki = ilkVeri.irtifa - ikinciVeri.irtifa;

    *gercek = sqrt(pow(*kusucumu, 2) + pow(irtifaFarki, 2));
}