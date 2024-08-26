/*
*************************************************************************************************************************

Haberleşme Testi - Yer İstasyon Kod Bloğu

HABERLEŞME TESTİ NEDİR?
-----------------------
Yer   İstasyonları,   görev   yükü   ve   ana   aviyonik   sistem   arasındaki   haberleşmenin  en az 5 kilometre öteden
gerçekleşebildiğinin kanıtlanması gerektiren testtir. Test içeriği olarak  haberleşmede  RF  modülü  kullanılacaktır  ve
gönderilen veri paketinde GPS verilerinin bulunması zorunludur.


YER İSTASYONU GENEL ÖZET
------------------------
    Yer İstasyonu, haberleşme testini gerçekleştirmek üzere  setup()  fonksiyonu  ile  ilk  olarak  GPS  ve RF UART port
ayarlamalarını yapacak olup sonrasında RF modülünün çip ayarlamalarını yapacaktır.
    loop() fonksiyonunda sürekli kendi GPS verisini güncelleyecektir. Ayrıca RF UART portu üzerinden herhangi bir   veri
gelmesi üzerine gelen veri paketine CRC kontrolü uygulayacaktır. Eğer gelen mesaj eksiksiz ve hatasız ise veri paketinin
hangi göndericiden geldiğini tespit edecek ve göndericinin sahip olduğu GPS değişkenlerini güncelleyecektir. Ayrıca  her
4 saniyede bir olmak üzere sistemin sahip olduğu değişkenleri yazdıracaktır. Yazdırma işlemini  gerçekleştirirken    yer
istasyonu ve diğer birimler arasındaki uzaklığı yazdırmak üzere Haversin metodu ile uzaklıklar hesaplanmaktadır.


KULLANILAN MODÜLLER
-------------------
RF Modülü         -- EByte E32-433T30D
GPS Modülü        -- U-Blok NEO-6M-V2
Mikro Denetleyici -- STM32 Blackpill (STM32F411CE)

RF MODÜLÜ ÇİP AYARLARI
----------------------
İletişim Frekansı           --  433 MHz
Cihaz Adresi (High / Low)   --  0x03 0x02
UART BaudRate               --  115200 Bps
UART Parity                 --  8 Bit None Parity 1 Stop Bit
Data Aktarım Hızı           --  0.3k Bps
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
#include "gps.h"
#include "rf.h"

#define UartRXPini PA10
#define UartTXPini PA9
#define UartBaudRate 115200

#define GPSUartRXPini PA3
#define GPSUartTXPini PA2
#define GPSBaudRate 9600

#define RFUartRXPini PC7
#define RFUartTXPini PC6

#define RFLowAdresi 0x02
#define RFHighAdresi 0x03
#define RFKanal 23U

#define SeriPortBeklemeSuresi 4000
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

int16_t uyduSayisi = 0;

void converter(GPSVeri *kayityeri);
void mesafeHesapla(GPSVeri ilkVeri, GPSVeri ikinciVeri, float *kusucumu, float *gercek);

void setup(){
    SeriPort.begin(UartBaudRate);
    DEBUG_PRINTLN(F("Seri Port Baslatildi..."));
    GPSPort.begin(GPSBaudRate);
    DEBUG_PRINTLN(F("GPS Port Baslatildi..."));
    RFBegin(&rfayarlari, RFHighAdresi, RFLowAdresi, RFKanal, UARTPARITY_8N1, UARTBAUDRATE_115200,
            AIRDATARATE_03k, FIXEDMODE, IO_PUSHPULL, WIRELESSWAKEUP_250, FEC_ON, TRANSMISSIONPOWER_30);
    sure = millis();
}

void loop(){
    getGPSData(&YerVeri.enlem, &YerVeri.boylam, &YerVeri.irtifa, &uyduSayisi);

    if(receiveDataPacket(MessageBuffer, sizeof(MessageBuffer)) == E32_Success){
        if(calculateCRC8(MessageBuffer, 13) == MessageBuffer[13]){
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
        DEBUG_PRINTLN(F("------------------------------------------------------------------------------------------"));

        if(AnaCheck == true){
            AnaCheck = false;
            DEBUG_PRINTLN(F("Ana Veri Güncellendi..."));
            DEBUG_PRINTLN();
        }
        if(GorevCheck == true){
            GorevCheck = false;
            DEBUG_PRINTLN(F("Gorev Yuku Veri Güncellendi..."));
            DEBUG_PRINTLN();
        }

        mesafeHesapla(YerVeri, GorevYukuVeri, &GorevKusMesafe, &GorevGercekMesafe);
        mesafeHesapla(YerVeri, AnaVeri, &AnaKusMesafe, &AnaGercekMesafe);

        DEBUG_PRINT(F("Yer Istasyonu  --  Enlem: "));DEBUG_PRINT(YerVeri.enlem);
        DEBUG_PRINT(F(" Boylam: "));DEBUG_PRINT(YerVeri.boylam);
        DEBUG_PRINT(F(" Irtifa: "));DEBUG_PRINTLN(YerVeri.irtifa);DEBUG_PRINTLN();

        DEBUG_PRINT(F("Ana Kart  --  Enlem: "));DEBUG_PRINT(AnaVeri.enlem);
        DEBUG_PRINT(F(" Boylam: "));DEBUG_PRINT(AnaVeri.boylam);
        DEBUG_PRINT(F(" Irtifa: "));DEBUG_PRINT(AnaVeri.irtifa);
        DEBUG_PRINT(F(" Kus Ucumu Mesafe: "));DEBUG_PRINT(AnaKusMesafe);DEBUG_PRINT(F(" m"));
        DEBUG_PRINT(F(" Gercek Mesafe: "));DEBUG_PRINT(AnaGercekMesafe);DEBUG_PRINTLN(F(" m"));DEBUG_PRINTLN();

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