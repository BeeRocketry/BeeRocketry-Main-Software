/*
*************************************************************************************************************************

Haberleşme Testi - Görev Yükü Kod Bloğu

HABERLEŞME TESTİ NEDİR?
-----------------------
Yer   İstasyonları,   görev   yükü   ve   ana   aviyonik   sistem   arasındaki   haberleşmenin  en az 5 kilometre öteden
gerçekleşebildiğinin kanıtlanması gerektiren testtir. Test içeriği olarak  haberleşmede  RF  modülü  kullanılacaktır  ve
gönderilen veri paketinde GPS verilerinin bulunması zorunludur.


YER İSTASYONU GENEL ÖZET
------------------------
    Görev Yükü,  haberleşme  testini  gerçekleştirmek üzere  setup()  fonksiyonu  ile  ilk  olarak  GPS  ve RF UART port
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
Cihaz Adresi (High / Low)   --  0x01 0x06
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
#include "main.h"
#include "rf.h"

#define UartRXPini PA10
#define UartTXPini PA9
#define UartBaudRate 115200

#define GPSUartRXPini PA3
#define GPSUartTXPini PA2
#define GPSBaudRate 9600

#define RFUartRXPini PC7
#define RFUartTXPini PC6

#define RFLowAdresi 0x06
#define RFHighAdresi 0x01
#define RFKanal 23U

#define RFGondericiLowAdresi 0x02
#define RFGondericiHighAdresi 0x03
#define RFGondericiKanal 23U

#define BeklemeSuresi 1000

HardwareSerial SeriPort(UartRXPini, UartTXPini);
HardwareSerial GPSPort(GPSUartRXPini, GPSUartTXPini);
HardwareSerial SerialRF(RFUartRXPini, RFUartTXPini);

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
int16_t uyduSayisi = 0;

void converter(GPSVeri kayityeri);
void printArayuz(void);

long sureRF;

void setup(){
    SeriPort.begin(UartBaudRate);
    DEBUG_PRINTLN(F("Seri Port Baslatildi..."));
    GPSPort.begin(GPSBaudRate);
    DEBUG_PRINTLN(F("GPS Port Baslatildi..."));
    RFBegin(&rfayarlari, RFHighAdresi, RFLowAdresi, RFKanal, UARTPARITY_8N1, UARTBAUDRATE_115200,
            AIRDATARATE_03k, FIXEDMODE, IO_PUSHPULL, WIRELESSWAKEUP_250, FEC_ON, TRANSMISSIONPOWER_30);
    sureRF = millis();
}

void loop(){
    getGPSData(&AnaVeri.enlem, &AnaVeri.boylam, &AnaVeri.irtifa, &uyduSayisi);

    printArayuz();

    if(millis() > sureRF + 3000){
        converter(AnaVeri);
        sendFixedDataPacket(RFGondericiHighAdresi, RFGondericiLowAdresi, RFGondericiKanal, MessageBuffer, sizeof(MessageBuffer));

        DEBUG_PRINTLN(F("RF verisi gonderildi..."));
        DEBUG_PRINTLN();
        sureRF = millis();
    }

    delay(BeklemeSuresi);
}

void printArayuz(void){
    DEBUG_PRINTLN(F("------------------------------------------"));
    DEBUG_PRINTLN(F("GPS Verisi Alindi..."));
    DEBUG_PRINT(F("ENLEM: "));DEBUG_PRINT(AnaVeri.enlem);DEBUG_PRINT(F("    "));
    DEBUG_PRINT(F("BOYLAM: "));DEBUG_PRINT(AnaVeri.boylam);DEBUG_PRINT(F("    "));
    DEBUG_PRINT(F("IRTIFA: "));DEBUG_PRINT(AnaVeri.irtifa);DEBUG_PRINT(F("    "));
    DEBUG_PRINT(F("UYDU SAYISI: "));DEBUG_PRINTLN(uyduSayisi);
    DEBUG_PRINTLN(F("------------------------------------------"));
    DEBUG_PRINTLN();
}

void converter(GPSVeri kayityeri){
    Float2ByteConverter convert;

    MessageBuffer[0] = 'b';
    
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

    MessageBuffer[13] = calculateCRC8(MessageBuffer, 13);
}