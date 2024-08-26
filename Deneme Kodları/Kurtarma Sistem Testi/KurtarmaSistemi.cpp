/*
*************************************************************************************************************************

Kurtarma Sistem Testi - Kurtarma Sistemi Kod Bloğu

KURTARMA SİSTEM TESTİ NEDİR?
----------------------------
    Kurtarma sistem testi roketin sıkı geçirilmiş olan gövde bölümlerinin başarı ile ayrılabildiğini kanıtlayan testtir.
Gövde bölümlerinin ayrılması roketten rokete değişmektedir. BeeRocketry takımının roketi  Sıcak Gaz  Üretici  ile  barut
kullanarak basınç oluşturur ve sıkı geçen gövdelerin ayrılmasını sağlar. Roketimiz temelde 3 adet gövdeden oluşmaktadır.
Bunlar sırasıyla motor blok, aviyonik blok ve burun konisidir. 2 adet gövde ayıracak sıcak gaz üreticimiz bulunmaktadır.
Bunlardan ilki tepe noktasında tetiklenecek ve burun konisi ile aviyonik bloğunu ayıracak sıcak gaz üreticisiyken diğeri
motor blok ve aviyonik bloğunu ayıracak sıcak gaz üreticisidir.

ANA AVİYONİK GENEL ÖZET
-----------------------
    Testte bu sistemlerimizin çalışırlığını kanıtlamak üzere 2 adet sistem çalıştırılacaktır. Bunlardan biri tetiklemeyi
yapacak olan ana aviyonik sistem diğeri ise ana aviyonik sisteme tetikleme emrini RF üzerinden iletecek yer istasyonudur
Ana aviyonik sistem düzenli olarak RF üzerinden gelecek olan veri paketlerini CRC-8 ile  kontrol  edecek  ve  sonrasında
Pakette bulunan tanımlayıcı karakterleri kontrol edecektir. Eğer gelen karakter 'a' ise  main  paraşütü,  eğer  'b'  ise
apogee paraşütü tetikleyecektir. Yer istasyonu ise seri porttan sürekli olarak gelecek karakterleri  kontrol  edecek  ve
istenen harfte veri gelince ana aviyonik sistemin sahip olduğu adres ve frekansa veri paketini yollayacaktır.

KULLANILAN MODÜLLER
-------------------
RF Modülü         -- EByte E32-433T30D
Mikro Denetleyici -- STM32 Blackpill (STM32F411RE)

RF MODÜLÜ ÇİP AYARLARI
----------------------
İletişim Frekansı           --  433 MHz
Cihaz Adresi (High / Low)   --  0x03 0x07
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
#include "gps.h"
#include "main.h"
#include "rf.h"
#include "BNO055.h"
#include "I2C.h"
#include "bmp388.h"
#include <SPI.h>
//#include "MMC5603.h"
#include <ReefwingAHRS.h>

#define UartRXPini PA10
#define UartTXPini PA9
#define UartBaudRate 115200

#define RFUartRXPini PC7
#define RFUartTXPini PC6

#define RFLowAdresi 0x07
#define RFHighAdresi 0x03
#define RFKanal 23U

#define RFGondericiLowAdresi 0x04
#define RFGondericiHighAdresi 0x02
#define RFGondericiKanal 23U

HardwareSerial SeriPort(UartRXPini, UartTXPini);
HardwareSerial SerialRF(RFUartRXPini, RFUartTXPini);

ConfigRF rfayarlari;

uint8_t messagebuffer[3];
uint8_t eleman;

#define MainParasutPini PB8
#define ApogeeParasutPini PB9

void setup(){
    SeriPort.begin(UartBaudRate);
    DEBUG_PRINTLN(F("Seri Port Baslatildi..."));

    pinMode(MainParasutPini, OUTPUT);
    pinMode(ApogeeParasutPini, OUTPUT);
    digitalWrite(MainParasutPini, LOW);
    digitalWrite(ApogeeParasutPini, LOW);

    RFBegin(&rfayarlari, RFHighAdresi, RFLowAdresi, RFKanal, UARTPARITY_8N1, UARTBAUDRATE_115200,
            AIRDATARATE_48k, FIXEDMODE, IO_PUSHPULL, WIRELESSWAKEUP_250, FEC_ON, TRANSMISSIONPOWER_30);

    delay(1000);
    messagebuffer[0] = 'z';
    messagebuffer[1] = 'e';
    messagebuffer[2] = calculateCRC8(messagebuffer, 2);
    sendFixedDataPacket(RFGondericiHighAdresi, RFGondericiLowAdresi, RFGondericiKanal, messagebuffer, sizeof(messagebuffer));
    delay(500);
}

void loop(){
    if(receiveDataPacket(messagebuffer, sizeof(messagebuffer)) == E32_Success){
        if(calculateCRC8(messagebuffer, 2) == messagebuffer[2]){
            if(messagebuffer[1] == 'a'){
                DEBUG_PRINTLN(F("Main icin Gerekli Karakter Alindi Patlatma Moduna Geciliyor..."));
                for(int i = 5; i > 0; i--){
                    DEBUG_PRINT(i);DEBUG_PRINTLN(F(" Saniye Sonra Patlatilacak..."));
                    delay(1000);
                }
                digitalWrite(MainParasutPini, HIGH);
                DEBUG_PRINTLN(F("Main Yandi"));
                delay(500);
                digitalWrite(MainParasutPini, LOW);
                DEBUG_PRINTLN(F("Main Sondu"));
                messagebuffer[1] == 'z';
            }
            else if(messagebuffer[1] == 'b'){
                DEBUG_PRINTLN(F("Apogee icin Gerekli Karakter Alindi Patlatma Moduna Geciliyor..."));
                for(int i = 5; i > 0; i--){
                    DEBUG_PRINT(i);DEBUG_PRINTLN(F(" Saniye Sonra Patlatilacak..."));
                    delay(1000);
                }
                digitalWrite(ApogeeParasutPini, HIGH);
                DEBUG_PRINTLN(F("Apogee Yandi"));
                delay(500);
                digitalWrite(ApogeeParasutPini, LOW);
                DEBUG_PRINTLN(F("Apogee Sondu"));
                messagebuffer[1] == 'z';
            }
        }
        else{
            DEBUG_PRINTLN(F("CRC Uyusmadi..."));
        }
    }

    delay(50);
}