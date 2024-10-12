/*
*************************************************************************************************************************

Arayüz Testi - Gönderici Kod Bloğu

ARAYÜZ TESTİ NEDİR?
-------------------
    Arayüz testi Teknofest Roket yarışması için gereken yer istasyonu arayüzünün çalışırlığını kanıtlayacak olan testtir.

UYARI : Bu repoda bulunan arayüz test kod blokları arayüzün kod blokları değildir. Arayüzün  reposuna  aşağıdaki  linkten
ulaşabilirsin. Bu repoda bulunan kodlar arayüzün test edilmesi için veri  üretecek  ve  gönderecek  devrelerin  kodlarını
içermektedir.

Arayüz Reposu : https://github.com/btnrv/BeeRocketry_RocketGroundStation


GÖNDERİCİ GENEL ÖZET
--------------------
    Gönderici devre arayüz testini gerçekleştirmek üzere başlangıç ayarlarını yapacak ve sürekli olarak veri üretecektir.
Her 1.5 saniyede bir olmak üzere verileri paketleyerek yer istasyonuna gönderecektir.


KULLANILAN MODÜLLER
-------------------
RF Modülü         -- EByte E32-433T30D
Mikro Denetleyici -- STM32 Blackpill (STM32F411CE)


RF MODÜLÜ ÇİP AYARLARI
----------------------
İletişim Frekansı           --  433 MHz
Cihaz Adresi (High / Low)   --  0x03 0x07
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
#include "BNO055.h"
#include "I2C.h"
#include "bmp388.h"
#include <SPI.h>
//#include "MMC5603.h"
#include <ReefwingAHRS.h>

#define UartRXPini PA10
#define UartTXPini PA9
#define UartBaudRate 115200

#define GPSUartRXPini PA3
#define GPSUartTXPini PA2
#define GPSBaudRate 9600

#define RFUartRXPini PC7
#define RFUartTXPini PC6

#define RFLowAdresi 0x07
#define RFHighAdresi 0x03
#define RFKanal 23U

#define I2CSDAPini PB7
#define I2CSCLPini PB6

#define RFGondericiLowAdresi 0x01
#define RFGondericiHighAdresi 0x03
#define RFGondericiKanal 23U

#define BeklemeSuresi 150
#define RFBeklemeSuresi 1500

HardwareSerial SeriPort(UartRXPini, UartTXPini);
HardwareSerial SerialRF(RFUartRXPini, RFUartTXPini);
HardwareSerial GPSSeriPort(GPSUartRXPini, GPSUartTXPini);
ReefwingAHRS ahrs;

typedef union{
    float floatvar;
    uint8_t bytevar[4];
}Float2ByteConverter;

typedef struct{
    float enlem = 0;
    float boylam = 0;
    float irtifa = 0;
}GPSVeri;

void converter();
void printDebug();
void printConverter();
void printArayuz();

GPSVeri AnaVeri;
GPSVeri gorevVeri;

int16_t uyduSayisi = 0;
BNO_DOF3_Float ivmeData, gyroData, magData;
float pitch = 0, roll = 0, yaw = 0;
SensorData imuData;

float nem = 0;
float sicaklik = 0;
float irtifaGorev = 0;
float basinc = 0;

float basincData, sicaklikData, irtifaData, irtifaRampa, gercekIrtifa;

uint8_t MessageBuffer[55] = {0};

struct ConfigRF rfayarlari;
BNO_STR_REGISTERS bnoayarlari;

long sureGPS, sureRF, sureCurrent;

uint8_t durum;
uint8_t paketSayaci = 0;

void setup(){
    SeriPort.begin(UartBaudRate);
    DEBUG_PRINTLN(F("Seri Port Baslatildi..."));

    GPSSeriPort.begin(GPSBaudRate);
    DEBUG_PRINTLN(F("GPS Port Baslatildi..."));

    RFBegin(RFHighAdresi, RFLowAdresi, RFKanal, UARTPARITY_8N1, UARTBAUDRATE_9600,
            AIRDATARATE_03k, FIXEDMODE, IO_PUSHPULL, WIRELESSWAKEUP_250, FEC_ON, TRANSMISSIONPOWER_30);

    I2CBegin(I2CSDAPini, I2CSCLPini);
    DEBUG_PRINTLN(F("I2C Portu Baslatildi..."));

    BNOBegin(bnoayarlari);
    DEBUG_PRINTLN(F("BNO055 Baslatildi..."));

    //MMCBegin(false, 0);

    BMPInit(BMP_OverSampling_16x, BMP_OverSampling_2x, BMP_IIR_OFF, BMP_ODR_40ms);
    DEBUG_PRINTLN(F("BMP388 Baslatildi..."));

    ahrs.begin();
    ahrs.setBoardType(BoardType::NOT_DEFINED);
    ahrs.setImuType(ImuType::UNKNOWN);
    ahrs.setDeclination(6.25);
    ahrs.setDOF(DOF::DOF_9);
    ahrs.setFusionAlgorithm(SensorFusion::KALMAN);
    DEBUG_PRINTLN(F("AHRS Algoritmasi Baslatildi..."));

    /*for(int i = 0; i < 100; i++){
        BMPGetData(&sicaklikData, &basincData, &irtifaData);
        irtifaRampa += irtifaData;
        delay(11);
    }
    irtifaRampa /= 100;
    DEBUG_PRINTLN(F("Rampa Irtifasi Hesaplandi..."));*/

    sureGPS = millis() + 1000;
    sureRF = millis();
}

void loop(){
    durum = random(1,5);
    gorevVeri.enlem = 39 + (random(0, 100) / 100.f);
    gorevVeri.boylam = 32 + (random(0, 100) / 100.f);
    gorevVeri.irtifa = random(500, 600);

    BMPGetData(&sicaklikData, &basincData, &irtifaData);

    delay(5);

    ivmeData = getAccData();
    gyroData = getGyroData();
    magData = getMagData();

    imuData.ax = ivmeData.x / 9.80665;
    imuData.ay = ivmeData.y / 9.80665;
    imuData.az = ivmeData.z / 9.80665;

    imuData.gx = gyroData.x;
    imuData.gy = gyroData.y;
    imuData.gz = gyroData.z;

    imuData.mx = magData.x * 0.01;
    imuData.my = magData.y * 0.01;
    imuData.mz = magData.z * 0.01;

    ahrs.setData(imuData);
    ahrs.update();

    pitch = ahrs.angles.pitch;
    yaw = ahrs.angles.yaw;
    roll = ahrs.angles.roll;

    sureCurrent = millis();

    gercekIrtifa = irtifaData - irtifaRampa;

    if(sureCurrent > sureGPS + 3000){
        getGPSData(&AnaVeri.enlem, &AnaVeri.boylam, &AnaVeri.irtifa, &uyduSayisi);
        sureGPS = millis();
    }

    //printArayuz();

    printDebug();

    if(millis() - sureRF > RFBeklemeSuresi){
        converter();
        printConverter();
        Status ret = sendFixedDataPacket(RFGondericiHighAdresi, RFGondericiLowAdresi, RFGondericiKanal, MessageBuffer, sizeof(MessageBuffer), RFBeklemeSuresi);
        if(ret == E32_Timeout){
            DEBUG_PRINTLN(F("Paket Gönderilemedi..."));
        }
        else if(ret == E32_NoPackageTime){
            exit(E32_NoPackageTime);
        }
        sureRF = millis();
    }

    managedDelay(BeklemeSuresi);
}

void converter(){
    Float2ByteConverter convert;

    // message buffer -- 0 1-4 5-8 9-12 13-16 17-20 21-24 25-28 29-32  33-36   37-40  41-44 45-48 49-52 53    54
    //                   a ax  ay  az   gx    gy    gz    enlem boylam girtifa irtifa pitch roll  yaw   durum crc

    // comm buffer --    0    1   2     3 4-7 8-11 12-15 16-19 20-23 24-27 28-31 32-35  36-39   40-43  44-47 48-51 52-55 56    57
    //                   high low kanal a ax  ay   az    gx    gy    gz    enlem boylam girtifa irtifa pitch roll  yaw   durum crc

    MessageBuffer[0] = 'a';

    convert.floatvar = imuData.ax;
    for(int i = 0; i < 4; i++)MessageBuffer[i+1] = convert.bytevar[i];

    convert.floatvar = imuData.ay;
    for(int i = 0; i < 4; i++)MessageBuffer[i+5] = convert.bytevar[i];
    
    convert.floatvar = imuData.az;
    for(int i = 0; i < 4; i++)MessageBuffer[i+9] = convert.bytevar[i];

    convert.floatvar = gyroData.x;
    for(int i = 0; i < 4; i++)MessageBuffer[i+13] = convert.bytevar[i];

    convert.floatvar = gyroData.y;
    for(int i = 0; i < 4; i++)MessageBuffer[i+17] = convert.bytevar[i];

    convert.floatvar = gyroData.z;
    for(int i = 0; i < 4; i++)MessageBuffer[i+21] = convert.bytevar[i];

    convert.floatvar = AnaVeri.enlem;
    for(int i = 0; i < 4; i++)MessageBuffer[i+25] = convert.bytevar[i];

    convert.floatvar = AnaVeri.boylam;
    for(int i = 0; i < 4; i++)MessageBuffer[i+29] = convert.bytevar[i];

    convert.floatvar = AnaVeri.irtifa;
    for(int i = 0; i < 4; i++)MessageBuffer[i+33] = convert.bytevar[i];

    convert.floatvar = irtifaData;
    for(int i = 0; i < 4; i++)MessageBuffer[i+37] = convert.bytevar[i];

    convert.floatvar = pitch;
    for(int i = 0; i < 4; i++)MessageBuffer[i+41] = convert.bytevar[i];

    convert.floatvar = roll;
    for(int i = 0; i < 4; i++)MessageBuffer[i+45] = convert.bytevar[i];
    
    convert.floatvar = yaw;
    for(int i = 0; i < 4; i++)MessageBuffer[i+49] = convert.bytevar[i];

    MessageBuffer[53] = durum;

    uint8_t crc = calculateCRC8(MessageBuffer, 54);
    MessageBuffer[54] = crc;
}

void printConverter(){
    for(int i = 0; i < sizeof(MessageBuffer); i++){
        DEBUG_PRINT(MessageBuffer[i], DEC);DEBUG_PRINT(F(" "));
    }
    DEBUG_PRINTLN();
    DEBUG_PRINTLN();
}

void printDebug(){
    float totalIvme = sqrtf(powf(imuData.ax, 2) + powf(imuData.ay,2) + powf(imuData.az,2));
    DEBUG_PRINT(F("IvmeX: "));DEBUG_PRINT(imuData.ax);DEBUG_PRINT(F("   "));
    DEBUG_PRINT(F("IvmeY: "));DEBUG_PRINT(imuData.ay);DEBUG_PRINT(F("   "));
    DEBUG_PRINT(F("IvmeZ: "));DEBUG_PRINT(imuData.az);DEBUG_PRINT(F("   "));
    DEBUG_PRINT(F("Total Ivme: "));DEBUG_PRINTLN(totalIvme);

    DEBUG_PRINT(F("GyroX: "));DEBUG_PRINT(gyroData.x);DEBUG_PRINT(F("   "));
    DEBUG_PRINT(F("GyroY: "));DEBUG_PRINT(gyroData.y);DEBUG_PRINT(F("   "));
    DEBUG_PRINT(F("GyroZ: "));DEBUG_PRINTLN(gyroData.z);

    DEBUG_PRINT(F("GPSEnlem: "));DEBUG_PRINT(AnaVeri.enlem);DEBUG_PRINT(F("   "));
    DEBUG_PRINT(F("GPSBoylam: "));DEBUG_PRINT(AnaVeri.boylam);DEBUG_PRINT(F("   "));
    DEBUG_PRINT(F("GPSIrtifa: "));DEBUG_PRINTLN(AnaVeri.irtifa);

    DEBUG_PRINT(F("Pitch: "));DEBUG_PRINT(pitch);DEBUG_PRINT(F("   "));
    DEBUG_PRINT(F("Roll: "));DEBUG_PRINT(roll);DEBUG_PRINT(F("   "));
    DEBUG_PRINT(F("Yaw: "));DEBUG_PRINTLN(yaw);

    DEBUG_PRINT(F("Irtifa: "));DEBUG_PRINT(irtifaData);DEBUG_PRINT(F("   "));
    DEBUG_PRINT(F("Sicaklik: "));DEBUG_PRINT(sicaklikData);DEBUG_PRINT(F("   "));
    DEBUG_PRINT(F("Basinc: "));DEBUG_PRINTLN(basincData);
    DEBUG_PRINTLN();
    DEBUG_PRINTLN();
}

void printArayuz(){
    paketSayaci %= 256;

    DEBUG_PRINT(irtifaData);DEBUG_PRINT(F(","));
    DEBUG_PRINT(AnaVeri.irtifa);DEBUG_PRINT(F(","));
    DEBUG_PRINT(AnaVeri.enlem);DEBUG_PRINT(F(","));
    DEBUG_PRINT(AnaVeri.boylam);DEBUG_PRINT(F(","));
    DEBUG_PRINT(gorevVeri.irtifa);DEBUG_PRINT(F(","));
    DEBUG_PRINT(gorevVeri.enlem);DEBUG_PRINT(F(","));
    DEBUG_PRINT(gorevVeri.boylam);DEBUG_PRINT(F(","));
    DEBUG_PRINT(gyroData.x);DEBUG_PRINT(F(","));
    DEBUG_PRINT(gyroData.y);DEBUG_PRINT(F(","));
    DEBUG_PRINT(gyroData.z);DEBUG_PRINT(F(","));
    DEBUG_PRINT(imuData.ax);DEBUG_PRINT(F(","));
    DEBUG_PRINT(imuData.ay);DEBUG_PRINT(F(","));
    DEBUG_PRINT(imuData.az);DEBUG_PRINT(F(","));
    DEBUG_PRINT(pitch - 90);DEBUG_PRINT(F(","));
    DEBUG_PRINT(yaw);DEBUG_PRINT(F(","));
    DEBUG_PRINT(roll);DEBUG_PRINT(F(","));
    DEBUG_PRINT(pitch);DEBUG_PRINT(F(","));
    DEBUG_PRINT(nem);DEBUG_PRINT(F(","));
    DEBUG_PRINT(sicaklik);DEBUG_PRINT(F(","));
    DEBUG_PRINT(durum);DEBUG_PRINT(F(","));
    DEBUG_PRINT(paketSayaci++);DEBUG_PRINT(F(","));
}