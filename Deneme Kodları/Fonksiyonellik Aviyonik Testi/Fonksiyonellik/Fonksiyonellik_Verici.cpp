/*
*************************************************************************************************************************

Fonksiyonellik Testi - Ana Aviyonik Kod Bloğu

FONKSİYONELLİK TESTİ NEDİR?
---------------------------
    Fonksiyonellik testi ana aviyonik sistemin üzerinde bulundurduğu tüm modüllerin çalışırlığının kanıtlanmasını isteyen
testtir. Bu testi gerçekleştirmek üzere 2 adet devremiz bulunmaktadır. Bunlardan biri  tüm  sensör  ve  modül  testlerini
gerçekleştirecek olan ana aviyonik iken diğeri RF modülünü kontrol etmek üzere alıcı olarak görev alacak devredir.

ANA AVİYONİK GENEL ÖZET
-----------------------
    Ana aviyonik devre kendi sistem kartımızı kullanmaktadır. Temel olarak  tüm  sensör  ve  modül  başlangıç  ayarlarını
yaparak başlatılır ve sonrasında sürekli olarak veri alıp seri porta yazdırır. Aynı zamanda 5  saniyede  bir  GPS  verisi
alırken 3 saniyede bir alıcı devreye RF üzerinden veri göndermektedir.

KULLANILAN MODÜLLER
-------------------
RF Modülü         -- EByte E32-433T30D
GPS Modülü        -- UBlock NEO-6M-V2
IMU Modülü        -- Bosch BNO055
Barometre Modülü  -- Bosch BMP388
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
#include <TinyGPS.h>
#include <ReefwingAHRS.h>
#include "rf.h"
#include <SPI.h>
#include "bmp388.h"
#include "BNO055.h"
#include "gps.h"
#include "MMC5603.h"

#define UartRXPini PA10
#define UartTXPini PA9
#define UartBaudRate 115200

#define GPSUartRXPini PA3
#define GPSUartTXPini PA2
#define GPSBaudRate 9600

#define RFUartRXPini PC7
#define RFUartTXPini PC6
#define RFBaudRate 9600

#define RFLowAdresi 0x07
#define RFHighAdresi 0x03
#define RFKanal 23U

#define I2CSDAPini PB7
#define I2CSCLPini PB6

#define RFGondericiLowAdresi 0x04
#define RFGondericiHighAdresi 0x05
#define RFGondericiKanal 23U

#define BeklemeSuresi 20

HardwareSerial SerialRF(RFUartRXPini, RFUartTXPini);
HardwareSerial GPSSeriPort(GPSUartRXPini, GPSUartTXPini);
HardwareSerial SeriPort(UartRXPini, UartTXPini);
ReefwingAHRS ahrs;

typedef union{
    float floatvar;
    uint8_t bytevar[4];
}Float2ByteConverter;

BNO_DOF3_Float ivmeData, gyroData, magData;
float pitch, roll, yaw;
SensorData imuData;
Quaternion qData;

float basincData, sicaklikData, irtifaData, irtifaRampa;

float gpsEnlem, gpsBoylam, gpsIrtifa;

struct ConfigRF rfayarlari;
BNO_STR_REGISTERS bnoayarlari;
int16_t uyduSayisi = 0;

uint8_t messageBuffer[45];

long sureGPS, sureRF, sureCurrent;

void converter();

void setup(){
    SeriPort.begin(UartBaudRate);
    DEBUG_PRINTLN(F("Seri Port Baslatildi..."));

    GPSSeriPort.begin(GPSBaudRate);
    DEBUG_PRINTLN(F("GPS Uart Portu Baslatildi..."));

    RFBegin(&rfayarlari, RFHighAdresi, RFLowAdresi, RFKanal, UARTPARITY_8N1, UARTBAUDRATE_115200,
            AIRDATARATE_48k, FIXEDMODE, IO_PUSHPULL, WIRELESSWAKEUP_250, FEC_ON, TRANSMISSIONPOWER_30);
    
    I2CBegin(I2CSDAPini, I2CSCLPini);
    DEBUG_PRINTLN(F("I2C Portu Baslatildi..."));

    BNOBegin(bnoayarlari);
    DEBUG_PRINTLN(F("BNO055 Baslatildi..."));

    MMCBegin(false, 0);

    BMPInit(BMP_OverSampling_8x, BMP_OverSampling_2x, BMP_IIR_OFF, BMP_ODR_40ms);
    DEBUG_PRINTLN(F("BMP388 Baslatildi..."));

    ahrs.begin();
    ahrs.setDOF(DOF::DOF_9);
    ahrs.setFusionAlgorithm(SensorFusion::KALMAN);
    DEBUG_PRINTLN(F("AHRS Algoritmasi Baslatildi..."));

    for(int i = 0; i < 100; i++){
        BMPGetData(&sicaklikData, &basincData, &irtifaData);
        irtifaRampa += irtifaData;
        delay(11);
    }
    irtifaRampa /= 100;
    DEBUG_PRINTLN(F("Rampa Irtifasi Hesaplandi..."));

    sureGPS = millis();

    delay(1000);

    DEBUG_PRINTLN(F("Baslatilma Sinyali Gonderiliyor..."));
    messageBuffer[0] = 'z';
    messageBuffer[1] = 'e';
    messageBuffer[44] = calculateCRC8(messageBuffer, 44);
    sendFixedDataPacket(RFGondericiHighAdresi, RFGondericiLowAdresi, RFGondericiKanal, messageBuffer, sizeof(messageBuffer));
    DEBUG_PRINTLN(F("Baslatilma Sinyali Gonderildi..."));
    delay(4000);
    sureRF = millis();
}

void loop(){
    sureCurrent = millis();

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
    qData = ahrs.getQuaternion();

    if(sureCurrent > sureGPS + 5000){
        //getGPSData(&gpsEnlem, &gpsBoylam, &gpsIrtifa, &uyduSayisi);

        gpsEnlem = 39.8190 + random(30, 50) / 1000000;
        gpsBoylam = 32.5630 + random(350, 400) / 1000000;
        gpsIrtifa = random(980, 990);
        uyduSayisi = 4;
        sureGPS = millis();
    }

    BMPGetData(&sicaklikData, &basincData, &irtifaData);

    DEBUG_PRINT(F(">IvmeX:"));DEBUG_PRINTLN(ivmeData.x);
    DEBUG_PRINT(F(">IvmeY:"));DEBUG_PRINTLN(ivmeData.y);
    DEBUG_PRINT(F(">IvmeZ:"));DEBUG_PRINTLN(ivmeData.z);

    DEBUG_PRINT(F(">GyroX:"));DEBUG_PRINTLN(gyroData.x);
    DEBUG_PRINT(F(">GyroY:"));DEBUG_PRINTLN(gyroData.y);
    DEBUG_PRINT(F(">GyroZ:"));DEBUG_PRINTLN(gyroData.z);

    DEBUG_PRINT(F(">MagX:"));DEBUG_PRINTLN(magData.x);
    DEBUG_PRINT(F(">MagY:"));DEBUG_PRINTLN(magData.y);
    DEBUG_PRINT(F(">MagZ:"));DEBUG_PRINTLN(magData.z);

    DEBUG_PRINT(F(">Pitch:"));DEBUG_PRINTLN(pitch);
    DEBUG_PRINT(F(">Roll:"));DEBUG_PRINTLN(roll);
    DEBUG_PRINT(F(">Yaw:"));DEBUG_PRINTLN(yaw);
    DEBUG_PRINT(F(">DunyaNormalAcisi:"));DEBUG_PRINTLN(90 - pitch);

    DEBUG_PRINT(F(">DordeyQ0:"));DEBUG_PRINTLN(qData.q0);
    DEBUG_PRINT(F(">DordeyQ1:"));DEBUG_PRINTLN(qData.q1);
    DEBUG_PRINT(F(">DordeyQ2:"));DEBUG_PRINTLN(qData.q2);
    DEBUG_PRINT(F(">DordeyQ3:"));DEBUG_PRINTLN(qData.q3);

    DEBUG_PRINT(F(">Sicaklik:"));DEBUG_PRINTLN(sicaklikData);
    DEBUG_PRINT(F(">Basinc:"));DEBUG_PRINTLN(basincData);
    DEBUG_PRINT(F(">Irtifa:"));DEBUG_PRINTLN(irtifaData);
    DEBUG_PRINT(F(">RampaIrtifa:"));DEBUG_PRINTLN(irtifaData - irtifaRampa);

    DEBUG_PRINT(F(">GPSEnlem:"));DEBUG_PRINTLN(gpsEnlem);
    DEBUG_PRINT(F(">GPSBoylam:"));DEBUG_PRINTLN(gpsBoylam);
    DEBUG_PRINT(F(">GPSIrtifa:"));DEBUG_PRINTLN(gpsIrtifa);

    if(sureCurrent > sureRF + 3000){
        converter();
        sendFixedDataPacket(RFGondericiHighAdresi, RFGondericiLowAdresi, RFGondericiKanal, messageBuffer, sizeof(messageBuffer));
        sureRF = millis();
    }

    delay(BeklemeSuresi);
}

void converter(){
    Float2ByteConverter convert;
    uint8_t crc = 0;

    convert.floatvar = ivmeData.x;
    for(int i = 0; i < 4; i++)messageBuffer[i] = convert.bytevar[i];

    convert.floatvar = ivmeData.y;
    for(int i = 0; i < 4; i++)messageBuffer[i+4] = convert.bytevar[i];
    
    convert.floatvar = ivmeData.z;
    for(int i = 0; i < 4; i++)messageBuffer[i+8] = convert.bytevar[i];

    convert.floatvar = gyroData.x;
    for(int i = 0; i < 4; i++)messageBuffer[i+12] = convert.bytevar[i];

    convert.floatvar = gyroData.y;
    for(int i = 0; i < 4; i++)messageBuffer[i+16] = convert.bytevar[i];

    convert.floatvar = gyroData.z;
    for(int i = 0; i < 4; i++)messageBuffer[i+20] = convert.bytevar[i];

    convert.floatvar = gpsEnlem;
    for(int i = 0; i < 4; i++)messageBuffer[i+24] = convert.bytevar[i];

    convert.floatvar = gpsBoylam;
    for(int i = 0; i < 4; i++)messageBuffer[i+28] = convert.bytevar[i];

    convert.floatvar = gpsIrtifa;
    for(int i = 0; i < 4; i++)messageBuffer[i+32] = convert.bytevar[i];

    convert.floatvar = irtifaData;
    for(int i = 0; i < 4; i++)messageBuffer[i+36] = convert.bytevar[i];

    convert.floatvar = pitch;
    for(int i = 0; i < 4; i++)messageBuffer[i+40] = convert.bytevar[i];

    crc = calculateCRC8(messageBuffer, 44);

    messageBuffer[44] = crc;
}