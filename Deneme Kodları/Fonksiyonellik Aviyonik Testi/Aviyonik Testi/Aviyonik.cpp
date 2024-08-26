/*
*************************************************************************************************************************

Aviyonik Sistem Testi - Ana Aviyonik Kod Bloğu

AVİYONİK SİSTEM TESTİ NEDİR?
----------------------------
    Aviyonik sistem testi roketin ana uçuş planında bulunan  gövde  ayrılmalarının  algoritmik  çalışırlığını  kanıtlamak
için istenen testtir.

AVİYONİK SİSTEM GENEL ÖZET
--------------------------
    Bu testi gerçekleştirmek üzere 1 adet devremiz  bulunmaktadır. Bu  devre  hem  tüm  verileri  devamlı  olarak  sensör
verilerini yazdırmaktadır, hem de uçuş planında  gerçekleşen  tetiklemeleri  yazdırmaktadır.  Ana  aviyonik  sistemimizde
erken ve yanlış tetiklemeleri engellemek üzere 3 adet durum değişkeni  bulunmaktadır.  Bunlar  sırasıyla  rampa,  güvenli
irtifa ve kurtarma sistem kontrol değişkenidir. Bu değişkenlerin açıklaması aşağıda belirtilmiştir.

    Rampa Durum Değişkeni : Roketin rampada olduğunu kontrol eden durum değişkenidir. Başlangıçta doğru  durumda  başlar.
                            Herhangi bir yüksek ivmeyle karşılaştığında ya da rampada  bulunduğu  irtifadan  belirli  bir
                            yükselikte ayrıldıktan sonra rampa değişkeni yanlış durumuna geçer.

    Güvenli İrtifa Durum Değişkeni : Roketin güvenli  irtifayı  aşıp  aşmadığını  kontrol  eden  durum  değişkenidir.  Bu
                                     değişken yanlış olarak başlar ve önceden belirlenmiş bir  irtifa  değeri  aşıldıktan
                                     sonra doğru duruma geçer.
                
    Kurtarma Sistem Kontrol Durum Değişkeni : Roketin  2.   kurtarma    sisteminin    1.    kurtarma   sisteminden   önce
                                              tetiklenmemesi için bulunan durum değişkenidir. Bu değişken  yanlış  olarak
                                              başlar ve 1. kurtarma sistem değişkeni çalışmasıyla doğru duruma geçer.

Durum değişkenlerinden sonra sistemimizin asıl kurtarma sistemlerini tetikleyecek olan veri  değişkenleri  bulunmaktadır.
Bunlardan biri irtifa düşüş diğeri ise 0g kontrolüdür. İrtifa düşüş irtifadan ve ivmeden dikey hız hesabı yaparak belirli
bir hızın altında olmasını kontrol ederken 0g kontrolü sistem üzerindeki  total  ivme  vektörünün  0g'ye  yakın  olmasını
kontrol eder. İki durumunda gerçekleşmesi durumunda  1. kurtarma  sistemi  tetiklenirken  irtifanın  belirli  bir  değere
ulaşmasıyla 2. kurtarma sistemi tetiklenir.

KULLANILAN MODÜLLER
-------------------
RF Modülü         -- EByte E32-433T30D
IMU Modülü        -- Bosch BNO055
Barometre Modülü  -- Bosch BMP388
Mikro Denetleyici -- STM32 Blackpill (STM32F411RE)

*************************************************************************************************************************
*/

#include <Arduino.h>

#define DEBUG_MODE

#undef Serial
#define Serial SeriPort

#include "debugprinter.h"
#include <HardwareSerial.h>
#include "rf.h"
#include "bmp388.h"
#include "BNO055.h"
#include "altitude.h"
#include "MMC5603.h"

#define UartRXPini PA10
#define UartTXPini PA9
#define UartBaudRate 115200

#define RFUartRXPini PC7
#define RFUartTXPini PC6
#define RFBaudRate 9600

#define RFLowAdresi 0x01
#define RFHighAdresi 0x03
#define RFKanal 23U

#define I2CSDAPini PB7
#define I2CSCLPini PB6

#define RFGondericiLowAdresi 0x05
#define RFGondericiHighAdresi 0x03
#define RFGondericiKanal 23U

#define BeklemeSuresi 25

#define BuzzerPini PB0
#define BuzzerPIN PB0

#define RampaSinirIvmeDegeri 3
#define RampaSinirIrtifaDegeri 50
#define IrtifaSinirDegeri 200
#define KurtarmaTotalIvmeDegeri 0.05f
#define KurtarmaHizDegeri -50

HardwareSerial SerialRF(RFUartRXPini, RFUartTXPini);
HardwareSerial SeriPort(UartRXPini, UartTXPini);
static AltitudeEstimator altitude = AltitudeEstimator(0.0005, 0.0005, 0.018, 0.5, 0.1);

void printDebug();
void converter();

typedef union{
    float floatvar;
    uint8_t bytevar[4];
}Float2ByteConverter;

BNO_DOF3_Float ivmeData, gyroData; // m^2/s , dps
float ivmeDataAlg[3], gyroDataAlg[3]; // g , rps
float totalIvme = 0; // g

float basincData, sicaklikData, irtifaData, irtifaRampa, gercekIrtifa;
float maksIrtifa = 0, apogeeIrtifa = 0;

float gpsEnlem, gpsBoylam, gpsIrtifa;

float dikeyHiz = 0, dikeyIvme = 0, filtreIrtifa = 0;

struct ConfigRF rfayarlari;
BNO_STR_REGISTERS bnoayarlari;

uint8_t messageBuffer[32];

long sureCurrent;

bool rampaCheck = true;
bool irtifaSinirCheck = false;
bool kurtarmaSistemCheck = false;
bool ikinciKurtarmaSistemCheck = false;

uint8_t rampaIvmeCounter = 10;
uint8_t rampaIrtifaCounter = 10;
uint8_t irtifaSinirCounter = 10;
uint8_t birinciKurtarmaIvmeCounter = 10;
uint8_t birinciKurtarmaIrtifaCounter = 10;
uint8_t ikinciKurtarmaCounter = 10;

long kurtarmaDelay;
bool kurtarmaDelayCheck = false;
long IkinciKurtarmaDelay;
bool IkinciKurtarmaDelayCheck = false;

uint8_t durumKarakteri = '0';
uint8_t paketSayaci = 0;

void setup(){
    SeriPort.begin(UartBaudRate);
    DEBUG_PRINTLN(F("Seri Port Baslatildi..."));

    //RFBegin(&rfayarlari, RFHighAdresi, RFLowAdresi, RFKanal, UARTPARITY_8N1, UARTBAUDRATE_115200,
    //        AIRDATARATE_03k, FIXEDMODE, IO_PUSHPULL, WIRELESSWAKEUP_250, FEC_ON, TRANSMISSIONPOWER_30);
    
    I2CBegin(I2CSDAPini, I2CSCLPini);
    DEBUG_PRINTLN(F("I2C Portu Baslatildi..."));

    BNOBegin(bnoayarlari);
    DEBUG_PRINTLN(F("BNO055 Baslatildi..."));

    MMCBegin(false, 0);

    BMPInit(BMP_OverSampling_8x, BMP_OverSampling_4x, BMP_IIR_OFF, BMP_ODR_10ms);
    DEBUG_PRINTLN(F("BMP388 Baslatildi..."));

    pinMode(BuzzerPIN, OUTPUT);

    for(int i = 0; i < 100; i++){
        BMPGetData(&sicaklikData, &basincData, &irtifaData);
        irtifaRampa += irtifaData;
        delay(11);
    }
    irtifaRampa /= 10;
    DEBUG_PRINTLN(F("Rampa Irtifasi Hesaplandi..."));

    DEBUG_PRINTLN(F("Baslangic Buzzer Sesi Baslatildi..."));
    pinMode(BuzzerPIN, OUTPUT);

    digitalWrite(BuzzerPIN, HIGH);
    delay(500);
    digitalWrite(BuzzerPIN, LOW);
    delay(500);
    digitalWrite(BuzzerPIN, HIGH);
    delay(500);
    digitalWrite(BuzzerPIN, LOW);
    delay(500);
    DEBUG_PRINTLN(F("Baslangic Buzzer Sesi Bitti..."));

    durumKarakteri = '1';
}

void loop(){
    sureCurrent = millis();

    ivmeData = getAccData();
    gyroData = getGyroData();

    ivmeDataAlg[0] = ivmeData.x / 9.80665;
    ivmeDataAlg[1] = ivmeData.y / 9.80665;
    ivmeDataAlg[2] = ivmeData.z / 9.80665;

    totalIvme = sqrtf(powf(ivmeDataAlg[0], 2) + powf(ivmeDataAlg[1], 2) + powf(ivmeDataAlg[2], 2));

    gyroDataAlg[0] = gyroData.x * DEG_TO_RAD;
    gyroDataAlg[1] = gyroData.y * DEG_TO_RAD;
    gyroDataAlg[2] = gyroData.z * DEG_TO_RAD;

    BMPGetData(&sicaklikData, &basincData, &irtifaData);
    gercekIrtifa = irtifaData - irtifaRampa;
    if(gercekIrtifa > maksIrtifa){
        maksIrtifa = gercekIrtifa;
    }

    uint32_t timestamp = micros();

    altitude.estimate(ivmeDataAlg, gyroDataAlg, gercekIrtifa, timestamp);
    dikeyHiz = altitude.getVerticalVelocity();
    dikeyIvme = altitude.getVerticalAcceleration();
    filtreIrtifa = altitude.getAltitude();

    if(rampaCheck == true){
        if(totalIvme > RampaSinirIvmeDegeri){
            rampaIvmeCounter--;
            if(rampaIvmeCounter <= 0){
                DEBUG_PRINTLN(F("Rampa Sinir Ivmesi Asildi..."));
                DEBUG_PRINTLN();
                rampaCheck = false;
                durumKarakteri = 'b';
            }
        }
        else{
            rampaIvmeCounter = 10;
        }

        if(rampaCheck == true && gercekIrtifa > RampaSinirIrtifaDegeri){
            rampaIrtifaCounter--;
            if(rampaIrtifaCounter <= 0){
                DEBUG_PRINTLN(F("Rampa Sinir Irtifasi Asildi..."));
                DEBUG_PRINTLN();
                rampaCheck = false;
                durumKarakteri = 'b';
            }
        }
        else{
            rampaIrtifaCounter = 10;
        }
    }
    else{
        if(irtifaSinirCheck == false){
            if(gercekIrtifa > IrtifaSinirDegeri){
                irtifaSinirCounter--;
                if(irtifaSinirCounter <= 0){
                    DEBUG_PRINTLN(F("Irtifa Sinir Degeri Asildi..."));
                    irtifaSinirCheck = true;
                    durumKarakteri = 'c';
                }
            }
            else{
                irtifaSinirCounter = 10;
            }
        }
        else{
            if(kurtarmaSistemCheck == false){
                if(totalIvme < KurtarmaTotalIvmeDegeri && abs(dikeyHiz) < 1){
                    birinciKurtarmaIvmeCounter--;
                    if(birinciKurtarmaIvmeCounter <= 0){
                        DEBUG_PRINTLN(F("Birinci Kurtarma Sistemi Aktiflestirdi..."));
                        DEBUG_PRINTLN();

                        kurtarmaSistemCheck = true;
                        apogeeIrtifa = gercekIrtifa;
                        digitalWrite(BuzzerPIN, HIGH);
                        kurtarmaDelayCheck = true;
                        kurtarmaDelay = millis();
                        durumKarakteri = 'd';
                    }
                }
                else{
                    birinciKurtarmaIvmeCounter = 10;
                }

                if(kurtarmaSistemCheck == false && dikeyHiz < KurtarmaHizDegeri){
                    birinciKurtarmaIrtifaCounter--;
                    if(birinciKurtarmaIrtifaCounter <= 0){
                        DEBUG_PRINTLN(F("Birinci Kurtarma Sistemi Aktiflestirdi..."));
                        DEBUG_PRINTLN();

                        kurtarmaSistemCheck = true;
                        apogeeIrtifa = gercekIrtifa;
                        digitalWrite(BuzzerPIN, HIGH);
                        kurtarmaDelayCheck = true;
                        kurtarmaDelay = millis();
                        durumKarakteri = 'd';
                    }
                }
                else{
                    birinciKurtarmaIrtifaCounter = 10;
                }
            }
            else{
                if(gercekIrtifa < 470){
                    ikinciKurtarmaCounter--;
                    if(ikinciKurtarmaCounter <= 0){
                        DEBUG_PRINTLN(F("Ikinci Kurtarma Sistemi Aktiflestirdi..."));
                        DEBUG_PRINTLN();

                        ikinciKurtarmaSistemCheck = true;
                        digitalWrite(BuzzerPIN, HIGH);
                        IkinciKurtarmaDelay = millis();
                        IkinciKurtarmaDelayCheck = true;
                        durumKarakteri = 'e';
                    }
                }
                else{
                    ikinciKurtarmaCounter = 10;
                }
            }
        }
    }

    if(kurtarmaDelayCheck == true){
        if(millis() > kurtarmaDelay + 3000){
            digitalWrite(BuzzerPIN, LOW);
            kurtarmaDelayCheck = false;
        }
    }

    if(IkinciKurtarmaDelayCheck == true){
        if(millis() > IkinciKurtarmaDelay + 3000){
            digitalWrite(BuzzerPIN, LOW);
            IkinciKurtarmaDelayCheck = false;
        }
    }

    printDebug();

    //converter();
    //sendFixedDataPacket(RFGondericiHighAdresi, RFGondericiLowAdresi, RFGondericiKanal, messageBuffer, sizeof(messageBuffer));

    delay(BeklemeSuresi);
}

void printDebug(){
    DEBUG_PRINT(F(">RampaDeger: "));DEBUG_PRINTLN(rampaCheck);
    DEBUG_PRINT(F(">IrtifaSinirCheck: "));DEBUG_PRINTLN(irtifaSinirCheck);
    DEBUG_PRINT(F(">KurtarmaCheck: "));DEBUG_PRINTLN(kurtarmaSistemCheck);
    DEBUG_PRINT(F(">IkinciKurtarmaCheck: "));DEBUG_PRINTLN(ikinciKurtarmaSistemCheck);

    DEBUG_PRINT(F(">GercekIrtifa: "));DEBUG_PRINTLN(gercekIrtifa);
    DEBUG_PRINT(F(">RampaIrtifa: "));DEBUG_PRINTLN(irtifaRampa);
    DEBUG_PRINT(F(">FiltreIrtifa: "));DEBUG_PRINTLN(filtreIrtifa);
    DEBUG_PRINT(F(">OlculenIrtifa: "));DEBUG_PRINTLN(irtifaData);
    DEBUG_PRINT(F(">MaksIrtifa: "));DEBUG_PRINTLN(maksIrtifa);
    DEBUG_PRINT(F(">ApogeeIrtifa: "));DEBUG_PRINTLN(apogeeIrtifa);

    DEBUG_PRINT(F(">IvmeX(m2): "));DEBUG_PRINTLN(ivmeData.x);
    DEBUG_PRINT(F(">IvmeY(m2): "));DEBUG_PRINTLN(ivmeData.y);
    DEBUG_PRINT(F(">IvmeZ(m2): "));DEBUG_PRINTLN(ivmeData.z);

    DEBUG_PRINT(F(">IvmeX(g): "));DEBUG_PRINTLN(ivmeDataAlg[0]);
    DEBUG_PRINT(F(">IvmeY(g): "));DEBUG_PRINTLN(ivmeDataAlg[1]);
    DEBUG_PRINT(F(">IvmeZ(g): "));DEBUG_PRINTLN(ivmeDataAlg[2]);
    DEBUG_PRINT(F(">TotalIvme(g): "));DEBUG_PRINTLN(totalIvme);

    DEBUG_PRINT(F(">GyroX: "));DEBUG_PRINTLN(gyroData.x);
    DEBUG_PRINT(F(">GyroY: "));DEBUG_PRINTLN(gyroData.y);
    DEBUG_PRINT(F(">GyroZ: "));DEBUG_PRINTLN(gyroData.z);

    DEBUG_PRINT(F(">DikeyHiz: "));DEBUG_PRINTLN(dikeyHiz);
    DEBUG_PRINT(F(">DikeyIvme: "));DEBUG_PRINTLN(dikeyIvme);

    DEBUG_PRINT(F(">RampaIvmeCounter: "));DEBUG_PRINTLN(rampaIvmeCounter);
    DEBUG_PRINT(F(">RampaIrtifaCounter: "));DEBUG_PRINTLN(rampaIrtifaCounter);
    DEBUG_PRINT(F(">IrtifaSinirCounter: "));DEBUG_PRINTLN(irtifaSinirCounter);
    DEBUG_PRINT(F(">BirinciKurtarmaIvmeCounter: "));DEBUG_PRINTLN(birinciKurtarmaIvmeCounter);
    DEBUG_PRINT(F(">BirinciKurtarmaIrtifaCounter: "));DEBUG_PRINTLN(birinciKurtarmaIrtifaCounter);
    DEBUG_PRINT(F(">IkinciKurtarmaCounter: "));DEBUG_PRINTLN(ikinciKurtarmaCounter);
}

void converter(){
    Float2ByteConverter convert;
    if(paketSayaci == 256){
        paketSayaci = 0;
    }

    messageBuffer[0] = paketSayaci++;
    messageBuffer[1] = durumKarakteri;
    messageBuffer[2] = (uint8_t)rampaCheck;
    messageBuffer[3] = (uint8_t)irtifaSinirCheck;
    messageBuffer[4] = (uint8_t)kurtarmaSistemCheck;
    messageBuffer[5] = (uint8_t)ikinciKurtarmaSistemCheck;

    convert.floatvar = gercekIrtifa;
    for(int i = 0; i < 4; i++)messageBuffer[i+6] = convert.bytevar[i];

    convert.floatvar = ivmeDataAlg[0];
    for(int i = 0; i < 4; i++)messageBuffer[i+10] = convert.bytevar[i];
    convert.floatvar = ivmeDataAlg[1];
    for(int i = 0; i < 4; i++)messageBuffer[i+14] = convert.bytevar[i];
    convert.floatvar = ivmeDataAlg[2];
    for(int i = 0; i < 4; i++)messageBuffer[i+18] = convert.bytevar[i];

    convert.floatvar = dikeyHiz;
    for(int i = 0; i < 4; i++)messageBuffer[i+22] = convert.bytevar[i];

    messageBuffer[26] = rampaIvmeCounter;
    messageBuffer[27] = rampaIrtifaCounter;
    messageBuffer[28] = irtifaSinirCounter;
    messageBuffer[29] = birinciKurtarmaIvmeCounter;
    messageBuffer[30] = birinciKurtarmaIrtifaCounter;
    messageBuffer[31] = ikinciKurtarmaCounter;
}