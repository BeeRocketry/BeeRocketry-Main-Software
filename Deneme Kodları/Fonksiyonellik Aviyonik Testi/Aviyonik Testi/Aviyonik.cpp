#include <Arduino.h>

#define DEBUG_MODE

#undef Serial
#define Serial SeriPort

#include "debugprinter.h"
#include <HardwareSerial.h>
#include <TinyGPS.h>
#include <ReefwingAHRS.h>
#include "main.h"
#include "rf.h"
#include "bmp388.h"
#include "BNO055.h"
#include "gps.h"
#include "altitude.h"
#include "MMC5603.h"

#define UartRXPini PA3
#define UartTXPini PA2
#define UartBaudRate 115200

#define RFUartRXPini PA13
#define RFUartTXPini PA12
#define RFBaudRate 9600

#define RFLowAdresi 0x01
#define RFHighAdresi 0x03
#define RFKanal 23U

#define I2CSDAPini PB7
#define I2CSCLPini PB6

#define RFGondericiLowAdresi 0x05
#define RFGondericiHighAdresi 0x03
#define RFGondericiKanal 23U

#define BeklemeSuresi 20

#define BuzzerPini PB3

#define RampaSinirIvmeDegeri 5
#define RampaSinirIrtifaDegeri 50
#define IrtifaSinirDegeri 200
#define KurtarmaTotalIvmeDegeri 0.05f
#define KurtarmaHizDegeri -50

HardwareSerial SerialRF(RFUartRXPini, RFUartTXPini);
HardwareSerial SeriPort(UartRXPini, UartTXPini);
ReefwingAHRS ahrs;
static AltitudeEstimator altitude = AltitudeEstimator(0.0005, 0.0005, 0.018, 0.5, 0.1);

typedef union{
    float floatvar;
    uint8_t bytevar[4];
}Float2ByteConverter;

BNO_DOF3_Float ivmeData, gyroData, magData; // m^2/s , dps , uT
float ivmeDataAlg[3], gyroDataAlg[3]; // g , rps
float totalIvme = 0; // g
float pitch, roll, yaw;
SensorData imuData;
Quaternion qData;

float basincData, sicaklikData, irtifaData, irtifaRampa, gercekIrtifa;
float maksIrtifa = 0, apogeeIrtifa = 0;

float gpsEnlem, gpsBoylam, gpsIrtifa;

float dikeyHiz = 0, dikeyIvme = 0, filtreIrtifa = 0;

struct ConfigRF rfayarlari;
BNO_STR_REGISTERS bnoayarlari;

uint8_t messageBuffer[44];

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

void setup(){
    SeriPort.begin(UartBaudRate);
    DEBUG_PRINTLN(F("Seri Port Baslatildi..."));

    RFBegin(&rfayarlari, RFHighAdresi, RFLowAdresi, RFKanal, UARTPARITY_8N1, UARTBAUDRATE_9600,
            AIRDATARATE_03k, FIXEDMODE, IO_PUSHPULL, WIRELESSWAKEUP_250, FEC_ON, TRANSMISSIONPOWER_30);
    
    I2CBegin(I2CSDAPini, I2CSCLPini);
    DEBUG_PRINTLN(F("I2C Portu Baslatildi..."));

    BNOBegin(bnoayarlari);
    DEBUG_PRINTLN(F("BNO055 Baslatildi..."));

    MMCBegin(false, 0);

    BMPInit(BMP_OverSampling_8x, BMP_OverSampling_4x, BMP_IIR_OFF, BMP_ODR_10ms);
    DEBUG_PRINTLN(F("BMP388 Baslatildi..."));

    ahrs.begin();
    ahrs.setDOF(DOF::DOF_9);
    ahrs.setFusionAlgorithm(SensorFusion::KALMAN);
    DEBUG_PRINTLN(F("AHRS Algoritmasi Baslatildi..."));

    pinMode(BuzzerPIN, OUTPUT);

    for(int i = 0; i < 100; i++){
        BMPGetData(&sicaklikData, &basincData, &irtifaData);
        irtifaRampa += irtifaData;
        delay(11);
    }
    irtifaRampa /= 100;
    DEBUG_PRINTLN(F("Rampa Irtifasi Hesaplandi..."));

    DEBUG_PRINTLN(F("Baslangic Buzzer Sesi Baslatildi..."));
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
    magData = getMagData();

    ivmeDataAlg[0] = ivmeData.x / 9.80665;
    ivmeDataAlg[1] = ivmeData.y / 9.80665;
    ivmeDataAlg[2] = ivmeData.z / 9.80665;

    totalIvme = sqrt(pow(ivmeDataAlg[0], 2) + pow(ivmeDataAlg[1], 2) + pow(ivmeDataAlg[2], 2));

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
                rampaCheck = false;
                durumKarakteri = 'b';
            }
        }
        else{
            rampaIvmeCounter = 10;
        }

        if(gercekIrtifa > RampaSinirIrtifaDegeri){
            rampaIrtifaCounter--;
            if(rampaIrtifaCounter <= 0){
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
            if(irtifaData - irtifaRampa > IrtifaSinirDegeri){
                irtifaSinirCounter--;
                if(irtifaSinirCounter <= 0){
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

                if(dikeyHiz < KurtarmaHizDegeri){
                    birinciKurtarmaIrtifaCounter--;
                    if(birinciKurtarmaIrtifaCounter <= 0){
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
}

void converter(){

}