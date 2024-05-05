#include "rf_main.h"
#include <Arduino.h>

#define E32_TTL_1W
#define FREQUENCY_433

#include <LoRa_E32.h>

#define AUX_PIN PB1
#define AUX_PIN_Receiver PB4

void setsMainOpt(void);
void rfInit(void);

int counter = 0;

HardwareSerial UART1(PA3, PA2);

LoRa_E32 e32ttl1w(&UART1, AUX_PIN);


struct BMPData
{
    byte temp[4];
    byte pres[4];
    byte alt[4];
};

void rfInit()
{
    e32ttl1w.begin();

    setsMainOpt();

    e32ttl1w.setMode(MODE_0_NORMAL);
}

void setsMainOpt()
{
    // Bu Function RF'in başlangıçtaki temel ayarlarını yapmaktadır.

    ResponseStructContainer ayar;
    ayar = e32ttl1w.getConfiguration();

    Configuration conf = *(Configuration *)ayar.data;

    conf.ADDL = 0x0;
    conf.ADDH = 0x01;
    conf.CHAN = 23;

    conf.OPTION.fec = FEC_1_ON;
    conf.OPTION.fixedTransmission = FT_FIXED_TRANSMISSION;
    conf.OPTION.ioDriveMode = IO_D_MODE_PUSH_PULLS_PULL_UPS;
    conf.OPTION.wirelessWakeupTime = WAKE_UP_250;
    conf.OPTION.transmissionPower = POWER_30;

    conf.SPED.airDataRate = AIR_DATA_RATE_010_24;
    conf.SPED.uartBaudRate = UART_BPS_9600;
    conf.SPED.uartParity = MODE_00_8N1;

    ResponseStatus rs = e32ttl1w.setConfiguration(conf, WRITE_CFG_PWR_DWN_SAVE);
    //Serial2.println("Config: " + rs.getResponseDescription());

    //printParameters(conf);
    ayar.close();
}

void printParameters(struct Configuration conf)
{
    // Bu Fonksiyon RF modülünün tüm ayarlarını printliyor.

    Serial2.println("----------------------------------------");

    Serial2.print(F("HEAD : "));
    Serial2.print(conf.HEAD, BIN);
    Serial2.print(" ");
    Serial2.print(conf.HEAD, DEC);
    Serial2.print(" ");
    Serial2.println(conf.HEAD, HEX);
    Serial2.println(F(" "));
    Serial2.print(F("AddH : "));
    Serial2.println(conf.ADDH, BIN);
    Serial2.print(F("AddL : "));
    Serial2.println(conf.ADDL, BIN);
    Serial2.print(F("Chan : "));
    Serial2.print(conf.CHAN, DEC);
    Serial2.print(" -> ");
    Serial2.println(conf.getChannelDescription());
    Serial2.println(F(" "));
    Serial2.print(F("SpeedParityBit     : "));
    Serial2.print(conf.SPED.uartParity, BIN);
    Serial2.print(" -> ");
    Serial2.println(conf.SPED.getUARTParityDescription());
    Serial2.print(F("SpeedUARTDatte  : "));
    Serial2.print(conf.SPED.uartBaudRate, BIN);
    Serial2.print(" -> ");
    Serial2.println(conf.SPED.getUARTBaudRate());
    Serial2.print(F("SpeedAirDataRate   : "));
    Serial2.print(conf.SPED.airDataRate, BIN);
    Serial2.print(" -> ");
    Serial2.println(conf.SPED.getAirDataRate());

    Serial2.print(F("OptionTrans        : "));
    Serial2.print(conf.OPTION.fixedTransmission, BIN);
    Serial2.print(" -> ");
    Serial2.println(conf.OPTION.getFixedTransmissionDescription());
    Serial2.print(F("OptionPullup       : "));
    Serial2.print(conf.OPTION.ioDriveMode, BIN);
    Serial2.print(" -> ");
    Serial2.println(conf.OPTION.getIODroveModeDescription());
    Serial2.print(F("OptionWakeup       : "));
    Serial2.print(conf.OPTION.wirelessWakeupTime, BIN);
    Serial2.print(" -> ");
    Serial2.println(conf.OPTION.getWirelessWakeUPTimeDescription());
    Serial2.print(F("OptionFEC          : "));
    Serial2.print(conf.OPTION.fec, BIN);
    Serial2.print(" -> ");
    Serial2.println(conf.OPTION.getFECDescription());
    Serial2.print(F("OptionPower        : "));
    Serial2.print(conf.OPTION.transmissionPower, BIN);
    Serial2.print(" -> ");
    Serial2.println(conf.OPTION.getTransmissionPowerDescription());

    Serial2.println("----------------------------------------");
}

/* void sendMessage(void){
    // Yarışma için
    struct Message
    {
        byte altitude[4];
        byte gpsAltitude[4];
        byte gpsEnlem[4];
        byte gpsBoylam[4];
        byte gyroX[4];
        byte gyroY[4];
        byte gyroZ[4];
        byte accX[4];
        byte accY[4];
        byte accZ[4];
    } mes;

    ResponseStatus mes = e32ttl1w.sendMessage(&mes, sizeof(mes));
} */

void haberlesmeTestTransmitter(void)
{
    counter++;
    Serial2.println("Sending Message...");
    ResponseStatus rs = e32ttl1w.sendMessage(String(counter));
    Serial2.println(rs.getResponseDescription());

    /*while(!e32ttl1w.available()){
        delay(100);
    }

    if(e32ttl1w.available() > 1){
        Serial2.println("Message Arrived...");
        ResponseContainer rc = e32ttl1w.receiveMessage();

        String mes = rc.data;

        Serial2.println("Message: " + mes);
        Serial2.println(rc.status.getResponseDescription());
    }*/
}

void haberlesmeTransmitterAct(String action)
{
    struct Message
    {
        char type;
        char mes[10];
    } file;

    file.type = 'A';
    *(String *)file.mes = action;

    ResponseStatus rs = e32ttl1w.sendMessage(&file, sizeof(file));
    Serial2.println(rs.getResponseDescription());
}

void haberlesmeReceiverAct()
{
    struct Message
    {
        char type;
        char mes[10];
    };

    if (e32ttl1w.available() > 1)
    {
        ResponseStructContainer rsc = e32ttl1w.receiveMessage(sizeof(Message));
        struct Message message = *(Message *)rsc.data;

        if (message.type == 'A')
        {
        }

        else if (message.type == 'D')
        {
        }

        free(rsc.data);
    }
}

int i = 0;
void denemeHaberlesmeTransmitter()
{
    i++;
    ResponseStatus rs = e32ttl1w.sendBroadcastFixedMessage(23,String(i));
    //Serial2.println(rs.getResponseDescription());
}

void denemeHaberlesmeReceiver()
{
    if (e32ttl1w.available() > 1)
    {
        ResponseContainer rsc = e32ttl1w.receiveMessage();
        if(rsc.status.code != 1){
            rsc.status.getResponseDescription();
        }
        else{
            Serial2.print("Data: ");
            Serial2.println(rsc.data);
        }
    }
}