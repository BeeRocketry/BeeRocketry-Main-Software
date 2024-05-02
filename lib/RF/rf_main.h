#include <Arduino.h>

#define E32_TTL_1W
#define FREQUENCY_433

#include <LoRa_E32.h>

void setsMainOpt(void);
void setsMainOptreceiver();
void rfInitReceiver();
void rfInit(void);
void sendMessage(void);
void printParameters(struct Configuration conf);
void haberlesmeTestTransmitter(void);
void haberlesmeTestReceiver(void);