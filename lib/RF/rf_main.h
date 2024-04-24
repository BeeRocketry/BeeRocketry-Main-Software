#include <Arduino.h>

#define E32_TTL_1W
#define FREQUENCY_433

#include <LoRa_E32.h>

void setsMainOpt(void);
void rfInit(void);
void sendMessage(void);
void sendFixedMessage(String message, int highadr, int lowadr, int chan);
void printParameters(struct Configuration conf);