#include "rf.h"

// CRC8 Hesaplama Fonksiyonu
uint8_t calculateCRC8(const uint8_t *data, size_t length){
    if(length - 1 > MAX_TX_BUFFER_SIZE){
        Serial1.println("CRC8 Fonksiyonu Maksimum Paketten Büyük");
        return;
    }

    uint8_t crc = 0x00;

    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; ++j) {
            if (crc & 0x80){
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

int setSettings(struct ConfigRF confs){
    uint8_t SpedByte = 0, OptionByte = 0;
    uint8_t MesArr[7];

    SpedByte = (confs.RFSped.UARTParity << 6) | (confs.RFSped.UARTBaud << 3) | (confs.RFSped.AirDataRate);
    OptionByte = (confs.RFOption.TransmissionMode << 7) | (confs.RFOption.IODriver << 6) | (confs.RFOption.WirelessWakeUp << 3) | (confs.RFOption.FECset << 2) | (confs.RFOption.TransmissionPower);

    digitalWrite(RF_M0, HIGH);
    digitalWrite(RF_M1, HIGH);

    managedDelay(40);

    MesArr[0] = 0xC0;
    MesArr[1] = confs.AddressHigh;
    MesArr[2] = confs.AddressLow;
    MesArr[3] = SpedByte;
    MesArr[4] = confs.Channel;
    MesArr[5] = OptionByte;
    MesArr[6] = calculateCRC8(MesArr, 6);

    SerialRF.write((uint8_t *)MesArr, sizeof(MesArr) / sizeof(MesArr[0]));

    long startTime = millis();
    while(digitalRead(RF_AUX) != HIGH){
        if (millis() - startTime > 2000) {
            Serial1.println(F("RF_AUX zaman aşimina uğradi."));
            return -1;
        }
        managedDelay(20);
    }

    managedDelay(750);

    digitalWrite(RF_M0, LOW);
    digitalWrite(RF_M1, LOW);

    return 0;
}

int getSettings(struct ConfigRF *confs){
    uint8_t SpedByte = 0, OptionByte = 0;
    uint8_t MesArr[7];
    uint8_t receivedCRC;

    digitalWrite(RF_M0, HIGH);
    digitalWrite(RF_M1, HIGH);

    managedDelay(40);

    for(int i = 0; i < 3; i++){
        SerialRF.write(0xC1);
    }
    
    long startTime = millis();
    while(SerialRF.available() < sizeof(MesArr)){
        if(millis() - startTime > 1000){
            Serial1.println(F("Veri okuma zaman aşimina uğradi."));
            return -1;
        }
        managedDelay(20);
    }

    SerialRF.readBytes(MesArr, sizeof(MesArr));

    receivedCRC = MesArr[6];
    
    if(receivedCRC != calculateCRC8(MesArr, 6)){
        Serial1.println("CRC Hatasi: Veri Paketi Bozuk");
        return -2;
    }

    confs->AddressHigh = MesArr[1];
    confs->AddressLow = MesArr[2];
    SpedByte = MesArr[3];
    confs->Channel = MesArr[4];
    OptionByte = MesArr[5];

    confs->RFSped.UARTParity = (SpedByte >> 6) & 0b11;
    confs->RFSped.UARTBaud = (SpedByte >> 3) & 0b111;
    confs->RFSped.AirDataRate = (SpedByte) & 0b111;

    confs->RFOption.TransmissionMode = (OptionByte >> 7) & 0b1;
    confs->RFOption.IODriver = (OptionByte >> 6) & 0b1;
    confs->RFOption.WirelessWakeUp = (OptionByte >> 3) & 0b111;
    confs->RFOption.FECset = (OptionByte >> 2) & 0b1;
    confs->RFOption.TransmissionPower = (OptionByte) & 0b11;

    Serial1.println(F("------------------------------------------------------"));
    Serial1.print(F("Yüksek Adres: "));    Serial1.println(confs->AddressHigh);

    Serial1.print(F("Düşük Adres: "));    Serial1.println(confs->AddressLow);

    Serial1.print(F("Kanal: "));    Serial1.print(confs->Channel);
    Serial1.print(F(" - "));    Serial1.print(410+confs->Channel);   Serial1.println(F(" MHz"));
    Serial1.println();

    Serial1.println(F("Sped Ayarlari"));
    Serial1.print(F("  UART Baud Rate: "));     Serial1.println(getUARTBaudRate(confs->RFSped.UARTBaud));
    Serial1.print(F(" UART Parity: "));     Serial1.println(getUARTParity(confs->RFSped.UARTParity));
    Serial1.print(F("  Air Data Rate: "));     Serial1.println(getAirData(confs->RFSped.UARTParity));
    Serial1.println();

    Serial1.println(F("Option Ayarlari"));
    Serial1.print(F("  Transfer Türü: "));      Serial1.println(getTransmissionType(confs->RFOption.TransmissionMode));
    Serial1.print(F("  IO Türü: "));        Serial1.println(getIOMode(confs->RFOption.IODriver));
    Serial1.print(F("  Wireless Uyanma Süresi: "));     Serial1.println(getWirelessWakeup(confs->RFOption.WirelessWakeUp));
    Serial1.print(F("  FEC Filtresi: "));       Serial1.println(getFECFilter(confs->RFOption.FECset));
    Serial1.print(F("  Aktarim Gücü: "));       Serial1.println(getTranmissionPower(confs->RFOption.TransmissionPower));
    Serial1.println();
    Serial1.println(F("------------------------------------------------------"));

    managedDelay(750);

    digitalWrite(RF_M0, LOW);
    digitalWrite(RF_M1, LOW);

    return 0;
}

int receiveSingleData(uint8_t *data){
    unsigned long t = millis();

    while(digitalRead(RF_AUX) == LOW){
        if(millis() - t > 1000){
            Serial1.println("AUX Pini hazir durumda degil...");
            return -2;
        }
        managedDelay(20);
    }

    t = millis();
    while(SerialRF.available() == 0){
        if(millis() - t > 1000){
            Serial1.println("Veri Okuma Zaman Asimina Ugradi...");
            return -1;
        }
        managedDelay(20);
    }

    *data = SerialRF.read();
    Serial1.println("Veri Alindi...");
    return 0;
}

int receiveDataPacket(uint8_t *data, size_t size){
    unsigned long t = millis();

    while(digitalRead(RF_AUX) == LOW){
        if(millis() - t > 1000){
            Serial1.println("AUX Pini hazir durumda degil...");
            return -2;
        }
        managedDelay(20);
    }

    t = millis();
    while(SerialRF.available() < size){
        if(SerialRF.available() == 0 && millis() - t > 100){
            Serial1.println("Herhangi bir Veri Paketi gelmedi...");
            return -2;
        }
        else if(millis() - t > 1000){
            Serial1.println("Veri okuma zaman asimina ugradi...");
            return -1;
        }
        managedDelay(20);
    }

    SerialRF.readBytes(data, size);
    return 0;
}

int sendFixedSingleData(uint8_t AddressHigh, uint8_t AddressLow, uint8_t Channel, uint8_t data){
    uint8_t packet[4];
    packet[0] = AddressHigh;
    packet[1] = AddressLow;
    packet[2] = Channel;
    packet[3] = data;

    unsigned long t = millis();
    while(digitalRead(RF_AUX) == LOW){
        if(millis() - t > 500){
            Serial1.println("AUX Pini Veri Gönderim durumuna gecmedi....");
            return -1;
        }
        managedDelay(10);
    }

    SerialRF.write((uint8_t *)packet, sizeof(packet) / sizeof(packet[0]));

    return 0;
}

int sendTransparentSingleData(uint8_t data){
    unsigned long t = millis();
    while(digitalRead(RF_AUX) == LOW){
        if(millis() - t > 500){
            Serial1.println("AUX Pini Veri Gönderim durumuna gecmedi....");
            return -1;
        }
        managedDelay(10);
    }
    
    SerialRF.write(data);

    return 0;
}

int sendFixedDataPacket(uint8_t AddressHigh, uint8_t AddressLow, uint8_t Channel, uint8_t *data, size_t size){
    size_t totalPacket = (size + MAX_TX_BUFFER_SIZE - 1) / MAX_TX_BUFFER_SIZE;

    for(size_t i = 0; i < totalPacket; i++){
        size_t offset = i * MAX_TX_BUFFER_SIZE;
        size_t packetSize = (size - offset > MAX_TX_BUFFER_SIZE) ? MAX_TX_BUFFER_SIZE : (size - offset);

        uint8_t packet[packetSize + 3];

        packet[0] = AddressHigh;
        packet[1] = AddressLow;
        packet[2] = Channel;
        memcpy(&packet[3], &data[offset], packetSize);

        unsigned long t = millis();
        while(digitalRead(RF_AUX) == LOW){
            if(millis() - t > 500){
                Serial1.println("AUX Pini Veri Gönderim durumuna gecmedi....");
                return -1;
            }
            managedDelay(10);
        }

        SerialRF.write((uint8_t *)packet, sizeof(packet) / sizeof(packet[0]));

        managedDelay(10);
    }

    return 0;
}

int sendBroadcastDataPacket(uint8_t Channel, uint8_t *data, size_t size){
    sendFixedDataPacket(0x00, 0x00, Channel, data, size);

    return 0;
}

int sendTransparentDataPacket(uint8_t *data, size_t size){
    size_t totalPacket = (size + MAX_TX_BUFFER_SIZE - 1) / MAX_TX_BUFFER_SIZE;

    for(size_t i = 0; i < totalPacket; i++){
        size_t offset = i * MAX_TX_BUFFER_SIZE;
        size_t packetSize = (size - offset > MAX_TX_BUFFER_SIZE) ? MAX_TX_BUFFER_SIZE : (size - offset);

        uint8_t packet[packetSize];

        memcpy(packet, &data[offset], packetSize);

        unsigned long t = millis();
        while(digitalRead(RF_AUX) == LOW){
            if(millis() - t > 500){
                Serial1.println("AUX Pini Veri Gönderim durumuna gecmedi....");
                return -1;
            }
            managedDelay(10);
        }

        SerialRF.write((uint8_t *)packet, sizeof(packet) / sizeof(packet[0]));

        managedDelay(10);
    }

    return 0;
}

int setTransmissionMode(struct ConfigRF *config, uint8_t Mode){
    switch (Mode)
    {
    case TRANSPARENTMODE:
        config->RFOption.TransmissionMode = TRANSPARENTMODE;
        break;
    
    case FIXEDMODE:
        config->RFOption.TransmissionMode = FIXEDMODE;
        break;

    case BROADCASTMODE:
        config->RFOption.TransmissionMode = FIXEDMODE;
        config->AddressHigh = 0x00;
        config->AddressLow = 0x00;
        break;

    default:
        Serial1.println("Gonderim Turu belirlemede yanlis mod girildi...");
        return -1;
    }

    return 0;
}

int setAddresses(struct ConfigRF *config, uint8_t AddHigh, uint8_t AddLow){
    if(config->RFOption.TransmissionMode == FIXEDMODE){
        config->AddressHigh = AddHigh;
        config->AddressLow = AddLow;
        return 0;
    }

    else{
        Serial1.println("Adres Ayarlamasi Yapilamadi. Cihaz Şeffaf Iletisim Modunda...");
        return -1;
    }
}

int setChannel(struct ConfigRF *config, uint8_t channel){
    if(config->RFOption.TransmissionMode == FIXEDMODE){
        config->Channel = channel;
        return 0;
    }

    else{
        Serial1.println("Kanal Ayarlamasi Yapilamadi. Cihaz Şeffaf Iletisim Modunda...");
        return -1;
    }
}

int setTransmissionPower(struct ConfigRF *config, uint8_t power){
    switch (power)
    {
    case TRANSMISSIONPOWER_30:
        config->RFOption.TransmissionPower = TRANSMISSIONPOWER_30;
        break;

    case TRANSMISSIONPOWER_27:
        config->RFOption.TransmissionPower = TRANSMISSIONPOWER_27;
        break;

    case TRANSMISSIONPOWER_24:
        config->RFOption.TransmissionPower = TRANSMISSIONPOWER_24;
        break;
    
    case TRANSMISSIONPOWER_21:
        config->RFOption.TransmissionPower = TRANSMISSIONPOWER_21;
        break;
    
    default:
        Serial1.println("Güc Ayarlama icin yanlis ayar girildi...");
        return -1;
    }

    return 0;
}

int setIODriver(struct ConfigRF *config, uint8_t driver){
    switch (driver)
    {
    case IO_PUSHPULL:
        config->RFOption.IODriver = IO_PUSHPULL;
        break;
    
    case IO_OPENDRAIN:
        config->RFOption.IODriver = IO_OPENDRAIN;
        break;
    
    default:
        Serial1.println("IO driver belirlemede yanlis mod girildi...");
        return -1;
    }

    return 0;
}

int setFECSettings(struct ConfigRF *config, uint8_t mode){
    switch (mode)
    {
    case FEC_OFF:
        config->RFOption.FECset = FEC_OFF;
        break;
    
    case FEC_ON:
        config->RFOption.FECset = FEC_ON;
        break;
    
    default:
        Serial1.println("FEC belirlemede yanlis mod girildi...");
        return -1;
    }

    return 0;
}

int setWirelesWakeup(struct ConfigRF *config, uint8_t time){
    switch (time)
    {
    case WIRELESSWAKEUP_250:
        config->RFOption.WirelessWakeUp = WIRELESSWAKEUP_250;
        break;

    case WIRELESSWAKEUP_500:
        config->RFOption.WirelessWakeUp = WIRELESSWAKEUP_500;
        break;
    
    case WIRELESSWAKEUP_750:
        config->RFOption.WirelessWakeUp = WIRELESSWAKEUP_750;
        break;

    case WIRELESSWAKEUP_1000:
        config->RFOption.WirelessWakeUp = WIRELESSWAKEUP_1000;
        break;

    case WIRELESSWAKEUP_1250:
        config->RFOption.WirelessWakeUp = WIRELESSWAKEUP_1250;
        break;
    
    case WIRELESSWAKEUP_1500:
        config->RFOption.WirelessWakeUp = WIRELESSWAKEUP_1500;
        break;

    case WIRELESSWAKEUP_1750:
        config->RFOption.WirelessWakeUp = WIRELESSWAKEUP_1750;
        break;

    case WIRELESSWAKEUP_2000:
        config->RFOption.WirelessWakeUp = WIRELESSWAKEUP_2000;
        break;

    default:
        Serial1.println("Wireles Zaman belirlemede yanlis mod girildi...");
        return -1;
    }

    return 0;
}

int setUARTParity(struct ConfigRF *config, uint8_t paritybyte){
    switch (paritybyte)
    {
    case UARTPARITY_8N1:
        config->RFSped.UARTParity = UARTPARITY_8N1;
        break;
    
    case UARTPARITY_8O1:
        config->RFSped.UARTParity = UARTPARITY_8O1;
        break;

    case UARTPARITY_8E1:
        config->RFSped.UARTParity = UARTPARITY_8E1;
        break;
    
    default:
        Serial1.println("UART Parity belirlemede yanlis mod girildi...");
        return -1;
    }

    return 0;
}

int setUARTBaudRate(struct ConfigRF *config, uint8_t baudrate){
    switch (baudrate)
    {
    case UARTBAUDRATE_1200:
        config->RFSped.UARTBaud = UARTBAUDRATE_1200;
        break;
    
    case UARTBAUDRATE_2400:
        config->RFSped.UARTBaud = UARTBAUDRATE_2400;
        break;

    case UARTBAUDRATE_4800:
        config->RFSped.UARTBaud = UARTBAUDRATE_4800;
        break;
    
    case UARTBAUDRATE_9600:
        config->RFSped.UARTBaud = UARTBAUDRATE_9600;
        break;
    
    case UARTBAUDRATE_19200:
        config->RFSped.UARTBaud = UARTBAUDRATE_19200;
        break;

    case UARTBAUDRATE_38400:
        config->RFSped.UARTBaud = UARTBAUDRATE_38400;
        break;

    case UARTBAUDRATE_57600:
        config->RFSped.UARTBaud = UARTBAUDRATE_57600;
        break;

    case UARTBAUDRATE_115200:
        config->RFSped.UARTBaud = UARTBAUDRATE_115200;
        break;

    default:
        Serial1.println("UART Baud Rate belirlemede yanlis mod girildi...");
        return -1;
    }

    return 0;
}

int setAirDataRate(struct ConfigRF *config, uint8_t airdatarate){
    switch (airdatarate)
    {
    case AIRDATARATE_03k:
        config->RFSped.AirDataRate = AIRDATARATE_03k;
        break;
    
    case AIRDATARATE_12k:
        config->RFSped.AirDataRate = AIRDATARATE_12k;
        break;

    case AIRDATARATE_24k:
        config->RFSped.AirDataRate = AIRDATARATE_24k;
        break;
    
    case AIRDATARATE_48k:
        config->RFSped.AirDataRate = AIRDATARATE_48k;
        break;
    
    case AIRDATARATE_96k:
        config->RFSped.AirDataRate = AIRDATARATE_96k;
        break;

    case AIRDATARATE_192k:
        config->RFSped.AirDataRate = AIRDATARATE_192k;
        break;

    default:
        Serial1.println("Air Data Rate belirlemede yanlis mod girildi...");
        return -1;
    }
}