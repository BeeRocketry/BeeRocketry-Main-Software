#define DEBUG_MODE

#include "rf.h"

/*
---------------------------------------------------------------
 Cyclic Redundancy Check Fonksiyonu
    Alınan veri paketini x^8 + x^2 + x + 1 polinomunu
    kullanarak kendi algoritması ile kontrol byte'ı
    oluşturur.

    Gönderilecek veriye crc8 kontrol byte'ı ek bir byte
    olarak eklenir. Alıcı ise veri paketindeki verileri
    işleyerek crc8 verisini elde eder ve gönderilendeki
    crc8 kontrol byte'ı ile karşılaştırır. Eğer uyuşmaz
    ise veri paketi bozuldu demektir.

    Parameter
        *data --> veri paketi array'i
        length --> veri paketinde crc kontrol byte'ı oluşturmak için
                   işlenecek veri sayısı (örnek olarak 1'i crc byte'ı
                   olarak kullanılmak üzere 8 byte'lık bir array
                   yolladıysak length 7 olmalıdır.)

    Returns
        crc --> işlenen veri paketinden oluşturulan crc kontrol byte'ı
---------------------------------------------------------------
*/
uint8_t calculateCRC8(const uint8_t *data, size_t length){
    if(sizeof(data) + 1 > MAX_TX_BUFFER_SIZE){
        DEBUG_PRINTLN(F("CRC8 Fonksiyonu Maksimum Paketten Büyük"));
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

/*
------------------------------------------------
 AUX Durum Bekleme Fonksiyonu
    RF cihazı müsait olduğu durumlarda AUX pininden HIGH
    sinyal yollamaktadır. Bu fonksiyon cihazın müsait
    olmasını beklemektedir. Eğer parametre olarak alınan
    timeout süresi aşılır ise zaman aşımı döndürür.

    Parameter
        timeout --> RF cihazın müsait duruma gelmesi için beklenecek
                    maksimum süre

    Returns
        E32_Timeout --> Zaman Aşımı
        E32_Success --> Müsait Durum
------------------------------------------------
*/
Status waitAUX(unsigned long timeout){
    long startTime = millis();
    while(digitalRead(RF_AUX) == LOW){
        if (millis() - startTime > timeout) {
            DEBUG_PRINTLN(F("RF_AUX zaman aşimina uğradi."));
            return E32_Timeout;
        }
        managedDelay(20);
    }

    return E32_Timeout;
}

/*
------------------------
 RF Cihazı Başlangıç Ayarları Fonksiyonu
    RF cihazının pin mod ayarlamaları, seri port ayarlamaları ve
    mod ayarlamalarının yapıldığı başlangıç fonksiyonudur.

    Parameter
        confs --> Cihaz ayarlamalarını içeren struct yapısı
    
    Returns
        E32_Timeout --> Zaman Aşımı
        E32_Success --> İşlem Başarılı
------------------------
*/
Status RFBegin(RF_UART_PARITY parity = UARTPARITY_8N1, RF_UART_BAUD baud = UARTBAUDRATE_9600, RF_AIR_DATA airdata = AIRDATARATE_03k, RF_TRANS_MODE transmode = FIXEDMODE, RF_IO_MODE IOmode = IO_PUSHPULL, RF_WIRELESS wirelesswake = WIRELESSWAKEUP_250, RF_FEC fecmode = FEC_ON, RF_TRANS_POWER transpower = TRANSMISSIONPOWER_30){
    struct ConfigRF confs, getConfs;
    setFECSettings(&confs, fecmode);
    setIODriver(&confs, IOmode);
    setTransmissionMode(&confs, transmode);
    setTransmissionPower(&confs, transpower);
    setWirelesWakeup(&confs, wirelesswake);
    setAirDataRate(&confs, airdata);
    setUARTBaudRate(&confs, baud);
    setUARTParity(&confs, parity);

    pinMode(RF_AUX, INPUT);
    pinMode(RF_M0, OUTPUT);
    pinMode(RF_M1, OUTPUT);

    if(setSerialBaudRateBegin(confs) == E32_FailureMode){
        return E32_FailureMode;
    }

    digitalWrite(RF_M0, LOW);
    digitalWrite(RF_M1, LOW);

    if(waitAUX(TIMEOUT_AUX_RESPOND) == E32_Timeout){
        return E32_Timeout;
    }

    Status res = setSettings(confs);
    if(res == E32_Timeout){
        return E32_Timeout;
    }
    else if(res == E32_Success){
        DEBUG_PRINTLN(F("Mod Ayarlamalari Basarili bir sekilde yapildi..."));
    }

    managedDelay(20);

    getSettings(&getConfs);

    DEBUG_PRINTLN(F("RF Begin Fonksiyonu basarili bir sekilde tamamlandi..."));
    return E32_Success;
}

int8_t setSerialParityBegin(struct ConfigRF confs){
    switch (confs.RFSped.UARTBaud)
    {
    case UARTPARITY_8N1:
        return SERIAL_8N1;
        break;

    case UARTPARITY_8O1:
        return SERIAL_8O1;
        break;

    case UARTPARITY_8E1:
        return SERIAL_8E1;
        break;

    default:
        DEBUG_PRINTLN(F("Baslangic Ayarlari yapilirken yanlis uartparity girildi..."));
        DEBUG_PRINTLN(F("Otomatik olarak SERIAL_8N1 ayarina gecildi..."));
        return SERIAL_8N1;
    }
}

Status setSerialBaudRateBegin(struct ConfigRF confs){
    switch (confs.RFSped.UARTBaud)
    {
    case UARTBAUDRATE_1200:
        SerialRF.begin(1200, setSerialParityBegin(confs));
        break;

    case UARTBAUDRATE_2400:
        SerialRF.begin(2400, setSerialParityBegin(confs));
        break;

    case UARTBAUDRATE_4800:
        SerialRF.begin(4800, setSerialParityBegin(confs));
        break;

    case UARTBAUDRATE_9600:
        SerialRF.begin(9600, setSerialParityBegin(confs));
        break;
    
    case UARTBAUDRATE_19200:
        SerialRF.begin(19200, setSerialParityBegin(confs));
        break;

    case UARTBAUDRATE_38400:
        SerialRF.begin(38400, setSerialParityBegin(confs));
        break;

    case UARTBAUDRATE_57600:
        SerialRF.begin(57600, setSerialParityBegin(confs));
        break;

    case UARTBAUDRATE_115200:
        SerialRF.begin(115200, setSerialParityBegin(confs));
        break;

    default:
        DEBUG_PRINTLN(F("Baslangic Ayarlari yapilirken yanlis uartbaudrate girildi..."));
        return E32_FailureMode;
    }

    return E32_Success;
}

Status setSettings(struct ConfigRF confs){
    uint8_t SpedByte = 0, OptionByte = 0;
    uint8_t MesArr[6];

    SpedByte = (confs.RFSped.UARTParity << 6) | (confs.RFSped.UARTBaud << 3) | (confs.RFSped.AirDataRate);
    OptionByte = (confs.RFOption.TransmissionMode << 7) | (confs.RFOption.IODriver << 6) | (confs.RFOption.WirelessWakeUp << 3) | (confs.RFOption.FECset << 2) | (confs.RFOption.TransmissionPower);

    if(waitAUX(TIMEOUT_AUX_RESPOND) == E32_Timeout){
        return E32_Timeout;
    }

    digitalWrite(RF_M0, HIGH);
    digitalWrite(RF_M1, HIGH);

    managedDelay(20);

    MesArr[0] = 0xC0;
    MesArr[1] = confs.AddressHigh;
    MesArr[2] = confs.AddressLow;
    MesArr[3] = SpedByte;
    MesArr[4] = confs.Channel;
    MesArr[5] = OptionByte;

    SerialRF.write((uint8_t *)MesArr, sizeof(MesArr) / sizeof(MesArr[0]));

    if(waitAUX(TIMEOUT_AUX_RESPOND) == E32_Timeout){
        return E32_Timeout;
    }

    managedDelay(750);

    digitalWrite(RF_M0, LOW);
    digitalWrite(RF_M1, LOW);

    managedDelay(20);

    return E32_Success;
}

Status getSettings(struct ConfigRF *confs){
    uint8_t SpedByte = 0, OptionByte = 0;
    uint8_t MesArr[6], sendpack[3];
    uint8_t receivedCRC;

    if(waitAUX(TIMEOUT_AUX_RESPOND) == E32_Timeout){
        return E32_Timeout;
    }

    digitalWrite(RF_M0, HIGH);
    digitalWrite(RF_M1, HIGH);

    managedDelay(40);

    sendpack[0] = 0xC1;
    sendpack[1] = 0xC1;
    sendpack[2] = 0xC1;

    SerialRF.write((uint8_t *)sendpack, sizeof(sendpack) / sizeof(sendpack[0]));
    
    long startTime = millis();
    while(SerialRF.available() < sizeof(MesArr)){
        if(millis() - startTime > 1000){
            DEBUG_PRINTLN(F("Veri okuma zaman aşimina uğradi."));
            return E32_Timeout;
        }
        managedDelay(20);
    }

    SerialRF.readBytes(MesArr, sizeof(MesArr));

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

    DEBUG_PRINTLN(F("------------------------------------------------------"));
    DEBUG_PRINT(F("Yüksek Adres: "));    DEBUG_PRINTLN(confs->AddressHigh);

    DEBUG_PRINT(F("Düşük Adres: "));    DEBUG_PRINTLN(confs->AddressLow);

    DEBUG_PRINT(F("Kanal: "));    DEBUG_PRINT(confs->Channel);
    DEBUG_PRINT(F(" - "));    DEBUG_PRINT(410+confs->Channel);   DEBUG_PRINTLN(F(" MHz"));
    DEBUG_PRINTLN();

    DEBUG_PRINTLN(F("Sped Ayarlari"));
    DEBUG_PRINT(F("  UART Baud Rate: "));     DEBUG_PRINTLN(getUARTBaudRate(confs->RFSped.UARTBaud));
    DEBUG_PRINT(F(" UART Parity: "));     DEBUG_PRINTLN(getUARTParity(confs->RFSped.UARTParity));
    DEBUG_PRINT(F("  Air Data Rate: "));     DEBUG_PRINTLN(getAirData(confs->RFSped.UARTParity));
    DEBUG_PRINTLN();

    DEBUG_PRINTLN(F("Option Ayarlari"));
    DEBUG_PRINT(F("  Transfer Türü: "));      DEBUG_PRINTLN(getTransmissionType(confs->RFOption.TransmissionMode));
    DEBUG_PRINT(F("  IO Türü: "));        DEBUG_PRINTLN(getIOMode(confs->RFOption.IODriver));
    DEBUG_PRINT(F("  Wireless Uyanma Süresi: "));     DEBUG_PRINTLN(getWirelessWakeup(confs->RFOption.WirelessWakeUp));
    DEBUG_PRINT(F("  FEC Filtresi: "));       DEBUG_PRINTLN(getFECFilter(confs->RFOption.FECset));
    DEBUG_PRINT(F("  Aktarim Gücü: "));       DEBUG_PRINTLN(getTranmissionPower(confs->RFOption.TransmissionPower));
    DEBUG_PRINTLN();
    DEBUG_PRINTLN(F("------------------------------------------------------"));

    managedDelay(750);

    digitalWrite(RF_M0, LOW);
    digitalWrite(RF_M1, LOW);

    managedDelay(20);

    return E32_Success;
}

Status receiveSingleData(uint8_t *data){
    if(waitAUX(TIMEOUT_AUX_RESPOND) == E32_Timeout){
        return E32_Timeout;
    }

    unsigned long t = millis();
    while(SerialRF.available() == 0){
        if(millis() - t > 1000){
            DEBUG_PRINTLN(F("Veri Okuma Zaman Asimina Ugradi..."));
            return E32_Timeout;
        }
        managedDelay(20);
    }

    *data = SerialRF.read();
    DEBUG_PRINTLN(F("Veri Alindi..."));
    return E32_Success;
}

Status receiveDataPacket(uint8_t *data, size_t size){
    if(waitAUX(TIMEOUT_AUX_RESPOND) == E32_Timeout){
        return E32_Timeout;
    }

    unsigned long t = millis();
    while(SerialRF.available() < size){
        if(SerialRF.available() == 0 && millis() - t > 100){
            DEBUG_PRINTLN(F("Herhangi bir Veri Paketi gelmedi..."));
            return E32_NoMessage;
        }
        else if(millis() - t > 1000){
            DEBUG_PRINTLN(F("Veri okuma zaman asimina ugradi..."));
            return E32_Timeout;
        }
        managedDelay(20);
    }

    SerialRF.readBytes(data, size);
    return E32_Success;
}

Status sendFixedSingleData(uint8_t AddressHigh, uint8_t AddressLow, uint8_t Channel, uint8_t data){
    uint8_t packet[4];
    packet[0] = AddressHigh;
    packet[1] = AddressLow;
    packet[2] = Channel;
    packet[3] = data;

    if(waitAUX(TIMEOUT_AUX_RESPOND) == E32_Timeout){
        return E32_Timeout;
    }

    SerialRF.write((uint8_t *)packet, sizeof(packet) / sizeof(packet[0]));

    return E32_Success;
}

Status sendTransparentSingleData(uint8_t data){
    if(waitAUX(TIMEOUT_AUX_RESPOND) == E32_Timeout){
        return E32_Timeout;
    }

    SerialRF.write(data);

    return E32_Success;
}

Status sendFixedDataPacket(uint8_t AddressHigh, uint8_t AddressLow, uint8_t Channel, uint8_t *data, size_t size){
    size_t totalPacket = (size + MAX_TX_BUFFER_SIZE - 1) / MAX_TX_BUFFER_SIZE;

    for(size_t i = 0; i < totalPacket; i++){
        size_t offset = i * MAX_TX_BUFFER_SIZE;
        size_t packetSize = (size - offset > MAX_TX_BUFFER_SIZE) ? MAX_TX_BUFFER_SIZE : (size - offset);

        uint8_t packet[packetSize + 3];

        packet[0] = AddressHigh;
        packet[1] = AddressLow;
        packet[2] = Channel;
        memcpy(&packet[3], &data[offset], packetSize);

        if(waitAUX(TIMEOUT_AUX_RESPOND) == E32_Timeout){
            return E32_Timeout;
        }

        SerialRF.write((uint8_t *)packet, sizeof(packet) / sizeof(packet[0]));

        managedDelay(10);
    }

    return E32_Success;
}

Status sendBroadcastDataPacket(uint8_t Channel, uint8_t *data, size_t size){
    return sendFixedDataPacket(0x00, 0x00, Channel, data, size);
}

Status sendTransparentDataPacket(uint8_t *data, size_t size){
    size_t totalPacket = (size + MAX_TX_BUFFER_SIZE - 1) / MAX_TX_BUFFER_SIZE;

    for(size_t i = 0; i < totalPacket; i++){
        size_t offset = i * MAX_TX_BUFFER_SIZE;
        size_t packetSize = (size - offset > MAX_TX_BUFFER_SIZE) ? MAX_TX_BUFFER_SIZE : (size - offset);

        uint8_t packet[packetSize];

        memcpy(packet, &data[offset], packetSize);

        if(waitAUX(TIMEOUT_AUX_RESPOND) == E32_Timeout){
            return E32_Timeout;
        }

        SerialRF.write((uint8_t *)packet, sizeof(packet) / sizeof(packet[0]));

        managedDelay(10);
    }

    return E32_Success;
}

Status setTransmissionMode(struct ConfigRF *config, uint8_t Mode){
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
        DEBUG_PRINTLN(F("Gonderim Turu belirlemede yanlis mod girildi..."));
        return E32_FailureMode;
    }

    return E32_Success;
}

Status setAddresses(struct ConfigRF *config, uint8_t AddHigh, uint8_t AddLow){
    if(config->RFOption.TransmissionMode == FIXEDMODE){
        config->AddressHigh = AddHigh;
        config->AddressLow = AddLow;
        return E32_Success;
    }

    else{
        DEBUG_PRINTLN(F("Adres Ayarlamasi Yapilamadi. Cihaz Şeffaf Iletisim Modunda..."));
        config->AddressHigh = 0x00;
        config->AddressLow = 0x00;
        return E32_FailureMode;
    }
}

Status setChannel(struct ConfigRF *config, uint8_t channel){
    if(config->RFOption.TransmissionMode == FIXEDMODE){
        config->Channel = channel;
        return E32_Success;
    }

    else{
        DEBUG_PRINTLN(F("Kanal Ayarlamasi Yapilamadi. Cihaz Şeffaf Iletisim Modunda..."));
        config->Channel = 0x17;
        return E32_FailureMode;
    }
}

Status setTransmissionPower(struct ConfigRF *config, uint8_t power){
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
        DEBUG_PRINTLN(F("Güc Ayarlama icin yanlis ayar girildi..."));
        return E32_FailureMode;
    }

    return E32_Success;
}

Status setIODriver(struct ConfigRF *config, uint8_t driver){
    switch (driver)
    {
    case IO_PUSHPULL:
        config->RFOption.IODriver = IO_PUSHPULL;
        break;
    
    case IO_OPENDRAIN:
        config->RFOption.IODriver = IO_OPENDRAIN;
        break;
    
    default:
        DEBUG_PRINTLN(F("IO driver belirlemede yanlis mod girildi..."));
        return E32_FailureMode;
    }

    return E32_Success;
}

Status setFECSettings(struct ConfigRF *config, uint8_t mode){
    switch (mode)
    {
    case FEC_OFF:
        config->RFOption.FECset = FEC_OFF;
        break;
    
    case FEC_ON:
        config->RFOption.FECset = FEC_ON;
        break;
    
    default:
        DEBUG_PRINTLN(F("FEC belirlemede yanlis mod girildi..."));
        return E32_FailureMode;
    }

    return E32_Success;
}

Status setWirelesWakeup(struct ConfigRF *config, uint8_t time){
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
        DEBUG_PRINTLN(F("Wireles Zaman belirlemede yanlis mod girildi..."));
        return E32_FailureMode;
    }

    return E32_Success;
}

Status setUARTParity(struct ConfigRF *config, uint8_t paritybyte){
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
        DEBUG_PRINTLN(F("UART Parity belirlemede yanlis mod girildi..."));
        return E32_FailureMode;
    }

    return E32_Success;
}

Status setUARTBaudRate(struct ConfigRF *config, uint8_t baudrate){
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
        DEBUG_PRINTLN(F("UART Baud Rate belirlemede yanlis mod girildi..."));
        return E32_FailureMode;
    }

    return E32_Success;
}

Status setAirDataRate(struct ConfigRF *config, uint8_t airdatarate){
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
        DEBUG_PRINTLN(F("Air Data Rate belirlemede yanlis mod girildi..."));
        return E32_FailureMode;
    }

    return E32_Success;
}