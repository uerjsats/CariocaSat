#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <DHT.h>
#include <Adafruit_MPU6050.h>
#include "LoRaWan_APP.h"
#include "Arduino.h"

#define RF_FREQUENCY 908000000 // Hz
#define TX_OUTPUT_POWER 5

#define LORA_BANDWIDTH 0
#define LORA_SPREADING_FACTOR 7
#define LORA_CODINGRATE 1
#define LORA_PREAMBLE_LENGTH 8
#define LORA_SYMBOL_TIMEOUT 0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

#define RX_TIMEOUT_VALUE 1000
#define BUFFER_SIZE 64

#define MY_ADDRESS 43 //Endereço do rádio do satélite
#define DEST_ADDRESS 42 //Endereço da estação base

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

double txNumber;
bool lora_idle = true;

static RadioEvents_t RadioEvents;
void OnTxDone(void);
void OnTxTimeout(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssiValue, int8_t snr);  

TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

#define SEALEVELPRESSURE_HPA (1010)
#define DHTPIN 48
#define DHTTYPE DHT22
#define TIMEZONE_OFFSET -3

Adafruit_BME280 bme;
DHT dht(DHTPIN, DHTTYPE);
Adafruit_MPU6050 mpu;

unsigned long startTime;
unsigned long lastTxTime = 0;
const unsigned long txInterval = 5000; // envia a cada 5s

class EE24CXXX {
private:
    byte _device_address;

public:
    EE24CXXX(byte device_address) : _device_address(device_address) {}
    void write(unsigned int eeaddress, unsigned char *data, unsigned int data_len);
    void read(unsigned int eeaddress, unsigned char *data, unsigned int data_len);

    template <class T>
    int write(unsigned int eeaddress, const T &value);
    template <class T>
    int read(unsigned int eeaddress, T &value);
};

void EE24CXXX::write(unsigned int eeaddress, unsigned char *data, unsigned int data_len) {
    while (data_len > 0) {
        Wire.beginTransmission(_device_address);
        Wire.write((int)(eeaddress >> 8));
        Wire.write((int)(eeaddress & 0xFF));

        byte bytesToWrite = min(data_len, (unsigned int)16);
        for (byte i = 0; i < bytesToWrite; i++) {
            Wire.write(data[i]);
        }

        Wire.endTransmission();
        eeaddress += bytesToWrite;
        data += bytesToWrite;
        data_len -= bytesToWrite;

        delay(5);
    }
}

void EE24CXXX::read(unsigned int eeaddress, unsigned char *data, unsigned int data_len) {
    while (data_len > 0) {
        Wire.beginTransmission(_device_address);
        Wire.write((int)(eeaddress >> 8));
        Wire.write((int)(eeaddress & 0xFF));
        Wire.endTransmission();

        byte bytesToRead = min(data_len, (unsigned int)28);
        Wire.requestFrom(_device_address, bytesToRead);

        for (byte i = 0; i < bytesToRead && Wire.available(); i++) {
            data[i] = Wire.read();
        }

        data += bytesToRead;
        eeaddress += bytesToRead;
        data_len -= bytesToRead;
    }
}

template <class T> int EE24CXXX::write(unsigned int eeaddress, const T &value) {
    write(eeaddress, (unsigned char *)&value, sizeof(T));
    return sizeof(T);
}

template <class T> int EE24CXXX::read(unsigned int eeaddress, T &value) {
    read(eeaddress, (unsigned char *)&value, sizeof(T));
    return sizeof(T);
}

EE24CXXX eeprom(0x50);
int currentAddress = 0;

struct sensorsData {
    int seconds;
    float temperatureDHT;
    float humidityDHT;
    float pressure;
    float altitude;
    float latitude;
    float longitude;
    float accelX;
    float accelY;
    float accelZ;
};

void setup() {
    Serial.begin(115200);
    gpsSerial.begin(9600, SERIAL_8N1, 45, 46);

    if (!bme.begin(0x76)) {
        Serial.println("Erro ao inicializar o BME280!");
    }

    dht.begin();
    Wire.begin();

    if (!mpu.begin()) {
        Serial.println("Erro ao inicializar o MPU6050!");
    }

    startTime = millis();

    Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

    txNumber = 0;

    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;

    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);
    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                      true, 0, 0, LORA_IQ_INVERSION_ON, 3000);
    Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                      LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                      LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                      0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

    Radio.Rx(0);

}

void loop() {
    while (gpsSerial.available()) {
        char c = gpsSerial.read();
        gps.encode(c);
    }

    Radio.IrqProcess();
    
    //Contador 
    if (millis() - lastTxTime >= txInterval && lora_idle) {
        sensorsData dados;
        unsigned long elapsedTime = (millis() - startTime) / 1000;
        dados.seconds = elapsedTime;
        
        //Dados DHT
        dados.temperatureDHT = dht.readTemperature();
        if (isnan(dados.temperatureDHT)) dados.temperatureDHT = 0.0;
        dados.humidityDHT = dht.readHumidity();
        if (isnan(dados.humidityDHT)) dados.humidityDHT = 0.0;
        
        //Dados do BME
        dados.pressure = bme.readPressure() / 100.0F;
        dados.altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
        
        //Dados GPS
        dados.latitude = gps.location.lat();
        dados.longitude = gps.location.lng();
        dados.sats = gps.satellites.value();

        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        
        //Dados GY
        dados.accelX = a.acceleration.x;
        dados.accelY = a.acceleration.y;
        dados.accelZ = a.acceleration.z;

        printSensorsData(dados);
        
        //Escrita na EEPROM
        if (currentAddress >= 1484) currentAddress = 0;

        eeprom.write(currentAddress, dados.seconds); currentAddress += 4;
        eeprom.write(currentAddress, dados.temperatureDHT); currentAddress += 4;
        eeprom.write(currentAddress, dados.humidityDHT); currentAddress += 4;
        eeprom.write(currentAddress, dados.pressure); currentAddress += 4;
        eeprom.write(currentAddress, dados.altitude); currentAddress += 4;
        eeprom.write(currentAddress, dados.latitude); currentAddress += 4;
        eeprom.write(currentAddress, dados.longitude); currentAddress += 4;
        eeprom.write(currentAddress, dados.accelX); currentAddress += 4;
        eeprom.write(currentAddress, dados.accelY); currentAddress += 4;
        eeprom.write(currentAddress, dados.accelZ); currentAddress += 4;

        memset(txpacket, 0, BUFFER_SIZE);
        txpacket[0] = MY_ADDRESS;
        txpacket[1] = DEST_ADDRESS;
        
        //Monta o pacote LoRa
        snprintf(&txpacket[2], BUFFER_SIZE - 2, "%.d:%.2f:%.2f:%.2f:%.2f:%.6f:%.6f:%.2f:%.2f:%.2f",
                 dados.seconds,
                 dados.temperatureDHT, dados.humidityDHT, dados.altitude, dados.pressure
                 dados.latitude, dados.longitude,
                 dados.accelX, dados.accelY, dados.accelZ);
        
        //Envia dados para estação base
        if (txpacket[1] == DEST_ADDRESS) {
            Radio.Send((uint8_t *)txpacket, strlen((char *)txpacket));
            Serial.printf("Enviado para %d: %s\n", DEST_ADDRESS, &txpacket[2]);
            lora_idle = false;
            lastTxTime = millis();
        } 
    }
}

void OnTxDone() {
    Serial.println("TX done......");
    lora_idle = true;
    Radio.Rx(0);
}

void OnTxTimeout() {
    Radio.Sleep();
    Serial.println("TX Timeout......");
    lora_idle = true;
    Radio.Rx(0);
}
//Recebe Telecomando
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssiValue, int8_t snr) {
    Serial.println("Chegou algo no RX!");
    if (size < 2) return;

    uint8_t sender = payload[0];
    uint8_t receiver = payload[1];

    Serial.printf("Recebido de %d para %d\n", sender, receiver);

    if (receiver != MY_ADDRESS || sender != DEST_ADDRESS) {
        lora_idle = true;
        Radio.Rx(0);
        return;
    }

    memset(rxpacket, 0, BUFFER_SIZE);
    memcpy(rxpacket, &payload[2], size - 2);
    rxpacket[size - 2] = '\0';
    Radio.Sleep();

    if (strcmp(rxpacket, "1") == 0) {
        Serial.println("foi");
    }

    lora_idle = true;
}

void printSensorsData(struct sensorsData dados) {
    Serial.print(dados.seconds); Serial.print(":");
    Serial.print(dados.temperatureDHT); Serial.print(":");
    Serial.print(dados.humidityDHT); Serial.print(":");
    Serial.print(dados.pressure); Serial.print(":");
    Serial.print(dados.altitude); Serial.print(":");
    Serial.print(dados.latitude); Serial.print(":");
    Serial.print(dados.longitude); Serial.print(":");
    Serial.print(dados.sats); Serial.print(":");
    Serial.print(dados.accelX); Serial.print(":");
    Serial.print(dados.accelY); Serial.println(":");
    Serial.println(dados.accelZ);
}
