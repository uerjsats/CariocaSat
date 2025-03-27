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

#define TX_OUTPUT_POWER 5 // dBm

#define LORA_BANDWIDTH 0        // [0: 125 kHz,
                                //  1: 250 kHz,
                                //  2: 500 kHz,
                                //  3: Reserved]
#define LORA_SPREADING_FACTOR 7 // [SF7..SF12]
#define LORA_CODINGRATE 1       // [1: 4/5,
                                //  2: 4/6,
                                //  3: 4/7,
                                //  4: 4/8]
#define LORA_PREAMBLE_LENGTH 8  // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0   // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

#define RX_TIMEOUT_VALUE 1000
#define BUFFER_SIZE 44 // Define the payload size here

char txpacket[BUFFER_SIZE];

double txNumber;

bool lora_idle = true;

static RadioEvents_t RadioEvents;
void OnTxDone(void);
void OnTxTimeout(void);

// Criar instância do GPS e da Serial2
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

#define SEALEVELPRESSURE_HPA (1010)
#define DHTPIN 48          // Pino onde o DHT22 está conectado
#define DHTTYPE DHT22      // Define o tipo do sensor
#define TIMEZONE_OFFSET -3 // Ajuste para horário de Brasília (UTC-3)

Adafruit_BME280 bme;      // Objeto do sensor BME280
DHT dht(DHTPIN, DHTTYPE); // Objeto do sensor DHT22
Adafruit_MPU6050 mpu;     // Objeto do sensor MPU6050 (GY-521)

unsigned long startTime;

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
        Wire.write((int)(eeaddress >> 8));   // MSB
        Wire.write((int)(eeaddress & 0xFF)); // LSB

        byte bytesToWrite = min(data_len, (unsigned int)16); // Limita a 16 bytes por página
        for (byte i = 0; i < bytesToWrite; i++) {
            Wire.write(data[i]);
        }

        Wire.endTransmission();
        eeaddress += bytesToWrite;
        data += bytesToWrite;
        data_len -= bytesToWrite;

        delay(5); // Tempo de gravação da EEPROM
    }
}

void EE24CXXX::read(unsigned int eeaddress, unsigned char *data, unsigned int data_len) {
    while (data_len > 0) {
        Wire.beginTransmission(_device_address);
        Wire.write((int)(eeaddress >> 8));   // MSB
        Wire.write((int)(eeaddress & 0xFF)); // LSB
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
    int sats;

    float accelX;
    float accelY;
    float accelZ;
};

void setup() {
    Serial.begin(115200);
    gpsSerial.begin(9600, SERIAL_8N1, 45, 46); // RX=45, TX=46

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
    RadioEvents.TxTimeout = OnTxTimeout;

    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);
    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                      true, 0, 0, LORA_IQ_INVERSION_ON, 3000);
}

void loop() {
    while (gpsSerial.available()) {
        char c = gpsSerial.read();
        gps.encode(c);
    }

    // Leitura dos sensores e armazenamento na struct dados
    sensorsData dados;
    unsigned long elapsedTime = (millis() - startTime) / 1000;
    dados.seconds = elapsedTime;

    dados.temperatureDHT = dht.readTemperature();
    if(isnan(dados.temperatureDHT)) {dados.temperatureDHT = 0.0;}

    dados.humidityDHT = dht.readHumidity();
    if(isnan(dados.humidityDHT)) {dados.humidityDHT = 0.0;}

    dados.pressure = bme.readPressure() / 100.0F;
    dados.altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);

    dados.latitude = gps.location.lat();
    dados.longitude = gps.location.lng();
    dados.sats = gps.satellites.value();

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    dados.accelX = a.acceleration.x;
    dados.accelY = a.acceleration.y;
    dados.accelZ = a.acceleration.z;

    printSensorsData(dados);

    if (currentAddress >= 1484) {
        currentAddress = 0; // Reseta após atingir a capacidade máxima
    }

    if (dados.altitude >= 21.0) {
        eeprom.write(currentAddress, dados.seconds);
        currentAddress += 4;
        eeprom.write(currentAddress, dados.temperatureDHT);
        currentAddress += 4;
        eeprom.write(currentAddress, dados.humidityDHT);
        currentAddress += 4;
        eeprom.write(currentAddress, dados.pressure);
        currentAddress += 4;
        eeprom.write(currentAddress, dados.altitude);
        currentAddress += 4;
        eeprom.write(currentAddress, dados.latitude);
        currentAddress += 4;
        eeprom.write(currentAddress, dados.longitude);
        currentAddress += 4;
        eeprom.write(currentAddress, dados.sats);
        currentAddress += 4;
        eeprom.write(currentAddress, dados.accelX);
        currentAddress += 4;
        eeprom.write(currentAddress, dados.accelY);
        currentAddress += 4;
        eeprom.write(currentAddress, dados.accelZ);
        currentAddress += 4;

        Serial.print("Gravando na EEPROM: ");
        Serial.println(dados.altitude, 2);
    }
    else {
        Serial.print("Não está gravando na EEPROM: ");
        Serial.println(dados.altitude, 2);
    }

    if (lora_idle == true) {
        delay(1000);
        txNumber += 0.01;
        sprintf(txpacket, "%.2f:%.2f:%.2f:%.2f:%.6f:%.6f:%.2f:%.2f:%.2f:%0.2f", dados.seconds,
                dados.temperatureDHT, dados.humidityDHT, dados.altitude,
                dados.latitude, dados.longitude,
                dados.accelX, dados.accelY, dados.accelZ, txNumber);

        Radio.Send((uint8_t *)txpacket, strlen(txpacket)); // send the package out
        lora_idle = false;
    }
    Radio.IrqProcess();

    delay(2000); // Aguarda 2 segundos antes da próxima leitura
}

void OnTxDone() {
    Serial.println("TX done......");
    lora_idle = true;
}

void OnTxTimeout() {
    Radio.Sleep();
    Serial.println("TX Timeout......");
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
    Serial.print(dados.accelY); Serial.print(":");
    Serial.println(dados.accelZ);
}
