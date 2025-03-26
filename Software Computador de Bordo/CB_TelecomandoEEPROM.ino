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
#define LORA_BANDWIDTH 0
#define LORA_SPREADING_FACTOR 7
#define LORA_CODINGRATE 1
#define LORA_PREAMBLE_LENGTH 8
#define LORA_SYMBOL_TIMEOUT 0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define RX_TIMEOUT_VALUE 1000
#define BUFFER_SIZE 44 // Define the payload size

char txpacket[BUFFER_SIZE];
double txNumber;
bool lora_idle = true;
bool recording = false; // Flag para controle de gravação

static RadioEvents_t RadioEvents;
void OnTxDone(void);
void OnTxTimeout(void);

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
    float roll;
    float pitch;
};

void setup() {
    Serial.begin(115200);
    gpsSerial.begin(9600, SERIAL_8N1, 45, 46);
    if (!bme.begin(0x76)) Serial.println("Erro ao inicializar o BME280!");
    dht.begin();
    Wire.begin();
    if (!mpu.begin()) Serial.println("Erro ao inicializar o MPU6050!");
    startTime = 0;
    Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
    txNumber = 0;
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);
    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE, LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON, true, 0, 0, LORA_IQ_INVERSION_ON, 3000);
}

void loop() {
    while (gpsSerial.available()) gps.encode(gpsSerial.read());
    
    if (Serial.available()) {
        char command = Serial.read();
        if (command == '1') {
            recording = true;
            Serial.println("Iniciando gravação na EEPROM...");
        }
    }
    
    sensorsData dados;
    unsigned long elapsedTime = (millis() - startTime) / 1000;
    dados.seconds = elapsedTime;
    dados.temperatureDHT = dht.readTemperature();
    dados.humidityDHT = dht.readHumidity();
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
    dados.roll = atan2(dados.accelY, dados.accelZ) * 180.0 / PI;
    dados.pitch = atan2(-dados.accelX, sqrt(dados.accelY * dados.accelY + dados.accelZ * dados.accelZ)) * 180.0 / PI;
    
    if (recording) {
        if (currentAddress >= 1484) currentAddress = 0;
        eeprom.write(currentAddress, dados);
        currentAddress += sizeof(dados);
    }

    if (lora_idle) {
        delay(1000);
        txNumber += 0.01;
        sprintf(txpacket, "%.2f:%.2f:%.2f:%.6f:%.6f:%.2f:%.2f:%.2f:%.2f:%.2f:%.2f", dados.temperatureDHT, dados.humidityDHT, dados.altitude, dados.latitude, dados.longitude, dados.accelX, dados.accelY, dados.accelZ, dados.roll, dados.pitch, txNumber);
        Radio.Send((uint8_t *)txpacket, strlen(txpacket));
        lora_idle = false;
    }
    Radio.IrqProcess();
    delay(2000);
}

void OnTxDone() {
    Serial.println("TX done...");
    lora_idle = true;
}

void OnTxTimeout() {
    Radio.Sleep();
    Serial.println("TX Timeout...");
    lora_idle = true;
}
