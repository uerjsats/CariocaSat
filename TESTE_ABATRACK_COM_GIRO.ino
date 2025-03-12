#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <DHT.h>
#include <Adafruit_MPU6050.h>

// Criar instância do GPS e da Serial2
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

#define SEALEVELPRESSURE_HPA (1010)
#define DHTPIN 48        // Pino onde o DHT22 está conectado
#define DHTTYPE DHT22   // Define o tipo do sensor
#define TIMEZONE_OFFSET -3  // Ajuste para horário de Brasília (UTC-3)

Adafruit_BME280 bme; // Objeto do sensor BME280
DHT dht(DHTPIN, DHTTYPE); // Objeto do sensor DHT22
Adafruit_MPU6050 mpu; // Objeto do sensor MPU6050 (GY-521)

void setup() {
    Serial.begin(115200);  // Comunicação com o PC
    gpsSerial.begin(9600, SERIAL_8N1, 45, 46);  // RX=45, TX=46

    // Inicialização dos sensores
    bme.begin(0x76); // Endereço padrão I2C do BME280
    dht.begin(); // Inicializa o sensor DHT22
    Wire.begin();
    mpu.begin(); // Inicializa o sensor MPU6050
}

void loop() {
    // Lê os dados do GPS
    while (gpsSerial.available()) {
        char c = gpsSerial.read();
        gps.encode(c);
    }

    // Coletando dados
    float tempBME = bme.readTemperature();
    float pressao = bme.readPressure() / 100.0F;
    float altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
    float umidadeBME = bme.readHumidity();
    
    float tempDHT = dht.readTemperature();
    float humDHT = dht.readHumidity();

    // Verifica se os dados do DHT são válidos
    if (isnan(tempDHT) || isnan(humDHT)) {
        tempDHT = 0.0;
        humDHT = 0.0;
    }

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    Serial.print(tempBME); 
    Serial.print(":");
    Serial.print(gps.time.second());
    Serial.print(":");
    Serial.print(pressao);
    Serial.print(":");
    Serial.print(altitude);
    Serial.print(":");
    Serial.print(gps.location.lat(), 6);
    Serial.print(":");
    Serial.print(gps.location.lng(), 6);
    Serial.print(":");
    Serial.print(gps.satellites.value());
    Serial.print(":");
    Serial.print(a.acceleration.x);
    Serial.print(":");
    Serial.print(a.acceleration.y);
    Serial.print(":");
    Serial.print(a.acceleration.z);
    Serial.print(":8");
    Serial.print(":9");
    Serial.println(":89");


    delay(2000); // Aguarda 2 segundos antes da próxima leitura
}