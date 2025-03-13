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

unsigned long startTime;


class EE24CXXX {
  private:
    byte _device_address;
  public:
    EE24CXXX(byte device_address) : _device_address(device_address) {}
    void write(unsigned int eeaddress, unsigned char *data, unsigned int data_len);
    void read(unsigned int eeaddress, unsigned char *data, unsigned int data_len);

    template <class T> int write(unsigned int eeaddress, const T &value);
    template <class T> int read(unsigned int eeaddress, T &value);
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
int i = 0;

struct dadosVoo {
  float temperatura;
  float tempo;
  float umidade;
  float pressao;
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

}

void loop() {
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    gps.encode(c);
  }

  float tempBME = bme.readTemperature();
  float pressao = bme.readPressure() / 100.0F;
  float altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  float umidadeBME = bme.readHumidity();

  float tempDHT = dht.readTemperature();
  float humDHT = dht.readHumidity();

  if (isnan(tempDHT) || isnan(humDHT)) {
    tempDHT = 0.0;
    humDHT = 0.0;
  }

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long elapsedTime = (millis() - startTime) / 1000; // Converte milissegundos para segundos

  dadosVoo voo;
  voo.temperatura = tempBME;
  voo.umidade = humDHT;
  voo.tempo = elapsedTime;
  voo.pressao = pressao;
  voo.altitude = altitude;
  voo.latitude = gps.location.lat();
  voo.longitude = gps.location.lng();
  voo.sats = gps.satellites.value();
  voo.accelX = a.acceleration.x;
  voo.accelY = a.acceleration.y;
  voo.accelZ = a.acceleration.z;

  Serial.print(tempBME);
  Serial.print(":");
  Serial.print(humDHT);
  Serial.print(":");
  Serial.print(elapsedTime);
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
  Serial.println(a.acceleration.z);

  if (i >= 1484) 
  {
  i = 0; // Reseta após atingir a capacidade máxima
  }

  eeprom.write(44 * i, voo);
  i++;

  delay(2000); // Aguarda 2 segundos antes da próxima leitura
}
