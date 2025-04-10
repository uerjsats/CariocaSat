#include <Wire.h>
#include <cmath>
#include "float16.h"

class EE24CXXX {
    private:
        byte _device_address;
    public:
        EE24CXXX(byte device_address) { _device_address = device_address; }
        void read(unsigned int eeaddress, unsigned char * data, unsigned int data_len);

        template <class T> int read(unsigned int eeaddress, T& value);
};

void EE24CXXX::read(unsigned int eeaddress, unsigned char * data, unsigned int data_len) {
    unsigned char i = 0;
    unsigned int size = data_len;
    unsigned int j = 0;

    while (size > 0) {
        Wire.beginTransmission(_device_address);
        eeaddress += j * 28;

        Wire.write((int)(eeaddress >> 8));   // MSB
        Wire.write((int)(eeaddress & 0xFF)); // LSB
        Wire.endTransmission();

        if (size >= 28) {
        Wire.requestFrom(_device_address, (unsigned int)28);
        size -= 28;
        } else {
        Wire.requestFrom(_device_address, (unsigned int)size);
        size = 0;
        }

        while (Wire.available()) { data[i++] = Wire.read(); }
        j++;
    }
}

template <class T> int EE24CXXX::read(unsigned int eeaddress, T& value) {
    byte *p = (byte *)(void *)&value;
    unsigned char c[sizeof(value)];
    read(eeaddress, c, sizeof(value));
    for (int i = 0; i < sizeof(value); i++) { *p++ = c[i]; }
    return sizeof(value);
}

struct sensorsData {
    unsigned short seconds;

    float16 temperature;
    float16 humidity;
    float16 pressure;
    float16 altitude;

    float16 latitude;
    float16 longitude;

    float16 accelX;
    float16 accelY;
    float16 accelZ;
};

EE24CXXX eeprom(0x50);

sensorsData dados;

void setup() {
    Serial.begin(115200);
    Wire.begin();

    int i = 0;
    while (true) {
        eeprom.read(20 * i, dados);

        // Verifica se os dados lidos são inválidos (NaN ou todos os valores zerados)
        if (dados.temperature.isNaN() && dados.humidity.isNaN() &&
            dados.pressure.isNaN() && dados.altitude.isNaN() && dados.latitude.isNaN() &&
            dados.longitude.isNaN() &&
            dados.accelX.isNaN() && dados.accelY.isNaN() && dados.accelZ.isNaN()) {
        break;
        }

        Serial.print("Registro ");
        Serial.print(i);
        Serial.print(" - Tempo: ");
        Serial.print(dados.seconds);
        Serial.print(" | Temperatura: ");
        Serial.print(dados.temperature.toString());
        Serial.print(" | Umidade: ");
        Serial.print(dados.humidity.toString());
        Serial.print(" | Pressão: ");
        Serial.print(dados.pressure.toString());
        Serial.print(" | Altitude: ");
        Serial.print(dados.altitude.toString());
        Serial.print(" | Latitude: ");
        Serial.print(dados.latitude.toString());
        Serial.print(" | Longitude: ");
        Serial.print(dados.longitude.toString());
        Serial.print(" | Aceleração X: ");
        Serial.print(dados.accelX.toString());
        Serial.print(" | Aceleração Y: ");
        Serial.print(dados.accelY.toString());
        Serial.print(" | Aceleração Z: ");
        Serial.println(dados.accelZ.toString());

        i++;
    }
}

void loop() {
  // Nada no loop
}
