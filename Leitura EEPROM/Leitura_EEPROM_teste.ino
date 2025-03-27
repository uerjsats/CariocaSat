#include <Wire.h>
#include <cmath>

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
    int seconds;

    float temperature;
    float humidity;
    float pressure;
    float altitude;

    float latitude;
    float longitude;
    int sats;

    float accelX;
    float accelY;
    float accelZ;
};

EE24CXXX eeprom(0x50);

sensorsData dados;

void setup() {
    Serial.begin(115200);
    Wire.begin();

    int i = 0;
    while (true) {
        eeprom.read(44 * i, dados);

        // Verifica se os dados lidos são inválidos (NaN ou todos os valores zerados)
        if (isnan(dados.temperature) && isnan(dados.humidity) &&
            isnan(dados.pressure) && isnan(dados.altitude) &&
            dados.seconds == -1 && isnan(dados.latitude) &&
            isnan(dados.longitude) && dados.sats == -1 &&
            isnan(dados.accelX) && isnan(dados.accelY) && isnan(dados.accelZ)) {
        break;
        }

        Serial.print("Registro ");
        Serial.print(i);
        Serial.print(" - Tempo: ");
        Serial.print(dados.seconds);
        Serial.print(" | Temperatura: ");
        Serial.print(dados.temperature);
        Serial.print(" | Umidade: ");
        Serial.print(dados.humidity);
        Serial.print(" | Pressão: ");
        Serial.print(dados.pressure);
        Serial.print(" | Altitude: ");
        Serial.print(dados.altitude);
        Serial.print(" | Latitude: ");
        Serial.print(dados.latitude);
        Serial.print(" | Longitude: ");
        Serial.print(dados.longitude);
        Serial.print(" | Satélites: ");
        Serial.print(dados.sats);
        Serial.print(" | Aceleração X: ");
        Serial.print(dados.accelX);
        Serial.print(" | Aceleração Y: ");
        Serial.print(dados.accelY);
        Serial.print(" | Aceleração Z: ");
        Serial.println(dados.accelZ);

        i++;
    }
}

void loop() {
  // Nada no loop
}
