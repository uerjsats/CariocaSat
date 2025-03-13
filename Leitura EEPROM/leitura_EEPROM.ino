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

struct dadosVoo {
  float temperatura;
  float tempo;
  float pressao;
  float altitude;
  float umidade;
  float accelX;
  float accelY;
  float accelZ;
  float latitude;
  float longitude;
  float sats;
};

EE24CXXX eeprom(0x50);

dadosVoo voo;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  int i = 0;
  while (true) {
    eeprom.read(44 * i, voo);

    // Verifica se os dados lidos são inválidos (NaN ou todos os valores zerados)
    if (isnan(voo.temperatura) && isnan(voo.umidade) && isnan(voo.pressao) && isnan(voo.altitude) && isnan(voo.tempo) && isnan(voo.latitude) && isnan(voo.longitude) && isnan(voo.sats) && isnan(voo.accelX) && isnan(voo.accelY) && isnan(voo.accelZ)) {
      break;
    }

    Serial.print("Registro ");
    Serial.print(i);
    Serial.print(" - Temp: ");
    Serial.print(voo.temperatura);
    Serial.print(" | Tempo: ");
    Serial.print(voo.tempo);
    Serial.print(" | Umidade: ");
    Serial.print(voo.umidade);
    Serial.print(" | Pressão: ");
    Serial.print(voo.pressao);
    Serial.print(" | Altitude: ");
    Serial.print(voo.altitude);
    Serial.print(" | Latitude: ");
    Serial.print(voo.latitude);
    Serial.print(" | Longitude: ");
    Serial.print(voo.longitude);
    Serial.print(" | Satélites: ");
    Serial.print(voo.sats);
    Serial.print(" | Aceleração X: ");
    Serial.print(voo.accelX);
    Serial.print(" | Aceleração Y: ");
    Serial.print(voo.accelY);
    Serial.print(" | Aceleração Z: ");
    Serial.println(voo.accelZ);

    i++;
  }
}

void loop() {
  // Nada no loop
}
