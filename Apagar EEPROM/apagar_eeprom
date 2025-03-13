#include <Wire.h>

const int eepromAddress = 0x50; // Endereço I2C da AT24C512
const int eepromSize = 65536;  // Tamanho total da EEPROM AT24C512 em bytes

void clearEEPROM() {
  for (int i = 0; i < eepromSize; i += 64) { // Escreve em blocos de 64 bytes
    Wire.beginTransmission(eepromAddress);
    Wire.write((i >> 8) & 0xFF); // Parte alta do endereço
    Wire.write(i & 0xFF);       // Parte baixa do endereço

    for (int j = 0; j < 64; j++) {
      Wire.write(0xFF); // Escreve 0xFF para apagar os dados
    }

    Wire.endTransmission();
    delay(5); // Pequeno atraso para garantir que a escrita foi concluída
  }
}

void setup() {
  Wire.begin();
  Serial.begin(115200);
  Serial.println("Apagando a EEPROM...");
  clearEEPROM();
  Serial.println("EEPROM apagada com sucesso.");
}

void loop() {
  // Nada aqui
}
