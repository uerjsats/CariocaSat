#include "HardwareSerial.h"

// Cria um objeto HardwareSerial usando o UART 2 (pode ser 1 ou 2 no ESP32)
HardwareSerial MySerial(2); // UART2

void setup() {
  // Inicializa a serial padrão para depuração
  Serial.begin(115200);
  while (!Serial); // Espera abrir a serial

  Serial.println("Iniciando...");

  // Inicializa a serial simulada nos pinos 19 (TX) e 20 (RX)
  MySerial.begin(115200, SERIAL_8N1, 20, 19); 
  //            (baudrate, config, RX, TX)

  delay(1000); // Pequena espera para garantir que tudo iniciou
}

void loop() {
  // Envia os comandos pela serial simulada
  MySerial.println("G1 X0 Y0 Z0");
  MySerial.println("G1 X100 Y100 Z100 F50");

  delay(2000); // Espera 2 segundos antes de enviar de novo
}

