#include <Wire.h>
#include <DallasTemperature.h>
#include <Adafruit_INA219.h>

// Definir pinos dos sensores DS18B20
#define DS18B20_1_PIN 2
#define DS18B20_2_PIN 3
#define DS18B20_3_PIN 4

// Inicializa o OneWire e DallasTemperature para cada sensor em pinos diferentes
OneWire oneWire1(DS18B20_1_PIN);
DallasTemperature sensor1(&oneWire1);

OneWire oneWire2(DS18B20_2_PIN);
DallasTemperature sensor2(&oneWire2);

OneWire oneWire3(DS18B20_3_PIN);
DallasTemperature sensor3(&oneWire3);

Adafruit_INA219 ina219;

unsigned long tempoInicial = 0;
bool comandoRecebido = false;

void setup() {
  Serial.begin(9600);
  
  // Inicializa os sensores DS18B20
  sensor1.begin();
  sensor2.begin();
  sensor3.begin();

  // Inicializa o INA219
  if (!ina219.begin()) {
    Serial.println("Falha ao encontrar INA219!");
    while (1) { delay(10); }
  } 

  // Calibra o INA219 para leituras mais precisas (padrão para 32 V, 2A)
  ina219.setCalibration_32V_2A();
}

void loop() {
  
  // Ler a tensão do barramento em volts
  float busVoltage = ina219.getBusVoltage_V();
  // Ler a corrente em miliamperes
  float current_mA = ina219.getCurrent_mA();

  // Lê a Temperatura dos sensores
  sensor1.requestTemperatures();
  sensor2.requestTemperatures();
  sensor3.requestTemperatures();

  float tempbateria1 = sensor1.getTempCByIndex(0);
  float tempbateria2 = sensor2.getTempCByIndex(0);
  float tempbateria3 = sensor3.getTempCByIndex(0);
  
  Serial.println(String(busVoltage) + ":"+ String(current_mA) + ":" + String(tempbateria1) + ":" + String(tempbateria2) + ":" + String(tempbateria3));
  

  delay(1000); // Aguarda 1 segundo antes de ler novamente
}

