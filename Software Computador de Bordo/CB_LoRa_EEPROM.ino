#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <DHT.h>
#include <Adafruit_MPU6050.h>
#include "LoRaWan_APP.h"
#include "Arduino.h"


#define RF_FREQUENCY                                908000000 // Hz

#define TX_OUTPUT_POWER                             5        // dBm

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false


#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 44 // Define the payload size here

char txpacket[BUFFER_SIZE];

double txNumber;

bool lora_idle=true;

static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );

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

   Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
	
    txNumber=0;

    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    
    Radio.Init( &RadioEvents );
    Radio.SetChannel( RF_FREQUENCY );
    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

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

  if(lora_idle == true)
	{
    delay(1000);
		txNumber += 0.01;
    sprintf(txpacket, "%.2f:%.2f:%.2f:%.6f:%.6f:%.2f:%.2f:%.2f:%0.2f",
            tempBME, humDHT, altitude, gps.location.lat(), gps.location.lng(),
            a.acceleration.x, a.acceleration.y, a.acceleration.z,txNumber);

		Radio.Send( (uint8_t *)txpacket, strlen(txpacket) ); //send the package out	
    lora_idle = false;
	}
  Radio.IrqProcess( );

  delay(2000); // Aguarda 2 segundos antes da próxima leitura
}

void OnTxDone( void )
{
	Serial.println("TX done......");
	lora_idle = true;
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    Serial.println("TX Timeout......");
    lora_idle = true;
}
