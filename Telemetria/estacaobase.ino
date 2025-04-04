#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_SSD1306Wire.h"

#define RF_FREQUENCY                                908000000 // Hz
#define TX_OUTPUT_POWER                             14        // dBm
#define LORA_BANDWIDTH                              0         // 125 kHz
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // 4/5
#define LORA_PREAMBLE_LENGTH                        8
#define LORA_SYMBOL_TIMEOUT                         0
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 80  

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

static RadioEvents_t RadioEvents;

int16_t txNumber;
int16_t rssi, rxSize;

bool lora_idle = true;
String inputString = ""; // Para guardar o que foi digitado no serial

// Display OLED
SSD1306Wire factory_display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);


void setup() {
    Serial.begin(115200);
    Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);

    // Previne falhas no barramento I2C
    Wire.begin(SDA_OLED, SCL_OLED);

    // Inicializa o display OLED
    factory_display.init();
    factory_display.clear();
    factory_display.display();
    factory_display.drawString(0, 0, "Iniciando...");
    factory_display.display();
    
    txNumber = 0;
    rssi = 0;

    RadioEvents.RxDone = OnRxDone;
    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);
    Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                      LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                      LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                      0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
    
    // Mensagem inicial no display
    delay(1000);
    factory_display.clear();
    factory_display.drawString(0, 0, "Aguardando pacotes...");
    factory_display.display();
}

void loop() {
    // Verifica se recebeu algo no serial
    while (Serial.available()) {
        char inChar = (char)Serial.read();
        if (inChar == '\n') {
            inputString.trim(); 
            if (inputString.length() > 0 && inputString.length() < BUFFER_SIZE) {
                memset(txpacket, 0, BUFFER_SIZE); 
                inputString.toCharArray(txpacket, BUFFER_SIZE);
                Radio.Send((uint8_t *)txpacket, strlen(txpacket));
                Serial.printf("Mensagem enviada via LoRa: \"%s\"\r\n", txpacket);
                lora_idle = true;
            }
            inputString = ""; // Limpa para próxima entrada
        } else {
            inputString += inChar;
        }
    }

    if (lora_idle) {
        lora_idle = false;
        Radio.Rx(0);
    }

    Radio.IrqProcess();
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssiValue, int8_t snr) {
    rssi = rssiValue;
    rxSize = size;
    memcpy(rxpacket, payload, size);
    rxpacket[size] = '\0';
    Radio.Sleep();

    String displayMessage = String((char*)payload).substring(0, size);

    // Debug via Serial (incluindo RSSI e tamanho)
    Serial.printf("%s:%d:%d\n", displayMessage.c_str(), rssi, rxSize);

    factory_display.clear();
    factory_display.drawString(0, 0, "Telemetria IREC");
    factory_display.drawString(0, 10, "Mensagem:");
    factory_display.drawString(0, 20, displayMessage);
    factory_display.display();
    
    lora_idle = true;
}
