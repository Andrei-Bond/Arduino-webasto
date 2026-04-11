#include <CustomSoftwareSerial.h>

#define WBUS_RX 6
#define WBUS_TX 7

CustomSoftwareSerial wBusSerial(WBUS_RX, WBUS_TX);

unsigned long lastByteTime = 0;
unsigned long lastRequestTime = 0;
const int packetTimeout = 20; 
const unsigned long requestInterval = 2000; // Опрос раз в 2 секунды

// Пакет запроса данных (Query Sensors/Data)
// 44 (адрес) 05 (длина) 05 (команда) -> контрольная сумма XOR = 44
uint8_t queryTemp[] = {0x44, 0x05, 0x05, 0x44};

void setup() {
  Serial.begin(115200); 
  // Твои настройки: 2400 бод, четность Even (E), 8 бит
  wBusSerial.begin(2400, CSERIAL_8E1);
  
  Serial.println("\n--- W-bus Sniffer + Temp Request (2400 8E1) ---");
}

void loop() {
  // 1. Читаем ответы из шины
  if (wBusSerial.available()) {
    uint8_t b = wBusSerial.read();
    unsigned long currentTime = millis();

    if (currentTime - lastByteTime > packetTimeout) {
      Serial.println(); 
      Serial.print("[Data]: ");
    }

    if (b < 0x10) Serial.print("0");
    Serial.print(b, HEX);
    Serial.print(" ");

    lastByteTime = currentTime;
  }

  // 2. Отправляем запрос температуры каждые 2 секунды
  if (millis() - lastRequestTime > requestInterval) {
    sendTempRequest();
    lastRequestTime = millis();
  }
}

void sendTempRequest() {
  // Отправляем пакет в шину
  for (int i = 0; i < sizeof(queryTemp); i++) {
    wBusSerial.write(queryTemp[i]);
  }
  // Помечаем в логе, что это был наш запрос
  Serial.println();
  Serial.print(">>> Sent Temp Request");
}