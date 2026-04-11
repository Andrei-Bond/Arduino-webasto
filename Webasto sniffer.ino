#include <CustomSoftwareSerial.h>

// Твои параметры
#define WBUS_RX 6
#define WBUS_TX 7

// Инициализируем софт-сериал (8 бит данных, без четности, 1 стоп-бит)
CustomSoftwareSerial wBusSerial(WBUS_RX, WBUS_TX);


unsigned long lastByteTime = 0;
const int packetTimeout = 20; // Пауза в мс, означающая конец пакета

void setup() {
  // Монитор порта (USB) - используем высокую скорость
  Serial.begin(115200); 
  
  // Настройка порта для W-bus (2400 baud, 8 bits, Even parity, 1 stop bit)
  // На Nano это перенастроит аппаратный RX/TX
  wBusSerial.begin(2400, CSERIAL_8E1);
  
  Serial.println("\n--- W-bus Sniffer Started (2400 8E1) ---");
}

void loop() {
  if (wBusSerial.available()) {
    uint8_t b = wBusSerial.read();
    unsigned long currentTime = millis();

    // Если пауза между байтами большая — печатаем разделитель (новый пакет)
    if (currentTime - lastByteTime > packetTimeout) {
      Serial.println(); 
      Serial.print("[Packet]: ");
    }

    // Выводим байт в формате HEX с ведущим нулем
    if (b < 0x10) Serial.print("0");
    Serial.print(b, HEX);
    Serial.print(" ");

    lastByteTime = currentTime;
  }
}
