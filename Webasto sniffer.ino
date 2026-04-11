#include <CustomSoftwareSerial.h>

// Твои параметры
#define WBUS_RX 6
#define WBUS_TX 7

// Инициализируем софт-сериал (8 бит данных, без четности, 1 стоп-бит)
CustomSoftwareSerial wBusSerial(WBUS_RX, WBUS_TX);

void setup() {
  // Скорость для монитора порта (USB)
  Serial.begin(115200);
  
  // Стандартная скорость W-Bus / K-Line
  wBusSerial.begin(10400, CSERIAL_8N1);
  
  Serial.println(F("--- W-Bus Sniffer Started (RX:6, TX:7, 10400 baud) ---"));
  Serial.println(F("Waiting for data..."));
}

void loop() {
  // Если в шине появились данные
  if (wBusSerial.available()) {
    byte data = wBusSerial.read();
    
    // Красивый вывод HEX: добавляем ноль впереди, если число меньше 16
    if (data < 0x10) Serial.print("0");
    Serial.print(data, HEX);
    Serial.print(" ");
    
    // Необязательно: если видим паузу в передаче, делаем перенос строки (разделитель пакетов)
    // Для этого можно добавить проверку времени, но для начала хватит и просто потока.
  }
}




// Используем аппаратный Serial для W-bus (Pin 0 - RX)
// ВНИМАНИЕ: При прошивке нужно отключать W-bus от пина 0!

unsigned long lastByteTime = 0;
const int packetTimeout = 20; // Пауза в мс, означающая конец пакета

void setup() {
  // Монитор порта (USB) - используем высокую скорость
  Serial.begin(115200); 
  
  // Настройка порта для W-bus (2400 baud, 8 bits, Even parity, 1 stop bit)
  // На Nano это перенастроит аппаратный RX/TX
  Serial.begin(2400, SERIAL_8E1);
  
  Serial.println("\n--- W-bus Sniffer Started (2400 8E1) ---");
}

void loop() {
  if (Serial.available()) {
    uint8_t b = Serial.read();
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
