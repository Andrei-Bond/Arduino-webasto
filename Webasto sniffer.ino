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
