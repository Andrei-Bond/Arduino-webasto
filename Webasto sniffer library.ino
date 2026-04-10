#include <WBusLibrary.h>
#include <CustomSoftwareSerial.h>

// Назначаем пины согласно твоему конфигу
#define WBUS_RX 6
#define WBUS_TX 7

// Инициализируем CustomSoftwareSerial (8N1 по умолчанию)
CustomSoftwareSerial wBusSerial(WBUS_RX, WBUS_TX);

// Передаем объект серийного порта в библиотеку W-Bus
WBusLibrary wbus(&wBusSerial);

void setup() {
  // Скорость для монитора порта на ПК
  Serial.begin(115200);
  
  // W-Bus работает на 10400 бод
  wBusSerial.begin(10400);
  
  Serial.println(F("--- Webasto W-Bus Sniffer (CustomSerial) ---"));
}

void loop() {
  // Пытаемся обновить данные из шины
  if (wbus.update()) {
    Serial.print(F("Status: "));
    Serial.print(wbus.getStateString());
    
    Serial.print(F(" | Temp: "));
    Serial.print(wbus.getTemp());
    Serial.print(F("°C"));
    
    Serial.print(F(" | Volt: "));
    Serial.print(wbus.getVoltage());
    Serial.println(F("V"));
  } else {
    // Если данных нет, просто ждем. 
    // Периодически можно вызывать keepAlive(), если котел на столе.
    // wbus.keepAlive();
  }

  delay(500); // Опрос дважды в секунду
}
