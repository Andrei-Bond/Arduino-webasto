#include <WebastoHeaterWBus.h>
#include <CustomSoftwareSerial.h>

// Твои пины
#define WBUS_RX 6
#define WBUS_TX 7

// Инициализируем программный порт
CustomSoftwareSerial wBusSerial(WBUS_RX, WBUS_TX);

// Передаем порт в библиотеку
WebastoHeaterWBus wbus(&wBusSerial);

void setup() {
  // Монитор порта для отладки
  Serial.begin(115200);
  
  // W-Bus всегда работает на 10400
  wBusSerial.begin(10400);

  Serial.println(F("--- Webasto Touareg 2008 W-Bus Interface ---"));
}

void loop() {
  // update() опрашивает котел и возвращает true, если данные получены
  if (wbus.update()) {
    
    Serial.print(F("Status: "));
    Serial.print(wbus.getStateString()); // Состояние (Heating, Purge и т.д.)
    
    Serial.print(F(" | Temp: "));
    Serial.print(wbus.getTemp());
    Serial.print(F("C"));
    
    Serial.print(F(" | Volt: "));
    Serial.print(wbus.getVoltage());
    Serial.println(F("V"));
    
    // Если нужно увидеть ошибки
    if (wbus.getErrorCount() > 0) {
      Serial.print(F("Errors found: "));
      Serial.println(wbus.getErrorCount());
    }
  }

  // Для Touareg 2008 (VAG) критично поддерживать связь, 
  // если котел запущен не штатно, иначе он уйдет в Standby
  wbus.keepAlive();

  delay(1000); // Опрос раз в секунду
}
