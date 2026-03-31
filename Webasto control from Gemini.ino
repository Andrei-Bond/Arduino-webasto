#include <CustomSoftwareSerial.h> // Библиотека для 8E1 (четности) на программных пинах
#include <SPI.h>                  // Для связи с MCP2515 (CAN-модуль)

/* --- НАЗНАЧЕНИЕ ПИНОВ (Arduino Nano) --- */
#define WBUS_RX      6      // К K-Line адаптеру (RX)
#define WBUS_TX      7      // К K-Line адаптеру (TX)
#define FAN_PWM_PIN  9      // Управление вентилятором (Таймер 1, строго 400 Гц)
#define PUMP_RELAY   5      // Реле включения помпы
#define CLIMATE_RELAY 4     // Реле перехвата управления (Климат Авто <-> Ардуино)
#define ERR_LED      A1     // Светодиод ошибки (Аналоговый пин в режиме выхода)
#define CURRENT_PIN  A0     // Датчик тока ACS712 или Шунт

/* --- ПАРАМЕТРЫ СИСТЕМЫ --- */
#define ADDR_HEATER  0x4F   // Адрес Webasto в шине W-Bus
const float CUR_MIN = 0.4;  // Минимальный ток помпы (А) - ниже значит обрыв
const float CUR_MAX = 3.0;  // Максимальный ток помпы (А) - выше значит заклинивание

/* --- ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ --- */
CustomSoftwareSerial wBus(WBUS_RX, WBUS_TX);
int coolantTemp = 0;        // Температура ОЖ из Webasto
bool wbusPumpState = false; // Статус помпы (нужна ли она сейчас котлу)
bool systemHalted = false;  // Флаг критической ошибки (блокировка системы)
unsigned long lastWBusQuery = 0;
unsigned long lastErrorBlink = 0;

void setup() {
  Serial.begin(9600);       // Порт для отладки (монитор порта ПК)
  wBus.begin(2400, CSERIAL_8E1); // W-Bus работает на 2400 бод с четностью Even
  
  pinMode(PUMP_RELAY, OUTPUT);
  pinMode(CLIMATE_RELAY, OUTPUT);
  pinMode(ERR_LED, OUTPUT);
  
  // Изначально всё выключено (High на реле обычно означает выкл)
  digitalWrite(PUMP_RELAY, HIGH);   
  digitalWrite(CLIMATE_RELAY, HIGH); 
  digitalWrite(ERR_LED, LOW);

  /* НАСТРОЙКА ТАЙМЕРА 1 ДЛЯ ШИМ 400 Гц (Пин D9) */
  // Мы настраиваем регистры процессора напрямую, чтобы получить ровно 400 Гц для Peugeot 508
  pinMode(FAN_PWM_PIN, OUTPUT);
  TCCR1A = _BV(COM1A1) | _BV(WGM11);            // Режим Fast PWM, выход на канал A (D9)
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11); // Предделитель 8
  ICR1 = 4999;                                  // Потолок счета: (16МГц / (8 * 400Гц)) - 1
  OCR1A = 0;                                    // Стартовое заполнение 0%

  // ТУТ ИНИЦИАЛИЗАЦИЯ ВАШЕЙ MCP2515 (код опущен)
  Serial.println("System Started. Waiting for CAN traffic...");
}

void loop() {
  /* --- 1. ЛОГИКА CAN-ШИНЫ (MCP2515) --- */
  // Если в CAN-шине есть любая активность (котел проснулся)
  /* if (CAN.checkReceive() == CAN_MSGAVAIL) {
      // Читаем сообщение, чтобы очистить буфер
      long unsigned int rxId; unsigned char len = 0; unsigned char rxBuf[8];
      CAN.readMsgBuf(&rxId, &len, rxBuf);

      // Как только видим активность — опрашиваем W-Bus (раз в 3 сек)
      if (millis() - lastWBusQuery > 3000) {
        sendWBus(0x05); // Запрос состояния (Query)
        lastWBusQuery = millis();
      }
  } */

  /* --- 2. ПАРСИНГ ОТВЕТА ИЗ W-BUS --- */
  if (wBus.available() > 0) {
    parseWBus(); // Извлекаем температуру и статус помпы
  }

  /* --- 3. ЛОГИКА БЕЗОПАСНОСТИ И УПРАВЛЕНИЯ --- */
  if (!systemHalted) {
    // Если котел по W-Bus требует работу помпы
    if (wbusPumpState) {
      digitalWrite(PUMP_RELAY, LOW); // Включаем физическое реле
      checkPumpHealth();             // Сразу проверяем ток (защита)
    } else {
      digitalWrite(PUMP_RELAY, HIGH);
    }

    // Управление климатом (вентилятором)
    manageClimate();
  } 
  else {
    /* РЕЖИМ АВАРИИ (Помпа неисправна) */
    // Если котел запущен (например, сигнализацией), а помпа мертва — шлем СТОП
    if (wbusPumpState) { 
      sendExtended((byte[]){0x21, 0x00}, 2); // Команда STOP в Webasto
      Serial.println("SAFETY: Webasto STOP sent due to Pump Error!");
    }
    
    // Мигаем светодиодом ошибки без остановки основного цикла
    if (millis() - lastErrorBlink > 300) {
      digitalWrite(ERR_LED, !digitalRead(ERR_LED));
      lastErrorBlink = millis();
    }

    // Силовое обесточивание всех систем
    digitalWrite(PUMP_RELAY, HIGH);
    digitalWrite(CLIMATE_RELAY, HIGH);
    OCR1A = 0; 
  }
}

/* --- ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ --- */

// Установка скорости вентилятора (0-100%)
void setFanDuty(int percent) {
  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;
  OCR1A = map(percent, 0, 100, 0, 4999); // Перевод процентов в диапазон таймера
}

// Управление климатом Peugeot 508
void manageClimate() {
  if (coolantTemp > 40) {
    digitalWrite(CLIMATE_RELAY, LOW); // Переключаем реле на Ардуино
    setFanDuty(70);                  // ШИМ 400Гц, заполнение 70% (Low Side)
  } else {
    digitalWrite(CLIMATE_RELAY, HIGH); // Возвращаем управление штатному климату
    setFanDuty(0);
  }
}

// Проверка исправности помпы по току
void checkPumpHealth() {
  static unsigned long pumpTimer = 0;
  if (pumpTimer == 0) pumpTimer = millis();

  if (millis() - pumpTimer > 3000) { // Пропускаем пусковой ток (3 сек)
    float amps = readAmps();
    if (amps < CUR_MIN || amps > CUR_MAX) {
      systemHalted = true; // Уходим в необратимую ошибку
      Serial.print("PUMP FAILURE! Amps: "); Serial.println(amps);
    }
  }
}

// Чтение тока с датчика ACS712-05B
float readAmps() {
  long sum = 0;
  for(int i=0; i<15; i++) sum += analogRead(CURRENT_PIN);
  float voltage = (sum / 15.0 * 5.0) / 1024.0;
  return abs(voltage - 2.5) / 0.185; // 2.5V - центр, 185мВ - 1 Ампер
}

// Разбор пакета из W-Bus
void parseWBus() {
  byte b[20];
  int len = 0;
  while(wBus.available() && len < 20) {
    b[len++] = wBus.read();
    delay(2); // Ждем байты пакета
  }
  
  // Пакет ответа 0x4F (котел) или 0xF4 (диагностика)
  if (len > 7 && (b[0] == 0x4F || b[0] == 0xF4)) {
    coolantTemp = b[4] - 50; // 5-й байт минус 50 = градусы
    // Статус работы: проверяем биты горения (0x01) или помпы (0x02)
    wbusPumpState = (b[6] & 0x02) || (b[6] & 0x01); 
  }
}

// Отправка коротких команд (Query)
void sendWBus(byte cmd) {
  byte p[] = { ADDR_HEATER, 0x01, cmd };
  byte crc = ADDR_HEATER ^ 0x01 ^ cmd;
  for(int i=0; i<3; i++) wBus.write(p[i]);
  wBus.write(crc);
}

// Отправка длинных команд (Start/Stop)
void sendExtended(byte* data, int len) {
  wBus.write(ADDR_HEATER);
  wBus.write((byte)len);
  byte crc = ADDR_HEATER ^ (byte)len;
  for(int i=0; i<len; i++) {
    wBus.write(data[i]);
    crc ^= data[i];
  }
  wBus.write(crc);
}

/*
Итоговые рекомендации и нюансы:
Защита MOSFET: При управлении вентилятором Peugeot 508 "Low Side" (замыкание на массу), MOSFET будет коммутировать значительный ток. Используй IRLZ44N (он управляется логическим уровнем 5В) и обязательно закрепи его на радиаторе.
Резистор 10кОм: Обязательно поставь резистор между Gate (затвором) и Source (истоком) MOSFET. Это гарантирует, что вентилятор не включится на максимум, пока Arduino загружается.
Контроль тока: Если помпа VAG штатная, она потребляет около 1А. Если ты увидишь в мониторе порта значения 0.0 или 0.1 при работающей помпе — проверь питание ACS712 (оно должно быть строго 5В) или исправность шунта.
Развязка W-Bus: Если в линию W-Bus подключена сигнализация, убедись, что твоя Arduino подключена через полноценный K-Line адаптер (на транзисторах или микросхеме L9637D), а не просто проводом. Это защитит пины Arduino от бортового напряжения 12В.
Питание системы: В Peugeot 508 и Touareg 2008 очень "шумное" бортовое питание. Питать Arduino Nano лучше через качественный DC-DC преобразователь (например, на базе MP1584), настроенный на 5В, а не через пин VIN.
Индексы байт: После первого запуска обязательно включи HEX-дамп (который мы обсуждали) и убедись, что в твоей версии Webasto температура и статус помпы находятся именно в 5-м и 7-м байтах. У штатных VAG котлов это стандарт, но иногда бывают исключения.
Безопасность CAN: Поскольку ты читаешь CAN, чтобы активировать W-Bus, убедись, что твой модуль MCP2515 имеет общий минус (GND) с Arduino и остальной проводкой.

*/