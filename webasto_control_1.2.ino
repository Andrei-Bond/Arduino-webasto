// Webasto CAN simulation + w-bus + pump relay + control pump

#include <mcp_can.h>
#include <CustomSoftwareSerial.h>
#include <SPI.h>


// --- ПИНЫ ---
#define WBUS_RX      6
#define WBUS_TX      7
#define FAN_PWM_PIN  9      // Таймер 1 (400 Гц)
#define PUMP_RELAY   5      
#define CLIMATE_RELAY 4     
#define ERR_LED      A1     
#define CURRENT_PIN  A0     
#define CAN0_INT      2      // Прерывание MCP2515

//#define ADDR_HEATER  0xF4 // Адресация: Таймер (F) -> Котел (4) w-bus

CustomSoftwareSerial wBus(WBUS_RX, WBUS_TX);

int coolantTemp = 0;
bool wbusPumpState = false; 
bool pumpIsBroken = false;  
bool isRunning = false;
unsigned long lastWBusQuery = 0;

bool canPumpActive = false;       // Статус помпы по CAN
bool canPumpTimeout = false;     // Флаг отсутствия сигнала работы помпы из CAN
unsigned long lastCanPumpMsg = 0; // Таймер последнего пакета помпы по КАН 0x20 0x08
unsigned long lastBlink = 0;      // Таймер для мигания LED

long unsigned int rxId; // хранилище ИД из CAN
unsigned char len = 0; // хранилище длины данных из CAN
unsigned char rxBuf[8]; // хранилище массива данных из CAN
byte askStat1[8] = {0x00, 0x71, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // ответ для вебасто (есть связь?)

MCP_CAN CAN0(10);     // Set CS to pin 10

void setup() {
  Serial.begin(115200);
  
  // Initialize MCP2515 running at 8MHz with a baudrate of 100kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_100KBPS, MCP_8MHZ) == CAN_OK) Serial.println("MCP2515 Инициализирован успешно!");
  else Serial.println("Ошибка инициализации MCP2515...");

  CAN0.setMode(MCP_NORMAL);   // Выбираем нормальный режии, чтобы разрешить отправку сообщений

  pinMode(CAN0_INT, INPUT);                            // Configuring pin for /INT input

   // Инициализация шины Webasto на 2400 бод
  wBus.begin(2400, CSERIAL_8E1);

  // Настройки для вентилятора, помпы и светодиода ошибки помпы
  pinMode(PUMP_RELAY, OUTPUT);
  pinMode(CLIMATE_RELAY, OUTPUT);
  pinMode(ERR_LED, OUTPUT);
  pinMode(CAN0_INT, INPUT); // Пин прерывания CAN

  // Выключаем всё (инверсная логика реле: HIGH = выкл)
  digitalWrite(PUMP_RELAY, LOW);   
  digitalWrite(CLIMATE_RELAY, HIGH); 
  digitalWrite(ERR_LED, LOW);       // Ошибки нет

  // Настройка Таймера 1 (400 Гц на D9)
  pinMode(FAN_PWM_PIN, OUTPUT);
  TCCR1A = _BV(COM1A1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);
  ICR1 = 4999; 
  OCR1A = 0; 
  
}




void loop() {
  // --- 1. ЛОГИКА CAN (MCP2515) ---
  if (!digitalRead(CAN0_INT)) { 
    // Читаем CAN, отвечаем котлу (твой код)          // Если вывод CAN0_INT is LOW, отправляем подтверждение связи
     
    CAN0.readMsgBuf(&rxId, &len, rxBuf);  // Считывем данные: len = длина данных, buf = байт(ы) данны
    CAN0.sendMsgBuf(0x427, 0, 8, askStat1);
  
        if (rxId == 0x3E5 && len >= 2) {
        if ((rxBuf[1] & 0x0A)) {
           Serial.println("КАН: помпа вкл");
          canPumpTimeout = false; // Сигнал на пуск помпы есть, сбрасываем таймаут
            canPumpActive = true;
            lastCanPumpMsg = millis(); // Обновляем время активности
          } else {
            canPumpActive = false; // Выключили штатно
          
        }
      }
  
    // Как только есть активность — планируем опрос W-Bus
  //  if (millis() - lastWBusQuery > 3000) {
  //    sendWBusQuery(); 
   //   lastWBusQuery = millis();
 //   }
  }

  // --- 2. ЗАЩИТА: ТАЙМАУТ СВЯЗИ CAN (5 секунд) ---
  if (canPumpActive && (millis() - lastCanPumpMsg > 5000)) {
    canPumpActive = false;
    canPumpTimeout = true; // Устанавливаем флаг отсутствия сигнала работы помпы по CAN
    Serial.println("Сигнал работы помпы из CAN отсутствует!");
  }

  // --- 3. УПРАВЛЕНИЕ РЕЛЕ ПОМПЫ ---
  // Работает, если ХОТЯ БЫ ОДНА шина активна И нет аппаратной поломки (pumpIsBroken)
  if ((wbusPumpState || canPumpActive) && !pumpIsBroken) {
    digitalWrite(PUMP_RELAY, HIGH); 
    checkPumpHealth(); // Твоя защита по току ACS712
  } else {
    digitalWrite(PUMP_RELAY, LOW);
  }
  manageClimate();

  // --- 4. ИНДИКАЦИЯ ОШИБОК ---
  if (pumpIsBroken) {
    
    digitalWrite(PUMP_RELAY, LOW);
    digitalWrite(CLIMATE_RELAY, HIGH);
    OCR1A = 0;
    
    //if (wbusPumpState || canPumpActive) {
    //stopSystem();
//}
    // МЕДЛЕННОЕ мигание (500мс) - Поломка помпы (Ток)
    if (millis() - lastBlink > 500) {
      digitalWrite(ERR_LED, !digitalRead(ERR_LED));
      lastBlink = millis();
    }
  } 
  else if (canPumpTimeout) {
    // БЫСТРОЕ мерцание (100мс) - Отсутствие сигнала работы помпы по CAN
    if (millis() - lastBlink > 100) {
      digitalWrite(ERR_LED, !digitalRead(ERR_LED));
      lastBlink = millis();
    }
  } 
  else {
    digitalWrite(ERR_LED, LOW); // Ошибок нет
  }

  // --- 5. НЕБЛОКИРУЮЩЕЕ ЧТЕНИЕ W-BUS ---
  checkWBusSerial(); 

  // Пример управления через монитор порта: '1' - старт, '0' - стоп
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '1') startSystem(60); 
    if (c == '0') stopSystem();
    if (c == '2') sendWBusQuery();
  }
  
}

// --- ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ---


void checkWBusSerial() {
  static byte buf[20];
  static int idx = 0;
  static unsigned long timeout = 0;

  while (wBus.available()) {
    buf[idx++] = wBus.read();
    timeout = millis();
    if (idx >= 20) break;
  }

  if (idx > 0 && (millis() - timeout > 10)) {
    if (idx > 3 && (buf[0] == 0x43)&&(buf[2] == 0xD0)&&(buf[3] == 0x05)) {
      coolantTemp = map((int)buf[4], 153, 204, 36, 10);
      Serial.println(coolantTemp);
      //Serial.println((int)buf[4]);
      //wbusPumpState = (buf[4] & 0x08);
    //  Serial.println(wbusPumpState);
    } else if (idx > 3 && (buf[0] == 0x43)&&(buf[2] == 0xD0)&&(buf[3] == 0x03)) {
      wbusPumpState = (buf[6] & 0x08);
      Serial.println(wbusPumpState);
    }
    idx = 0;
  }
}

void manageClimate() {
  if (coolantTemp > 40) {
    digitalWrite(CLIMATE_RELAY, LOW);
    OCR1A = 3500; // 70% от 4999
  } else {
    digitalWrite(CLIMATE_RELAY, HIGH);
    OCR1A = 0;
  }
}

void checkPumpHealth() {
  static unsigned long pTimer = 0;
  if (pTimer == 0) pTimer = millis();
  if (millis() - pTimer > 3000) {
    float amps = readAmps();
    if (amps < 0.4 || amps > 3.0) pumpIsBroken = true;
  }
}

float readAmps() {
  long sum = 0;
  for(int i=0; i<10; i++) sum += analogRead(CURRENT_PIN);
  float voltage = (sum / 10.0 * 5.0) / 1024.0;
  return abs(voltage - 2.5) / 0.185; 
}

// --- ФУНКЦИИ УПРАВЛЕНИЯ W-bus ---
  // Команда 0x21 (управление), 0x01 (старт), mins (время в минутах)
void startSystem(byte mins) {
  isRunning = true;
 // byte startData[] = {0x21, 0x01, mins};
  byte startData[] = {0x21, mins};
  sendExtendedWBus(startData, 2);
  Serial.println("W-Bus: START sent");
}

  // Команда 0x21 (управление), 0x00 (выключить)
  
void stopSystem() {
  isRunning = false;
  byte stopData[] = {0x10};
  sendExtendedWBus(stopData, 1);
  Serial.println("W-Bus: STOP sent");
}

void sendWBusQuery() {
  // Команда 0x05 (запрос состояния/температуры)
  byte cmdTemp[] = {0x50, 0x05};
  sendExtendedWBus(cmdTemp, 2);
  
  byte cmdPump[] = {0x50, 0x03};
  sendExtendedWBus(cmdPump, 2);
}

void sendExtendedWBus(byte* data, int len) {
  wBus.write(0x24); wBus.write((byte)len+1);
  byte crc = 0x24 ^ ((byte)len+1);
  for(int i=0; i<len; i++) { wBus.write(data[i]); crc ^= data[i]; }
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


Рекомендации по эксплуатации:
Охлаждение: MOSFET на пине D9 (обдув салона) будет нагружен — обязательно используй радиатор.
Помехи: На линии W-Bus в авто много шумов. Используй экранированный провод или витую пару для стабильности.
Защита: Если помпа подключена длинным проводом, добавь параллельно ей диод (катодом к "+"), чтобы обратный ток индуктивности не сжег реле или Ардуино.
Прошивка: При загрузке скетча в Nano не забудь выбрать правильный процессор: ATmega328P или ATmega328P (Old Bootloader). 


 Список транзисторов для вентилятора
 1. Самый лучший выбор (Logic Level)
Эти модели идеально работают с Arduino и имеют огромный запас по току (не будут греться на сигнальных линиях):
IRLZ44N — «Золотой стандарт». Мощный, надежный, очень распространенный. (Не путай с IRFZ44N без буквы L).
IRL540N — Аналог предыдущего, чуть дороже, но тоже отличный.
IRL3705N — Очень низкое сопротивление, будет абсолютно холодным.
2. Малогабаритные (если мало места)
Если не хочешь громоздкий корпус TO-220 (с дыркой под болт), эти крохи справятся с током до 1-2А без радиатора:
2N7000 — Самый дешевый и маленький (как обычный транзистор). Подойдет, если ток управления в Peugeot 508 действительно слабый (до 200мА).
BS170 — Чуть мощнее, чем 2N7000 (до 500мА).
FJN3303 — Тоже часто встречается в магазинах.
3. "На крайний случай" (если нет Logic Level)
Если в магазине только серия IRF (обычные), можно взять их, но они могут потребовать радиатор даже на малых токах из-за неполного открытия:
IRFZ44N (самый частый гость на полках).
IRF540N.

//код анализа кан для включения помпы


bool canPumpActive = false;       // Статус помпы по CAN
bool canPumpTimeout = false;     // Флаг отсутствия сигнала работы помпы из CAN
unsigned long lastCanPumpMsg = 0; // Таймер последнего пакета 0x20 0x08
unsigned long lastBlink = 0;      // Таймер для мигания LED

void loop() {
  // --- 1. ПРИЕМ СООБЩЕНИЙ MCP2515 (CAN) ---
  if (!digitalRead(CAN_INT)) { 
    long unsigned int rxId; unsigned char len = 0; unsigned char rxBuf[8]; 
    if (CAN.readMsgBuf(&rxId, &len, rxBuf) == CAN_OK) {
      
      if (rxId == 0x3E5 && len >= 2) {
        if ((rxBuf[1] & 0x0A)) {
          canPumpTimeout = false; // Сигнал на пуск помпы есть, сбрасываем таймаут
            canPumpActive = true;
            lastCanPumpMsg = millis(); // Обновляем время активности
          } else {
            canPumpActive = false; // Выключили штатно
          
        }
      }
    }
  }

  // --- 2. ЗАЩИТА: ТАЙМАУТ СВЯЗИ CAN (5 секунд) ---
  if (canPumpActive && (millis() - lastCanPumpMsg > 5000)) {
    canPumpActive = false;
    canPumpTimeout = true; // Устанавливаем флаг отсутствия сигнала работы помпы по CAN
    Serial.println("Сигнал работы помпы из CAN отсутствует!");
  }

  // --- 3. УПРАВЛЕНИЕ РЕЛЕ ПОМПЫ ---
  // Работает, если ХОТЯ БЫ ОДНА шина активна И нет аппаратной поломки (systemHalted)
  if ((wbusPumpState || canPumpActive) && !systemHalted) {
    digitalWrite(PUMP_RELAY, LOW); 
    checkPumpHealth(); // Твоя защита по току ACS712
  } else {
    digitalWrite(PUMP_RELAY, HIGH);
  }

  // --- 4. ИНДИКАЦИЯ ОШИБОК ---
  if (systemHalted) {
    // МЕДЛЕННОЕ мигание (500мс) - Поломка помпы (Ток)
    if (millis() - lastBlink > 500) {
      digitalWrite(ERR_LED, !digitalRead(ERR_LED));
      lastBlink = millis();
    }
  } 
  else if (canPumpTimeout) {
    // БЫСТРОЕ мерцание (100мс) - Отсутствие сигнала работы помпы по CAN
    if (millis() - lastBlink > 100) {
      digitalWrite(ERR_LED, !digitalRead(ERR_LED));
      lastBlink = millis();
    }
  } 
  else {
    digitalWrite(ERR_LED, LOW); // Ошибок нет
  }

  // ... (Опрос W-Bus и управление климатом 400Гц остаются ниже)
}


Для сборки надежного адаптера на L9637D и подключения всей периферии (силовая часть, датчики, CAN) тебе понадобится следующий набор. Я разбил его на группы, чтобы было удобно проверять в магазине.
1. Обвязка для L9637D (W-Bus интерфейс)
Микросхема чувствительна к питанию, поэтому берем компоненты для её «жизни»:
Резистор 510 Ом или 620 Ом (0.25 Вт) — 1 шт. (Это Pull-up. Ставится между пином Kout и +12В. Без него связи с котлом не будет).
Резистор 1 кОм — 2 шт. (Защитные, в разрыв линий RX и TX между Ардуино и микросхемой).
Конденсатор керамический 0.1 мкФ (100нФ) — 2 шт. (Ставятся максимально близко к ножкам питания микросхемы Vs и Vcc на землю).
Конденсатор электролитический 100 мкФ (25В или 50В) — 1 шт. (На вход питания +12В для сглаживания скачков напряжения авто).
2. Силовая часть (Климат и Помпа)
То, что мы обсуждали для управления вентилятором 400 Гц и реле:
MOSFET IRLZ44N — 1 шт. (Логический, для управления вентилятором. L в названии обязательна!).
Резистор 220 Ом — 1 шт. (В затвор MOSFET).
Резистор 10 кОм — 1 шт. (С затвора MOSFET на массу).
Реле автомобильное 12В (4 или 5 контактов) — 2 шт. (Одно на помпу, второе на перехват климата). Или готовый модуль реле для Ардуино на 2 канала.
Диод 1N4007 — 2 шт. (Обязательно! Ставятся параллельно обмоткам реле, чтобы при выключении не сжечь Ардуино обратным током).
3. Датчик тока и Питание
Модуль ACS712 (на 5А или 20А) — 1 шт. (Для контроля исправности помпы).
DC-DC преобразователь (Step-Down) — 1 шт. (На базе LM2596 или миниатюрный на MP1584). Настраиваешь его на выход 5В и питаешь от него Ардуино, MCP2515 и L9637D. Питать напрямую от 12В через пин Vin нельзя — сгорит.
4. Защита (Рекомендую для авто)
Предохранитель флажковый (5А) с держателем — 1 шт. (На общий провод питания всей твоей схемы).
Супрессор (стабилитрон) на 18В или 24В (например, P6KE18A) — 1 шт. (Параллельно входу питания, чтобы «срезать» высоковольтные иголки от генератора).
Короткий чек-лист перед походом в магазин:
L9637D (само собой).
IRLZ44N (именно с буквой L).
Резисторы: 220 Ом, 510 Ом, 1 кОм, 10 кОм.
Конденсаторы: 0.1 мкФ керамика 2 шт, 100 мкФ электролит.
Диоды: 1N4007.
Модули: ACS712, DC-DC mini, MCP2515 (если еще нет).
Подсказать, как правильно соединить L9637D и Ардуино, чтобы они не конфликтовали с сигнализацией на одной линии?









*/
