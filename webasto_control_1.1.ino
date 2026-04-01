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
#define CAN0_INT      3      // Прерывание MCP2515

#define ADDR_HEATER  0x4F // Адресация: Таймер (F) -> Котел (4) w-bus

CustomSoftwareSerial wBus(WBUS_RX, WBUS_TX);

int coolantTemp = 0;
bool wbusPumpState = false; 
bool systemHalted = false;  
unsigned long lastWBusQuery = 0;
unsigned long lastErrorBlink = 0;

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
  digitalWrite(PUMP_RELAY, HIGH);   
  digitalWrite(CLIMATE_RELAY, HIGH); 
  digitalWrite(ERR_LED, LOW);       // Ошибки нет

  // Настройка Таймера 1 (400 Гц на D9)
  pinMode(FAN_PWM_PIN, OUTPUT);
  TCCR1A = _BV(COM1A1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);
  ICR1 = 4999; 
  OCR1A = 0; 
  
  // Тут твоя инициализация CAN: CAN.begin(...);
}




void loop() {
  // --- 1. ЛОГИКА CAN (MCP2515) ---
  if (!digitalRead(CAN0_INT)) { 
    // Читаем CAN, отвечаем котлу (твой код)
     if(!digitalRead(CAN0_INT))          // Если вывод CAN0_INT is LOW, отправляем подтверждение связи
  {   
    CAN0.readMsgBuf(&rxId, &len, rxBuf);  // Считывем данные: len = длина данных, buf = байт(ы) данны
    CAN0.sendMsgBuf(0x427, 0, 8, askStat1);
  
}
    // Как только есть активность — планируем опрос W-Bus
    if (millis() - lastWBusQuery > 3000) {
      sendWBus(0x05); 
      lastWBusQuery = millis();
    }
  }

  // --- 2. НЕБЛОКИРУЮЩЕЕ ЧТЕНИЕ W-BUS ---
  checkWBusSerial(); 

  // --- 3. ЛОГИКА БЕЗОПАСНОСТИ ---
  if (!systemHalted) {
    if (wbusPumpState) {
      digitalWrite(PUMP_RELAY, LOW); 
      checkPumpHealth();             
    } else {
      digitalWrite(PUMP_RELAY, HIGH);
    }
    manageClimate();
  } 
  else {
    // АВАРИЙНЫЙ РЕЖИМ: Блокируем пуск, если помпа мертва
    if (wbusPumpState) {
    byte stopCmd[] = {0x21, 0x00};
    sendExtended(stopCmd, 2);
}
    
    if (millis() - lastErrorBlink > 300) {
      digitalWrite(ERR_LED, !digitalRead(ERR_LED));
      lastErrorBlink = millis();
    }
    digitalWrite(PUMP_RELAY, HIGH);
    digitalWrite(CLIMATE_RELAY, HIGH);
    OCR1A = 0;
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
    if (idx > 7 && (buf[0] == 0x4F || buf[0] == 0xF4)) {
      coolantTemp = buf[4] - 50;
      wbusPumpState = (buf[6] & 0x02) || (buf[6] & 0x01);
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
    if (amps < 0.4 || amps > 3.0) systemHalted = true;
  }
}

float readAmps() {
  long sum = 0;
  for(int i=0; i<10; i++) sum += analogRead(CURRENT_PIN);
  float voltage = (sum / 10.0 * 5.0) / 1024.0;
  return abs(voltage - 2.5) / 0.185; 
}

void sendWBus(byte cmd) {
  byte p[] = { 0x4F, 0x01, cmd };
  wBus.write(p[0]); wBus.write(p[1]); wBus.write(p[2]);
  wBus.write(p[0] ^ p[1] ^ p[2]);
}

void sendExtended(byte* data, int len) {
  wBus.write(0x4F); wBus.write((byte)len);
  byte crc = 0x4F ^ (byte)len;
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





*/
