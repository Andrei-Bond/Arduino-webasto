// Webasto CAN simulation + w-bus + pump relay + control pump
// + corntrol climat + button & led

#include <mcp_can.h>
#include <CustomSoftwareSerial.h> // Библиотека для работы с K-line на любых пинах
#include <SPI.h>
#include <OneButton.h> // Библиотека работы с кнопкой

// --- ПИНЫ ---
#define WBUS_RX      6  // Пин Ардуино, куда приходит сигнал от Webasto
#define WBUS_TX      7 // Пин Ардуино, который отправляет сигнал в Webasto
#define FAN_PWM_PIN  9      // Таймер 1 (400 Гц)
#define PUMP_RELAY   5      // Управление реле помпы
#define CLIMATE_RELAY 4     
#define LED_PIN      A1     
#define CURRENT_PIN  A0      // Измерение тока, потребляемого помпой
#define CAN0_INT      2      // Прерывание MCP2515
#define BUTTON_PIN    3 

#define ADDR_TO_HEATER  0x24 // Адресация: Таймер (2) -> Котел (4) w-bus
//#define ADDR_FROM_HEATER  0x42 // Адресация: Котел (4) -> таймер (2) w-bus
#define timeWorkWebasto 60  // сколько времени будет работать Вебасто, мин

OneButton button(BUTTON_PIN, true);
CustomSoftwareSerial wBus(WBUS_RX, WBUS_TX); // Создаем объект "виртуального" порта


int coolantTemp = 0;   // Сюда сохраняем температуру
float voltageVal = 13.0; // Сюда сохраняем напряжение
bool wbusPumpState = false; // Флаг сигнала работы помпы из W-Bus
bool pumpActive = false; // Флаг: работает ли помпа (true/false)
bool pumpIsBroken = false;  // Флаг: сломана ли помпа (true/false
bool isHeaterRunning = false; // Флаг: запущен ли котел в целом
bool isTimerActive = false; // Флаг работы таймера запуска Вебасто
unsigned long timerStartTime = 0; // Хранение времени начала работы таймера запуска Вебсто
const unsigned long DELAY_TIME = 27000000; // Время, через которое запустится Вебасто по таймеру, мс


  // Состояния системы отправки команд w-bus
// --- Глобальные переменные и настройки ---
enum WBusState { IDLE, SENDING_START, SENDING_SUPPORT, SENDING_STOP }; // Состояния: Ожидание, Пуск, Стоп
WBusState currentState = IDLE;       // Текущее состояние системы


int retryCount = 0;                  // Счетчик попыток (до 5) отправки wbus
byte expectedResponse = 0;           // Байт, который мы ждем от Вебасто (Команда + 0x80)

// Константы времени
const unsigned long TIMEOUT = 500;   // Ждем ответ от печки 500мс


//




// Параметры защиты аккумулятора
const float MIN_VOLTAGE = 11.4;         // Порог, ниже которого отключаем котел
const unsigned long LOW_VOLT_TIMEOUT = 10000; // 10 сек (время ожидания перед отключением)
unsigned long lowVoltStartTime = 0;     // Таймер: когда именно упало напряжение
bool isVoltageLow = false;              // Флаг: находится ли вольтаж в опасной зоне сейчас
//

// Тайминги работы шины w-bus
unsigned long now; // переменная хранения текущего времени для запросов wBus
unsigned long lastWBusActivity = 0;  // Время последнего сообщения в линии (любого)
unsigned long lastQueryTime = 0;    // Время, когда МЫ последний раз что-то спрашивали

unsigned long lastSupportTime = 0;    // Время, когда МЫ последний раз отправляли поддержку горения

const unsigned long BUS_IDLE_TIME = 250; // Ждем 250мс тишины, чтобы не мешать другим (Starline)
const unsigned long QUERY_INTERVAL = 1000; // Опрашиваем котел раз в 1 секунду
const unsigned long SUPPORT_INTERVAL = 5000; // Шлём сигнал на поддержание горения раз в 5 секунду (не более 15 сек?)

byte currentQueryIndex = 0; // Номер текущего запроса из списка ниже
byte queries[] = {0x05, 0x03}; // Список ID параметров: Темп, Помпа, Вольты, Ошибки

byte rxBufWBus[13]; // Корзина (буфер), куда складываем приходящие байты
byte rxIdxWBus = 0;  // Счетчик: сколько байт уже лежит в корзине


bool canPumpActive = false;       // Статус помпы по CAN
bool canPumpTimeout = false;     // Флаг отсутствия сигнала работы помпы из CAN
unsigned long lastCanPumpMsg = 0; // Таймер последнего пакета помпы по КАН 0x20 0x08



// Переменные для мигания
int blinkCount = 0;           
unsigned long lastBlinkMs = 0; 
bool blinkState = false;      

long unsigned int rxId; // хранилище ИД из CAN
unsigned char len = 0; // хранилище длины данных из CAN
unsigned char rxBuf[8]; // хранилище массива данных из CAN
byte askStat1[8] = {0x00, 0x71, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // ответ для вебасто (есть связь?)

MCP_CAN CAN0(10);     // Set CS to pin 10

void setup() {
  Serial.begin(115200);
  
  // Initialize MCP2515 running at 8MHz with a baudrate of 100kb/s and the masks and filters disabled.
  // ДИАГНОСТИКА. Эта же команда
  if(CAN0.begin(MCP_ANY, CAN_100KBPS, MCP_8MHZ) == CAN_OK) Serial.println("MCP2515 Инициализирован успешно!");
  else Serial.println("Ошибка инициализации MCP2515...");

  CAN0.setMode(MCP_NORMAL);   // Выбираем нормальный режим, чтобы разрешить отправку сообщений

  pinMode(CAN0_INT, INPUT);                            // Configuring pin for /INT input

   // Инициализация шины Webasto на 2400 бод
  wBus.begin(2400, CSERIAL_8E1);

  // Настройки для вентилятора, помпы
  pinMode(PUMP_RELAY, OUTPUT);
  pinMode(CLIMATE_RELAY, OUTPUT);
  pinMode(CAN0_INT, INPUT); // Пин прерывания CAN

  // Настройки для светодиода
  pinMode(LED_PIN, OUTPUT);
  button.attachMultiClick(handleMultiClick)
  button.attachLongPressStart(handleLongPress);
  button.setClickMs(400); 

  // Выключаем всё
  digitalWrite(PUMP_RELAY, LOW);   
  digitalWrite(CLIMATE_RELAY, LOW); 
  digitalWrite(LED_PIN, LOW);       // Ошибки нет

  // Настройка Таймера 1 (400 Гц на D9)
  pinMode(FAN_PWM_PIN, OUTPUT);
  TCCR1A = _BV(COM1A1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);
  ICR1 = 4999; 
  OCR1A = 0; 
  
}




void loop() {
  now = millis();
  
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
              Serial.println("КАН: помпа выкл штатно");
            canPumpActive = false; // Выключили штатно
          
        }
      }
  
    // Как только есть активность — планируем опрос W-Bus
  //  if (millis() - lastWBusQuery > 3000) {
  //    sendWBusQuery(); 
   //   lastWBusQuery = millis();
 //   }
  }

  // --- 2. ЗАЩИТА: ТАЙМАУТ СВЯЗИ CAN (15 секунд) ---
     //Делаем запрос состояния помпы каждые 15 секунд
    if (canPumpActive && (millis() - lastCanPumpMsg > 15000)) {
    CAN0.sendMsgBuf(0x3E5, 0, 8, askStat1);
  }
      //если нет связи по кан более 30 секунд - включаем ошибку
  
  if (canPumpActive && (millis() - lastCanPumpMsg > 30000)) {
    canPumpActive = false;
    canPumpTimeout = true; // Устанавливаем флаг отсутствия сигнала работы помпы по CAN
    Serial.println("Сигнал работы помпы из CAN отсутствует!");
  }

  // --- 3. УПРАВЛЕНИЕ РЕЛЕ ПОМПЫ И КЛИМАТОМ---
  // Работает, если ХОТЯ БЫ ОДНА шина активна И нет аппаратной поломки (pumpIsBroken)
  if ((wbusPumpState || canPumpActive)/* && !pumpIsBroken*/) {
    digitalWrite(PUMP_RELAY, HIGH); 
    checkPumpHealth(); // Твоя защита по току ACS712
  } else {
    digitalWrite(PUMP_RELAY, LOW);
  }
  manageClimate();

  // --- 4. ДЕЙСТВИЯ ПРИ ПОЛОМКЕ ПОМПЫ  ---
  if (pumpIsBroken) {
    
    //digitalWrite(PUMP_RELAY, LOW);      !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    //digitalWrite(CLIMATE_RELAY, LOW);     !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    OCR1A = 0;

    // Защита котла от перегрева. Если помпа сломана, шлём котлу сигнал на стоп.
    //if (wbusPumpState || canPumpActive || isHeatingRunning) {   !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    //stopSystem("Сработка защиты по помпе");     !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//}   !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  }  


  // --- 5. НЕБЛОКИРУЮЩЕЕ ЧТЕНИЕ W-BUS ---

  // Пример управления через монитор порта: '1' - старт, '0' - стоп
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '1') startSystem(); 
    if (c == '0') stopSystem("Команда пользователя");
    if (c == '2') sendWBusQuery();
    if (c == '3') sendWBusDelERR();
    if (c == '4') testClimate(0);
    if (c == '5') testClimate(30);
    if (c == '6') testClimate(70);
  }
 
    // СЛУШАЕМ ШИНУ W-BUS
  if (wBus.available()) {
    byte b = wBus.read();
    lastWBusActivity = now; // Фиксируем, что на шине кто-то говорит (мы или Starline)
    
    // Ищем начало пакета (адрес 4F, 43 и т.д.)
    if (rxIdxWBus == 0 && (b & 0xF0) == 0x40) {
      rxBufWBus[rxIdxWBus++] = b;
    } else if (rxIdxWBus > 0) {
      rxBufWBus[rxIdxWBus++] = b; // Складываем байты в буфер
      if (rxIdxWBus > 1) {
        byte expectedLen = rxBufWBus[1] + 2; // Вычисляем, сколько байт должно быть в пакете всего
        if (rxIdxWBus == expectedLen) { // Если пакет собрался целиком
          byte crc = 0;
          for (int i = 0; i < rxIdxWBus - 1; i++) crc ^= rxBufWBus[i]; // Считаем CRC пришедшего пакета
          if (crc == rxBufWBus[rxIdxWBus - 1]){ 
            if (currentState != IDLE) {
              checkWBusResponse(rxBufWBus[2]);
            };
            decodeMessage(rxBufWBus, rxIdxWBus); // Если CRC совпал — расшифровываем
          rxIdxWBus = 0; // Чистим буфер для нового сообщения
          }
        }
      }
    } 
    if (rxIdxWBus >= 13) rxIdxWBus = 0; // Защита от переполнения корзины
  } else if (currentState != IDLE) {
    checkWBusResponse(0xFF); // Если нет активности в шине W-bus, но ожидается ответ - запустить механизм проверки ответа с неверным байтом FF
  }

  // ОТПРАВЛЯЕМ СВОИ ОПРОСЫ И ПОДДЕРЖАНИЕ ГОРЕНИЯ

  if  (isHeaterRunning) {
    supportHeating();
    sendWBusQuery();
  }
  //sendWBusQuery();
    // Функции кнопки и лампочки
    button.tick();
    checkTimer();
    updateLedStatus();
    
}

                                      // --- ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ---

void decodeMessage(byte* data, byte len) {
  if (data[2] == 0xD0) { // Если 3-й байт равен D0 — это правильный ответ от котла на запрос состояний
    byte id = data[3]; // Смотрим, на какой именно ID пришел ответ
    switch (id) {
      case 0x05: // Пришел ответ на запрос температуры и напряжения
        if (data[4] < 204) {
        coolantTemp = map((int)data[4], 153, 204, 36, 10);
        } else if (data[4] < 234) {
        coolantTemp = map((int)data[4], 204, 234, 10, -20);
        } else {
        coolantTemp = map((int)data[4], 234, 240, -20, -35);
        }
        Serial.print("W-Bus. Температура: ");Serial.println(coolantTemp);
        voltageVal = ((float)data[5] * 0.0683); // Считаем вольты
        Serial.print("W-Bus. Напряжение: "); Serial.println(voltageVal);
                // ЛОГИКА ЗАЩИТЫ
        if (isHeaterRunning && voltageVal < MIN_VOLTAGE) { // Если запущен и напряжение упало
          if (!isVoltageLow) { 
            isVoltageLow = true; // Заметили просадку первый раз
            lowVoltStartTime = millis(); // Включили секундомер
            Serial.println("Warning: Low voltage detected, starting timer...");
          } else if (millis() - lowVoltStartTime > LOW_VOLT_TIMEOUT) {
            stopSystem("Bat.Low 10сек"); // Если 10 сек прошло — СТОП
            isVoltageLow = false; 
          }
        } else {
          isVoltageLow = false; // Напряжение поднялось — обнулили таймер защиты
        }
        break;
        
      case 0x03: // Пришел ответ по компонентам
        pumpActive = (data[4] & 0x08); // Проверяем 3-й бит (помпа)
        if (pumpActive) Serial.println("W-Bus. Помпа вкл");
        break;
    }
  } else if (data[2] == 0xC4) { // Если 3-й байт равен C4 —  ответ от котла на поддержание работы
    byte id = data[3]; // Смотрим, на какой именно ID пришел ответ
    switch (id) {
      case 0x00: // Пришел ответ - котёл работает
        Serial.println("W-Bus. Котёл работает");
        isHeaterRunning = true;
        break;
      case 0xFF: // Пришел ответ - котел не работает
        Serial.println("W-Bus. Котёл НЕ работает");
        isHeaterRunning = false;
        sendWBusDelERR();
        break;
    }
  } else if (data[2] == 0xA1) { // Если 3-й байт равен A1 —  ответ от котла на команду запуска
    isHeaterRunning = true;
  } else if (data[2] == 0x90) { // Если 3-й байт равен 90 —  ответ от котла на команду стоп
    isHeaterRunning = false;
  }
}

void manageClimate() {
  if ((coolantTemp > 35) && isHeaterRunning) {
    digitalWrite(CLIMATE_RELAY, HIGH);
    OCR1A = 3500; // 70% от 4999
  } else if (coolantTemp > 5 && isHeaterRunning) {
    digitalWrite(CLIMATE_RELAY, HIGH);
    OCR1A = 1500; // 30% от 4999  
  } else {
    digitalWrite(CLIMATE_RELAY, LOW);
    OCR1A = 0;
  }
}
// Тестовая функция запуска вентилятора климата
void testClimate(int powerVent) {
  // Проверяем, входит ли число в диапазон от 1 до 100%
  if (powerVent >= 1 && powerVent <= 100) {
    digitalWrite(CLIMATE_RELAY, HIGH); // Включаем реле климата
    
    // Альтернативный вариант (ограничит максимум на 4999, если это предел таймера):
    OCR1A = map(powerVent, 1, 100, 50, 4999);
  } 
  // Если передали 0 или любые другие числа вне диапазона 1-100
  else {
    digitalWrite(CLIMATE_RELAY, LOW);  // Отключаем реле климата
    OCR1A = 0;                         // Обнуляем ШИМ-регистр
  }
}

void checkPumpHealth() {
  static unsigned long pTimer = 0;
  if (pTimer == 0) pTimer = millis();
  if (millis() - pTimer > 3000) {
    float amps = readAmps();
    Serial.print("Ток помпы: "); Serial.println(amps);
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


              // --- ФУНКЦИИ УПРАВЛЕНИЯ (Вызываются извне, например по кнопке) ---

void startSystem() {
  if (currentState != IDLE) return;  // Если уже идет процесс, игнорируем новый вызов
  retryCount = 0;                    // Сбрасываем счетчик для новой операции
  currentState = SENDING_START;      // Переходим в режим запуска
  executeStart();                    // Делаем первую попытку
  Serial.println("!!! ACTION: START");
}

void supportHeating() {
  if (now - lastSupportTime > SUPPORT_INTERVAL) {
  retryCount = 0;                    // Сбрасываем счетчик для новой операции
  currentState = SENDING_SUPPORT;      // Переходим в режим запуска
  executeSupport();                    // Делаем первую попытку
  Serial.println("!!! ACTION: SUPPORT");
  }
}


void stopSystem(String reason) {
  // Стоп имеет приоритет, поэтому не проверяем IDLE, а просто прерываем всё
  retryCount = 0;
  currentState = SENDING_STOP;
  Serial.print("!!! ACTION: STOP. Reason: "); Serial.println(reason);
  executeStop();                     // Делаем первую попытку стопа
}

                          // --- ФУНКЦИИ ФИЗИЧЕСКОЙ ОТПРАВКИ W-Bus ---

void executeStart() {
  byte startData[] = {0x21, timeWorkWebasto};
  sendExtendedWBus(startData, 2);    // Отправляем байты в шину
  expectedResponse = 0x21 + 0x80;    // Ждем ответ 0xA1 (21+80)
  retryCount++;                      // Увеличиваем счетчик попыток
}

void executeSupport() {
  byte supportData[] = {0x44, 0x21, 0x00};
  sendExtendedWBus(supportData, 3);    // Отправляем байты в шину
  expectedResponse = 0x44 + 0x80;    // Ждем ответ 0xC4 (44+80)
  lastSupportTime = now; // фиксируем время отправки для этой команды
  retryCount++;                      // Увеличиваем счетчик попыток
}

void executeStop() {
  byte stopData[] = {0x10};
  sendExtendedWBus(stopData, 1);    // Отправляем команду стоп
  expectedResponse = 0x10 + 0x80;    // Ждем ответ 0x90 (10+80)
  retryCount++;
}


void sendWBusDelERR() {
  // Это отправляет сигнализация Старлайн после получения ответа СТОП на поддеражние
      //uint8_t dataDelERR[] = {0x56, 0x01};
      //sendExtendedWBus(dataDelERR, 2);
  // Это отправляет программа WTT при команде "Очистить архив сбоев"
      uint8_t dataDelERR[] = {0x52};
      sendExtendedWBus(dataDelERR, 1);
}

void sendWBusQuery() {
    // Если на шине тишина 250мс И прошел 1 сек с нашего последнего вопроса:
    if (now - lastWBusActivity > BUS_IDLE_TIME && now - lastQueryTime > QUERY_INTERVAL) {
      uint8_t dataQuery[] = {0x50, queries[currentQueryIndex]};
      //uint8_t dataQuery[] = {0x50, 0x05};
      sendExtendedWBus(dataQuery, 2); // Шлем следующий запрос из очереди
      currentQueryIndex = (currentQueryIndex + 1) % 2; // Переходим к следующему параметру (0->1(->2->3)->0)
  }
}

void sendExtendedWBus(byte* data, int len) {
  wBus.write(ADDR_TO_HEATER); wBus.write((byte)len+1);
  Serial.println("отправка в бас");
  byte crc = ADDR_TO_HEATER ^ ((byte)len+1);
  for(int i=0; i<len; i++) { wBus.write(data[i]); crc ^= data[i]; }
  wBus.write(crc);
  lastQueryTime = now;  // Запоминаем время отправки
  lastWBusActivity = now; // Считаем отправку тоже активностью на шине
}
                    // --- ГЛАВНЫЙ ОБРАБОТЧИК (ДИСПЕТЧЕР) ---

void checkWBusResponse(byte byteCheck) {

  // Блок 1: Если мы ждем подтверждения команды (Start или Stop)
    if (expectedResponse == byteCheck) {       // Если получен нужный байт
      if ((currentState == SENDING_START) || (currentState == SENDING_SUPPORT)) isHeaterRunning = true;
      if (currentState == SENDING_STOP)  isHeaterRunning = false;
      currentState = IDLE;           // Команда принята, возвращаемся в покой
      Serial.println("W-Bus: OK! Ответ получен.");
    } 
    else if (now - lastWBusActivity > TIMEOUT) { // Если ответа нет дольше 500мс
      if (retryCount < 5) {          // Если попытки еще остались
        Serial.print("W-Bus: Попытка #"); Serial.println(retryCount + 1);
        if (currentState == SENDING_START) executeStart();
        else if (currentState == SENDING_SUPPORT) executeSupport();
        else executeStop();
      } else {                       // Если все 5 попыток провалены
        Serial.println("W-Bus: Ошибка! Нет ответа после 5 попыток.");
        currentState = IDLE;         // Сдаемся и выходим в покой
      }
    }
}
                     // --- ФУНКЦИИ КНОПКИ И ЛАМПОЧКИ ---

void handleMultiClick() {
  int clicks = button.getNumberClicks();
  blinkCount = clicks; 
  switch (clicks) {
    case 1:
    Serial.println("Нажата кнопка 1 раз");
      isTimerActive = true;
      timerStartTime = millis();
      break;
    case 2:
    Serial.println("Нажата кнопка 2 раза");
      isTimerActive = false;
      startSystem();
      break;
    case 3:
    Serial.println("Нажата кнопка 3 раза");
      sendWBusDelERR(); //сброс ошибок вебасто
      pumpIsBroken = false;  //сброс ошибки помпы
      break;
  }
}

void handleLongPress() {
  Serial.println("Нажата кнопка долгим нажатием");
  isTimerActive = false;
  stopSystem("По кнопке");
  blinkCount = 1; 
}

void checkTimer() {
  if (isTimerActive && (millis() - timerStartTime >= DELAY_TIME)) {
    isTimerActive = false;
    startSystem();
  }
}

void updateLedStatus() {

  // 1. Подтверждение нажатий (самый высокий приоритет)
  if (blinkCount > 0) {
    if (now - lastBlinkMs >= 150) {
      lastBlinkMs = now;
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);
      if (!blinkState) blinkCount--; 
    }
  }
  // 2. Система запущена (горит постоянно)
  else if (isHeaterRunning) {
    digitalWrite(LED_PIN, HIGH);
  } 
  // 3. ОШИБКА: Сработала защита по току. Частое моргание (например, 5 раз в секунду)
  else if (pumpIsBroken) {
    // Период 200мс (100мс включен / 100мс выключен)
    digitalWrite(LED_PIN, (now / 100) % 2 == 0);
  }
  // 4. ОШИБКА: Нет сигнала помпы по CAN. Частое моргание (например, 5 раз в секунду)
  else if (canPumpTimeout) {
    // Период 400мс (200мс включен / 200мс выключен)
    digitalWrite(LED_PIN, (now / 100) % 3 == 0);  
  }
  // 5. Работает таймер (медленное моргание 1 раз в 2 сек)
  else if (isTimerActive) {
    digitalWrite(LED_PIN, (now / 1000) % 2 == 0);
  } 
  // 6. Выключено
  else {
    digitalWrite(LED_PIN, LOW);
  }
}








  



  
  

/*
Итоговые рекомендации и нюансы:
Защита MOSFET: При управлении вентилятором Peugeot 508 "Low Side" (замыкание на массу), MOSFET будет коммутировать значительный ток. Используй IRLZ44N (он управляется логическим уровнем 5В) и обязательно закрепи его на радиаторе.
Резистор 10кОм: Обязательно поставь резистор между Gate (затвором) и Source (истоком) MOSFET. Это гарантирует, что вентилятор не включится на максимум, пока Arduino загружается.
Контроль тока: Если помпа VAG штатная, она потребляет около 1А. Если ты увидишь в мониторе порта значения 0.0 или 0.1 при работающей помпе — проверь питание ACS712 (оно должно быть строго 5В) или исправность шунта.
Безопасность CAN: Поскольку ты читаешь CAN, чтобы активировать W-Bus, убедись, что твой модуль MCP2515 имеет общий минус (GND) с Arduino и остальной проводкой.


Рекомендации по эксплуатации:
Охлаждение: MOSFET на пине D9 (обдув салона) будет нагружен — обязательно используй радиатор.
Помехи: На линии W-Bus в авто много шумов. Используй экранированный провод или витую пару для стабильности.
Защита: Если помпа подключена длинным проводом, добавь параллельно ей диод (катодом к "+"), чтобы обратный ток индуктивности не сжег реле или Ардуино.



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
  
  
