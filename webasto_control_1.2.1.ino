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
#define ADDR_FROM_HEATER  0x42 // Адресация: Котел (4) -> таймер (2) w-bus
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
unsigned long lastActionTime = 0;    // Таймер для отслеживания таймаута ответа wbus
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
unsigned long now; // переменная хранения текущего врнмени для запросоы wBus
unsigned long lastBusActivity = 0;  // Время последнего сообщения в линии (любого)
unsigned long lastQueryTime = 0;    // Время, когда МЫ последний раз что-то спрашивали

unsigned long lastSupportTime = 0;    // Время, когда МЫ последний раз отправляли поддержку горения

const unsigned long BUS_IDLE_TIME = 250; // Ждем 250мс тишины, чтобы не мешать другим (Starline)
const unsigned long QUERY_INTERVAL = 1000; // Опрашиваем котел раз в 1 секунду
const unsigned long SUPPORT_INTERVAL = 5000; // Шлём сигнал на поддержание горения раз в 5 секунду (не более 15 сек?)

byte currentQueryIndex = 0; // Номер текущего запроса из списка ниже
byte queries[] = {0x05, 0x03, 0x07}; // Список ID параметров: Темп, Помпа, Вольты, Ошибки

byte rxBufWBus[13]; // Корзина (буфер), куда складываем приходящие байты
byte rxIdxWBus = 0;  // Счетчик: сколько байт уже лежит в корзине
// int echoSkip = 0; // Счетчик для удаления "эха" (своих же отправленных байт)
//

// unsigned long lastWBusQuery = 0;

bool canPumpActive = false;       // Статус помпы по CAN
bool canPumpTimeout = false;     // Флаг отсутствия сигнала работы помпы из CAN
// вероято уже лишнее bool startCommandAccepted = false; //флаг принятия команды на пуск
unsigned long lastCanPumpMsg = 0; // Таймер последнего пакета помпы по КАН 0x20 0x08
//unsigned long lastBlink = 0;      // Таймер для мигания LED

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
  // CAN0.begin(MCP_ANY, CAN_100KBPS, MCP_8MHZ)
  // ДИАГНОСТИКА. Эта же команда
  if(CAN0.begin(MCP_ANY, CAN_100KBPS, MCP_8MHZ) == CAN_OK) Serial.println("MCP2515 Инициализирован успешно!");
  else Serial.println("Ошибка инициализации MCP2515...");

  CAN0.setMode(MCP_NORMAL);   // Выбираем нормальный режии, чтобы разрешить отправку сообщений

  pinMode(CAN0_INT, INPUT);                            // Configuring pin for /INT input

   // Инициализация шины Webasto на 2400 бод
  wBus.begin(2400, CSERIAL_8E1);

  // Настройки для вентилятора, помпы
  pinMode(PUMP_RELAY, OUTPUT);
  pinMode(CLIMATE_RELAY, OUTPUT);
  pinMode(CAN0_INT, INPUT); // Пин прерывания CAN

  // Настройки для светодиода
  pinMode(LED_PIN, OUTPUT);
  button.attachClick(handleClickLogic);
  button.attachLongPressStop(handleLongPress);
  button.setClickMs(400); 

  // Выключаем всё (инверсная логика реле: HIGH = выкл)
  digitalWrite(PUMP_RELAY, LOW);   
  digitalWrite(CLIMATE_RELAY, HIGH); 
  digitalWrite(LED_PIN, LOW);       // Ошибки нет (Надо ли это???)

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

  // --- 3. УПРАВЛЕНИЕ РЕЛЕ ПОМПЫ ---
  // Работает, если ХОТЯ БЫ ОДНА шина активна И нет аппаратной поломки (pumpIsBroken)
  if ((wbusPumpState || canPumpActive) && !pumpIsBroken) {
    digitalWrite(PUMP_RELAY, HIGH); 
    checkPumpHealth(); // Твоя защита по току ACS712
  } else {
    digitalWrite(PUMP_RELAY, LOW);
  }
  manageClimate();

  // --- 4. ДЕЙСТВИЯ ПРИ ПОЛОМКЕ ПОМПЫ  ---
  if (pumpIsBroken) {
    
    digitalWrite(PUMP_RELAY, LOW);
    digitalWrite(CLIMATE_RELAY, HIGH);
    OCR1A = 0;

    // Защита котла от перегрева. Если помпа сломана, шлём котлу сигнал на стоп.
    //if (wbusPumpState || canPumpActive) {
    //stopSystem("Сработка защиты по помпе");
//}
    // МЕДЛЕННОЕ мигание (500мс) - Поломка помпы (Ток)
    //if (millis() - lastBlink > 500) {
    //  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    //  lastBlink = millis();
    //}
  } 
 // else if (canPumpTimeout) {
    // БЫСТРОЕ мерцание (100мс) - Отсутствие сигнала работы помпы по CAN
   // if (millis() - lastBlink > 100) {
    //  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    //  lastBlink = millis();
   // }
  //} 
 // else {
   // digitalWrite(LED_PIN, LOW); // Ошибок нет
 // }

  // --- 5. НЕБЛОКИРУЮЩЕЕ ЧТЕНИЕ W-BUS ---
  //checkWBusSerial(); 

  // Пример управления через монитор порта: '1' - старт, '0' - стоп
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '1') startSystem(); 
    if (c == '0') stopSystem("Команда пользователя");
    if (c == '2') sendWBusQuery();
    if (c == '3') sendWBusDelERR();
  }
  
    // СЛУШАЕМ ШИНУ W-BUS
  while (wBus.available()) {
    byte b = wBus.read();
    lastBusActivity = millis(); // Фиксируем, что на шине кто-то говорит (мы или Starline)
    
   // if (echoSkip > 0) { echoSkip--; continue; } // Если это наше эхо — просто выкидываем байт

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
  }

  // ОТПРАВЛЯЕМ СВОИ ОПРОСЫ И ПОДДЕРЖАНИЕ ГОРЕНИЯ

  if  (isHeaterRunning) {
    supportHeating();
    sendWBusQuery();
  }
  //sendWBusQuery();
    // Функции кнопци и лампочки
    button.tick();
    checkTimer();
    updateLedStatus();
  
}

                                      // --- ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ---

void decodeMessage(byte* data, byte len) {
  if (data[2] == 0xD0) { // Если 3-й байт равен D0 — это правильный ответ от котла
    byte id = data[3]; // Смотрим, на какой именно ID пришел ответ
    switch (id) {
      case 0x05: // Пришел ответ на запрос температуры
        coolantTemp = map((int)data[4], 153, 204, 36, 10);
        Serial.print("W-Bus. Температура: ");Serial.println(coolantTemp);
        voltageVal = ((float)data[5] * 0.0683); // Считаем вольты
        Serial.print("W-Bus. Напряжение: "); Serial.println(voltageVal);
        break;
      case 0x03: // Пришел ответ по компонентам
        pumpActive = (data[4] & 0x08); // Проверяем 3-й бит (помпа)
        if (pumpActive) Serial.println("W-Bus. Помпа вкл");
        ///???isHeaterRunning = pumpActive; // Если помпа крутит, значит процесс идет
        break;

        // ЛОГИКА ЗАЩИТЫ
        if (isHeaterRunning && voltageVal < MIN_VOLTAGE) { // Если запущен и напряжение упало
          if (!isVoltageLow) { 
            isVoltageLow = true; // Заметили просадку первый раз
            lowVoltStartTime = millis(); // Включили секундомер
            Serial.println("Warning: Low voltage detected, starting timer...");
          } else if (millis() - lowVoltStartTime > LOW_VOLT_TIMEOUT) {
            stopSystem("Battery Low (Critical Timeout)"); // Если 10 сек прошло — СТОП
            isVoltageLow = false; 
          }
        } else {
          isVoltageLow = false; // Напряжение поднялось — обнулили таймер защиты
        }
        break;
    }
  }
  if (data[2] == 0xC4) { // Если 3-й байт равен D0 — это правильный ответ от котла
    byte id = data[3]; // Смотрим, на какой именно ID пришел ответ
    switch (id) {
      case 0x00: // Пришел ответ - котёл работает
        Serial.println("W-Bus. Котёл работает");
        break;
      case 0xFF: // Пришел ответ по компонентам
        Serial.println("W-Bus. Котёл НЕ работает");
        isHeaterRunning = false;
        sendWBusDelERR();
        break;
    }
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
  lastActionTime = millis();         // Фиксируем время отправки
  retryCount++;                      // Увеличиваем счетчик попыток
}

void executeSupport() {
  byte supportData[] = {0x44, 0x21, 0x00};
  sendExtendedWBus(supportData, 3);    // Отправляем байты в шину
  expectedResponse = 0x44 + 0x80;    // Ждем ответ 0xC4 (44+80)
  lastActionTime = millis();         // Фиксируем время отправки для любой активности
  lastSupportTime = millis(); // фиксируем время отправки для этой команды
  retryCount++;                      // Увеличиваем счетчик попыток
}

void executeStop() {
  byte stopData[] = {0x10};
  sendExtendedWBus(stopData, 1);    // Отправляем команду стоп
  expectedResponse = 0x10 + 0x80;    // Ждем ответ 0x90 (10+80)
  lastActionTime = millis();
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
    if (now - lastBusActivity > BUS_IDLE_TIME && now - lastQueryTime > QUERY_INTERVAL) {
      uint8_t dataQuery[] = {0x50, queries[currentQueryIndex]};
      //uint8_t dataQuery[] = {0x50, 0x05};
      sendExtendedWBus(dataQuery, 2); // Шлем следующий запрос из очереди
      currentQueryIndex = (currentQueryIndex + 1) % 3; // Переходим к следующему параметру (0->1->2(->3)->0)
  }
}

void sendExtendedWBus(byte* data, int len) {
  wBus.write(ADDR_TO_HEATER); wBus.write((byte)len+1);
  byte crc = ADDR_TO_HEATER ^ ((byte)len+1);
  for(int i=0; i<len; i++) { wBus.write(data[i]); crc ^= data[i]; }
  wBus.write(crc);
//  echoSkip = len + 2;       // Помечаем, что эти байты вернутся к нам как эхо, их надо проигнорировать
  lastQueryTime = millis();  // Запоминаем время отправки
  lastBusActivity = millis(); // Считаем отправку тоже активностью на шине
}
                    // --- ГЛАВНЫЙ ОБРАБОТЧИК (ДИСПЕТЧЕР) ---

void checkWBusResponse(byte byteCheck) {

  // Блок 1: Если мы ждем подтверждения команды (Start или Stop)
    if (expectedResponse == byteCheck) {       // Если получен нужный байт
      if ((currentState == SENDING_START) || (currentState == SENDING_SUPPORT)) isHeaterRunning = true;
      if (currentState == SENDING_STOP)  isHeaterRunning = false;
      currentState = IDLE;           // Команда принята, возвращаемся в покой
      Serial.println("W-Bus: OK! Response received.");
    } 
    else if (now - lastActionTime > TIMEOUT) { // Если ответа нет дольше 500мс
      if (retryCount < 5) {          // Если попытки еще остались
        Serial.print("W-Bus: Retry #"); Serial.println(retryCount + 1);
        if (currentState == SENDING_START) executeStart();
        else if (currentState == SENDING_SUPPORT) executeSupport();
        else executeStop();
      } else {                       // Если все 5 попыток провалены
        Serial.println("W-Bus: Error! No response after 5 attempts.");
        currentState = IDLE;         // Сдаемся и выходим в покой
      }
    }
  // Блок 2: Обычный опрос состояния (работает только в IDLE)
  
  /*
  if (now - lastQueryTime > QUERY_INTERVAL) {
    byte queryData[] = {0x50, queries[currentQueryIndex]};
    sendExtendedWBus(queryData, 2);  // Шлем запрос из очереди
    currentQueryIndex = (currentQueryIndex + 1) % 4; // Листаем очередь 0-1-2-3
    lastQueryTime = now;             // Сбрасываем таймер интервала
  }*/
}
                     // --- ФУНКЦИИ КНОПКИ И ЛАМПОЧКИ ---

void handleClickLogic() {
  int clicks = button.getNumberClicks();
  switch (clicks) {
    case 1:
      isTimerActive = true;
      timerStartTime = millis();
      break;
    case 2:
      isTimerActive = false;
      startSystem();
      break;
    case 3:
      blinkCount = 3;
      sendWBusDelERR(); //сброс ошибок вебасто
      pumpIsBroken = false;  //сброс ошибки помпы
      break;
  }
}

void handleLongPress() {
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
  else if (pumpIsBroken) {
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
  
  
  
  
void startSystem() {
  isHeaterRunning = true;
 // byte startData[] = {0x21, 0x01, mins};
  byte startData[] = {0x21, timeWorkWebasto};
  sendExtendedWBus(startData, 2);
  Serial.println("W-Bus: START sent");
}
*/


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
