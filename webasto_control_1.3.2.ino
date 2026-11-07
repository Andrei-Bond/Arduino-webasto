// Webasto CAN simulation + w-bus + pump relay + control pump
// + corntrol climat + button & led + energy low


//TTT - тестовые команды


#include <mcp_can.h>
#include <CustomSoftwareSerial.h> // Библиотека для работы с K-line на любых пинах
#include <SPI.h>
#include <OneButton.h> // Библиотека работы с кнопкой

// ЭНЕРГОСБЕРЕЖЕНИЕ И СОН 
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>


// --- ПИНЫ ---
#define WBUS_RX      6  // Пин Ардуино, куда приходит сигнал от Webasto
#define WBUS_TX      7 // Пин Ардуино, который отправляет сигнал в Webasto
#define FAN_PWM_PIN  9      // Таймер 1 (400 Гц)
#define PUMP_RELAY   5      // Управление реле помпы
#define CLIMATE_RELAY 4
#define DEFROST_SENSOR  A2      // Герконы на положении заслонок климата "НАГРЕВ"   
#define LED_PIN      A1     
#define CURRENT_PIN  A0      // Измерение тока, потребляемого помпой
#define CAN0_INT      2      // Прерывание MCP2515
#define BUTTON_PIN    3 

#define ADDR_TO_HEATER  0x24 // Адресация: Таймер (2) -> Котел (4) w-bus
//#define ADDR_FROM_HEATER  0x42 // Адресация: Котел (4) -> таймер (2) w-bus
#define timeWorkWebasto 60  // сколько времени будет работать Вебасто, мин

OneButton button(BUTTON_PIN, true);
CustomSoftwareSerial wBus(WBUS_RX, WBUS_TX); // Создаем объект "виртуального" порта

// --- Глобальные переменные и настройки ---

int coolantTemp = 0;   // Сюда сохраняем температуру
float voltageVal = 13.0; // Сюда сохраняем напряжение
bool wbusPumpActive = false; // Флаг сигнала работы помпы из W-Bus
bool pumpIsBroken = false;  // Флаг: сломана ли помпа (true/false)
bool isHeaterRunning = false; // Флаг: запущен ли котел в целом
bool manualStart = false; // Флаг: ручной рапуск (с кнопким в салоне), отключаем ограничение времени климата
bool startFromStarline =false; // Флаг запуска котла с сигнализации Starline, для другой логики поддержания и опроса, что бы не мешать сигнализации
bool isTimerActive = false; // Флаг работы таймера запуска Вебасто
unsigned long timerStartTime = 0; // Хранение времени начала работы таймера запуска Вебсто
//const unsigned long DELAY_TIME = 31800000; // Время, через которое запустится Вебасто по таймеру, мс
unsigned long timeStartClimate = 0; // Хранение времени начала работы климата
const unsigned long limitClimateTime = 25UL * 60UL * 1000UL; // Лимит работы вентилятора салона, 30 минут
unsigned long timeStartPump = 0; // Таймер запуска помпы
float filteredAmps = 0.0; // Хранит сглаженное значение тока помпы


// --- Параметры защиты аккумулятора ---
const float MIN_VOLTAGE = 11.0;         // Порог, ниже которого отключаем котел
const unsigned long LOW_VOLT_TIMEOUT = 20000; // 20 сек (время ожидания перед отключением)
unsigned long lowVoltStartTime = 0;     // Таймер: когда именно упало напряжение
bool isVoltageLow = false;              // Флаг: находится ли вольтаж в опасной зоне сейчас
bool voltageClimateOK = false;         // Флаг напряжения для климата
//

// --- Для работы с шиной w-bus ---
unsigned long now; // переменная хранения текущего времени для запросов wBus
unsigned long lastWBusActivity = 0;  // Время последнего сообщения в линии (любого)
unsigned long lastQueryTime = 0;    // Время, когда МЫ последний раз что-то спрашивали
unsigned long wBusPumpMsgTime = 0;  // Время последнего сообщения о работе помы по вБас
unsigned long lastSupportTime = 0;    // Время, когда последний раз была команда на поддержку горения (и в принятых W-bus сообщениях тоже)

const unsigned long TIMEOUT = 500;   // Ждем ответ от печки 500мс, если нету, повторяем отправку (для важных запросов)
const unsigned long BUS_IDLE_TIME = 250; // Ждем 250мс тишины, чтобы не мешать другим (Starline)
const unsigned long QUERY_INTERVAL = 2000; // Опрашиваем котел (t, V, PUMP) раз в 2 секунду
const unsigned long SUPPORT_INTERVAL = 11000; // Шлём сигнал на поддержание горения раз в 11 секунду (не более 15 сек)

// --- Состояния системы отправки команд w-bus ---

enum WBusState { IDLE, SENDING_START, SENDING_SUPPORT, SENDING_STOP }; // Состояния (ожидание ответа по W-Bus): Ожидание, Пуск, Поддержание работы, Стоп
WBusState currentState = IDLE;       // Текущее состояние системы 

int retryCount = 0;                  // Счетчик попыток (до 5) отправки wbus (для важных запросов)
byte expectedResponse = 0;           // Байт, который мы ждем от Вебасто (Команда + 0x80)

byte currentQueryIndex = 0; // Номер текущего запроса состояний котла (t, V, PUMP) по W-Bus из списка ниже
byte queries[] = {0x05, 0x03}; // Список ID параметров: t V, PUMP

byte rxBufWBus[13]; // Корзина (буфер), куда складываем приходящие байты
byte rxIdxWBus = 0;  // Счетчик: сколько байт уже лежит в корзине


// --- Переменные для мигания светодиодом кнопки ---
int blinkCount = 0;           
unsigned long lastBlinkMs = 0; 
bool blinkState = false;      

// --- Для работы CAN-модуля ---
long unsigned int rxId; // хранилище ИД из CAN
unsigned char len = 0; // хранилище длины данных из CAN
unsigned char rxBuf[8]; // хранилище массива данных из CAN
byte askStat1[8] = {0x00, 0x71, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // ответ для вебасто (есть связь?)

MCP_CAN CAN0(10);     // Set CS to pin 10

// --- Для обработки сигналов о помпе по CAN ---
bool canPumpActive = false;       // Статус помпы по CAN
bool canPumpTimeout = false;     // Флаг отсутствия сигнала работы помпы из CAN
unsigned long lastCanPumpMsg = 0; // Таймер последнего пакета помпы по КАН 0x20 0x08



// --- ПЕРЕМЕННЫЕ ДЛЯ УПРАВЛЕНИЯ СНОМ И ТАЙМЕРОМ ---
unsigned long lastActivityTime = 0;
const unsigned long INACTIVITY_SLEEP_TIMEOUT = 30000UL; // 30 секунд простоя (30 000 мс) в обычном режиме

// Для 9-часового таймера с использованием сторожевого таймера (WDT)
unsigned long remainingWdtTicks = 0;
const unsigned long TOTAL_TICKS_9_HOURS = (32400000UL - INACTIVITY_SLEEP_TIMEOUT) / 10000UL; // 9 часов (31.8 млн мс) / 8 сек (макс. интервал WDT) (на опыте оказалсядля этой платы 10 сек)

// --- ОБРАБОТЧИКИ ПРЕРЫВАНИЙ (БУДИЛЬНИКИ) ---
// уже есть в AVR библиотеке, вызывает конфликт // ISR(INT0_vect) {} // Прерывание от CAN-контроллера (INT0 / пин D2)
// уже есть в AVR библиотеке, вызывает конфликт // ISR(INT1_vect) {} // Прерывание от кнопки (INT1 / пин D3)
ISR(PCINT22_vect) {} // Прерывание по изменению состояния пина для W-Bus (пин D6 / PCINT22)
ISR(WDT_vect) {} // Прерывание сторожевого таймера (срабатывает каждые 8 секунд во время долгого сна таймера)



void setup() {
//TTT  Serial.begin(115200);
  
  // Initialize MCP2515 running at 8MHz with a baudrate of 100kb/s and the masks and filters disabled.
  // ДИАГНОСТИКА. Эта же команда
/*TTT  if(CAN0.begin(MCP_ANY, CAN_100KBPS, MCP_8MHZ) == CAN_OK) Serial.println("MCP2515 Инициализирован успешно!");
  else Serial.println("Ошибка инициализации MCP2515...");
*/ 
  CAN0.begin(MCP_ANY, CAN_100KBPS, MCP_8MHZ);
  CAN0.setMode(MCP_NORMAL);   // Выбираем нормальный режим, чтобы разрешить отправку сообщений

  pinMode(CAN0_INT, INPUT);                            // Configuring pin for /INT input

   // Инициализация шины Webasto на 2400 бод
  wBus.begin(2400, CSERIAL_8E1);

  // Настройки для вентилятора, помпы
  pinMode(DEFROST_SENSOR, INPUT_PULLUP);
  pinMode(PUMP_RELAY, OUTPUT);
  pinMode(CLIMATE_RELAY, OUTPUT);
  pinMode(CAN0_INT, INPUT); // Пин прерывания CAN

  // Настройки для светодиода
  pinMode(LED_PIN, OUTPUT);
  button.attachClick(handleClickLogic);
  button.attachDoubleClick(handleClickLogic);
  button.attachMultiClick(handleClickLogic);
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
  
  // Инициализируем таймер последней активности для режима сна
  lastActivityTime = millis(); 
}




void loop() {
  now = millis();
  
  // --- 1. ЛОГИКА CAN (MCP2515) ---
  if (!digitalRead(CAN0_INT)) { 
    // Читаем CAN, отвечаем котлу (твой код)          // Если вывод CAN0_INT is LOW, отправляем подтверждение связи
//TTT     Serial.println("Прием кан");
    CAN0.readMsgBuf(&rxId, &len, rxBuf);  // Считывем данные: len = длина данных, buf = байт(ы) данны
    CAN0.sendMsgBuf(0x427, 0, 8, askStat1);
/*
byte errorStatus = CAN0.checkError();
if (errorStatus != 0) {
    Serial.print("CAN Error Status: Code ");
    Serial.println(errorStatus, HEX); 
    // Если выдает коды ошибок или уходит в Bus-Off — дело 100% в физике и резисторах
}
*/
    
  
        if (rxId == 0x3E5 && len >= 2) {
        if ((rxBuf[1] & 0x0A)) {
//TTT         Serial.println("КАН: помпа вкл");
          canPumpTimeout = false; // Сигнал на пуск помпы есть, сбрасываем таймаут
            canPumpActive = true;
            lastCanPumpMsg = now; // Обновляем время активности
          } else {
//TTT              Serial.println("КАН: помпа выкл штатно");
            canPumpActive = false; // Выключили штатно
          
        }
      }
  }

  // --- 2. ЗАЩИТА: ТАЙМАУТ СВЯЗИ CAN (15 секунд) ---
     //Делаем запрос состояния помпы каждые 15 секунд
    if (canPumpActive && (now - lastCanPumpMsg > 15000)) {
    CAN0.sendMsgBuf(0x3E5, 0, 8, askStat1);
    //CAN0.sendMsgBuf(0x427, 0, 8, askStat1);
  }
      //если нет связи по кан более 60 секунд - включаем ошибку
  
  if (canPumpActive && (now - lastCanPumpMsg > 60000)) {
    canPumpActive = false;
    canPumpTimeout = true; // Устанавливаем флаг отсутствия сигнала работы помпы по CAN
//TTT    Serial.println("Сигнал работы помпы из CAN отсутствует!");
  }

  // --- 3. УПРАВЛЕНИЕ РЕЛЕ ПОМПЫ И КЛИМАТОМ---
  // Работает, если ХОТЯ БЫ ОДНА шина активна И нет аппаратной поломки (pumpIsBroken)
  if ((wbusPumpActive || canPumpActive || isHeaterRunning) && !pumpIsBroken) {
    if (digitalRead(PUMP_RELAY) == LOW) {
      timeStartPump = now;
      }
    digitalWrite(PUMP_RELAY, HIGH);
    
    checkPumpHealth(); // Твоя защита по току ACS712
  } else {
    digitalWrite(PUMP_RELAY, LOW);
  }
  

  // --- 4. ДЕЙСТВИЯ ПРИ ПОЛОМКЕ ПОМПЫ  ---
  if (pumpIsBroken) {
    
    digitalWrite(PUMP_RELAY, LOW);      
    digitalWrite(CLIMATE_RELAY, LOW);     
    OCR1A = 0;                          

    // Защита котла от перегрева. Если помпа сломана, шлём котлу сигнал на стоп.
    if (wbusPumpActive || canPumpActive || isHeaterRunning) { 
    stopSystem(/*"Сработка защиты по помпе"*/);  
  }
  }  


  // --- 5. НЕБЛОКИРУЮЩЕЕ ЧТЕНИЕ W-BUS ---
//CAN0.sendMsgBuf(0x427, 0, 8, askStat2);
  // Пример управления через монитор порта: '1' - старт, '0' - стоп
/*TTT  if (Serial.available()) {
    char c = Serial.read();
    if (c == '1') startSystem(); 
    if (c == '0') stopSystem();
    if (c == '2') sendWBusQuery();
    if (c == '3') sendWBusDelERR();
//    if (c == '4') testClimate(0);
//    if (c == '5') testClimate(30);
//    if (c == '6') testClimate(70);
    if (c == '7') CAN0.sendMsgBuf(0x427, 0, 8, askStat1);
 
  }
 */
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


  // ПРОВЕРЯЕМ ЕСТЬ ЛИ СООБЩЕНИЯ О РАБОТЕ ПОМПЫ ПО ВБАС ЗА ПОСЛЕДНИЕ Х СЕКУНД
 if ((now - wBusPumpMsgTime) > 15000) {
  wbusPumpActive = false;
 }
  // ОТПРАВЛЯЕМ СВОИ ОПРОСЫ И ПОДДЕРЖАНИЕ ГОРЕНИЯ

  if  (isHeaterRunning) {
    supportHeating();
    sendWBusQuery();

  }
  //sendWBusQuery();
    // Функции кнопки и лампочки
    button.tick();
    updateLedStatus();
    manageClimate();
    checkTimer();
    
// --- ЭНЕРГОСБЕРЕЖЕНИЕ И СОН ---

// 1. БЛОК СБРОСА ТАЙМЕРА АКТИВНОСТИ
  // Если происходит любое из этих событий, плата не уснет
  if (isHeaterRunning || isTimerActive || canPumpActive || wbusPumpActive || 
      pumpIsBroken || canPumpTimeout || (currentState != IDLE) || 
      (blinkCount > 0) || (digitalRead(PUMP_RELAY) == HIGH)) {
    
    lastActivityTime = now; // Сдвигаем время возможного сна еще на 5 минут вперед
  }

  // 2. БЛОК ПРОВЕРКИ УСЛОВИЯ ДЛЯ ОБЫЧНОГО СНА
  // Если таймер отложенного запуска НЕ активен и прошло 5 минут (300 000 мс)
  if (!isTimerActive && (now - lastActivityTime >= INACTIVITY_SLEEP_TIMEOUT)) {
//TTT    Serial.println("GO SLEEP");
    enterSleep(); // Уходим в глубокий сон до первого внешнего прерывания
  }

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
//TTT       Serial.print("W-Bus. Температура: ");Serial.println(coolantTemp);
        voltageVal = ((float)data[5] * 0.0683); // Считаем вольты
//TTT        Serial.print("W-Bus. Напряжение: "); Serial.println(voltageVal);
                // ЛОГИКА ЗАЩИТЫ
        if (isHeaterRunning && voltageVal < MIN_VOLTAGE) { // Если запущен и напряжение упало
          if (!isVoltageLow) { 
            isVoltageLow = true; // Заметили просадку первый раз
            lowVoltStartTime = now; // Включили секундомер
//TTT            Serial.println("Низкое напряжение! Запуск таймера.");
          } else if (now - lowVoltStartTime > LOW_VOLT_TIMEOUT) {
            stopSystem(/*TTT"Низкое напряжение 10сек"*/); // Если 10 сек прошло — СТОП
            isVoltageLow = false; 
          }
        } else {
          isVoltageLow = false; // Напряжение поднялось — обнулили таймер защиты
        }

        if (!voltageClimateOK && voltageVal > 11.8) {
        voltageClimateOK = true;
        } else if (voltageClimateOK && voltageVal < 11.4) {
        voltageClimateOK = false;
        }
        
        break;
      case 0x03: // Пришел ответ по компонентам
        wbusPumpActive = (data[4] & 0x08); // Проверяем 3-й бит (помпа)
        wBusPumpMsgTime = now;
//TTT        if (wbusPumpActive) Serial.println("W-Bus. Помпа вкл");
        break;
    }
  } else if (data[2] == 0xC4) { // Если 3-й байт равен C4 —  ответ от котла на поддержание работы
    byte id = data[3]; // Смотрим, на какой именно ID пришел ответ
    switch (id) {
      case 0x00: // Пришел ответ - котёл работает
//TTT        Serial.println("W-Bus. Котёл работает");
        isHeaterRunning = true;
        lastSupportTime = now; // принимаем, что любые ответы на поддержание защитываеются
        if(data[0] == 0x43) {
          startFromStarline = true;
        } else {
          startFromStarline = false;
        }
        break;
      case 0xFF: // Пришел ответ - котел не работает
//TTT        Serial.println("W-Bus. Котёл НЕ работает");
        isHeaterRunning = false;
        initSystem ();
//!!!!!!        sendWBusDelERR();
        break;
    }
  } else if (data[2] == 0xA1) { // Если 3-й байт равен A1 —  ответ от котла на команду запуска
    isHeaterRunning = true;
    if(data[0] == 0x43) {
      startFromStarline = true;
    } else {
      startFromStarline = false;
    }
    lastSupportTime = now; // принимаем, что любые ответы на поддержание (и пуска) защитываеются
  } else if (data[2] == 0x90) { // Если 3-й байт равен 90 —  ответ от котла на команду стоп
    isHeaterRunning = false;
    initSystem ();
  }
}

void manageClimate() {
  
  if (isHeaterRunning && (coolantTemp > 60) && voltageClimateOK && digitalRead(DEFROST_SENSOR == LOW)) {
    if (digitalRead(CLIMATE_RELAY == LOW)) {
      timeStartClimate = now; 
    }
    if (manualStart || ((now - timeStartClimate) < limitClimateTime )) {
    digitalWrite(CLIMATE_RELAY, HIGH);
    OCR1A = 2000; // 70% от 4999 
    } else {
    digitalWrite(CLIMATE_RELAY, LOW);
    OCR1A = 0;
    }
  } else {
    digitalWrite(CLIMATE_RELAY, LOW);
    OCR1A = 0;
  }
}

// Тестовая функция запуска вентилятора климата
/*TTT
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
*/
void checkPumpHealth() {

  static unsigned long pTimer = 0;
  
  if (now - pTimer > 300) { // Опрашиваем чаще (раз в 300 мс), чтобы фильтр работал плавно
    pTimer = now; 
    
    // Получаем текущее мгновенное значение тока
    float rawAmps = readAmps(); 
    
    // Коэффициент фильтрации (0.1 — сильное сглаживание, 1.0 — без сглаживания)
    float k = 0.1; 
    
    // Формула экспоненциального сглаживания
    filteredAmps = (rawAmps * k) + (filteredAmps * (1.0 - k));
    
    // Выводим сглаженный ток в монитор порта
 //TTT   Serial.print("Ток помпы: "); 
 //TTT   Serial.println(filteredAmps);
    
    // Проверку здоровья делаем по сглаженному току, чтобы избежать ложных срабатываний
    if ((filteredAmps < 0.4 || filteredAmps > 3.0) && ((now - timeStartPump) > 5000)) {
      pumpIsBroken = true;
    }
 
  }
}

float readAmps() {
  long sum = 0;
  for(int i = 0; i < 10; i++) sum += analogRead(CURRENT_PIN);

  // Опорное напряжение теперь 4.6V
  float voltage = ((float)sum * 4.6) / (1024.0 * 10.0);
  // Ноль тока сместился на 2.3V (4.6 / 2)
  // Чувствительность упала до ~0.1702 V/A

  return fabs((voltage - 2.47) / 0.1702);
}

                              // --- ФУНКЦИИ УПРАВЛЕНИЯ W-bus ---


              // --- ФУНКЦИИ УПРАВЛЕНИЯ (Вызываются извне, например по кнопке) ---

void startSystem() {
  if (currentState != IDLE) return;  // Если уже идет процесс, игнорируем новый вызов
  retryCount = 0;                    // Сбрасываем счетчик для новой операции
  currentState = SENDING_START;      // Переходим в режим запуска
  executeStart();                    // Делаем первую попытку
//TTT  Serial.println("!!! Запуск Вебсто");
}

void supportHeating() {
  if (now - lastSupportTime > SUPPORT_INTERVAL && now - lastWBusActivity > BUS_IDLE_TIME) {
  retryCount = 0;                    // Сбрасываем счетчик для новой операции
  currentState = SENDING_SUPPORT;      // Переходим в режим запуска
  executeSupport();                    // Делаем первую попытку
//TTT  Serial.println("!!! Поддержание работы Вебасто");
  }
}


void stopSystem() {
  if (currentState == SENDING_STOP) return;  // Если уже идет процесс, игнорируем новый вызов
  // Стоп имеет приоритет, поэтому не проверяем IDLE, а просто прерываем всё
  retryCount = 0;
  currentState = SENDING_STOP;
//TTT  Serial.print("!!! Стоп Вебсто по W-Bus: ");
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
    if (now - lastWBusActivity > BUS_IDLE_TIME) { 
      if((now - lastQueryTime > QUERY_INTERVAL && !startFromStarline ) || (startFromStarline && now - lastSupportTime < 3000 && now - lastQueryTime > 500)) { 
        uint8_t dataQuery[] = {0x50, queries[currentQueryIndex]};
        //uint8_t dataQuery[] = {0x50, 0x05};
        sendExtendedWBus(dataQuery, 2); // Шлем следующий запрос из очереди
        currentQueryIndex = (currentQueryIndex + 1) % 2; // Переходим к следующему параметру (0->1(->2->3)->0)
        lastQueryTime = now;  // Запоминаем время отправки
      }
  }
}

void sendExtendedWBus(byte* data, int len) {
  wBus.write(ADDR_TO_HEATER); wBus.write((byte)len+1);
//TTT  Serial.println("Отправка команды в W-Bus");
  byte crc = ADDR_TO_HEATER ^ ((byte)len+1);
  for(int i=0; i<len; i++) { wBus.write(data[i]); crc ^= data[i]; }
  wBus.write(crc);
  lastWBusActivity = now; // Считаем отправку тоже активностью на шине
}
                    // --- ГЛАВНЫЙ ОБРАБОТЧИК (ДИСПЕТЧЕР) ---

void checkWBusResponse(byte byteCheck) {

  // Блок 1: Если мы ждем подтверждения команды (Start или Stop)
    if (expectedResponse == byteCheck) {       // Если получен нужный байт
      if ((currentState == SENDING_START) || (currentState == SENDING_SUPPORT)) isHeaterRunning = true;
      if (currentState == SENDING_STOP) {
         isHeaterRunning = false;
         initSystem ();
        }
      currentState = IDLE;           // Команда принята, возвращаемся в покой
      //Serial.println("W-Bus: OK! Ответ получен.");
    } 
    else if (now - lastWBusActivity > TIMEOUT) { // Если ответа нет дольше 500мс
      if (retryCount < 10) {          // Если попытки еще остались
//TTT        Serial.print("W-Bus: Попытка #"); Serial.println(retryCount + 1);
        if (currentState == SENDING_START) executeStart();
        else if (currentState == SENDING_SUPPORT) executeSupport();
        else executeStop();
      } else {                       // Если все попытки провалены
        isHeaterRunning = false;
        initSystem ();
//TTT        Serial.println("W-Bus: Ошибка! Нет ответа после 5 попыток.");
        currentState = IDLE;         // Сдаемся и выходим в покой
      }
    }
}
                     // --- ФУНКЦИИ КНОПКИ И ЛАМПОЧКИ ---

void handleClickLogic() {
  int clicks = button.getNumberClicks();
  blinkCount = clicks;
//TTT  Serial.println(blinkCount); 
  switch (clicks) {
    case 1:
      isTimerActive = true;
      timerStartTime = now;
      remainingWdtTicks = TOTAL_TICKS_9_HOURS;
      break;
    case 2:
      isTimerActive = false;
      manualStart = true;
      startSystem();
      break;
    case 3:
      sendWBusDelERR(); //сброс ошибок вебасто
      pumpIsBroken = false;  //сброс ошибки помпы
      initSystem ();
      break;
  }
}

void handleLongPress() {
  stopSystem(/*TTT"По кнопке"*/);
  initSystem ();
  blinkCount = 1;
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
  // 4. ОШИБКА: Нет сигнала помпы по CAN. Частое моргание (например, 2 раз в секунду)
  else if (canPumpTimeout) {
    // Период 300мс (100мс включен / 200мс выключен)
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

                     // --- ФУНКЦИЯ ОБНУЛЕНИЯ ПЕРЕМЕННЫХ ---

void initSystem () {
    coolantTemp = 0;   // Обнуляем температуру
    voltageVal = 13.0; // Обнуляем напряжение
    manualStart = false; // Флаг: ручной рапуск (с кнопким в салоне), отключаем ограничение времени климата
    isTimerActive = false; // Флаг работы таймера запуска Вебасто
    startFromStarline = false; 
}

                     // --- ТАЙМЕР ОТЛОЖЕННОГО ЗАПУСКА WEBASTO ---

void checkTimer() {
  if (isTimerActive && (now - timerStartTime > INACTIVITY_SLEEP_TIMEOUT)) {
    // В момент ожидания отложенного запуска используем циклический сон
//TTT    Serial.println("L");
    enterLongSleep(); 

    // После каждого 8-секундного цикла или пробуждения отслеживаем время
    if (remainingWdtTicks > 0) {
      remainingWdtTicks--;
    }

    if (remainingWdtTicks == 0) {
//TTT      Serial.println(" Wake up after LONG SLEEP");
      isTimerActive = false;
      startSystem(); // Время вышло, запускаем систему
    }
  }
}





// --- ЭНЕРГОСБЕРЕЖЕНИЕ И СОН ---

// Пустая функция-обработчик для внешних прерываний (просто выдергивает процессор из sleep_cpu)
void wakeUpDummy() {}


// --- НАСТРОЙКА И СБРОС ПРЕРЫВАНИЙ W-BUS (PCINT) ---

void setupPinChangeInterrupts() {
    PCICR |= (1 << PCIE2);     // Разрешаем прерывания группы PCIE2 (пины от D0 до D7)
    PCMSK2 |= (1 << PCINT22);  // Включаем маску конкретно для пина D6 (PCINT22)
}

void disablePinChangeInterrupts() {
    PCMSK2 &= ~(1 << PCINT22); // Выключаем маску пина D6
    PCICR &= ~(1 << PCIE2);
}


// --- НАСТРОЙКА СТОРОЖЕВОГО ТАЙМЕРА (WDT) НА 8 СЕКУНД ---

void setupWatchdog() {
    MCUSR &= ~(1 << WDRF);
    WDTCSR |= (1 << WDCE) | (1 << WDE);
    // Устанавливаем максимальный интервал 8 секунд (WDP3, WDP0) и режим прерывания (WDIE)
    WDTCSR = (1 << WDIE) | (1 << WDP3) | (1 << WDP0); 
}

void disableWatchdog() {
    MCUSR &= ~(1 << WDRF);
    WDTCSR |= (1 << WDCE) | (1 << WDE);
    WDTCSR = 0; // Полное отключение сторожевого таймера
}


// --- 1. ОБЫЧНЫЙ ГЛУБОКИЙ СОН (когда таймер НЕ активен, ждем 5 минут простоя) ---
void enterSleep() {
    // Гасим всю периферию и силовые ключи для максимальной экономии
    digitalWrite(PUMP_RELAY, LOW);
    digitalWrite(CLIMATE_RELAY, LOW);
    digitalWrite(LED_PIN, LOW);
    OCR1A = 0;

    // Привязываем внешние прерывания-будильники
    attachInterrupt(digitalPinToInterrupt(CAN0_INT), wakeUpDummy, LOW);
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), wakeUpDummy, LOW);
    setupPinChangeInterrupts();

    // Переводим процессор в режим максимального энергосбережения
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();

    // Отключаем АЦП и ненужные аппаратные модули питания
    byte oldADCSRA = ADCSRA;
    ADCSRA = 0;
    power_adc_disable();
    power_spi_disable();
    power_timer0_disable();
    power_timer2_disable();

    // --- УХОДИМ В СОН (процессор останавливается здесь на неопределенный срок) ---
    sleep_cpu(); 
    // -------------------------------------------------------------------------

    // --- ПРОБУЖДЕНИЕ (программа продолжается с этой строки после срабатывания прерывания) ---
    
    sleep_disable();
//TTT    Serial.println("I WAKE UP");
    // Восстанавливаем питание модулей и АЦП
    power_all_enable();
    ADCSRA = oldADCSRA;

    // Отключаем будильники
    detachInterrupt(digitalPinToInterrupt(CAN0_INT));
    detachInterrupt(digitalPinToInterrupt(BUTTON_PIN));
    disablePinChangeInterrupts();
    wBus.begin(2400, CSERIAL_8E1);
    // Сбрасываем таймер бездействия после пробуждения
    lastActivityTime = millis();
}


// --- 2. ДОЛГИЙ ЦИКЛИЧНЫЙ СОН С WDT (когда активен 9-часовой отложенный запуск) ---
void enterLongSleep() {
    // Гасим силовую периферию
    digitalWrite(PUMP_RELAY, LOW);
    digitalWrite(CLIMATE_RELAY, LOW);
    digitalWrite(LED_PIN, LOW);
    OCR1A = 0;

    // Настраиваем прерывания-будильники (чтобы можно было разбудить кнопкой, CAN или W-Bus досрочно)
    attachInterrupt(digitalPinToInterrupt(CAN0_INT), wakeUpDummy, LOW);
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), wakeUpDummy, LOW);
    setupPinChangeInterrupts();

    // Включаем сторожевой таймер на 8 секунд
    setupWatchdog();



    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();

    // Отключаем АЦП и периферию для экономии во время ожидания
    byte oldADCSRA = ADCSRA;
    ADCSRA = 0;
    power_adc_disable();
    power_spi_disable();
    power_timer0_disable();
    power_timer2_disable();

    // --- ЗАСЫПАЕМ НА 8 СЕКУНД (или просыпаемся раньше от внешней активности) ---
    sleep_cpu(); 
    // --------------------------------------------------------------------------

    // --- ПРОБУЖДЕНИЕ (каждые 8 секунд или при внешнем событии) ---
//    wBus.begin(2400, CSERIAL_8E1);
    sleep_disable();
    disableWatchdog(); // Обязательно гасим WDT после пробуждения
    
    power_all_enable();
    ADCSRA = oldADCSRA;

    // Снимаем внешние прерывания
    detachInterrupt(digitalPinToInterrupt(CAN0_INT));
    detachInterrupt(digitalPinToInterrupt(BUTTON_PIN));
    disablePinChangeInterrupts();
    wBus.begin(2400, CSERIAL_8E1);
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




Starline шлёт сигнал на поддержание Webasto каждые 9 секунд
Если слать каждые 15 секунд сигнал на поддержание работы, котёл работает, но не стабильно, вероятно, 15 секунд это предел






*/
  
  
