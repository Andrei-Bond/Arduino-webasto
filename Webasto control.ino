// Webasto CAN simulation + w-bus

#include <mcp_can.h>
#include <SPI.h>

#include <CustomSoftwareSerial.h> // Библиотека для 8E1 на программных пинах для w-bus
// Пины для K-Line адаптера
#define WBUS_RX 6
#define WBUS_TX 7
#define FAN_PWM_PIN 9   // ШИМ вентилятора, здесь будет строго 400 Гц
#define PUMP_RELAY 5       // Реле помпы
#define CLIMATE_RELAY 4    // Реле переключения климата (2 Питания + Сигнал климата)

// Адресация: Таймер (F) -> Котел (4) w-bus
#define ADDR_HEATER 0x4F 
// Настройка порта: пины 8 и 10, режим 8 бит, Even parity, 1 стоп-бит
CustomSoftwareSerial wBus(WBUS_RX, WBUS_TX); 


long unsigned int rxId; // хранилище ИД из CAN
unsigned char len = 0; // хранилище длины данных из CAN
unsigned char rxBuf[8]; // хранилище массива данных из CAN

#define CAN0_INT 3      // Set INT to pin 3


byte askStat1[8] = {0x00, 0x71, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // ответ для вебасто (есть связь?)


MCP_CAN CAN0(10);     // Set CS to pin 10

int coolantTemp = 0;
bool isRunning = false;
unsigned long lastQuery = 0;
bool wbusPumpState = false;

void setup()
{
  Serial.begin(115200);
  
  // Initialize MCP2515 running at 8MHz with a baudrate of 100kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_100KBPS, MCP_8MHZ) == CAN_OK) Serial.println("MCP2515 Инициализирован успешно!");
  else Serial.println("Ошибка инициализации MCP2515...");

  CAN0.setMode(MCP_NORMAL);   // Выбираем нормальный режии, чтобы разрешить отправку сообщений

  pinMode(CAN0_INT, INPUT);                            // Configuring pin for /INT input
  
    // Инициализация шины Webasto на 2400 бод
  wBus.begin(2400, CSERIAL_8E1); 
  
  Serial.println("W-Bus Control Ready");
  
  // Настройки для вентилятора и помпы
  pinMode(PUMP_RELAY, OUTPUT);
  pinMode(CLIMATE_RELAY, OUTPUT);
  
  // Выключаем всё (инверсная логика реле: HIGH = выкл)
  digitalWrite(PUMP_RELAY, HIGH); 
  digitalWrite(CLIMATE_RELAY, HIGH);
  
  // --- НАСТРОЙКА ТАЙМЕРА 1 НА 400 Гц (Пин 9) ---
  pinMode(FAN_PWM_PIN, OUTPUT);
  TCCR1A = _BV(COM1A1) | _BV(WGM11);            // Режим Fast PWM, TOP в ICR1
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11); // Делитель 8
  ICR1 = 4999;                                  // (16MHz / (8 * 400Hz)) - 1 = 4999
  OCR1A = 0;                                    // Изначально 0% (выключено)

}



// --- ФУНКЦИИ УПРАВЛЕНИЯ W-bus ---
  // Команда 0x21 (управление), 0x01 (старт), mins (время в минутах)
void startSystem(byte mins) {
  isRunning = true;
  byte startData[] = {0x21, 0x01, mins};
  sendPacket(startData, 3);
  Serial.println("W-Bus: START sent");
}

  // Команда 0x21 (управление), 0x00 (выключить)
  
void stopSystem() {
  isRunning = false;
  byte stopData[] = {0x21, 0x00};
  sendPacket(stopData, 2);
  Serial.println("W-Bus: STOP sent");
}


void sendWBusQuery() {
  // Команда 0x05 (запрос состояния/температуры)
  byte cmd[] = {0x05};
  sendPacket(cmd, 1);
}

// --- НИЗКОУРОВНЕВАЯ ОТПРАВКА ПАКЕТА ---

void sendPacket(byte* data, int len) {
  byte crc = 0;
  
  // 1. Отправляем адрес котла (0x4F)
  wBus.write(ADDR_HEATER);
  crc ^= ADDR_HEATER;
  
  // 2. Отправляем длину данных
  wBus.write((byte)len);
  crc ^= (byte)len;
  
  // 3. Отправляем сами данные
  for (int i = 0; i < len; i++) {
    wBus.write(data[i]);
    crc ^= data[i];
  }
  
  // 4. Отправляем контрольную сумму (XOR всех байтов)
  wBus.write(crc);
}

// Функции управления вентилятором печки

void setFanDuty(int percent) {
  // Установка скважности (0-100%) для 400 Гц
  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;
  OCR1A = map(percent, 0, 100, 0, 4999);
}

void manageClimateControl() {
  if (!isRunning) {
    setFanDuty(0);
    digitalWrite(CLIMATE_RELAY, HIGH)
    return;
  }
  
  // Логика по инструкции (например, 70% при прогреве)
  if (coolantTemp > 40) {
    digitalWrite(CLIMATE_RELAY, LOW);
    setFanDuty(70); 
  } else {
    setFanDuty(0);
    digitalWrite(CLIMATE_RELAY, HIGH);
  }
}


void parseWBusStatus() {
  byte buf[20];
  int i = 0;
  
  // Читаем пакет ответа (обычно 0x4F ...)
  while (wBus.available() && i < 20) {
    buf[i++] = wBus.read();
  }
// Анализ пакета ответа на команду 0x05
  // В Thermo Top C (VAG) статус компонентов обычно в 7-м байте (индекс 6)
  if (i > 7 && (buf[0] == 0x4F || buf[0] == 0xF4)) {
    
    // Бит помпы (обычно 1-й бит в 7-м байте)
    // Маска 0x02 проверяет второй бит (00000010)
    if (buf[6] & 0x02) { 
      wbusPumpState = true;
    } else {
      wbusPumpState = false;
    }

    // Заодно обновляем температуру (обычно 5-й байт, индекс 4)
    coolantTemp = buf[4] - 50;
  }
}


void loop()
{

    // 1. ЧИТАЕМ ШИНУ CAN
    
     if(!digitalRead(CAN0_INT))          // Если вывод CAN0_INT is LOW, отправляем подтверждение связи
  {
    CAN0.sendMsgBuf(0x427, 0, 8, askStat1);
  
}


    // 4. Блок управления по W-bus, вариант от Gemini


  // Пример управления через монитор порта: '1' - старт, '0' - стоп
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '1') startSystem(30); 
    if (c == '0') stopWebasto();
  }
  
  if (wBus.available() > 0) {
    parseWBusStatus();
  }
  
  if (isRunning && (millis() - lastQuery > 4000)) {
    sendWBusQuery(0x05); // Запрос статуса
    lastQuery = millis();
  }
  //  Прямое управление реле помпы по статусу из шины
  if (wbusPumpState) {
    digitalWrite(PUMP_RELAY, LOW);  // Включаем реле (помпа качает)
  } else {
    digitalWrite(PUMP_RELAY, HIGH); // Выключаем реле
  }
  
manageClimateControl();

  

}
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/

