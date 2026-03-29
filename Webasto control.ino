// Webasto CAN simulation + w-bus

#include <mcp_can.h>
#include <SPI.h>

#include <CustomSoftwareSerial.h> // Библиотека для 8E1 на программных пинах для w-bus
// Пины для K-Line адаптера
#define WBUS_RX 8
#define WBUS_TX 9

// Адресация: Таймер (F) -> Котел (4) w-bus
#define ADDR_HEATER 0x4F 
// Настройка порта: пины 8 и 10, режим 8 бит, Even parity, 1 стоп-бит
CustomSoftwareSerial wBus(WBUS_RX, WBUS_TX); 

unsigned long lastKeepAlive = 0; //w-bus
bool webastoIsOn = false; //w-bus




long unsigned int rxId; // хранилище ИД из CAN
unsigned char len = 0; // хранилище длины данных из CAN
unsigned char rxBuf[8]; // хранилище массива данных из CAN

#define CAN0_INT 3      // Set INT to pin 3
char msgCodeCanDscr[20]; // переменная для описания кода Вебасто
byte askStat1[8] = {0x00, 0x71, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // ответ для вебасто (есть связь?)


MCP_CAN CAN0(10);     // Set CS to pin 10



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
}



// --- ФУНКЦИИ УПРАВЛЕНИЯ W-bus ---

void startWebasto(byte mins) {
  // Команда 0x21 (управление), 0x01 (старт), mins (время в минутах)
  byte cmd[] = {0x21, 0x01, mins}; 
  sendPacket(cmd, 3);
  webastoIsOn = true;
  Serial.println("W-Bus: START sent");
}

void stopWebasto() {
  // Команда 0x21 (управление), 0x00 (выключить)
  byte cmd[] = {0x21, 0x00};
  sendPacket(cmd, 2);
  webastoIsOn = false;
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




void loop()
{

    // 1. ЧИТАЕМ ШИНУ CAN
    
     if(!digitalRead(CAN0_INT))          // Если вывод CAN0_INT is LOW, отправляем подтверждение связи
  {
    CAN0.sendMsgBuf(0x427, 0, 8, askStat1);
  
}


    // 4. Блок управления по W-bus, вариант от Gemini

  // Если Webasto запущена, нужно раз в 5-10 сек слать запрос, 
  // чтобы штатный котел Touareg не уснул (Keep-alive)
  if (webastoIsOn && (millis() - lastKeepAlive > 5000)) {
    sendWBusQuery(); 
    lastKeepAlive = millis();
  }

  // Пример управления через монитор порта: '1' - старт, '0' - стоп
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '1') startWebasto(30); 
    if (c == '0') stopWebasto();
  }


  

}
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
