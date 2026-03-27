// Webasto CAN simulation + w-bus

#include <mcp_can.h>
#include <SPI.h>

#include <CustomSoftwareSerial.h> // Библиотека для 8E1 на программных пинах для w-bus
// Пины для K-Line адаптера
#define WBUS_RX 8
#define WBUS_TX 10

// Адресация: Таймер (F) -> Котел (4) w-bus
#define ADDR_HEATER 0x4F 
// Настройка порта: пины 8 и 10, режим 8 бит, Even parity, 1 стоп-бит
CustomSoftwareSerial wBus(WBUS_RX, WBUS_TX); 

unsigned long lastKeepAlive = 0; //w-bus
bool webastoIsOn = false; //w-bus




long unsigned int rxId; // хранилище ИД из CAN
unsigned char len = 0; // хранилище длины данных из CAN
unsigned char rxBuf[8]; // хранилище массива данных из CAN
char msgString[128];    // Array to store serial string (строка для вывода в порт монитора)
#define CAN0_INT 3      // Set INT to pin 3
char msgCodeCanDscr[20]; // переменная для описание кода Вебасто
byte askStat1[8] = {0x00, 0x71, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // ответ для вебасто (есть связь?)


MCP_CAN CAN0(9);     // Set CS to pin 9
const int BTN_PIN = 2;     // Пин, к которому подключена кнопка запуска
//unsigned long lastSendTime = 0; // Время последней отправки пакетов "жизни"
const int sendInterval = 500;   // Интервал отправки пакетов (500 мс)

void setup()
{
  Serial.begin(115200);
  pinMode(BTN_PIN, INPUT_PULLUP); // Подтягиваем + к пину кнопки, что бы не зависала
  // Initialize MCP2515 running at 8MHz with a baudrate of 100kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_100KBPS, MCP_8MHZ) == CAN_OK) Serial.println("MCP2515 Инициализирован успешно!");
  else Serial.println("Ошибка инициализации MCP2515...");

  CAN0.setMode(MCP_NORMAL);   // Выбираем нормальный режии, чтобы разрешить отправку сообщений

  pinMode(CAN0_INT, INPUT);                            // Configuring pin for /INT input
  
    // Инициализация шины Webasto на 2400 бод
  wBus.begin(2400, CSERIAL_8E1); 
  
  Serial.println("W-Bus Control Ready");
}


// Функция вывода принятых сообщений в монитор порта
void msgToMonitor(char msgCodeCanDscr[40]){

      sprintf(msgString, "%s ID: 0x%.3lX Data:", msgCodeCanDscr, rxId);
  
    Serial.print(msgString);
  {
      for(byte i = 0; i<len; i++){
        sprintf(msgString, " 0x%.2X", rxBuf[i]);
        Serial.print(msgString);
      }
    }
    Serial.println();
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






//Управление по W-bus
//     1. Вариант с drive2
/*
#include <wire.h>
//#include <liquidcrystal_I2C.h>
//LiquidCrystal_I2C lcd(0x27,16,2);

byte Init1[] = {0x81, 0x51, 0xF1, 0x81, 0x44};
byte Init2[] = {0x82, 0x51, 0xF1, 0x3C, 0x00, 0x00};
byte Request1[] = {0x83, 0x51, 0xF1, 0x2A, 0x01, 0x01, 0xF1};
byte Request2[] = {0x83, 0x51, 0xF1, 0x2A, 0x01, 0x02, 0xF2};
byte Request3[] = {0x83, 0x51, 0xF1, 0x2A, 0x01, 0x05, 0xF5};
byte Wakeup[] = {0x81, 0x51, 0xF1, 0xA1, 0x64};
byte StartWebasto[] = {0x83, 0x51, 0xF1, 0x31, 0x22, 0xFF, 0x17};
byte StopWebasto[] = {0x83, 0x51, 0xF1, 0x31, 0x22, 0x00, 0x18};
byte Answer[18]; // вообще говоря, в ответе 11 байт. Но ещё 7 байт придут перед ответом, это сам запрос, т.к. в протоколе K-Line присутствует эхо
float voltage = 0.00;
int temperature = 0;

void setup() {

Serial.begin(9600);

pinMode(18, OUTPUT); // TX1

// Пробуждение Webasto?
digitalWrite(18, LOW);
delay(300);
digitalWrite(18, HIGH);
delay(50);
digitalWrite(18, LOW);
delay(25);
digitalWrite(18, HIGH);

delay(3025);
Serial1.begin(10400);

Serial1.write(Init1, 5);
delay(40);

while(Serial1.available() > 0) {
Serial1.read();
}

delay(120);

Serial1.write(Init2, 6);
delay(40);

while(Serial1.available() > 0) {
Serial1.read();
}

delay(120);

//lcd.init();
//lcd.backlight();

}

void loop() {

Serial1.write(Wakeup, 5);
delay(40);

while(Serial1.available() > 0) {
Serial1.read();
delay(50);
}

Serial1.write(Request1, 7);
delay(40);

while(Serial1.available() > 0) {
for(int i = 0; i < 18; i++) {

Answer[i] = Serial1.read();
delay(10);

}
}

for(int i = 7; i < 18; i++)
{
Serial.print(Answer[i], HEX);
Serial.print(' ');
}

Serial.println(' ');

if (Answer[7] == 0x87) {
temperature = (222 — Answer[12]) / 1.77;
voltage = Answer[14] / 14.5;
}

//lcd.clear();
//lcd.setCursor(0,0);
//lcd.print("Temp:");
//lcd.setCursor(5,0);
//lcd.print(temperature);
//lcd.setCursor(0,1);
//lcd.print("Volt:");
//lcd.setCursor(5,1);
//lcd.print(voltage);

delay(100);
}


*/



void loop()
{

    // 1. ЧИТАЕМ ШИНУ
    // 1.2 Код из примера библиотеки
     if(!digitalRead(CAN0_INT))          // Если вывод CAN0_INT is LOW, считываем буфер приема
  {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);  // Считывем данные: len = длина данных, buf = байт(ы) данных

    // 2 Обрабатывем полученные данные
        // 2.1 ОБРАБОТКА ID 427 (Запросы от котла)
        if (rxId == 0x427) {
                          switch(rxBuf[0]) {
                              case 0x07:{
                                    switch(rxBuf[1])
                                        {
                                          case 0x01: {
                                                      CAN0.sendMsgBuf(0x427, 0, 8, askStat1);
                                                      msgToMonitor("Статусное 1               ");

                                                      break;}
                                          case 0x02: {
                                                      CAN0.sendMsgBuf(0x427, 0, 8, askStat1);
                                                      msgToMonitor("Статусное 2                ");
                                                      break;}
                                          case 0x04:{
                                                      //CAN0.sendMsgBuf(0x427, 0, 8, askStat1);
                                                      msgToMonitor("НЕИЗВЕСТНО 07 04           ");  
                                                      break;}
                                          case 0x14: {
                                                      //CAN0.sendMsgBuf(0x427, 0, 8, askStat1);
                                                      msgToMonitor("НЕИЗВЕСТНО 07 14           ");  
                                                      break; }
                                          case 0xD8: {
                                                      //CAN0.sendMsgBuf(0x427, 0, 8, askStat1);
                                                      msgToMonitor("НЕИЗВЕСТНО 07 D8           "); 
                                                      break;}                      
                                          default:   {
                                                      msgToMonitor("НЕИЗВЕСТНО 07              ");              
                                                      }
                                                     }
                                             break;}
                                
                        default: {
                          msgToMonitor("НЕИЗВЕСТНО                 ");  
                                           }                  
                                                       }
            // 2.2 ОБРАБОТКА ID 3E5 (Запросы от котла)
            } else if (rxId == 0x3E5) {
                          switch(rxBuf[0]) {
                              case 0x20:{
                                    switch(rxBuf[1])
                                    {

                               //ОТОПИТЕЛЬ ЗАПУЩЕН  
                                          case 0x0A: {//byte ackStop[] = {0x20, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00};
                                                      //CAN0.sendMsgBuf(0x3E5, 0, 8, ackStop);
                                                      msgToMonitor("ОТОПИТЕЛЬ ЗАПУЩЕН              ");
                                                      break;}
                                          case 0x8A: {//byte ackStop[] = {0x20, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00};
                                                      //CAN0.sendMsgBuf(0x3E5, 0, 8, ackStop);
                                                      msgToMonitor("ОТОПИТЕЛЬ ЗАПУЩЕН 2            ");
                                                      break;}
                                             
                                          case 0x02: {//byte ackStop[] = {0x20, 0x02, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00};
                                                      //CAN0.sendMsgBuf(0x3E5, 0, 8, ackStop);
                                                      msgToMonitor("ПРИ ПУСКЕ ЕСТЬ (РОЗЖИГ)       ");
                                                      break;}


                               //ПУСК ПОМПЫ                       
                                          case 0x08:{ //byte ackStop[] = {0x07, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
                                                      //CAN0.sendMsgBuf(0x427, 0, 8, ackStop);
                                                      msgToMonitor("ПУСК ПОМПЫ                  ");
                                                      break;}
                                          case 0xD8:  {
                                                      //byte PuskPomp2[] = {0x20, 0xD9, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
                                                      //CAN0.sendMsgBuf(0x3E5, 0, 8, PuskPomp2);
                                                      msgToMonitor("ПУСК ПОМПЫ (с ошибкой 49)   ");  
                                                      break;}
                                          case 0x88: {//byte ackStop[] = {0x20, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
                                                      //CAN0.sendMsgBuf(0x427, 0, 8, ackStop);
                                                      msgToMonitor("Пуск ПОМПЫ (с ошибкой 49)2  ");
                                                      break;}
                                          case 0x68: {//byte ackStop[] = {0x20, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
                                                      //CAN0.sendMsgBuf(0x427, 0, 8, ackStop);
                                                      msgToMonitor("Пуск ПОМПЫ (с блокировкой) ");
                                                      break;}           

                             //ГОТОВ К РАБОТЕ
                                          case 0x00:  {
                                                      //byte StartWebasto[] = {0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
                                                      //CAN0.sendMsgBuf(0x3E5, 0, 8, StartWebasto);
                                                      msgToMonitor("Готов к работе               ");
                                                      break;}
                                          case 0xD0: {//byte ackStop[] = {0x07, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
                                                      //CAN0.sendMsgBuf(0x427, 0, 8, ackStop);
                                                      msgToMonitor("Готов к работе(с ошибкой 49) ");
                                                      break;}

                                          case 0x80: {//byte ackStop[] = {0x20, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
                                                      //CAN0.sendMsgBuf(0x427, 0, 8, ackStop);
                                                      msgToMonitor("Готов к работе(с ошибкой 49)2");
                                                      break;}                                                 
                                                      

                              // ПУСК ВЕНТИЛЯТОРА САЛОНА
                                          case 0xD1: {
                                                     //byte ventSal1[8] = {0x20, 0xD1, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00};
                                                     //CAN0.sendMsgBuf(0x3E5, 0, 8, ventSal1);
                                                      msgToMonitor("Пуск вен.сал. с ошибкой 49 ");
                                                      break;}
                                          case 0x81: {//byte ackStop[8] = {0x07, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
                                                      //CAN0.sendMsgBuf(0x427, 0, 8, ackStop);
                                                      msgToMonitor("Пуск вен.сал. с ошибкой 49(2)");
                                                      break;}
                                          case 0x01: {//byte ackStop[8] = {0x07, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
                                                      //CAN0.sendMsgBuf(0x427, 0, 8, ackStop);
                                                      msgToMonitor("Пуск вентилятора салона    ");
                                                      break;}        
                               // БЛОКИРОВАНИЕ ОТОПИТЕЛЯ
                                          case 0xE0: {
                                                      //byte blockOt[8] = {0x20, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
                                                      //CAN0.sendMsgBuf(0x3E5, 0, 8, blockOt);  
                                                      msgToMonitor("БЛОКИРОВАНИЕ ОТОПИТЕЛЯ + ошибка");
                                                      break;}
                                                      
                                          case 0x60: {
                                                      //byte blockOt[8] = {0x20, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
                                                      //CAN0.sendMsgBuf(0x3E5, 0, 8, blockOt);
                                                      msgToMonitor("БЛОКИРОВАНИЕ ОТОПИТЕЛЯ  2  ");
                                                      break;}
   
                                          default:  {
                                                      msgToMonitor("НОВОЕ СООБЩЕНИЕ  ");}              
                                                        }
                                                      break;}

                                case 0x21:  {
                                            //byte WebON[] = {0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
                                            //CAN0.sendMsgBuf(0x3E5, 0, 8, WebON);
                                            msgToMonitor("Вебасто вкл               ");
                                            break;}

                                                      
                                case 0x22: {
                                    switch(rxBuf[1])
                                    {
                                          case 0x08: {//byte ackStop[] = {0x22, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
                                                      //CAN0.sendMsgBuf(0x427, 0, 8, ackStop);
                                                      msgToMonitor("ПРИ ПУСКЕ ЕСТЬ (РОЗЖИГ)       ");
                                                      break;
                                          case 0x00:  //byte WebOFF[] = {0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
                                                      //CAN0.sendMsgBuf(0x3E5, 0, 8, WebOFF);
                                                      msgToMonitor("Вебасто выкл              ");
                                                      break;
                                          case 0x80:  //byte WebOFF[] = {0x22, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
                                                      //CAN0.sendMsgBuf(0x3E5, 0, 8, WebOFF);
                                                      msgToMonitor("Вебасто выкл с ошибкой 49  ");
                                                      break;
                                                                                                            
                                          case 0xD0:  //byte WebOFF[] = {0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
                                                      //CAN0.sendMsgBuf(0x3E5, 0, 8, WebOFF);
                                                      msgToMonitor("Вебасто выкл с ошибкой 49");
                                                      break;
                                          }
   
                                          default:  {
                                                      msgToMonitor("НОВОЕ СООБЩЕНИЕ  ");}              
                                                  
                                  }
                                                        break;}   
                                          default:  {
                                                      msgToMonitor("НОВОЕ СООБЩЕНИЕ  ");}              
                                                                      
                                                       }
   }else    { msgToMonitor("НОВОЕ СООБЩЕНИЕ  ");
   }              
                                                           
  }


                 
    
                          
     // 3. ФОНОВАЯ ЭМУЛЯЦИЯ (раз в 100мс шлем статус авто, чтобы котел не «отвалился»)
    //static unsigned long lastTime = 0;
   // if (millis() - lastTime > 100) {
  //      lastTime = millis();
       //byte msg427[8] = {0x00, 0x71, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //Без этого выдаёт ошибку
      // CAN0.sendMsgBuf(0x427, 0, 8, msg427);


    //   if((digitalRead(BTN_PIN) == LOW)) {
   //    }
  //  }   
    
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



    
/*

                            // --- БЛОК 4.1: УПРАВЛЕНИЕ ЗАПУСКОМ (ПО КНОПКЕ) ---
                            
       static unsigned long lastTimeSTART = 0;

                                  //              Вариант 1 F1
        if ((digitalRead(BTN_PIN) == LOW) && (millis() - lastTimeSTART > 500)) { 
            // Если кнопка нажата (пин замкнут на землю) — шлем команду СТАРТ
             
               //if(StartStat == CAN_OK){
               //    Serial.println(">>> ОТПРАВЛЕНА КОМАНДА: СТАРТ");
                   
              // } else {
              //  Serial.println(">>>!!!!!!! Не ОТПРАВЛЕНА КОМАНДА: СТАРТ");
               //       }
              //        lastTimeSTART = millis();
        } else if (digitalRead(BTN_PIN) == HIGH && (millis() - lastTimeSTART > 800))
        {
            // Если кнопка отпущена — шлем команду СТОП (байт 01 меняем на 00)
    
               if(StopStat == CAN_OK){
                   Serial.println(">>> ОТПРАВЛЕНА КОМАНДА: СТОП");
                 } else {
                Serial.println(">>>!!!!!!! Не ОТПРАВЛЕНА КОМАНДА: СТОП");
                      }
                      lastTimeSTART = millis();
        } 
        


        */
  

/*
    // --- БЛОК 5: ЧТЕНИЕ СТАТУСА ИЗ WEBASTO (Слушаем шину) ---
    long unsigned int rxId;
   unsigned char len = 0;
    unsigned char buf[8];

    if (CAN0.checkReceive() == CAN_MSGAVAIL) {
        CAN0.readMsgBuf(&rxId, &len, buf);

        // ID 0x6C2 — это ответный пакет от Webasto со всеми данными
        if (rxId == 0x6C2) {
            Serial.print("Webasto: ");
            
            // Расшифровка режима (1-й байт)
            switch(buf[0]) {
                case 0x01: Serial.print("ОЖИДАНИЕ | "); break;
                case 0x02: Serial.print("РОЗЖИГ | "); break;
                case 0x03: Serial.print("РАБОТА (MAX) | "); break;
                case 0x04: Serial.print("РАБОТА (PART) | "); break;
                case 0x05: Serial.print("ОСТАНОВКА | "); break;
            }
            // Расшифровка температуры (2-й байт минус 40)
            int temp = buf[1] - 40; 
            Serial.print("Темп: "); Serial.print(temp); Serial.print("°C | ");

            // Флаги активных компонентов (3-й байт)
            if (buf[2] & 0x01) Serial.print("[ПОМПА] "); 
            if (buf[2] & 0x02) Serial.print("[НАСОС ТОПЛИВА] "); 
            if (buf[2] & 0x08) Serial.print("[ВЕНТИЛЯТОР] ");

            Serial.println(); // Переход на новую строку
        }
    }
*/

}
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
