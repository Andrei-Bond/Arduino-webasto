// Webasto CAN Analyzing

#include <mcp_can.h>
#include <SPI.h>

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
    static unsigned long lastTime = 0;
    if (millis() - lastTime > 100) {
        lastTime = millis();
       //byte msg427[8] = {0x00, 0x71, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //Без этого выдаёт ошибку
      // CAN0.sendMsgBuf(0x427, 0, 8, msg427);


       if((digitalRead(BTN_PIN) == LOW)) {
       }
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
