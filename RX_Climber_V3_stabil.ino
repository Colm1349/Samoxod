/**
   Этот файл является частью библиотеки MBee-Arduino.
   MBee-Arduino является бесплатным программным обеспечением.
   Подробная информация о лицензиях находится в файле mbee.h.
   \author </i> von Boduen. Special thanx to Andrew Rapp.
*/

#include <MBee.h>
//#include <SoftwareSerial.h>
#include <PacketSerial.h>
#include <AltSoftSerial.h>
#include <avr/wdt.h>

// ОПИСАНИЕ ЗАДАЧИ ДОБАВЬ!
/*   Приёмник сигнала с пульта. Если сигнале нет 0.5 сек - перазгрузка
     Приём сигнала даёт возможность начать движение самохода через подачу управляющих сигналов на контроллеры ESCON с использованием ШИМ
     Вперед/Назад/Стоп и получени телеметрии с ESCON'a через АЦП , 4 входа АЦП для 4ех параметров (Скорость и обороты каждого из 2ух двигателей)
     и отправляет отладочные сообщения на пинах 7 и 8 на PC
     Отображает светодиодами состояние батареи, направление движения, наличие ошибок, выбор режима связи (радио или провод)
     Отсутствие сигнала приводит к включению звуковой индикации и подаче сигнала СТОП на двигатели при дальнейшем отсутствии сигнала приводит перезагрузке.
*/

// add AnalogPin - as selector
// add buzzer
// add


//For UNO
#define PC_Debug altSerial         // nss2
#define MBee_Serial Serial

//For MEGA
//#define PC_Debug Serial
//#define MBee_Serial Serial1

//Const
#define EmergencyStopCode 52
#define BOOST_Code_ON  42
#define BOOST_Code_OFF 41
#define Stop 0
#define Forward 5
#define Backward 2
#define Corrupted_Command 10
#define ZeroPWM 30
#define Max_SpeedValue 225
#define Min_SpeedValue -225

//Analog Pins
// A0 - xt3
// A1 - SWITCHER
// A2 - LED#7
// A3 - Battery LvL
// A4 - Speed 2
// A5 - Current 2
// A6 - Speed 1
// A7 - Current 1
///////////////////////////////////////////////////////////////
//ADC_Pin_0 no connect   (XT3 on plate)
#define SWITCHER_PIN      A1   // Switcher + Led_Pin
#define ErrorLed_ADCPin   A2   // ErrorLed Pin
#define ADCpin_BatteryLvL A3   // Roper_Battery Voltage
#define ADCpin_Speed2     A4   // A4 Speed 
#define ADCpin_Current2   A5   // 
#define ADCpin_Speed1     A6
#define ADCpin_Current1   A7

//Digital PINs
#define Permission_Of_Move 2
#define Direction_Of_Move  3
#define ForwardLed         4
#define BackwardLed        5
#define PWM_Pin            6
#define BUZZER_PIN         7
#define ssRX               8
#define ssTx               9
// 8 + 9 -> RX/TX Software Serial for DEBUG
#define Wire_Connection_Check_Pin 10
#define Red_Led_Battery_lvl    11
#define Yellow_Led_Battery_lvl 12
#define Green_Led_Battery_lvl  13

//Wireless settings + values
int SpeedValue_Now = ZeroPWM; //stop command
int Step_For_Move = 0;
//CounterRxPackets (0),Speed_1H/L (1-2) , Speed_2H/L (3-4), I_1H/L (5-6), I_2H/L (7-8), V_Roper_H/L (9-10), PWM_Value (11), Direction (12)
uint8_t testArray [] = {0, 111, 22, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
SerialStar mbee = SerialStar();
RxResponse rx = RxResponse();
//ЗАЮЗАТЬ?
const uint16_t remoteAddress = 0x0001; //Адрес модема, которому будут передаваться данные.
TxRequest tx = TxRequest(); //Пакет с данными для отправки удаленному модему. Для экономии памяти будем использовать один и тот же объект для отправки всех пакетов.
TxStatusResponse txStatus = TxStatusResponse(); //Локальный ответ со статусом команды.
RxAcknowledgeResponse remoteRxStatus = RxAcknowledgeResponse(); //Пакет с подтверждением от удаленного модема получение данных.

//
int InputValue = 0;
int CounterOfPacketsFromTx = 0;
uint8_t option = 0;
uint8_t data = 0;
int cntr = 0;
int Counter_To_Start_WDT = 0;
int E1 = 0; // Failed Checksum
int E2 = 0; // Wrong length of RX packet
int E3 = 0; // Not correct Start byte
int E4 = 0; // Corrupted frame
int ErrorSum = 0;
int ChainComboErrors = 0;
bool WDT_ACTIVE = false;
bool AlarmTrigger = false;
bool Low_Battery = false;
bool wrong_command = false;
bool FLAG_Release_Command = false;
bool SuddenReverse = false;
bool Release_The_Brakes = false;
bool Current_Overload = false;
// new feature
bool WireConnectionFlag_RX = false;
bool Packet_Received_Flag = false;
int GoodRx = 0;
double KoefGood = 0;
int Current_1 = 0;
int Speed_1 = 0;
int Current_2 = 0;
int Speed_2 = 0;
int BatteryCharge = 0;

//SoftwareSerial nss2(ssRX, ssTX);
PacketSerial WireSerial;
AltSoftSerial altSerial;

//Обработка прерывания по переполнению счётчика. Должны пищалкой пищать
//
//// MAIN
ISR(TIMER2_OVF_vect)
{
  cli();
  TCCR2B = 0b00000000; // STOP COUNT!!
  // it means no signal from RC
  Counter_To_Start_WDT++; // overflow  x1000 then start WDT.
  if (Counter_To_Start_WDT > 2000)  //1000 ok
  {
    //ALARM
    Alarm_ON();
    if (WDT_ACTIVE == false)
    {
     //wdt_enable(WDTO_4S); // WDT ENABLE!
      WDT_ACTIVE = true;
    }
    Counter_To_Start_WDT = 0;
    //Stop NOW
    digitalWrite(Direction_Of_Move , LOW);
    digitalWrite(Permission_Of_Move, LOW);
    SpeedValue_Now = ZeroPWM;
    analogWrite(PWM_Pin, SpeedValue_Now);
  }
  //it must rotate non stop
  if (FLAG_Release_Command == false)
  {
    if (cntr > 300)
    {
      FLAG_Release_Command = true;
      cntr = 0;
    }
    else cntr++;
  }
  TCCR2B = 0b00000011; // START COUNT!!
  sei();
}

void Reset_Error_Timer_And_Check_WDT()
{
  //  cli();
  if (AlarmTrigger == true)
  {
    if (WDT_ACTIVE == true)
    {
      wdt_disable();           //TURN OFF WDT!
      WDT_ACTIVE = false;
    }
    Alarm_OFF();
  }
  Counter_To_Start_WDT = 0;
  //  sei();
}

void Alarm_ON()
{
  AlarmTrigger = true;
  digitalWrite(BUZZER_PIN, HIGH);
  digitalWrite(ErrorLed_ADCPin, HIGH);
}

void Alarm_OFF()
{
  AlarmTrigger = false;
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(ErrorLed_ADCPin, LOW);
}

void setup()
{
  //Analog Pins setup
  pinMode(A0, INPUT);                 //  A0 XT3 hole
  pinMode(SWITCHER_PIN, OUTPUT);      // A1
  pinMode(ErrorLed_ADCPin, OUTPUT);   // A2
  pinMode(ADCpin_BatteryLvL, INPUT);  // A3 lvlBattery
  pinMode(ADCpin_Speed2, INPUT);      // A4
  pinMode(ADCpin_Current2, INPUT);    // A5
  pinMode(ADCpin_Speed1, INPUT);      // A6
  pinMode(ADCpin_Current1, INPUT);    // A7

  //Digital Pins setup
  pinMode(Permission_Of_Move, OUTPUT);             // 2
  pinMode(Direction_Of_Move, OUTPUT);              // 3
  pinMode(ForwardLed, OUTPUT);                     // 4
  pinMode(BackwardLed, OUTPUT);                    // 5
  //PWM_Pin for Escons                             // 6
  pinMode(BUZZER_PIN, OUTPUT);                     // 7
  // 8 ssRx 9 ssTx for Software Debug              // 8 - 9
  pinMode(Wire_Connection_Check_Pin, INPUT);       // 10
  pinMode(Red_Led_Battery_lvl, OUTPUT);            // 11
  pinMode(Yellow_Led_Battery_lvl, OUTPUT);         // 12
  pinMode(Green_Led_Battery_lvl, OUTPUT);          // 13

  //Serials setup
  PC_Debug.begin(57600);
  MBee_Serial.begin(115200);
  while (!Serial);
  WireSerial.begin(115200);
  WireSerial.setPacketHandler(&onPacketReceived);
  //  MBee_Serial.println("start_mbee_line");
  mbee.begin(MBee_Serial);

  //  //Настройка таймера 0 (T1) 16 bit
  //  TCNT0 = 0;   // счетный регистр (туда "капают" значения)
  //  OCR0A =  200;           //0b1000000000000000 // регистр сравнения. (уставка)
  //  OCR0B =  0;
  //  TIMSK0 = 0b00000011; //0b00000010 - OCIE0A (1) разрешают прерывания при совпадении с регистром A (OCR1A)
  //  TIFR0  = 0;
  //  TCCR0A = 0b00000011; //0b00000000 -> Normal режим (никаких ШИМ) / Не ставили условий для ноги OC1A(12)
  //  TCCR0B = 0b00000011;  // WGM12 == 1 -> [0100] - режим счета импульсов (OCR1A) (сброс при совпадении) + 101 - CLK/1024 == 0b00001101;

  //  MAIN
  //    Настройка таймера 2 (T2)
  TCNT2 = 0;       //  Это просто регистр со значениями куда всё тикает
  OCR2A =  100;    //уставка в регистр
  TIMSK2 = 0b00000001;  //0b00000001 - TOIE - до переполнения  + 0b00000010 до уставки в OCR2A // ИЛИ -> TIMSK2 = 1; ИЛИ -> TIMSK2 |= (1 << TOIE2);
  TCCR2A = 0;      // 0b00000000 -> Normal режим (никаких ШИМ) / Не ставили условий для ноги OC0A(12)
  TCCR2B = 0b00000011;  //START_TIMER -> 0b10 -> clk/8 (CS02 / CS01 / CS00) START_TIMER

  //GOTO
  //добавить посыл СТОП на двигатели без зажима тормозами. Уложись в 1-2 секунды!
  Starting_Command_For_Motors();
  //GOTO
  //Settings for MBEE
  //ДОПИЛИТЬ КУСКИ ДЛЯ СТАРТОВЫХ НАСТРОЕК Mbee
  Refresh_WireConnectionFlag_RX();
  if (WireConnectionFlag_RX == false)
  {
    //   setDefault_parameters_For_Mbee(); // AP = 6, no sleep,
    //   +setDefault_TX_Power_Level();
  }
  Send_Telemetry(WireConnectionFlag_RX); //starting scream

  PC_Debug.println();
  PC_Debug.println("__RX started!__");
  //  delay(500);  //Задержка не обязательна и вставлена для удобства работы с терминальной программой.
}

void loop()
{
  //  MBee_Serial.println("MBEE_loop _ x");
  ADCread();
  setBatteryCharge_Leds(BatteryCharge);
  setArrayForTelemetry();

  //Connection Type check
  Refresh_WireConnectionFlag_RX();
  Set_SWITCHER_PIN(WireConnectionFlag_RX);

  //Check connection type PIN and SEND telemetry
  if (WireConnectionFlag_RX == true)
  {
    PC_Debug.println("Wire mode");
    WireSerial.update();
    if (WireSerial.overflow())
    {
      // Send an alert via a pin (e.g. make an overflow LED) or return a
      // user-defined packet to the sender.
      //
      // Ultimately you may need to just increase your recieve buffer via the
      // template parameters (see the README.md).
      PC_Debug.println(F("WireSerial.overflow() oO"));
    }
    if (Packet_Received_Flag == true)
    {
      Send_Telemetry(WireConnectionFlag_RX);
      Packet_Received_Flag = false;
    }
  }
  else // WireConnectionFlag_RX == false
  {
    PC_Debug.println("Wireless mode");
    tx.setPayload((uint8_t*)testArray); //Устанавливаем указатель на тестовый массив
    tx.setPayloadLength(sizeof(testArray));  //Устанавливаем длину поля данных
    mbee.readPacket(); //Постоянно проверяем наличие данных от модема.
    if (mbee.getResponse().isAvailable())
    {
      PC_Debug.println("RX received an input packet!");
      ChainComboErrors = 0; // reset combo
      PC_Debug.print("API ID === ");
      PC_Debug.println( mbee.getResponse().getApiId() );
      PC_Debug.print("RSSI -> ");
      PC_Debug.println(rx.getRssi());
      if (mbee.getResponse().getApiId() == RECEIVE_PACKET_API_FRAME || mbee.getResponse().getApiId() == RECEIVE_PACKET_NO_OPTIONS_API_FRAME) // 0x81==129 and 0x8F==143
      {
        //        cli();
        mbee.getResponse().getRxResponse(rx); //Получаем пакет с данными.
        //      Debug info
        //      Debug_information_About_Rx_Packet();
        //      PC_Debug.println( mbee.getResponse().getApiId() ); // tut RECEIVE_PACKET_NO_OPTIONS_API_FRAME == 143 == 0x8F
        //      Debug
        //      int8_t size = rx.getDataLength();
        //      bool WriteAllArrayFlag = false;
        //      Print_All_Array(size, WriteAllArrayFlag);

        if (rx.getDataLength() > 1)
        {
          PC_Debug.println("Input Data :");
          for (int i = 0 ; i < rx.getDataLength() ; i++ )
          {
            if ( i == 8 || i == 9 || i == 10 ) {
              PC_Debug.print("Value [");
              PC_Debug.print(i);
              PC_Debug.print("] = ");
              PC_Debug.println(rx.getData()[i]);
              if (i == 8) {
                CounterOfPacketsFromTx = rx.getData()[i]; // 8ой байт номер пакета посланный с хоста (маркер)
              }
            }
          }
        }
        PC_Debug.print("Counter Received Packets :");
        PC_Debug.println(CounterOfPacketsFromTx);
        PC_Debug.println();
        if (rx.getData()[9] == Stop || rx.getData()[9] == Forward || rx.getData()[9] == Backward)
        {
          InputValue = rx.getData()[9];  // 9ый байт наша команда (куда ехать)
          Reset_Error_Timer_And_Check_WDT();
          Alarm_OFF();
        }


        //        Release_The_Brakes = false;
        //        if (rx.getData()[3] == EmergencyStopCode && WireConnectionFlag_RX == false) //52
        //        {
        //          //GOTO проверить аварийку
        //          InputValue = Stop;
        //          Release_The_Brakes = true;
        //          Command_To_Motor(InputValue);
        //        }
        //
        //        if (rx.getData()[3] == BOOST_Code_ON && WireConnectionFlag_RX == false)  // ??
        //        {
        //          //GOTO
        //          BOOST_ON();
        //        }
        //        if (rx.getData()[3] == BOOST_Code_OFF && WireConnectionFlag_RX == false) // ??
        //        {
        //          //GOTO
        //          BOOST_OFF();
        //        }


        //Command_To_Motor(InputValue); // old version to move ours legs
        GoodRx = GoodRx + 1;
        Packet_Received_Flag = true;
        //        sei();
      }
      else
      {
        PC_Debug.println(F("Corrupted frame (maybe )"));
        E4 = E4 + 1;
        ChainComboErrors = ChainComboErrors + 1;
        delay(100);
      }
      /////////----------------------------------------------/////////
      if (Packet_Received_Flag == true)
      {
        Send_Telemetry(WireConnectionFlag_RX);
        Packet_Received_Flag = false;
      }
      /////////----------------------------------------------/////////
    }
    else if (mbee.getResponse().isError())
    {
      PC_Debug.print("ERRORS!! ");
      PC_Debug.print(" № == ");
      PC_Debug.println(mbee.getResponse().getErrorCode());
      //      ChainComboErrors = ChainComboErrors + 1;
      delay(100);
      switch (mbee.getResponse().getErrorCode())
      {
        case 1:
          E1 = E1 + 1;
          break;
        case 2:
          E2 = E2 + 1;
          break;
        case 3:
          E3 = E3 + 1;
          break;
      }
      //Debug Errored array
      int8_t size = rx.getDataLength();
      bool WriteAllArrayFlag = false;
      Print_All_Array(size, WriteAllArrayFlag);
      //При разборе принятого пакета произошли ошибки.
    }
    if (ChainComboErrors >= 100 )
    {
      PC_Debug.println(F("RESET ARDUINO. A terrible long combo ERRORS !!!")); //RESET
      WDT_ACTIVE = true;
      wdt_enable(WDTO_4S); // WDT ENABLE!
      delay(50);
    }
    ErrorSum = E1 + E2 + E3 + E4;
    if (ErrorSum >= 500)
    {
      PC_Debug.print("ErrorSum = ");
      PC_Debug.println(ErrorSum);
      WDT_ACTIVE = true;
      wdt_enable(WDTO_4S); // WDT ENABLE!
      //      PC_Debug.print("GoodRx = ");
      //      PC_Debug.println(GoodRx);
      //      PC_Debug.print("E1 = ");
      //      PC_Debug.println(E1);
      //      PC_Debug.print("E2 = ");
      //      PC_Debug.println(E2);
      //      PC_Debug.print("E3 = ");
      //      PC_Debug.println(E3);
      //      PC_Debug.print("E4 = ");
      //      PC_Debug.println(E4);
    }
  }
  /////////////////////////////////////////////////////////////////

  //  Data_Send_To_Processing();
  if (FLAG_Release_Command == true)
  {
    PC_Debug.println("Rotate again");
    PC_Debug.print("InputValue - ");
    PC_Debug.println(InputValue);
    Command_To_Motor(InputValue);
    FLAG_Release_Command = false;
  }
  /////////////////////////////////////////////////////////////////

  // Alarm ON/OFF Check
  if (Low_Battery == false && AlarmTrigger == false && WDT_ACTIVE == false && wrong_command == false)
  {
    Alarm_OFF();
  }
  if (Low_Battery == true || AlarmTrigger == true || WDT_ACTIVE == true || wrong_command == true)
  {
    Alarm_ON();
  }

  PC_Debug.print("CounterTX: ");
  PC_Debug.println(CounterOfPacketsFromTx);
  PC_Debug.print("Command: ");
  PC_Debug.println(InputValue);
  PC_Debug.println("END loop");
  delay(30);
  return;
}

// input command packet
void onPacketReceived(const uint8_t* buffer, size_t size)
{
  if (WireConnectionFlag_RX != true)
  {
    PC_Debug.println("Wrong Handler");
    return;
  }
  PC_Debug.println("TX Packet is Read");
  uint8_t tempBuffer[size];
  // Copy the packet into our temporary buffer.
  memcpy(tempBuffer, buffer, size);
  PC_Debug.println("Buffer:");
  for (int i = 0 ; i < size ; i++)
  {
    PC_Debug.print("[");
    PC_Debug.print(i);
    PC_Debug.print("]=");
    PC_Debug.print(tempBuffer[i]);
    PC_Debug.print("; ");
    if ( size >= 2)
    {
      if (i == 0)
      {
        CounterOfPacketsFromTx = tempBuffer[i];
      }
      if (i == 1)
      {
        if (tempBuffer[i] == Stop || tempBuffer[i] == Forward || tempBuffer[i] == Backward)
        {
          InputValue = tempBuffer[i];
          Reset_Error_Timer_And_Check_WDT();
          wrong_command == false;
        }
        else
        {
          PC_Debug.println("wrong command");
          InputValue = Stop;
          wrong_command = true;
        }
      }

    }
  }

  PC_Debug.println("");

  //GOTO обработку Буст команды и аварийного стопа.
  //  PC_Debug.println("");
  //  InputValue = tempBuffer[1];  // 1ый( второй по смыслу) байт наша команда (куда ехать)
  //  if (tempBuffer[3] == EmergencyStopCode)
  //  {
  //    InputValue = Stop;
  //    Command_To_Motor(InputValue);
  //    Release_The_Brakes = true;
  //    Release_The_Brakes = false;
  //  }
  //  if (tempBuffer[3] == BOOST_Code_ON)
  //  {
  //    //GOTO
  //    BOOST_ON();
  //  }
  //  if (tempBuffer[3] == BOOST_Code_OFF)
  //  {
  //    //GOTO
  //    BOOST_OFF();
  //  }
  //  Release_The_Brakes = false;
  //  //  Counter_From_RC = tempBuffer[0];
  //  //  RX_counter++;
  delay(10);
  Packet_Received_Flag = true;
  return;
}

void Refresh_WireConnectionFlag_RX()
{
  PC_Debug.println(F("Refresh_WireConnectionFlag_TX!"));
  bool First_WireConnection_Check = digitalRead(Wire_Connection_Check_Pin);
  if (WireConnectionFlag_RX != First_WireConnection_Check)
  {
    delay(5);
    bool Second_WireConnection_Check = digitalRead(Wire_Connection_Check_Pin);
    if (Second_WireConnection_Check == First_WireConnection_Check)
    {
      if (Second_WireConnection_Check == true) // WIRE ON
      {
        PC_Debug.println("-|- Change to -> Wire connect");
        WireConnectionFlag_RX = true;
      }
      else                                      // WIRE OFF
      {
        PC_Debug.println("-))  ((- Change to -> WireLESS connect");
        WireConnectionFlag_RX = false;
      }
    }
  }
  return;
}

void Set_SWITCHER_PIN(bool WireFlag)
{
  PC_Debug.print("WireFlag =>");
  PC_Debug.println(WireFlag);
  if (WireFlag == true)
  {
    digitalWrite(SWITCHER_PIN, LOW); // inverted COM(4) <-> NC (3)
    PC_Debug.println("Radio_Led LOW");
    delay(5);
  }
  if (WireFlag == false)
  {
    digitalWrite(SWITCHER_PIN, HIGH); // inverted D3A/D6A COM(4) <-> NO (1)
    PC_Debug.println("Radio_Led HIGH");
    delay(5);
  }
  return;
}

void Send_Telemetry( bool WireFlag_RX )
{

  if (WireFlag_RX == true)
  {
    PC_Debug.println("Wire Connect...wait");
    // No send!
    WireSerial.send(testArray, sizeof(testArray) / sizeof(testArray[0]) );
    delay(50);
  }
  else
  {
    mbee.send(tx);
    PC_Debug.print("Current_1 ==");
    PC_Debug.println(Current_1);
    PC_Debug.print("Speed_1 ==");
    PC_Debug.println(Speed_1);
    PC_Debug.print("Current_2 ==");
    PC_Debug.println(Current_2);
    PC_Debug.print("Speed_2 ==");
    PC_Debug.println(Speed_2);
    PC_Debug.println("RX SEND telemetry.");
    //  if (tx.getFrameId()) //Проверяем, не заблокировано ли локальное подтверждение отправки.
    //    getLocalResponse(50);
    //  if ((tx.getFrameId() == 0) || (txStatus.isSuccess() && tx.getSleepingDevice() == false)) //Ждем ответного пакета от удаленного модема только если локальный ответ выключен или пакет отправлен в эфир и не предназначается спящему модему.
    //    getRemoteResponse(5);
    delay(50);
    return;
  }
}

void Command_To_Motor(int instruction)
{
  if (instruction == Corrupted_Command)
  { //GOTO
    // NEED ADD REACTION On "ERROR_VALUE"
    PC_Debug.println(F("Corrupted_Command"));
  }
  //KURKUMA
  if ( (instruction == Forward & SpeedValue_Now < ZeroPWM ) | (instruction == Backward & SpeedValue_Now > ZeroPWM))
  {
    instruction = Stop;
    SuddenReverse = true;
    PC_Debug.println("Reverse_detected ");
    PC_Debug.print("instruction - ");
    PC_Debug.println(instruction);
    //    delay(3000);
  }
  if (instruction == Stop)
  {
    if (SpeedValue_Now == ZeroPWM)
    {
      Step_For_Move = 0; // command already complete
    }

    if (SpeedValue_Now > ZeroPWM || SpeedValue_Now < -ZeroPWM )
    {
      Step_For_Move = 0; // pull to zero // 1 - FOR TESTS , 7 - for real fights
      SpeedValue_Now = ZeroPWM;
    }

    //    if (SpeedValue_Now > ZeroPWM)
    //    {
    //      Step_For_Move = -4; // pull to zero // 1 - FOR TESTS , 7 - for real fights
    //    }
    //    if (SpeedValue_Now < ZeroPWM)
    //    {
    //      Step_For_Move = 4; // pull to zero // 1 - FOR TESTS , 7 - for real fights
    //    }

    //Calculate
    SpeedValue_Now = SpeedValue_Now + Step_For_Move;
    //Teleport
    if ( ( (SpeedValue_Now <= ZeroPWM) & (SpeedValue_Now >= ZeroPWM - abs(Step_For_Move)) )                       //  (X <= 30) & (X >= 25)
         | ( (SpeedValue_Now >= (-1) * ZeroPWM) & (SpeedValue_Now <= ( (-1) * ZeroPWM + abs(Step_For_Move)) ) ) ) // ( X >= -30) & ( X <= -25 )
    {
      SpeedValue_Now = ZeroPWM;
      Step_For_Move = 0;  // command complete
      //PC_Debug.println("STOP NOW");
    }
    //Debug
    PC_Debug.print("(Stopping)Speed == ");
    PC_Debug.println(SpeedValue_Now);
    PC_Debug.println("---------------");
  }
  if (instruction == Forward)
  {
    if (SpeedValue_Now == Max_SpeedValue) // check for Wished_Speed
    {
      //PC_Debug.println("Max Speed is reached! (already)");
      Step_For_Move = 0;
    }
    if (SpeedValue_Now >= Min_SpeedValue && SpeedValue_Now < Max_SpeedValue)
    {
      Step_For_Move = 4; //pull to 225 (Max_SpeedValue)
    }
    SpeedValue_Now = SpeedValue_Now + Step_For_Move;  // increment or decrement PWM
    //Teleport
    if (SpeedValue_Now >= ZeroPWM * (-1) & (SpeedValue_Now <= ((-1) * ZeroPWM + abs(Step_For_Move)) ) )  // (X >= -30 & X <= -25 )
      SpeedValue_Now = ZeroPWM;
    // FINISHER
    if (SpeedValue_Now >= Max_SpeedValue & SpeedValue_Now <= ( Max_SpeedValue + abs(Step_For_Move) )) // ==Wished_Speed
    {
      SpeedValue_Now = Max_SpeedValue;
      Step_For_Move = 0;  //command complete
      //PC_Debug.println("Max Speed RIGHT NOW! LvL UP! ");
    }
    //Debug
    PC_Debug.print("(Grow) Speed == ");
    PC_Debug.println(SpeedValue_Now);
    PC_Debug.println("---------------");
  }
  //Backward
  if (instruction == Backward)
  {
    if (SpeedValue_Now == Min_SpeedValue)
    {
      //PC_Debug.println("Min Speed is reached! (already)");
      Step_For_Move = 0; // pull to 255 (Max_SpeedValue)
    }
    if (SpeedValue_Now > Min_SpeedValue && SpeedValue_Now <= Max_SpeedValue)
    {
      Step_For_Move = -4;  // -1 - FOR TESTS , -7 - for real fights
    }
    //Teleport
    SpeedValue_Now = SpeedValue_Now + Step_For_Move;
    if ( (SpeedValue_Now <= ZeroPWM) & (SpeedValue_Now >= ZeroPWM - abs(Step_For_Move) ) ) // == 30 |  ( X <= 30 & X >= 25)
      SpeedValue_Now = ZeroPWM * (-1);
    // FINISHER
    if (SpeedValue_Now <= Min_SpeedValue)
    {
      SpeedValue_Now = Min_SpeedValue;
      Step_For_Move = 0;  // command complete
      //PC_Debug.println("We Reversed");
    }
    //Debug
    PC_Debug.print("(Drop) Speed == ");
    PC_Debug.println(SpeedValue_Now);
  }
  Execute_The_Command(SpeedValue_Now);
  return;
}

void Execute_The_Command(int Speed)
{
  if (Speed >= Min_SpeedValue & Speed <= Max_SpeedValue)
  {
    if (Speed > ZeroPWM) //F
    {
      digitalWrite(Direction_Of_Move, HIGH);
      digitalWrite(Permission_Of_Move, HIGH);
      analogWrite(PWM_Pin, Speed);
      //Debug
      PC_Debug.println("FORWARD --->>>>");
      //digitalWrite(ForwardLed, HIGH);
      //digitalWrite(BackwardLed, LOW);
      digitalWrite(BackwardLed, HIGH);
    }
    if (Speed < ZeroPWM) //B
    {
      digitalWrite(Direction_Of_Move, LOW);
      digitalWrite(Permission_Of_Move, HIGH);
      analogWrite(PWM_Pin, abs(Speed)); // since it is less than 0
      //Debug
      PC_Debug.println("<<<<--- BACKWARD");
      //digitalWrite(ForwardLed, LOW);
      digitalWrite(BackwardLed, HIGH);
    }
    if (Speed == ZeroPWM) //S
    {
      if (Release_The_Brakes == false)
      {
        digitalWrite(Direction_Of_Move, LOW);
        digitalWrite(Permission_Of_Move, HIGH);  // HIGH for normal braking
        analogWrite(PWM_Pin, Speed);
        //Debug
        //PC_Debug.println("|| STOP ||");
        //digitalWrite(ForwardLed, LOW);
        digitalWrite(BackwardLed, LOW);
      }
      else
      {
        digitalWrite(Direction_Of_Move, LOW);
        digitalWrite(Permission_Of_Move, LOW);  // LOW EMERGENCY braking ONLY
        analogWrite(PWM_Pin, Speed);
        //Debug
        //PC_Debug.println("|| STOP ||");
        //digitalWrite(ForwardLed, HIGH);
        digitalWrite(BackwardLed, HIGH);
      }
      if ( SuddenReverse == true);
      {
        SuddenReverse = false;
      }
    }
    digitalWrite(ForwardLed, LOW);  // expected data
  }
  else
  {
    // STOP RIGHT NOW
    digitalWrite(Direction_Of_Move, LOW);
    digitalWrite(Permission_Of_Move, LOW);
    analogWrite(PWM_Pin, ZeroPWM);
    SpeedValue_Now = ZeroPWM;
    Step_For_Move = 0;
    //Debug
    PC_Debug.print("Error!!! Speed == ");
    PC_Debug.println(SpeedValue_Now);
    //    PC_Debug.println("");
    //    PC_Debug.print("RESET -> Speed Value_Now = ");
    //    PC_Debug.println(SpeedValue_Now);
    //    PC_Debug.println("");
    //      PC_Debug.println("ERROR from Host (Switcher have bugs or lags) , sadness ");
    //Debug on LEDs
    digitalWrite(ForwardLed, HIGH);
    digitalWrite(BackwardLed, HIGH);
  }
}

//DEBUG feature
void Debug_information_About_Rx_Packet()
{
  PC_Debug.println("---------------------DEBUG------------------");
  //Print of Received Packet  Length
  PC_Debug.print("Packet Length -> ");
  PC_Debug.println(rx.getPacketLength());
  //Print of MSB Length
  PC_Debug.print("MSB Lenght -> ");
  PC_Debug.println(rx.getMsbLength());
  //Print of LSB Length
  PC_Debug.print("LSB Lenght -> ");
  PC_Debug.println(rx.getLsbLength());
  //Print of API ID
  PC_Debug.print("API ID -> ");
  PC_Debug.println(rx.getApiId());
  // Print Checksum
  PC_Debug.print("Checksum == ");
  PC_Debug.println(rx.getChecksum());
  // Print Frame Data Length
  PC_Debug.print("Frame Data Lenght -> ");
  PC_Debug.println(rx.getFrameDataLength());
  // Print Data Length
  PC_Debug.print("Data Length -> ");
  PC_Debug.println(rx.getDataLength());
  // Print RSSI
  PC_Debug.print("RSSI -> ");
  PC_Debug.println(rx.getRssi());

  PC_Debug.println("---------------------DEBUG------------------");
}

void Print_All_Array(int8_t Size, boolean WriteEnable)
{
  // ALL PACK PRINT!!
  for (int i = 0 ; i < Size ; i ++ )
  {
    //"PRINT"
    PC_Debug.print("Input testArray[");
    PC_Debug.print(i);
    PC_Debug.print("]= ");
    PC_Debug.println(rx.getData()[i]);
    //"WRITE"
    if (WriteEnable == true)
    {
      PC_Debug.print(" WRITE inside i -> ");
      PC_Debug.write(rx.getData()[i]);
      PC_Debug.println();
    }
  }
}

// not use
void EmergencyStop()
{
  //  cli();
  // Alarm_ON();
  // STOP RIGHT NOW
  SpeedValue_Now = ZeroPWM;
  Step_For_Move = 0;
  digitalWrite(Direction_Of_Move, LOW);   // оттормаживаем двигатели. ппадение под собственным весом
  digitalWrite(Permission_Of_Move, LOW);
  analogWrite(PWM_Pin, ZeroPWM);
  //  sei();
  return;
}

// goto (x5 read / 5)
// 4 analog reads - 8 value for telemetry
void ADCread() {
  int C1 = 0;
  int C2 = 0;
  int S1 = 0;
  int S2 = 0;
  int Bat = 0;

  for (int i = 0; i < 5 ; i++)
  {
    C1 = analogRead(ADCpin_Current1) + C1;
    C2 = analogRead(ADCpin_Current2) + C2;
    S2 = analogRead(ADCpin_Speed2)   + S2;
    S1 = analogRead(ADCpin_Speed1)   + S1;
    Bat = analogRead(ADCpin_BatteryLvL) + Bat;
  }

  Current_1 = C1 / 5;
  Current_2 = C2 / 5;
  Speed_2 = S2 / 5;
  Speed_1 = S1 / 5;
  BatteryCharge = Bat / 5;
  PC_Debug.print("BatteryCharge-");
  PC_Debug.println(BatteryCharge);
  //
  //  Current_1 = analogRead(ADCpin_Current1);
  //  Current_2 = analogRead(ADCpin_Current2);
  //  Speed_2 = analogRead(ADCpin_Speed2);
  //  Speed_1 = analogRead(ADCpin_Speed1);
  //  BatteryCharge = analogRead(ADCpin_BatteryLvL);
  return;
}




void setBatteryCharge_Leds(int ChargeFromADC)
{
  //GOTO
  //ПЕРЕСЧИТАЙ ЗНАЧЕНИЯ ПОД РЕАЛИИ
  //  int percent_ADC = map(ChargeFromADC, 0 , 1023, 0, 100);

  if (ChargeFromADC >= 0 & ChargeFromADC < 573)  // [0 - 22) V
  {
    digitalWrite(Red_Led_Battery_lvl, LOW);
    digitalWrite(Yellow_Led_Battery_lvl, LOW);
    digitalWrite(Green_Led_Battery_lvl, LOW);
    //SetAlarm
    Low_Battery = true;
  }
  if (ChargeFromADC >= 573 & ChargeFromADC < 600)  // [22 - 23) V
  {
    digitalWrite(Red_Led_Battery_lvl, HIGH);
    digitalWrite(Yellow_Led_Battery_lvl, LOW);
    digitalWrite(Green_Led_Battery_lvl, LOW);
    if (Low_Battery == true)
    {
      Low_Battery = false;
    }
  }
  if (ChargeFromADC >= 600 & ChargeFromADC < 627)   // [23 - 24) V
  {
    digitalWrite(Red_Led_Battery_lvl, HIGH);
    digitalWrite(Yellow_Led_Battery_lvl, HIGH);
    digitalWrite(Green_Led_Battery_lvl, LOW);
    if (Low_Battery == true)
    {
      Low_Battery = false;
    }
  }
  if (ChargeFromADC >= 627 & ChargeFromADC <= 1023)   // [24- 40.1] V, but in real life [24 - 30] V  {{ (627 - ~ 800] }}
  {
    digitalWrite(Red_Led_Battery_lvl, HIGH);
    digitalWrite(Yellow_Led_Battery_lvl, HIGH);
    digitalWrite(Green_Led_Battery_lvl, HIGH);
    PC_Debug.println(F("Full_Charge")); // DELETE THIS
    if (Low_Battery == true)
    {
      Low_Battery = false;
    }
  }
  return;
}

// 0-13 = 14 elements
void setArrayForTelemetry() {
  int Direction = 111;
  if (SpeedValue_Now == 30 || SpeedValue_Now == -30)
    Direction = 0; //stop
  if (SpeedValue_Now > 30)
    Direction = 5; //rush
  if (SpeedValue_Now < -30)
    Direction = 2; //back
  //set values
  testArray [0] = CounterOfPacketsFromTx;
  testArray [1] = Speed_1;       // Speed_1L
  testArray [2] = Speed_1 >> 8;  // Speed_1H
  testArray [3] = Speed_2;
  testArray [4] = Speed_2 >> 8;
  testArray [5] = Current_1;
  testArray [6] = Current_1 >> 8;
  testArray [7] = Current_2;
  testArray [8] = Current_2 >> 8;
  testArray [9] = BatteryCharge;               // V_Roper L
  testArray [10] = BatteryCharge >> 8;         // V_Roper_H
  testArray [11] = abs(SpeedValue_Now); //PWM_value
  testArray [12] = Direction;      //Direction
  //  testArray [13] = Direction;
  return;
}

void Starting_Command_For_Motors()
{
  digitalWrite(Direction_Of_Move, LOW);
  digitalWrite(Permission_Of_Move, LOW);  // LOW EMERGENCY braking ONLY
  analogWrite(PWM_Pin, ZeroPWM);
  //Debug
  //      PC_Debug.println("|| STOP ||");
  digitalWrite(ForwardLed, LOW);
  digitalWrite(BackwardLed, LOW);
  return;
}

void Data_Send_To_Processing() {
  int Garbage = random(-1023, 1023);
  PC_Debug.print("Speed_1=");
  PC_Debug.print(Speed_1);
  PC_Debug.print(" ");

  PC_Debug.print("Speed_2=");
  PC_Debug.print(Speed_2);
  PC_Debug.print(" ");

  PC_Debug.print("I_1=");
  PC_Debug.print(Current_1);
  PC_Debug.print(" ");

  PC_Debug.print("I_2=");
  PC_Debug.print(Current_2);
  PC_Debug.print(" ");

  //  Serial.println(val);
  PC_Debug.print("Garbage=");
  PC_Debug.print(Garbage);
  PC_Debug.print(" ");
  PC_Debug.println();
  delay(30);
  return;
}

void setDefault_parameters_For_Mbee()
{

  return;
}



//TX_Power_Level
bool setDefault_TX_Power_Level()  //
{
  /*
    1) check connect to Mbee
    2) if YES -> at pl 0x1F (11111 or 31) ~14dbm
      if NO  -> return 1;
    3) catch "OK"
    4) if YES -> at ac
      if NO  ->
    5) catch "OK"
    6) at pl
    8) catch "0x1F" or 11111 or 31
    7) if YES -> finish + return 0;
      if NO  -> return 1
  */

  delay(1);
  return 0;
}

void BOOST_ON()
{
  // Create CODE
  /*
    1) Check connect with Mbee
    2) if YES -> at pl 0x81 /( 10000001 or 129 ) ~27 dbm for Mbee 2.0
      if NO -> send telemetry BOOST_ON_Flag == false;
    3) Catch answer like "OK"
    4)  if YES -> at ac + wait ( delay(10?) )//write EEPROM
       if NO -> send telemetry BOOST_ON_Flag == false;
    5) check like "OK"
    6)  if YES -> finish this function + return;
       if NO  -> send telemetry BOOST_ON_Flag == false;
  */

  delay(1);
  return;
}

void BOOST_OFF()
{
  // Create CODE
  /*
    1) Check connect with Mbee
    2) if YES -> at pl 0x1F /( 11111 or 31 ) ~14 dbm for Mbee 2.0
       if NO -> send telemetry BOOST_ON_Flag == false;
    3) Catch answer like "OK"
    4)  if YES -> at ac + wait ( delay(10?) )//write EEPROM
       if NO -> send telemetry BOOST_ON_Flag == false;
    5) check like "OK"
    6)  if YES -> finish this function + return;
       if NO  -> send telemetry BOOST_ON_Flag == false;
  */

  delay(2);
  return;
}


//
