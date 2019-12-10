//Описание
/*  Пульт радиоуправления способный передать команду самоходу по радиуправлению через MBee или по кабелю
    и принять телеметрию , далее отобразить основные парамерты на дисплее.
    Имеет возможность активировать звуковую индикацию и посылать отладочные сообщения через пины 8 и 9 на PC (SoftWareSerial)
    Отображение на дисплее скорости, нагрузки на двигатели, качества сигнала (RSSI) для беспроводного управения,
    напряжения батареи пульта и заряда аккумлятора самохода.
    Защита от одновременного зажатия ВПЕРЕД И НАЗАД
    Пока отсутствует перезагрузка пульта, не найдено весомых причин для перезагрузки.
    //GO TO Quests
    // EASY
    !! добавить отладку параметров с самохода через DEBUG Serial
    //MEDIUM
    !! добавить индикацию светодиодами для Проводного/Беспроводного управления
    !! добавить управление ключами через SWITCHER_Pin для работы через нужные "рельсы"
    !- добавить функции для работы с дисплеем
    // не работает SOFTWARESERIAL =(
    //HARD
    + дальнейшая проверка работы через кабель/провода и используя дисплей выводить нужные данные
    + дальнейшая работа через кабель и Mbee Wireless и отображая текущие значения на дисплее!
*/

//INCLUDE
#include <MBee.h>
#include <SoftwareSerial.h>
//#include <AltSoftSerial.h>
#include <PacketSerial.h>
//#include <LiquidCrystalRus.h>
#include <LiquidCrystal.h>
//DEFINE
//Ports Rename
#define DEBUG nss
//#define DEBUG altSerial  // nss or altSerial
#define MBee_Serial Serial
//Const
#define Stop 0
#define Forward 5
#define Backward 2
#define CorruptedCommand 10
//const for PC Processing debug
#define ADC_Speed_RPM_Step 16.67
#define ADC_Current_Step 0.0125
#define ADC_Climber_Voltage_Step 0.0047
#define ADC_RC_Voltage_Step 1
#define ADC_PWM_Step 0.4

//Pins  Digital // 0 - 1   tx - rx
#define HardwareRX_pin     0    // useless name
#define HardwareTX_pin     1    // useless name
#define Forward_Check_Pin  2    // +++
#define Backward_Check_Pin 3    // +++
#define DISPAY_DB7         4    // test
#define DISPAY_DB6         5    // test
#define DISPAY_DB5         6    // test
#define DISPAY_DB4         7    // test
#define SoftwareRX         8    // ???
#define SoftwareTX         9    // ???
#define Wire_Connection_Check_Pin 10  // specify in the scheme
#define DISPLAY_E          11   // test
#define DISPLAY_RS         12   // test
#define SWITCHER_PIN       13   // 0 - WIRE?  1 - Mbee?
//

//#define SWITCHER_Pin 6   // specify in the scheme
//#define Wire_Connection_Check_Pin 10  // specify in the scheme
//#define ssRX 8           // 100% certainty
//#define ssTX 9           // 100% certainty
//#define Forward_Led  11  // specify in the scheme
//#define Backward_Led 12  // specify in the scheme
//#define Stop_Led     13  // specify in the scheme?

//
//#define Green_WireLess_Led YesOrNo //does it exist?
//Analog Pins
#define Forward_Led  A0        // check on the scheme
#define Stop_Led     A1        // check on the scheme
#define Backward_Led A2        // check on the scheme
#define Radio_Led    A3        // check on the scheme      // GOTO RADIOLED indication
#define BOOST_Led    A4        // check on the scheme
#define BUZZER_Pin   A5        // specify in the scheme 
#define BOOST_CHECK  A6        // check on the scheme      // GOTO boost feature + BOO
#define ADC_Battery_LvL_Pin A7 // specify in the scheme

SoftwareSerial nss(SoftwareRX, SoftwareTX);
//AltSoftSerial altSerial;
PacketSerial WireSerial;
SerialStar mbee = SerialStar();

// specify in the scheme
//LiquidCrystalRus disp(DISPLAY_RS, DISPLAY_E, DISPAY_DB4, DISPAY_DB5, DISPAY_DB6, DISPAY_DB7); // создаем объект  // (12, 11, 7, 6 , 5 , 4)
LiquidCrystal disp(DISPLAY_RS, DISPLAY_E, DISPAY_DB4, DISPAY_DB5, DISPAY_DB6, DISPAY_DB7); // создаем объект  // (12, 11, 7, 6 , 5 , 4)
//TX SETUP
TxRequest tx = TxRequest(); //Пакет с данными для отправки удаленному модему. Для экономии памяти будем использовать один и тот же объект для отправки всех пакетов.
TxStatusResponse txStatus = TxStatusResponse(); //Локальный ответ со статусом команды.
//RX SETUP
RxResponse rx = RxResponse();
RxAcknowledgeResponse remoteRxStatus = RxAcknowledgeResponse(); //Пакет с подтверждением от удаленного модема получение данных.
//Arrays
// number_of_packet , CommandForRaper , ERROR Code?
uint8_t Command_Pack [] = {0, 1, 23};
//CounterRxPackets (0),Speed_1H/L (1-2) , Speed_2H/L (3-4), I_1H/L (5-6), I_2H/L (7-8), V_Roper_H/L (9-10), PWM_Value (11), Direction (12)
uint8_t Telemetry_Pack_From_RX [] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};

//Telemetry members
uint8_t RSSI_LvL = 0;
float Speed_1_RPM = 0;
float Speed_2_RPM = 0;
float I_1_A = 0;
float I_2_A = 0;
float Battery_Roper_Voltage = 0;
float PWM_Value_Percent = 12.3;
uint8_t Roper_Direction = 1;
//Data from TX
float Battery_RC_Voltage = 0;  //
// raw Telemetry H+L
int Speed_1_raw = 0;
int Speed_2_raw = 0;
int I_1_raw = 0;
int I_2_raw = 0;
int V_Roper_raw = 0; // 0-1023
//Raw Data from TX
int V_RC_raw = 0;    // 0-1023
uint8_t PWM_Value_raw = 1;

//Variables
bool WireConnectionFlag_TX = false;
int number_of_packet = 0;
uint8_t CommandForRaper = 0;
bool AlarmFlag = false;
bool ReadyToPrint_DataFromRX = false;
const uint16_t remoteAddress = 0x0001; //Адрес модема, которому будут передаваться данные.
//
//Create Custom Sign for Disp
byte Triangle0[8] = {
  0b11111,
  0b01110,
  0b00100,
  0b00100,
  0b00100,
  0b00000,
  0b00001,
  0b00111
};

byte Triangle1[8] = {
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00001,
  0b00111,
  0b11111,
  0b11111
};

byte Triangle2[8] = {
  0b00000,
  0b00000,
  0b00001,
  0b00111,
  0b11111,
  0b11111,
  0b11111,
  0b11111
};

byte Triangle3[8] = {
  0b00001,
  0b00111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111
};

byte RC_Char[8] = {
  0b01000,
  0b01000,
  0b11111,
  0b10001,
  0b10001,
  0b11111,
  0b11101,
  0b11111
};

byte Roper[8] = {
  0b11111,
  0b11011,
  0b10001,
  0b00100,
  0b10001,
  0b11011,
  0b11111,
  0b11111
};

void setup() {

  // Digital Pins Setup
  pinMode(Forward_Check_Pin , INPUT);   // D2
  pinMode(Backward_Check_Pin, INPUT);   // D3
  //  pinMode(DISPAY_DB7, XX) ;         // D4 -- 14 pin lcd             ( -- DB 7)
  //  pinMode(DISPAY_DB6, XX) ;         // D5 -- 13 pin lcd             ( -- DB 6)
  //  pinMode(DISPAY_DB5, XX) ;         // D6 -- 12 pin lcd             ( -- DB 6)
  //  pinMode(DISPAY_DB4, XX) ;         // D7 -- 11 pin lcd             ( -- DB 4)
  //  pinMode(SoftwareRX, XX) ;         // D8 SoftwareRX
  //  pinMode(SoftwareTX, XX) ;         // D9 SoftwareTX
  pinMode(Wire_Connection_Check_Pin, INPUT); // D10
  //  pinMode(DISPAY_E, XX) ;           // D11 -
  //  pinMode(DISPAY_RS, XX);           // D12
  pinMode(SWITCHER_PIN, OUTPUT);        // D13

  //Analog Pins Setup
  pinMode(Forward_Led, OUTPUT);               // A0
  pinMode(Stop_Led, OUTPUT);                  // A1
  pinMode(Backward_Led, OUTPUT);              // A2
  pinMode(Radio_Led, OUTPUT);                 // A3
  pinMode(BOOST_Led, OUTPUT);                 // A4
  pinMode(BUZZER_Pin, OUTPUT);                // A5
  pinMode(BOOST_CHECK, INPUT);                // A6
  pinMode(ADC_Battery_LvL_Pin, INPUT);        // A7

  //Ports setup
  DEBUG.begin(115200);
  MBee_Serial.begin(115200);
  while (!Serial);
  WireSerial.begin(115200);
  mbee.begin(MBee_Serial);
  DEBUG.println("Go");
  Refresh_WireConnectionFlag_TX();
  //DEBUG Starter handler
  WireSerial.setPacketHandler(&onPacketReceived);
  delay(50);
  disp.begin(20, 4);     // WEH2004 Winstar Display
  disp.setCursor(0, 0);  //
  //Display setup //PAIN
  reset_textmode();
  delay(50);
  OLED_Display_Const_Data();
  disp.createChar(2, Roper);
  disp.createChar(3, RC_Char);
  disp.createChar(4, Triangle0);
  disp.createChar(5, Triangle1);
  disp.createChar(6, Triangle2);
  disp.createChar(7, Triangle3);
}

void loop() {
  DEBUG.println("TX");

  //Read RC BatteryLvL
  V_RC_raw = analogRead(ADC_Battery_LvL_Pin);
  bool F = digitalRead(Forward_Check_Pin);
  bool B = digitalRead(Backward_Check_Pin);

  //Read command (and maybe error situation)  //kost
  CommandForRaper = ReadCommandFromSwitcher(F, B);
  //  CommandForRaper = Forward;

  //Fill the Command_Pack
  Fill_Command_Pack();

  // wire / wireless SEND with depends on WireConnectionFlag_TX
  Refresh_WireConnectionFlag_TX();
  Set_SWITCHER_PIN ( WireConnectionFlag_TX );
  Send_Command_Pack( WireConnectionFlag_TX );


  //Need Refresh this for PacketSerial if WireFlag == true
  //Wire packet catcher (through WireSerial.update ->  )
  Check_Recieve_Buffer( WireConnectionFlag_TX ); // WireSerial.update();

  //  Read_Telemetry_Pack_From_RX();

  // Display data from Telemetry
  if (ReadyToPrint_DataFromRX == true)
  {
    DEBUG.print("LCD_Print ");
    DEBUG.println(ReadyToPrint_DataFromRX);
    //
//    OLED_Display_Dynamic_Data();
    ReadyToPrint_DataFromRX = false;
  }
  //
  //
  //
  //
  //  char source [5] = { '-', '0', '1', '3', '\n'};
  //  DEBUG.print("src |");
  //  String Str = String(source);
  //  // int
  //  int intoviy = random(-150, 150);
  //  DEBUG.print("intoviy |");
  //  DEBUG.println(intoviy);
  //
  //  //---------------------------
  //  disp.setCursor(2, 0);
  //  disp.print(intoviy);
  //
  //  disp.setCursor(7, 0);
  //  disp.print("try->");
  //
  //  if (intoviy < 0 & intoviy >= -999 )
  //  {
  //    if (intoviy < 0 & intoviy > -10)
  //      disp.print("  ");
  //    if (intoviy <= -10 & intoviy > -100)
  //      disp.print(" ");
  //  }
  //  if (intoviy >= 0 & intoviy <= 999 )
  //  {
  //    if (intoviy >= 0 & intoviy < 10)
  //      disp.print("   ");
  //    if (intoviy >= 10 & intoviy < 100)
  //      disp.print("  ");
  //    if (intoviy >= 100 & intoviy < 999)
  //      disp.print(" ");
  //  }
  //
  //  if ( intoviy >= -999 & intoviy <= 999)
  //  {
  //    itoa (intoviy, source, 10);
  //    DEBUG.print("itoa(src) |");
  //    DEBUG.println(source);
  //  }
  //  disp.print(source);

  //  delay(500);

  delay(150);
  return;
}

int ReadCommandFromSwitcher(bool Rush, bool Back)
{
  //Check  a move command from  switch(remote controller)
  //  bool Rush = digitalRead(Forward_Check_Pin);
  //  bool Back = digitalRead(Backward_Check_Pin);
  int Command = CorruptedCommand;
  // Forward command
  if (Rush == true & Back == false)
  {
    Command = Forward;
    digitalWrite(Backward_Led, LOW);
    digitalWrite(Forward_Led, HIGH);
    digitalWrite(Stop_Led, LOW);
    //    DEBUG.println("Command RUSH ");
  }
  // Backward command
  if (Rush == false & Back == true)
  {
    Command = Backward;
    digitalWrite(Backward_Led, HIGH);
    digitalWrite(Forward_Led, LOW);
    digitalWrite(Stop_Led, LOW);
    //    DEBUG.println("Command BACK ");
  }
  // Stop command
  if (Rush == false & Back == false)
  {
    Command = Stop;
    digitalWrite(Backward_Led,  LOW);
    digitalWrite(Forward_Led, LOW);
    digitalWrite(Stop_Led, HIGH);
    //    DEBUG.println("Command STOP !!! ");
  }
  // Error
  if (Rush == true & Back == true)
  {
    DEBUG.println();
    //    DEBUG.println("ERROR_Control_Switch");
    DEBUG.println();
    Command = Stop;
    digitalWrite(Backward_Led,  HIGH);
    digitalWrite(Forward_Led, HIGH);
    Alarm_ON();
    //    DEBUG.println("NO MOVE,error");
  }

  //  DEBUG.print("CommandFor XXX --> ");
  DEBUG.println(Command);
  return Command;
}

void Fill_Command_Pack()
{
  Command_Pack[0] = number_of_packet;
  Command_Pack[1] = CommandForRaper;
  if (AlarmFlag == false)
    Command_Pack[2] = 77; // OK code
  else
    Command_Pack[2] = 21;
}
//
void Set_SWITCHER_PIN(bool WireFlag)
{
  DEBUG.print("WireFlag =>");
  DEBUG.println(WireFlag);
  if (WireFlag == true)
  {
    digitalWrite(SWITCHER_PIN, LOW); // inverted COM(4) <-> NC (3) connect Radio_Led
    DEBUG.println("Radio_Led !HIGH");
    digitalWrite(Radio_Led, LOW);   // connect Radio_Led but means WIRE connect
    //    DEBUG.println("Radio_Led HIGH");
    delay(5);
    //    digitalWrite(Green_WireLess_Led, LOW); // does it exist?
  }
  if (WireFlag == false)
  {
    digitalWrite(SWITCHER_PIN, HIGH); // inverted D3A/D6A COM(4) <-> NO (1)
    DEBUG.println("Radio_Led !LOW");
    delay(5);
    digitalWrite(Radio_Led, HIGH);
    //    digitalWrite(Green_WireLess_Led, HIGH); //  does it exist?
  }
  return;
}

void Refresh_WireConnectionFlag_TX()
{
  //  DEBUG.println("Refresh_WireConnectionFlag_TX!");
  bool First_WireConnection_Check = digitalRead(Wire_Connection_Check_Pin);
  if (WireConnectionFlag_TX != First_WireConnection_Check)
  {
    delay(5);
    bool Second_WireConnection_Check = digitalRead(Wire_Connection_Check_Pin);
    if (Second_WireConnection_Check == First_WireConnection_Check)
    {
      if (Second_WireConnection_Check == true) // WIRE ON
      {
        DEBUG.println(F("-|- Change to -> Wire connect"));
        WireConnectionFlag_TX = true;
      }
      else                                      // WIRE OFF
      {
        DEBUG.println(F("-))  ((- Change to -> WireLESS connect"));
        WireConnectionFlag_TX = false;
      }
    }
  }
  return;
}

void Send_Command_Pack(bool CommunicationTypeWireFlag)
{
  if (CommunicationTypeWireFlag == true)
  {
    DEBUG.println("Wire send");  // GO TO Send
    WireSerial.send(Command_Pack, sizeof(Command_Pack) / sizeof(Command_Pack[0]) ); // 3 or sizeof(Command_Pack) / sizeof(Command_Pack[0]) // GO TO Send actual size
    number_of_packet++;
  }
  else // CommunicationTypeWireFlag false
  {
    DEBUG.println("WireLESS send");
    tx.setRemoteAddress(remoteAddress);
    tx.setPayload((uint8_t*)Command_Pack);      //Устанавливаем указатель на тестовый массив
    tx.setPayloadLength(sizeof(Command_Pack));  //Устанавливаем длину поля данных
    mbee.send(tx);
    number_of_packet++;
  }
  DEBUG.print("number_of_packet -> ");
  DEBUG.println(number_of_packet);
  //RESET counter
  if (number_of_packet >= 256)
    number_of_packet = 0;
  return;
}
//
void Check_Recieve_Buffer( bool WireFlag)
{
  //Need Refresh this for PacketSerial if WireFlag == true
  if (WireFlag == true)
  {
    WireSerial.update();
    // mat be go to handler....
    if (WireSerial.overflow())
    {
      // Send an alert via a pin (e.g. make an overflow LED) or return a
      // user-defined packet to the sender.

      //Alarm Check + disarm
      if (AlarmFlag == false)
        Alarm_ON();
      //DEBUG.println("WireSerial Buffer Overflow! need increase buffer!");

      // Ultimately you may need to just increase your recieve buffer via the
      // template parameters (see the README.md).
    }
  }
  else  // WireConnectionFlag_TX == false
  {
    //read input Telemetry from Mbee
    // Telemetry from WireConnect discribed on -> onPacketReceived(const uint8_t* buffer, size_t size)
    mbee.readPacket(); //Постоянно проверяем наличие данных от модема.
    if (mbee.getResponse().isAvailable())
    {
      DEBUG.println("mbee packet is readable");
      Processing_Mbee_Telemetry_Packet();
      DEBUG.println("YEAH!");
    }
    else if (mbee.getResponse().isError())
    {
      DEBUG.println(F("mbee packet isError() == true"));
      DEBUG.print(F("Error №="));
      DEBUG.println(mbee.getResponse().getErrorCode());
      delay(500);
    }
    else
      DEBUG.println("No receive=(");
    DEBUG.println();
  }
  return;
}
//
//
// In this example, we  will only simply print the contents of the array=)
void onPacketReceived(const uint8_t* buffer, size_t size)
{
  if (WireConnectionFlag_TX == false)
  {
    DEBUG.println("NOOOOOOO");
    //    DEBUG.println("This packet is for Mbee. This is not my Job guys *going to relax*");
    return;
  }
  if (AlarmFlag == true)
  {
    Alarm_OFF();
  }

  // Make a temporary buffer.
  uint8_t tempBuffer[size];

  // Copy the packet into our temporary buffer.
  memcpy(tempBuffer, buffer, size);

  // Fill Telemetry_Pack_From_RXtempBuffer
  Fill_Telemetry_Pack_From_RX(tempBuffer, size);
  // Print our temporaray buffer. //debug feature
  Print_Telemetry_Packet(Telemetry_Pack_From_RX, size);

  //Processing Data
  Calculate_Data_From_Telemetry_Pack();
  //Printing_Values_On_A_PC_Monitor();  //debug feature for Processing program (not actual on 30.11.2019)


  ReadyToPrint_DataFromRX = true;
  DEBUG.print("ReadyToPrint_DataFromRX ");
  DEBUG.println(ReadyToPrint_DataFromRX);
  //

  // Send the reversed buffer back to the sender. The send() method will encode
  // the whole buffer as as single packet, set packet markers, etc.
  // The `tempBuffer` is a pointer to the `tempBuffer` array and `size` is the
  // number of bytes to send in the `tempBuffer`.
  //myPacketSerial.send(tempBuffer, size);
  return;
}

void Fill_Telemetry_Pack_From_RX(const uint8_t* buffer, size_t size)
{
  DEBUG.println("Fill the Telemetry_Pack");
  if (size == 13 )
  {
    DEBUG.println("pack fill from WIRE RX");
    for (int i = 0 ; i < size ; i++)
    {
      Telemetry_Pack_From_RX[i] = buffer[i];
    }
  }
  return;
}
//
void Print_Telemetry_Packet(uint8_t* buffer, size_t size)
{
  DEBUG.println("Telemtry message:");
  for (int i = 0 ; i < size ; i++)
  {
    DEBUG.print("[");
    DEBUG.print(i);
    DEBUG.print("]=");
    DEBUG.print(buffer[i]);
    DEBUG.print("; ");
  }
//  delay(5);
  return;
}

void Processing_Mbee_Telemetry_Packet()
{
  // Print API ID
  //  DEBUG.println("TX Packet from Rx READED, NICE!");
  DEBUG.print("API ID === ");
  DEBUG.println( mbee.getResponse().getApiId() );

  // Print RSSI
  DEBUG.print("RSSI -> ");
  DEBUG.println(rx.getRssi());

  //    Debug_information_About_Rx_Packet(); // debug info

  // Fill Telemetry_Pack_From_RX
  if (mbee.getResponse().getApiId() == RECEIVE_PACKET_API_FRAME || mbee.getResponse().getApiId() == RECEIVE_PACKET_NO_OPTIONS_API_FRAME)
  {
    // Alarm check
    if (AlarmFlag == true) Alarm_OFF();

    // Processing
    mbee.getResponse().getRxResponse(rx); //Получаем пакет с данными. //вписываем данные в "rx"
    if (rx.getDataLength() > 1)
    {
      int j = 0;
      //      DEBUG.println("TX Packet have many Symbols");
      // 0 - 7 system info , 8 - X important info
      for (int i = 0 ; i < rx.getDataLength() ; i++ )
      {
        //print all Array
        //        if ( i >= 0)
        //        {
        //          DEBUG.print("Value [");
        //          DEBUG.print(i);
        //          DEBUG.print("] = ");
        //          DEBUG.println(rx.getData()[i]);
        //        }

        //Fill Telemetry Pack
        if ( i >= 8 )
        {
          Telemetry_Pack_From_RX[j] = rx.getData()[i];
          j++;
        }
      }

      Calculate_Data_From_Telemetry_Pack();
      Printing_Values_On_A_PC_Monitor();
      ReadyToPrint_DataFromRX = true;
      DEBUG.print("ReadyToPrint_DataFromRX ");
      DEBUG.println(ReadyToPrint_DataFromRX);
      delay(10);
    }
    else
    {
      DEBUG.println("1 symbols from RX");
    }

  }
  else
    DEBUG.println("Unexpected API ID from RX");

  return;
}

void Alarm_ON()
{
  DEBUG.println("Alarm ON");
  digitalWrite(BUZZER_Pin, HIGH);
  AlarmFlag = true;
  // GOTO
  // Watch Dog Timer on?
  return;
}

void Alarm_OFF()
{
  // digitalWrite(BUZZER_Pin, HIGH);
  DEBUG.println("Alarm OFF");
  digitalWrite(BUZZER_Pin, LOW);
  AlarmFlag = false;
  // GOTO
  // Watch Dog Timer OFF?
  return;
}

void Calculate_Data_From_Telemetry_Pack()
{
  DEBUG.println("Calculate_Data_From_Telemetry_Pack()...");
  int temp = 0;
  //1-2
  Speed_1_raw = Telemetry_Pack_From_RX[1] + (Telemetry_Pack_From_RX[2] << 8);
  Speed_1_RPM = (540 - Speed_1_raw) * ADC_Speed_RPM_Step; //because invertor after ADC
  //Debug Feature
  //  Speed_1_raw = map(Speed_1_raw, 0, 1023, -100, 100);

  //3-4
  Speed_2_raw = Telemetry_Pack_From_RX[3] + (Telemetry_Pack_From_RX[4] << 8);
  Speed_2_RPM = (540 - Speed_2_raw) * ADC_Speed_RPM_Step;
  //Debug Feature
  //  Speed_2_raw = map(Speed_2_raw, 0, 1023, -100, 100);

  //5-6
  I_1_raw = Telemetry_Pack_From_RX[5] + (Telemetry_Pack_From_RX[6] << 8);
  I_1_A = (550 - I_1_raw) * ADC_Current_Step;
  //Debug Feature
  //  I_1_raw = map(I_1_raw, 0, 1023, -100, 100);

  //7-8
  I_2_raw = Telemetry_Pack_From_RX[7] + (Telemetry_Pack_From_RX[8] << 8);
  I_2_A = I_2_raw * ADC_Current_Step;
  //Debug Feature
  //  I_2_raw = map(I_2_raw, 0, 1023, -100, 100);

  //9-10 Get Roper_Voltage
  V_Roper_raw = Telemetry_Pack_From_RX[9] + (Telemetry_Pack_From_RX[10] << 8); //0-1023 from ADC
  Battery_Roper_Voltage = V_Roper_raw * ADC_Climber_Voltage_Step;              //0-6 V
  //Debug Feature
  //  V_Roper_raw = map(V_Roper_raw, 0, 1023, 0, 100);

  //11
  //  PWM_Value_Percent = Telemetry_Pack_From_RX[11] * ADC_PWM_Step;
  PWM_Value_raw = Telemetry_Pack_From_RX[11];

  //12
  Roper_Direction = Telemetry_Pack_From_RX[12];  // 0-S;2-B;5-F
  //Get RSSI 12+1
  if ( WireConnectionFlag_TX == false)
    RSSI_LvL = rx.getRssi();
  else
    RSSI_LvL = 0;
  //Get myself RC Voltage
  //V_RC_raw - 0 -1023
  Battery_RC_Voltage = V_RC_raw * ADC_RC_Voltage_Step; // 0-24 V

  return;
}


void Printing_Values_On_A_PC_Monitor()
{
  DEBUG.print("Speed_1=");
  DEBUG.print(Speed_1_RPM);
  DEBUG.print("~Speed_2=");
  DEBUG.print(Speed_2_RPM);
  DEBUG.print("~I_1=");
  DEBUG.print(I_1_A);  //I_1_A
  DEBUG.print("~I_2=");
  DEBUG.print(I_2_A);  //I_2_A
  DEBUG.print("~V_Roper=");
  DEBUG.print(Battery_Roper_Voltage);
  DEBUG.print("~PWM_Value=");
  DEBUG.print(PWM_Value_Percent);
  DEBUG.print("~Direction=");
  DEBUG.print(Roper_Direction);
  // GO TO  //mb REWORK this V_Control_Panel !!!!
  DEBUG.print("~Battery_RC_Voltage=");  // x.y volt  5.2 V
  DEBUG.print( Battery_RC_Voltage );
  DEBUG.print("~RSSI=");
  DEBUG.print(rx.getRssi());
  // End Marker
  DEBUG.println("~ #");
  DEBUG.println();

  delay(30);
  return;
}





//Display Functions
//OLED_Set_Start_Settings
void OLED_Display_Const_Data()
{
  //Zero line
  //  disp.setCursor(0, 0);
  //  disp.print("TEST");

  //Second line
  disp.setCursor(0, 1);
  disp.print("SPD ");
  disp.setCursor(9, 1);
  disp.print("%");
  disp.setCursor(16, 1);
  disp.print("%");

  //Third line
  disp.setCursor(0, 2);
  disp.print("LOAD");
  disp.setCursor(9 , 2);
  disp.print("%");
  disp.setCursor(16 , 2);
  disp.print("%");

  //Fourth line
  disp.setCursor(0, 3);
  disp.print("BAT");
  disp.setCursor(5, 3);
  disp.write(byte(2));  //symbol Climber
  disp.setCursor(9, 3);
  disp.print("%");
  disp.setCursor(12, 3);
  disp.write(byte(3));  //symbol RC
  disp.setCursor(16, 3);
  disp.print("%");
  return;
}

void OLED_Display_Dynamic_Data()
{
  uint8_t RSSI_LvL_for_Display = 0;
  //int input, int pos1, int pos2
  //Second line
  //  int temp = random(0, 9);

  // DEBUG
  //  DEBUG.print("TX_Telemetry_Pack_From_RX[0]= ");
  //  DEBUG.println(Telemetry_Pack_From_RX[0]);

  //  disp.setCursor(2, 0);
  //  disp.print(RSSI_LvL);

  //  buffer1 = Compiler_for_OLED(RSSI_LvL); // 5
  //  DEBUG.println("Buffer: ");
  //  DEBUG.println(buffer1[0]);
  //  disp.setCursor(8, 0);
  //  disp.print(buffer1);

  //  Print_to_OLED(Speed_1_raw, 5, 1);
  //  Print_to_OLED(Speed_2_raw, 12, 1);
  //  Print_to_OLED = (I_1_raw, 5, 2); // 5
  //  Print_to_OLED = (I_2_raw, 12, 2);

  //

  //GGOOOOOOOOO
  Print_to_OLED(random(-100, 100), 5, 1);
  Print_to_OLED(random(-100, 100), 12, 1);
  Print_to_OLED(random(-100, 100), 5, 2); // 5
  Print_to_OLED(random(-100, 100), 12, 2);
  //
  Print_Battery_LvL_for_OLED(random(0, 100), 6, 3);
  Print_Battery_LvL_for_OLED(random(0, 100), 13, 3);
  //  Print_Battery_LvL_for_OLED(V_Roper_raw, 6, 3);    // 9
  //  Print_Battery_LvL_for_OLED(V_RC_raw, 13, 3);      // self

  RSSI_LvL_for_Display = RSSI_Processing();
  if (WireConnectionFlag_TX == false)
    RSSI_plot(RSSI_LvL_for_Display);
  else
    Wire_Symbol_plot();

  //  // DEBUG
  //  buffer1 = Compiler_for_OLED (Speed_1_raw);  // 1
  //  //  buffer2 = Compiler_for_OLED (Speed_2_raw);  // 3
  //  buffer2 = Compiler_for_OLED (Telemetry_Pack_From_RX[0]);
  //  disp.setCursor(5, 1);
  //  disp.print(buffer1);
  //  disp.setCursor(12, 1);
  //  //  disp.print(buffer2);
  //  disp.print(Telemetry_Pack_From_RX[0]);
  //
  //  delay(5);
  //  //Third line // goto
  //  //  buffer1 = Compiler_for_OLED (I_1_raw);
  //
  //  buffer1 = Compiler_for_OLED (I_1_raw); // 5
  //  buffer2 = Compiler_for_OLED (I_2_raw); // 7
  //  //  buffer2 = Compiler_for_OLED (number_of_packet);  // counter RX _ RC
  //  disp.setCursor(5, 2);
  //  disp.print(buffer1);
  //  disp.setCursor(12, 2);
  //  //  disp.print(buffer2);
  //  disp.print(number_of_packet);
  //
  //  //Fourth line // goto
  //  //  buffer1 = Compiler_Battery_LvL_for_OLED(V_Roper_raw);
  //  disp.setCursor(6, 3);
  //  disp.print(buffer1);
  //  disp.setCursor(13, 3);
  //  disp.print(buffer2);
  //  RSSI_LvL_for_Display = RSSI_Processing();
  //  if (WireConnectionFlag_TX == false)
  //    RSSI_plot(RSSI_LvL_for_Display);
  //  else
  //    Wire_Symbol_plot();

  //  delay(100);
  return;
}

void Print_Battery_LvL_for_OLED(int intoviy, int pos1, int pos2)
{
  char source [5] = { '-', '2', '3', '4', '\n'};
  if (intoviy >= 110 || intoviy < 0)
  {
    return;
  }
  disp.setCursor(pos1, pos2);
  if (intoviy >= 0 & intoviy <= 999 )
  {
    if (intoviy >= 0 & intoviy < 10)
      disp.print("   ");
    if (intoviy >= 10 & intoviy < 100)
      disp.print("  ");
    if (intoviy >= 100 & intoviy < 999)
      disp.print(" ");
  }
  if ( intoviy >= 0 & intoviy <= 110)
  {
    itoa (intoviy, source, 10);
    DEBUG.print("itoa(src) |");
    DEBUG.println(source);
  }
  disp.print(source);

  //
  //  if (input >= 0)
  //  {
  //    small = String(input);
  //    if (input >= 0 & input < 10)
  //      small = String("  " + String(input));
  //    if (input >= 10 & input < 100)
  //      small = String(" " + String(input));
  //    if (input >= 100)
  //      small = String(input);
  //    delay(1);
  //    DEBUG.println("small: ");
  //    DEBUG.println(small);
  //    return small;
  //  }
  //  else
  //  {
  //    String err = "ERR";
  //    return err;
  //  }
}

void Print_to_OLED(int intoviy, int pos1, int pos2)
{
  // compile the buffer
  char source [5] = { '-', '2', '3', '4', '\n'};
  if (intoviy > 120 || intoviy < -120)
  {
    DEBUG.print("outrange");
    return;
  }
  //setCursor
  disp.setCursor(pos1, pos2);
  //print __+ val
  if (intoviy < 0 & intoviy >= -999 )
  {
    if (intoviy < 0 & intoviy > -10)
      disp.print("__");
    if (intoviy <= -10 & intoviy > -100)
      disp.print("_");
  }
  if (intoviy >= 0 & intoviy <= 999 )
  {
    if (intoviy >= 0 & intoviy < 10)
      disp.print("___");
    if (intoviy >= 10 & intoviy < 100)
      disp.print("__");
    if (intoviy >= 100 & intoviy < 999)
      disp.print("_");
  }
  if ( intoviy >= -120 & intoviy <= 120)
  {
    itoa (intoviy, source, 10);
    DEBUG.print("itoa(src) |");
    DEBUG.println(source);
  }
  disp.print(source);
  DEBUG.print("suffer ");
  DEBUG.println(intoviy);
  DEBUG.println();
  return;
}

//
uint8_t RSSI_Processing()
{
  uint8_t LvL = 0;
  if (RSSI_LvL >= 0 & RSSI_LvL < 32)
    LvL = 1;
  if (RSSI_LvL >= 32 & RSSI_LvL < 64)
    LvL = 2;
  if (RSSI_LvL >= 64 & RSSI_LvL < 96)
    LvL = 3;
  if (RSSI_LvL >= 96 & RSSI_LvL < 128)
    LvL = 4;
  if (RSSI_LvL >= 128 & RSSI_LvL < 160)
    LvL = 5;
  if (RSSI_LvL >= 160 & RSSI_LvL < 192)
    LvL = 6;
  if (RSSI_LvL >= 192 & RSSI_LvL < 224)
    LvL = 7;
  if (RSSI_LvL >= 224 & RSSI_LvL < 255)
    LvL = 8;
  return LvL;
}

// Plot
void RSSI_plot(int LvL)
{
  if (LvL == 1 )
  {
    disp.setCursor(16, 0);
    disp.write(byte(4));
    disp.print("   ");
    // disp.setCursor(16, 0);
    // disp.blink();
  }
  if (LvL == 2 )
  {
    disp.setCursor(16, 0);
    disp.write(byte(4));
    disp.print("   ");
  }
  if (LvL == 3 )
  {
    disp.setCursor(16, 0);
    disp.write(byte(4));
    disp.write(byte(5));
    disp.print("  ");
    disp.setCursor(17, 0);
    //    disp.blink();
  }
  if (LvL == 4 )
  {
    disp.setCursor(16, 0);
    disp.write(byte(4));
    disp.write(byte(5));
    disp.print("  ");
  }
  if (LvL == 5 )
  {
    disp.setCursor(16, 0);
    disp.write(byte(4));
    disp.write(byte(5));
    disp.write(byte(6));
    disp.print(" ");
    disp.setCursor(18, 0);
    //    disp.blink();
  }
  if (LvL == 6 )
  {
    disp.setCursor(16, 0);
    disp.write(byte(4));
    disp.write(byte(5));
    disp.write(byte(6));
    disp.print(" ");
  }
  if (LvL == 7 )
  {
    disp.setCursor(16, 0);
    disp.write(byte(4));
    disp.write(byte(5));
    disp.write(byte(6));
    disp.write(byte(7));
    disp.setCursor(19, 0);
    //    disp.blink();
  }
  if (LvL == 8 )
  {
    disp.setCursor(16, 0);
    disp.write(byte(4));
    disp.write(byte(5));
    disp.write(byte(6));
    disp.write(byte(7));
  }
  return;
}

void Wire_Symbol_plot()
{
  //GOTO maybe plot special symbols? like "-==-"
  disp.setCursor(16, 0);
  disp.print("WIRE");
  return;
}

void reset_textmode() //функция для установки графического режима
{
  disp.command(0x08);//выключили экран
  disp.command(0x17);//переключение в текстовый режим
  disp.command(0x01);//очистили от мусора ОЗУ
  disp.command(0x04 | 0x08);//включили экран
  return;
}

//
