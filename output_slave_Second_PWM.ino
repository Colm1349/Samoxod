/* ЗАДАЧА
  //new_git_life+2
    ПРИЁМНИК

    //НАЧАЛО РАБОТЫ

    Собираем информацию о НАПРЯЖЕНИИ и зажигаем диод в зависимости от значения
    ~(12 - 15 Вольт == ЗЕЛЕНЫЙ)
    ~(8 - 12  Вольт == ЖЕЛТЫЙ)
    ~(8 и меньше == КРАСНЫЙ)

    Взводим таймер на 1 секунду и разрешаем прерывания по таймеру

    //ОСНОВНАЯ ЗАДАЧА

    Ловим данные от пульта (скорее всего без пакетной передачи) и сохраняем в переменную.
    Выключаем таймер.
    Смотрим значение переменной и даём команду на мотор/двигателя (контроллер мотора если точнее)
    И зажигаем один из диодов вперёд/назад
    //(Зел - вперёд / Красный - назад   -- тестовый вариант)
    //для индикации будем по началу использовать два диода - ЗЕЛЕНЫЙ и КРАСНЫЙ (pin5 и pin8)

    //Если данных нет более секунды начинаем кричать пищалкой (вызов функции ALERT() )
    //и ловим дальше команду

    Если удалось дать команду то идём дальше в сбор телеметрии

    // СБОР ТЕЛЕМЕТРИИ

    Забираем значения с датчиков ТОКА, ТЕМПЕРАТУРЫ.
    Сохраняем в переменные

    //ОТПРАВКА ДАННЫХ НА ПУЛЬТ

    Данные с датчиков собираем в пакет и отправляем на пульт
    Dозвращаемся к началу цикла loop() [ НАЧАЛО РАБОТЫ - сбор данных о напряжении]

*/

// скетч для Arduino, который принимает данные с пульта и даёт команду двигателям, собирает телеметрию и отправляет её на пульт

#define Stop 0
#define Forward 5
#define Backward -5
#define Max_SpeedValue 255
#define Min_SpeedValue -255
#define ErrorMove 10
#define SystemLed 13
//#define

#include <avr/wdt.h>

// main_value
int incomingByte = 0;
int Actual_Command = 0; // -5 / 0 / 5
int Ordered_Command = 0;
int Receive_Error_Counter = 0;
int cntr = 0;
int PWM_Value = 0;
int SpeedValue_Now = 0; // 0 -== stop [-255  - 0 - 255]
//int Min_SpeedValue = -255;
//int Max_SpeedValue = 255;
int Step_For_Move = 0;
int Wished_Speed = 0;
// boolean Flags
boolean lastButton = LOW;
boolean currentButton = LOW;
//BIND PINS NAMES
int Permission_To_Move = 2;    // выставляется после направления движения
int Direction_Of_Movement = 3; //выставляется раньше разрешения
int PWM_Pin = 11;
int systemLed = 13;
int BackwardLed = 5; // debug
int ForwardLed = 6;  // debug
int BUZZER_PIN = 10;
int ForwardButton = 8;
//int switchPin = 8;
int BackwardButton = 9;
//Flags
bool AlarmTrigger = false;
bool WDT_ACTIVE = false;

//Обработка прерывания по переполнению счётчика. Должны пищалкой пищать
ISR(TIMER2_OVF_vect) {
  cli();
  TCNT2 = 0; //55; Костыльное решение для изменения скорости моргания. ЛОЛ
  cntr++;
  if (cntr > 1000)
  {
    // ALARM
    Alarm_ON();
    if (WDT_ACTIVE == false)
    {
      //      wdt_enable(WDTO_4S); // WDT ENABLE!
      WDT_ACTIVE = true;
    }
    cntr = 0;
    digitalWrite(Direction_Of_Movement  , LOW);
    digitalWrite(Permission_To_Move, LOW);
    Actual_Command = 1; // FAIL MARKER
  }
  sei();
}

void setup()
{
  Serial.begin(9600);
  pinMode(Permission_To_Move, OUTPUT);
  pinMode(Direction_Of_Movement, OUTPUT);
  pinMode(systemLed, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(ForwardButton, INPUT); // debug
  pinMode(BackwardLed, OUTPUT);
  pinMode(ForwardLed, OUTPUT);

  //STOP на двигатели в самом начале. На всякий случай!!!
  // Stop_Command or Command_to_Motor(0);
  digitalWrite(Direction_Of_Movement , LOW);
  digitalWrite(Permission_To_Move, LOW);
  analogWrite(PWM_Pin, PWM_Value); // (11, 0)

  //Настройка таймера 2
  TCCR2A = 0;     // 0b00000000 -> Normal режим (никаких ШИМ) / Не ставили условий для ноги OC0A(12)
  TCNT2 = 0;      // 0b00111011 -> Это просто регистр со значениями куда всё тикает. зачем там 59?
  TIMSK2 = 0b00000001;  //0b00000001 - TOIE - до переполнения // ИЛИ -> TIMSK2 = 1; ИЛИ -> TIMSK2 |= (1 << TOIE2);
//  TCCR2B = 0b00000011;  // 0b00000010 -> clk/8 (CS02 / CS01 / CS00)
  TCCR2B = 0b00000000; // STOp Timer2
}

void loop()
{
  //Debug Method
  InputButtons();
  Command_To_Motor(Ordered_Command);
  Send_Telemetry (Actual_Command);
}

//СОБЫТИЕ ПРЕРЫВАНИЯ И ДЕЙСТВИЯ ПОСЛЕ ПРИЕМА БАЙТА
//приём команды от пульта по прерыванию на UART
void serialEvent()
{
  cli(); // запрет прерываний пока принимаем данные
  while (Serial.available())
  {
    incomingByte = (int) Serial.read();
  }
  sei(); // разрешаем прерывания
  //Обработка принятого
  //По идее тут должна быть функция разбора CREEPY пакета с Mbee
  //Выясним направление с пульта
  //Команда без смысла
  Ordered_Command = 1;
  if (incomingByte >= 140 & incomingByte <= 255)
  {
    Ordered_Command = 5; // FORWARD
  }
  if (incomingByte >= 90 & incomingByte < 140)
  {
    Ordered_Command = -5; // BACKWARD
  }
  if (incomingByte >= 0 & incomingByte < 90)
  {
    Ordered_Command = 0; // STOP
  }
  if (Ordered_Command != -5 & Ordered_Command != 0 & Ordered_Command != 5)
  {
    Receive_Error_Counter = Receive_Error_Counter + 1;
    if (Receive_Error_Counter >= 50 )
    {
      cli();
      //Emergency Stop!!!
      digitalWrite(Permission_To_Move, LOW);
      digitalWrite(Direction_Of_Movement, LOW);
      digitalWrite(systemLed, LOW);
      //Debug
      digitalWrite(BackwardLed, LOW);
      digitalWrite(ForwardLed, LOW);
      Alarm_ON();
      if (WDT_ACTIVE == false)
      {
        //        wdt_enable(WDTO_1S); // WDT ENABLE!
        WDT_ACTIVE = true;
      }
      sei();
    }
  }
  else
  {
    Reset_Error_Timer_And_Check_WDT();
  }
}

void Command_To_Motor (int instruction)
{
  if (instruction == ErrorMove)
  {
    Serial.println("Wrong Move command.");
    //Stop motors GO TO ZERO PWM
    analogWrite(PWM_Pin, PWM_Value);
    //Debug
    digitalWrite(ForwardLed, LOW);
    digitalWrite(BackwardLed, LOW);
    digitalWrite(systemLed, HIGH);
    delay(100);
    return;
  }
  if (instruction == Stop)
  {
    Wished_Speed = Stop;
    if (SpeedValue_Now == Stop)
    {
      Serial.println("We stopped");
      Step_For_Move = 0;
    }
    if (SpeedValue_Now != Wished_Speed)
    {
      //Need Change Step
      if (SpeedValue_Now > 0)
      {
        Step_For_Move = -1; // pull to zero
      }
      if (SpeedValue_Now < 0)
      {
        Step_For_Move = 1; // pull to zero
      }
      SpeedValue_Now = SpeedValue_Now + Step_For_Move;
      if (SpeedValue_Now == Wished_Speed) // && Actual_Command == STOP
      {
        Step_For_Move = 0;  // command complete
        Serial.println("STOP NOW");
      }
      else
      {
        // ??????
      }
    }
    Serial.print("(Stopping)Speed == ");
    Serial.println(SpeedValue_Now);
    Serial.println("---------------");
  }

  if (instruction == Forward)
  {
    Wished_Speed = Max_SpeedValue;
    if (instruction == Actual_Command)
    {
      // do nothing
    }
    if (Wished_Speed == SpeedValue_Now == Max_SpeedValue)
    {
      Serial.println("Max Speed is reached!");
      Step_For_Move = 0;
    }
    if (SpeedValue_Now >= Min_SpeedValue && SpeedValue_Now < Max_SpeedValue)
    {
      Step_For_Move = 1; // pull to 255 (Max_SpeedValue)
    }
    SpeedValue_Now = SpeedValue_Now + Step_For_Move;
    if (SpeedValue_Now == Max_SpeedValue)
    {
      Step_For_Move = 0;  // command complete
      Serial.println("Max Speed RIGHT NOW! LvL UP! ");
    }
    Serial.print("(Grow) Speed == ");
    Serial.println(SpeedValue_Now);
    Serial.println("---------------");
  }

  if (instruction == Backward)
  {
    Wished_Speed = Min_SpeedValue;
    if (instruction == Actual_Command)
    {
      // do nothing
    }
    if (Wished_Speed == SpeedValue_Now == Min_SpeedValue)
    {
      Serial.println("Min Speed is reached!");
      Step_For_Move = 0; // pull to 255 (Max_SpeedValue)
    }
    if (SpeedValue_Now > Min_SpeedValue && SpeedValue_Now <= Max_SpeedValue)
    {
      Step_For_Move = -1;
    }
    SpeedValue_Now = SpeedValue_Now + Step_For_Move;
    if (SpeedValue_Now == Min_SpeedValue) // && Actual_Command == STOp
    {
      Step_For_Move = 0;  // command complete
      Serial.println("We Reversed");
    }
    // Command Release
    Serial.print("(Drop) Speed == ");
    Serial.println(SpeedValue_Now);
    Serial.println("---------------");
  }
  // Command Release
  Execute_The_Command(SpeedValue_Now);
  //        Alarm_ON();
  //      if (WDT_ACTIVE == false)
  //      {
  //        wdt_enable(WDTO_1S); // WDT ENABLE!
  //        WDT_ACTIVE = true;
  //      }
  //}
}

void Send_Telemetry (int instruction)
{
  if (instruction == 5)
  {
    Serial.print("F"); //FORWARD
  }
  if (instruction == -5)
  {
    Serial.print("B"); //BACKWARD
  }
  if (instruction == 0)
  {
    Serial.print("S"); //STOP
  }
}

void Execute_The_Command(int Speed)
{
  if (Speed >= -255 & Speed <= 255)
  {
    if (Speed > 0) //F
    {
      // Command Release
      digitalWrite(Direction_Of_Movement, HIGH);
      digitalWrite(Permission_To_Move, HIGH);
      analogWrite(PWM_Pin, PWM_Value);
      //Debug
      digitalWrite(ForwardLed, HIGH);
      digitalWrite(BackwardLed, LOW);
      digitalWrite(systemLed, LOW);
    }
    if (Speed < 0) //B
    {
      // Command Release
      digitalWrite(Direction_Of_Movement, LOW);
      digitalWrite(Permission_To_Move, HIGH);
      analogWrite(PWM_Pin, PWM_Value);
      //Debug
      digitalWrite(ForwardLed, LOW);
      digitalWrite(BackwardLed, HIGH);
      digitalWrite(systemLed, LOW);
    }
    if (Speed == 0)
    {
      // Command Release
      digitalWrite(Direction_Of_Movement, LOW);
      digitalWrite(Permission_To_Move, LOW);
      analogWrite(PWM_Pin, PWM_Value);
      //Debug
      digitalWrite(ForwardLed, LOW);
      digitalWrite(BackwardLed, LOW);
      digitalWrite(systemLed, LOW);
    }
  }
  else
  {
    // STOP RIGHT NOW
    // Command Release
    digitalWrite(Direction_Of_Movement, LOW);
    digitalWrite(Permission_To_Move, LOW);
    analogWrite(PWM_Pin, PWM_Value);
    //Debug
    Serial.print("Error!!! Speed == ");
    Serial.println(SpeedValue_Now);
    Serial.println("---------------");
    SpeedValue_Now = 0;
    Serial.print("Speed Value_Now = ");
    Serial.println(SpeedValue_Now);
    Serial.println("---------------");
    digitalWrite(ForwardLed, HIGH);
    digitalWrite(BackwardLed, HIGH);
    digitalWrite(systemLed, HIGH);
  }
}

void InputButtons()
{
  //  currentButton = debounce(lastButton);
  //  if (lastButton == LOW && currentButton == HIGH)
  //  {
  //    //    ledLevel = ledLevel + Step;
  //    Serial.print("PWMvalue == ");
  //    Serial.println(PWMvalue);
  //  }
  //  if ()
  //  lastButton = currentButton;

  //Ordered_Command

  bool ForwardCommand = 0;
  bool BackwardCommand = 0;
  if (digitalRead(ForwardButton) == true)
  {
    delay(10);
    if (digitalRead(ForwardButton) == true)
      ForwardCommand = true;
  }
  if (digitalRead(BackwardButton) == true)
  {
    delay(10);
    if (digitalRead(BackwardButton) == true)
      BackwardCommand = true;
  }

  if ( ForwardCommand == true & BackwardCommand == false)
    Ordered_Command = Forward;  // 5
  if ( ForwardCommand == false & BackwardCommand == true)
    Ordered_Command = Backward; // -5
  if ( ForwardCommand == false & BackwardCommand == false)
    Ordered_Command = Stop;  // 0
  if ( ForwardCommand == true & BackwardCommand == true)
    Ordered_Command = ErrorMove; // 10
  //
}

boolean debounce(boolean last)
{
  boolean current = digitalRead(ForwardButton);
  if (last != current)
  {
    delay(5);
    Serial.println("last != current");
    current = digitalRead(ForwardButton);
  }
  return current;
}

void Reset_Error_Timer_And_Check_WDT()
{
  cli();
  if (AlarmTrigger == true)
  {
    Alarm_OFF();
    if (WDT_ACTIVE = true)
    {
      WDT_ACTIVE = false;
      wdt_disable();           //TURN OFF WDT!
    }
  }
  TCCR2B = 0b00000000;     // 0b00000000 -> STOP_TIMER (CS02 = CS01 = CS00 = 0)
  TCNT2 = 0;  // обнуляем регистр счёта
  TCCR2B = 0b00000011;     // 0b00000010 -> clk/8 (CS02 = 0 / CS01 = 1 / CS00 = 0)
  cntr = 0;
  sei();
}

void Alarm_ON()
{
  AlarmTrigger = true;
  digitalWrite(BUZZER_PIN, HIGH);
}

void Alarm_OFF()
{
  AlarmTrigger = false;
  digitalWrite(BUZZER_PIN, LOW);
}
