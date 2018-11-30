/* Задача.
  //new_git_life+1
    ПУЛЬТ.

    //ОСНОВНАЯ ЗАДАЧА

    Ловим данные от датчика/руля/переключателя и сохраняем в переменную (например ИНТ) и зажигаем СООТВЕТСТВУЮЩИЙ ДИОД
    Далее передаём команду другой плате (плате-приёмнику)

    //Забор Собственных данных

    Пока передаём значения , прерывания выключены.
    Забираем данные с датчика напряжения пульта и записываем в переменную.

    //Приём телеметрии

    После этого мы включаем прерывания и взводим таймер на 100 мс.
    В течении этого времени мы ловим пакет от второй платы.
    Во время приёма короткого пакета (около 40 байт) мы выключаем прерывания и таймер.
   //если за это время пакет не пришел, то мы снова посылаем сигнал.
   //+1 к счётчику мертвой телеметрии. Если счетчик больше 100 ставим флаг TELEMETRY_IS_DEAD = TRUE
    Разбираем пакет, вычленяем нужные нам в массиве байты СКОРОСТИ/ТОКА1/ТОКА2/НАПРЯЖЕНИЯ/ТЕМПЕРАТУРЫ
    Записываем их в нужные нам переменные.
    Разбор пакета завершен.
   //флаг TELEMETRY_IS_DEAD = FALSE .

   //Вывод данных на дисплей

    Выводим эти данные на экранчик/монитор/дисплей/другая индикация. + данные от заряда батареи у самого пульта.
   //ЕСЛИ флаг TELEMETRY_IS_DEAD = TRUE - На дисплей надо вывести значок/надпись что телеметрия сдохла
    После вывода на экран возвращаемся к началу цикла loop()   [Ловим данные от переключателя]


*/

// скетч для Arduino пульта (Control Panel), который посылает сигнал на плату-управления, обрабатывает телеметрию, выводит информацию телеметрии на дисплей
//зажигаем когда обрабатываем пакет телеметрии

#include <avr/wdt.h>

int led = LED_BUILTIN;  //диод на плате
int input_sensor = 0;
String TelemetryStringArr;
int ErrorCounter_Combo = 0;
int Good_Receive_Combo = 0;
int cntr = 0;

int FORWARD_LED = 11;   //DEBUG
int BACKWARD_LED = 12;  //DEBUG
int BUZZER_ALARM = 4;   //DEBUG
int BackwardPin = 3;
int ForwardPin = 2;
int Self_Led_Forward = 8;
int Self_Led_Backward = 7;
int Self_Output_Command = 0;
bool AlarmTrigger = false;
bool WDT_ACTIVE = false;

//Обработка прерывания по переполнению счётчика. BUZZER_ON if cntr > X
ISR(TIMER2_OVF_vect) // Если мы даём сигнал о тревоге то нужно запускать таймер для ребута! => в функции таймера сбрасываем
{
  cli();
  TCNT2 = 0; //55; Костыльное решение для изменения скорости моргания. ЛОЛ
  cntr++;
  if (cntr > 1000)
  {
    START_Alarm();
    if (WDT_ACTIVE == false)
    {
      wdt_enable(WDTO_2S); //Start WatchDog Timer
      WDT_ACTIVE = true;
    }
    cntr = 0;
    digitalWrite(led, LOW);
    digitalWrite(BACKWARD_LED, LOW);
    digitalWrite(FORWARD_LED, LOW);
  }
  sei();
}

void setup()
{
  Serial.begin(9600);
  pinMode(input_sensor, INPUT);
  pinMode(led, OUTPUT);
  pinMode(FORWARD_LED, OUTPUT);
  pinMode(BACKWARD_LED, OUTPUT);
  pinMode(BUZZER_ALARM, OUTPUT);
  pinMode(Self_Led_Forward, OUTPUT);
  pinMode(Self_Led_Backward, OUTPUT);
  pinMode(ForwardPin, INPUT);
  pinMode(BackwardPin, INPUT);

  //Индикация OFF на начале работы
  digitalWrite(led, LOW);
  digitalWrite(FORWARD_LED, LOW);
  digitalWrite(BACKWARD_LED, LOW);
  digitalWrite(Self_Led_Forward, HIGH);
  digitalWrite(Self_Led_Backward, HIGH);
  //Настройка таймера 2 (T2)
  TCCR2A = 0;     // 0b00000000 -> Normal режим (никаких ШИМ) / Не ставили условий для ноги OC0A(12)
  TCNT2 = 0;      // 0b00111011 -> Это просто регистр со значениями куда всё тикает. зачем там 59?
  TIMSK2 = 0b00000001;  // 0b00000001 - TOIE - до переполнения // ИЛИ -> TIMSK2 = 1; ИЛИ -> TIMSK2 |= (1 << TOIE2);
  //START TIMER
  TCCR2B = 0b00000011;  // 0b00000010 -> clk/8 (CS02 / CS01 / CS00)
}

void loop()
{
  cli(); //передать данные важнее чем принять телеметрию и вывести данные на дисплей
  // Read_Data
  //  int RAW_value = analogRead(input_sensor);
  //  int reborn = map(RAW_value, 0, 1023, 0, 255); //приводим к значению от 0 до 255
  bool CommandForward = digitalRead(ForwardPin);
  bool CommandBackward = digitalRead(BackwardPin);
  int  Command = 0;

  if (CommandForward == true)
  {
    digitalWrite(Self_Led_Forward, HIGH);
    digitalWrite(Self_Led_Backward, LOW);
    Self_Output_Command = 5;
    Command = 200;    //FORWARD 
  }
  if(CommandBackward == true)
  {
    digitalWrite(Self_Led_Backward, HIGH);
    digitalWrite(Self_Led_Forward, LOW);
    Self_Output_Command = -5; //Backward
    Command = 100;
  }
  if (CommandForward == false & CommandBackward == false)
  {
    digitalWrite(Self_Led_Forward, LOW);
    digitalWrite(Self_Led_Backward, LOW);
    Self_Output_Command = 0;
    Command = 0;    //STOP 
  }
  if (CommandForward == true & CommandBackward == true)
  {
    digitalWrite(Self_Led_Forward, HIGH);
    digitalWrite(Self_Led_Backward, HIGH);
    Self_Output_Command = 0;
    START_Alarm();
    Command = 666;    //error 
  }

  // Send_Data
  Serial.write(Command);
  sei(); // разрешаем приём телеметрии
  // Работа с дисплеем
  PrintOnDisplay();  // Какую команду дали, какую получили, токи/напряжение/обороты/прочий угар выводим на дисплей
}

//Отрисовка на дисплее
int PrintOnDisplay()
{
  //GOTO
}

//Приём телеметрии - по прерыванию с порта UART
void serialEvent()
{
  // Приём информации и запись в переменную TelemetryStringArr
  cli(); // Запретим прерывания
  while (Serial.available())
  {
    char inChar = (char) Serial.read();
    TelemetryStringArr = TelemetryStringArr + inChar;
    if (inChar == '\n')         //если это последний символ
    {
      //sei();
    }
  }
  Reset_Error_Timer_And_Check_WDTimer();
  Telemetry_Data_Processing(TelemetryStringArr); //Обработка полученных данных
  TelemetryStringArr = "";   // чистка переменной String
  sei(); // разрешим прерывания
}

void START_Alarm()
{
  AlarmTrigger = true;
  digitalWrite(BUZZER_ALARM, HIGH);
}

void STOP_Alarm()
{
  AlarmTrigger = false;
  digitalWrite(BUZZER_ALARM, LOW);
}


//Обработка полученных данных
void Telemetry_Data_Processing(String StringArr)
{
  //Проверка на верность полученных данных
  //if ( ( StringArr == "F" & Self_Output_Command == 5 ) | ( StringArr == "B" & Self_Output_Command == -5 ) | ( StringArr == "S" & Self_Output_Command == 0 ) )
  if ( ( StringArr == "F" ) | ( StringArr == "B") | ( StringArr == "S" ) )
  {
    Good_Receive_Combo = Good_Receive_Combo + 1;
    cli();
    Reset_Error_Timer_And_Check_WDTimer();
    ErrorCounter_Combo = 0;
    sei();
    if (Good_Receive_Combo == 1)
    {
      if (StringArr == "F" )    // FORWARD
      {
        digitalWrite(FORWARD_LED, HIGH);
        digitalWrite(BACKWARD_LED, LOW);
        digitalWrite(led, LOW);
      }
      if (StringArr == "B")     // BACKWARD
      {
        digitalWrite(BACKWARD_LED, HIGH);
        digitalWrite(FORWARD_LED, LOW);
        digitalWrite(led, LOW);
      }
      if (StringArr == "S")     // STOP
      {
        digitalWrite(led, HIGH);
        digitalWrite(BACKWARD_LED, LOW);
        digitalWrite(FORWARD_LED, LOW);
      }
      Good_Receive_Combo = 0;
    }
  }
  else    //TELEMETRY_WITH_FAIL =(
  {
    Good_Receive_Combo = 0;
    ErrorCounter_Combo = ErrorCounter_Combo + 1;
    if (ErrorCounter_Combo >= 60)
    {
      START_Alarm(); // ТЕСТИТЬ !!!! мб и не 100 надо а другое число
      if (WDT_ACTIVE == false)
      {
        wdt_enable(WDTO_4S); // WDT ENABLE!
        WDT_ACTIVE = true;
      }
      ErrorCounter_Combo = 0;
      //Выключаем наши диоды - у нас потеря связи или тип того.
      digitalWrite(led, HIGH);
      digitalWrite(BACKWARD_LED, HIGH);
      digitalWrite(FORWARD_LED, HIGH);
      //debug_delay
      delay(1000); //!!!! ТУТ ПРОК Оо
      //Serial.println("THRASH on Channel =(  -> " + TelemetryStringArr);   // мусор =(
    }
  }
  // END FOR Processing Data from Telemetry
}

void Reset_Error_Timer_And_Check_WDTimer()
{
  cli();
  if (AlarmTrigger == true)
  {
    STOP_Alarm();  // + AlarmTrigger == false
    if (WDT_ACTIVE == true)
    {
      wdt_disable();         //TURN OFF WDT!
      WDT_ACTIVE = false;
    }
  }
  TCCR2B = 0b00000000;     // 0b00000000 -> STOP_TIMER (CS02 = CS01 = CS00 = 0)
  TCNT2 = 0;               // обнуляем регистр счёта
  TCCR2B = 0b00000011;     // 0b00000010 -> clk/8 (CS02 = 0 / CS01 = 1 / CS00 = 0)
  cntr = 0;
  sei();
}
