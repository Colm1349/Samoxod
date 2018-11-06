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
int led = LED_BUILTIN;  //диод на плате
int input_sensor = 0;
String TelemetryStringArr;
int ErrorCounter = 0;
int cntr = 0;

int FORWARD_LED = 11;   //DEBUG
int BACKWARD_LED = 12;  //DEBUG
int BUZZER_ALARM = 4;   //DEBUG
bool AlarmTrigger = false;

//Обработка прерывания по переполнению счётчика. Должны пищалкой пищать
ISR(TIMER2_OVF_vect)
{
  TCNT2 = 0; //55; Костыльное решение для изменения скорости моргания. ЛОЛ
  cntr++;
  if (cntr > 1000)
  {
    START_Alarm();
    cntr = 0;
    digitalWrite(led, LOW);
    digitalWrite(BACKWARD_LED, LOW);
    digitalWrite(FORWARD_LED, LOW);
  }
}

void setup()
{
  Serial.begin(9600);
  pinMode(input_sensor, INPUT);
  pinMode(led, OUTPUT);
  pinMode(FORWARD_LED, OUTPUT);
  pinMode(BACKWARD_LED, OUTPUT);
  pinMode(BUZZER_ALARM, OUTPUT);

  //Индикация OFF на начале работы
  digitalWrite(led, LOW);
  digitalWrite(FORWARD_LED, LOW);
  digitalWrite(BACKWARD_LED, LOW);
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
  int value = analogRead(input_sensor);
  int reborn = map(value, 0, 1023, 0, 255); //приводим к значению от 0 до 255
  // Send_Data
  Serial.write(reborn);
  sei(); // разрешаем приём телетрии
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
  Reset_Error_Timer();
  //Обработка полученных данных
  Telemetry_Data_Processing(TelemetryStringArr);
  // чистка переменной String
  TelemetryStringArr = "";
  sei(); // разрешим прерывания
}

void START_Alarm()
{
  AlarmTrigger = true;
  digitalWrite(led, HIGH);
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
  bool Telemetry_okay = false;
  if (StringArr == "F" )    // FORWARD
  {
    digitalWrite(FORWARD_LED, HIGH);
    digitalWrite(BACKWARD_LED, LOW);
    digitalWrite(led, LOW);
    Telemetry_okay = true;
  }
  if (StringArr == "B")     // BACKWARD
  {
    digitalWrite(BACKWARD_LED, HIGH);
    digitalWrite(FORWARD_LED, LOW);
    digitalWrite(led, LOW);
    Telemetry_okay = true;
  }
  if (StringArr == "S")     // STOP
  {
    digitalWrite(led, HIGH);
    digitalWrite(BACKWARD_LED, LOW);
    digitalWrite(FORWARD_LED, LOW);
    Telemetry_okay = true;
  }

  //TELEMETRY_WITH_FAIL =(
  if (Telemetry_okay == false)
  {
    ErrorCounter = ErrorCounter + 1;
    if (ErrorCounter >= 60)
    {
      START_Alarm(); // ТЕСТИТЬ !!!! мб и не 60 надо а другое число
      ErrorCounter = 0;
      //Выключаем наши диоды - у нас потеря связи или тип того.
      digitalWrite(led, LOW);
      digitalWrite(BACKWARD_LED, LOW);
      digitalWrite(FORWARD_LED, LOW);
      //Serial.println("THRASH on Channel =(  -> " + TelemetryStringArr);   // мусор =(
    }
  }
  else
  {
    // ALL OK
    STOP_Alarm();
    ErrorCounter = 0;
    Reset_Error_Timer();
  }
  // END FOR Processing Data from Telemetry
}

void Reset_Error_Timer()
{
  cli();
  if (AlarmTrigger == true)
  {
    STOP_Alarm();
  }
  TCCR2B = 0b00000000;     // 0b00000000 -> STOP_TIMER (CS02 = CS01 = CS00 = 0)
  TCNT2 = 0;               // обнуляем регистр счёта
  TCCR2B = 0b00000011;     // 0b00000010 -> clk/8 (CS02 = 0 / CS01 = 1 / CS00 = 0)
  cntr = 0;
  sei();
}
