/* ЗАДАЧА

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

int incomingByte = 0;
int Command = 0; // -5 / 0 / 5
int cntr = 0;
//BIND PINS NAMES
int Permission_To_Move = 2;    // выставляется после направления движения
int Direction_Of_Movement = 3; //выставляется раньше разрешения
int systemLed = 13;
int BUZZER_PIN = 10;
bool AlarmTrigger = false;

//Обработка прерывания по переполнению счётчика. Должны пищалкой пищать
ISR(TIMER2_OVF_vect) {
  TCNT2 = 0; //55; Костыльное решение для изменения скорости моргания. ЛОЛ
  cntr++;
  if (cntr > 1000)
  {
    // ALARM
    Alarm_ON();
    cntr = 0;
  }
}


void setup()
{
  Serial.begin(9600);
  pinMode(Permission_To_Move, OUTPUT);
  pinMode(Direction_Of_Movement, OUTPUT);
  pinMode(systemLed, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  //STOP на двигатели в самом начале. На всякий случай!!!
  digitalWrite(Direction_Of_Movement  , LOW);
  digitalWrite(Permission_To_Move, LOW);

  //Настройка таймера 2
  TCCR2A = 0;     // 0b00000000 -> Normal режим (никаких ШИМ) / Не ставили условий для ноги OC0A(12)
  TCNT2 = 0;      // 0b00111011 -> Это просто регистр со значениями куда всё тикает. зачем там 59?
  TIMSK2 = 0b00000001;  //0b00000001 - TOIE - до переполнения // ИЛИ -> TIMSK2 = 1; ИЛИ -> TIMSK2 |= (1 << TOIE2);
  TCCR2B = 0b00000011;  // 0b00000010 -> clk/8 (CS02 / CS01 / CS00)
}

void loop()
{
  Command_To_Motor(Command);
  Send_Telemetry (Command);
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
  if (incomingByte >= 140 & incomingByte <= 255)
  {
    Command = 5; // FORWARD
    Reset_Error_Timer();
  }
  if (incomingByte >= 90 & incomingByte < 140)
  {
    Command = -5; // BACKWARD
    Reset_Error_Timer();
  }
  if (incomingByte >= 0 & incomingByte < 90)
  {
    Command = 0; // STOP
    Reset_Error_Timer();
  }

  if (Command != -5 & Command != 0 & Command != 5 )
  {
    Serial.print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAASHIBKAAAAA");
    digitalWrite(Permission_To_Move, LOW);
    digitalWrite(Direction_Of_Movement, LOW);
    digitalWrite(systemLed, LOW);
    delay(500);
  }

}

void Command_To_Motor (int instruction)
{
  switch (instruction)
  {
    case -5: //BACKWARD
      digitalWrite(Direction_Of_Movement, HIGH);
      digitalWrite(Permission_To_Move, HIGH);
      digitalWrite(systemLed, LOW);
      break;
    case 0: //STOP
      digitalWrite(Direction_Of_Movement, LOW);
      digitalWrite(Permission_To_Move, LOW);
      digitalWrite(systemLed, HIGH);
      break;
    case 5: //FORWARD
      digitalWrite(Permission_To_Move, HIGH);
      digitalWrite(Direction_Of_Movement, LOW);
      digitalWrite(systemLed, LOW);
      break;
    default:
      Alarm_ON();
      // выполняется, если не выбрана ни одна альтернатива
      // default необязателен
  }
}

void Send_Telemetry (int instruction)
{
  cli();
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
  sei();
  delay(5);
}

void Reset_Error_Timer()
{
  cli();
  if (AlarmTrigger == true)
  {
    digitalWrite(BUZZER_PIN, LOW);
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
