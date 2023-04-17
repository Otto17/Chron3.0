/*


  Форк хронографа от Алекс Гайвера (2.1 с дисплеем).


  Изменения:
    - Переделывал под ATmega16 / ATmega168;
    - Вынес в дефайны настройки пинов;
    - Убран режим сна (на практике толку от него не много, к тому же бывает раздрожает);
    - Заменена библиотека со старой "TM1637" (на ней зависала ATmega) на "GyverTM1637";
    - Увеличено время отобравения выбранного режима на дисплее (400 миллисекунд);
    - Частично Русифицирован вывод в Serial (для полной русификации нехватаем ПЗУ, (Кирилица занимает больше места)


  Автор форка: Otto
  Версия: 3.0
  Дата: 17.04.2023


*/



//Аналоговые входы
#define ANALOG_SENSOR_ONE A1  // Вход аналогового ЦАП (Первый фототранзистор), для калибровки значений в предела [400] через Serial
#define ANALOG_SENSOR_TWO A3  // Вход аналогового ЦАП (Второй фототранзистор), для калибровки значений в предела [400] через Serial

//Прерывание
#define SENSOR_ONE 0  // Вход для аппаратного прерывания (Первый фототранзистор)
#define SENSOR_TWO 1  // Вход для аппаратного прерывания (Второй фототранзистор)

//Дисплей на TM1637
#define CLK 5              // Пин CLK дисплея
#define DIO 4              // Пин DIO дисплея
#define BRIGHTNESS_DISP 3  // Яркость дисплея (0-7)

//Кнопка
#define BUT_PIN 8    // Сюда подключена кнопка
#define VCC_PIN 9    // Пин для питания кнопки
#define DEBOUNCE 50  // Время задержки (в мс) для антидребезга

//Расстояние между датчиками (в метрах)
#define DISTANCE 0.0836  // Мерием расстояние штангенциркулем и конвертируем через онлайн конверте (миллиметры в метры)

//Масса снаряда по умолчанию (в граммах)
#define MASS 0.51




#include <EEPROM.h>       //библиотека для работы со внутренней памятью ардуино
#include <GyverTM1637.h>  //библиотека дисплея


GyverTM1637 disp(CLK, DIO);  //обозвать дисплей disp

byte FAIL[4] = { _F, _A, _i, _L };              //надпись FAIL
byte tire[4] = { _dash, _dash, _dash, _dash };  //надпись ----
byte SPED[4] = { _empty, _S, _P, _empty };      //надпись SP
byte EN[4] = { _empty, _E, _N, _empty };        //надпись EN
byte RAP[4] = { _empty, _r, _A, _empty };       //надпись RA
byte CO[4] = { _empty, _C, _O, _empty };        //надпись CO

float dist = DISTANCE;  // Расстояние между датчиками в метрах
char masschar[5];       // Массив символов для перевода

String massstring, velstring, velstring_km, rapidstring, rapidstring_s, shotstring, velstring_aver, energystring;  //строки
int mode, setmass, i, rapidtime;
bool initial, flagmass, flagmassset, rapidflag, button, bstate, show, vel_en, state, flag_m, flag_m2, blink_flag;  //флажки

int n = 1;  //номер выстрела, начиная с 1
unsigned int n_shot;
float velocity, energy;             //переменная для хранения скорости
float mass = MASS;                  //масса снаряда в граммах
volatile unsigned long gap1, gap2;  //отметки времени прохождения пулей датчиков

unsigned long lastshot, time_press, lst;

int disp_text[4];
bool set[4];
byte n_aver;
int mass_array[4], aver[5], sum, aver_velocity;
int num, dig;
uint8_t mass_mem;  //запоминает массу для записи в EEPROM


void setup() {
  Serial.begin(9600);  //открываем COM порт

  pinMode(BUT_PIN, INPUT);      //кнопка подключена сюда
  pinMode(VCC_PIN, OUTPUT);     //питание кнопки
  digitalWrite(VCC_PIN, HIGH);  //питание кнопки вкл

  attachInterrupt(SENSOR_ONE, start, RISING);   // Аппаратное прерывание при прохождении первого датчика
  attachInterrupt(SENSOR_TWO, finish, RISING);  // Аппаратное прерывание при прохождении второго датчика

  disp.clear();                      //инициализация дисплея
  disp.brightness(BRIGHTNESS_DISP);  //яркость (0-7)

  mass = EEPROM.read(0) + (float)EEPROM.read(1) / 100;  //прочитать массу из внутренней памяти
}


void start() {
  if (gap1 == 0) {    //если измерение еще не проводилось
    gap1 = micros();  //получаем время работы ардуино с момента включения до момента пролетания первой пули
  }
}


void finish() {
  if (gap2 == 0) {    //если измерение еще не проводилось
    gap2 = micros();  //получаем время работы ардуино с момента включения до момента пролетания второй пули
  }
}


void print_disp(int x[]) {  //функция для удобной работы с дисплеем (на вход полаётся массив из 4 чисел, они выводятся на дисплей)
  for (int i = 0; i <= 3; i++) {
    disp.display(i, x[i]);  //вывести на дисплей содержимое
  }
}


void mass_set() {
  disp.point(POINT_ON);                  //включить двоеточие
  int mass1_1 = floor(mass);             //взять целую часть от массы пули
  int mass2_1 = (mass - mass1_1) * 100;  //взять дробную часть от массы и превратить ей в целые числа
  mass_array[0] = floor(mass1_1 / 10);   //далее все 4 цифры массы присваиваются в строку mass_array
  mass_array[1] = mass1_1 - mass_array[0] * 10;
  mass_array[2] = floor(mass2_1 / 10);
  mass_array[3] = mass2_1 - mass_array[2] * 10;

  print_disp(mass_array);  //выводим на дисплей полученную массу
  flag_m = 0;              //обнуляем флаги
  num = 0;
  while (flag_m == 0) {
    if (flag_m2 == 0) {
      dig = mass_array[num];  //изменяем цифру массы под номером num
      flag_m2 = 1;
    }
    mass_array[num] = dig;


    //-----------------------------------------------------
    if (digitalRead(BUT_PIN) == 1 && state == 0) {  //выбор режимов кнопкой. Если кнопка нажата
      delay(DEBOUNCE);                              //защита от дребезга
      state = 1;
      button = 1;
      time_press = millis();  //запомнить время нажатия

      while (millis() - time_press < 500) {  //выполнять, пока кнопка нажата не менее 500 миллисекунд
        if (digitalRead(BUT_PIN) == 0) {
          button = 0;
          break;
        }
      }

      if (button == 0) {  //обработка нажатия: короткое - прибавить единицу, длинное - переключить на следующую цифру
        dig++;
        print_disp(mass_array);
        if (dig >= 10) { dig = 0; }
      }

      if (button == 1) {
        num++;
        flag_m2 = 0;
        if (num >= 4) {
          flag_m = 1;
        }
      }
    }

    if (digitalRead(BUT_PIN) == 0 && state == 1) {  //если кнопка отпущена
      state = 0;                                    //скинуть флажок
    }


    //-----------------------------------------------------
    if (millis() - lst > 700 && blink_flag == 0) {
      print_disp(mass_array);
      disp.display(num, _empty);
      lst = millis();
      blink_flag = 1;
    }

    if (millis() - lst > 200 && blink_flag == 1) {
      print_disp(mass_array);
      lst = millis();
      blink_flag = 0;
    }
  }

  mass = mass_array[0] * 10 + mass_array[1] + (float)mass_array[2] / 10 + (float)mass_array[3] / 100;
  mass_mem = mass_array[0] * 10 + mass_array[1];
  EEPROM.write(0, mass_mem);
  mass_mem = mass_array[2] * 10 + mass_array[3];
  EEPROM.write(1, mass_mem);
  disp.point(POINT_OFF);
  delay(500);
  disp.displayByte(tire);
}


void energy_print() {
  disp.point(POINT_ON);
  switch (energystring.length()) {  //кароч тут измеряется длина строки и соотвествено выводится всё на дисплей
    case 4:
      disp.display(0, _empty);
      disp.display(1, energystring[0] - '0');
      disp.display(2, energystring[2] - '0');
      disp.display(3, energystring[3] - '0');
      break;

    case 5:
      disp.display(0, energystring[0] - '0');
      disp.display(1, energystring[1] - '0');
      disp.display(1, energystring[3] - '0');
      disp.display(1, energystring[4] - '0');
      break;
  }
}


void black_print(String x) {
  disp.point(POINT_OFF);
  switch (x.length()) {  //кароч тут измеряется длина строки и соотвествено выводится всё на дисплей
    case 1:
      disp.display(0, _empty);
      disp.display(1, _empty);
      disp.display(2, _empty);
      disp.display(3, x[0] - '0');
      break;

    case 2:
      disp.display(0, _empty);
      disp.display(1, _empty);
      disp.display(2, x[0] - '0');
      disp.display(3, x[1] - '0');
      break;

    case 3:
      disp.display(0, _empty);
      disp.display(1, x[0] - '0');
      disp.display(2, x[1] - '0');
      disp.display(3, x[2] - '0');
      break;
  }
}


void loop() {
  if (initial == 0) {                                          // Флажок первого запуска
    Serial.println("0 - Режим измерения скорости (default)");  // Выход из режимов
    Serial.println("1 - Режим энергии");                       // Режим измерения энергии
    Serial.println("2 - Ускоренный режим");                    // Режим измерения скорострельности
    Serial.println("3 - Режим счета");                         // Режим счёта выстрелов
    Serial.println("4 - Режим набора массы");                  // Режим выбора массы снаряда
    Serial.println("5 - Сервисный режим");                     // Режим отладки (резисторы)
    Serial.println("");

    initial = 1;  // Первый запуск, больше не показываем сообщения
    delay(500);
    disp.displayByte(tire);  // Вывести тире "----"
  }

  if (Serial.available() > 0 && mode != 2) {  //еси есть какие буквы на вход с порта и не выбран 2 режим
    int val = Serial.read();                  //прочитать что было послано в порт
    switch (val) {                            //оператор выбора

      case 48:
        mode = 0;
        flagmass = 0;
        rapidflag = 0;
        initial = 0;
        break;  //если приняли 0 то выбрать 0 режим

      case 49: mode = 1; break;  //если приняли 1 то запустить режим 1
      case 50: mode = 2; break;  //если приняли 2 то запустить режим 2
      case 51: mode = 3; break;  //если приняли 3 то запустить режим 3
      case 52: mode = 4; break;  //если приняли 4 то запустить режим 4
      case 53: mode = 5; break;  //если приняли 5 то запустить режим 5
    }
  }

  if (mode == 5) {  //если 5 режим
    Serial.print("sens 1: ");
    Serial.println(analogRead(ANALOG_SENSOR_ONE));  //показать значение на первом датчике
    Serial.print("sens 2: ");
    Serial.print(analogRead(ANALOG_SENSOR_TWO));  //показать значение на втором датчике
    Serial.println();                             // Перенос строки
    delay(200);
  }

  if (mode == 4) {                           //если 4 режим
    if (flagmass == 0) {                     //флажок чтобы показать надпись только 1 раз
      Serial.print("Масса пули (грамм): ");  //надпись
      flagmass = 1;
    }

    if (Serial.available() > 0)  //если есть что на вход с порта
    {
      massstring = Serial.readStringUntil('\n');  //присвоить massstring всё что было послано в порт
      flagmassset = 1;                            //поднять флажок
    }

    if (flagmassset == 1) {                                //если флажок поднят (приняли значение в порт)
      Serial.println(massstring);                          //написать введённое значение
      massstring.toCharArray(masschar, sizeof(masschar));  //перевод значения в float (десятичная дробь)
      mass = atof(masschar) / 1000;                        //всё ещё перевод
      flagmass = 0;                                        //опустить все флажки
      flagmassset = 0;
      initial = 0;  //показать приветственную надпись
      mode = 0;
    }
  }

  if (gap1 != 0 && gap2 != 0 && gap2 > gap1 && (mode == 0 || mode == 1)) {  //если пуля прошла оба датчика в 0 режиме
    velocity = (1000000 * (dist) / (gap2 - gap1));                          //вычисление скорости как расстояние/время
    energy = velocity * velocity * mass / 1000 / 2;                         //вычисление энергии
    Serial.print("Shot #");
    Serial.println(n);  //вывод номера выстрела
    Serial.print("Speed: ");
    Serial.println(velocity);  //вывод скорости в COM
    Serial.print("Energy: ");
    Serial.println(energy);  //вывод энергии в COM

    velstring = String(round(velocity));  //сделать строку velstring из округлённой скорости
    velstring_km = String(round(velocity * 3.6));
    energystring = String(energy);
    aver[n_aver] = velocity;
    n_aver++;

    if (n_aver >= 5) { n_aver = 0; }
    sum = 0;
    for (int i = 0; i <= 4; i++) {
      sum = sum + aver[i];
    }

    aver_velocity = sum / 5;
    velstring_aver = String(round(aver_velocity));
    gap1 = 0;  //сброс значений
    gap2 = 0;
    show = 1;  //показать значение на дисплее
    n++;       //номер выстрела +1
  }

  if (mode == 2) {  //тест скорострельности
    if (rapidflag == 0) {
      Serial.println("Тест на скорость!");
      Serial.println("");
      rapidflag = 1;
    }

    if (gap1 != 0) {
      rapidtime = 60 / ((float)(gap1 - lastshot) / 1000000);  //расчет
      lastshot = gap1;                                        //запоминаем время последнего выстрела
      rapidstring = String(round(rapidtime));                 //перевод в строку
      rapidstring_s = String(round(rapidtime / 60));
      Serial.print("Rapidity (shot/min): ");
      Serial.println(rapidtime);
      Serial.println("");
      gap1 = 0;
      show = 1;  //показать на дисплее
    }
  }


  //------------------------------------ ОТРАБОТКА НАЖАТИЯ -----------------------------------

  if (digitalRead(BUT_PIN) == 1 && state == 0) {  //выбор режимов кнопкой. Если кнопка нажата
    state = 1;
    button = 1;
    time_press = millis();                 //запомнить время нажатия
    while (millis() - time_press < 500) {  //выполнять, пока кнопка нажата не менее 500 миллисекунд
      if (digitalRead(BUT_PIN) == 0) {
        button = 0;
        break;
      }
    }
    switch (button) {
      case 0:
        mode++;
        if (mode >= 4) { mode = 0; }
        disp.point(POINT_OFF);
        switch (mode) {
          case 0:
            disp.displayByte(SPED);
            delay(500);
            disp.displayByte(tire);
            break;

          case 1:
            disp.displayByte(EN);
            delay(500);
            disp.displayByte(tire);
            break;

          case 2:
            disp.displayByte(RAP);
            delay(500);
            disp.displayByte(tire);
            break;

          case 3:
            disp.displayByte(CO);
            delay(500);
            disp.displayByte(tire);
            break;
        }
        break;

      case 1:
        set[mode] = !set[mode];
        break;
    }
    show = 1;
  }

  if (digitalRead(BUT_PIN) == 0 && state == 1) {  //если кнопка отпущена
    state = 0;                                    //скинуть флажок
  }


  //----------------------------------------------------------------------------------------

  if (show == 1) {
    switch (mode) {
      case 0:
        if (set[mode] == 0) {
          black_print(velstring);
        } else {
          black_print(velstring_aver

          );
        }
        break;

      case 1:
        if (set[mode] == 0) {
          energy_print();
        } else {
          mass_set();
          set[mode] = 0;
        }
        break;

      case 2:
        if (set[mode] == 0) {
          black_print(rapidstring);
        } else {
          black_print(rapidstring_s);
        }
        break;

      case 3:
        if (set[mode] == 0) {
          black_print(shotstring);
        } else {
          n_shot = 0;
          set[3] = 0;
          shotstring = String(n_shot);
          black_print(shotstring);
        }
        break;
    }
    show = 0;
  }

  if (mode == 3) {
    if (gap1 != 0) {
      n_shot++;
      shotstring = String(n_shot);
    }
    gap1 = 0;
    show = 1;
  }

  if (micros() - gap1 > 1000000 && gap1 != 0 && mode != 5) {  // (если пуля прошла первый датчик) И (прошла уже 1 секунда, а второй датчик не тронут)
    disp.point(POINT_OFF);
    disp.displayByte(FAIL);     //вывести fail
    Serial.println("НЕУДАЧА");  //выдаёт "НЕУДАЧА" через 1 секунду, если пуля прошла через первый датчик, а через второй нет
    gap1 = 0;
    gap2 = 0;
    delay(500);
    disp.displayByte(tire);  // вывести -----
  }

  delay(3);  // Задержка перед повторным выполнением кода
}
