
//Команда разработчиков:  Рожков Юрий Валерьевич
//                        Белянин Дмитрий


#include "pitches.h"

#include <LiquidCrystal_I2C.h>
#include <Bounce2.h>
#include <Wire.h>
#include <SPI.h>      // подключаем библиотеку SPI
#include <EEPROM.h>

// Определяем пины кнопок

#define pinButtZaliv 23 // пин кнопки "ЗАЛИВ В ДОРОЖКУ ИЗ БАКА" - Кн1
#define pinButtSliv 25  // пин кнопки "СЛИВ ИЗ ДОРОЖКИ В БАК" - Кн2
#define pinButtTok 27 // пин кнопки ПРОТИВОТОК - Кн3
#define pinButtSliv1 29 // пин кнопки "СЛИВ ИЗ ДОРОЖКИ В КАНАЛИЗАЦИЮ" - Кн4
#define pinButtSliv2 31 // пин кнопки "СЛИВ ИЗ БАКА В КАНАЛИЗАЦИЮ" - Кн5
#define pinButtZaliv1 33 // пин кнопки "ЗАЛИВ ИЗ ВОДОПРОВОДА В БАК" - Кн6
#define pinKoncevik2 35 // пин концевика №2 в баке
#define pinKoncevik1 37 // пин концевика №1 в баке
#define pinButtPlus 41 // пин кнопки "+" СКОРОСТИ МОТОРА - Кн9
#define pinButtMinus 39 // пин кнопки "-" СКОРОСТИ МОТОРА - Кн10
#define pinButtTokPlus 43 // пин кнопки "+" ПРОТИВОТОКА - Кн11
#define pinButtTokMinus 45 // пин кнопки "-" ПРОТИВОТОКА - Кн12
#define pinButtRevers 47 // пин кнопки "РЕВЕРС" - Кн13
#define pinButtStop 49 // пин кнопки STOP - Кн14

//Дополнительные выходы
#define pinZummer 8 // пин Зуммера

// Определяем пины выходов на управление реле

#define pinLampZaliv 5 // пин выхода индикации режива ЗАЛИВ (Кн1)
#define pinLampSliv 3 // пин выхода индикации режива СЛИВ ИЗ ДОРОЖКИ В БАК (Кн2)
#define pinLampTok 4 // пин выхода индикации включенного ПРОТИВОТОКА (Кн3)
#define pinLampSliv1 2 // пин выхода индикации режива СЛИВ ИЗ ДОРОЖКИ В КАНАЛИЗАЦИЮ (Кн4)


#define pinLampRevers 10 // пин выхода индикации режива Revers

// Пины на релелейные выходы
#define pinN11 22 // пин выход на реле1 Канал1 (Кран1)
#define pinN12 24 // пин выход на реле1 Канал2 (Кран2)
#define pinN13 26 // пин выход на реле1 Канал3 (Кран3)
#define pinN14 28 // пин выход на реле1 Канал4 (Кран4)
#define pinN15 30 // пин выход на реле1 Канал5 (Кран5)
#define pinN16 32 // пин выход на реле1 Канал6 (Кран6)
#define pinN17 34 // пин выход на реле1 Канал7 (Кран7)
#define pinN18 36 // пин выход на реле1 Канал8 (Реверс)
#define pinN21 38 // пин выход на реле2 Канал1 (помпа1)
#define pinN22 40 // пин выход на реле2 Канал2 (помпа2)
#define pinN23 42 // пин выход на реле2 Канал3 (противоток)
#define pinMotor 44 // пин выход на реле МОТОР
#define pinN25 46 // пин выход на реле2 Канал5 (индикация Кн5)
#define pinN26 48 // пин выход на реле2 Канал6 (индикация Кн6)
#define pinN27 A1 // пин выход на реле2 Канал7 (включение зарядки АКБ)
#define pinRelays 6 // пин выход на активацию релейных модулей
#define pinRevers 36 // пин выход на реле2 РЕВЕРС
#define pinPWMMotor 11 // пин выход ШИМ на управление МОТОРОМ

LiquidCrystal_I2C lcd(0x3F, 16, 2); // Устанавливаем дисплей

const int CS = 53; // пин управления цифровым потенциометром MCP41010 для регулировки скорости противотока

const int delay1 = 10; // время в мс задержки при удержании кнопок "+" или "-"
const int delay2 = 50; // время в мс задержки для плавной остановки мотора

const int antiDrebezg = 5; // Параметр антидребезга
const int minSpeed = 12; // Минимальный шим для скорости 0,1 км/ч
const int maxSpeed = 160; // Максимальная скорость 16 км/ч*10
const int accumLimit = 400; // порог измерения напряжения АКБ
const float koeffSpeed = 0.03; // коэффициент расчета скорости = передат.отношение ремня*радиус вала полотна (м)/1000

const long timeTimer120s = 180000; // таймер задержки открытия крана 7 в режиме Кн.2
const long timeTimer30s = 30000; // таймер задержки выключения помпы (каналы 7 и 8)
const long timeTimer80s = 80000; // таймер определяющий необходимость включения помпы (каналы 7 и 8) после выключения режима Кн.2
const long timeTimer25s = 25000; // таймер задержки включения помпы (каналы 7 и 8) при срабатывании конц.2
const long timeAccumReset = 5000; // время удержания кнопки СТОП для сброса проверки АКБ
const long timeAccumCharge = 1200000; // время зарядки АКБ
const long time1AccumCharge = 10000; // время зарядки АКБ
const long timeAccumTest = 5000; // время тестирования просадки напряжения АКБ

const long koeffProbeg = 36000; // коэффициент для расчета пробега

byte bukva_B[8]   = {B11110, B10000, B10000, B11110, B10001, B10001, B11110, B00000,}; // Буква "Б"
byte bukva_G[8]   = {B11111, B10001, B10000, B10000, B10000, B10000, B10000, B00000,}; // Буква "Г"
byte bukva_D[8]   = {B01111, B00101, B00101, B01001, B10001, B11111, B10001, B00000,}; // Буква "Д"
byte bukva_ZH[8]  = {B10101, B10101, B10101, B11111, B10101, B10101, B10101, B00000,}; // Буква "Ж"
byte bukva_Z[8]   = {B01110, B10001, B00001, B00010, B00001, B10001, B01110, B00000,}; // Буква "З"
byte bukva_I[8]   = {B10001, B10011, B10011, B10101, B11001, B11001, B10001, B00000,}; // Буква "И"
byte bukva_IY[8]  = {B01110, B00000, B10001, B10011, B10101, B11001, B10001, B00000,}; // Буква "Й"
byte bukva_L[8]   = {B00011, B00111, B00101, B00101, B01101, B01001, B11001, B00000,}; // Буква "Л"
byte bukva_P[8]   = {B11111, B10001, B10001, B10001, B10001, B10001, B10001, B00000,}; // Буква "П"
byte bukva_Y[8]   = {B10001, B10001, B10001, B01010, B00100, B01000, B10000, B00000,}; // Буква "У"
byte bukva_F[8]   = {B00100, B11111, B10101, B10101, B11111, B00100, B00100, B00000,}; // Буква "Ф"
byte bukva_TS[8]  = {B10010, B10010, B10010, B10010, B10010, B10010, B11111, B00001,}; // Буква "Ц"
byte bukva_CH[8]  = {B10001, B10001, B10001, B01111, B00001, B00001, B00001, B00000,}; // Буква "Ч"
byte bukva_Sh[8]  = {B10101, B10101, B10101, B10101, B10101, B10101, B11111, B00000,}; // Буква "Ш"
byte bukva_Shch[8] = {B10101, B10101, B10101, B10101, B10101, B10101, B11111, B00001,}; // Буква "Щ"
byte bukva_Mz[8]  = {B10000, B10000, B10000, B11110, B10001, B10001, B11110, B00000,}; // Буква "Ь"
byte bukva_IYI[8] = {B10001, B10001, B10001, B11001, B10101, B10101, B11001, B00000,}; // Буква "Ы"
byte bukva_Yu[8]  = {B10010, B10101, B10101, B11101, B10101, B10101, B10010, B00000,}; // Буква "Ю"
byte bukva_Ya[8]  = {B01111, B10001, B10001, B01111, B00101, B01001, B10001, B00000,}; // Буква "Я"
byte bukva_che[8]  = {B00000, B00000, B01001, B01001, B01111, B00001, B00001, B00000,}; // Буква "ч"


int table[8][12] = {
  {pinN11, pinN12, pinN13, pinN14, pinN15, pinN16, pinN17, pinN21, pinN22, pinN23, pinN25, pinN26},
  {HIGH, LOW, LOW, HIGH, HIGH, HIGH, HIGH, LOW, LOW, HIGH, HIGH, HIGH},
  {LOW, HIGH, HIGH, LOW, HIGH, HIGH, HIGH, LOW, LOW, HIGH, HIGH, HIGH},
  {LOW, HIGH, HIGH, HIGH, LOW, HIGH, HIGH, LOW, HIGH, LOW, HIGH, HIGH},
  {LOW, HIGH, HIGH, HIGH, HIGH, LOW, HIGH, HIGH, HIGH, HIGH, HIGH, LOW},
  {HIGH, HIGH, LOW, HIGH, HIGH, LOW, HIGH, LOW, LOW, HIGH, LOW, HIGH},
  {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, LOW},
  {LOW, HIGH, HIGH, HIGH, HIGH, LOW, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}
};

int melody[] = {
  NOTE_A4, NOTE_G5, NOTE_F4, NOTE_A5, NOTE_C5
};
// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {
  8, 8, 4, 8, 8
};

Bounce Button1 = Bounce(); //Объект для программного устранения дребезга кнопки 1 "Залив в дорожку из бака"
Bounce Button2 = Bounce(); //Объект для программного устранения дребезга кнопки 2 "СЛИВ из дорожки в бак"
Bounce Button3 = Bounce(); //Объект для программного устранения дребезга кнопки 3 ПРОТИВОТОК
Bounce Button4 = Bounce(); //Объект для программного устранения дребезга кнопки 4 "СЛИВ ИЗ ДОРОЖКИ В КАНАЛИЗАЦИЮ"
Bounce Button5 = Bounce(); //Объект для программного устранения дребезга кнопки 5 "СЛИВ ИЗ БАКА В КАНАЛИЗАЦИЮ"
Bounce Button6 = Bounce(); //Объект для программного устранения дребезга кнопки 6 "Залив в бак"
Bounce Button7 = Bounce(); //Объект для программного устранения дребезга концевика2 в баке
Bounce Button8 = Bounce(); //Объект для программного устранения дребезга концевика1 в баке
Bounce Button9 = Bounce(); //Объект для программного устранения дребезга кнопки "+" СКОРОСТИ МОТОРА
Bounce Button10 = Bounce(); //Объект для программного устранения дребезга кнопки "-" СКОРОСТИ МОТОРА
Bounce Button11 = Bounce(); //Объект для программного устранения дребезга кнопки "+" ПРОТИВОТОКА
Bounce Button12 = Bounce(); //Объект для программного устранения дребезга кнопки "-" ПРОТИВОТОКА
Bounce Button13 = Bounce(); //Объект для программного устранения дребезга кнопки "РЕВЕРС"
Bounce Button14 = Bounce(); //Объект для программного устранения дребезга кнопки 10 STOP

// Определяем глобальные переменные

boolean statusMotor = false; //Статус мотора
boolean statusZaliv = false; //Статус залива
boolean statusSliv = false; //Статус слива из дорожки в бак
boolean statusSliv1 = false; //Статус слива из дорожки в канализацию
boolean statusSliv2 = false; //Статус слива из бака в канализацию
boolean statusZaliv1 = false; //Статус залива из водопровода в бак
boolean statusRevers = false; //Статус включения реверса
boolean statusTok = false; //Статус включения противотока
boolean statusKoncevik2 = false; //Статус концевика 2
boolean statusKoncevik1 = false; //Статус концевика 1
boolean statusBadAccum = false; //Статус аккумулятора
boolean status80s = false; //Статус прошло 80 секунд режима кн.2
boolean printProbeg = false;
boolean status2to4 = false;
boolean statusWaitAccum = false;


boolean timer120s = false; //Статус включения отсчета времени на включение канала 7
boolean timer30s = false; //Статус включения отсчета времени на выключение каналов 8 и 9
boolean timer25s = false; //Статус включения отсчета времени на включение 1 и 2 помпы реле 2.1 и 2.2
boolean timer20m = false; //Статус включения отсчета времени зарядки АКБ
boolean timer10m = false; //Статус включения отсчета времени на тест АКБ
boolean timer80s = false; //Статус включения отсчета времени режима кн.2
boolean firstStart = true; //

int resist = 0; // позиция потенциометра
int resist1 = 0; //

int speedMotor = 0; // параметр ШИМ управления скоростью мотора
unsigned long timeStamp = 0; // переменная для засечки интервала времени
unsigned long time120s = 0; // переменная для засечки интервала времени
unsigned long time30s = 0; // переменная для засечки интервала времени
unsigned long time25s = 0; // переменная для засечки интервала времени
unsigned long time20m = 0; // переменная для засечки интервала времени
unsigned long time10m = 0; // переменная для засечки интервала времени
unsigned long time1m = 0; // переменная для засечки интервала времени
unsigned long time10s = 0; // переменная для засечки интервала времени
unsigned long time80s = 0; // переменная для засечки интервала времени
unsigned long timeOneS = 0; // переменная для засечки интервала времени

unsigned long timeSpeed1 = 0; // переменная для засечки интервала времени
float probeg = 0; // переменная для подсчета пробега
float probeg1 = 0; // переменная для подсчета пробега
int speed10Kmch = 0; // Скорость полотна в км/10ч
int motorPWM = 0; //переменная для храниения значения ШИМ мотора

int indexChar = 0;
const char arrayChars[2] = {'\x7F', ' '};// массив на 2 символа для индикации мигающей стрелочки

int indexCharSet = 0;// переменная для записи текущего набора русских символов

int value1 = HIGH; // Задаем значение кнопки3
int value2 = HIGH; // Задаем значение кнопки4
int value3 = HIGH; // Задаем значение кнопки3
int value4 = HIGH; // Задаем значение кнопки4
int value5 = HIGH; // Задаем значение кнопки5
int value6 = HIGH; // Задаем значение кнопки6
int value7 = HIGH; // Задаем значение кнопки7
int value8 = HIGH; // Задаем значение кнопки8
int value9 = HIGH; //Задаем значение кнопки9
int value10 = HIGH; // Задаем значение кнопки10
int value11 = HIGH; // Задаем значение кнопки11
int value12 = HIGH; // Задаем значение кнопки12
int value13 = HIGH; // Задаем значение кнопки13
int value14 = HIGH; // Задаем значение кнопки14

void setup() {
  Serial.begin(9600);
  pinMode (pinButtZaliv, INPUT_PULLUP); // настройка пина кнопки "ЗАЛИВ"
  pinMode (pinButtSliv, INPUT_PULLUP); // настройка пина кнопки "СЛИВ ИЗ ДОРОЖКИ В БАК"
  pinMode (pinButtTok, INPUT_PULLUP); // настройка пин кнопки ПРОТИВОТОК
  pinMode (pinButtSliv1, INPUT_PULLUP); // настройка пин кнопки "СЛИВ ИЗ ДОРОЖКИ В КАНАЛИЗАЦИЮ"
  pinMode (pinButtSliv2, INPUT_PULLUP); // настройка пин кнопки "СЛИВ ИЗ БАКА В КАНАЛИЗАЦИЮ"
  pinMode (pinButtZaliv1, INPUT_PULLUP); // настройка пин кнопки "Залив в БАКА из водопровода"

  pinMode (pinKoncevik2, INPUT_PULLUP); // настройка пин концевика №2 в баке
  pinMode (pinKoncevik1, INPUT_PULLUP); // настройка пин концевика №1 в баке

  pinMode (pinButtPlus, INPUT_PULLUP); // настройка пин кнопки "+" СКОРОСТИ МОТОРА
  pinMode (pinButtMinus, INPUT_PULLUP); // настройка пин кнопки "-" СКОРОСТИ МОТОРА
  pinMode (pinButtTokPlus, INPUT_PULLUP); // настройка пин кнопки "+" ПРОТИВОТОКА
  pinMode (pinButtTokMinus, INPUT_PULLUP); // настройка пин кнопки "-" ПРОТИВОТОКА
  pinMode (pinButtRevers, INPUT_PULLUP); // настройка пин кнопки "РЕВЕРС"
  pinMode (pinButtStop, INPUT_PULLUP); // настройка пин кнопки STOP

  pinMode (pinZummer, OUTPUT); // пин Зуммера

  // Определяем пины выходов на реле

  pinMode (pinN11, OUTPUT); // настройка пин выход на реле 1
  pinMode (pinN12, OUTPUT); // настройка пин выход на реле 2
  pinMode (pinN13, OUTPUT); // настройка пин выход на реле 3
  pinMode (pinN14, OUTPUT); // настройка пин выход на реле 4
  pinMode (pinN15, OUTPUT); // настройка пин выход на реле 5
  pinMode (pinN16, OUTPUT); // настройка пин выход на реле 6
  pinMode (pinN17, OUTPUT); // настройка пин выход на реле 7
  pinMode (pinN18, OUTPUT); // настройка пин выход на реле 8
  pinMode (pinN21, OUTPUT); // настройка пин выход на реле 2.1
  pinMode (pinN22, OUTPUT); // настройка пин выход на реле 2.2
  pinMode (pinN23, OUTPUT); // настройка пин выход на реле 2.3

  pinMode (pinRelays, OUTPUT); // настройка пин выход на включение блока реле

  pinMode (pinLampZaliv, OUTPUT); // настройка пин выхода индикации режима - Залив в дорожку (Кн1)
  pinMode (pinLampSliv, OUTPUT); // настройка пин выхода индикации режима - Слив из дорожки в бак (Кн2)
  pinMode (pinLampTok, OUTPUT); // настройка пин выхода индикации режима - Противоток (Кн3)
  pinMode (pinLampSliv1, OUTPUT); // настройка пин выхода индикации режима - Слив из дорожки в бак (Кн4)
  pinMode (pinN25, OUTPUT); // настройка пин выхода индикации режима - (Кн5)
  pinMode (pinN26, OUTPUT); // настройка пин выхода индикации режима - (Кн6)
  pinMode (pinN27, OUTPUT); // настройка пин выхода включения зарядки АКБ

  pinMode (pinLampRevers, OUTPUT); // настройка пин выход на реле 1.8 (реверс)

  pinMode (CS, OUTPUT);  // настройка пин выход на MCP41010
  pinMode (pinMotor, OUTPUT); // настройка пин выход на реле МОТОР
  pinMode (pinPWMMotor, OUTPUT); // настройка пин выход на реле МОТОР

  //В цикле подаем подплюсовку на все реле в релейных модулях
  for (int j = 0; j <= 11; j++) digitalWrite(table[0][j], HIGH ); // выключаем реле

  digitalWrite(pinMotor, HIGH );// выключаем выход на МОТОР
  digitalWrite(pinRevers, HIGH );// выключаем выход Revers
  digitalWrite(pinN27, HIGH );// выключаем выход Зарядка АКБ

  SPI.begin(); // инициализация SPI для управления цифровым потенциометром

  Button1.attach(pinButtZaliv); //Назначение пина для антидребезга кнопки 1
  Button2.attach(pinButtSliv); //Назначение пина для антидребезга кнопки 2
  Button3.attach(pinButtTok); //Назначение пина для антидребезга кнопки 3
  Button4.attach(pinButtSliv1); //Назначение пина для антидребезга кнопки 4
  Button5.attach(pinButtSliv2); //Назначение пина для антидребезга кнопки 5
  Button6.attach(pinButtZaliv1); //Назначение пина для антидребезга кнопки 6
  Button7.attach(pinKoncevik2); //Назначение пина для антидребезга концевика 2
  Button8.attach(pinKoncevik1); //Назначение пина для антидребезга концевика 1
  Button9.attach(pinButtPlus); //Назначение пина для антидребезга кнопки "+" скорости мотора
  Button10.attach(pinButtMinus); //Назначение пина для антидребезга кнопки "-" скорости мотора
  Button11.attach(pinButtTokPlus); //Назначение пина для антидребезга кнопки "+" противотока
  Button12.attach(pinButtTokMinus); //Назначение пина для антидребезга кнопки "-" противотока
  Button13.attach(pinButtRevers); //Назначения пина для антидребезга кнопки Revers
  Button14.attach(pinButtStop); //Назначения пина для антидребезга кнопки STOP

  Button1.interval(antiDrebezg);// параметр антидребезга
  Button2.interval(antiDrebezg);// параметр антидребезга
  Button3.interval(antiDrebezg);// параметр антидребезга
  Button4.interval(antiDrebezg);// параметр антидребезга
  Button5.interval(antiDrebezg);// параметр антидребезга
  Button6.interval(antiDrebezg);// параметр антидребезга
  Button7.interval(antiDrebezg);// параметр антидребезга
  Button8.interval(antiDrebezg);// параметр антидребезга
  Button9.interval(antiDrebezg);// параметр антидребезга
  Button10.interval(antiDrebezg);// параметр антидребезга
  Button11.interval(antiDrebezg);// параметр антидребезга
  Button12.interval(antiDrebezg);// параметр антидребезга
  Button13.interval(antiDrebezg);// параметр антидребезга
  Button14.interval(antiDrebezg);// параметр антидребезга

  lcd.init();             //инициализация дисплея

  lcd.clear();

  lcd.backlight();        // Включаем подсветку дисплея

  printFit4Pet();         // Печать верхней строчки с Локотипом Fit4Pet
  setChars5();

  // вывод надписи "Загрузка"
  lcd.setCursor(3, 1);    
  lcd.write(byte(0));
  lcd.print("A\1P\5");
  lcd.write(byte(0));
  lcd.print("KA..."); 
  delay(2000);

  playMelody();// мелодия - приветствие

  TCCR1A = TCCR1A & 0xe0 | 3;    // Определяем работу ШИМ на 11 пине - 10 бит, 244,14 Гц
  TCCR1B = TCCR1B & 0xe0 | 0x0b; // Определяем работу ШИМ на 11 пине - 10 бит, 244,14 Гц

  digitalWrite(pinRelays, HIGH);// включаем подачу + на релейные модули


  //*******************************************************************************************
  //Проверка АКБ
  lcd.setCursor(0, 1);    //курсор на первую строчку посередине
  lcd.print("  TECT AK\2...  ");
  delay(3000);
  if (analogRead(A0) < accumLimit) {
    digitalWrite(pinN27, LOW);
    lcd.setCursor(10, 1);    //курсор на первую строчку посередине
    lcd.print("- NG!");
    delay(1000);
    lcd.setCursor(0, 1);    //курсор на первую строчку посередине
    lcd.write(byte(0));
    lcd.print("AP\3\4KA AK\2...  ");
    time1m = millis();
    do {
      lcd.setCursor(11, 1);    //курсор
      lcd.print("     ");
      delay(500);
      lcd.setCursor(11, 1);    //курсор
      lcd.print(" .   ");
      delay(500);
      lcd.setCursor(11, 1);    //курсор
      lcd.print(" ..  ");
      delay(500);
      lcd.setCursor(11, 1);    //курсор
      lcd.print(" ... ");
      delay(500);
    } while ((millis() - time1m) < time1AccumCharge);
    delay(5000);
    if (analogRead(A0) < accumLimit) {
      lcd.setCursor(0, 1);    //курсор на первую строчку посередине
      lcd.print(" ");
      lcd.write(byte(0));
      lcd.print("AMEHA AK\2!!! ");
      statusBadAccum = true;
      time10s = millis();
      do {
        buttonRead ();
        if (value14 == HIGH) time10s = millis();
      } while (!((value14 == LOW) && ((millis() - time10s) > timeAccumReset)));
      do buttonRead (); while ( value14 == LOW);
      timer20m = false;
      statusBadAccum = false;
      lcd.clear();
    }
    else {
      lcd.setCursor(11, 1);
      lcd.print(" - ok!");
      delay(2000);
      statusBadAccum = false;
    }
  }
  else {
    lcd.setCursor(10, 1);
    lcd.print(" - ok!");
    delay(2000);
    statusBadAccum = false;
  }

  lcd.setCursor(0, 1);
  lcd.print(" C\6CTEMA ");
  lcd.write(byte(1));
  lcd.print("OTOBA  ");

  //***************************************************************************
  time20m = millis();
  timer20m = true;
  digitalWrite(pinN27, LOW);

  //  EEPROM.write(1, 0);  // обнуление счетчика пробега
  //  EEPROM.write(2, 0); // обнуление счетчика пробега
  //  EEPROM.write(3, 0); // обнуление счетчика пробега

  probeg = readProbeg(); // считываем пробег из энергонезависимой памяти

}

void loop() {
  while (statusBadAccum) {
    setChars1();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("  ");
    lcd.write(byte(0));
    lcd.print("AMEHA AK\1!!  ");
    time10s = millis();
    do {
      buttonRead ();
      if (value14 == HIGH) {
        time10s = millis();
      }
      tone(pinZummer, 698, 200); // сигнал зуммера
      delay(400);
    } while (!((value14 == LOW) && ((millis() - time10s) > timeAccumReset)));
    setChars2();
    do buttonRead (); while ( value14 == LOW);
    lcd.clear();
    statusBadAccum = false;
    timer20m = false;
    timer10m = false;
  }

  if ((timer20m) && ((millis() - time20m) > timeAccumCharge)) {
    timer20m = false;
    digitalWrite(pinN27, HIGH);
    timer10m = true;
    time10m = millis();
  }

  if ((timer10m) && ((millis() - time10m) > timeAccumTest)) {
    if (analogRead(A0) > accumLimit) {
      timer10m = false;
      timer20m = false;
      statusWaitAccum = true;
    }
    else statusBadAccum = true;
  }

  if ((analogRead(A0) < accumLimit) && statusWaitAccum) {
    timer20m = true;
    time20m = millis();
    digitalWrite(pinN27, LOW);
    statusWaitAccum = false;
  }

  if ((timer120s) && ((millis() - time120s) > timeTimer120s)) {
    digitalWrite(table[0][6], LOW); // включаем реле 7 через время определенное константой timeTimer120s
    digitalWrite(table[0][11], LOW); // включаем реле 11 через время определенное константой timeTimer120s
    statusZaliv1 = true;
    timer120s = false;
  }

  if ((timer30s) && ((millis() - time30s) > timeTimer30s)) {
    digitalWrite(table[0][7], HIGH); // выключаем реле 8 через 30с
    digitalWrite(table[0][8], HIGH); // выключаем реле 9 через 30с
    timer30s = false;
  }

  if ((timer25s) && ((millis() - time25s) > timeTimer25s)) {
    digitalWrite(table[0][7], LOW); // выключаем реле 8 через 25с
    digitalWrite(table[0][8], LOW); // выключаем реле 9 через 25с
    timer25s = false;
  }

  if ((timer80s) && ((millis() - time80s)) > timeTimer80s) status80s = true;


  // Мигание стрелочки при реверсе
  if (statusRevers && statusMotor && (speed10Kmch >= 1)) {
    if ((millis() - timeOneS) > 500) {
      timeOneS = millis();
      lcd.setCursor(8, 0);
      lcd.print(arrayChars[indexChar]);
      indexChar++;
      if (indexChar == 3) indexChar = 0;
    }
  }

  //Опрос нажатия кнопок
  buttonRead ();

  statusKoncevik1 = (value8 == HIGH) ? false : true;// обновляем статус нажатого концевика 1
  statusKoncevik2 = (value7 == HIGH) ? false : true;// обновляем статус нажатого концевика 2

  if ( value1 == LOW || value2 == LOW || value3 == LOW || value4 == LOW || value5 == LOW || value6 == LOW || value7 == LOW || value8 == LOW || value9 == LOW || value10 == LOW || value11 == LOW || value12 == LOW || value13 == LOW || value14 == LOW) {
    if (firstStart) {
      firstStart = false;
      lcd.setCursor(0, 1);
      lcd.print("                ");
      lcd.createChar(0, bukva_Z); // Создаем символ под номером 0
      lcd.createChar(1, bukva_B); // Создаем символ под номером 1
      lcd.createChar(2, bukva_L); // Создаем символ под номером 2
      lcd.createChar(3, bukva_IYI); // Создаем символ под номером 3
      lcd.createChar(4, bukva_P); // Создаем символ под номером 4
      lcd.createChar(5, bukva_I); // Создаем символ под номером 5
      lcd.createChar(6, bukva_Mz); // Создаем символ под номером 6
      lcd.createChar(7, bukva_che); // Создаем символ под номером 7
    }

    if ( value1 == LOW || value2 == LOW || value3 == LOW || value4 == LOW || value5 == LOW || value6 == LOW || value9 == LOW || value10 == LOW || value13 == LOW || value14 == LOW) {
      tone(pinZummer, 698, 50); // сигнал зуммера на нажатие кнопки кроме концевиков в баке и скорости противотока
    }

    //Если нажата кнопка №14 - СТОП
    if ( value14 == LOW ) buttonStopPressed();

    //Если нажата кнопка №1 - ЗАЛИВ в дорожку из бака
    if ( value1 == LOW ) buttonZalivPressed();

    //Если нажата кнопка №2 - СЛИВ из дорожки в бак
    if ( value2 == LOW ) buttonSlivPressed();

    //Если нажата кнопка №3 - ПРОТИВОТОК
    if ( value3 == LOW ) buttonTokPressed();

    //Если нажата кнопка №4 - СЛИВ ИЗ ДОРОЖКИ В КАНАЛИЗАЦИЮ
    if ( value4 == LOW ) buttonSliv1Pressed();

    //Если нажата кнопка №5 - СЛИВ ИЗ БАКА В КАНАЛИЗАЦИЮ
    if ( value5 == LOW ) buttonSliv2Pressed();

    //Если нажата кнопка №6 - Наполнение бака из водопровода
    if ( value6 == LOW ) {
      buttonZaliv1Pressed();
    }

    //Если нажат концевик №2 в баке
    if ( value7 == LOW ) koncevik2Pressed();

    //Если нажат концевик №1 в баке
    if ( value8 == LOW ) koncevik1Pressed();

    //Если нажата кнопка - " + " СКОРОСТИ МОТОРА
    if ( value9 == LOW ) buttonPlusPressed();

    //Если нажата кнопка - " - " СКОРОСТИ МОТОРА
    if ( value10 == LOW ) buttonMinusPressed();

    //Если нажата кнопка - " + " СКОРОСТИ ПРОТИВОТОКА
    if ( value11 == LOW ) {
      if (statusTok) tone(pinZummer, 698, 50); // сигнал зуммера на нажатие кнопки
      buttonPlusTokPressed();
    }

    //Если нажата кнопка - " - " СКОРОСТИ ПРОТИВОТОКА
    if ( value12 == LOW ) {
      if (statusTok) tone(pinZummer, 698, 50); // сигнал зуммера на нажатие кнопки
      buttonMinusTokPressed();
    }

    //Если нажата кнопка - РЕВЕРС
    if ( value13 == LOW ) buttonReversPressed();
  }
  delay(20);
}

void setSpeedMotor(int value) {
  analogWrite(pinPWMMotor, value); //Команда на изменение ШИМ управления скоростью мотора
}

void MCP41010Write(byte value3) {
  digitalWrite(CS, LOW); // выбор микросхемы
  SPI.transfer(B00010001); // командный байт
  SPI.transfer(255 - value3); // байт данных
  digitalWrite(CS, HIGH); // снимаем выбор с микросхемы
}



void buttonRead () {
  // обновляем статусы кнопок
  Button1.update();
  Button2.update();
  Button3.update();
  Button4.update();
  Button5.update();
  Button6.update();
  Button7.update();
  Button8.update();
  Button9.update();
  Button10.update();
  Button11.update();
  Button12.update();
  Button13.update();
  Button14.update();
  value1 = Button1.read(); // Получаем значение кнопки1
  value2 = Button2.read(); // Получаем значение кнопки2
  value3 = Button3.read(); // Получаем значение кнопки3
  value4 = Button4.read(); // Получаем значение кнопки4
  value5 = Button5.read(); // Получаем значение кнопки5
  value6 = Button6.read(); // Получаем значение кнопки6
  value7 = Button7.read(); // Получаем значение кнопки7
  value8 = Button8.read(); // Получаем значение кнопки8
  value9 = Button9.read(); // Получаем значение кнопки9
  value10 = Button10.read(); // Получаем значение кнопки10
  value11 = Button11.read(); // Получаем значение кнопки11
  value12 = Button12.read(); // Получаем значение кнопки12
  value13 = Button13.read(); // Получаем значение кнопки13
  value14 = Button14.read(); // Получаем значение кнопки14
}

//****************************************************************************
// Обработка нажатия кнопки STOP
void  buttonStopPressed() {
  if (statusMotor) {
    probeg1 = (millis() - timeSpeed1);
    probeg1 = probeg1 * speed10Kmch / 1000;
    probeg1 = probeg1 / 36000;
    timeSpeed1 = millis();
    probeg = probeg + probeg1;
    writeProbeg(probeg);
  }
  speed10Kmch = 0;
  setSpeedMotor(0); // Команда на ШИМ для нулевой скорости мотора
  delay(2);
  digitalWrite(pinMotor, HIGH);
  tone(pinZummer, 698, 1000);
  statusMotor = false; // статус мотора - выключен
  resetProg();
  statusZaliv1 = false;
  statusRevers = false; // статус Revers - выключен
  statusTok = false;// статус Противоток - выключен
  resist = 0; // позиция потенциометра
  digitalWrite(pinLampZaliv, LOW );
  digitalWrite(pinLampSliv, LOW );
  digitalWrite(pinLampSliv1, LOW );
  digitalWrite(pinLampTok, LOW);
  digitalWrite(pinLampRevers, LOW );
  for (int j = 0; j <= 11; j++) digitalWrite(table[0][j], HIGH); // выключаем реле
  //LCD_Print(speed10Kmch, statusMotor); // Вывод статусов на LCD
  setChars1();
  lcd.clear();
  lcd.setCursor(0, 0);    //курсор на первую строчку посередине
  lcd.print("Fit4Pet    v.1.0"); //
  lcd.setCursor(0, 1);
  lcd.print("      CTO\4");
  writeProbeg(probeg);
  do buttonRead (); while ( value14 == LOW);
  delay(1000);
  lcd.setCursor(0, 1);
  lcd.print("                ");

}

//****************************************************************************
//Обработка нажатия кнопки №1 - Залив в дорожку из бака
void buttonZalivPressed() {
  if (statusZaliv) {
    digitalWrite(pinLampZaliv, LOW);
    statusZaliv = false;
    //В цикле выключаем все реле в релейных модулях
    for (int j = 0; j <= 5; j++) digitalWrite(table[0][j], HIGH); // выключаем реле
    for (int j = 7; j <= 10; j++) digitalWrite(table[0][j], HIGH); // выключаем реле
    printText(0);
  }
  else {
    resetProg();
    digitalWrite(pinLampZaliv, HIGH);
    statusZaliv = true;
    //В цикле подаем минус на нужные реле в релейных модулях
    for (int j = 0; j <= 5; j++) digitalWrite(table[0][j], table[1][j]); // включаем нужные реле согласно таблицы с логикой
    for (int j = 7; j <= 10; j++) digitalWrite(table[0][j], table[1][j]); // включаем нужные реле согласно таблицы с логикой
    printText(1);
  }
  do buttonRead (); while ( value1 == LOW);
}

//****************************************************************************
//Обработка нажатия кнопки №2 - Слив из дорожки в бак
void  buttonSlivPressed() {
  if (!statusKoncevik2) {
    if (statusSliv) {
      digitalWrite(pinLampSliv, LOW);
      statusSliv = false;
      timer120s = false;
      if (status80s) {
        timer30s = true;
        time30s = millis();
        timer80s = false;
        status80s = false;
      }
      else {
        digitalWrite(table[0][7], HIGH); // выключаем реле
        digitalWrite(table[0][8], HIGH); // выключаем реле
      }
      //В цикле выключаем все реле в релейных модулях
      for (int j = 0; j <= 5; j++) digitalWrite(table[0][j], HIGH); // выключаем реле
      digitalWrite(table[0][9], HIGH); // выключаем реле
      digitalWrite(table[0][10], HIGH); // выключаем реле

      printText(0);
    }
    else {
      resetProg();
      if (!statusKoncevik1) {
        time120s = millis();
        timer120s = true;
      }
      digitalWrite(pinLampSliv, HIGH);
      statusSliv = true;
      time80s = millis();
      timer80s = true;
      //В цикле подаем минус на нужные реле в релейных модулях
      for (int j = 0; j <= 5; j++) digitalWrite(table[0][j], table[2][j]); // включаем нужные реле согласно таблицы с логикой
      for (int j = 7; j <= 10; j++) digitalWrite(table[0][j], table[2][j]); // включаем нужные реле согласно таблицы с логикой

      printText(2);
    }
  }
  else {
    if (statusSliv1) beep5Times();
    else {
      status2to4 = true;
      buttonSliv1Pressed();
    }
  }
  do buttonRead (); while ( value2 == LOW);
}

//****************************************************************************
//Обработка нажатия кнопки №3 - ПРОТИВОТОК
void buttonTokPressed() {
  if (statusZaliv || statusSliv || statusSliv1 || statusSliv2) beep5Times();
  else {
    if (statusTok) {
      digitalWrite(pinLampTok, LOW);
      //В цикле выключаем все реле в релейных модулях
      for (int j = 0; j <= 5; j++) digitalWrite(table[0][j], HIGH); // выключаем реле
      for (int j = 7; j <= 10; j++) digitalWrite(table[0][j], HIGH); // выключаем реле
      statusTok = false;
      resist = 0;
      MCP41010Write(resist); // команда на изменение сопротивления цифровым потенциометром
      printText(0);
    }
    else {
      resetProg();
      //В цикле подаем минус на нужные реле в релейных модулях
      for (int j = 0; j <= 5; j++) digitalWrite(table[0][j], table[3][j]); // включаем нужные реле согласно таблицы с логикой
      for (int j = 7; j <= 10; j++) digitalWrite(table[0][j], table[3][j]); // включаем нужные реле согласно таблицы с логикой
      digitalWrite(pinLampTok, HIGH);
      statusTok = true;
      resist = 0;
      MCP41010Write(resist); // команда на изменение сопротивления цифровым потенциометром
      printText(3);
    }
  }
  do buttonRead (); while ( value3 == LOW);
}

//****************************************************************************
//Обработка нажатия кнопки №4 - Слив из дорожки в канализацию
void buttonSliv1Pressed() {
  if (statusSliv1) {
    digitalWrite(pinLampSliv1, LOW);
    statusSliv1 = false;
    //В цикле выключаем все реле в релейных модулях
    for (int j = 0; j <= 5; j++) digitalWrite(table[0][j], HIGH); // выключаем реле
    for (int j = 7; j <= 10; j++) digitalWrite(table[0][j], HIGH); // выключаем реле
    printText(0);
  }
  else {
    resetProg();
    digitalWrite(pinLampSliv1, HIGH);
    statusSliv1 = true;
    statusZaliv1 = true;
    //В цикле подаем минус на нужные реле в релейных модулях
    for (int j = 0; j <= 10; j++) digitalWrite(table[0][j], table[4][j]); // включаем нужные реле согласно таблицы с логикой
    if (!statusKoncevik1) digitalWrite(table[0][6], LOW); // включаем нужные реле согласно таблицы с логикой
    if (!statusKoncevik1) digitalWrite(table[0][11], LOW); // включаем нужные реле согласно таблицы с логикой
    if (!status2to4) {
      digitalWrite(table[0][7], LOW); // включаем нужные реле согласно таблицы с логикой
      digitalWrite(table[0][8], LOW); // включаем нужные реле согласно таблицы с логикой
    }
    if (status2to4) {
      timer25s = true;
      digitalWrite(table[0][7], HIGH); // выключаем нужные реле согласно таблицы с логикой
      digitalWrite(table[0][8], HIGH); // выключаем нужные реле согласно таблицы с логикой
      status2to4 = false;
    }
    printText(4);
  }
  do buttonRead (); while ( value4 == LOW);
}

//****************************************************************************
//Обработка нажатия кнопки №5 - Слив из бака в канализацию
void buttonSliv2Pressed() {
  if (statusSliv2) {
    statusSliv2 = false;
    //В цикле выключаем все реле в релейных модулях
    for (int j = 0; j <= 11; j++) digitalWrite(table[0][j], HIGH); // выключаем реле
    printText(0);
  }
  else {
    resetProg();
    statusZaliv1 = false;
    statusSliv2 = true;
    //В цикле подаем минус на нужные реле в релейных модулях
    for (int j = 0; j <= 11; j++) digitalWrite(table[0][j], table[5][j]); // включаем нужные реле согласно таблицы с логикой
    printText(5);
  }
  do buttonRead (); while ( value5 == LOW);
}

//****************************************************************************
//Обработка нажатия кнопки №6 - Наполнение бака из водопровода
void buttonZaliv1Pressed() {
  if (statusKoncevik1) {
    printText(61);
    beep5Times();
    do buttonRead (); while ( value6 == LOW);

    if (statusZaliv || statusSliv || statusTok || statusSliv1 || statusSliv2) delay(1000);
    if (statusZaliv) printText(1);
    if (statusSliv) printText(2);
    if (statusTok) printText(3);
    if (statusSliv1) printText(4);
    if (statusSliv2) printText(5);
  }
  else {
    if (statusZaliv1) {
      statusZaliv1 = false;
      digitalWrite(pinN17, HIGH); // выключаем реле
      digitalWrite(pinN26, HIGH); // выключаем реле

      if (statusZaliv || statusSliv || statusSliv1 || statusTok || statusSliv2) delay(1000); else printText(0);
      if (statusZaliv) printText(1);
      if (statusSliv) printText(2);
      if (statusTok) printText(3);
      if (statusSliv1) printText(4);
      if (statusSliv2) printText(5);

    }
    else {
      if (!statusKoncevik1) {
        //resetProg();
        statusZaliv1 = true;
        digitalWrite(pinN26, LOW); // включаем лампу на щите
        digitalWrite(pinN17, LOW); // включаем реле
        printText(6);
      }
    }
  }
  do buttonRead (); while ( value6 == LOW);
}


//****************************************************************************
//Обработка нажатия концевика №2 в баке
void koncevik2Pressed() {
  if (statusSliv) {
    for (int j = 0; j <= 11; j++) digitalWrite(table[0][j], table[7][j]); // включаем нужные реле согласно таблицы с логикой
    timer25s = true;
    time25s = millis();
    statusSliv = false;
    status2to4 = true;
    digitalWrite(pinLampSliv, LOW); // выключаем индикацию включенного режима Кн.2
    buttonSliv1Pressed();
  }
}

//****************************************************************************
//Обработка нажатия концевика №1 в баке
void koncevik1Pressed() {
  if (statusSliv || statusSliv1 || statusZaliv1) {
    timer120s = false;
    digitalWrite(pinN17, HIGH);
    if (statusZaliv1) {
      statusZaliv1 = false;
      digitalWrite(pinN17, HIGH); // выключаем реле
      digitalWrite(pinN26, HIGH); // выключаем реле
      printText(61);
      beep5Times();
      if (statusSliv || statusSliv1) delay(1000);
      if (statusSliv) printText(2);
      if (statusSliv1) printText(4);
    }
  }
}

//****************************************************************************
//Обработка нажатия кнопки " + " скорости мотора
void buttonPlusPressed() {
  if (not statusMotor) {
    speed10Kmch = 0;
    motorPWM = 0;
    statusMotor = true;
    setSpeedMotor(motorPWM); // задаем 0 ШИМ мотора
    setChars3();
    LCD_Print(speed10Kmch, statusMotor); // Вывод статусов на LCD
    digitalWrite(pinMotor, LOW );
    do buttonRead (); while ( value9 == LOW);
  }
  else {
    speed10Kmch++;
    motorPWM = getPWM(speed10Kmch / koeffSpeed);
    if (motorPWM == 255) {
      motorPWM = 256;
    }
    if (speed10Kmch <= 1) {
      timeSpeed1 = millis();
      setChars4();
    }
    else {
      probeg1 = (millis() - timeSpeed1);
      probeg1 = probeg1 * speed10Kmch / 1000;
      probeg1 = probeg1 / 36000;
      timeSpeed1 = millis();
      probeg = probeg + probeg1;
    }
    if (speed10Kmch > maxSpeed) speed10Kmch = maxSpeed;
    setSpeedMotor(motorPWM); // задаем ШИМ управления скоростью мотора
    //delay(delay1);
    LCD_Print(speed10Kmch, statusMotor); // Вывод статусов на LCD
    timeStamp = millis();
    do {
      if ((millis() - timeStamp) > 500) {
        do {
          speed10Kmch++;
          if (speed10Kmch > maxSpeed) speed10Kmch = maxSpeed;
          motorPWM = getPWM(speed10Kmch / koeffSpeed);
          if (motorPWM == 255) motorPWM = 256;
          probeg1 = (millis() - timeSpeed1);
          probeg1 = probeg1 * speed10Kmch / 1000;
          probeg1 = probeg1 / 36000;
          timeSpeed1 = millis();
          probeg = probeg + probeg1;
          setSpeedMotor(motorPWM); // Вывод статусов на LCD
          LCD_Print(speed10Kmch, statusMotor); // Вывод статусов на LCD
          delay(delay1);
          timeStamp = millis();
          buttonRead ();
        } while ( value9 == LOW);
        if (speed10Kmch <= 0.1) timeSpeed1 = millis();
        else {
          probeg1 = (millis() - timeSpeed1);
          probeg1 = probeg1 * speed10Kmch / 1000;
          probeg1 = probeg1 / 36000;
          timeSpeed1 = millis();
          probeg = probeg + probeg1;
        }
      }
      buttonRead ();
    } while ( value9 == LOW);
  }
}

//****************************************************************************
//Обработка нажатия кнопки " - " скорости мотора
void buttonMinusPressed() {
  speed10Kmch--;
  motorPWM = getPWM(speed10Kmch / koeffSpeed);
  if (motorPWM == 255) motorPWM = 254;
  if (speed10Kmch <= 0) {
    statusMotor = false;
    speed10Kmch = 0;
    motorPWM = 0;
    setSpeedMotor(motorPWM); // задаем ШИМ МОТОРА
    digitalWrite(pinMotor, HIGH );
    setChars3();
    probeg1 = (millis() - timeSpeed1);
    probeg1 = probeg1 * speed10Kmch / 1000;
    probeg1 = probeg1 / 36000;
    timeSpeed1 = millis();
    probeg = probeg + probeg1;
    writeProbeg(probeg);
  }
  setSpeedMotor(motorPWM); // задаем ШИМ МОТОРА
  LCD_Print(speed10Kmch, statusMotor); // Вывод статусов на LCD
  timeStamp = millis();
  do {
    if ((millis() - timeStamp) > 500) {
      do {
        speed10Kmch--;
        motorPWM = getPWM(speed10Kmch / koeffSpeed);
        if (motorPWM == 255) motorPWM = 254;
        if (speed10Kmch <= 0) {
          statusMotor = false;
          speed10Kmch = 0;
          motorPWM = 0;
          setSpeedMotor(motorPWM); // задаем ШИМ МОТОРА
          digitalWrite(pinMotor, HIGH );
          setChars3();
          probeg1 = (millis() - timeSpeed1);
          probeg1 = probeg1 * speed10Kmch / 1000;
          probeg1 = probeg1 / 36000;
          timeSpeed1 = millis();
          probeg = probeg + probeg1;
          writeProbeg(probeg);
        }
        setSpeedMotor(motorPWM); // задаем ШИМ МОТОРА
        LCD_Print(speed10Kmch, statusMotor); // Вывод статусов на LCD
        probeg1 = (millis() - timeSpeed1);
        probeg1 = probeg1 * speed10Kmch / 1000;
        probeg1 = probeg1 / 36000;
        timeSpeed1 = millis();
        probeg = probeg + probeg1;
        buttonRead ();
      } while ( value10 == LOW);
      timeStamp = millis();
    }
    setSpeedMotor(motorPWM); // задаем ШИМ МОТОРА
    buttonRead ();
  } while ( value10 == LOW);
  if (!statusMotor) {
    delay(1000);
    lcd.setCursor(0, 0);
    if (statusRevers) lcd.print("   PEBEPC BK\2.  "); else lcd.print("Fit4Pet    v.1.0"); //
  }

}

//****************************************************************************
//Обработка нажатия кнопки " + " противотока
void  buttonPlusTokPressed() {
  if (statusTok) {
    resist = resist + 5;
    if (resist >= 255) resist = 255;
    MCP41010Write(resist); // команда на изменение сопротивления цифровым потенциометром
    printText(3);
    timeStamp = millis();
    do {
      if ((millis() - timeStamp) > 500) {
        do {
          resist = resist + 5;
          if (resist >= 255) resist = 255;
          if (resist < 255)tone(pinZummer, 698, 10);
          MCP41010Write(resist); // команда на изменение сопротивления цифровым потенциометром
          printText(3);
          buttonRead ();
          delay(100);
        } while ( value11 == LOW);
        timeStamp = millis();
      }
      buttonRead ();
    } while ( value11 == LOW);
    do buttonRead (); while ( value11 == LOW);
  }

  time10s = millis();
  do {
    buttonRead ();
    if ((!statusTok) && (!statusMotor) && (value12 == LOW) && ((millis() - time10s) > timeAccumReset)) {
      printProbeg = true;
      break;
    }
  } while ( (value11 == LOW) && (value12 == LOW));
  if (printProbeg) {
    lcd.setCursor(0, 1);
    lcd.print(probeg); // показываем значение пробега дорожки
    Serial.println(probeg);
    lcd.print("          ");
    printProbeg = false;
  }
  do buttonRead (); while ( value11 == LOW);
}

//****************************************************************************
//Обработка нажатия кнопки " - " противотока
void buttonMinusTokPressed() {
  if (statusTok) {
    resist = resist - 5;
    if (resist <= 0) resist = 0;
    MCP41010Write(resist); // команда на изменение сопротивления цифровым потенциометром
    printText(3);
    timeStamp = millis();
    do {
      if ((millis() - timeStamp) > 500) {
        do {
          resist = resist - 5;
          if (resist <= 0) resist = 0;
          if (resist > 0) tone(pinZummer, 698, 10);
          MCP41010Write(resist); // команда на изменение сопротивления цифровым потенциометром
          printText(3);
          buttonRead ();
          delay(100);
        } while ( value12 == LOW);
        timeStamp = millis();
      }
      buttonRead ();
    } while ( value12 == LOW);
    do buttonRead (); while ( value12 == LOW);
  }
}

//****************************************************************************
//Обработка нажатия кнопки РЕВЕРС
void buttonReversPressed() {
  if (not(speed10Kmch == 0)) {
    probeg1 = (millis() - timeSpeed1);
    probeg1 = probeg1 * speed10Kmch / 1000;
    probeg1 = probeg1 / 36000;
    timeSpeed1 = millis();
    probeg = probeg + probeg1;
    writeProbeg(probeg);
    for (int iter1 = speed10Kmch; iter1 >= 0; iter1--) {
      if (iter1 == 0) setChars3();
      LCD_Print(iter1, statusMotor); // Вывод статусов на LCD
      setSpeedMotor(getPWM(iter1 / koeffSpeed)); // команда на изменение скорости мотора
      delay(50); // Задержка плавного торможения
    }
  }
  statusMotor = false;
  speed10Kmch = 0;
  digitalWrite(pinMotor, HIGH );
  LCD_Print(speed10Kmch, statusMotor); // Вывод статусов на LCD
  delay(1000);
  if (statusRevers) {
    digitalWrite(pinRevers    , HIGH);
    digitalWrite(pinLampRevers    , LOW);
    statusRevers = false;
    setChars3();
    lcd.setCursor(0, 0);
    lcd.print("  PEBEPC B\6K\2.  ");
    delay(1000);
    lcd.setCursor(0, 0);    //курсор на первую строчку посередине
    lcd.print("Fit4Pet    v.1.0"); //
    setChars4();
  }
  else {
    digitalWrite(pinRevers    , LOW);
    digitalWrite(pinLampRevers    , HIGH);
    statusRevers = true;
    lcd.setCursor(0, 0);
    lcd.print("   PEBEPC BK\2.  ");
  }
  do buttonRead(); while ( value13 == LOW);
}

void resetProg() {//обнуляем статусы и гасим лампы при переключении режима
  statusZaliv = false;
  statusSliv = false;
  statusSliv1 = false;
  statusSliv2 = false;
  //statusZaliv1 = false;
  statusTok = false;
  timer120s = false;
  timer25s = false;
  timer30s = false;
  timer80s = false;
  status80s = false;
  digitalWrite(pinLampZaliv, LOW);
  digitalWrite(pinLampSliv, LOW);
  digitalWrite(pinLampSliv1, LOW);
  digitalWrite(pinN25, HIGH);
  digitalWrite(pinLampTok, LOW);
  //  digitalWrite(pinN26, HIGH);
}



void LCD_Print (int resValue, boolean status1) {
  lcd.setCursor(0, 0);
  if (resValue == 0) {
    if (status1) {
      lcd.print("   MOTOP BK\2    ");
    }
    else {
      //setChars1();
      lcd.print("  MOTOP B\6K\2     ");
    }
  }
  else {
    lcd.print("CKOPOCT\6 ");
    if (resValue < 100) lcd.print(" ");
    lcd.print(resValue / 10);
    lcd.print(".");
    lcd.print(resValue % 10);
    lcd.print("km\7");
  }
}
void beep5Times() {
  for (int i = 1; i <= 5; i++) {
    tone(pinZummer, 698, 20);
    delay(100);
  }
}
int getPWM (float speed1) {
  int result;
  if (speed1 <= 1300)                               result = 0.000004 * sq(speed1) + 0.0236 * speed1 + 11, 719;
  if ((speed1 > 1300) && (speed1 <= 3200))          result = 0.0000098 * sq(speed1) + 0.0089 * speed1 + 21, 961;
  if (speed1 > 3200)                                result = 0.00000528 * sq(speed1) + 0.0383 * speed1 - 25, 436;
  if (result < 11) return 0; else  return (result + 1);

}

void writeProbeg(float probeg2) {
  int drobProbeg = (probeg2 * 100);
  drobProbeg = drobProbeg % 100;
  long tempProbeg = probeg2;
  byte hi = highByte(tempProbeg); // старший байт
  byte low = lowByte(tempProbeg); // младший байт
  EEPROM.write(1, hi);  // записываем в ячейку 1 старший байт
  EEPROM.write(2, low); // записываем в ячейку 2 младший байт
  EEPROM.write(3, byte(drobProbeg)); // записываем в ячейку 3 значение сотых км пробега

}

float readProbeg() {
  byte hi = EEPROM.read(1); // считываем 2 байт по адресу ячейки
  byte low = EEPROM.read(2); // считываем 2 байт по адресу ячейки
  long probeg2 = word(hi, low); //
  int probeg3 = EEPROM.read(3); // считываем 2 байт по адресу 3 ячейки
  float probeg4 = probeg3;
  probeg4 = probeg4 / 100;
  probeg4 += probeg2;
  return probeg4;
}

void printFit4Pet() {
  lcd.setCursor(0, 0);    //курсор на первую строчку посередине
  lcd.print("Fit4Pet    v.1.0"); //
}

void printText(int number) {
  if ((number == 1) && ( indexCharSet != 2)) {
    setChars2();
    indexCharSet = 2;
  }
  if ((number != 0) && (number != 1) && ( indexCharSet != 1)) {
    setChars1();
    indexCharSet = 1;
  }
  lcd.setCursor(0, 1);

  if (number == 0) lcd.print("                ");
  if (number == 1) {
    lcd.write(byte(0));
    lcd.print("A\2\5B B \1OPO\3K\4.");
  }
  if (number == 2) lcd.print("  C\2\5B B \1AK...  ");
  if (number == 3) {
    lcd.print("\4POT\5BOTOK     %");
    if (resist == 255) lcd.setCursor(12, 1); else lcd.setCursor(13, 1);
    lcd.print((resist * 100) / 255);
    if (((resist * 100) / 255) < 10)  lcd.print(" ");
  }
  if (number == 4) {
    lcd.print("C\2\5B B KAHA\2\5");
    lcd.write(byte(0));
    lcd.print(". ");
  }
  if (number == 5) {
    lcd.print("C\2\5B \5");
    lcd.write(byte(0));
    lcd.print(" \1AKA... ");
  }
  if (number == 6) lcd.print("HA\4O\2HEH\5E \1AKA.");
  if (number == 61) lcd.print("  \1AK HA\4O\2HEH! ");
}

void setChars1() {
  lcd.createChar(1, bukva_B); // Создаем символ под номером 1
  lcd.createChar(3, bukva_IYI); // Создаем символ под номером 3
  lcd.createChar(4, bukva_P); // Создаем символ под номером 4
}
void setChars2() {
  lcd.createChar(1, bukva_D); // Создаем символ под номером 1
  lcd.createChar(3, bukva_ZH); // Создаем символ под номером 3
  lcd.createChar(4, bukva_Y); // Создаем символ под номером 4
}
void setChars3() {
  lcd.createChar(6, bukva_IYI); // Создаем символ под номером 6
}
void setChars4() {
  lcd.createChar(6, bukva_Mz); // Создаем символ под номером 6
}
void setChars5() {
  lcd.createChar(0, bukva_Z); // Создаем символ под номером 0
  lcd.createChar(1, bukva_G); // Создаем символ под номером 1
  lcd.createChar(2, bukva_B); // Создаем символ под номером 2
  lcd.createChar(3, bukva_Ya); // Создаем символ под номером 3
  lcd.createChar(4, bukva_D); // Создаем символ под номером 4
  lcd.createChar(5, bukva_Y); // Создаем символ под номером 5
  lcd.createChar(6, bukva_I); // Создаем символ под номером 6
  lcd.createChar(7, bukva_L); // Создаем символ под номером 7
}
void playMelody() {
  for (int thisNote = 0; thisNote < 5; thisNote++) {
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(8, melody[thisNote], noteDuration);
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    noTone(8);
  }
}
