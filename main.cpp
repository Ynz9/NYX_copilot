#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "buildTime.h"
#include <SD.h>
#include <STM32RTC.h>
#include "def.h"        // variables, definitions, settings, pinout etc
#include "GyverOLED.h"  //i2c monitor 1-5 128x64
#include "Tachometer.h" //lib for tacho
#include <EncButton.h>  //library for buttons
#include <GyverLBUF.h>  //linear buffer
#include <GyverFIFO.h>
GyverLBUF<uint8_t, 256> rpmBuf; // buffer for storring last ... from rpm functionality
GyverLBUF<float, 256> accXBuf;
GyverLBUF<float, 256> accYBuf;
GyverLBUF<float, 256> accZBuf;
GyverLBUF<uint16_t, 256> accTimeBuf; // in millis;
GyverFIFO<int, 16> bufFIFO;
// тип данных: любой
// размер буфера: код выполняется быстрее при размере буфера, кратном степени двойки (2, 4, 8, 16, 32...)

#include <GyverFilters.h>
// GFilterRA analog0; //RindBuffer for avgsum of last measurments of rpm

/*
#include <> //I2C monitor0 128x128
#include <> //filters
#include <> //FIFO array library
#include <> //for thrmometer dh22
#include <> //eeprom saver library

#include <> //spi falsh ?
*/
STM32RTC &rtc = STM32RTC::getInstance();
const int chipSelect = chipSelect_PIN; // for SPI connection of SD card
File myFile;                           // instanse of LOG file
uint16_t NumberOfSamplesFromAccel = 256;
/* Change these values to set the current initial time */
const byte seconds = 0;
const byte minutes = 0;
const byte hours = 16;

/* Change these values to set the current initial date */
/* Monday 15th June 2015 */
const byte weekDay = 1; // mondaay
const byte day = 15;    // nth day of month
const byte month = 6;
const byte year = 15;   // 20XX
bool InitDone = false;  // flag for first power on
uint32_t mainloopTimer; // loop cycle time
uint32_t betwinSamplesTime;
bool newRoundStarted; // 0 degrees passed
volatile uint32_t loopTimer;
static uint32_t tachoTmr;
uint8_t MODE = 0;                  // 0 only collect, store, calculate data/ 1 0+make decision and make adjustments
uint32_t CyclesTimesArray[200];    // array of times from one Cmark() to another. Use it for calculation of avg rpm.
volatile uint64_t RevolutionsDone; // quantity of revilutions so far
uint32_t softTimer1 = 1000;        // use it for periodical event1
GyverOLED<SSH1106_128x64> oled_1;
Adafruit_MPU6050 mpu;
// GyverOLED<SSH1106_128x64> oled_2; // oled
// GyverOLED<SSH1106_128x64> oled_3; // oled
// GyverOLED<SSH1106_128x64> oled_4; // oled
// GyverOLED<SSH1106_128x64> oled_5; // oled
Tachometer tacho;             // create tacho object
Button bDEV(devboardUserKEY); // this is dev board button
// Button b0(button1pin);
// Button b1(button2pin);
// Button b2(button3pin);
// VirtButton b3;

void displaySensorDetails(uint8_t mode) // 1-serial, 0-oled
{
  /*sensor_t sensor;
  accel.getSensor(&sensor);
  if(mode==0)
  {Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");
  Serial.println("------------------------------------");
  Serial.println("");}
   else
    {sensors_event_t event;
  accel.getEvent(&event);
  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");

}
  */
}

int getMpuReadings(bool aX, bool aY, bool aZ)
{
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
volatile uint16_t sampleNumber;
 volatile uint16_t circleTimer;
  /* Print out the values */

  if (aX)
    accXBuf.write(a.acceleration.x);

  if (aY)
    accYBuf.write(a.acceleration.y);

  if (aZ)
    accZBuf.write(a.acceleration.z);

  /*
  Serial.println(" m/s^2");
  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");
  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");
  Serial.println("");
  */
  if (sampleNumber >= NumberOfSamplesFromAccel)
  {
    sampleNumber = 0;
    circleTimer=0;
    return 0;
  }
  sampleNumber++;
  return 0;
}
void interrupt_Tacho()
{
  newRoundStarted = 1; // set flag that tis is 0 degrees mark
  tacho.tick();        // сообщаем библиотеке об этом
  mainloopTimer = millis();

  RevolutionsDone++;
}

int writeLogToSD(String logstring) // see SD examples for extended details of usage
{
  if (!SD.begin(chipSelect_PIN))
    return 1;

  myFile = SD.open("LOG_File.txt", FILE_WRITE);
  Serial.println("initialization done.");
  if (myFile)
  {
    myFile.printf("%02d:%02d:%02d.%03d\n", rtc.getHours(), rtc.getMinutes(), rtc.getSeconds(), rtc.getSubSeconds()); // I realy dont know why it works only this way
    myFile.println(logstring);
    myFile.close();
  }
  return 0; // all done. Exit
}

void btn_isr()
{
  bDEV.tick();
  // if (bDEV.press())
  //   digitalWrite(devboardLED, !digitalRead(devboardLED));
  if (bDEV.press())
  {
    if (MODE == 0)
      MODE = 1;
    else
      MODE = 0;
    Serial.println("MODE changed to " + (char)MODE);
  }
  // b0.tick();
  // b1.tick();
  // b2.tick();
  //  b2.tick(b0, b1);
}
void setup()
{

  Serial.begin(115200); // serial interface

  Serial.println("Setup begin...");
  // Try to initialize MPU6050
  if (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  rtc.begin();         // initialize RTC 24H format
  tacho.setWindow(10); // установка количества тиков для счёта времени (по умолч 10)
  // tacho.setTimeout(2000); // таймаут прерываний (мс), после которого считается что вращение прекратилось
  /* Set the time
rtc.setHours(hours);
rtc.setMinutes(minutes);
rtc.setSeconds(seconds);
*/
  // Set the date
  // rtc.setWeekDay(weekDay);
  rtc.setDay(BUILD_DAY);
  rtc.setMonth(BUILD_MONTH);
  rtc.setYear(BUILD_YEAR);

  // you can use also
  rtc.setTime(BUILD_HOUR, BUILD_MIN, BUILD_SEC);
  // rtc.setDate(weekDay, day, month, year);
  oled_1.init();
  oled_1.clear();

  oled_1.setCursor(0, 0);

  oled_1.setScale(2);
  oled_1.print("SETUP...");
  oled_1.update();

  /*
  pinMode(20, INPUT_PULLUP);
  pinMode(21, INPUT_PULLUP);
  pinMode(22, INPUT_PULLUP);
  pinMode(23, INPUT_PULLUP);
  */
  pinMode(PA5, INPUT);
  pinMode(devboardLED, OUTPUT);
  pinMode(TACH_PIN, INPUT_PULLUP);        // пин тахометра подтягиваем к VCC
  pinMode(devboardUserKEY, INPUT_PULLUP); // use it instead of real tacho
  digitalWrite(devboardLED, 0);           // turn it OFF at startup
  /*
  pinMode(); // analog in port 1
  pinMode(); // analog in port 2
  pinMode(); // analog in port 3
  pinMode(); // analog in port 4
  pinMode(); // analog in port 5
  pinMode(); // analog in port 1
  pinMode();
  pinMode();
  pinMode();
  pinMode();
  */
  attachInterrupt(TACH_PIN, interrupt_Tacho, FALLING); // tacho, not used, Use button instead
  // attachInterrupt(devboardUserKEY, btn_isr, FALLING);  // does not work

  // SerialUSB.begin();

  /*
   oled_2.init();
   oled_2.clear();
   oled_3.init();
   oled_3.clear();
   oled_4.init();
   oled_4.clear();
   oled_5.init();
   oled_5.clear();

  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");

  }
  Serial.println("ADXL345 detected");
  */
}
int analitycs()
{ // figure out amplitude etc
}

void interrupt_readRPMcycle()
{
  RevolutionsDone++;                                    // add 1 to counter
  volatile uint i;                                      // variable for array indexing
  i = RevolutionsDone / numberOfVibrationSensorSamples; // get leftover TBD: Replace with bitShift division
  if (i >= cyclesArraySize)
  {
    i = 0;
  } // TBD: replace it with FILO array
  uint32_t entryTime = micros();  // mark entry time
  volatile uint32_t previousTime; // this is time of prev cycle called
  i = RevolutionsDone;            // leftover from (all cycles quantity)/(array length)
  CyclesTimesArray[i] = entryTime - previousTime;
  volatile uint32_t currentCycleTime = entryTime - previousTime; // calculate time delta from prev time to current
  // array store oneRpm
  previousTime = entryTime;
  rpmBuf.write(currentCycleTime);
}

int calculateAverageRPM()
{               // transfer here pointer to cycles array
  uint avgTime; // calculate avg value from array
  uint avgRPMArraySum;
  for (int i; i = 0; i <= 20)
  {
    // avgRPMArraySum = avgRPMArray[i];
  }

  int averageRPM = 60 * 1000000 / avgTime; // is it so ? rly?
  return averageRPM;
  for (uint i; i >= cyclesArraySize; i++)
  {
    rpmBuf.read(i);
  }
}

void sDisplay(int data_1, int data_2, int data_3, int data_4, int data_5) // this is for output on small displays
{
  oled_1.clear();
  oled_1.setCursor(0, 2);
  oled_1.setScale(1);
  oled_1.print("RPM ");
  oled_1.print(data_1);
  oled_1.print("  ");
  oled_1.print(tacho.getUs());
  oled_1.print(" uS");
  oled_1.setCursor(0, 1);
  oled_1.println(analogRead(PA5));
  oled_1.setCursor(0, 0);
  oled_1.printf("%02d:%02d:%02d.%03d\n", rtc.getHours(), rtc.getMinutes(), rtc.getSeconds(), rtc.getSubSeconds()); // I realy dont know why it works only this way
  oled_1.update();
  /*
  oled_2.setCursor(0, 0);
  oled_2.setScale(2);
  oled_2.print("inactive ");
  oled_2.print(data_2);
  oled_3.setCursor(0, 0);
  oled_3.setScale(2);
  oled_3.print("inactive ");
  oled_3.print(data_3);
  oled_4.setCursor(0, 0);
  oled_4.setScale(2);
  oled_4.print("inactive ");
  oled_4.print(data_4);
  oled_5.setCursor(0, 0);
  oled_5.setScale(2);
  oled_5.print("inactive ");
  oled_5.print(data_5);
  */
}

void bigDisplay() // work with main upper display
{
}

void testDisplay()
{ // not used
  oled_1.init();
  oled_1.clear();

  while (1)
  {
    oled_1.println(1);
    oled_1.update();
    digitalWrite(devboardLED, (!digitalRead(devboardLED)));
    delay(1000);
  }
}

int firstStart() // initialisation of all perepherals, report
{
  uint8_t result;
  // Get settings from memory
  // set time

  // ACCELEROMETER setup section
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth())
  {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  switch (mpu.getAccelerometerRange())
  {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  if (!mpu.begin())
    return 1;
  if (!Serial)
    return 1; //
  InitDone = true;
  return 0; // 0 - everything is ok
}

int serialExchange()
{
  if (Serial.available()) // tbd : serial data exchange in uint to make it short. Or use -1 to
  // mode,rpm,vibroAmp,upLimRpm,dwnLimRpm
  //
  { // lets try plaing with mode setting from serial interface
    String serialInputString = Serial.readString();
    if (serialInputString == "1")
      MODE = 1;
    if (serialInputString == "0")
      MODE = 0;
  }
}

void loop()
{
  if (!InitDone)
    firstStart(); // check if MCU just booted. Go to setup
  // getMpuReadings(1,0,0);
  loopTimer = micros(); // let's count time betwin some points in code
  bDEV.tick();


  if (tacho.getRPM() && newRoundStarted) // 
  {
    betwinSamplesTime = tacho.getUs() / NumberOfSamplesFromAccel;
    newRoundStarted = 0; // keep this setting for next round
  }
  if (millis() - tachoTmr > betwinSamplesTime&&!tacho.getRPM())
  {
    tachoTmr = millis();
    getMpuReadings(1, 1, 1);
  }

  sDisplay(tacho.getRPM(), 2, 3, 4, 5); // display info on small monitorsParams: data to display

  static uint32_t TIMled;
  if (millis() - TIMled > softTimer1)
  {
    TIMled = millis();
    oled_1.update();
    digitalWrite(PC13, !digitalRead(PC13));
    Serial.print(accXBuf.read(25)); Serial.print(accXBuf.size());
  }

  uint16_t looptime = micros() - loopTimer;
}
