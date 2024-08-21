
#define numberOfVibrationSensorSamples 20 //360 divided by this value IN CASE @NO ENCODER@
#define encoderByRevolutionCounts 10
#define cyclesArraySize 200 //how many samples to use to calculate RPM
#define ARRAY_SIZE 10 // wtf
#define button1pin 99 // buttons input
#define button2pin 99
#define button3pin 99
#define button4pin 99
#define TACH_PIN PB9
#define chipSelect_PIN 99
#define devboardUserKEY PA0 // physical button on devboard
#define devboardLED PC13    // led on devboard
#define port1AnalogIn 99
#define port2AnalogIn 99
#define port3AnalogIn 99
#define port4AnalogIn 99
#define port5AnalogIn 99 // not used

#define port1DigitalIn 99
#define port2DigitalIn 99
#define port3Digitalln 99
#define port4DigitalIn 99
#define port5DigitalIn 99 // not used

#define ConnectionPortPinRx 99
#define ConnectionPortPinTx 99

/*#define MY_PERIOD 500  // период в мс
uint32_t tmr1;         // переменная таймера
void setup() {}
void loop() {
  if (millis() - tmr1 >= MY_PERIOD) {   // ищем разницу
    tmr1 += MY_PERIOD;                  // сброс таймера
    // выполнить действие
  }
}*/