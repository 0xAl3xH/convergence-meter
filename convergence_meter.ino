#include <ArduinoUnit.h>
#include <DS3232RTC.h>
#include <Time.h>
#include <Wire.h>
#include <TimerOne.h>

#define BRIGHTNESS_LEVELS 64
//conversion into 254 steps so we can compare to counter easily
#define TICKER_STEP 254/BRIGHTNESS_LEVELS
#define TIMER_DELAY 280    //in microseconds

//Counter for our interrupt process, used to determine output to BCD
int ticker = 0;

//Arduino pins D4,D5,D6,D7 for lowest to highest bit of the K155ID1 input, needs to be part of port D
byte bcd0 = PD4;
byte bcd1 = PD5;
byte bcd2 = PD6;
byte bcd3 = PD7;

byte debugPin = PB0;

byte brightness;
int previousSec, currentSec;

boolean flashConvergence;

//Make a time element of the date Rebecca and I met (after next house dinner at the swings)
tmElements_t tmet = {0, 30, 18, 4, 28, 8, 2013 - 1970}; //second, minute, hour, wday (day of week sunday is day 1), day, month, year (offset from 1970)
time_t dateMet = makeTime(tmet);

//Helper function to setup output ports to drive the K155ID1, has to be port D 
void setupPins() {
  DDRD = (1 << bcd0) | (1 << bcd1) | (1 << bcd2) | (1 << bcd3);
  DDRB = (1 << debugPin);
}

//Interrupt routine that PWMs the BCD to make nixie tubes appear as if they're fading
void timeDisplayInterrupt() {
  ticker++;

  //Debug interrupt frequency
  PORTB = ~(PORTB >> debugPin) << debugPin;
  
  if (ticker > BRIGHTNESS_LEVELS) {
    ticker = 0;
  }

  int currentTick = ticker * TICKER_STEP;

  if (currentTick >= brightness) {
    outBCD(previousSec);
  } else {
    outBCD(currentSec);
  }
}

// Helper function to set appropriate 4 bit output to BCD
// if second argument is invalid e.g >60, the BCD will disable
// nixie tube output
void outBCD(int sec) {
  byte pinVals;
  //Force invalid input to turn tube off
  if (sec > 60) {
    pinVals = 10;
  } else {
    //Nixie tube can display 0-9, so mod 10 to prevent overflow
    pinVals = sec % 10;
  }

  PORTD = (((pinVals & B0001) == B0001) << bcd0) | (((pinVals & B0010) == B0010) << bcd1) |
          (((pinVals & B0100) == B0100) << bcd2) | (((pinVals & B1000) == B1000) << bcd3);
}

//Calculate the difference between the two time_t elements
uint32_t dayDifference (time_t timeInit, time_t timeNow) {
  return (uint32_t) (timeNow - timeInit);
}

void setup() {
  setupPins();

  //Debug
  pinMode(13, OUTPUT);


  //Setup random number generator
  randomSeed(analogRead(A0));
  pinMode(2, INPUT);
  pinMode(A2, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), displayConvergence, FALLING);
  flashConvergence = false;

  //Start the timer interrupt
  Timer1.initialize(TIMER_DELAY);
  Timer1.attachInterrupt(timeDisplayInterrupt);

  Serial.begin(9600);
  Serial.println("Serial started");

  // For wire (I2C) on the Uno: A4 (SDA), A5 (SCL)
 
  setSyncProvider(RTC.get);
  if (timeStatus() != timeSet)
    Serial.println("Unable to sync with the RTC");
  else
    Serial.println("RTC has set the system time");
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(' ');
  Serial.print(day());
  Serial.print(' ');
  Serial.print(month());
  Serial.print(' ');
  Serial.print(year());
  Serial.println();
  uint32_t daysPassed = dayDifference(dateMet, now()) / 86400;
  Serial.print("Days since meeting Rebecca: ");
  Serial.println(daysPassed);

  currentSec = 61;  //start off blank
  //brightness = 0;
  Serial.println(currentSec);
  Serial.println("Starting loop");
}

void loop() {
  if (currentSec != second() || flashConvergence) {

    //upon detecting that the second has changed, fade to new second
    //unsigned long currentMil = millis();
    if (flashConvergence) {
      digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(1000);              // wait for a second
      digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
      delay(1000);              // wait for a second

      fadeClear();  //clear display
      randomFlash();  //cool lightshow
      uint32_t daysPassed = dayDifference(dateMet, now()) / 86400;  //compute # of days since meeting Rebecca

      //Assume Unix time will end by 2106 using 32 bit unsigned int as datatype, maximum # days can only be in the
      //ten thousands range, extract that and display it sequentially
      if (int days_5 = daysPassed / 10000) {
        daysPassed %= 10000;
        crossFade(days_5);
        fadeClear(0.25);
      }
      if (int days_4 = daysPassed / 1000) {
        daysPassed %= 1000;
        crossFade(days_4);
        fadeClear(0.25);
      }
      //No need for check next because at the time of writing, I've already known rebecca for more than 100 days
      crossFade(daysPassed / 100);
      fadeClear(0.25);
      daysPassed %= 100;
      crossFade(daysPassed / 10);
      fadeClear(0.25);
      crossFade(daysPassed % 10);

      fadeClear();  //clear display
      delay(1500);  //delay for emphasis
      flashConvergence = false;   //allow for
    } else {
      crossFade(second());
    }
    //Serial.println(millis()-currentMil);
  }

  //previousSec = 61;
  //currentSec = 7;
  //Serial.println(brightness);
  //if (Serial.available()) {
  //  brightness = Serial.readString().toInt() % 255;
  //;}

  //Serial.println(analogRead(A0));
  //brightness = (analogRead(A0)/4)%255;

  //Test::run();
}

//Function that takes a value to fade to and cross fade between the previous number
//and next number within 0.5 seconds
void crossFade(int sec) {
  previousSec = currentSec;
  currentSec = sec;
  for (int i = 0; i <= 255; i += 5) {
    brightness = i;
    //Take ~0.5 seconds to complete loop
    delayMicroseconds(10000);
  }
}

//fade value within duration specified duration (max is 0.835533 seconds)
void crossFade (int sec, float duration) {
  previousSec = currentSec;
  currentSec = sec;
  int timeDelay = (duration * 1000000.0) / 51.0;
  for (int i = 0; i <= 255; i += 5) {
    brightness = i;
    //Take ~0.5 seconds to complete loop
    delayMicroseconds(timeDelay);
  }
}

//Function that fades the display to clear it by fading between previous number and no light on
void fadeClear() {
  crossFade(61);
}

void fadeClear(float duration) {
  crossFade(61, duration);
}

//TODO function that randomizes digits as it fades
void randomFlash() {
  previousSec = 61;
  for (int i = 0; i <= 255; i += 5) {
    brightness = i;
    currentSec = random(9);
    delay(25);
  }
  for (int i = 0; i <= 255; i += 5) {
    brightness = 255 - i;
    currentSec = random(9);
    delay(25);
  }
  //Reset seconds to be empty
  currentSec = 61;
  delay(1000);
}

//Interrupt function to know when to display days since meeting
void displayConvergence() {
  if (!flashConvergence) {
    flashConvergence = true;
  }
}

void printDigits(int digits)
{
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(':');
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

test(outBCD)
{
  for (int i = 0; i < 10; i++) {
    byte pinVals = i % 10;
    outBCD(i);
    //**NOTE** Assuming that data out pins are consecutive and start at PD5, change bitshift accordingly
    byte correctVal = i << 4;
    Serial.println(pinVals);
    assertEqual(correctVal, PORTD);
  }
}

