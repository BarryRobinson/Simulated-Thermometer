/* Bluetooth_sim_thermometer.ino
   This program uses an Arduino Pro Mini (3.3v), a HC-05 Bluetooth
   transceiver and a 1.44" TFT colour display to simulate a digital
   thermometer. The simulated reading can be changed at anytime
   during the scenario over the Bluetooth link. The device reacts
   like a real device would and sends back the reading and time to
   the controlling app.
*/

/* Bluetooth_sim_thermometer.ino
   Copyright (C) 2018 Barry M. Robinson, Toronto, Ontario, Canada

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <TFT_ILI9163C.h>

// Colour definitions for the TFT screen
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
#define GRAY    0x38E7

/* AT_MODE Arduino pin 9 is wired to Pin 34 on the HC-05
   Bluetooth module. AFTER the HC-05 is powereed on Pin 9
   goes low for one second then goes high to force the HC-05
   into the AT command mode. Pin 9 does not need to stay
   high to keep the HC-05 in the AT command mode. Exit the
   AT mode by sending a AT+RESET to the HC-05.

   ATNME The name string is the public name for the
   Bluetooth device. It can be changed as necessary.
   AT command strings must end with "\r\n"

   ATSPD The speed string for the Bluetooth device

   ATRST The reset string for the Bluetooth device

   MILLIS is the number of milliseconds in a second. 1000UL
   
   KA_DELAY is the keep alive delay that prevents the Bluetooth
   connection from timing out. Set as an unsigned long integer UL
*/

#define AT_MODE 9
#define ATNME "AT+NAME=Sim1234\r\n" //name string
#define ATSPD "AT+UART=38400,1,0\r\n" //speed string
#define ATRST "AT+RESET\r\n" //reset string
#define MILLIS 1000UL
#define KA_DELAY 10000UL;

/* Used hardware SPI on the Arduino Pro Mini
   pin 13 SCK is LCD serial clock (Clk)
   pin 11 MOSI is LCD (Din)
   pin 5 - Data/Command select (DC)
   pin 7 - LCD chip select/enable (CS)
   pin 6 - LCD reset (RST)
   Pins left to right, from the top
   VCC GND CS=7 RST=6 A0=5 SDA=11 SCK=13 LED
   The LED is supplied with 5.3vdc through
   a 220 ohm resistor.
*/

#define __CS 7
#define __DC 5
#define _RST 6

#define SWCH 8  // general purpose switch on digital pin 8
#define HALL 1  // hall effect sensor analog input 1
#define RHAL 50 // range above/below which sensor is true

/*
   I am using a BeStar F-B-P3009EPB-01 LF Piezo Sounder
   as the sound output device. It has an operating voltage
   of 1-30v point-to-point, a rated voltage of 9v square wave
   a rated current of 5mA and a resonant frequency of 2500Hz.
   Pin 3 is connected to the positive (+) pin of the sounder
   and the negative pin is connected to ground.
   The tone generator uses Timer 2 (the third of 3 timers on
   Arduino. It will not interfer with the millis() function 
   which uses Timer 0
 */
 
#define TPIN 3    // digital pin for tone generation
#define TTON 2500 // tone in hertz
#define TOND 250  // tone duration in milliseconds

#define TBUF 10   // time buffer length

TFT_ILI9163C tft = TFT_ILI9163C(__CS, __DC, _RST);

boolean celsius = true;  // temperature scale
boolean actionT = false; // time that an action can be performed
boolean displyT = false; // a reading is being displayed
boolean manualT = false; // take a manual reading

float currentTemp = 36.8;

unsigned long cTime = 0UL;
unsigned long aTime = 0UL;
unsigned long dTime = 0UL;
unsigned long alTme = 0UL;
unsigned long corre = 0UL; // correction factor
unsigned long keep_alive = 0UL;

char tBuff[TBUF]; // time buffer

uint16_t colr = BLACK;

int highHall;
int lowHall;
int hallRead = 512;

int currentAction = 15;  // seconds to perform action
int currentDisplay = 20; // seconds to display results

void upDate(char fc, String numb);
void enterATcmd(char *cmdSrng);
void zeroSensor(int *hHall, int *lHall);
void displayTemp(float cTmp, boolean cf);
void displayWait(uint16_t colour, boolean cf);
char *toMinSec(unsigned long millsT, unsigned long correction,
      char *tbf, int bufflen);

void setup() {
  pinMode(SWCH,INPUT_PULLUP); // general purpose MC switch
  pinMode(AT_MODE, OUTPUT);   // pin 9 to HC-05 pin 34 (KEY)
  tft.begin();
  tft.setRotation(2);
  tft.clearScreen(); // BLACK by default
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  zeroSensor(&highHall, &lowHall);
  tft.setCursor(0, 40);
  tft.println("Setup Bluetooth");
  digitalWrite(AT_MODE, LOW);  // start low
  digitalWrite(AT_MODE, HIGH); // set HC-05 to AT mode
  Serial.begin(38400); // set for AT mode rate
  delay(2000); // wait for two seconds to enter AT mode
  enterATcmd(ATNME);   // set name
  delay(1000);
  enterATcmd(ATSPD);   // set speed 38400
  delay(1000);
  digitalWrite(AT_MODE, LOW); // prepare to
  enterATcmd(ATRST);  // leave command mode
  tft.println("Setup complete");
  tone(TPIN,TTON,TOND); // sound tone to indicate success
  delay(1000);
  tft.clearScreen();
  tft.setTextSize(3);
  Serial.flush();
}

void loop() {
  cTime = millis();

  if (cTime > keep_alive){
    Serial.print('\0'); // send NULL character
    keep_alive = cTime + KA_DELAY;
  }

  if (actionT && (aTime <= cTime)){ //action state timed out
      actionT = false;
      tft.clearScreen();
      colr = BLACK;
  }

  if (displyT && (dTime <= cTime)){ // display timed out
     displyT = false;
     tft.clearScreen();
     colr = BLACK;
  }
  
  if (digitalRead(SWCH) == LOW){ // switch detected, action set
    delay(100); // switch pressed, debounce?
    if (displyT || actionT){ // display is on, so reset
       displyT = false;
       actionT = false;
       tft.clearScreen();
       colr = BLACK;
    }
     else if (!actionT){ // set action state true
      actionT = true;
      displyT = false;
      tft.clearScreen();
      colr = BLACK;
      aTime = cTime + (MILLIS * (unsigned long)currentAction);
      cTime = millis(); // update cTime
    }   
  }

  if (actionT && alTme <= cTime){ // wait for reading, action set
   if (colr == BLACK) colr = GRAY;
   else colr = BLACK;
   displayWait(colr, celsius);
   alTme = cTime + 500UL; // blink 500 milliseconds
  }
  
  if (actionT){
      hallRead = analogRead(HALL);
       if (manualT || hallRead > highHall || hallRead < lowHall){
        actionT = false; // action taken
        manualT = false; // reset
        displayTemp(currentTemp, celsius);
        dTime = cTime + (MILLIS * (unsigned long)currentDisplay);
        displyT = true;
        Serial.print("Temperature read: ");
        Serial.print(currentTemp, 1);
        if (celsius) Serial.print("C at ");
        else Serial.print("F at "); 
        Serial.println(toMinSec(cTime, corre, tBuff, TBUF));
        tone(TPIN,TTON,TOND); // sound tone to indicate success
       }
  }
}
// end of main loop

void serialEvent() {
  char numChar;
  char inBuff[8];
  int n = 0;
  char firstChar = (char)Serial.read();
  delay(10);
  while (Serial.available()) {
    numChar = (char)Serial.read();
    // throw away everything except numbers and decimal point
    if (isDigit(numChar) || numChar == '.') inBuff[n++] = numChar;
  }
  inBuff[n] = '\0'; // terminate string
  upDate(firstChar, inBuff);
  Serial.flush();
}

void upDate(char fc, char *numb) {
  switch (fc) {
    case 'C':    // set scale to Celsius
    case 'c':
      celsius = true;
      Serial.println("Temperature scale Celsius");
      break;
    case 'F':    // set scale to Farenheit
    case 'f':
      celsius = false;
      Serial.println("Temperature scale Farenheit");
      break;
    case 'R':  // reset timer to zero
    case 'r':
    corre = millis(); // correction factor set
    Serial.println("Zero timer now");
    break;
    case 'G':  // give the learner the reading
    case 'g':  // really easy
    aTime = millis() + 500UL; // force actionT
    actionT = true;  // to stay true for 500ms
    manualT = true;
    Serial.print("Given reading at ");
    Serial.println(toMinSec(millis(), corre, tBuff, TBUF));
    break;  
    case 'M':  // manually trigger reading
    case 'm':  // use for standardized patients
    if (actionT){
    manualT = true;
    Serial.print("Manual reading at ");
    Serial.println(toMinSec(millis(), corre, tBuff, TBUF));
    }
    else{
    Serial.println("Manual reading failed!");
    Serial.print("Thermometer is OFF at: ");
    Serial.println(toMinSec(millis(), corre, tBuff, TBUF));
    }
    break;
    case 'D':    // set display interval
    case 'd':
      currentDisplay = atoi(numb);
      Serial.print("Set display: ");
      Serial.println(currentDisplay);
      break;
      case 'A':    // set action interval
      case 'a':
      currentAction = atoi(numb);
      Serial.print("Set action: ");
      Serial.println(currentAction);
      break;
    case 'S':     // set temperature
    case 's':
      currentTemp = atof(numb);
      if (currentTemp < 1.0) currentTemp = 37.0;
      Serial.print("Set temperature:");
      Serial.print(currentTemp, 1); // to 1 decimal
      if (celsius) Serial.println("C");
      else Serial.println("F");
      break; 
    default:
      break;
  }
}

void enterATcmd(char *cmdSrng) {
  // AT command strings must end with "\r\n"
  char iBuff[8];
  int n = 0;
  tft.print(cmdSrng);
  Serial.print(cmdSrng);
  delay(5000);
  while (Serial.available())
    iBuff[n++] = (char)Serial.read();
  Serial.flush();
  iBuff[n] = '\0';
  tft.println(iBuff);
}

void zeroSensor(int *hHall, int *lHall){
   int sens = 0;
  tft.setCursor(22, 40);
  tft.println("Zero Sensor");
  tft.println(" ");
  tft.println("This takes 5 seconds");
  tft.println("Keep magnetic objects");
  tft.println("away from the sensor");
  tft.println("Press button to begin");
  while (digitalRead(SWCH) == HIGH); // wait
  // when switch goes LOW continue
  tft.clearScreen();
  tft.setCursor(0, 40);
   tft.println("Zeroing");
  for (int i=0; i < 5; i++){
    sens = sens + analogRead(HALL);
    tft.print(" .");    
    delay(1000);
  }
  tft.println("\r\nComplete");
  sens = sens/5; // average sensor readings
  *hHall = sens + RHAL;
  *lHall = sens - RHAL;
  delay(1000);
  tft.clearScreen();
}

void displayTemp(float cTmp, boolean cf){
    tft.clearScreen();
    tft.setTextColor(WHITE);
    tft.setCursor(19, 77);
    tft.print(cTmp, 1);
    tft.setTextColor(BLUE);
    
    /* 
       since the degree symbol is not
       available, use a lowercase 'o'
       from a smaller font. Shift
       upwards by resetting the cursor.
     */
     
    tft.setTextSize(2);
    tft.setCursor(65,100);
    tft.print("o");
    tft.setTextSize(3);
    tft.setCursor(80,104);
    if (cf) tft.print("C");
    else tft.print("F");
    // set to default of white
    tft.setTextColor(WHITE);
}

void displayWait(uint16_t colour, boolean cf){
   tft.setCursor(19, 77);
   tft.setTextColor(colour);
   if (cf) tft.print("__._");
   else tft.print("___._");
}

char *toMinSec(unsigned long millsT, unsigned long correction,
      char *tbf, int bufflen){
  // clear the buffer      
  for (int n = 0; n < bufflen; n++) tbf[n] = '\0';
  // convert milliseconds into hours:minutes:seconds
  unsigned long sec = (millsT - correction)/MILLIS;
  int scs = (int) sec % 60UL;
  int mns = (int) sec/60UL;
  int hrs = mns/60;
  mns = mns % 60;
  sprintf(tbf, "%d:%02d:%02d", hrs, mns, scs);
  return tbf;
}
