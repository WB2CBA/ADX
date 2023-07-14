//*********************************************************************************************************
//******************* ADX  - ARDUINO DIGITAL MODES HF TRANSCEIVER WITH CAT CONTROL  ***********************
//********************************* Write up start: 02/01/2022 ********************************************
// FW VERSION: ADX_CAT_V1.6 - Version release date: 07/14/2023
// Barb(Barbaros ASUROGLU) - WB2CBA - 2022
//  Version History
//  V1.0  Modified Calibration EEPROM to protect R/W cycle of EEPROM.
//  V1.1 10m/28Mhz band support added.
//  V1.2 CAT Control and JE1RAV improved FSK TX code fixing less than 500 Hz TX error correction added.
//  V1.4 Serial timeout delay change.
// Joerg Frede - DK3JF - 2023
//  V1.5 Cleanup Code
//  V1.6 Band change display Bug correction - Thanks to KD5RXT and LA7WRA for reporting this isuue.
//*********************************************************************************************************
// Required Libraries
// ----------------------------------------------------------------------------------------------------------------------
// Etherkit Si5351 (Needs to be installed via Library Manager to arduino ide) - SI5351 Library by Jason Mildrum (NT7S) - https://github.com/etherkit/Si5351Arduino
// Arduino "Wire.h" I2C library(built-into arduino ide)
// Arduino "EEPROM.h" EEPROM Library(built-into arduino ide)
//*************************************[ LICENCE and CREDITS ]*********************************************
//  Initial FSK TX Signal Generation code by: Burkhard Kainka(DK7JD) - http://elektronik-labor.de/HF/SDRtxFSK2.html
//  SI5351 Library by Jason Mildrum (NT7S) - https://github.com/etherkit/Si5351Arduino
//  ADX CAT code inspired from Lajos Höss, HA8HL TS2000 CAT implementation for Arduino.
//  Improved FSK TX  signal generation code is from JE1RAV https://github.com/je1rav/QP-7C
//
// License
// -------
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject
// to the following conditions:
//
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
// ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
// CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

//*****************[ SI5351 VFO CALIBRATION PROCEDURE ]****************************************
// For SI5351 VFO Calibration Procedure follow these steps:
// 1 - Connect CAL test point and GND test point on ADX PCB to a Frequency meter or Scope that can measure 1 Mhz up to 1Hz accurately.
// 2 - Press SW2 / --->(CAL) pushbutton and hold.
// 4 - Power up with 12V or with 5V by using arduino USB socket while still pressing SW2 / --->(CAL) pushbutton.
// 5 - FT8 and WSPR LEDs will flash 3 times and stay lit. Now Release SW2 / --->(CAL). Now Calibration mode is active.
// 6 - Using SW1(<---) and SW2(--->) pushbuttons change the Calibration frequency.
// 7 - Try to read 1 Mhz = 1000000 Hz exact on  Frequency counter or Oscilloscope.
//     The waveform is Square wave so freqency calculation can be performed esaily.
// 8 - If you read as accurate as possible 1000000 Hz then calibration is done.
// 9 - Now we must save this calibration value to EEPROM location.
//     In order to save calibration value, press TX button briefly. TX LED will flash 3 times which indicates that Calibration value is saved.
//
//********************************[ CAT CONTROL SETTINGS and CAT Functionality ]********************
// CAT CONTROL RIG EMULATION: KENWOOD TS2000
// SERIAL PORT SETTINGS: 9600 baud,8 bit,1 stop bit
// When CAT is active FT4 and JS8 leds will be solid lit.
// In CAT mode none of the Switches and leds are active including TX SWITCH in order to avoid different setting clashes except TX LED.
// TX LED WILL BE LIT briefly on/off and then solid during TX WHEN TRANSMITTING IN CAT Mode.
// In CAT mode ADX can be controlled ONLY by CAT interface. Frequency and TX can be controlled via CAT.
// To get out of CAT mode and to use ADX with Switch and led interface just recycle power. Once activated CAT mode stays active as rig control Until power recycle.
// In CAT mode manual Band setup is deactivated and ADX can be operated on any band as long as the right lpf filter module is plugged in.
// IN CAT MODE MAKE SURE THE CORRECT LPF MODULE IS PLUGGED IN WHEN OPERATING BAND IS CHANGED!!! IF WRONG LPF FILTER MODULE IS PLUGGED IN then PA POWER MOSFETS CAN BE DAMAGED!

//*******************************[ LIBRARIES ]*************************************************
#include <si5351.h>
#include "Wire.h"
#include <EEPROM.h>
//*******************************[ VARIABLE DECLERATIONS ]*************************************
unsigned int temp = 0;
unsigned int mode = 0;
unsigned int Band_slot = 0;
unsigned int Band = 0;
unsigned int Bdly = 250;  // Blick delay

boolean TX_State = 0;
boolean UP_State = 0;
boolean DOWN_State = 0;
boolean TXSW_State = 0;

unsigned long freq = 0;
unsigned long cal_factor = 0;
unsigned long Cal_freq = 1000000UL; // Calibration Frequency: 1 Mhz = 1000000 Hz
unsigned long F_FT8;
unsigned long F_FT4;
unsigned long F_JS8;
unsigned long F_WSPR;

//------------ CAT Variables
boolean cat_stat = 0;
boolean TxStatus = 0; //0 =  RX 1=TX
unsigned int CAT_mode = 2;
String received;
String receivedPart1;
String receivedPart2;
String command;
String command2;
String parameter;
String parameter2;
String sent;
String sent2;

// **********************************[ DEFINE's ]***********************************************
#define UP    2 // UP Switch
#define DOWN  3 // DOWN Switch
#define TXSW  4 // TX Switch
#define RX    8 // RX SWITCH
#define TX   13 // TX LED
#define WSPR  9 // WSPR LED
#define JS8  10 // JS8 LED
#define FT4  11 // FT4 LED
#define FT8  12 // FT8 LED

#define SI5351_REF     25000000UL  //change this to the frequency of the crystal on your si5351’s PCB, usually 25 or 27 MHz

//**********************************[ BAND SELECT ]************************************************
// ADX can support up to 4 bands on board. Those 4 bands needs to be assigned to Band1 ... Band4 from supported 6 bands.
// To change bands press SW1 and SW2 simultaneously. Band LED will flash 3 times briefly and stay lit for the stored band. also TX LED will be lit to indicate
// that Band select mode is active. Now change band bank by pressing SW1(<---) or SW2(--->). When desired band bank is selected press TX button briefly to exit band select mode.
// Now the new selected band bank will flash 3 times and then stored mode LED will be lit.
// TX won't activate when changing bands so don't worry on pressing TX button when changing bands in band mode.

// Assign your prefered bands to B1,B2,B3 and B4
// Supported Bands are: 80m, 40m, 30m, 20m,17m, 15m

unsigned int Band1 = 40; // Band 1 // These are my default bands. Feel free to swap with yours
unsigned int Band2 = 30; // Band 2
unsigned int Band3 = 20; // Band 3
unsigned int Band4 = 17; // Band 4

Si5351 si5351;

//*************************************[ SETUP FUNCTION ]**************************************
void setup() {
  pinMode(UP, INPUT);
  pinMode(DOWN, INPUT);
  pinMode(TXSW, INPUT);
  pinMode(TX, OUTPUT);
  pinMode(WSPR, OUTPUT);
  pinMode(JS8, OUTPUT);
  pinMode(FT4, OUTPUT);
  pinMode(FT8, OUTPUT);
  pinMode(RX, OUTPUT);
  pinMode(7, INPUT); //PD7 = AN1 = HiZ, PD6 = AN0 = 0

  //SET UP SERIAL FOR CAT CONTROL

  Serial.begin(9600);
  Serial.setTimeout(4);
  
  INIT(); // Get data from EEprom

  //------------------------------- SET SI5351 VFO -----------------------------------
  // The crystal load value needs to match in order to have an accurate calibration
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA); // SET For Max Power
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_2MA); // Set for reduced power for RX
  si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_2MA); // Set for lower power for calibration
  si5351.set_clock_pwr(SI5351_CLK2, 0); // Turn off Calibration Clock

  if ( digitalRead(DOWN) == LOW ) {
    Calibration();
  }

  TCCR1A = 0x00;
  TCCR1B = 0x01; // Timer1 Timer 16 MHz
  TCCR1B = 0x81; // Timer1 Input Capture Noise Canceller
  ACSR |= (1<<ACIC);  // Analog Comparator Capture Input

  digitalWrite(RX,LOW);

  Band_assign();
  Freq_assign();
  Mode_assign();
}

//**************************[ END OF SETUP FUNCTION ]************************

//***************************[ Main LOOP Function ]**************************
void loop() {
  // CAT CHECK SERIAL FOR NEW DATA FROM WSJT-X AND RESPOND TO QUERIES

  if ((Serial.available() > 0) || (cat_stat == 1)) {
    cat_stat = 1;
    CAT_control();
    si5351.set_freq(freq * 100ULL, SI5351_CLK1);
    si5351.output_enable(SI5351_CLK1, 1);   //RX on
    digitalWrite(WSPR, LOW);
    digitalWrite(JS8, HIGH);
    digitalWrite(FT4, HIGH);
    digitalWrite(FT8, LOW);
  }
  else {  
    UP_State = digitalRead(UP);
    DOWN_State = digitalRead(DOWN);

    if ((UP_State == LOW) && (DOWN_State == LOW) && (TX_State == 0)) {
      delay(100);
      UP_State = digitalRead(UP);
      DOWN_State = digitalRead(DOWN);
      if ((UP_State == LOW) && (DOWN_State == LOW) && (TX_State == 0)) {
        Band_Select();
      }
    }

    if ((UP_State == LOW) && (DOWN_State == HIGH) && (TX_State == 0)) {
      delay(100);
      UP_State = digitalRead(UP);
      DOWN_State = digitalRead(DOWN);
      if ((UP_State == LOW) && (DOWN_State == HIGH) && (TX_State == 0)) {
        mode = mode - 1;
        if (mode < 1) {
          mode = 4;
        }
        EEPROM.put(40, mode);
        Mode_assign();
      }
    }

    UP_State = digitalRead(UP);
    DOWN_State = digitalRead(DOWN);

    if ((UP_State == HIGH) && (DOWN_State == LOW) && (TX_State == 0)) {
      delay(100);
      UP_State = digitalRead(UP);
      DOWN_State = digitalRead(DOWN);
      if ((UP_State == HIGH) && (DOWN_State == LOW) && (TX_State == 0)) {
        mode = mode + 1;
        if (mode > 4){
          mode = 1;
        }
        EEPROM.put(40, mode);
        Mode_assign();
      }
    }
  }
  
  TXSW_State = digitalRead(TXSW);

  if ((TXSW_State == LOW) && (TX_State == 0)) {
    delay(100);
    TXSW_State = digitalRead(TXSW);
    if ((TXSW_State == LOW) && (TX_State == 0)) {
      Mode_assign();
      ManualTX();
    }
  }

  // The following code is from JE1RAV https://github.com/je1rav/QP-7C
  //(Using 3 cycles for timer sampling to improve the precision of frequency measurements)
  //(Against overflow in low frequency measurements)

  int FSK = 1;
  int FSKtx = 0;

  while (FSK>0) {
    int Nsignal = 10;
    int Ncycle01 = 0;
    int Ncycle12 = 0;
    int Ncycle23 = 0;
    int Ncycle34 = 0;
    unsigned int d1=1,d2=2,d3=3,d4=4;

    TCNT1 = 0;
    while (ACSR &(1<<ACO)) {
      if (TIFR1&(1<<TOV1)) {
        Nsignal--;
        TIFR1 = _BV(TOV1);
        if (Nsignal <= 0) {
          break;
        }
      }
    }
    while ((ACSR &(1<<ACO))==0) {
      if (TIFR1&(1<<TOV1)) {
        Nsignal--;
        TIFR1 = _BV(TOV1);
        if (Nsignal <= 0) {
          break;
        }
      }
    }
    if (Nsignal <= 0) {
      break;
    }
    TCNT1 = 0;
    while (ACSR &(1<<ACO)) {
        if (TIFR1&(1<<TOV1)) {
        Ncycle01++;
        TIFR1 = _BV(TOV1);
        if (Ncycle01 >= 2) {
          break;
        }
      }
    }
    d1 = ICR1;
    while ((ACSR &(1<<ACO))==0) {
      if (TIFR1&(1<<TOV1)) {
        Ncycle12++;
        TIFR1 = _BV(TOV1);
        if (Ncycle12 >= 3) {
          break;
        }
      }
    }
    while (ACSR &(1<<ACO)) {
      if (TIFR1&(1<<TOV1)) {
        Ncycle12++;
        TIFR1 = _BV(TOV1);
        if (Ncycle12 >= 6) {
          break;
        }
      }
    }
    d2 = ICR1;
    while ((ACSR &(1<<ACO))==0) {
      if (TIFR1&(1<<TOV1)) {
        Ncycle23++;
        TIFR1 = _BV(TOV1);
        if (Ncycle23 >= 3) {
          break;
        }
      }
    }
    while (ACSR &(1<<ACO)) {
      if (TIFR1&(1<<TOV1)) {
        Ncycle23++;
        TIFR1 = _BV(TOV1);
        if (Ncycle23 >= 6) {
          break;
        }
      }
    }
    d3 = ICR1;
    while ((ACSR &(1<<ACO))==0) {
      if (TIFR1&(1<<TOV1)) {
        Ncycle34++;
        TIFR1 = _BV(TOV1);
        if (Ncycle34 >= 3) {
          break;
        }
      }
    }
    while (ACSR &(1<<ACO)) {
      if (TIFR1&(1<<TOV1)) {
        Ncycle34++;
        TIFR1 = _BV(TOV1);
        if (Ncycle34 >= 6) {
          break;
        }
      }
    }
    d4 = ICR1;
    unsigned long codefreq1 = 1600000000/(65536*Ncycle12+d2-d1);
    unsigned long codefreq2 = 1600000000/(65536*Ncycle23+d3-d2);
    unsigned long codefreq3 = 1600000000/(65536*Ncycle34+d4-d3);
    unsigned long codefreq = (codefreq1 + codefreq2 + codefreq3)/3;
    if (d3==d4) codefreq = 5000;
    if ((codefreq < 310000) && (codefreq >= 10000)) {   // Frequency between 100 Hz and 3100 Hz
      if (FSKtx == 0) {
        TX_State = 1;
        digitalWrite(RX,LOW);
        digitalWrite(TX,HIGH);
        delay(10);
        si5351.output_enable(SI5351_CLK1, 0);   // RX off
        si5351.output_enable(SI5351_CLK0, 1);   // TX on
      }
      si5351.set_freq((freq * 100ULL + codefreq), SI5351_CLK0);
      if (cat_stat == 1) CAT_control();
      FSKtx = 1;
    }
    else {
      FSK--;
    }
  }

  digitalWrite(TX,LOW);

  si5351.output_enable(SI5351_CLK0, 0);   // TX off
  si5351.set_freq(freq * 100ULL, SI5351_CLK1);
  si5351.output_enable(SI5351_CLK1, 1);   // RX on
  TX_State = 0;
  digitalWrite(RX,HIGH);
  FSKtx = 0;
}

//*********************[ END OF MAIN LOOP FUNCTION ]*************************

//********************************************************************************
//******************************** [ FUNCTIONS ] *********************************
//********************************************************************************


//************************************[ MODE Assign ]**********************************

void Mode_assign() {

  EEPROM.get(40, mode);

  switch(mode) {
    case 1:
      freq = F_WSPR;
      digitalWrite(WSPR, HIGH);
      digitalWrite(JS8, LOW);
      digitalWrite(FT4, LOW);
      digitalWrite(FT8, LOW);
      break;
    case 2:
      freq = F_JS8;
      digitalWrite(WSPR, LOW);
      digitalWrite(JS8, HIGH);
      digitalWrite(FT4, LOW);
      digitalWrite(FT8, LOW);
      break;
    case 3:
      freq = F_FT4;
      digitalWrite(WSPR, LOW);
      digitalWrite(JS8, LOW);
      digitalWrite(FT4, HIGH);
      digitalWrite(FT8, LOW);
      break;
    case 4:
      freq = F_FT8;
      digitalWrite(WSPR, LOW);
      digitalWrite(JS8, LOW);
      digitalWrite(FT4, LOW);
      digitalWrite(FT8, HIGH);
      break;
  }
}

//********************************[ END OF MODE ASSIGN ]*******************************

//*********************[ Band dependent Frequency Assign Function ]********************
void Freq_assign() {

  switch (Band) {
    case 80:
      F_FT8 = 3573000;
      F_FT4 = 3575000;
      F_JS8 = 3578000;
      F_WSPR = 3568600;
      break;
    case 40:
      F_FT8 = 7074000;
      F_FT4 = 7047500;
      F_JS8 = 7078000;
      F_WSPR = 7038600;
      break;
    case 30:
      F_FT8 = 10136000;
      F_FT4 = 10140000;
      F_JS8 = 10130000;
      F_WSPR = 10138700;
      break;
    case 20:
      F_FT8 = 14074000;
      F_FT4 = 14080000;
      F_JS8 = 14078000;
      F_WSPR = 14095600;
      break;
    case 17:
      F_FT8 = 18100000;
      F_FT4 = 18104000;
      F_JS8 = 18104000;
      F_WSPR = 18104600;
      break;
    case 15:
      F_FT8 = 21074000;
      F_FT4 = 21140000;
      F_JS8 = 21078000;
      F_WSPR = 21094600;
      break;
    case 10:
      F_FT8 = 28074000;
      F_FT4 = 28180000;
      F_JS8 = 28078000;
      F_WSPR = 28124600;
      break;
  }
}
//************************[ End of Frequency assign function ]*************************

//******************************[ Band  Assign Function ]******************************

void Band_assign() {
  digitalWrite(WSPR, LOW);
  digitalWrite(JS8, LOW);
  digitalWrite(FT4, LOW);
  digitalWrite(FT8, LOW);

  EEPROM.get(50, Band_slot);

  switch (Band_slot) {
    case 1:
      Band = Band1;
      blinkLED(WSPR, Bdly, 3);
      break;
    case 2:
      Band = Band2;
      blinkLED(JS8, Bdly, 3);
      break;
    case 3:
      Band = Band3;
      blinkLED(FT4, Bdly, 3);
      break;
    case 4:
      Band = Band4;
      blinkLED(FT8, Bdly, 3);
      break;
  }

  delay(1000);
  Freq_assign();
  Mode_assign();
}
//***************************[ End of Band assign function ]***************************

//*******************************[ Manual TX FUNCTION ]********************************
void ManualTX() {

  digitalWrite(RX,LOW);
  si5351.output_enable(SI5351_CLK1, 0);   // RX off
  digitalWrite(TX,HIGH);
  si5351.set_freq((freq*100ULL), SI5351_CLK0);
  si5351.output_enable(SI5351_CLK0, 1);   // TX on
  TX_State = 1;
  TXSW_State = digitalRead(TXSW);

  while (TXSW_State == LOW) {  // As long as TX is pressed
    TXSW_State = digitalRead(TXSW);
    if (cat_stat == 1) CAT_control();
    delay(50);
  }

  digitalWrite(TX,LOW);
  si5351.output_enable(SI5351_CLK0, 0);   // TX off
  TX_State = 0;
}

//********************************[ END OF Manual TX ]*********************************

//******************************[ BAND SELECT Function]********************************
void Band_Select() {

  digitalWrite(TX,HIGH);
  EEPROM.get(50, Band_slot);

  digitalWrite(WSPR,LOW);
  digitalWrite(JS8, LOW);
  digitalWrite(FT4, LOW);
  digitalWrite(FT8, LOW);

  switch (Band_slot) {
    case 1:
      blinkLED(WSPR, Bdly, 3);
      break;
    case 2:
      blinkLED(JS8, Bdly, 3);
      break;
    case 3:
      blinkLED(FT4, Bdly, 3);
      break;
    case 4:
      blinkLED(FT8, Bdly, 3);
      break;
  }

  while (TXSW_State == HIGH) {  // Use TX switch to end Band select

    switch (Band_slot) {
      case 1:
        digitalWrite(WSPR, HIGH);
        digitalWrite(JS8, LOW);
        digitalWrite(FT4, LOW);
        digitalWrite(FT8, LOW);
        break;
      case 2:
        digitalWrite(JS8, HIGH);
        digitalWrite(WSPR, LOW);
        digitalWrite(FT4, LOW);
        digitalWrite(FT8, LOW);
        break;
      case 3:
        digitalWrite(FT4, HIGH);
        digitalWrite(WSPR, LOW);
        digitalWrite(JS8, LOW);
        digitalWrite(FT8, LOW);
        break;
      case 4:
        digitalWrite(JS8, LOW);
        digitalWrite(WSPR, LOW);
        digitalWrite(FT4, LOW);
        digitalWrite(FT8, HIGH);
        break;
    }

    UP_State = digitalRead(UP);
    DOWN_State = digitalRead(DOWN);
    TXSW_State = digitalRead(TXSW);
    
    if ((UP_State == LOW) && (DOWN_State == HIGH)) {
      delay(100);
      UP_State = digitalRead(UP);
      if ((UP_State == LOW) && (DOWN_State == HIGH)) {
        Band_slot = Band_slot - 1;
        if (Band_slot < 1) {
          Band_slot = 4;
        }
      }
    }

    if ((UP_State == HIGH) && (DOWN_State == LOW)) {
      delay(100);
      DOWN_State = digitalRead(DOWN);
      if ((UP_State == HIGH) && (DOWN_State == LOW)) {
        Band_slot = Band_slot + 1;
        if (Band_slot > 4) {
          Band_slot = 1;
        }
      }
    }
  }
  
  digitalWrite(TX,0);
  EEPROM.put(50, Band_slot);

  Band_assign();
}

//*********************************[ END OF BAND SELECT ]*****************************


//************************** [SI5351 VFO Calibration Function] ************************

void Calibration(){

  digitalWrite(FT8, LOW);
  digitalWrite(FT4, LOW);
  digitalWrite(JS8, LOW);
  digitalWrite(WSPR, LOW);

  for (int i = 0; i < 3; i++) {
    digitalWrite(WSPR, HIGH);
    digitalWrite(FT8, HIGH);
    delay(Bdly);
    digitalWrite(WSPR, LOW);
    digitalWrite(FT8, LOW);
    delay(Bdly);
  }
  
  digitalWrite(WSPR, HIGH);
  digitalWrite(FT8, HIGH);

  EEPROM.get(10, cal_factor);

  // Set CLK2 output
  si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
  si5351.set_freq(Cal_freq * 100ULL, SI5351_CLK2);
  si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_2MA); // Set for lower power for calibration
  si5351.set_clock_pwr(SI5351_CLK2, 1); // Enable the clock for calibration

  TXSW_State = digitalRead(TXSW);

  while (TXSW_State == HIGH) {  // Use TX switch to end calibration

    UP_State = digitalRead(UP);

    if (UP_State == LOW) {
      delay(100);

      UP_State = digitalRead(UP);
      if (UP_State == LOW) {
        cal_factor = cal_factor - 100;
        si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
        TXSW_State = digitalRead(TXSW);
        continue;
      }
    }

    DOWN_State = digitalRead(DOWN);

    if (DOWN_State == LOW) {
      delay(100);
      DOWN_State = digitalRead(DOWN);
      if (DOWN_State == LOW) {
        cal_factor = cal_factor + 100;
        si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
        TXSW_State = digitalRead(TXSW);
        continue;
      }
    }
    delay(100);
    TXSW_State = digitalRead(TXSW);
  }

  EEPROM.put(10, cal_factor);           // Write cal_factor to EEProm
  si5351.set_clock_pwr(SI5351_CLK2, 0); // Disable the clock after calibration
  blinkLED(TX, Bdly, 3);
}

//****************************** [ End Of Calibration Function ]****************************************

//*********************************[ INITIALIZATION FUNCTION ]******************************************
void INIT() {

  EEPROM.get(30,temp);

  if (temp != 100) {
    // INIT EEPROM WITH DEFAULTS
    EEPROM.put(10, 100000); // cal_factor
    EEPROM.put(30, 100);    // EEPROM OK
    EEPROM.put(40, 4);      // Default Mode 1
    EEPROM.put(50, 1);      // Default Band 1
  }
  else {
    // READ EEPROM VALUES
    EEPROM.get(10, cal_factor);
    EEPROM.get(40, mode);
    EEPROM.get(50, Band_slot);
  }
}

//********************************[ END OF INITIALIZATION FUNCTION ]*************************************

//*********************************[ LED BLINK FUNCTION ]******************************************

void blinkLED(int ledPin, int blinkTime, int numBlinks) {
  pinMode(ledPin, OUTPUT);
  for (int i = 0; i < numBlinks; i++) {
    digitalWrite(ledPin, HIGH);
    delay(blinkTime);
    digitalWrite(ledPin, LOW);
    delay(blinkTime);
  }
}
//*********************************[ END OF BLINK FUNCTION ]***************************************
//*****************************[ CAT CONTROL FUNCTION ]***************************

void CAT_control(void) {

  received = Serial.readString();
  received.toUpperCase();
  received.replace("\n","");

  String data = "";
  int bufferIndex = 0;

  for (int i = 0; i < received.length(); ++i) {
    char c = received[i];
    if (c != ';') {
      data += c;
    }
    else {
      if (bufferIndex == 0) {
        data += '\0';
        receivedPart1 = data;
        bufferIndex++;
        data = "";
      }
      else {
        data += '\0';
        receivedPart2 = data;
        bufferIndex++;
        data = "";
      }
    }
  }

  command = receivedPart1.substring(0,2);
  command2 = receivedPart2.substring(0,2);
  parameter = receivedPart1.substring(2,receivedPart1.length());
  parameter2 = receivedPart2.substring(2,receivedPart2.length());

  if (command == "FA") {
    if (parameter != "") {
      freq = parameter.toInt();
      //VfoRx = VfoTx;
    }
    sent = "FA" // Return 11 digit frequency in Hz.
      + String("00000000000").substring(0,11-(String(freq).length()))
      + String(freq) + ";";
  }
  else if (command == "PS") {
    sent = "PS1;";
  }
  else if (command == "TX") {
    sent = "TX0;";
    //VfoTx = freq;
    digitalWrite(TX, HIGH);
    TxStatus = 1;
  }
  else if (command == "RX") {
    sent = "RX0;";
    //VfoRx = freq;
    digitalWrite(TX, LOW);
    TxStatus = 0;
  }
  else  if (command == "ID") {
    sent = "ID019;";
  }
  else if (command == "AI") {
    sent = "AI0;";
  }
  else if (command == "IF") {
    if (TxStatus == 1) {
      sent = "IF" // Return 11 digit frequency in Hz.
        + String("00000000000").substring(0,11-(String(freq).length()))
        + String(freq) + "00000" + "+" + "0000" + "0" + "0" + "0" + "00" + "1" + String(CAT_mode) + "0" + "0" + "0" + "0" + "000" + ";";
    }
    else {
      sent = "IF" // Return 11 digit frequency in Hz.
        + String("00000000000").substring(0,11-(String(freq).length()))
        + String(freq) + "00000" + "+" + "0000" + "0" + "0" + "0" + "00" + "0" + String(CAT_mode) + "0" + "0" + "0" + "0" + "000" + ";";
    }
  }
  else if (command == "MD") {
    sent = "MD2;";
  }

  //------------------------------------------------------------------------------

  if (command2 == "ID") {
    sent2 = "ID019;";
  }

  if (bufferIndex == 2) {
    Serial.print(sent2);
  }
  else {
    Serial.print(sent);
  }

  if ((command == "RX") || (command = "TX")) delay(50);
  sent = String("");
  sent2 = String("");
}
//*****************************[ END OF CAT CONTROL FUNCTION ]***************************
