//*********************************************************************************************************
//************* ADX - ARDUINO based DIGITAL MODES 4 BAND HF TRANSCEIVER  WITH CAT CONTROL******************
//********************************* Write up start: 02/01/2022 ********************************************
// FW VERSION: ADX_QUAD_V3.1 - Version release date: 05/1/2022
// Barb(Barbaros ASUROGLU) - WB2CBA - 2022
// V4: Alan Altman AG7XS ADX CAT software, CAT CONTROL added, now, stable release 6/14/2022
//*********************************************************************************************************
// Required Libraries
// ----------------------------------------------------------------------------------------------------------------------
// Etherkit Si5351 (Needs to be installed via Library Manager to arduino ide) - SI5351 Library by Jason Mildrum (NT7S) - https://github.com/etherkit/Si5351Arduino
// Arduino "Wire.h" I2C library(built-into arduino ide)
// Arduino "EEPROM.h" EEPROM Library(built-into arduino ide)
//*************************************[ LICENSE and CREDITS ]*********************************************
//  FSK TX Signal Generation code by: Burkhard Kainka(DK7JD) - http://elektronik-labor.de/HF/SDRtxFSK2.html
//  SI5351 Library by Jason Mildrum (NT7S) - https://github.com/etherkit/Si5351Arduino

//********************* CONNECTING THIS SOFTWARE TO WSJT-X AND ADX ****************************
//  SETUP WITH WSJT-X, - Setup WSJT-X for IC-746, 8N2 19,200 Baud. POLL INT 60 SEC
//  CONNECT USB FROM ADX TO COMPUTER AND ESTABLISH USB CONNECTION BEFORE CONNECTING +12V SOURCE

//*************************** LICENSE *************************
// 
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
// 4-  Power up with 12V or with 5V by using arduino USB socket while still pressing SW2 / --->(CAL) pushbutton.
// 5 - FT8 and WSPR LEDs will flash 3 times and stay lit. Now Release SW2 / --->(CAL). Now Calibration mode is active.
// 6 - Using SW1(<---) and SW2(--->) pushbuttons change the Calibration frequency.
// 7 - Try to read 1 Mhz = 1000000 Hz exact on  Frequency counter or Oscilloscope.
//     The waveform is Square wave so freqency calculation can be performed easily.
// 7 - If you read as accurate as possible 1000000 Hz then calibration is done.
// 8 - Power off ADX.

//*******************************[ LIBRARIES ]*************************************************
#include <si5351.h>
#include "Wire.h"
#include <EEPROM.h>
//*******************************[ VARIABLE DECLERATIONS ]*************************************
uint32_t val;
uint32_t val_EE;
int addr = 0;
int mode;
unsigned long freq =  14074000;
unsigned long freq1 = 14074000;
int32_t cal_factor;
int TX_State = 0;

unsigned long Cal_freq = 1000000UL; // Calibration Frequency: 1 Mhz = 1000000 Hz

int UP_State;
int DOWN_State;
int TXSW_State;

//CAT CONTROL DATA FROM WSJT-X AND RESPONSES 

byte CommandToArduino[12] =  {0xFE, 0xFE, 0x56, 0xE0, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0xFD}; //received message from WSJT or other program

//CAT RESPONSES TO EACH DATA REQUEST FROM WSJT-X format is IC-746 forma, reason: code originally for IC-720a and IC-746 simplest to simulate

byte replyFreq[12] =    {0xFE, 0xFE, 0xE0, 0x56, 0x03, 0x00, 0x40, 0x07, 0x14, 0x00, 0xFD};//14.0740 default
byte replyFreqOld[12] = {0xFE, 0xFE, 0xE0, 0x56, 0x03, 0x00, 0x40, 0x07, 0x18, 0x00, 0xFD};//18.0740 default
byte replyMode[8] =     {0xFE, 0xFE, 0xE0, 0x56, 0x04, 0x00, 0x01, 0xFD}; //USB default, filter
byte replyRig[8] =      {0xFE, 0xFE, 0xE0, 0x56, 0x19, 0x00, 0x56, 0xFD};
byte OK[6] =            {0xFE, 0xFE, 0xE0, 0x56, 0xFB, 0xFD};
byte NFG[6] =           {0xFE, 0xFE, 0xE0, 0x56, 0xFA, 0xFD};
byte replyRx[8] =       {0xFE, 0xFE, 0xE0, 0x56, 0x1C, 0x00, 0x00, 0xFD};
byte replyTx[8] =       {0xFE, 0xFE, 0xE0, 0x56, 0x1C, 0x00, 0x01, 0xFD};
int ArdToADX[10] =      {1, 4, 0, 7, 4, 0, 0, 0, 0, 0}; // frequency for ADX converted from 746 BCD format

int RxOrTx = 0; //transmit = 1,  receive = 0
int modeIndex = 0x01; //saves mode in 746 format as received from WSJT or Flrig
// 746 mode =     LSB is 0x00 USB is 0x01  AM is 0x02  CW is 0x03  RTTY iis 0x04 other modes default to USB Ex. mode 0 on 746 is LSB == 0x0B on 720a
int baud(19200);//USB baud rate

unsigned long CATFreqForSi5351;

// **********************************[ DEFINE's ]***********************************************
#define UP    2  // UP Switch
#define DOWN  3  // DOWN Switch
#define TXSW  4  // TX Switch

#define TX   13 //TX LED
#define WSPR  9 //WSPR LED 
#define JS8  10 //JS8 LED
#define FT4  11 //FT4 LED
#define FT8  12 //FT8 LED

#define RX  8 // RX SWITCH
#define SI5351_REF 		25000000UL  //change this to the frequency of the crystal on your si5351’s PCB, usually 25 or 27 MHz

#define PTT  5 //PIN# output PTT To radio HIGH to 2N3904 with snubber diode 1N4001 to ground pin  on radio

Si5351 si5351;

//*************************************[ SETUP FUNCTION ]**************************************
void setup()
{
  //SET UP SERIAL FOR CAT CONTROL

  Serial.begin(baud, SERIAL_8N2); //SERIAL_8N1) is default
  while (!Serial) {
  } //wait for serial port to connect. Needed for native USB port only
  Serial.setTimeout(10);

  pinMode(UP, INPUT);
  pinMode(DOWN, INPUT);
  pinMode(TXSW, INPUT);

  pinMode(TX, OUTPUT);
  pinMode(WSPR, OUTPUT);
  pinMode(JS8, OUTPUT);
  pinMode(FT4, OUTPUT);
  pinMode(FT8, OUTPUT);
  pinMode(RX, OUTPUT);
  pinMode(PTT,OUTPUT); // for CAT CONTROL

  pinMode(7, INPUT); //PD7 = AN1 = HiZ, PD6 = AN0 = 0

  //------------------------------- SET SI5351 VFO -----------------------------------
  // The crystal load value needs to match in order to have an accurate calibration
  addr = 10;       //--------------- EEPROM INIT VALUES FOR CALIBRATION
  EEPROM.get(addr, cal_factor);
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);// SET For Max Power
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_2MA); // Set for reduced power for RX

  if ( digitalRead(DOWN) == LOW ) {
    Calibration();
  }

  TCCR1A = 0x00;
  TCCR1B = 0x01; // Timer1 Timer 16 MHz
  TCCR1B = 0x81; // Timer1 Input Capture Noise Canceller
  ACSR |= (1 << ACIC); // Analog Comparator Capture Input

  pinMode(7, INPUT); //PD7 = AN1 = HiZ, PD6 = AN0 = 0
  digitalWrite(RX, LOW);

}

//***************************[ Main LOOP Function ]**************************
void loop()
{
// CAT CHECK SERIAL FOR NEW DATA FROM WSJT-X AND RESPOND TO QUERIES

  if (Serial.available() > 1)
  {
    Serial.readBytesUntil(0xFD, CommandToArduino, 12);
    compToArdAndBack();
    for (int i = 3; i < 12; i++) {
      if ( replyFreq[i] != replyFreqOld[i]) {
        //FREQUENCY CALCULATION FUNCTION AND SEND TO SI5351 AND MODE/PTT WHEN CW ADDED
        convertFreqFormat() ;    
        si5351.set_freq(freq1 * 100ULL, SI5351_CLK1);
        si5351.output_enable(SI5351_CLK1, 1);   //RX on
     }
    }
  }
    for (int i = 0; i < 12; i++ ) {
    replyFreqOld[i] = replyFreq[i];
  }

  unsigned int d1, d2;
  int FSK = 10;
  int FSKtx = 0;
  while (FSK > 0) {
    TCNT1 = 0;
    while (ACSR & (1 << ACO)) {
      if (TCNT1 > 65000) {
        break;
      }
    }  while ((ACSR & (1 << ACO)) == 0) {
      if (TCNT1 > 65000) {
        break;
      }
    }
    TCNT1 = 0;
    while (ACSR & (1 << ACO)) {
      if (TCNT1 > 65000) {
        break;
      }
    }
    d1 = ICR1;
    while ((ACSR & (1 << ACO)) == 0) {
      if (TCNT1 > 65000) {
        break;
      }
    }
    while (ACSR & (1 << ACO)) {
      if (TCNT1 > 65000) {
        break;
      }
    }
    d2 = ICR1;
    if (TCNT1 < 65000) {
      unsigned long codefreq = 1600000000 / (d2 - d1);
      if (codefreq < 350000) {
        if (FSKtx == 0) {
          TX_State = 1;
          digitalWrite(TX, HIGH);
          digitalWrite(RX, LOW);

          si5351.output_enable(SI5351_CLK1, 0);   //RX off
          si5351.output_enable(SI5351_CLK0, 1);   // TX on
        }
        si5351.set_freq((freq * 100 + codefreq), SI5351_CLK0);

        FSKtx = 1;
      }
    }
    else {
      FSK--;
    }
  }
  digitalWrite(TX, 0);

  si5351.output_enable(SI5351_CLK0, 0);   //TX off
  si5351.set_freq(freq * 100ULL, SI5351_CLK1);
  si5351.output_enable(SI5351_CLK1, 1);   //RX on
  TX_State = 0;
  digitalWrite(RX, HIGH);
  FSKtx = 0;

}

//*************************COMPUTER TO ARDUINO AND RESPONSE FOR CAT CONTROL******************

void compToArdAndBack() {   

  switch (CommandToArduino[4]) {

    case 0x03:  // get rig freq

      Serial.write (replyFreq, 11);
      break;

    case 0x04:   //read mode

      replyMode[5] = modeIndex; //checks last mode sent to 720a saved directly from received command (746 format)
      Serial.write (replyMode, 8);
      break;

    case 0x05:   //set frequency

      for (int i = 5; i < 10; i++) { //this stores the frequency in the 746 format, low byte first, only split into BCD
        replyFreq[i] = CommandToArduino[i];
        }
      Serial.write (OK, 6);
      break;

    case 0x06:     //set mode/filter

      modeIndex = CommandToArduino[5];//saves mode in 746 format which is 0 to 4
      Serial.write (OK, 6);
      break;

    case 0x07:  //set vfo a/b

      if  (CommandToArduino[5] == 0x01) {
      }
      else {
      }

      Serial.write (OK, 6);
      break;

    case 0x19:   //radio ID

      Serial.write (replyRig, 8);
      break;

    case 0x1C:   //get & set ptt status

      if  (CommandToArduino[6] == 0xFD && RxOrTx == 0) // Get PTT status
      {
        Serial.write (replyRx, 8);
      }
      if  (CommandToArduino[6] == 0xFD && RxOrTx == 1)
      {
        Serial.write (replyTx, 8);
      }

      if  (CommandToArduino[7] == 0xFD && CommandToArduino[6] == 0x00) // Set PTT to RX
      {
        digitalWrite( PTT, LOW);
        RxOrTx = 0;
        Serial.write (OK, 6);
      }

      if  (CommandToArduino[7] == 0xFD && CommandToArduino[6] == 0x01) // Set PTT to TX
      {
        digitalWrite(PTT, HIGH);
        RxOrTx = 1;
        Serial.write (OK, 6);
      }
      break;

    default:
      {
        Serial.write (NFG, 6);
      }
  }
}


//******************************** Convert Frequency from 746 BCD to ADX format for Si5351 ************************************

void convertFreqFormat() { //converts frequency from 746 BCD 2 digits per word format to ADX format

  for (int i = 5; i < 10; i++)  //this stores the frequency in the 746 format, low byte first, only split into BCD
  {
    replyFreq[i] = CommandToArduino[i];
  }
  ArdToADX[0] =  replyFreq[8] >> 4 & 0x0F; //takes byte with 2 BCD digits and shifts high nibble to low nibble
  ArdToADX[1] =  replyFreq[8] & 0x0F; // blanks out high nibble so low BCD digit saved
  ArdToADX[2] =  replyFreq[7] >> 4 & 0x0F; //& bit wise AND
  ArdToADX[3] =  replyFreq[7] & 0x0F;
  ArdToADX[4] =  replyFreq[6] >> 4 & 0x0F;
  ArdToADX[5] =  replyFreq[6] & 0x0F;
  freq = 100UL * (ArdToADX[0] * 100000UL + ArdToADX[1] * 10000UL + ArdToADX[2] * 1000UL + ArdToADX[3] * 100UL + ArdToADX[4] * 10UL + ArdToADX[5]);
  freq1 = freq;
}

//************************** [SI5351 VFO Calibration Function] ************************

void Calibration() {

  for (int i = 0; i < 4; i++){

    delay(100);
    digitalWrite(WSPR, LOW);
    digitalWrite(FT8, LOW);
    delay(100);
    digitalWrite(WSPR, HIGH);
    digitalWrite(FT8, HIGH);
  }

  addr = 10;
  EEPROM.get(addr, cal_factor);
  //Serial.print("cal factor= ");

Calibrate:

  UP_State = digitalRead(UP);

  if (UP_State == LOW) {
    delay(50);

    UP_State = digitalRead(UP);
    if (UP_State == LOW) {

      cal_factor = cal_factor - 100;
      EEPROM.put(addr, cal_factor);

      si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);

      // Set CLK2 output
      si5351.set_freq(Cal_freq * 100, SI5351_CLK2);
      si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_2MA); // Set for lower power for calibration
      si5351.set_clock_pwr(SI5351_CLK2, 1); // Enable the clock for calibration

    }
  }

  DOWN_State = digitalRead(DOWN);

  if (DOWN_State == LOW) {
    delay(50);

    DOWN_State = digitalRead(DOWN);
    if (DOWN_State == LOW) {

      cal_factor = cal_factor + 100;
      EEPROM.put(addr, cal_factor);

      si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);

      // Set CLK2 output
      si5351.set_freq(Cal_freq * 100, SI5351_CLK2);
      si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_2MA); // Set for lower power for Calibration
      si5351.set_clock_pwr(SI5351_CLK2, 1); // Enable clock2

    }
  }

  goto Calibrate;
}
