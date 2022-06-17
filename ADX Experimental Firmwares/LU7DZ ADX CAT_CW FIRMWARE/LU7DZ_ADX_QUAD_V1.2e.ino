//*********************************************************************************************************
//********************* ADX - ARDUINO based DIGITAL MODES 4 BAND HF TRANSCEIVER ***************************
//********************************* Write up start: 02/01/2022 ********************************************
// FW VERSION: ADX_QUAD_V1.1 - Version release date: 04/11/2022
// Barb(Barbaros ASUROGLU) - WB2CBA - 2022
//*********************************************************************************************************
// FW VERSION: ADX_QUAD_V1.2e (Experimental) release date 01/jun/2022
// PEC (Dr. Pedro E. Colla) - LU7DZ - 2022
//*********************************************************************************************************
// Required Libraries
// ----------------------------------------------------------------------------------------------------------------------
// Etherkit Si5351 (Needs to be installed via Library Manager to arduino ide) - 
//          SI5351 Library by Jason Mildrum (NT7S) - https://github.com/etherkit/Si5351Arduino
// Arduino "Wire.h" I2C library(built-into arduino ide)
// Arduino "EEPROM.h" EEPROM Library(built-into arduino ide)
// AVR "wdt.h" Watchdog Library
//*************************************[ LICENCE and CREDITS ]*********************************************
//  FSK TX Signal Generation code by: Burkhard Kainka(DK7JD) - http://elektronik-labor.de/HF/SDRtxFSK2.html
//  SI5351 Library by Jason Mildrum (NT7S) - https://github.com/etherkit/Si5351Arduino
//*-----------------------------------------------------------------------------------------------------------------*
//* Modified by Dr. P.E.Colla (LU7DZ)                                                                               
//*     X re-style of the code to facilitate customization for multiple boards
//*     X Add all frequency definitions for HF bands
//*     X Optimize EEPROM read/write cycles
//*     X add CW support (includes keyer support)
//*     X add CAT support (TS-440), thru FLRig (see README.md)
//*     X add timeout & watchdog support
//* Forked version of the original ADX firmware located at http://www.github.com/lu7did/ADX
//*-----------------------------------------------------------------------------------------------------------------*
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
// 4-  Power up with 12V or with 5V by using arduino USB socket while still pressing SW2 / --->(CAL) pushbutton. 
// 5 - FT8 and WSPR LEDs will flash 3 times and stay lit. Now Release SW2 / --->(CAL). Now Calibration mode is active.
// 6 - Using SW1(<---) and SW2(--->) pushbuttons change the Calibration frequency.
// 7 - Try to read 1 Mhz = 1000000 Hz exact on  Frequency counter or Oscilloscope. 
//     The waveform is Square wave so freqency calculation can be performed esaily.
// 7 - If you read as accurate as possible 1000000 Hz then calibration is done. 
// 8 - Power off ADX.
//*******************************[ LIBRARIES ]*************************************************
/*-----------------------------------------------------------*
 * System Includes
 *-----------------------------------------------------------*/
#include <stdint.h>
#include <avr/wdt.h> 
#include <si5351.h>
#include "Wire.h"
#include <EEPROM.h>
//********************************[ DEFINES ]***************************************************
#define VERSION        "1.2e"
#define BOOL2CHAR(x)  (x==true ? "True" : "False")
#undef  _NOP
#define _NOP          (byte)0
/*****************************************************************
 * CONFIGURATION Properties                                      *
 *****************************************************************/
#define WDT            1      //Hardware watchdog enabled
#define EE             1      //User EEPROM for persistence
#define CAT            1      //Emulates a TS-480 transceiver CAT protocol (reduced footprint)

/*
 * The following definitions are disabled but can be enabled selectively
 */
 
//#define CW           1      //Enable CW operation
//#define DEBUG        1      //DEBUG turns on different debug, information and trace capabilities, it is nullified when CAT is enabled to avoid conflicts
//#define SHIFTLIMIT   1      //Enforces tunning shift range into +/- 15 KHz when in CW mode
//#define ONEBAND      1      //Forces a single band operation in order not to mess up because of a wrong final filter
//#define CAT_FULL_PROTOCOL 1 //Extend CAT support to the entire TS480 CAT commands
/*****************************************************************
 * Consistency rules                                             *
 *****************************************************************/
#if (defined(DEBUG) || defined(CAT))

    #define BAUD 115200    
    char hi[120];
    
#endif //DEBUG or CAT

#if (defined(CAT) && defined(DEBUG))  //Rule for conflicting usage of the serial port
   #undef  DEBUG
#endif // CAT && DEBUG

#if (!defined(CAT))  //Rule for conflicting usage of the CAT Protocol (can't activate extended without basic)
   #undef  CAT_EXTENDED_PROTOCOL
#endif // CAT && DEBUG


#ifdef CAT
   //#define CAT_FULL_PROTOCOL   1
   
   #define CATCMD_SIZE          18
   #define SERIAL_TOUT          10
   
   volatile char    CATcmd[CATCMD_SIZE];
   const int        BUFFER_SIZE = CATCMD_SIZE;
   char             buf[BUFFER_SIZE];

#endif

/*****************************************************************
 * Trace and debugging macros                                    *
 *****************************************************************/
#ifdef DEBUG        //Remove comment on the following #define to enable the type of debug macro
   //#define INFO  1   //Enable _INFO and _INFOLIST statements
   //#define EXCP  1   //Enable _EXCP and _EXCPLIST statements
   //#define TRACE 1   //Enable _TRACE and _TRACELIST statements
#endif //DEBUG


/*-------------------------------------------------------------------------*
 * Define Info,Exception and Trace macros (replaced by NOP if not enabled) *
 *-------------------------------------------------------------------------*/
#ifdef DEBUG
   #define _DEBUG           sprintf(hi,"%s: Ok\n",__func__); Serial.print(hi);
   #define _DEBUGLIST(...)  sprintf(hi,__VA_ARGS__);Serial.print(hi);
#else
   #define _DEBUG _NOP
   #define _DEBUGLIST(...)  _DEBUG
#endif

#ifdef TRACE
   #define _TRACE           sprintf(hi,"%s: Ok\n",__func__); Serial.print(hi);
   #define _TRACELIST(...)  sprintf(hi,__VA_ARGS__);Serial.print(hi);
#else
   #define _TRACE _NOP
   #define _TRACELIST(...)  _TRACE
#endif

#ifdef INFO
   #define _INFO           sprintf(hi,"%s: Ok\n",__func__); Serial.print(hi);
   #define _INFOLIST(...)  sprintf(hi,__VA_ARGS__);Serial.print(hi);
#else
   #define _INFO _NOP
   #define _INFOLIST(...)  _INFO
#endif

#ifdef EXCP
   #define _EXCP           sprintf(hi,"%s: Ok\n",__func__); Serial.print(hi);
   #define _EXCPLIST(...)  sprintf(hi,__VA_ARGS__);Serial.print(hi);
#else
   #define _EXCP           _NOP
   #define _EXCPLIST(...)  _EXCP
#endif

/*---------------------------------------------------------------*
 * Pin Assignment                                                *
 *---------------------------------------------------------------*/
#define UP             2           //UP Switch
#define DOWN           3           //DOWN Switch
#define TXSW           4           //TX Switch

#define AIN0           6           //(PD6)
#define AIN1           7           //(PD7)

#define RX             8           //RX Switch

#define WSPR           9           //WSPR LED 
#define JS8           10           //JS8 LED
#define FT4           11           //FT4 LED
#define FT8           12           //FT8 LED

#define TX            13           //(PB5) TX LED
/*----------------------------------------------------------------*
 *  Global State Variables (Binary)                               *
 *----------------------------------------------------------------*/
#define TXON   0B00000001    //State of the TX
#define VOX    0B00000010    //Audio input detected
#define UPPUSH 0B00000100    //UP button (analog) pressed
#define DNPUSH 0B00001000    //DOWN button (analog) pressed
#define TXPUSH 0B00010000    //TXSW button (analog) pressed
#define CATTX  0B00100000    //TX turned on via CAT (disable VOX)
#define SAVEEE 0B01000000    //Mark of EEPROM updated
#define CWMODE 0B10000000    //CW Active

/*----------------------------------------------------------------*
 * Operating switch                                               *
 * ---------------------------------------------------------------*/
#define PUSHSTATE   0B00000001
#define SHORTPUSH   0B00000010    //simple push flag
#define LONGPUSH    0B00000100    //long push flag
#define INPROGRESS  0B00001000    //in progress mark


/*----------------------------------------------------------------*
 * General purpose global define                                  *
 * ---------------------------------------------------------------*/
#define BOUNCE_TIME 20          //mSec minimum to debounce
#define SHORT_TIME  500         //mSec minimum to consider long push
#define SI5351_REF  25000000UL  //change this to the frequency of the crystal on your si5351’s PCB, usually 25 or 27 MHz
#define CPU_CLOCK   16000000UL  //Processor clock
#define VOX_MAXTRY  10          //Max number of attempts to detect an audio incoming signal
#define CNT_MAX     65000       //Max count of timer1
#define FRQ_MAX     30000       //Max divisor for frequency allowed
#define BDLY        200         //Delay when blinking LED
#define DELAY_WAIT  BDLY*2      //Double Delay
#define DELAY_CAL   DELAY_WAIT/10
#define MAXMODE     5           //Max number of digital modes
#define MAXBLINK    4           //Max number of blinks
#define MAXBAND    10           //Max number of bands defined (actually uses 4 out of MAXBAND)
#define XT_CAL_F   33000 


#define INT0        0
#define INT1        1
#define INT2        2

/*------------------------------------------------------------------------*
 * Define EEPROM related configuration parameters, if EE is not defined   *
 * no EEPROM support is provided                                          *
 *------------------------------------------------------------------------*/
#ifdef EE
   #define EEPROM_CAL  10
   #define EEPROM_TEMP 30
   #define EEPROM_MODE 40
   #define EEPROM_BAND 50

   uint32_t tout=0;

   #define EEPROM_CLR    1   //Initialize EEPROM (only to be used to initialize contents)
   #define EEPROM_SAVE 100     //Signature of EEPROM being updated at least once
   #define EEPROM_TOUT 500     //Timeout in mSecs to wait till commit to EEPROM any change
#endif //EEPROM

/*-------------------------------------------------------------------------*
 * Define CW related configuration parameters                              *
 *-------------------------------------------------------------------------*/
#ifdef CW
   #define CWSHIFT        600
   #define CWSTEP         500
   #define MAXSHIFT     15000
   #define CWSLOT           5
#endif //CW

//*******************************[ VARIABLE DECLARATIONS ]*************************************


uint8_t  SSW=0;               //System SSW variable (to be used with getWord/setWord)
uint16_t mode=0;              //Default to mode=0 (FT8)
uint16_t Band_slot=0;         //Default to Bands[0]=40
uint16_t cal_factor=0;

unsigned long Cal_freq  = 1000000UL; // Calibration Frequency: 1 Mhz = 1000000 Hz

unsigned long f[MAXMODE]                  = { 7074000, 7047500, 7078000, 7038600, 7030000};   //Default frequency assignment   
const unsigned long slot[MAXBAND][MAXMODE]={{ 1840000, 1840000, 1842000, 1836600, 1810000},   //160m [0]
                                            { 3573000, 3575000, 3578000, 3568600, 3560000},   //80m [1]
                                            { 7074000, 7047500, 7078000, 7038600, 7030000},   //40m [2]
                                            {10136000,10140000,10130000,10138700,10106000},   //30m [3]
                                            {14074000,14080000,14078000,14095600,14060000},   //20m [4]
                                            {18100000,18104000,18104000,18104600,18096000},   //17m [5]
                                            {21074000,21140000,21078000,21094600,21060000},   //15m [6]                           
                                            {24915000,24915000,24922000,24924600,24906000},   //12m [7] FT4 equal to FT8                           
                                            {28074000,28074000,28078000,28124600,28060000},   //10m [8]                           
                                            {50310000,50310000,50318000,50293000,50060000}};  //6m  [9]          

                                                      
unsigned long freq      = f[Band_slot]; 
uint8_t       LED[4]    = {FT8,FT4,JS8,WSPR};
/*-------------------------------------*
 * Manage button state                 *
 *-------------------------------------*/
uint8_t       button[3]   ={0,0,0};
unsigned long downTimer[3]={PUSHSTATE,PUSHSTATE,PUSHSTATE};

#ifdef CW
unsigned long freqCW      = f[CWSLOT]; //default assignment consistent with digital mode's default, 40m
#endif //CW

//**********************************[ BAND SELECT ]************************************************

/*
 ADX can support up to 4 bands on board. Those 4 bands needs to be assigned to Band1 ... Band4 from supported 6 bands.
 To change bands press SW1 and SW2 simultaneously. Band LED will flash 3 times briefly and stay lit for the stored band. also TX LED will be lit to indicate
 that Band select mode is active. Now change band bank by pressing SW1(<---) or SW2(--->). When desired band bank is selected press TX button briefly to exit band select mode. 
 Now the new selected band bank will flash 3 times and then stored mode LED will be lit. 
 TX won't activate when changing bands so don't worry on pressing TX button when changing bands in band mode.
 Assign your prefered bands to B1,B2,B3 and B4
 Supported Bands are: 80m, 40m, 30m, 20m,17m, 15m
*/

#ifdef ONEBAND                       //If defined selects a single band to avoid mistakes with PA filter 
   uint16_t Bands[4]={40,40,40,40}; //All bands the same (change to suit needs)
#else
   uint16_t Bands[4]={40,30,20,17}; //Band1,Band2,Band3,Band4 (initial setup)
#endif //ONEBAND
/****************************************************************************************************************************************/
/*                                                     CODE INFRAESTRUCTURE                                                             */
/****************************************************************************************************************************************/

/*-------------------------------------------*
 * getWord                                   *
 * get boolean bitwise pseudo-variable       *
 *-------------------------------------------*/
bool getWord (uint8_t SysWord, uint8_t v) {
  return SysWord & v;
}
/*-------------------------------------------*
 * setSSW                                    *
 * set boolean bitwise pseudo-variable       *
 *-------------------------------------------*/
void setWord(uint8_t* SysWord,uint8_t v, bool val) {
  *SysWord = ~v & *SysWord;
  if (val == true) {
    *SysWord = *SysWord | v;
  }
}

#ifdef CAT

/*-----------------------------------------------------------------------------------------------------*
 *                                     CAT SubSystem                                                   *
 * cloned from uSDX (QCX-SSB) firmware                                                                 *                                    
 * adaptations and extensions by P.E.Colla (LU7DZ)                                                     *
 * At this point this code needs to work with FLRig as a HamLib server                                 *
 * ----------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------*
   CAT support inspired by Charlie Morris, ZL2CTM, contribution by Alex, PE1EVX, 
   source: http://zl2ctm.blogspot.com/2020/06/digital-modes-transceiver.html?m=1
   https://www.kenwood.com/i/products/info/amateur/ts_480/pdf/ts_480_pc.pdf
   Code excerpts from QCX-SSB by Guido (PE1NNZ)
   Mods by Pedro E. Colla(LU7DZ) 2022
 *----------------------------------------------------------------------------------------------------*/

void switch_RXTX(bool t);     //advanced definition for compilation purposes (interface only)
void Mode_assign();           //advanced definition for compilation purposes

//*--- Get Freq VFO A
void Command_GETFreqA()          //Get Frequency VFO (A)
{
  sprintf(hi,"FA%011ld;",freq);
  Serial.print(hi);
}
//*--- Set Freq VFO A
void setFreqCAT() {
  char Catbuffer[16];
  for (int i=2;i<13;i++) {
    Catbuffer[i-2]=CATcmd[i];
  }
  Catbuffer[11]='\0';
  freq=(uint32_t)(atol(Catbuffer)*1000/1000);
  
}

void Command_SETFreqA()          //Set Frequency VFO (A)
{
  setFreqCAT();
  Command_GETFreqA();
}

//*--- Get Freq VFO B
void Command_GETFreqB()          //Get Frequency VFO (B) -- fallback to VFO (A) until implementation of VFOA/B pair
{
  sprintf(hi,"FB%011ld;",freq);
  Serial.print(hi);
 
}
//*--- Set Freq VFO B
void Command_SETFreqB()          //Set Frequency VFO (B) -- fallback to VFO (B) until implementation for VFOA/B pair
{
  setFreqCAT();
  Command_GETFreqB();
}

//*--- Prepare and send response to IF; command
void Command_IF()               //General purpose status information command (IF), needs to be modified if RIT is implemented
{

  sprintf(hi,"IF%011ld00000+000000000%c%c0000000;",freq,txstatus(),modeTS480());   //Freq & mode the rest is constant (fake)
  Serial.print(hi);

}
//*--- Response to STEP command
void Command_ST() {       //STEP command Modify when STEP is implemented
   sprintf(hi,"ST01;");
   Serial.print(hi);
}

//*--- Get current Mode (only 2=USB and 3=CW are supported)

void Command_GetMD()      //Get MODE command, only USB (2) and CW (3) are supported, 4 digital modes (mode 0,1,2,3 are mapped as USB)
{
  sprintf(hi,"MD%c;",modeTS480());
  Serial.print(hi);
}

void Command_SetMD()      //Set MODE command, only USB (2) and CW (3) are supported
{
  if (CATcmd[2] != '3') {
     setWord(&SSW,CWMODE,false);
     mode=0;
     Mode_assign();
     sprintf(hi,"MD2;");    // at this time only USB is allowed, needs to be modified when CW is added
     
#ifdef CW     
  } else {
     setWord(&SSW,CWMODE,true);
     mode=4;
     Mode_assign();
     sprintf(hi,"MD3;");
#endif //CW
        
  }
  Serial.print(hi);
}

//*--- Place transceiver in RX mode
void Command_RX()
{
  setWord(&SSW,CATTX,false);
  switch_RXTX(LOW);
  Serial.print("RX0;");
}

//*--- Place transceiver in TX mode
void Command_TX()
{
  setWord(&SSW,CATTX,true);
  switch_RXTX(HIGH);
  Serial.print("TX0;");
}

//*--- Response for VOX command
void Command_VX()
{
  sprintf(hi,"VX%s;",(getWord(SSW,VOX)==true ? "1" : "0"));
  Serial.print(hi);
}

//*---- Translate ADX mode into the TS-480 coding for mode
char modeTS480() {

  if (mode==4) {
     return '3';
  }
  return '2';

}
char txstatus() {
  
  if (getWord(SSW,CATTX)==false) {
     return '0';
  }
  return '1';

  
}
//*--- Fake response for AS; command (not implemented)
void Command_AS() {

  sprintf(hi,"AS000%011ld%c;",freq,modeTS480());
  Serial.print(hi);
  return;
}
//*--- Fake response for XI; command (not implemented)
void Command_XI() {

  sprintf(hi,"XI%011ld%c0;",freq,modeTS480());
  Serial.print(hi);
  return;
}

//*--- Band change command (not implemented)
void Command_BChange(int c) { //Change band up or down

  //Do not return anything
}
/*---------------------------------------------------------------------------------------------
 *  CAT Main command parser and dispatcher
 *---------------------------------------------------------------------------------------------*/
void analyseCATcmd()
{
  char strcmd[4];

  const char *cID="ID";    
  const char *cIDr="ID020;";
  const char *cFR="FR"; const char *cFT="FT"; const char *cEX="EX";
  const char *cKY="KY"; const char *cXT="XT"; const char *cVX="VX";
  const char *cRU="RU"; const char *cPS="PS"; const char *cRD="RD";
  
#ifdef CAT_FULL_PROTOCOL

  const char *cISr="IS+0000;";
  const char *cPCr="PC005;";
  const char *cXOr="XO00000000000;";
  const char *cSUr="SU00000000000;";
  const char *cMRr="MR00000000000000000000000000000000000000000000000;";
  const char *cVDr="VD0100;";
  const char *cVGr="VG005;";
  const char *cSMr="SM00000;";
  const char *cPLr="PL000000;";
  const char *cRMr="RM00000;";
  const char *cLMr="LM00000;";
  const char *cSSr="SS0000000000000;";
  const char *cSM="SM"; const char *cRA="RA"; const char *cIS="IS";  const char *cAC="AC";  const char *cAG="AG";  const char *cSQ="SQ";
  const char *cMG="MG"; const char *cGT="GT"; const char *cNL="NL";  const char *cRT="RT";
  const char *cSH="SH"; const char *cCN="CN"; const char *cRL="RL";  const char *cPA="PA";  const char *cTN="TN";
  const char *cQR="QR"; const char *cLK="LK"; const char *cSC="SC"; 
  const char *cNB="NB"; const char *cBC="BC"; const char *cNR="NR";  const char *cBY="BY";
  const char *cFS="FS"; const char *cPR="PR"; const char *cRS="RS";  const char *cAN="AN";  const char *cMF="MF";  const char *cTO="TO";
  const char *cTS="TS"; const char *cBS="BS"; const char *cCA="CA";  const char *cCT="CT";
  const char *cUL="UL"; const char *cAI="AI"; const char *cFL="FL";
  
  
  const char *cML="ML"; const char *cOP="OP"; const char *cPB="PB"; const char *cDL="DL";
  const char *cRG="RG"; const char *cKS="KS"; const char *cTY="TY"; const char *cMC="MC";
  const char *cSD="SD"; const char *cFW="FW";
  const char *cDN="DN"; const char *cRC="RC"; const char *cCH="CH"; const char *cSV="SV"; const char *cMW="MW";  const char *cQT="QT";
  const char *cSR="SR"; const char *cUP="UP"; const char *cVR="VR"; const char *cVV="VV";
  const char *cPC="PC"; const char *cPL="PL"; const char *cSU="SU"; const char *cMR="MR"; const char *cXO="XO"; const char *cVG="VG";
  const char *cSS="SS"; const char *cRM="RM"; const char *cLM="LM"; const char *cVD="VD"; const char *cSL="SL"; const char *cQI="QI";

#endif //CAT_FULL_PROTOCOL
  
  if ((CATcmd[0] == 'F') && (CATcmd[1] == 'A') && (CATcmd[2] == ';'))  {Command_GETFreqA(); return;}
  if ((CATcmd[0] == 'F') && (CATcmd[1] == 'A') && (CATcmd[13] == ';')) {Command_SETFreqA(); return;}
  if ((CATcmd[0] == 'F') && (CATcmd[1] == 'B') && (CATcmd[2] == ';'))  {Command_GETFreqB(); return;}
  if ((CATcmd[0] == 'F') && (CATcmd[1] == 'B') && (CATcmd[13] == ';')) {Command_SETFreqB(); return;}
  if ((CATcmd[0] == 'I') && (CATcmd[1] == 'F') && (CATcmd[2] == ';'))  {Command_IF(); return;}
  if ((CATcmd[0] == 'M') && (CATcmd[1] == 'D') && (CATcmd[2] == ';'))  {Command_GetMD(); return;}
  if ((CATcmd[0] == 'M') && (CATcmd[1] == 'D') && (CATcmd[3] == ';'))  {Command_SetMD(); return;}
  if ((CATcmd[0] == 'R') && (CATcmd[1] == 'X'))                        {Command_RX(); return;}
  if ((CATcmd[0] == 'T') && (CATcmd[1] == 'X'))                        {Command_TX(); return;}

#ifdef CAT_FULL_PROTOCOL  

  if ((CATcmd[0] == 'V') && (CATcmd[1] == 'X'))                        {Command_VX(); return;} 
  if ((CATcmd[0] == 'A') && (CATcmd[1] == 'S'))                        {Command_AS(); return;}
  if ((CATcmd[0] == 'S') && (CATcmd[1] == 'T'))                        {Command_ST(); return;}      //Step when implemented
  if ((CATcmd[0] == 'X') && (CATcmd[1] == 'I'))                        {Command_XI(); return;}      //Step when implemented
  if ((CATcmd[0] == 'B') && (CATcmd[1] == 'D'))                        {Command_BChange(-1); return;}
  if ((CATcmd[0] == 'B') && (CATcmd[1] == 'U'))                        {Command_BChange(+1); return;}

#endif  //CAT_FULL_PROTOCOL
  
  strcmd[0]=CATcmd[0];
  strcmd[1]=CATcmd[1];
  strcmd[2]=0x00;

  if (strcmp(strcmd,cID)==0)                                                      {sprintf(hi,"%s",cIDr);Serial.print(hi);return;}
  if (strcmp(strcmd,cFR)==0 || strcmp(strcmd,cFT)==0 || strcmp(strcmd,cEX)==0 ||
      strcmp(strcmd,cVX)==0 || strcmp(strcmd,cXT)==0 || strcmp(strcmd,cKY)==0 ||
      strcmp(strcmd,cRU)==0 || strcmp(strcmd,cPS)==0 || strcmp(strcmd,cRD)==0)    {sprintf(hi,"%s",CATcmd);Serial.print(hi);return;}

#ifdef CAT_FULL_PROTOCOL

  if (strcmp(strcmd,cIS)==0)                                           {sprintf(hi,"%s",cISr);Serial.print(hi);return;}
  if (strcmp(strcmd,cPC)==0)                                           {sprintf(hi,"%s",cPCr);Serial.print(hi);return;}
  if (strcmp(strcmd,cPL)==0)                                           {sprintf(hi,"%s",cPLr);Serial.print(hi);return;}
  if (strcmp(strcmd,cSU)==0)                                           {sprintf(hi,"%s",cSUr);Serial.print(hi);return;}
  if (strcmp(strcmd,cMR)==0)                                           {sprintf(hi,"%s",cMRr);Serial.print(hi);return;}
  if (strcmp(strcmd,cXO)==0)                                           {sprintf(hi,"%s",cXOr);Serial.print(hi);return;}
  if (strcmp(strcmd,cVD)==0)                                           {sprintf(hi,"%s",cVDr);Serial.print(hi);return;}
  if (strcmp(strcmd,cVG)==0)                                           {sprintf(hi,"%s",cVGr);Serial.print(hi);return;}
  if (strcmp(strcmd,cSS)==0)                                           {sprintf(hi,"%s",cSSr);Serial.print(hi);return;}
  if (strcmp(strcmd,cSM)==0)                                           {sprintf(hi,"%s",cSMr);Serial.print(hi);return;}
  if (strcmp(strcmd,cRM)==0)                                           {sprintf(hi,"%s",cRMr);Serial.print(hi);return;}
  if (strcmp(strcmd,cLM)==0)                                           {sprintf(hi,"%s",cLMr);Serial.print(hi);return;}

  if (strcmp(strcmd,cSL)==0 || strcmp(strcmd,cSH)==0 || strcmp(strcmd,cCN)==0 ||
      strcmp(strcmd,cRL)==0 || strcmp(strcmd,cPA)==0 || strcmp(strcmd,cTN)==0 ||
      strcmp(strcmd,cQR)==0 || strcmp(strcmd,cLK)==0 || strcmp(strcmd,cSC)==0)   {sprintf(hi,"%s00;",strcmd);Serial.print(hi);return;}

     
  if (strcmp(strcmd,cNB)==0 || strcmp(strcmd,cBC)==0 || strcmp(strcmd,cNR)==0 ||
      strcmp(strcmd,cSM)==0 || strcmp(strcmd,cRT)==0 || strcmp(strcmd,cBY)==0 ||
      strcmp(strcmd,cFS)==0 || strcmp(strcmd,cPR)==0 || strcmp(strcmd,cRS)==0 ||
      strcmp(strcmd,cAN)==0 || strcmp(strcmd,cMF)==0 || strcmp(strcmd,cTO)==0 ||
      strcmp(strcmd,cTS)==0 || strcmp(strcmd,cBC)==0 || strcmp(strcmd,cBS)==0 ||
      strcmp(strcmd,cXT)==0 || strcmp(strcmd,cCA)==0 || strcmp(strcmd,cCT)==0 ||
      strcmp(strcmd,cUL)==0 || strcmp(strcmd,cAI)==0 || strcmp(strcmd,cFL)==0 )   {sprintf(hi,"%s0;",strcmd);Serial.print(hi);return;}

      

      
  if (strcmp(strcmd,cSM)==0 || strcmp(strcmd,cRA)==0 || strcmp(strcmd,cAC)==0  ||
      strcmp(strcmd,cGT)==0 || strcmp(strcmd,cNL)==0 || strcmp(strcmd,cML)==0  ||
      strcmp(strcmd,cOP)==0 || strcmp(strcmd,cPB)==0 || strcmp(strcmd,cDL)==0  ||
      strcmp(strcmd,cRG)==0 || strcmp(strcmd,cKS)==0 || strcmp(strcmd,cTY)==0  ||
      strcmp(strcmd,cMC)==0 || strcmp(strcmd,cMG)==0 )                                                    {sprintf(hi,"%s000;",strcmd);Serial.print(hi);return;}
      
  if (strcmp(strcmd,cAG)==0 || strcmp(strcmd,cSQ)==0 || strcmp(strcmd,cAC)==0 ||
      strcmp(strcmd,cSD)==0 || strcmp(strcmd,cFW)==0)                             {sprintf(hi,"%s0000;",strcmd);Serial.print(hi);return;}
  
  if (strcmp(strcmd,cRC)==0 || strcmp(strcmd,cDN)==0 || strcmp(strcmd,cCH)==0 ||
      strcmp(strcmd,cSV)==0 || strcmp(strcmd,cMW)==0 || strcmp(strcmd,cQI)==0 ||
      strcmp(strcmd,cSR)==0 || strcmp(strcmd,cUP)==0 || strcmp(strcmd,cVR)==0 ||
      strcmp(strcmd,cVV)==0)                                                      {return;}

#endif //CAT_FULL_PROTOCOL  
              
  Serial.print("?;");
}

/*-----------------------------------------------------------------*
 * serialEvent                                                     *
 * Process incoming characters from the serial buffer assemble     *
 * commands and process responses according with the TS480 cat prot*
 *-----------------------------------------------------------------*/
volatile uint8_t cat_ptr = 0;
volatile char serialBuffer[CATCMD_SIZE];

void serialEvent(){

  char strCmd[7];
  char const *strResp="RX0;ID020;";
  sprintf(strCmd,"RX;ID;");
  
  int nread=Serial.available();
  if (nread<=0) return;

  int rc=Serial.readBytes(buf,nread);
  if (rc<=0) {return;}
  buf[rc]=0x0;

  int k=0;
  for (int j;j<rc;j++){
    if (buf[j]!=0x0d && buf[j]!=0x0a) { 
        serialBuffer[k++]=buf[j];
    }    
  }
  
#ifdef DEBUG  
   _TRACELIST("%s cleansed buffer=%s len=%d\n",__func__,serialBuffer,rc);
#endif //DEBUG  

  if (strcmp((const char*)serialBuffer,strCmd)==0) { //coincidence

#ifdef DEBUG
     _TRACELIST("%s Hit RX;ID; string\n",__func__);
#endif //DEBUG
     
     Serial.write(strResp,10);
     setWord(&SSW,CATTX,false);
     switch_RXTX(LOW);
     delay(2);
  } else { 
    for (int i=0;i<k;i++) {
       char data=serialBuffer[i];
       CATcmd[cat_ptr++] = data;

#ifdef DEBUG
       _TRACELIST("%s data=%c CATcmd[%d]=%c\n",__func__,data,i,CATcmd[i]);
#endif //DEBUG

       if(data == ';'){      
         CATcmd[cat_ptr] = '\0'; // terminate the array
         cat_ptr = 0;            // reset for next CAT command

#ifdef DEBUG
        _TRACELIST("%s cmd(%s)\n",__func__,CATcmd);
#endif //DEBUG

        analyseCATcmd();
        delay(2);
      } else {
        if(cat_ptr > (CATCMD_SIZE - 1)){
           Serial.print("?;");  //Overrun, send error
           cat_ptr = 0;         //Overrun, cleanse buffer       
        }
      }  
   }   
 }
}

#endif //CAT
/**********************************************************************************************/
/*                                   Si5351 Management                                        */
/**********************************************************************************************/

/*--------------------------------------------------------------------------------------------*
 * Initialize DDS SI5351 object
 *--------------------------------------------------------------------------------------------*/
Si5351 si5351;

void setup_si5351() {
//------------------------------- SET SI5351 VFO -----------------------------------  
// The crystal load value needs to match in order to have an accurate calibration
//---------------------------------------------------------------------------------
 

long cal = XT_CAL_F;
  
  #ifdef DEBUG
    _INFO;
  #endif //DEBUG

  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);// SET For Max Power
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_2MA);// Set for reduced power for RX 
  
}

/*---------------------------------------------------------------------------------------------*
 * Switch between RX and TX
 *---------------------------------------------------------------------------------------------*/
void switch_RXTX(bool t) {  //t=False (RX) : t=True (TX)

#ifdef DEBUG
  if (t != getWord(SSW,TXON)) {
      _TRACELIST("%s (%s)\n",__func__,BOOL2CHAR(t));
  } 
#endif //DEBUG  
  
  if (t) {    //Set to TX

/*-----------------------------------*
 *               TX                  *
 *-----------------------------------*/
     digitalWrite(RX,LOW);
     si5351.output_enable(SI5351_CLK1, 0);   //RX off
     
#ifdef CW
     long int freqtx=(getWord(SSW,CWMODE)==false ? freq : freq+CWSHIFT);
#else
     long int freqtx=freq;
#endif //CW          
     
#ifdef DEBUG     
     _TRACELIST("%s TX+ f=%ld\n",__func__,freqtx);
#endif //DEBUG
     
     si5351.set_freq(freqtx*100ULL, SI5351_CLK0);
     si5351.output_enable(SI5351_CLK0, 1);   // TX on
     digitalWrite(TX,HIGH);
     setWord(&SSW,TXON,HIGH);

     
     return;
  }

/*------------------------------------*
 *                RX                  *
 *------------------------------------*/
    if (getWord(SSW,CATTX)==true) { return;}  //if the PTT is managed by the CAT subsystem get out of the way.

    digitalWrite(RX,HIGH);
    si5351.output_enable(SI5351_CLK0, 0);   //TX off    
    
#ifdef DEBUG
    if (getWord(SSW,TXON)==HIGH) {
       _TRACELIST("%s RX+ f=%ld\n",__func__,freq);
    }
#endif //DEBUG
    
    si5351.set_freq(freq*100ULL, SI5351_CLK1);
    si5351.output_enable(SI5351_CLK1, 1);   //RX on
    digitalWrite(TX,0); 
    setWord(&SSW,TXON,LOW);
    setWord(&SSW,VOX,LOW);

/*---------------------------------------------------------*
 * set to master frequency                                 *
 *---------------------------------------------------------*/
  
}
/**********************************************************************************************/
/*                                      LED Management                                        */
/**********************************************************************************************/
/*----- 
 * Turn off all LEDs
 */
void resetLED() {               //Turn-off all LEDs

   digitalWrite(WSPR, LOW); 
   digitalWrite(JS8, LOW); 
   digitalWrite(FT4, LOW); 
   digitalWrite(FT8, LOW); 

   #ifdef DEBUG
   _TRACE;
   #endif //DEBUG   

 
}

/*-----
 * Set a particular LED ON
 */
void setLED(uint8_t LEDpin,bool clrLED) {      //Turn-on LED {pin}
   
   (clrLED==true ? resetLED() : void(_NOP)); 
   digitalWrite(LEDpin,HIGH);

#ifdef DEBUG
   _TRACELIST("%s(%d)\n",__func__,LEDpin);
#endif //DEBUG   

}

/*-------
 * Blink a given LED
 */
void blinkLED(uint8_t LEDpin) {    //Blink 3 times LED {pin}

#ifdef DEBUG
   _TRACELIST("%s (%d)\n",__func__,LEDpin);
#endif //DEBUG   
   
   uint8_t n=(MAXBLINK-1);

   while (n>0) {
       digitalWrite(LEDpin,HIGH);
       delay(BDLY);
       digitalWrite(LEDpin,LOW);
       delay(BDLY);
       n--;

       #ifdef WDT       
          wdt_reset();
       #endif //WDT       
   }
}

/*-----
 * LED on callibration mode
 */
void callibrateLED(){           //Set callibration mode

   digitalWrite(WSPR, HIGH); 
   digitalWrite(FT8, HIGH);
   delay(DELAY_CAL);        
}

/**********************************************************************************************/
/*                               PushButton Management                                        */
/**********************************************************************************************/
/*---
 * ISR Handler
 * Handle push button interrupt
 */

ISR (PCINT2_vect) {

  long int timerDown=0;
  byte     v=0;
  
  for (byte p=INT0;p<=INT2;p++){ 

      #ifdef DEBUG
         _TRACELIST("%s check pin(%d)\n",__func__,p);
      #endif //DEBUG      

      switch (p) {
        case INT0 : {v=UPPUSH;break;}
        case INT1 : {v=DNPUSH;break;}
        case INT2 : {v=TXPUSH;break;}
      }
      bool pstate=PIND & v;    
      if (pstate != getWord(button[p],PUSHSTATE)) {   //Evaluate which pin changed

//*--- Change detected

         #ifdef DEBUG
            _TRACELIST("%s pin(%d) [%d]->[%d]\n",__func__,p,getWord(button[p],PUSHSTATE),pstate);
         #endif //DEBUG         
         
         setWord(&button[p],PUSHSTATE,pstate);
         if (pstate == LOW) {
           downTimer[p]=millis();
         } else {
           timerDown=millis()-downTimer[p];
           if (timerDown<BOUNCE_TIME) {
              #ifdef DEBUG
                 _TRACELIST("%s pin(%d) too short, ignored!\n",__func__,p);
              #endif //DEBUG   
            
           }
           setWord(&SSW,v,true);
           if (timerDown<SHORT_TIME){
              setWord(&button[p],SHORTPUSH,true);
              setWord(&button[p],LONGPUSH,false);
              
              #ifdef DEBUG
                 _TRACELIST("%s pin(%d) <SP>\n",__func__,p);
              #endif //DEBUG

           } else {
              setWord(&button[p],SHORTPUSH,false);
              setWord(&button[p],LONGPUSH,true);       
              #ifdef DEBUG
                 _TRACELIST("%s pin(%d) <LP>\n",__func__,p);
              #endif //DEBUG   
           }       
         }
      }
  }
}

/*----------------------------------------------------------*
 * Manually turn TX while pressed                           *
 *----------------------------------------------------------*/
bool getTXSW();  //prototype for forward reference
void ManualTX(){
   
    bool buttonTX=getTXSW();
    switch_RXTX(HIGH);
    while(buttonTX==LOW) {

       #ifdef WDT      
          wdt_reset();
       #endif //WDT
       buttonTX=getTXSW();
                
    }
    switch_RXTX(LOW);
}

/*---------------------------------------------------------------------*
 * getSwitchPL
 * Detect and clear the Long push condition on both UP/DOWN buttons 
 *---------------------------------------------------------------------*/
bool getSwitchPL(uint8_t pin) {

//*--- pin can be 2,3,4

    byte p=pin-2;
    byte v=0;
    switch(p) {
      case INT0: {v=UPPUSH;break;}
      case INT1: {v=DNPUSH;break;}
      case INT2: {v=TXPUSH;break;}
    }

    if (getWord(SSW,v) == true && getWord(button[p],LONGPUSH)==true) {

       #ifdef DEBUG
          _TRACELIST("%s (%d): <PL>\n",__func__,p);
       #endif //DEBUG   

       setWord(&SSW,v,false);
       setWord(&button[p],LONGPUSH,false);
       return LOW;
    } else {    
       return HIGH;
    }           
}
/*----------------------------------------------------------*
 * get value for a digital pin and return after debouncing  *
 *----------------------------------------------------------*/
bool getSwitch(uint8_t pin) {

//*--- pin can be 2,3,4

    byte p=pin-2;
    byte v=0;
    switch(p) {
      case INT0: {v=UPPUSH;break;}
      case INT1: {v=DNPUSH;break;}
      case INT2: {v=TXPUSH;break;}
    }

    if (getWord(SSW,v) == true && getWord(button[p],SHORTPUSH)==true) {

       #ifdef DEBUG
          _TRACELIST("%s (%d): <SP>\n",__func__,p);
       #endif //DEBUG

       setWord(&SSW,v,false);
       setWord(&button[p],SHORTPUSH,false);
       return LOW;
    } else {    
       return HIGH;
    }           
}
/*----------------------------------------------------------*
 * read UP switch
 *----------------------------------------------------------*/
bool getUPSSW() {

    return getSwitch(UP);

}
/*----------------------------------------------------------*
 * read DOWN Switch
 *----------------------------------------------------------*/
bool getDOWNSSW() {
  
    return getSwitch(DOWN); 

}
/*--------------------------------------------------------------*
 * getTXSW() -- read TXSW switch
 * This switch still required debouncing but might operate
 * over long pulsing periods because of the manual TX function
 * and CW operation. It doesn't require to distinguish between 
 * short and long pulse though.
 *---------------------------------------------------------------*/
bool getTXSW() {

    if ( getWord(button[INT2],PUSHSTATE)==LOW && (millis()-downTimer[INT2]>BOUNCE_TIME) ) {
       return LOW;
    }
    return HIGH;
}
/*----------------------------------------------------------*
 * Calibration function
 *----------------------------------------------------------*/
void Calibration(){


  #ifdef DEBUG
     _TRACE;
  #endif //DEBUG
  
  resetLED();
  uint8_t  n=4;
   
  while (n>0) {

     #ifdef WDT
        wdt_reset();
     #endif //WDT
         
     callibrateLED();
     n--;

     #ifdef WDT
        wdt_reset();
     #endif //WDT
  }

     #ifdef EE  
        EEPROM.get(EEPROM_CAL,cal_factor);
     #endif //EEPROM
  
  while (true) {

     #ifdef WDT
        wdt_reset();
     #endif //WDT
     
     if (getUPSSW() == LOW) {
        cal_factor = cal_factor - 100;

        #ifdef EE
           EEPROM.put(EEPROM_CAL, cal_factor); 
        #endif //EEPROM
        
        si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
    
  // Set Calibration CLK output
  
        si5351.set_freq(Cal_freq * 100, SI5351_CLK0);
        si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_2MA); // Set for lower power for calibration
        si5351.set_clock_pwr(SI5351_CLK0, 1); // Enable the clock for calibration
     } 
   

     if (getDOWNSSW() == LOW) {
        cal_factor = cal_factor + 100;

        #ifdef EE
           EEPROM.put(EEPROM_CAL, cal_factor);    
        #endif //EEPROM
        
        si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);

 // Set Calbration Clock output
           
        si5351.set_freq(Cal_freq * 100ULL, SI5351_CLK0);
        si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_2MA); // Set for lower power for Calibration
        si5351.set_clock_pwr(SI5351_CLK0, 1); // Enable clock2 
     } 

  }

}
#ifdef EE
/*------------------------------------------------------------------------------*
 * updateEEPROM                                                                 *
 * selectively sets values into EEPROM                                          *
 *------------------------------------------------------------------------------*/
void updateEEPROM() {

uint16_t save=EEPROM_SAVE;

   EEPROM.put(EEPROM_TEMP,save);
   EEPROM.put(EEPROM_CAL,cal_factor);
   EEPROM.put(EEPROM_MODE,mode);
   EEPROM.put(EEPROM_BAND,Band_slot);

   #ifdef DEBUG
      _TRACELIST("%s save(%d) cal(%d) m(%d) slot(%d)\n",__func__,save,cal_factor,mode,Band_slot);
   #endif //DEBUG

    setWord(&SSW,SAVEEE,false);

}
#endif //EE

/**********************************************************************************************/
/*                               Operational state management                                 */
/**********************************************************************************************/

/*----------------------------------------------------------*
 * Mode assign                                              *
 *----------------------------------------------------------*/
#ifdef CW
void displayFrequencyCW();
#endif //CW

void Mode_assign(){

   freq=f[mode];
   if (mode==4) {
      setWord(&SSW,CWMODE,true);

#ifdef CW
      freqCW=freq;    
      displayFrequencyCW();
#endif //CW
      
   } else {
      setWord(&SSW,CWMODE,false);
      setLED(LED[mode],true);

   }

/*--------------------------------------------*   
 * Update master frequency here               *
 *--------------------------------------------*/

   #ifdef EE
      tout=millis();
      setWord(&SSW,SAVEEE,true);
   #endif //EE

   #ifdef DEBUG
      _INFOLIST("%s mode(%d) f(%ld)\n",__func__,mode,f[mode]);
   #endif //DEBUG   
}
/*----------------------------------------------------------*
 * Frequency assign (band dependant)                        *
 *----------------------------------------------------------*/
void Freq_assign(){

    uint8_t b;
    uint16_t Band=Bands[Band_slot];
    
    switch(Band) {
      case 160: b=0;break;
      case 80 : b=1;break;
      case 40 : b=2;break;
      case 30 : b=3;break;
      case 20 : b=4;break;
      case 17 : b=5;break;
      case 15 : b=6;break;
      case 12 : b=7;break;
      case 10 : b=8;break;
      case 6  : b=9;break;
      default : b=2;break;     //40m is the default
    }
    for (int i=0;i<MAXMODE;i++) {
      f[i]=slot[b][i];

      #ifdef WDT      
         wdt_reset();    //Although quick don't allow loops to occur without a wdt_reset()
      #endif //WDT      

    }


/*---------------------------------------*          
 * Update master frequency here          *
 *---------------------------------------*/
    freq=f[Band_slot];

    #ifdef EE
       tout=millis();
       setWord(&SSW,SAVEEE,true);
    #endif //EE

    #ifdef DEBUG
       _TRACELIST("%s B(%d) b[%d] m[%d] slot[%d] f[0]=%ld f[1]=%ld f[2]=%ld f[3]=%ld f=%ld\n",__func__,Band,b,mode,Band_slot,f[0],f[1],f[2],f[3],freq);
    #endif //DEBUG   
}

/*----------------------------------------------------------*
 * Band assignment based on selected slot                   *
 *----------------------------------------------------------*/
void Band_assign(){

    resetLED();
    blinkLED(LED[3-Band_slot]);
    
    delay(DELAY_WAIT); 
    
    Freq_assign();
    Mode_assign();

    #ifdef DEBUG
       _INFOLIST("%s m(%d) slot(%d) f=%d\n",__func__,mode,Band_slot,freq);
    #endif //DEBUG   
  
}
/*----------------------------------------------------------*
 * Select band to operate
 *----------------------------------------------------------*/
void Band_Select(){
  
   resetLED();

   #ifdef DEBUG
      _INFOLIST("%s slot(%d) LED(%d)\n",__func__,Band_slot,LED[3-Band_slot]);
   #endif //DEBUG
   
   blinkLED(LED[3-Band_slot]);
   setLED(LED[3-Band_slot],true);
   
   while (true) {
   #ifdef WDT
      wdt_reset();
   #endif //WDT      
      
   bool upButton   = getUPSSW();
   bool downButton = getDOWNSSW();
   bool txButton   = getTXSW();
          
   if ((upButton == LOW) && (downButton == HIGH)) {
       Band_slot=(Band_slot-1)%4;
       setLED(LED[3-Band_slot],true);

       #ifdef DEBUG
          _INFOLIST("%s slot(%d)\n",__func__,Band_slot);
       #endif //DEBUG   
   } 
   
   if ((upButton == HIGH) && (downButton == LOW)) {
      Band_slot=(Band_slot+1)%4;
      setLED(LED[3-Band_slot],true);

      #ifdef DEBUG
         _INFOLIST("%s slot(%d)\n",__func__,Band_slot);
      #endif //DEBUG   

   }                                               
   if (txButton == LOW) {
      digitalWrite(TX,0);
      
      #ifdef EE
         tout=millis();
         setWord(&SSW,SAVEEE,true);
      #endif //EE

      Band_assign();
/*------------------------------------------*
 * Update master frequency                  *
 *------------------------------------------*/ 
      return; 
   } 
}
}
#ifdef CW
/*----------------------------------------------------------------*
 * select LED representation according with the frequency shift   *
 * with the center (initial) qrp calling frequency                *
 *----------------------------------------------------------------*/
void displayFrequencyCW() {

  if (freq==freqCW) {
     setLED(JS8,true);
     setLED(FT4,false);
     #ifdef DEBUG
        _TRACELIST("%s set QRP QRG f=%ld\n",__func__,freq);
     #endif //DEBUG   
     return;
  }

  long int df=freq-freqCW;

/*-----------------------------------------------------------*
 * compute a fancy LED lighting to signal where the frequency*
 * is located compared with the QRP calling frequency (only  *
 * valid for CW mode when activated                          *
 *-----------------------------------------------------------*/
  if ((df>     0) && (df<= 5000)) {setLED(FT4,true);}
  if ((df>  5000) && (df<=10000)) {setLED(FT4,true);setLED(FT8,false);}
  if ((df> 10000)) {setLED(FT8,true);}
  if (df<0) {
     df=abs(df);
     if ((df>     0) && (df<= 5000)) {setLED(JS8,true);}
     if ((df>  5000) && (df<=10000)) {setLED(JS8,true);setLED(WSPR,false);}
     if ((df> 10000)) {setLED(WSPR,true);}
  }
}
#endif //CW

#ifdef CW
/*-------------------------------------------------------------*
 * setFrequencyCW tuning function when CW mode is active       *
 *-------------------------------------------------------------*/
void setFrequencyCW(int f) {

  long int step=f*CWSTEP;
  
  #ifdef DEBUG
     _TRACELIST("%s f=%ld\n",__func__,freq+step);
  #endif //DEBUG   

#ifdef SHIFTLIMIT

  if ((freq+step)>(freqCW+MAXSHIFT)) {

     #ifdef DEBUG
        _TRACELIST("%s (%d): %ld out of band\n",__func__,step,freq+step);
     #endif //DEBUG   

     blinkLED(FT8);
     setLED(FT8,true);
     return;
  }
  if ((freq+step)<(freqCW-MAXSHIFT)) {

     #ifdef DEBUG
        _TRACELIST("%s step=%ld f=%ld out of band\n",__func__,step,freq+step);
     #endif //DEBUG   
     
     blinkLED(WSPR);
     setLED(WSPR,true);
     return;
  }
  #ifdef DEBUG
     _TRACELIST("%s f=%ld):\n",__func__,freq+step);
  #endif //DEBUG
     
#endif //SHIFTLIMIT

  freq=freq+step; 
  displayFrequencyCW();
  return;
  
}
#endif //CW
/*---------------------------------------------------------------------------*
 *  checkMode
 *  manage change in mode during run-time
 *---------------------------------------------------------------------------*/
void checkMode() {
  
bool upButton     = getUPSSW();
bool downButton   = getDOWNSSW();
bool txButton     = getTXSW();
bool upButtonPL   = getSwitchPL(UP);
bool downButtonPL = getSwitchPL(DOWN);

/*------------------------------------------------------*
 * Manage change to and from CW mode by detecting a     *
 * long push press of the UP button (to enter CW) and a *
 * long push press of the DOWN button (to exit CW)      *
 *------------------------------------------------------*/
  #ifdef CW

/*--------------------------------*
 * UP button long press && !CW    *
 * Start CW mode                  *
 *--------------------------------*/
  
  if (upButtonPL == LOW && getWord(SSW,CWMODE)==false) {
     mode=4;
     Mode_assign();
        
     #ifdef DEBUG
        _INFOLIST("%s CW+ f=%ld\n",__func__,freq);
     #endif //DEBUG   

  }

/*--------------------------------*
 * DN button long press && CW     *
 * Exit CW mode                   *
 *--------------------------------*/
  if (downButtonPL == LOW && getWord(SSW,CWMODE)== true) {
     mode=0;
     Mode_assign();
     
     #ifdef DEBUG
        _INFOLIST("%s CW- f=%ld\n",__func__,freq);
     #endif //DEBUG   
     
  }

/*-----------------------------------------------------------*  
 * While in CW mode UP means frequency up by CWSTEP and DOWN *
 * means frequency down by CWSTEP Hz. The tunning range is   *
 * +/- MAXRIT                                                *
 *-----------------------------------------------------------*/
  if (downButton == LOW && getWord(SSW,CWMODE)==true) {   
     setFrequencyCW(-1);
     
     #ifdef DEBUG
        _INFOLIST("%s f+ f=%ld\n",__func__,freq);      
     #endif //DEBUG   
  }

  if (upButton == LOW && getWord(SSW,CWMODE)==true) {     
     setFrequencyCW(+1);
     
     #ifdef DEBUG
        _INFOLIST("%s f- f=%ld\n",__func__,freq);     
     #endif //DEBUG   
  }
//*-------------------------[CW Implementation]------------------

  #endif //CW  

/*--------------------------------*
 * TX button short press          *
 * Transmit mode                  *
 *--------------------------------*/
  if ((txButton == LOW) && (getWord(SSW,TXON)==false)) {

     #ifdef DEBUG
        _INFOLIST("%s TX+\n",__func__);
     #endif //DEBUG
        
     ManualTX(); 
     
     #ifdef DEBUG
        _INFOLIST("%s TX-\n",__func__);
     #endif //DEBUG   
  }

/*------------------------------------------------------------*
 * while in CW mode just block band and mode changes          *
 *------------------------------------------------------------*/

#ifdef CW

  if (getWord(SSW,CWMODE)==true) {
     return;
  }
#endif //CW

/*-------------------------------------------------------------*
 * manage band, mode and  calibration selections               *
 *-------------------------------------------------------------*/
  if ((upButton == LOW)&&(downButton == LOW)&&(getWord(SSW,TXON)==false)) {
     Band_Select();
     
     #ifdef DEBUG
      _INFOLIST("%s U+D f=%ld",__func__,freq);
     #endif //DEBUG 
  }

  if ((upButton == LOW)&&(downButton == HIGH)&&(getWord(SSW,TXON)==false)) {

      mode=(mode-1)%4;
      
      #ifdef DEBUG
         _TRACELIST("%s m+(%d)\n",__func__,mode);
      #endif //DEBUG
      
      #ifdef EE
         EEPROM.put(EEPROM_MODE, mode); 
      #endif //EEPROM     

      Mode_assign();
  
  } 
   

  if ((upButton == HIGH) && (downButton == LOW)&&(getWord(SSW,TXON)==false)) {
      
      mode=(mode+1)%4;
      
      #ifdef DEBUG
         _TRACELIST("%s m-(%d)\n",__func__,mode);
      #endif //DEBUG   

      #ifdef EE
         tout=millis();
         setWord(&SSW,SAVEEE,true);
      #endif //EE Avoid the tear and wear of the EEPROM because of successive changes
     
      Mode_assign();
  } 


 
}
/*---------------------------------------------------------------------------------*
 * keepAlive()
 * Reference function for debugging purposes, called once per loop()
 *---------------------------------------------------------------------------------*/
void keepAlive() {

#ifdef DEBUG

   /* Code here -- Reserved for future usage -- hook into loop()*/

#endif //DEBUG
   
}

/**********************************************************************************************/
/*                            Initialization and Setup                                        */
/**********************************************************************************************/

/*----------------------------------------------------------*
 * Initialization function from EEPROM                      *
 *----------------------------------------------------------*/
void INIT(){


#ifdef EE

 uint16_t temp;
 uint16_t save=EEPROM_SAVE;

 
 EEPROM.get(EEPROM_TEMP,temp);
 
 #ifdef EEPROM_CLR
    temp=-1;
 #endif //EEPROM_CLR  
 
 if (temp != save){

    updateEEPROM();
    
    #ifdef DEBUG
       _TRACELIST("%s EEPROM Reset cal(%d) m(%d) slot(%d)\n",__func__,cal_factor,mode,Band_slot);
    #endif //DEBUG
    
 } else {

   /*-----------------------------------------------*
    * get configuration initialization from EEPROM  *            *
    ------------------------------------------------*/
   
  EEPROM.get(EEPROM_CAL,cal_factor);

/*---- Kludge Fix   
 *     to overcome wrong initial values, should not difficult calibration
 */
  if (cal_factor < -31000) {
      cal_factor=0;
  }
/* end of kludge */
   
  EEPROM.get(EEPROM_MODE,mode);
  EEPROM.get(EEPROM_BAND,Band_slot);
  
  #ifdef DEBUG
     _TRACELIST("%s EEPROM Read cal(%d) m(%d) slot(%d)\n",__func__,cal_factor,mode,Band_slot);
  #endif //DEBUG   
}  

#endif // EE

  Band_assign();
  Freq_assign();
  Mode_assign();
  switch_RXTX(LOW);   //Turn-off transmitter, establish RX LOW
}
/*--------------------------------------------------------------------------*
 * definePinOut
 * isolate pin definition on a board conditional procedure out of the main
 * setup() flow
 *--------------------------------------------------------------------------*/
void definePinOut() {

   pinMode(UP,   INPUT);
   pinMode(DOWN, INPUT);
   pinMode(TXSW, INPUT);
   pinMode(RX,   OUTPUT);
   pinMode(WSPR, OUTPUT);
   pinMode(JS8,  OUTPUT);
   pinMode(FT4,  OUTPUT);
   pinMode(FT8,  OUTPUT);  
   pinMode(TX,   OUTPUT);
   pinMode(AIN0, INPUT);  //PD6=AN0 must be grounded
   pinMode(AIN1, INPUT);  //PD7=AN1=HiZ

#ifdef DEBUG
   _INFO;
#endif //DEBUG      
}
//*************************************[ SETUP FUNCTION ]************************************** 
void setup()
{

   #ifdef DEBUG
      Serial.begin(BAUD);
      _DEBUGLIST("%s ADX Firmware V(%s)\n",__func__,VERSION);
   #endif //DEBUG

   #ifdef CAT
      Serial.begin(BAUD,SERIAL_8N2);
      while (!Serial) {    //Wait till Serial port ready  
      #ifdef WDT      
         wdt_reset();
      #endif //WDT         
      
      }
      Serial.setTimeout(SERIAL_TOUT);
      
   #endif //CAT   

   definePinOut();

   PCICR  |= B00000100; // Enable interrupts at PD port
   PCMSK2 |= B00011100; // Signal interrupts for D2,D3 and D4 pins (UP/DOWN/TX)
   setWord(&button[INT0],PUSHSTATE,HIGH);
   setWord(&button[INT1],PUSHSTATE,HIGH);
   setWord(&button[INT2],PUSHSTATE,HIGH);

   #ifdef DEBUG
      _INFOLIST("%s INT ok\n",__func__);
   #endif //DEBUG   

   setup_si5351();   
   INIT();

   if ( getDOWNSSW() == LOW ) {
      Calibration();
   }
  
/*--------------------------------------------------------*
 * initialize the timer1 as an analog comparator          *
 * this is the main feature of the VOX/Modulation scheme  *
 *--------------------------------------------------------*/
  TCCR1A = 0x00;
  TCCR1B = 0x01; // Timer1 Timer 16 MHz
  TCCR1B = 0x81; // Timer1 Input Capture Noise Canceller
  ACSR |= (1<<ACIC);  // Analog Comparator Capture Input
//--------------------------------------------------------*
  
  pinMode(AIN1, INPUT); //PD7 = AN1 = HiZ, PD6 = AN0 = 0

  #ifdef DEBUG
     _INFOLIST("%s Timer1 set\n",__func__);
  #endif //DEBUG   
  
  switch_RXTX(LOW);
  Mode_assign(); 

  #ifdef WDT
     wdt_disable();
     wdt_enable(WDTO_8S);
  #endif //WDT

  #ifdef DEBUG
     _INFOLIST("%s completed ok\n",__func__);
  #endif //DEBUG   
}
//*=*=*=*=*=*=*=*=*=*=*=*=*=[ END OF SETUP FUNCTION ]*=*=*=*=*=*=*=*=*=*=*=*=

//***************************[ Main LOOP Function ]**************************
//*                                                                         *
//*                                                                         *
//***************************************************************************
void loop()
{  

//*--- Debug hook
    keepAlive();

//*--- changes in mode, band, frequency and operational status
    checkMode();


    #ifdef EE
//*--- if EEPROM enabled check if timeout to write has been elapsed

    if((millis()-tout)>EEPROM_TOUT && getWord(SSW,SAVEEE)==true ) {
       updateEEPROM();
    }
    #endif //EEPROM

    #ifdef CAT 
//*--- if CAT enabled check for serial events
       serialEvent();
    #endif //CAT

    #ifdef WDT
//*--- if WDT enabled reset the watchdog
       wdt_reset();
    #endif //WDT

/*----------------------------------------------------------------------------------*
 * main transmission loop                                                           *
 * Timer1 (16 bits) with no pre-scaler (16 MHz) is checked to detect zero crossings *
 * if there is no overflow the frequency is calculated                              *
 * if activity is detected the TX is turned on                                      *
 * TX mode remains till no further activity is detected (operate like a VOX command)*
 *----------------------------------------------------------------------------------*/
uint16_t n = VOX_MAXTRY;

    setWord(&SSW,VOX,false);
    while (n>0){                                 //Iterate up to 10 times looking for signal to transmit

    #ifdef WDT
    wdt_reset();
    #endif //WDT
    
    TCNT1 = 0;                                  //While this iteration is performed if the TX is off 
    
    while (ACSR &(1<<ACO)){                     //the receiver is operating with autonomy
       if (TCNT1>CNT_MAX) break;
    }
    
    while ((ACSR &(1<<ACO))==0){
       if (TCNT1>CNT_MAX) break;
    }
    
    TCNT1 = 0;
    
    while (ACSR &(1<<ACO)){
      if (TCNT1>CNT_MAX) break;
    }
    
    uint16_t d1 = ICR1;  
    
    while ((ACSR &(1<<ACO))==0){
      if (TCNT1>CNT_MAX) break;
    } 
    
    while (ACSR &(1<<ACO)){
      if (TCNT1>CNT_MAX) break;
    }
    
    uint16_t d2 = ICR1;

/*-----------------------------------------------------*
 * end of waveform measurement, now check what is the  *
 * input frequency                                     *
 *-----------------------------------------------------*/
    
    if (TCNT1 < CNT_MAX){
       //if ((d2-d1) == 0) break;
       unsigned long codefreq = CPU_CLOCK/(d2-d1);
       

       if ((codefreq < FRQ_MAX) && (codefreq > 0)){
          if (getWord(SSW,VOX) == false){
             switch_RXTX(HIGH);
          }

       si5351.set_freq(((freq + codefreq) * 100ULL), SI5351_CLK0); 
       setWord(&SSW,VOX,true);

       #ifdef WDT
          wdt_reset();
       #endif //WDT
          
       }
    } else {
       n--;
    }
    #ifdef CAT 
//*--- if CAT enabled check for serial events (again)
       serialEvent();
    #endif
    
    #ifdef WDT
       wdt_reset();
    #endif //WDT
 }

/*---------------------------------------------------------------------------------*
 * when out of the loop no further TX activity is performed, therefore the TX is   *
 * turned off and the board is set into RX mode                                    *
 *---------------------------------------------------------------------------------*/
 if (getWord(SSW,CATTX)==true) {
    switch_RXTX(LOW);
    setWord(&SSW,VOX,false);
    setWord(&SSW,TXON,false);
 }   

 #ifdef WDT
    wdt_reset();
 #endif //WDT     

}

//****************************[ END OF MAIN LOOP FUNCTION ]*******************************
//********************************[ END OF FIRMWARE ]*************************************
