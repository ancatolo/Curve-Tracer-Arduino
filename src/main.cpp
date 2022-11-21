/*
Arduino Nano/Pro Curve Tracer
ver 1.0
*/

#include <Arduino.h>
#include "SimpleILI9341.h"
#include <SPI.h>

bool ExecSerialTx = false;  // Send scans to PC
bool SendAdcValues = false; // Send ADC values to PC

int prev_x = 0, prev_y = 0; // Graph line position
char str[16]; // For converting and concaternating

const int TFT_WID = 320;  // Display resolution
const int TFT_HGT = 240;

const int pin_DAC_CS = 10;      // Arduino Nano IO pins where ADCs & DAC CS are connected
const int pin_ADC_NPN_Vcc = 0;
const int pin_ADC_NPN_Vce = 1;
const int pin_ADC_PNP_Vce = 2;
const int pin_ADC_PNP_Vcc = 3;
const int pin_Adc_12V = 7;
const int pin_Adc_Bat = 6;

const int R1 = 33;  // ADC (NPN_Vcc, NPN_Vce, PNP_Vce & PNP_Vcc) input divider upper resistor k-ohms
const int R2 = 47;  // ADC (NPN_Vcc, NPN_Vce, PNP_Vce & PNP_Vcc) input divider lower resistor k-ohms
const int R3 = 100; // load resistor (collector) ohms
const int R4 = 68;  // DAC (DacBase & DacVcc) op amp feedback upper resistor k-ohms
const int R5 = 33;  // DAC (DacBase & DacVcc) op amp feedback lower resistor k-ohms
const int R6 = 3;   // battery volts divider upper resistor k-ohms (actual resistor 2.8k * 1.07 = 3k)
const int R7 = 35;  // battery volts divider lower resistor k-ohms (actual resistor 33k * 1.07 = 35k)

const int AdcMax = 1023;        // Arduino ADC max
const int AdcVref = 5;          // ADC Vref in V
const int MinDacVcc = 0;        // Min load (collector) DAC value 
const int MinDacBase = 0;       // Min base/gate DAC value
const int MidDacVcc = 128;      // Mid load (collector) DAC value 
const int MidDacBase = 128;     // Mid base/gate DAC value
const int MaxDacVcc = 255;      // Max load (collector) DAC value 
const int MaxDacBase = 255;     // Max base/gate DAC value
const int GatePMosFactor = 242; // DAC start gate value for PMosfet
const int BaseNpnFactor = 12;   // DAC base value when NPN transistor starts to coduct
const int BasePnpFactor = 232;  // DAC base value when PNP transistor starts to coduct

// Formatting parameters for menus
const int MenuCol1 = 10;
const int MenuCol2 = 140;
const int MenuCol3 = 180;
const int MenuCol4 = 215;
const int MenuCol5 = 255;
const int MenuRow1 = 45;
const int MenuRowHeight = 25;
const int MenuBox1 = MenuRow1 - 17;
const int MenuBoxSize = MenuRowHeight - 1;
const int MenuEraseWidth = 32;

// Default value for parameters possible to edit in Setup Menu on Curve Tracer Display.
int NoOfCurves = 8;  // No of curves between Min and Max
int mAmax = 50;      // Ic for top of graph view on display in mA
int MinIbase = 0;    // Default min base current for NPN/PNP in uA
int MaxIbase = 200;  // Default max base current for NPN/PNP in uA
int MinVgate =  0;   // Default min gate voltage for FETs in 100s mV
int MaxVgate = 30;   // Default max gate voltage for FETs in 100s mV

// be careful using these inside if statements:
#define SerialPrint(s) {if (ExecSerialTx) Serial.print(s);}
#define SerialPrintLn(s) {if (ExecSerialTx) Serial.println(s);}

enum TkindDUT {tkNothing, tkPNP, tkNPN, tkPMOSFET, tkNMOSFET, tkPDiode, tkNDiode}; // DUT kind, automatically selected
TkindDUT curKind = tkNothing;

enum TclassDUT {tcBipolar, tcMOSFET, tcDiode}; // DUT class, selected from display Main Menu
TclassDUT CurDUTclass = tcBipolar;

const uint8_t bmpPNP[] PROGMEM = {
  21, 0, // width
  30, 0, // height
  0x3F, 0xFF, 0xF9, 0xFF, 0xFF, 0xCF, 0xFF, 0xFE, 0x7F, 0xFF, 0xF1, 0xFF, 0x3F, 0xC7, 0xF9, 0xFF,
  0x1F, 0xCF, 0xFC, 0x7E, 0x7F, 0xF1, 0xF3, 0xFF, 0xC7, 0x9F, 0xFF, 0x1C, 0xFF, 0xFC, 0x67, 0xFF,
  0xF1, 0x3F, 0xFF, 0xC1, 0xFF, 0xFF, 0x00, 0x1F, 0xFC, 0x00, 0xFF, 0x93, 0xFF, 0xF0, 0x9F, 0xFE,
  0x0C, 0xFF, 0xC0, 0x67, 0xFC, 0x07, 0x3F, 0xF0, 0x39, 0xFF, 0x83, 0xCF, 0xF8, 0x1E, 0x7F, 0x8D,
  0xF3, 0xF8, 0xFF, 0x9F, 0xCF, 0xFF, 0xFE, 0x7F, 0xFF, 0xF3, 0xFF, 0xFF, 0x9F, 0xFF, 0xFC
};

const uint8_t bmpNPN[] PROGMEM = {
  21, 0, // width
  30, 0, // height
  0xFF, 0xFF, 0xE7, 0xFF, 0xFF, 0x3F, 0xFF, 0xF9, 0xFF, 0xFF, 0xCF, 0xE7, 0xFC, 0x7F, 0x3F, 0xC7,
  0xF9, 0xFC, 0x7F, 0xCF, 0xC7, 0xFE, 0x7C, 0x7F, 0xF3, 0xC7, 0xFF, 0x9C, 0x7F, 0xFC, 0xC7, 0xFF,
  0xE4, 0x7F, 0xFF, 0x07, 0xFC, 0x00, 0x7F, 0xE0, 0x03, 0xFF, 0xFE, 0x0F, 0xFF, 0xF2, 0x3B, 0xFF,
  0x98, 0x9F, 0xFC, 0xE0, 0x7F, 0xE7, 0x83, 0xFF, 0x38, 0x0F, 0xF9, 0x80, 0x7F, 0xCF, 0x01, 0xFE,
  0x7E, 0x0F, 0xF3, 0xFC, 0x3F, 0xFF, 0xF9, 0xFF, 0xFF, 0xCF, 0xFF, 0xFE, 0x7F, 0xFF, 0xF0
};

const uint8_t bmpNMOSFET[] PROGMEM = {
  23, 0, // width
  34, 0, // height
  0xFF, 0xFF, 0xF9, 0xFF, 0xFF, 0xF3, 0xFF, 0xFF, 0xE7, 0xFF, 0xFF, 0xCF, 0xFF, 0xFF, 0x9F, 0xFF,
  0xFF, 0x3F, 0xFF, 0xFE, 0x7F, 0xFF, 0xFC, 0xFE, 0x60, 0x01, 0xFC, 0xC0, 0x03, 0xF9, 0x9F, 0xFF,
  0xF3, 0x3F, 0xFF, 0xE7, 0xFF, 0xFF, 0xCF, 0xFB, 0xFF, 0x99, 0xC7, 0xFF, 0x32, 0x0F, 0xFE, 0x60,
  0x01, 0xFC, 0xC0, 0x03, 0xF9, 0x90, 0x67, 0xF3, 0x38, 0xCF, 0xE7, 0xFD, 0x9F, 0xCF, 0xFF, 0x3F,
  0x99, 0xFE, 0x7F, 0x33, 0xFC, 0x00, 0x60, 0x00, 0x00, 0xC0, 0x03, 0xFF, 0xFF, 0xE7, 0xFF, 0xFF,
  0xCF, 0xFF, 0xFF, 0x9F, 0xFF, 0xFF, 0x3F, 0xFF, 0xFE, 0x7F, 0xFF, 0xFC, 0xFF, 0xFF, 0xF9, 0xFF,
  0xFF, 0xF0
};

const uint8_t bmpPMOSFET[] PROGMEM = {
  23, 0, // width
  34, 0, // height
  0x3F, 0xFF, 0xFE, 0x7F, 0xFF, 0xFC, 0xFF, 0xFF, 0xF9, 0xFF, 0xFF, 0xF3, 0xFF, 0xFF, 0xE7, 0xFF,
  0xFF, 0xCF, 0xFF, 0xFF, 0x9F, 0xFF, 0xFF, 0x00, 0x0C, 0xFE, 0x00, 0x19, 0xFF, 0xFF, 0x33, 0xFF,
  0xFE, 0x67, 0xFF, 0xFF, 0xCF, 0xFF, 0xBF, 0x9F, 0xFC, 0x73, 0x3F, 0xE0, 0xE6, 0x7F, 0x80, 0x0C,
  0xFE, 0x00, 0x19, 0xFC, 0x07, 0x33, 0xF9, 0x8E, 0x67, 0xF3, 0xDF, 0xCF, 0xE7, 0xFF, 0x9F, 0xCF,
  0xF3, 0x3F, 0x9F, 0xE6, 0x7F, 0x00, 0x0C, 0x00, 0x00, 0x18, 0x00, 0xFF, 0xFF, 0xF9, 0xFF, 0xFF,
  0xF3, 0xFF, 0xFF, 0xE7, 0xFF, 0xFF, 0xCF, 0xFF, 0xFF, 0x9F, 0xFF, 0xFF, 0x3F, 0xFF, 0xFE, 0x7F,
  0xFF, 0xFC
};

const uint8_t bmpPDiodeBig[] PROGMEM = {
  20, 128, // width run-length encoded
  24, 0, // height
  0x00, 0x09, 0x02, 0x12, 0x02, 0x12, 0x02, 0x12, 0x02, 0x12, 0x02, 0x12, 0x02, 0x09, 0x14, 0x01, 0x12, 0x03, 0x10, 0x05,
  0x0E, 0x07, 0x0C, 0x09, 0x0A, 0x0B, 0x08, 0x0D, 0x06, 0x0F, 0x04, 0x11, 0x02, 0x09, 0x28, 0x09, 0x02, 0x12, 0x02, 0x12,
  0x02, 0x12, 0x02, 0x12, 0x02, 0x12, 0x02, 0x09
};

const uint8_t bmpNDiodeBig[] PROGMEM = {
  20, 128, // width run-length encoded
  24, 0, // height
  0x00, 0x09, 0x02, 0x12, 0x02, 0x12, 0x02, 0x12, 0x02, 0x12, 0x02, 0x12, 0x02, 0x09, 0x28, 0x09, 0x02, 0x11, 0x04, 0x0F,
  0x06, 0x0D, 0x08, 0x0B, 0x0A, 0x09, 0x0C, 0x07, 0x0E, 0x05, 0x10, 0x03, 0x12, 0x01, 0x14, 0x09, 0x02, 0x12, 0x02, 0x12,
  0x02, 0x12, 0x02, 0x12, 0x02, 0x12, 0x02, 0x09
};

//-------------------------------------------------------------------------
// SetDac
//   sets either of the DACs
//   Bits MCP4802 (Max  255): x001 nnnn nnnn 0000 (value << 4)
//   Bits MCP4812 (Max 1023): x001 nnnn nnnn nn00 (value << 2)
//   Bits MCP4822 (Max 4095): x001 nnnn nnnn nnnn (no shift)
//   
//-------------------------------------------------------------------------

void SetDac(uint8_t value, uint8_t cmd) {
  SPI.beginTransaction(SPISettings(4000000UL, MSBFIRST, SPI_MODE0));

  digitalWrite(pin_DAC_CS, LOW);
  SPI.transfer((value >> 4) | cmd);
  SPI.transfer(value << 4); // MCP4802: (value << 4), MCP4812: (value << 2), MCP4822: No shift
  digitalWrite(pin_DAC_CS, HIGH);

  SPI.endTransaction();
}

//-------------------------------------------------------------------------
// SetDacVcc
//   sets the collector DAC output
//-------------------------------------------------------------------------

void SetDacVcc(uint8_t value, int tDelay) {
  SetDac(value, 0x90);
  if (tDelay > 0)
    delay(tDelay);
}

//-------------------------------------------------------------------------
// SetDacBase
//   sets the base DAC output
//-------------------------------------------------------------------------

void SetDacBase(uint8_t value, int tDelay) {
  SetDac(value, 0x10);
  if (tDelay > 0)
    delay(tDelay);
}

//-------------------------------------------------------------------------
// GetAdcSmooth
//   mean of N readings of ADC
//-------------------------------------------------------------------------

int GetAdcSmooth(int pin) {
  int i, sum;
  const int n = 6;
  sum = 0;
  for (i = 1; i <= n; i++) {
    sum += analogRead(pin);
    delayMicroseconds(1);
  }
  return sum / n;
}

//-------------------------------------------------------------------------
// readSupply
//   calculate supply voltage of Arduino
//   uses internal bandgap reference
//-------------------------------------------------------------------------

long readSupply() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA, ADSC));
  result = ADCL;
  result |= ADCH << 8;
  result = 1125300L / result; // Back-calculate AVcc in mV
  return result;
}

//-------------------------------------------------------------------------
// SquareWave
//    debugging - generates a square wave with both of the DACs at maximum level
//-------------------------------------------------------------------------

void SquareWave(void) {
  while (true) {
    SetDacVcc(MinDacVcc, 0);
    SetDacBase(MinDacBase, 50);
    SetDacVcc(MaxDacVcc, 0);
    SetDacBase(MaxDacBase, 50);
  }
}

//-------------------------------------------------------------------------
// TurnOffLoad
//   sets load current to default values
//-------------------------------------------------------------------------

void TurnOffLoad(TkindDUT kind) {

  switch (kind) {
    case tkPMOSFET:
    case tkPNP:
    case tkPDiode:
      SetDacBase(MinDacBase, 0);
      SetDacVcc(MaxDacVcc, 0);
      break;
    case tkNPN:
    case tkNMOSFET:
    case tkNDiode:
      SetDacBase(MaxDacBase, 0);
      SetDacVcc(MinDacVcc, 0);
      break;
    default: // tkNothing
      SetDacBase(MidDacBase, 0);
      SetDacVcc(MidDacVcc, 0);
  }
}

//-------------------------------------------------------------------------
// TestDeviceKind
//    is there a DUT inserted?
//    returns kind of device
//-------------------------------------------------------------------------

TkindDUT TestDeviceKind() {
  bool NpnOn = false, NpnOff = false, PnpOn = false, PnpOff = false; // On state for NPN/PNP with base on/off
  int TestCur = 40; // Test current to indentify DUT in 117 uA steps (voltage drop over resistor R3 in 11.7 mV steps)
  TkindDUT kind;

  SetDacVcc(MidDacVcc, 0); // Set DACs for ON test of both NPN & PNP with medium DacVcc
  SetDacBase(MidDacBase, 30);
  if (analogRead(pin_ADC_NPN_Vcc) - analogRead(pin_ADC_NPN_Vce) > TestCur) {
    NpnOn = true;
  }
  if (analogRead(pin_ADC_PNP_Vce) - analogRead(pin_ADC_PNP_Vcc) > TestCur) {
    PnpOn = true;
  }

  SetDacBase(MinDacBase, 30); // Set BASE DAC for base OFF test of NPN 
  if (analogRead(pin_ADC_NPN_Vcc) - analogRead(pin_ADC_NPN_Vce) > TestCur) {
    NpnOff = true;
  }
  SetDacBase(MaxDacBase, 30); // Set BASE DAC for base OFF test of PNP
  if (analogRead(pin_ADC_PNP_Vce) - analogRead(pin_ADC_PNP_Vcc) > TestCur) {
    PnpOff = true;
  }

  if (CurDUTclass == tcDiode && !NpnOn && !PnpOn) { // Test with max DacVcc, needed for zener
    SetDacVcc(MaxDacVcc, 0); // Set DACs for base OFF test of NPN
    SetDacBase(MinDacBase, 30); 
    if (analogRead(pin_ADC_NPN_Vcc) - analogRead(pin_ADC_NPN_Vce) > TestCur) {
      NpnOff = true;
    }
    SetDacVcc(MinDacVcc, 0);// Set DACs for base OFF test of PNP
    SetDacBase(MaxDacBase, 30);
    if (analogRead(pin_ADC_PNP_Vce) - analogRead(pin_ADC_PNP_Vcc) > TestCur) {
      PnpOff = true;
    }
  }

// Automatically select DUT kind based on selected CurDUTclass and NpnOn, NpnOff, PnpOn & PnpOff
  switch (CurDUTclass) {
    case tcBipolar:
      if (!PnpOff && PnpOn) kind = tkPNP;
      else if (!NpnOff && NpnOn) kind = tkNPN;
      else kind = tkNothing;
      break;

    case tcMOSFET:
      if (!NpnOff && NpnOn) kind = tkNMOSFET;
      else if (!PnpOff && PnpOn) kind = tkPMOSFET;
      else kind = tkNothing;
      break;

    case tcDiode:
      if (NpnOff || NpnOn) kind = tkNDiode;
      else if (PnpOff || PnpOn) kind = tkPDiode;
      else kind = tkNothing;
      break;
  }

  TurnOffLoad(kind);
  return kind;
}

//-------------------------------------------------------------------------
// InitGraph
//   draws the grid background of the graph
//-------------------------------------------------------------------------

void InitGraph(TkindDUT kind) {
  long ix, x, iy, y;
  int step;

  ClearDisplay(TFT_BLACK);

  DrawStringAt(2, TFT_HGT - 4, "0", SmallFont, TFT_CYAN);
  DrawLine(0, 0, 0, TFT_HGT, TFT_DARKGREY);
  DrawLine(0, TFT_HGT - 1, TFT_WID, TFT_HGT - 1, TFT_DARKGREY);

// x-axix 1 V grid steps
  for (ix = 1; ix <= 12; ix++) {
    x = TFT_WID * ix * R1 / (R2 + R1) / AdcVref;
    ILI9341SetCursor(x + 2, TFT_HGT - 3);
    if (kind == tkPNP || kind == tkPMOSFET) {
      DrawString("-", SmallFont, TFT_CYAN);
    }
    DrawInt(ix, SmallFont, TFT_CYAN);
    DrawString("V", SmallFont, TFT_CYAN);
    DrawLine(x, 0, x, TFT_HGT, TFT_DARKGREY);
  }

// Different y-axis grid steps with different max current, steps in mA
  switch (mAmax) {
    case 10:  step =  2; break;
    case 20:  step =  4; break;
    case 30:  step =  5; break;
    case 40:  step =  8; break;
    case 50:  step = 10; break;
    case 60:  step = 10; break;
    case 70:  step = 10; break;
    case 80:  step = 10; break;
    case 90:  step = 15; break;
    default:  step = 20;
  }

  for (iy = step; iy <= mAmax; iy += step) {
    y = TFT_HGT - 1 - TFT_HGT * iy / mAmax;
    ILI9341SetCursor(2, y + 8);
    if (ix > 0 && (kind == tkPNP || kind == tkPMOSFET)) {
      DrawString("-", SmallFont, TFT_CYAN);
    }
    DrawInt(iy, SmallFont, TFT_CYAN);
    DrawString("mA", SmallFont, TFT_CYAN);
    DrawLine(0, y, TFT_WID, y, TFT_DARKGREY);
  }
}

//-------------------------------------------------------------------------
// Graph
//   draws one step of the curve
//   base is in 2 uA steps
//   Vcc, Vce in ADC counts
//     Collector current in 117 uA steps: 12V / 1023 ADC-steps / 100R = 117
//-------------------------------------------------------------------------

void Graph(bool NewCurve, TkindDUT kind, int Vcc, int Vce) {
  long i; // Display x-coordinate, Voltage in 11,7 mV steps
  long j; // Display y-coordinate, Current through load (R3) in 117 uA steps
  static int px, py; // Variable to smooth the curve
  int Adc_12V = GetAdcSmooth(pin_Adc_12V);

  switch (kind) {
    case tkNPN:
    case tkNMOSFET:
    case tkNDiode:
      i = Vce;
      j = Vcc - Vce;
      break;
    case tkPNP:
    case tkPMOSFET:
    case tkPDiode:
      i = Adc_12V - Vce;
      j = Vce - Vcc;
      break;
    case tkNothing:
      break;
  }

  i = TFT_WID * i / (AdcMax); // Adjust i to TFT_WID
  j = j * (R2 + R1) * 48000 / R3 / R1 / (AdcMax); // convert j to 100s of uA through load resistor R3
  j = TFT_HGT - 1 - TFT_HGT * j / (mAmax * 10); // Adjust j to TFT_HGT

  if (j > TFT_HGT - 1) j = TFT_HGT - 1;
  if (i > TFT_WID - 25) i = TFT_WID - 25; // Make space for curve label to the right

// Smooth the curve
  if (NewCurve) { // Dont draw line to first postion on a new curve
    px = i * 2;
    py = j * 2;
  } else { // Draw line
    px = px / 2 + i;
    py = py / 2 + j;
    i = px / 2;
    j = py / 2;
    DrawLine(prev_x, prev_y, i, j, TFT_WHITE);
  }
  prev_x = i;
  prev_y = j;
}

//-------------------------------------------------------------------------
// GetNpnGain
//   calc NPN gain, hFE
//   Base current in 1,66 uA steps: (12V - 0,6V) / 27000 / 255 = 1.66 uA
//   Collector current in 117 uA steps: 12V / 1023 ADC-steps / 100R = 117 uA
//-------------------------------------------------------------------------

int GetNpnGain() {
  int base, StartBase, hFE;
  int VccStart = 8; // Starting current for hFE calc. (117 uA steps)
  int VccEnd = 100; // Current for hFE calc. one (117 uA steps)
  SetDacVcc(MaxDacVcc, 0);
  SetDacBase(MinDacBase, 20);

  for (base = MinDacBase; base <= MaxDacBase; base++) {
    SetDacBase(base, 2);
    if (GetAdcSmooth(pin_ADC_NPN_Vcc) - GetAdcSmooth(pin_ADC_NPN_Vce) >= VccStart) { // Start point
      StartBase = base;
      break;
    }
  }
  for (base = MinDacBase; base <= MaxDacBase; base++) {
    SetDacBase(base, 5);
    if (GetAdcSmooth(pin_ADC_NPN_Vcc) - GetAdcSmooth(pin_ADC_NPN_Vce) >= VccEnd) { // Collector current through load resistor R3 (100R) in 0.117mA steps
      hFE = float((VccEnd - VccStart) * 117.0) / ((base - StartBase) * 166.0 / 100.0);
      TurnOffLoad(tkNPN);

      if (base - StartBase <= 5) {
        return 9999; // hFE too high to calculate
      } else {
        return (hFE);
      }
    }
  }
  return 0;
}

//-------------------------------------------------------------------------
// GetPnpGain
//   calc PNP gain, hFE
//   Base current in 1,66 uA steps: (12V - 0,6V) / 27000 / 255 = 1,66 uA
//   Collector current in 117 uA steps: 12V / 1023 ADC-steps / 100R = 117 uA
//-------------------------------------------------------------------------

int GetPnpGain() {
  int base, StartBase, hFE;
  int VccStart = 8; // Starting current for hFE calc. (117 uA steps)
  int VccEnd = 100; // Current for hFE calc. one (117 uA steps)
  SetDacVcc(MinDacVcc, 0);
  SetDacBase(MaxDacBase, 20);

  for (base = MinDacBase; base <= MaxDacBase; base++) {
    SetDacBase(MaxDacBase - base, 1);
    if (GetAdcSmooth(pin_ADC_PNP_Vce) - GetAdcSmooth(pin_ADC_PNP_Vcc) >= VccStart) {// Start point
      StartBase = base;
      break;
    }
  }
  SetDacBase(MaxDacBase, 10);
  for (base = MinDacBase; base <= MaxDacBase; base++) {
    SetDacBase(MaxDacBase - base, 1);
    if (GetAdcSmooth(pin_ADC_PNP_Vce) - GetAdcSmooth(pin_ADC_PNP_Vcc) >= VccEnd) { // Collector current through load resistor R3 (100R) in 117 uA steps
      hFE = float((VccEnd - VccStart) * 117.0) / ((base - StartBase) * 166.0 / 100.0);
      TurnOffLoad(tkPNP);

      if (base - StartBase <= 5) {
        return 9999; // hFE too high to calculate
      } else {
        return hFE;
      }
    }
  }
  return 0;
}

//-------------------------------------------------------------------------
// GetNMosfetThreshold
//   calc threshold V of MOSFET ; result in mV
//   Vth = DacBase * (R4 + R5) / R5 * 10 / MaxDacBase
//   Vth = DacBase * (68 + 33) * 10 / 33 / 255
//   Vth = DacBase * 117 / 255
//-------------------------------------------------------------------------

int GetNMosfetThreshold() {
  int gate;
  SetDacVcc(MaxDacVcc, 0);
  SetDacBase(MinDacBase, 20);
  for (gate = MinDacBase; gate <= MaxDacBase; gate++) {
    SetDacBase(gate, 1); // gate is approx in 50s of mV
    if (GetAdcSmooth(pin_ADC_NPN_Vcc) - GetAdcSmooth(pin_ADC_NPN_Vce) > 10) { // Voltage drop over load resistor R3 (100R) in 11.7 mV steps
      TurnOffLoad(tkNMOSFET);
      return float((gate * 11700.0) / MaxDacBase);
   }
  }
  return 0;
}

//-------------------------------------------------------------------------
// GetPMosfetThreshold
//   calc threshold V of MOSFET ; result in mV
//   Vth = DacBase * (R4 + R5) / R5 * 10 / MaxDacBase
//   Vth = DacBase * (68 + 33) * 10 / 33 / 255
//   Vth = DacBase * 117 / 255
//-------------------------------------------------------------------------

 int GetPMosfetThreshold() {
  int gate;
  SetDacVcc(MinDacVcc, 0);
  SetDacBase(MaxDacBase, 20);
  for (gate = 0; gate <= MaxDacBase; gate++) {
    SetDacBase((GatePMosFactor) - gate, 1);  // gate is approx in 50s of mV
    if (GetAdcSmooth(pin_ADC_PNP_Vce) - GetAdcSmooth(pin_ADC_PNP_Vcc) > 10) { // Voltage drop over load resistor R3 (100R) in 11.7 mV steps
      TurnOffLoad(tkPMOSFET);
      return float((gate * 11700.0) / MaxDacBase);
    }
  }
  return 0;
}

//-------------------------------------------------------------------------
// GetNDiodeForwardVoltage
//   calc forward voltage of Diode ; result in mV
//
//   Formula for Vf in 100s of mV:
//     Vf = ADC * 10 * (R2 + R1)*AdcVref / ADC_MAX / R1
//     Vf = ADC * 11.7 / 100
//-------------------------------------------------------------------------


int GetNDiodeForwardVoltage() {
  int Vf;
  SetDacVcc(MinDacVcc, 20);
  for (Vf = MinDacVcc; Vf <= MaxDacVcc; Vf++) {
    SetDacVcc(Vf, 1);
    if (GetAdcSmooth(pin_ADC_NPN_Vcc) - GetAdcSmooth(pin_ADC_NPN_Vce) > 10) { // Voltage drop over load resistor R3 in 11.7 mV steps
      int AdcVcc = GetAdcSmooth(pin_ADC_NPN_Vcc);
      TurnOffLoad(tkNDiode);
      return float((AdcVcc - 10) * 11.7);
    }
  }
  return 0;
}

//-------------------------------------------------------------------------
// GetPDiodeForwardVoltage
//   calc forward voltage of Diode ; result in mV
//   Formula for Vf in 100s of mV:
//     Vf = ADC * 10 * (R2 + R1)*AdcVref / ADC_MAX / R1
//     Vf = ADC * 11.7 / 100
//-------------------------------------------------------------------------

int GetPDiodeForwardVoltage() {
  int Vf;
//  int Adc_12V = GetAdcSmooth(pin_Adc_12V);
  SetDacVcc(MaxDacVcc, 20);
  for (Vf = MaxDacVcc; Vf >= MinDacVcc; Vf--) {
    SetDacVcc(Vf, 1);
    if (GetAdcSmooth(pin_ADC_PNP_Vce) - GetAdcSmooth(pin_ADC_PNP_Vcc) > 10) { // Voltage drop over load resistor R3 in 11.7 mV steps
      int AdcVce = GetAdcSmooth(pin_ADC_PNP_Vce);
      TurnOffLoad(tkPDiode);
      return float((AdcMax - AdcVce - 10) * 11.7);
    }
  }
  return 0;
}

//-------------------------------------------------------------------------
// EndScan
//   Calculate and draw BJT gain / MOSFET threshold / Diode forward voltage
//-------------------------------------------------------------------------

void EndScan(TkindDUT kind) {
  int i = 0;

  switch (kind) {
    case tkPNP:
      DrawBox((TFT_WID - 115), 0, 115, 16, TFT_BLACK); // Black background for readability
      DrawStringAt((TFT_WID - 115), 13, "PNP", MediumFont, TFT_GREEN);
      DrawString(" hFE ", MediumFont, TFT_GREEN);
      DrawInt(GetPnpGain(), MediumFont, TFT_GREEN);
      break;

    case tkNPN:
      DrawBox((TFT_WID - 115), 0, 115, 16, TFT_BLACK); // Black background for readability
      DrawStringAt((TFT_WID - 115), 13, "NPN", MediumFont, TFT_GREEN);
      DrawString(" hFE ", MediumFont, TFT_GREEN);
      DrawInt(GetNpnGain(), MediumFont, TFT_GREEN);

      break;

    case tkNMOSFET:
      DrawBox((TFT_WID - 125), 0, 125, 16, TFT_BLACK); // Black background for readability
      DrawStringAt((TFT_WID - 125), 13, "N-Mosfet", MediumFont, TFT_GREEN);
      DrawString(" Vth ", MediumFont, TFT_GREEN);
      DrawString(dtostrf(GetNMosfetThreshold() / 1000.0, 1, 1, str), MediumFont, TFT_GREEN);
      break;

    case tkPMOSFET:
      DrawBox((TFT_WID - 125), 0, 125, 16, TFT_BLACK); // Black background for readability
      DrawStringAt((TFT_WID - 125), 13, "P-Mosfet", MediumFont, TFT_GREEN);
      DrawString(" Vth ", MediumFont, TFT_GREEN);
      DrawString(dtostrf(GetPMosfetThreshold() / 1000.0, 1, 1, str), MediumFont, TFT_GREEN);
      break;

    case tkNDiode:
      DrawBox((TFT_WID - 115), 0, 115, 16, TFT_BLACK); // Black background for readability
      DrawStringAt((TFT_WID - 115), 13, "Diode", MediumFont, TFT_GREEN);
      DrawString(" Vf ", MediumFont, TFT_GREEN);
      DrawString(dtostrf(GetNDiodeForwardVoltage() / 1000.0, 1, 1, str), MediumFont, TFT_GREEN);
      break;

    case tkPDiode:
      DrawBox((TFT_WID - 115), 0, 115, 16, TFT_BLACK); // Black background for readability
      DrawStringAt((TFT_WID - 120), 13, "Diode", MediumFont, TFT_GREEN);
      DrawString(" Vf ", MediumFont, TFT_GREEN);
      DrawString(dtostrf(GetPDiodeForwardVoltage() / 1000.0, 1, 1, str), MediumFont, TFT_GREEN);
      break;

    case tkNothing:
      break;

  }
  SerialPrint("g "); SerialPrintLn(i);
}

//-------------------------------------------------------------------------
// DrawGraphLabel
//   draw base current / gate voltage at the end of each curve
//-------------------------------------------------------------------------

void DrawGraphLabel(TkindDUT kind, int base, int x, int y) {

  x = x > TFT_WID - 20 ? TFT_WID - 20 : x + 3; // Move slightly if rightmost of display
  y = y < 8 ? 8 : y + 3; // Move slightly if topmost of display 

  DrawBox(x - 3, y < 9 ? 0 : y - 9, 32, 12, TFT_BLACK); // Clear background for better readability
  ILI9341SetCursor(x, y);
  switch (kind) {
    case tkNPN:
    case tkPNP:
      DrawString(dtostrf(base * 1.66, 1, 0, str), SmallFont, TFT_CYAN); // Base current in 1,66 uA steps: (12V - 0,6V) / 27000 / 255 = 1.66 uA
      DrawString("uA", SmallFont, TFT_CYAN);
      break;
    case tkNMOSFET:
    case tkPMOSFET:
      DrawString(dtostrf(base * 11.7 / MaxDacBase, 1, 1, str), SmallFont, TFT_CYAN); // Gate voltage in 11.7 mV steps: 12V / 1023 ADC-steps = 11.7 mV
      DrawString("V", SmallFont, TFT_CYAN);
      break;
    case tkPDiode:
    case tkNDiode:
    case tkNothing:
      break;
  }
}

//-------------------------------------------------------------------------
// Scan
//   draw curves for a component in the DUT socket
//   i = iConst + base * iInc / 10
//   Base current in 1,66 uA steps: (12V - 0,6V) / 27000 / 255 = 1,66 uA
//-------------------------------------------------------------------------

void Scan(TkindDUT kind, int iConst, int iInc, int minBase, int maxBase, int incBase) {
  int i, base, DacVcc;
  SerialPrintLn("n");

  for (base = minBase; base <= maxBase; base += incBase) {
    i = iConst + base * iInc / 10;
//    if (i < MinDacBase || i > MaxDacBase) break; // Anders???
    SetDacBase(i, 0);

    switch (kind) {
      case tkNPN:
      case tkNMOSFET:
      case tkNDiode:
        for (DacVcc = 0; DacVcc <= MaxDacVcc; DacVcc += 2) {
          SetDacVcc(DacVcc, 0);
          if (DacVcc < 5) {
            delay(50); // Delay first round so DAC is seattled
          }
          Graph(DacVcc == MinDacVcc, kind, GetAdcSmooth(pin_ADC_NPN_Vcc), GetAdcSmooth(pin_ADC_NPN_Vce));
          if (prev_y < 0)
            DacVcc = MaxDacVcc + 1;
        };
        break;

      case tkPNP:
      case tkPMOSFET:
      case tkPDiode:
        for (DacVcc = MaxDacVcc; DacVcc >= 0; DacVcc -= 2) {
          SetDacVcc(DacVcc, 1);
          if (DacVcc > MaxDacVcc - 5) {
            delay(50); // Delay first round so output from DAC is seattled
          }
          Graph(DacVcc == MaxDacVcc, kind, GetAdcSmooth(pin_ADC_PNP_Vcc), GetAdcSmooth(pin_ADC_PNP_Vce));
          if (prev_y < 0)
            DacVcc = -1;
        };
        break;

      case tkNothing:
        break;
    }
    TurnOffLoad(kind);
    if (base > 0)
      DrawGraphLabel(kind, base, prev_x, prev_y);

    SerialPrintLn("z");
  }
  EndScan(kind);
}

//-------------------------------------------------------------------------
// ScanKind
//   draw curves for a component kind
//   Base current in 1,66 uA steps: (12V - 0,6V) / 27000 / 255 = 1,66 uA
//-------------------------------------------------------------------------

void ScanKind(TkindDUT kind) {
  int minBase = 0, maxBase = 0, incBase = 0;

  InitGraph(kind);
  switch (kind) {
    case tkPNP:
    case tkNPN:
      minBase = float(MinIbase / 1.66); // ???
      maxBase = float(MaxIbase / 1.66);
      break;

    case tkPMOSFET:
    case tkNMOSFET:
      minBase = MinVgate * 2; // 50 mV gate steps
      maxBase = MaxVgate * 2;
      break;

    case tkPDiode:
    case tkNDiode:
    case tkNothing:
      break;
  }

  incBase = (maxBase - minBase) / (NoOfCurves - 1);
  incBase = incBase < 1 ? 1 : incBase; // incBase must be at least 1

  switch (kind) {
    case tkNPN:     Scan(kind, BaseNpnFactor,  11, minBase, maxBase, incBase); break;
    case tkPNP:     Scan(kind, BasePnpFactor, -11, minBase, maxBase, incBase); break;
    case tkNMOSFET: Scan(kind, 0,              11, minBase, maxBase, incBase); break;
    case tkPMOSFET: Scan(kind, GatePMosFactor,-11, minBase, maxBase, incBase); break;
    case tkNDiode:  Scan(kind, 0,              11, minBase, minBase,       1); break;
    case tkPDiode:  Scan(kind, 0,             -11, minBase, minBase,       1); break;
    case tkNothing: break;
  }
}

//-------------------------------------------------------------------------
// DrawCheckBox
//   draw a CheckBox on the Main Menu
//-------------------------------------------------------------------------

void DrawCheckBox(int Left, const char *str, bool checked, const uint8_t *bitmap1, const uint8_t *bitmap2) {
  const int Top = 105;
  const int Width = 92;
  const int BoxWidth = 33;
  const int BoxHeight = 33;
  const int MenuCol2 = (Width - BoxWidth) / 2;
  const int BoxTop = 52;
  pen_width = 1;

  DrawFrame(Left + MenuCol2, Top + BoxTop, BoxWidth, BoxHeight, TFT_GREEN);
  if (checked) {
    DrawLine(Left + MenuCol2, Top + BoxTop, Left + MenuCol2 + BoxWidth - 1, Top + BoxTop + BoxHeight - 1, TFT_GREEN);
    DrawLine(Left + MenuCol2, Top + BoxTop + BoxHeight - 1, Left + MenuCol2 + BoxWidth - 1, Top + BoxTop, TFT_GREEN);
  }

  execDrawChar = false;
  DrawStringAt(0, Top + 40, str, LargeFont, TFT_WHITE);
  execDrawChar = true;
  DrawStringAt(Left + (Width - Cursorx) / 2, Top + 40, str, LargeFont, TFT_WHITE);
}

//-------------------------------------------------------------------------
// DrawCharColumn
//   draw a column of chars on main menu
//-------------------------------------------------------------------------

void DrawCharColumn(uint16_t x0, uint16_t y0, const char* str, uint16_t color) {
  int y;
  const char* c;
  y = y0;
  c = str;
  while (*c) {
    ILI9341SetCursor(x0, y);
    DrawChar(*c, MediumFont, color);
    y += 15;
    c++;
  }
}

//-------------------------------------------------------------------------
// DrawZIF
//   draw the ZIF-socket
//-------------------------------------------------------------------------

void DrawZIF(const char* str1, const char* str2) {
  const int Shape1Width = 65;
  const int Shape1Height = 109;
  const int Shape1Left = (TFT_WID - Shape1Width) / 2;
  const int Shape1Top = 1;

  DrawFrame(Shape1Left, Shape1Top + 5, Shape1Width, Shape1Height, TFT_WHITE);
  DrawCharColumn(Shape1Left - 20, Shape1Top + 20, str1, TFT_YELLOW);
  DrawCharColumn(Shape1Left + Shape1Width + 10, Shape1Top + 20, str2, TFT_YELLOW);
  DrawCharColumn(Shape1Left + 6, Shape1Top + 18, "ooooooo", TFT_WHITE);
  DrawCharColumn(Shape1Left + Shape1Width / 2 - 4, Shape1Top + 18, "^", TFT_WHITE);
  DrawCharColumn(Shape1Left + Shape1Width - 12, Shape1Top + 18, "ooooooo", TFT_WHITE);
}

//-------------------------------------------------------------------------
// DrawButton
//   draw standard button on x, y coordinate
//-------------------------------------------------------------------------

void DrawButton(int x, int y, const char *Txt) {
  const int ButtonWidth = 50;
  const int ButtonHeight = 25;
  DrawFrame(x, y, ButtonWidth, ButtonHeight, TFT_GREEN);
  DrawStringAt(x + 8, y + 18, Txt, MediumFont, TFT_GREEN);
}

//-------------------------------------------------------------------------
// DrawMainMenu
//   draw the main menu screen
//-------------------------------------------------------------------------

void DrawMainMenu(void) {
  ClearDisplay(TFT_BLACK);

  const int BipolarLeft = 10;
  const int MOSFETLeft = 114;
  const int DiodeLeft = 218;

  // Draw supply voltage in V
  DrawStringAt(2,  TFT_HGT - 8, "Bat ", MediumFont, TFT_LIGHTGREY);
  //DrawDecimal(GetAdcSmooth(pin_Adc_Bat) * readSupply() * (R6 + R7) / (100L * R7 * 1024), MediumFont, TFT_LIGHTGREY);
  DrawString(dtostrf(GetAdcSmooth(pin_Adc_Bat) * readSupply() * (R6 + R7) / (1000.0 * R7 * 1024.0), 1, 2, str), MediumFont, TFT_LIGHTGREY);

  switch (CurDUTclass) { // Draw top of Menu
    case tcMOSFET:
      DrawZIF("SgDSg", "SgDSg");
      DrawBitmapMono( 70, 13, bmpPMOSFET, TFT_WHITE);
      DrawBitmapMono(230, 13, bmpNMOSFET, TFT_WHITE);
      DrawStringAt(15,   73, "P-Mosfet", LargeFont, TFT_WHITE);
      DrawStringAt(230,  73, "N-Mosfet", LargeFont, TFT_WHITE);
      break;

    case tcDiode:
      DrawZIF("A K", "K A");
      DrawBitmapMono( 70, 18, bmpPDiodeBig, TFT_WHITE);
      DrawBitmapMono(230, 18, bmpNDiodeBig, TFT_WHITE);
      DrawStringAt(55,    73, "Diode", LargeFont, TFT_WHITE);
      DrawStringAt(220,   73, "Diode", LargeFont, TFT_WHITE);
      break;

    case tcBipolar:
      DrawZIF("EbCEb", "EbCEb");
      DrawBitmapMono( 70, 17, bmpPNP, TFT_WHITE);
      DrawBitmapMono(230, 17, bmpNPN, TFT_WHITE);
      DrawStringAt(53,    73, "PNP", LargeFont, TFT_WHITE);
      DrawStringAt(230,   73, "NPN", LargeFont, TFT_WHITE);
  }

  DrawCheckBox(BipolarLeft, "BJT",    CurDUTclass == tcBipolar, bmpPNP, bmpNPN);
  DrawCheckBox(MOSFETLeft,  "MOSFET", CurDUTclass == tcMOSFET,  bmpPMOSFET, bmpNMOSFET);
  DrawCheckBox(DiodeLeft,   "Diode",  CurDUTclass == tcDiode,   bmpNDiodeBig, bmpPDiodeBig);

  DrawButton(TFT_WID - 134, TFT_HGT - 27, "Tools");
  DrawButton(TFT_WID - 54, TFT_HGT - 27, "Setup");
}

//-------------------------------------------------------------------------
// PrintADCs
//   transmit ADC values on serial to PC
//-------------------------------------------------------------------------

void  PrintADCs(void) {
  if (SendAdcValues) {
    SerialPrint("x ");
    SerialPrint(GetAdcSmooth(pin_ADC_PNP_Vcc));
    SerialPrint(",");
    SerialPrint(GetAdcSmooth(pin_ADC_PNP_Vce));
    SerialPrint(",");
    SerialPrint(GetAdcSmooth(pin_ADC_NPN_Vcc));
    SerialPrint(",");
    SerialPrint(GetAdcSmooth(pin_ADC_NPN_Vce));
    SerialPrint(",");
    SerialPrint(GetAdcSmooth(pin_Adc_12V));
    SerialPrint(",");
    SerialPrintLn(readSupply());
  }
}

//-------------------------------------------------------------------------
// GetSerial
//   waits for and gets a serial input char
//-------------------------------------------------------------------------

uint8_t GetSerial() {
  while (Serial.available() == 0) {};
  return Serial.read();
}

//-------------------------------------------------------------------------
// ExecSerialCmd
//   execute a serial Rx command from PC
//-------------------------------------------------------------------------

void ExecSerialCmd(void) {
  if (Serial.available() == 0)
    return;

  switch (GetSerial()) {
    case 'A':
      SetDacVcc(GetSerial(), 2);
      break;

    case 'B':
      ExecSerialTx = true;
      SetDacBase(GetSerial(), 2);
      break;

    case 'N':
      ExecSerialTx = true;
      ScanKind(tkNPN);
      break;

    case 'P':
      ExecSerialTx = true;
      ScanKind(tkPNP);
      break;

    case 'F':
      ExecSerialTx = true;
      ScanKind(tkNMOSFET);
      break;

    case 'f':
      ExecSerialTx = true;
      ScanKind(tkPMOSFET);
      break;

    case 'D':
      ExecSerialTx = true;
      ScanKind(tkNDiode);
      break;

    case 'd':
      ExecSerialTx = true;
      ScanKind(tkPDiode);
      break;

    case 'Q':
      ExecSerialTx = true;
      SquareWave();
      break;

    case 'H':
      Serial.println(TestDeviceKind());
      break;

    case 'X':
      SendAdcValues = true;
      break;

    case 'M': // Show main menu and start automatic DUT scanning
      curKind = tkNothing;
      ExecSerialTx = false;
      DrawMainMenu();
      break;

    default:
      return;
  }
}

//-------------------------------------------------------------------------
// DrawMenuItem
//   draw menu item with possibility to erase old value
//-------------------------------------------------------------------------

void DrawMenuItem(int RowNo, int ColNo, const char* Txt, int EraseMultiple) {
  int Col; // Column position value
  switch (ColNo) {
    case 1: Col = MenuCol1; break;
    case 2: Col = MenuCol2; break;
    case 3: Col = MenuCol3; break;
    case 4: Col = MenuCol4; break;
    case 5: Col = MenuCol5; break;
  }
  if (EraseMultiple > 0) DrawBox(Col, MenuBox1 + MenuRowHeight * RowNo, MenuEraseWidth * EraseMultiple, MenuRowHeight, TFT_BLACK);
  DrawStringAt(Col, MenuRow1 + MenuRowHeight * RowNo, Txt, MediumFont, TFT_WHITE);
}

//-------------------------------------------------------------------------
// DrawIncDecButton
//   draw Increase & Decrease buttons on a row
//-------------------------------------------------------------------------

void DrawIncDecButton(int RowNo) {
  DrawFrame(MenuCol2, MenuBox1 + MenuRowHeight * RowNo, MenuBoxSize, MenuBoxSize, TFT_GREEN);
  DrawStringAt(MenuCol2 + 9, MenuRow1 - 2 + MenuRowHeight * RowNo, "-", LargeFont, TFT_WHITE);
  DrawFrame(MenuCol4, MenuBox1 + MenuRowHeight * RowNo, MenuBoxSize, MenuBoxSize, TFT_GREEN);
  DrawStringAt(MenuCol4 + 8, MenuRow1 + MenuRowHeight * RowNo, "+", LargeFont, TFT_WHITE);
}

//-------------------------------------------------------------------------
// TouchIncDec
//   return increase / decrease based on touch coordinates
//-------------------------------------------------------------------------

int TouchIncDec(int RowNo, int x, int y) {
//  if (x == 0 && y == 0) return 0; // Quick exit
  int Row;
  for (Row = 0; Row <= 7; Row++) {
    if ((y > MenuBox1 + MenuRowHeight * RowNo && y < MenuBox1 + MenuRowHeight * (RowNo + 1))) {
      if (x > MenuCol3) {
        return 1; // Increase
      } else if (x > MenuCol2 && x < MenuCol3) {
        return -1; // Decrease
      }
    }
  } 
  return 0;
}

//-------------------------------------------------------------------------
// FastIncDec
//   accelerate increase / decrease after some time and no of rounds
//-------------------------------------------------------------------------

int FastIncDec(int IncDec) {
  static unsigned long FastTimer = 0;
  static int FastCount = 0;
  int NewIncDec;
  if (millis() - FastTimer < 300) {
    if (FastCount >  9) NewIncDec = IncDec * 10; // Faster inc/dec
    if (FastCount > 15) NewIncDec = IncDec * 50; // Fastest inc/dec
    FastCount ++;
  } else {
    NewIncDec = IncDec; // Orig. inc/dec
    FastCount = 0;
  }
  FastTimer = millis();
  return NewIncDec;
}

//-------------------------------------------------------------------------
// ExecSetupMenu
//   draw and executes the setup menu screen
//-------------------------------------------------------------------------

void ExecSetupMenu() {
  bool ExitButton = false;
  bool FirstRun = true;
  int IncDec; // Increase / Decrease value for parameters
 
  ClearDisplay(TFT_BLACK);
  DrawStringAt((TFT_WID - 60) / 2, 15, "Setup", LargeFont, TFT_LIGHTGREY); // Header text
  DrawMenuItem(0, 1, "No of curves", 0);
  DrawIncDecButton(0);
  DrawMenuItem(1, 1, "Max load current", 0);
  DrawIncDecButton(1);
  DrawMenuItem(2, 1, "Min base current", 0);
  DrawIncDecButton(2);
  DrawMenuItem(3, 1, "Max base current", 0);
  DrawIncDecButton(3);
  DrawMenuItem(4, 1, "Min gate voltage", 0);
  DrawIncDecButton(4);
  DrawMenuItem(5, 1, "Max gate voltage", 0);
  DrawIncDecButton(5);

  DrawButton(TFT_WID - 54, TFT_HGT - 27, "Exit");
  while (!ExitButton) {
    int x = 0, y = 0; // Touch coordinates

    if (FirstRun || GetTouch(&x, &y)) {
      if (y > TFT_HGT - 54 && x > TFT_WID - 27) {// Exit button
        ExitButton = true;
      }

// Increment / decrement parameter values
      IncDec = TouchIncDec(0, x, y);
      if (FirstRun || IncDec != 0) {
        NoOfCurves += IncDec;
        if (NoOfCurves < 2) NoOfCurves = 2;
        if (NoOfCurves > 15) NoOfCurves = 15;
        DrawMenuItem(0, 3, dtostrf(NoOfCurves, 1, 0, str), 1);
      }
      
      IncDec = TouchIncDec(1, x, y);
      if (FirstRun || IncDec != 0) {
        mAmax += IncDec * 10;
        if (mAmax < 10) mAmax = 10;
        if (mAmax > 100) mAmax = 110; // Load resistor of 100R limits load current to 110 mA
        DrawMenuItem(1, 3, dtostrf(mAmax, 1, 0, str), 1);
      }

      IncDec = TouchIncDec(2, x, y);
      if (FirstRun || IncDec != 0) {
        MinIbase += FastIncDec(IncDec);
        if (MinIbase < 0) MinIbase = 0;
        if (MinIbase > MaxIbase) MinIbase = MaxIbase;
        DrawMenuItem(2, 3, dtostrf(MinIbase, 1, 0, str), 1);
      }

      IncDec = TouchIncDec(3, x, y);
      if (FirstRun || IncDec != 0) {
        MaxIbase += FastIncDec(IncDec);
        if (MaxIbase < MinIbase) MaxIbase = MinIbase;
        if (MaxIbase > 400) MaxIbase = 400; // Base resistor of 27K limits base current to about 400 uA
        DrawMenuItem(3, 3, dtostrf(MaxIbase, 1, 0, str), 1);
      }

      IncDec = TouchIncDec(4, x, y);
      if (FirstRun || IncDec != 0) {
        MinVgate += FastIncDec(IncDec);
        if (MinVgate < 0) MinVgate = 0;
        if (MinVgate > MaxVgate) MinVgate = MaxVgate;
        DrawMenuItem(4, 3, dtostrf(MinVgate / 10.0, 1, 1, str), 1);
      }

      IncDec = TouchIncDec(5, x, y);
      if (FirstRun || IncDec != 0) {
        MaxVgate += FastIncDec(IncDec);
        if (MaxVgate < MinVgate) MaxVgate = MinVgate;
        if (MaxVgate > 120) MaxVgate = 120;
        DrawMenuItem(5, 3, dtostrf(MaxVgate / 10.0, 1, 1, str), 1);
      }

      delay(100); // Delay for increase / decrease of parameter values
    }
    FirstRun = false;
  }
}

//-------------------------------------------------------------------------
// ExecToolMenu
//   draw and executes the setup menu screen
//-------------------------------------------------------------------------

void ExecToolMenu() {
  bool ExitButton = false;
  bool FirstRun = true;
  static unsigned long UpdTme = 0; // Update timer for read only parameters
  int IncDec; // Increase / Decrease value for parameters
  int DacBase = 130; // Initial values for DAC's
  int DacVcc = 130;

  ClearDisplay(TFT_BLACK);
  DrawStringAt((TFT_WID - 40) / 2, 15, "Tools", LargeFont, TFT_LIGHTGREY); // Header text
  DrawMenuItem(0, 1, "DAC Vcc (load)", 0);
  DrawIncDecButton(0);
  DrawMenuItem(1, 1, "DAC Base/Gate", 0);
  DrawIncDecButton(1);
  DrawMenuItem(2, 1, "ADC NPN side", 0);
  DrawMenuItem(3, 1, "ADC PNP side", 0);
  DrawMenuItem(4, 1, "BJT Gain (hFE)", 0);
  DrawMenuItem(6, 1, "ADC Supply V", 0);
  DrawMenuItem(7, 1, "ADC 12 V", 0);

  DrawButton(TFT_WID - 54, TFT_HGT - 27, "Exit");
  while (!ExitButton) {
    int x = 0, y = 0; // Touch coordinates

    if (FirstRun || GetTouch(&x, &y)) {
      if (y > TFT_HGT - 54 && x > TFT_WID - 27) { // Exit button
        ExitButton = true;
      }

// Increase / Decrease parameter values
      IncDec = TouchIncDec(0, x, y);
      if (FirstRun || IncDec != 0) {
        DacVcc += FastIncDec(IncDec);
        if (DacVcc < MinDacVcc) DacVcc = MinDacVcc;
        if (DacVcc > MaxDacVcc) DacVcc = MaxDacVcc;
        DrawMenuItem(0, 3, dtostrf(DacVcc, 3, 0, str), 1);
        DrawMenuItem(0, 5, strcat(dtostrf(DacVcc * 11.7 / MaxDacVcc, 5, 1, str), " V"), 2);
      }

      IncDec = TouchIncDec(1, x, y);
      if (FirstRun || IncDec != 0) {
        DacBase += FastIncDec(IncDec);
        if (DacBase < MinDacBase) DacBase = MinDacBase;
        if (DacBase > MaxDacBase) DacBase = MaxDacBase;
        DrawMenuItem(1, 3, dtostrf(DacBase, 3, 0, str), 1);
        DrawMenuItem(1, 5, strcat(dtostrf(DacBase * 11.7 / MaxDacBase, 5, 1, str), " V"), 2);
      }
      delay(100); // Delay for increase / decrease of parameter values

// Update read only papameters
    } else if (millis() - UpdTme > 1000) { // Update read only parameters if !GetTouch
      SetDacBase(DacBase, 0);
      SetDacVcc(DacVcc, 10);
      int NpnVcc = GetAdcSmooth(pin_ADC_NPN_Vcc);
      int NpnVce = GetAdcSmooth(pin_ADC_NPN_Vce);
      int PnpVce = GetAdcSmooth(pin_ADC_PNP_Vce);
      int PnpVcc = GetAdcSmooth(pin_ADC_PNP_Vcc);

      DrawMenuItem(2, 2, strcat(dtostrf(NpnVcc, 1, 0, str), " - "), 6);
      DrawMenuItem(2, 3, dtostrf(NpnVce, 1, 0, str), 0);
      DrawMenuItem(2, 4, dtostrf(NpnVcc - NpnVce, 1, 0, str), 0);
      DrawMenuItem(2, 5, strcat(dtostrf((NpnVcc - NpnVce) * 0.117, 5, 1, str), " mA"), 0);

      DrawMenuItem(3, 2, strcat(dtostrf(PnpVce, 1, 0, str), " - "), 6);
      DrawMenuItem(3, 3, dtostrf(PnpVcc, 1, 0, str), 0);
      DrawMenuItem(3, 4, dtostrf(PnpVce - PnpVcc, 1, 0, str), 0);
      DrawMenuItem(3, 5, strcat(dtostrf((PnpVce - PnpVcc) * 0.117, 5, 1, str), " mA"), 0);

      DrawMenuItem (4, 2, NULL, 4); // Clear old hFE values
      int NpnHfe = float((NpnVcc - NpnVce) * 117.0) / ((DacBase - BaseNpnFactor) * 166.0 / 100.0);
      if (NpnHfe > 1) DrawMenuItem (4, 2, dtostrf(NpnHfe, 1, 0, str), 0);

      int PnpHfe = float((PnpVce - PnpVcc) * 117.0) / ((BasePnpFactor - DacBase) * 166.0 / 100.0);
      if (PnpHfe > 1) DrawMenuItem (4, 4, dtostrf(PnpHfe, 1, 0, str), 0);

      DrawMenuItem(6, 2, dtostrf(GetAdcSmooth(pin_Adc_Bat), 1, 0, str), 1);
      DrawMenuItem(7, 2, dtostrf(GetAdcSmooth(pin_Adc_12V), 1, 0, str), 1);

      UpdTme = millis();
    }
    FirstRun = false;
  }
  TurnOffLoad(tkNothing); // Set DACs to default values
}

//-------------------------------------------------------------------------
// setup
//   Arduino setup function
//-------------------------------------------------------------------------

void setup(void) {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(pin_DAC_CS, OUTPUT);

  ILI9341fast = false; // ILI9341fast = true is not 100% stable!

// Set display rotation: ILI9341_Rotation0 = Portrait display, ILI9341_Rotation3 = Landscape display
  ILI9341Begin(2, 4, 3, TFT_WID, TFT_HGT, ILI9341_Rotation0);

// Set touch Rotation: 1 = Portrait display, 3 = Landscape display
  BeginTouch(7, 1);

  Serial.begin(9600);
  Serial.println("Curve Tracer");

  SPI.begin();
  SPI.beginTransaction(SPISettings (4000000UL, MSBFIRST, SPI_MODE0));  // SPI for DAC

  TurnOffLoad(tkNothing); // Set DACs to the default
  DrawMainMenu();
}

//-------------------------------------------------------------------------
// loop
//   Arduino main loop
//-------------------------------------------------------------------------

void loop(void) {
  static unsigned long time = 0;  // Delay timer between tests of DUT kind
  TkindDUT newKind;

  ExecSerialCmd();
  PrintADCs();

  if (millis() - time > 700 && !ExecSerialTx) { // Delay timer between tests of DUT kind

    newKind = TestDeviceKind();
    
    if ((curKind == tkNothing) && newKind != tkNothing) { // DUT inserted
      delay(100); // so DUT is fully inserted
      ScanKind(newKind);
      curKind = newKind;
    } else if ((curKind != tkNothing) && newKind == tkNothing) { // DUT removed
      curKind = tkNothing;
      DrawMainMenu(); // Show Main Menu when DUT is removed
    }

    time = millis();
  }

  if (curKind == tkNothing) { // Exec Main Menu, check for touch
    int x, y; // Touch coordinates
    if (!GetTouch(&x, &y)) return;
    
    if (y > TFT_HGT * 2 / 5 && y < TFT_HGT * 4 / 5) { // New selection of DUT class
      if (x < TFT_WID * 1 / 3 ) {
        if (CurDUTclass != tcBipolar) CurDUTclass = tcBipolar;
      }
      else if (x < TFT_WID * 2 / 3) {
        if (CurDUTclass != tcMOSFET) CurDUTclass = tcMOSFET;
      } else {
        if (CurDUTclass != tcDiode) CurDUTclass = tcDiode;
      }
      DrawMainMenu(); // Show Main Menu after new DUT class is selected
    }

    if (y > TFT_HGT * 4 / 5) {
      if (x > TFT_WID * 4 / 5) {
        ExecSetupMenu();
        DrawMainMenu(); // Show Main Menu after Setup Menu
      } else if (x > TFT_WID * 3 / 5) {
        ExecToolMenu();
        DrawMainMenu(); // Show Main Menu after Tool Menu
      }
    }
  }
}