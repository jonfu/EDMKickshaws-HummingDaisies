/*
PICCOLO is a tiny Arduino-based audio visualizer.
Hardware requirements:
 - Most Arduino or Arduino-compatible boards (ATmega 328P or better).
 - Adafruit Bicolor LED Matrix with I2C Backpack (ID: 902)
 - Adafruit Electret Microphone Amplifier (ID: 1063)
 - Optional: battery for portable use (else power through USB)
Software requirements:
 - elm-chan's ffft library for Arduino
Connections:
 - 3.3V to mic amp+ and Arduino AREF pin <-- important!
 - GND to mic amp-
 - Analog pin 0 to mic amp output
 - +5V, GND, SDA (or analog 4) and SCL (analog 5) to I2C Matrix backpack
Written by Adafruit Industries.  Distributed under the BSD license --
see license.txt for more information.  This paragraph must be included
in any redistribution.
ffft library is provided under its own terms -- see ffft.S for specifics.
*/

// IMPORTANT: FFT_N should be #defined as 128 in ffft.h.

#include <avr/pgmspace.h>
#include <ffft.h>
#include <math.h>
#include <Adafruit_NeoPixel.h>

#define VCC 12
#define GND 13

//#define VCC1 2
//#define GND1 7

#define VCC2 5
#define GND2 4
#define DIN 11

#define noiseThreshold 4
#define rotateThreshold 2048
#define BRI 100
#define BRIROLL 60
#define NUMCIR 4
#define NUMBAND 8
#define DIMMER 4
#define DIM 2
#define ROTATEFRAME 16
//#define rollingThreshold 40 --default
#define rollingThreshold 42
#define ROLLTIME 3000
#define COLORSWITCH 4

const byte WIPEINTERVAL = 30;
const byte CHASEINTERVAL = 8;
const byte THEATERINTERVAL = 25;
const byte WAVEINTERVAL = 30;
const byte RAINBOWINTERVAL = 3;

/*
#define N_LEDS 60
#define DATA_PIN 9

#define N_LEDS1 16
#define DATA_PIN1 8

#define N_LEDS2 12
#define DATA_PIN2 10

#define N_LEDS3 24
#define DATA_PIN3 11

Adafruit_NeoPixel circle0 = Adafruit_NeoPixel(N_LEDS, DATA_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel circle1 = Adafruit_NeoPixel(N_LEDS1, DATA_PIN1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel circle2 = Adafruit_NeoPixel(N_LEDS2, DATA_PIN2, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel circle3 = Adafruit_NeoPixel(N_LEDS3, DATA_PIN3, NEO_GRB + NEO_KHZ800);
*/

const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
const byte wheelIncr = 32;

const byte numLEDs[NUMCIR] = {24, 16, 12, 60}; 
//const byte numLEDs[NUMCIR] = {60, 16, 12, 24};
//const byte dataPin[NUMCIR] = {10, 9, 6, 3};
const byte dataPin[NUMCIR] = {9, 6, 3, 10};

Adafruit_NeoPixel circle[NUMCIR] = {
  Adafruit_NeoPixel(numLEDs[0], dataPin[0], NEO_GRB + NEO_KHZ800),
  Adafruit_NeoPixel(numLEDs[1], dataPin[1], NEO_GRB + NEO_KHZ800),
  Adafruit_NeoPixel(numLEDs[2], dataPin[2], NEO_GRB + NEO_KHZ800),
  Adafruit_NeoPixel(numLEDs[3], dataPin[3], NEO_GRB + NEO_KHZ800)
};

#define LED_RED circle[0].Color(200, 0, 0)
#define LED_GREEN circle[0].Color(0, 64, 0)
#define LED_YELLOW circle[0].Color(100, 100, 0)
#define LED_OFF 0

int16_t       capture[FFT_N];    // Audio capture buffer
complex_t     bfly_buff[FFT_N];  // FFT "butterfly" buffer
uint16_t      spectrum[FFT_N/2]; // Spectrum output buffer
volatile byte samplePos = 0;     // Buffer position counter

byte
  peak[8],      // Peak level of each column; used for falling dots
  co[8],        // saved clipped output
  wheelPos[8],  // current wheel position
  dotCount = 0, // Frame counter for delaying dot-falling speed
  colCount = 0; // Frame counter for storing past column data
int
  col[8][10],   // Column levels for the prior 10 frames
  cc[4],        // center of spectrum base on circles
  minLvlAvg[8], // For dynamic adjustment of low & high ends of graph,
  maxLvlAvg[8], // pseudo rolling averages for the prior few frames.
  colDiv[8];    // Used when filtering FFT output to 8 columns

uint16_t rotateCounter = 0;
//bool rotate[8];
bool roll = false;
unsigned long stopRolling = 0;
unsigned long next[4];
byte theaterChaseIndex, wipeIndex;
bool rollingForward;
int tick, chaseIndex, rainbowIndex;
uint32_t theaterChase1, theaterChase2, wipeColor;
byte colorThemeCounter = 0;
bool defaultColorTheme = true;

/*
These tables were arrived at through testing, modeling and trial and error,
exposing the unit to assorted music and sounds.  But there's no One Perfect
EQ Setting to Rule Them All, and the graph may respond better to some
inputs than others.  The software works at making the graph interesting,
but some columns will always be less lively than others, especially
comparing live speech against ambient music of varying genres.
*/
static const uint8_t PROGMEM
  // This is low-level noise that's subtracted from each FFT output column:
  noise[64]={ 8,6,6,5,3,4,4,4,3,4,4,3,2,3,3,4,
              2,1,2,1,3,2,3,2,1,2,3,1,2,3,4,4,
              3,2,2,2,2,2,2,1,3,2,2,2,2,2,2,2,
              2,2,2,2,2,2,2,2,2,2,2,2,2,3,3,4 },
  // These are scaling quotients for each FFT output column, sort of a
  // graphic EQ in reverse.  Most music is pretty heavy at the bass end.
  eq[64]={
    255, 175,218,225,220,198,147, 99, 68, 47, 33, 22, 14,  8,  4,  2,
      0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
  // When filtering down to 8 columns, these tables contain indexes
  // and weightings of the FFT spectrum output values to use.  Not all
  // buckets are used -- the bottom-most and several at the top are
  // either noisy or out of range or generally not good for a graph.
  col0data[] = {  2,  1,  // # of spectrum bins to merge, index of first
    111,   8 },           // Weights for each bin
  col1data[] = {  4,  1,  // 4 bins, starting at index 1
     19, 186,  38,   2 }, // Weights for 4 bins.  Got it now?
  col2data[] = {  5,  2,
     11, 156, 118,  16,   1 },
  col3data[] = {  8,  3,
      5,  55, 165, 164,  71,  18,   4,   1 },
  col4data[] = { 11,  5,
      3,  24,  89, 169, 178, 118,  54,  20,   6,   2,   1 },
  col5data[] = { 17,  7,
      2,   9,  29,  70, 125, 172, 185, 162, 118, 74,
     41,  21,  10,   5,   2,   1,   1 },
  col6data[] = { 25, 11,
      1,   4,  11,  25,  49,  83, 121, 156, 180, 185,
    174, 149, 118,  87,  60,  40,  25,  16,  10,   6,
      4,   2,   1,   1,   1 },
  col7data[] = { 37, 16,
      1,   2,   5,  10,  18,  30,  46,  67,  92, 118,
    143, 164, 179, 185, 184, 174, 158, 139, 118,  97,
     77,  60,  45,  34,  25,  18,  13,   9,   7,   5,
      3,   2,   2,   1,   1,   1,   1 },
  // And then this points to the start of the data for each of the columns:
  * const colData[]  = {
    col0data, col1data, col2data, col3data,
    col4data, col5data, col6data, col7data };


void setup() {
  //uint8_t i, j, nBins, binNum, *data;
  uint8_t i, j, nBins, *data;

  memset(peak, 0, sizeof(peak));
  memset(col , 0, sizeof(col));
  memset(co, 0, sizeof(co));
  memset(cc, 0, sizeof(cc));
  memset(next, 0, sizeof(next));

  for(i=0; i<8; i++) {
    minLvlAvg[i] = 0;
    maxLvlAvg[i] = 512;
    data         = (uint8_t *)pgm_read_word(&colData[i]);
    nBins        = pgm_read_byte(&data[0]) + 2;
    //binNum       = pgm_read_byte(&data[1]);
    for(colDiv[i]=0, j=2; j<nBins; j++)
      colDiv[i] += pgm_read_byte(&data[j]);

    //initial wheel position for each band
    wheelPos[i] = i*32;
  }

  pinMode(DIN, INPUT);  

  pinMode(GND, OUTPUT);
  digitalWrite(GND, LOW);
  pinMode(VCC, OUTPUT);
  digitalWrite(VCC, HIGH);  

  
  //pinMode(GND1, OUTPUT);
  //digitalWrite(GND1, LOW);
  //pinMode(VCC1, OUTPUT);
  //digitalWrite(VCC1, HIGH);  

  
  pinMode(GND2, OUTPUT);
  digitalWrite(GND2, LOW);
  pinMode(VCC2, OUTPUT);
  digitalWrite(VCC2, HIGH);  

  //Serial.begin(9600);

  for (byte i=0; i<NUMCIR; i++) {
    //circle[i] = Adafruit_NeoPixel(numLEDs[i], dataPin[i], NEO_GRB + NEO_KHZ800);
    circle[i].begin();
    circle[i].setBrightness(BRI);  
  }

  /*
 

        for (int i=0; i<60; i++) {
          circle[3].setPixelColor(i, circle[0].Color(0,255,0));
          circle[3].show();
          circle[3].setPixelColor(i, 0);
          delay(20);
          
        }
        circle[3].show();  

        */
    

  // http://www.microsmart.co.za/technical/2014/03/01/advanced-arduino-adc/  (code) 
  
  // set up the ADC
  ADCSRA &= ~PS_128;  // remove bits set by Arduino library

  // you can choose a prescaler from above.
  // PS_16, PS_32, PS_64 or PS_128
  ADCSRA |= PS_32;    // set our own prescaler to 32 - http://yaab-arduino.blogspot.com/2015/02/fast-sampling-from-analog-input.html  (table)
  
}

void loop() {

  /*

  int val = digitalRead(DIN);

  //Serial.print("digitalRead=");
  //Serial.println(val);
  
  Serial.print("analogRead=");
  Serial.println(analogRead(A0));
  delay(40);

  if (true) return;

  */

  
  
  uint8_t  i, x, L, *data, nBins, binNum, c;
  uint16_t minLvl, maxLvl;
  int      level, y, sum;

  if (roll) {
    if (stopRolling < millis()) {
      roll = false;
      for (byte i=0; i<NUMCIR; i++) {
        circle[i].clear();
        circle[i].setBrightness(BRI);
      }
    }
    else {
      rolling();
      return;
    }
  }
  

  while (samplePos < FFT_N) {

    int16_t sample = analogRead(A0);
    
    //Serial.println(sample);
  
    capture[samplePos] =
      ((sample > (512-noiseThreshold)) &&
       (sample < (512+noiseThreshold))) ? 0 :
      sample - 512; // Sign-convert for FFT; -512 to +511

    samplePos++;
    
  }

  fft_input(capture, bfly_buff);   // Samples -> complex #s
  samplePos = 0;                   // Reset sample counter

  fft_execute(bfly_buff);          // Process complex data
  fft_output(bfly_buff, spectrum); // Complex -> spectrum

  // Remove noise and apply EQ levels
  for(x=0; x<FFT_N/2; x++) {
    L = pgm_read_byte(&noise[x]);
    spectrum[x] = (spectrum[x] <= L) ? 0 :
      (((spectrum[x] - L) * (256L - pgm_read_byte(&eq[x]))) >> 8);
  }

  // Fill background w/colors, then idle parts of columns will erase
  /*
  matrix.fillRect(0, 0, 8, 3, LED_RED);    // Upper section
  matrix.fillRect(0, 3, 8, 2, LED_YELLOW); // Mid
  matrix.fillRect(0, 5, 8, 3, LED_GREEN);  // Lower section
  */

  // Downsample spectrum output to 8 columns:
  for(x=0; x<8; x++) {
    data   = (uint8_t *)pgm_read_word(&colData[x]);
    nBins  = pgm_read_byte(&data[0]) + 2;
    binNum = pgm_read_byte(&data[1]);
    for(sum=0, i=2; i<nBins; i++)
      sum += spectrum[binNum++] * pgm_read_byte(&data[i]); // Weighted
    col[x][colCount] = sum / colDiv[x];                    // Average
    minLvl = maxLvl = col[x][0];
    for(i=1; i<10; i++) { // Get range of prior 10 frames
      if(col[x][i] < minLvl)      minLvl = col[x][i];
      else if(col[x][i] > maxLvl) maxLvl = col[x][i];
    }
    // minLvl and maxLvl indicate the extents of the FFT output, used
    // for vertically scaling the output graph (so it looks interesting
    // regardless of volume level).  If they're too close together though
    // (e.g. at very low volume levels) the graph becomes super coarse
    // and 'jumpy'...so keep some minimum distance between them (this
    // also lets the graph go to zero when no sound is playing):
    if((maxLvl - minLvl) < 8) maxLvl = minLvl + 8;
    minLvlAvg[x] = (minLvlAvg[x] * 7 + minLvl) >> 3; // Dampen min/max levels
    maxLvlAvg[x] = (maxLvlAvg[x] * 7 + maxLvl) >> 3; // (fake rolling average)

    // Second fixed-point scale based on dynamic min/max levels:
    level = 10L * (col[x][colCount] - minLvlAvg[x]) /
      (long)(maxLvlAvg[x] - minLvlAvg[x]);

    // Clip output and convert to byte:
    if(level < 0L)      c = 0;
    else if(level > 10) c = 10; // Allow dot to go a couple pixels off top
    else                c = (uint8_t)level;

    if(c > peak[x]) peak[x] = c; // Keep dot on top

    display(x,c);

    /*

    if(peak[x] <= 0) { // Empty column?
      //matrix.drawLine(x, 0, x, 7, LED_OFF);
      continue;
    } else if(c < 8) { // Partial column?
      //matrix.drawLine(x, 0, x, 7 - c, LED_OFF);
    }

    // The 'peak' dot color varies, but doesn't necessarily match
    // the three screen regions...yellow has a little extra influence.
    y = 8 - peak[x];
    if(y < 2)      {
      //matrix.drawPixel(x, y, LED_RED);
    }
    else if(y < 6) {
      //matrix.drawPixel(x, y, LED_YELLOW);
    }
    else           {
      //matrix.drawPixel(x, y, LED_GREEN);
    }

    */

    
    
  }

  //matrix.show();


  // Every third frame, make the peak pixels drop by 1:
  if(++dotCount >= 3) {
    dotCount = 0;
    for(x=0; x<8; x++) {
      if(peak[x] > 0) peak[x]--;

      //also change color
      wheelPos[x]++;
    }
  }

  if(++colCount >= 10) colCount = 0;
}

void display(uint8_t x, uint8_t c) {

  byte pk, cco = 0, wp = rotateCounter+x*wheelIncr;
  int upperLimit, pos;
  uint32_t color, colorDimmer, colorDim, colorFull;
  

//  if ( c-co[x] > rotateThreshold) {
//    rotate[x] = true;
//  }

    rotateCounter++;

    if (rotateCounter % rotateThreshold == 0) {
      for (byte i=0; i<NUMCIR; i++) {
        if ( ++cc[i] >= circle[i].numPixels()) {
          cc[i]-=circle[i].numPixels();
        }   
      }
//      for (byte i=0; i<NUMBAND; i++) {
//        wheelPos[i]++;
//      }

      colorThemeCounter++;
      if (colorThemeCounter%COLORSWITCH==0) {
        defaultColorTheme = !defaultColorTheme;
      }
    }

  if (x%2 > 0) { //when x is odd

    cco = circle[x/2].numPixels()/2;
    /*
    if (rotate[x] || rotate[x-1]) {
      if ( ++cc[x/2] >= circle[x/2].numPixels()) {
        cc[x/2]-=circle[x/2].numPixels();
      }      
      rotate[x] = false;
      rotate[x-1] = false;
      wheelPos[x]++;
      wheelPos[x-1]++;
    }
    */
  }
  else {
    circle[x/2].clear();
  }

  pos = cc[x/2]+cco;

  circle[x/2].setPixelColor( getAdjPos(x, pos), Wheel(wp) );  

  if (defaultColorTheme) {

    colorDimmer =  LED_GREEN;
    colorDim = LED_YELLOW;
    colorFull = LED_RED;  
    
  }
  else {

    colorDimmer =  Wheel(wheelPos[x], DIMMER);
    colorDim = Wheel(wheelPos[x], DIM);
    colorFull = Wheel(wheelPos[x]);  

  }


  switch (x) {
    case 0:
    case 1:

/*

    Serial.print("x=");
    Serial.print(x);
    Serial.print(", pos=");
    Serial.print(pos);
    Serial.print(", getAdjPos=");
    Serial.println(getAdjPos(x, pos));
    
*/



    upperLimit = co[x];
    for (int i=1; i<=upperLimit; i++) {
      if (i<6) {
        color = colorDimmer;
      }
      else if (i<10) {
        color = colorDim;
      }
      else {
        color = colorFull;
      }
      circle[x/2].setPixelColor( getAdjPos(x, pos+i), color);
    }

    pk = peak[x];
    if (pk > upperLimit) {
      if (pk<6) {
        color = colorDimmer;
      }
      else if (pk<10) {
        color = colorDim;
      }
      else {
        color = colorFull;
      }
      circle[x/2].setPixelColor( getAdjPos(x, pos+pk), color);
    }


    if (defaultColorTheme) {
      color = circle[0].Color(0, 255, 0);
    }
    else {
      color = colorFull;
    }    

    if (pk > 0 || upperLimit > 0) {
      circle[x/2].setPixelColor( getAdjPos(x, pos), color);
    }





    break;
      
 
    case 2:
    case 3:      
    


    upperLimit = constrain(co[x],0,7);
    for (int i=1; i<=upperLimit; i++) {
      if (i<5) {
        color = colorDimmer;
      }
      else if (i<7) {
        color = colorDim;
      }
      else {
        color = colorFull;
      }
      circle[x/2].setPixelColor( getAdjPos(x, pos+i), color);
    }

    pk = constrain(peak[x],0,7);
    if (pk > upperLimit) {
      if (pk<5) {
        color = colorDimmer;
      }
      else if (pk<7) {
        color = colorDim;
      }
      else {
        color = colorFull;
      }
      circle[x/2].setPixelColor( getAdjPos(x, pos+pk), color);
    }

    if (defaultColorTheme) {
      color = circle[0].Color(0, 255, 0);
    }
    else {
      color = colorFull;
    }    

    if (pk > 0 || upperLimit > 0) {
      circle[x/2].setPixelColor( getAdjPos(x, pos), color);
    }






    break;


    case 4:
    case 5:



    upperLimit = co[x]/2;
    for (int i=1; i<=upperLimit; i++) {
      if (i<3) {
        color = colorDimmer;
      }
      else if (i<5) {
        color = colorDim;
      }
      else {
        color = colorFull;
      }
      circle[x/2].setPixelColor( getAdjPos(x, pos+i), color);
    }

    pk = peak[x]/2;
    if (pk > upperLimit) {
      if (pk<3) {
        color = colorDimmer;
      }
      else if (pk<5) {
        color = colorDim;
      }
      else {
        color = colorFull;
      }
      circle[x/2].setPixelColor( getAdjPos(x, pos+pk), color);
    }
    /*
    Serial.print("x=");
    Serial.print(x);
    Serial.print(": co[x]=");
    Serial.print(co[x]);
    Serial.print(", upperlimit=");
    Serial.print(upperLimit);    
    Serial.print(", peak[x]=");
    Serial.print(peak[x]);    
    Serial.print(", pk=");
    Serial.println(pk); 
    */

    if (defaultColorTheme) {
      color = circle[0].Color(0, 255, 0);
    }
    else {
      color = colorFull;
    }    
           
    if (pk > 0 || upperLimit > 0) {
      circle[x/2].setPixelColor( getAdjPos(x, pos), color);
    }

    break;


    case 6:
    case 7:


    upperLimit = constrain(co[x],0,8);
    for (int i=1; i<=upperLimit; i++) {
      if (i<5) {
        color = colorDimmer;
      }
      else if (i<8) {
        color = colorDim;
      }
      else {
        color = colorFull;
      }

        //circle[x/2].setPixelColor( getAdjPos(x, pos+(i+4)), color);
        //circle[x/2].setPixelColor( getAdjPos(x, pos-(i+4)), color);        

      if (i<5) {
        circle[x/2].setPixelColor( getAdjPos(x, pos+(i*2-1)), color);
        circle[x/2].setPixelColor( getAdjPos(x, pos-(i*2-1)), color);
        circle[x/2].setPixelColor( getAdjPos(x, pos+(i*2)), color);
        circle[x/2].setPixelColor( getAdjPos(x, pos-(i*2)), color);        
      }
      else {
        circle[x/2].setPixelColor( getAdjPos(x, pos+(i+4)), color);
        circle[x/2].setPixelColor( getAdjPos(x, pos-(i+4)), color);        
      }
    }

    pk = constrain(peak[x],0,8);
    if (pk > upperLimit) {
      /*
      if (pk<5) {
        color = colorDimmer;
      }
      else if (pk<8) {
        color = colorDim;
      }
      else {
        color = colorFull;
      }
      */
      if (pk<5) {
        color = colorDim;

        circle[x/2].setPixelColor( getAdjPos(x, pos+(pk*2-1)), color);
        circle[x/2].setPixelColor( getAdjPos(x, pos-(pk*2-1)), color);
        circle[x/2].setPixelColor( getAdjPos(x, pos+(pk*2)), color);
        circle[x/2].setPixelColor( getAdjPos(x, pos-(pk*2)), color);            
      }
      else {
        color = colorFull;

        circle[x/2].setPixelColor( getAdjPos(x, pos+(pk+4)), color);
        circle[x/2].setPixelColor( getAdjPos(x, pos-(pk+4)), color);            
      }      
      //circle[x/2].setPixelColor( getAdjPos(x, pos+pk), color);
      //circle[x/2].setPixelColor( getAdjPos(x, pos-pk), color);
      
    }

    if (defaultColorTheme) {
      color = circle[0].Color(0, 255, 0);
    }
    else {
      color = colorFull;
    }

    if (pk > 0 || upperLimit > 0) {
      circle[x/2].setPixelColor( getAdjPos(x, pos), color);
    }




    break;
    
  }

  circle[x/2].show();

  //rotateCounter++;

  co[x] = c;

  if (x==7) {
    int ref = 0;
    for (byte i=0; i<NUMBAND; i++) {
      ref += co[i];
    }
    if (ref >= rollingThreshold) {
      roll = true;
      rollingReset();
      stopRolling = millis() + ROLLTIME;
    }
    
  }
  


}


// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos, byte dimFactor)
{

  if(WheelPos < 85) {
    return circle[0].Color(WheelPos * 3 /dimFactor, (255 - WheelPos * 3)/dimFactor, 0);
  } else if(WheelPos < 170) {
    WheelPos -= 85;
    return circle[0].Color((255 - WheelPos * 3)/dimFactor, 0, WheelPos * 3 / dimFactor);
  } else {
    WheelPos -= 170;
    return circle[0].Color(0, WheelPos * 3 / dimFactor, (255 - WheelPos * 3)/dimFactor);
  }
  
  /*
    WheelPos = 255 - WheelPos;
    if(WheelPos < 85)
    {
        return circle[0].Color((255 - WheelPos * 3)/dimFactor, 0, WheelPos * 3 / dimFactor);
    }
    else if(WheelPos < 170)
    {
        WheelPos -= 85;
        return circle[0].Color(0, WheelPos * 3 /dimFactor, (255 - WheelPos * 3)/dimFactor);
    }
    else
    {
        WheelPos -= 170;
        return circle[0].Color(WheelPos * 3 /dimFactor, (255 - WheelPos * 3)/dimFactor, 0);
    }
    */
    
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  
  if(WheelPos < 85) {
  return circle[0].Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
  WheelPos -= 85;
  return circle[0].Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
  WheelPos -= 170;
  return circle[0].Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  
/*
    WheelPos = 255 - WheelPos;
    if(WheelPos < 85)
    {
        return circle[0].Color((255 - WheelPos * 3), 0, WheelPos * 3 );
    }
    else if(WheelPos < 170)
    {
        WheelPos -= 85;
        return circle[0].Color(0, WheelPos * 3 , (255 - WheelPos * 3));
    }
    else
    {
        WheelPos -= 170;
        return circle[0].Color(WheelPos * 3 , (255 - WheelPos * 3), 0);
    }  
*/
}

int getAdjPos(byte x, int pos) {
  byte circlePixels = circle[x/2].numPixels();
  while (pos < 0) pos += circlePixels;
  while (pos > (circlePixels-1)) pos -= circlePixels;

  return pos;
}


void rolling() {

  if ( millis() >= next[0] ) {

    next[0] = millis() + RAINBOWINTERVAL;

    for (int i = 0; i < circle[0].numPixels(); i++) {
      circle[0].setPixelColor(i, Wheel(((i * 256 / circle[0].numPixels()) + rainbowIndex) & 255));
      
      //if ((i+rainbowIndex)%4!=0) {
      if (i%2==0) {
        circle[0].setPixelColor(i, 0);
      }
      
    }

    rainbowIndex++;

    circle[0].show();
  }

  if ( millis() >= next[1] ) {

    next[1] = millis() + WAVEINTERVAL;

    int i, j;
    float ang, rsin, gsin, bsin, offset;
     
    tick++;
    offset = map2PI(tick);
     
    for (i = 0; i < circle[1].numPixels(); i++) {
      ang = map2PI(i) - offset;
      rsin = sin(ang);
      gsin = sin(2.0 * ang / 3.0 + map2PI(int(circle[1].numPixels()/6)));
      bsin = sin(4.0 * ang / 5.0 + map2PI(int(circle[1].numPixels()/3)));
      circle[1].setPixelColor(i, circle[1].Color(trigScale(rsin), trigScale(gsin), trigScale(bsin)));

      if ((i+tick)%3!=0) {
        circle[1].setPixelColor(i, 0);
      }
      
    }
     
    circle[1].show();    

    /*
    next[1] = millis() + THEATERINTERVAL;

    for(int i=0; i< circle[1].numPixels(); i++)
    {
        if ((i + theaterChaseIndex++) % 3 == 0)
        {
            circle[1].setPixelColor(i, theaterChase1);
        }
        else
        {
            circle[1].setPixelColor(i, theaterChase2);
        }
    }
    circle[1].show();

    */

  }



  if ( millis() >= next[2] ) {
    next[2] = millis() + WIPEINTERVAL;

    circle[2].setPixelColor(wipeIndex++, wipeColor);

    circle[2].show();

    if (wipeIndex == circle[2].numPixels()) {
      wipeIndex = 0;
      wipeColor = Wheel(random(255));
    }

  }

  

  if ( millis() >= next[3] ) {
    next[3] = millis() + CHASEINTERVAL;

    int rollingPos = (chaseIndex++)%circle[3].numPixels();

    circle[3].setPixelColor(rollingPos, Wheel(rotateCounter+=2));
    circle[3].show();
    circle[3].setPixelColor(rollingPos, 0);

    /*
    if (rollingForward) {
      if (++rollingPos == circle[3].numPixels()) {
        rollingPos--;
        rollingForward = false;
      }
    }
    else {
      if (rollingPos == 0) {
        rollingForward = true;
      }
      else {
        rollingPos--;
      }
    }
    */
  }

  
}

void rollingReset() {

  for (byte i=0; i<NUMCIR; i++) {
    circle[i].clear();
    circle[i].setBrightness(BRIROLL);
    next[i] = millis();
  }

  chaseIndex = 0;
  rainbowIndex = 0;
  rollingForward = true;  
  tick = 0;
  theaterChaseIndex = 0;
  theaterChase1 = Wheel(random(255));
  theaterChase2 = Wheel(random(255));
  wipeIndex = 0;
  wipeColor = Wheel(random(255));

}



 
/**
* Scale a value returned from a trig function to a byte value.
* [-1, +1] -> [0, 254]
* Note that we ignore the possible value of 255, for efficiency,
* and because nobody will be able to differentiate between the
* brightness levels of 254 and 255.
*/
byte trigScale(float val) {
  val += 1.0; // move range to [0.0, 2.0]
  val *= 127.0; // move range to [0.0, 254.0]
   
  return int(val) & 255;
}
 
/**
* Map an integer so that [0, striplength] -> [0, 2PI]
*/
float map2PI(int i) {
  return PI*2.0*float(i) / float(circle[1].numPixels());
}



