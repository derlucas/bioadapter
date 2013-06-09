#include <Wire.h>  // We use Wire.h to talk I2C to the sensor

// Digital Pins
#define LEDRED 7
#define LEDGREEN 4
#define LEDBLUE 2

#define MOTOR1 3
#define MOTOR2 5
#define MOTOR3 6
#define MOTOR4 9

#define DISTANCE 8

#define HALOGEN 12
#define FAN 11

// Analogue pins
#define BRIGHTNESS A0
#define ACCELX A1
#define ACCELY A2
#define ACCELZ A3

#define ADJD_S311_ADDRESS 0x74
#define RED 0
#define GREEN 1
#define BLUE 2
#define CLEAR 3

// ADJD-S311's register list
#define CTRL 0x00
#define CONFIG 0x01
#define CAP_RED 0x06
#define CAP_GREEN 0x07
#define CAP_BLUE 0x08
#define CAP_CLEAR 0x09
#define INT_RED_LO 0xA
#define INT_RED_HI 0xB
#define INT_GREEN_LO 0xC
#define INT_GREEN_HI 0xD
#define INT_BLUE_LO 0xE
#define INT_BLUE_HI 0xF
#define INT_CLEAR_LO 0x10
#define INT_CLEAR_HI 0x11
#define DATA_RED_LO 0x40
#define DATA_RED_HI 0x41
#define DATA_GREEN_LO 0x42
#define DATA_GREEN_HI 0x43
#define DATA_BLUE_LO 0x44
#define DATA_BLUE_HI 0x45
#define DATA_CLEAR_LO 0x46
#define DATA_CLEAR_HI 0x47
#define OFFSET_RED 0x48
#define OFFSET_GREEN 0x49
#define OFFSET_BLUE 0x4A
#define OFFSET_CLEAR 0x4B

// Pin definitions:
int sdaPin = A4;  // serial data, hardwired, can't change
int sclPin = A5;  // serial clock, hardwired, can't change
int ledPin = 10;  // LED light source pin, any unused pin will work
// initial values for integration time registers
unsigned char colorCap[4] = {9, 9, 2, 5};  // values must be between 0 and 15
unsigned int colorInt[4] = {2048, 2048, 2048, 2048};  // max value for these is 4095
unsigned int colorData[4];  // This is where we store the RGB and C data values
signed char colorOffset[4];  // Stores RGB and C offset values


boolean stringComplete = false;
char inData[100];
String inString = "";
int index = 0;

// L üfter  H alogen    W X Y Z finger

void setup() {
  Serial.begin(115200);
  
  pinMode(LEDBLUE, OUTPUT);
  pinMode(LEDGREEN, OUTPUT);
  pinMode(LEDRED, OUTPUT);  
  pinMode(10, OUTPUT);
  
  pinMode(MOTOR1, OUTPUT);
  pinMode(MOTOR2, OUTPUT);
  pinMode(MOTOR3, OUTPUT);
  pinMode(MOTOR4, OUTPUT);
  
  digitalWrite(LEDRED, LOW);
  digitalWrite(LEDGREEN, LOW);
  digitalWrite(LEDBLUE, LOW);
  digitalWrite(10, HIGH);  
  
  delay(10);
  
  
  digitalWrite(LEDRED, LOW);
  digitalWrite(LEDGREEN, HIGH);
  digitalWrite(LEDBLUE, LOW);
  
  Wire.begin();
  delay(1);
  initADJD_S311();  // Initialize the ADJD-S311, sets up cap and int registers
  
  
  uint8_t i;
  for(i = 0; i< 10; i++) {
    digitalWrite(10, HIGH);
    delay(100);
    digitalWrite(10, LOW);
    delay(100);
  }
  digitalWrite(10, HIGH);
  calibrateADJD();
  digitalWrite(10, LOW);
  
  for(i = 0; i< 2; i++) {
    digitalWrite(10, HIGH);
    delay(200);
    digitalWrite(10, LOW);
    delay(200);
  }
  digitalWrite(10, HIGH);
}

int val = 0;


void calibrateADJD() {
  calibrateColor();  // This calibrates R, G, and B int registers
  calibrateClear();  // This calibrates the C int registers
  calibrateCapacitors();  // This calibrates the RGB, and C cap registers
} 

void loop() {  
  
  outputValues();
  
  delay(100);
  
  if(Serial.available() ) {  
    char ident = Serial.read();
    int value = Serial.parseInt();
    
    if(Serial.read() == 'a') {
      switch(ident) {
        case 'R':
          if(value > 0) digitalWrite(LEDRED, HIGH);
          else digitalWrite(LEDRED, LOW);
          break;
        case 'G':
          if(value > 0) digitalWrite(LEDGREEN, HIGH);
          else digitalWrite(LEDGREEN, LOW);
          break;
        case 'B':
          if(value > 0) digitalWrite(LEDBLUE, HIGH);
          else digitalWrite(LEDBLUE, LOW);
          break;
        case 'W':
          analogWrite(MOTOR1, value);
          break;
        case 'X':
          analogWrite(MOTOR2, value);
          break;
        case 'Y':
          analogWrite(MOTOR3, value);
          break;
        case 'Z':
          analogWrite(MOTOR4, value);
          break;
        case 'L':
          if(value > 0) digitalWrite(FAN, HIGH);
          else digitalWrite(FAN, LOW);
          break;
        case 'H':
          if(value > 0) digitalWrite(HALOGEN, HIGH);
          else digitalWrite(HALOGEN, LOW);
          break;
      }
    }
  }
}


void outputValues() {
  Serial.print("D");
  Serial.print(getDistance());
  Serial.print("C");
  Serial.print(getBrightness());
  
  Serial.print("X");
  Serial.print(getAcceloX());
  Serial.print("Y");
  Serial.print(getAcceloY());
  Serial.print("Z");
  Serial.print(getAcceloZ());
 
  
  getRGBC();  
  Serial.print("R");
  Serial.print((colorData[0]/4));
  Serial.print("G");
  Serial.print((colorData[1]/4));
  Serial.print("B");
  Serial.print((colorData[2]/4));
}

uint8_t getBrightness() {
  uint16_t x;
  uint8_t i;
  
  for(i = 0; i < 4; i++) {
    x += analogRead(BRIGHTNESS);
  }
  return (1024- x) / 4;
}

uint8_t getAcceloX() {
  uint16_t x;
  uint8_t i;
  
  for(i = 0; i < 4; i++) {
    x += analogRead(ACCELX);
  }
  return (x / 4);
}
uint8_t getAcceloY() {
  uint16_t x;
  uint8_t i;
  
  for(i = 0; i < 4; i++) {
    x += analogRead(ACCELY);
  }
  return (x / 4);
}

uint8_t getAcceloZ() {
  uint16_t x;
  uint8_t i;
  
  for(i = 0; i < 4; i++) {
    x += analogRead(ACCELZ);
  }
  return (x / 4);
}

uint8_t getDistance() {
  uint8_t i;
  uint8_t x;
  
  for(i = 0; i < 2; i++) {
    x += digitalRead(DISTANCE);
    delay(5);
  }
  
  if( x == 0 ) return 1;
  return 0;
}

  

void initADJD_S311() { 

  writeRegister(colorCap[RED] & 0xF, CAP_RED);
  writeRegister(colorCap[GREEN] & 0xF, CAP_GREEN);
  writeRegister(colorCap[BLUE] & 0xF, CAP_BLUE);
  writeRegister(colorCap[CLEAR] & 0xF, CAP_CLEAR);
  writeRegister((unsigned char)colorInt[RED], INT_RED_LO);
  writeRegister((unsigned char)((colorInt[RED] & 0x1FFF) >> 8), INT_RED_HI);
  writeRegister((unsigned char)colorInt[BLUE], INT_BLUE_LO);
  writeRegister((unsigned char)((colorInt[BLUE] & 0x1FFF) >> 8), INT_BLUE_HI);
  writeRegister((unsigned char)colorInt[GREEN], INT_GREEN_LO);
  writeRegister((unsigned char)((colorInt[GREEN] & 0x1FFF) >> 8), INT_GREEN_HI);
  writeRegister((unsigned char)colorInt[CLEAR], INT_CLEAR_LO);
  writeRegister((unsigned char)((colorInt[CLEAR] & 0x1FFF) >> 8), INT_CLEAR_HI);
}

int calibrateClear()
{
  int gainFound = 0;
  int upperBox=4096;
  int lowerBox = 0;
  int half;
  
  while (!gainFound)
  {
    half = ((upperBox-lowerBox)/2)+lowerBox;
    //no further halfing possbile
    if (half==lowerBox)
      gainFound=1;
    else 
    {
      writeInt(INT_CLEAR_LO, half);
      performMeasurement();
      int halfValue = readRegisterInt(DATA_CLEAR_LO);

      if (halfValue>1000)
        upperBox=half;
      else if (halfValue<1000)
        lowerBox=half;
      else
        gainFound=1;
    }
  }
  return half;
}


int calibrateColor()
{
  int gainFound = 0;
  int upperBox=4096;
  int lowerBox = 0;
  int half;
  
  while (!gainFound)
  {
    half = ((upperBox-lowerBox)/2)+lowerBox;
    //no further halfing possbile
    if (half==lowerBox)
    {
      gainFound=1;
    }
    else {
      writeInt(INT_RED_LO, half);
      writeInt(INT_GREEN_LO, half);
      writeInt(INT_BLUE_LO, half);

      performMeasurement();
      int halfValue = 0;

      halfValue=max(halfValue, readRegisterInt(DATA_RED_LO));
      halfValue=max(halfValue, readRegisterInt(DATA_GREEN_LO));
      halfValue=max(halfValue, readRegisterInt(DATA_BLUE_LO));

      if (halfValue>1000) {
        upperBox=half;
      }
      else if (halfValue<1000) {
        lowerBox=half;
      }
      else {
        gainFound=1;
      }
    }
  }
  return half;
}


void calibrateCapacitors()
{
  int  calibrationRed = 0;
  int  calibrationBlue = 0;
  int  calibrationGreen = 0;
  int calibrated = 0;

  //need to store detect better calibration
  int oldDiff = 5000;

  while (!calibrated)
  {
    // sensor gain setting (Avago app note 5330)
    // CAPs are 4bit (higher value will result in lower output)
    writeRegister(calibrationRed, CAP_RED);
    writeRegister(calibrationGreen, CAP_GREEN);
    writeRegister(calibrationBlue, CAP_BLUE);

    // int colorGain = _calibrateColorGain();
    int colorGain = readRegisterInt(INT_RED_LO);
    writeInt(INT_RED_LO, colorGain);
    writeInt(INT_GREEN_LO, colorGain);
    writeInt(INT_BLUE_LO, colorGain);

    int maxRead = 0;
    int minRead = 4096;
    int red   = 0;
    int green = 0;
    int blue  = 0;
    
    for (int i=0; i<4 ;i ++)
    {
      performMeasurement();
      red   += readRegisterInt(DATA_RED_LO);
      green += readRegisterInt(DATA_GREEN_LO);
      blue  += readRegisterInt(DATA_BLUE_LO);
    }
    red   /= 4;
    green /= 4;
    blue  /= 4;

    maxRead = max(maxRead, red);
    maxRead = max(maxRead, green);
    maxRead = max(maxRead, blue);

    minRead = min(minRead, red);
    minRead = min(minRead, green);
    minRead = min(minRead, blue);

    int diff = maxRead - minRead;

    if (oldDiff != diff)
    {
      if ((maxRead==red) && (calibrationRed<15))
        calibrationRed++;
      else if ((maxRead == green) && (calibrationGreen<15))
        calibrationGreen++;
      else if ((maxRead == blue) && (calibrationBlue<15))
        calibrationBlue++;
    }
    else
      calibrated = 1;
      
    oldDiff=diff;

    int rCal = calibrationRed;
    int gCal = calibrationGreen;
    int bCal = calibrationBlue;
  }
  
}


void writeInt(int address, int gain)
{
  if (gain < 4096) 
  {
    byte msb = gain >> 8;
    byte lsb = gain;

    writeRegister(lsb, address);
    writeRegister(msb, address+1);
  }
}


void performMeasurement()
{  
  writeRegister(0x01, 0x00); // start sensing
  while(readRegister(0x00) != 0)
    ; // waiting for a result
}


void getRGBC()
{
  performMeasurement();
  
  colorData[RED] = readRegisterInt(DATA_RED_LO);
  colorData[GREEN] = readRegisterInt(DATA_GREEN_LO);
  colorData[BLUE] = readRegisterInt(DATA_BLUE_LO);
  colorData[CLEAR] = readRegisterInt(DATA_CLEAR_LO);
}


void getOffset()
{
  digitalWrite(ledPin, LOW);  // turn LED off
  delay(10);  // wait a tic
  writeRegister(0x02, 0x00); // start sensing
  while(readRegister(0x00) != 0)
    ; // waiting for a result
  //writeRegister(0x01, 0x01);  // set trim
  //delay(100);
  for (int i=0; i<4; i++)
    colorOffset[i] = (signed char) readRegister(OFFSET_RED+i);
  digitalWrite(ledPin, HIGH);
}


void writeRegister(unsigned char data, unsigned char address)
{
  Wire.beginTransmission(ADJD_S311_ADDRESS);
  Wire.write(address);
  Wire.write(data);
  Wire.endTransmission();
}


unsigned char readRegister(unsigned char address)
{
  unsigned char data;
  
  Wire.beginTransmission(ADJD_S311_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();
  
  Wire.requestFrom(ADJD_S311_ADDRESS, 1);
  while (!Wire.available())
    ;  // wait till we can get data
  
  return Wire.read();
}

// Write two bytes of data to ADJD-S311 address and addres+1
int readRegisterInt(unsigned char address)
{
  return readRegister(address) + (readRegister(address+1)<<8);
}




