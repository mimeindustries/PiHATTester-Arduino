#include <SoftwareWire.h>
#include "EEPROM.h"

//#define DEBUG

#define BLUE_LED_HAT 11
#define RED_LED_HAT 12
#define GREEN_LED_HAT 13

#define BLUE_LED_TEST 2
#define RED_LED_TEST 4
#define GREEN_LED_TEST 3

#define RESET_BUTTON 6
#define START_BUTTON 5

typedef enum {RED, GREEN, ORANGE, BLUE} ledColour;

SoftwareWire adc(10, 9);
SoftwareWire eeprom(7, 8);

long debounce;

void readADC(uint8_t * values){
  adc.beginTransmission(B1001000); // wake up PCF8591
  adc.write(0x04); // control byte - read ADC0 and increment counter
  adc.endTransmission();
  adc.requestFrom(B1001000, 6);
  adc.read(); // Padding bytes to allow conversion to complete
  values[0] = adc.read();
  values[1] = adc.read();
  values[2] = adc.read();
  values[3] = adc.read();
}

boolean testADC(){
  uint8_t adcValues[4];
  boolean pass = true;
  readADC(adcValues);
  for(uint8_t i=0; i<4; i++){
    if(adcValues[i] > 135) pass = false;
    if(adcValues[i] < 119) pass = false;
  }
  if(pass){
    Serial.println("ADC test passed");
    return true;
  }else{
    Serial.println("ADC test failed");
#ifdef DEBUG
    for(uint8_t i=0; i<4; i++){
      Serial.println(adcValues[i]);
    }
#endif
    return false;
  }
}

void writeByte(uint16_t dataAddress, uint8_t data){
  eeprom.beginTransmission((byte)(EEPROM_ADDRESS));
  eeprom.write(dataAddress >> 8);
  eeprom.write(dataAddress & 0xFF);
  eeprom.write(data);
  eeprom.endTransmission();
  delay(10);
}

uint8_t readByte(uint16_t dataAddress){
  eeprom.beginTransmission((byte)(EEPROM_ADDRESS));
  eeprom.write(dataAddress >> 8);
  eeprom.write(dataAddress & 0xFF);
  eeprom.endTransmission();
  eeprom.requestFrom((byte)(EEPROM_ADDRESS), (byte)1);
  return eeprom.read();
}

void writeEEPROMData(){
#ifdef DEBUG
  Serial.println("Writing EEPROM Data");
#endif
  for(uint16_t i=0; i<sizeof(eepromData); i++){
#ifdef DEBUG
    Serial.println(pgm_read_byte(eepromData + i), HEX);
#endif
    writeByte(i, pgm_read_byte(eepromData + i));
    delay(1);
  }
}

boolean verifyEEPROMData(){
  byte data;
#ifdef DEBUG
  Serial.println("Reading EEPROM Data");
#endif
  for(uint16_t i=0; i<sizeof(eepromData); i++){
    data = readByte(i);
#ifdef DEBUG
    Serial.println(data, HEX);
#endif
    if(data != pgm_read_byte(eepromData + i)){
      return false;
    }
  }
  return true;
}

boolean testEEPROM(){
  writeEEPROMData();
  delay(1);
  if(verifyEEPROMData()){
    Serial.println("EEPROM test passed");
    return true;
  }else{
    Serial.println("EEPROM test failed");
    return false;
  }
}

boolean testLED(){
  int red, green, blue;
  
  digitalWrite(RED_LED_HAT, HIGH);
  digitalWrite(GREEN_LED_HAT, HIGH);
  digitalWrite(BLUE_LED_HAT, HIGH);
  
  red = analogRead(A1);
  green = analogRead(A2);
  blue = analogRead(A0);

  
  digitalWrite(RED_LED_HAT, LOW);
  digitalWrite(GREEN_LED_HAT, LOW);
  digitalWrite(BLUE_LED_HAT, LOW);

  if(
    red < 740 || red > 870 ||
    green < 850 || green > 970 ||
    blue < 850 || blue > 970
  ){
    Serial.println("LED test failed");
#ifdef DEBUG
    Serial.println(red);
    Serial.println(green);
    Serial.println(blue);
#endif
    return false;
  }
  Serial.println("LED test passed");
  return true;
}

void setLED(ledColour colour){
  digitalWrite(RED_LED_TEST, LOW);
  digitalWrite(GREEN_LED_TEST, LOW);
  digitalWrite(BLUE_LED_TEST, LOW);
  switch(colour){
    case GREEN:
      digitalWrite(GREEN_LED_TEST, HIGH);
      break;
    case RED:
      digitalWrite(RED_LED_TEST, HIGH);
      break;
    case BLUE:
      digitalWrite(BLUE_LED_TEST, HIGH);
      break;
    case ORANGE:
      digitalWrite(GREEN_LED_TEST, HIGH);
      digitalWrite(RED_LED_TEST, HIGH);
      break;
  }
}

void ready(){
  // Set the led to blue
  setLED(BLUE);
}

void test(){
  // Set the LED orange
  Serial.println("");
  Serial.println("Testing Board");
  setLED(ORANGE);
  if(testADC() &&
     testLED() &&
     testEEPROM()){
    pass();   
  }else{
    fail();
  }
}

void pass(){
  // Set the LED green
  setLED(GREEN);
  Serial.println("Board passed");
}

void fail(){
  // Set the LED red
  setLED(RED);
  Serial.println("Board failed");
}

void setup() {
  pinMode(RED_LED_HAT, OUTPUT);
  pinMode(GREEN_LED_HAT, OUTPUT);
  pinMode(BLUE_LED_HAT, OUTPUT);
  
  pinMode(RED_LED_TEST, OUTPUT);
  pinMode(GREEN_LED_TEST, OUTPUT);
  pinMode(BLUE_LED_TEST, OUTPUT);
  
  pinMode(START_BUTTON, INPUT_PULLUP);
  pinMode(RESET_BUTTON, INPUT_PULLUP);
  
  adc.begin();
  eeprom.begin();
  ready();
  Serial.begin(57600);
  test();
}

void loop() {
  long now = millis();
  if(!digitalRead(START_BUTTON) && debounce < now){
    debounce = now + 1000;
    test();
  }
  if(!digitalRead(RESET_BUTTON) && debounce < now){
    debounce = now + 1000;
    ready();
  }
}
