#include "Adafruit_MCP9808.h"
#include <Wire.h>

Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while(!Serial);
  //Serial.println("MCP9808 demo"); //check if the serial monitor works

  if (!tempsensor.begin(0x18)) {
    Serial.println("not found");
    while(1);
  }

  //Serial.println("Found MCP9808!");

}

void loop() {
  // put your main code here, to run repeatedly:
  tempsensor.wake();
  
  float f = tempsensor.readTempF();

  Serial.print(f, 4); Serial.print("*F\n");

  delay(100);

  tempsensor.shutdown_wake(1);

  

}
