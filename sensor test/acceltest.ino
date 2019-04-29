#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MMA8451 mma = Adafruit_MMA8451();
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); 

  if (!mma.begin()) {
    Serial.println("Couldnt start");
    while (1);
  }
  Serial.println("MMA8451 found!");
  
  mma.setRange(MMA8451_RANGE_2_G);

}

void loop() {
  // put your main code here, to run repeatedly:

  mma.read();
  sensors_event_t event;
  mma.getEvent(&event);

  
  Serial.print("X:"); Serial.print(event.acceleration.x);
  Serial.print("  Y:"); Serial.print(event.acceleration.y);
  Serial.print("  Z:"); Serial.println(event.acceleration.z);

  delay(10);

}
