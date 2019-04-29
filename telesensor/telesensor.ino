// Feather9x_TX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (transmitter)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example Feather9x_RX
#include <Adafruit_GPS.h>
#include <Adafruit_MCP9808.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_MMA8451.h>
#include <Wire.h>
#include <Adafruit_Sensor.h> 
#include <stdio.h>
#include <SD.h>

//barometer
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp;

#include <SPI.h>
#include <RH_RF95.h>
 
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

//state variable to start recording data
bool record = false;

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0
 
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

int16_t packetnum = 0;  // packet counter, we increment per xmission

//temp
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();

//accelerometer
Adafruit_MMA8451 mma = Adafruit_MMA8451();
float curAccel = 123;

//time
unsigned long start_time;
unsigned long t;

void barometer(){
 
  //Serial.print("pressure: "); 
  Serial.print(bmp.pressure / 100.0, 2); Serial.print("\t");
  //Serial.println("hPa");
  //Serial.print("approx. alt: "); 
  Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA)*3.28, 2); Serial.print("\t");
  //Serial.println("ft");

  float pres = bmp.pressure / 100.0;
  float alt = bmp.readAltitude(SEALEVELPRESSURE_HPA)*3.28;

}

 void temp(){ 

 float temp = tempsensor.readTempF();

 Serial.print(temp, 2); Serial.print("\t"); 

 }

void accel(){
  mma.read();
  sensors_event_t event;
  mma.getEvent(&event);
  curAccel = event.acceleration.y;

  Serial.print(event.acceleration.x, 2); Serial.print("\t");
  Serial.print(event.acceleration.y, 2); Serial.print("\t");
  Serial.println(event.acceleration.z, 2);

}
 
void setup() 
{
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
 
  Serial.begin(9600);
  while (!Serial) {
    delay(1);
  }

 
  delay(100);

  //temp
 /*if(!tempsensor.begin(0x18)){
    Serial.println("tempsensor not found");
    while(1);
  }

  tempsensor.wake();

  //pressure
    if(!bmp.begin()){
    Serial.println("barometer not found");
    while(1);

    }

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);

 //accelerometer
  if(!mma.begin()){
    Serial.println("accel not found");
    while(1);
  }

  mma.setRange(MMA8451_RANGE_2_G);*/

  //telemetry
 
  Serial.println("Feather LoRa TX Test!");
 
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
 
  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");
 
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
 
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

  Serial.println("time\tpressu\taltitu\ttemp\tx   \ty   \tz\t");
  
}
 

void loop()
{
  delay(1000); // Wait 1 second between transmits, could also 'sleep' here!
  Serial.println("Transmitting..."); // Send a message to rf95_server
  
  char radiopacket[20] = "Hello World #      ";
  
  Serial.print("Sending "); Serial.println(radiopacket);
  //radiopacket[19] = 0;
  
  Serial.println("Sending...");
  delay(10);
  rf95.send((uint8_t *)radiopacket, 20);
 
  Serial.println("Waiting for packet to complete..."); 
  delay(10);
  rf95.waitPacketSent();
  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
 
  Serial.println("Waiting for reply...");
  if (rf95.waitAvailableTimeout(1000))
  { 
    // Should be a reply message for us now   
    if (rf95.recv(buf, &len))
   {
      Serial.print("Got reply: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);    
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
  else
  {
    Serial.println("No reply, is there a listener around?");
  }
 
}
