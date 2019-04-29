//#include <Adafruit_ADXL343.h>
#include <Adafruit_GPS.h>
#include <Adafruit_MCP9808.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_MMA8451.h>
#include <Wire.h>
#include <SPI.h>
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

//header files for telemetry
#include <SPI.h>
#include <RH_RF95.h>

//pins for telemetry
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

int16_t packetnum = 0;  // packet counter, we increment per xmission

//gps
float lattitude,longitude;

//temp
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();

//accelerometer
Adafruit_MMA8451 mma = Adafruit_MMA8451();

//change start time to reflect when the rocket actually takes off
unsigned long start_time = millis();

//to store

//FILE *f = fopen("data.txt","w");
File f;
//connect to the hardware SS pin (10 on arduino, check for the Adafruit Feather Lora)
int SD_Pin = 10;


void barometer(){
  
  Serial.print(bmp.pressure / 100.0); Serial.print(",");
  Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA)*3.28); Serial.println(",");

  if (f)
    f.print("%f %f", bmp.pressure/100.0, bmp.readAltitude(SEALEVELPRESSURE_HPA)*3.28);
}

void GPS(){

  
  if (f)
    f.print("%.4f %.4f ",GPS.lattitude, GPS.longitude);
  //fprintf(f, "%.4f %.4f ",GPS.lattitude, GPS.longitude);
}

void temp(){ 
 tempsensor.wake();

 float f = tempsensor.readTempF();

 Serial.print(f, 4); Serial.println(",");

  if (f)
    f.print("%f ",tempsensor.readTempF());
    
  tempsensor.shutdown_wake(1);
}

void accel(){
 
  mma.read();
  sensors_event_t event;
  mma.getEvent(&event);

  Serial.print("X:"); Serial.print(event.acceleration.x);
  Serial.print("  Y:"); Serial.print(event.acceleration.y);
  Serial.print("  Z:"); Serial.println(event.acceleration.z);
  
  if (f)
    f.print("%f %f %f ", event.acceleration.x, event.acceleration.y, event.acceleration.z);

}

void send_dat(){
  //telemetry code (insert the actual data to be send int he radiopacket variable
  delay(1000); // Wait 1 second between transmits, could also 'sleep' here!
  Serial.println("Transmitting..."); // Send a message to rf95_server
  
  char radiopacket[20] = "Hello World #      ";
  itoa(packetnum++, radiopacket+13, 10);
  Serial.print("Sending "); Serial.println(radiopacket);
  radiopacket[19] = 0;
  
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

void setup() {
  Serial.begin(9600);

  GPS.begin(9600);

  GPS.sendCommand(PGCMD_ANTENNA);

  pinMode(SD_Pin, output);

  while(!Serial);

//temp
  if(!tempsensor.begin(0x18)){
    Serial.println("tempsensor not found");
    while(1);
  }

//pressure
    if(!bmp.begin()){
    Serial.println("barometer not found");
    while(1);
  }

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);

}

//accelerometer
 if (! mma.begin()) {
    Serial.println("accelerometer couldnt start");
    while (1);
  }
  
  mma.setRange(MMA8451_RANGE_2_G);

  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
 
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  f = SD.open("data.txt", FILE_WRITE);

  //for telemetry
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

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

  rf95.setTxPower(23, false);

  delay(10);
}

void loop() {  

  Serial.print(millis()-start_time); Serial.print(",");
  barometer();
  //GPS();
  temp();
  accel(); 
  if (f)
    f.print("\n");
  
  send_dat();
  
  //close the file after 3 minutes from the rocket's launch
  if ((millis() - start_time)/1000 > 3*60 && f)
    f.close();
    
  delay(10);
}
