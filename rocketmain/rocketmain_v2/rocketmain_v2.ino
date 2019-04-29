#include <Adafruit_ADXL343.h>
#include <Adafruit_GPS.h>
#include <Wire.h>
#include <SoftwareSerial.h> 
#include <stdio.h>
#include <SD.h>

//header files for telemetry
#include <SPI.h>
#include <RH_RF95.h>

#define GPSSerial Seriall 
Adafruit_GPS GPS(&GPSSerial); 
#define GPSECHO true //make it false in the actual 

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
const int temperaturePin = 0;
float voltage, degreeC, degreeF;

//pressure
const int pressurePin = 0;
int val;
float pkPa, 
      pAtm;

//time

//change start time to reflect when the rocket actually takes off
start_time = millis();

//float T; 
  //T = 0;

//accelerometer
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

float AccelMinX = 0;
float AccelMaxX = 0;
float AccelMinY = 0;
float AccelMaxY = 0;
float AccelMinZ = 0;
float AccelMaxZ = 0;

//to store

//FILE *f = fopen("data.txt","w");
File f;
//connect to the hardware SS pin (10 on arduino, check for the Adafruit Feather Lora)
int SD_Pin = 10;

float getVoltage(int pin)
{
  return (analogRead(pin)) * 0.004882814; // check the datasheet for the voltage conversion
}

void barometer(){
  val = analogRead(pressurePin);
  pkPa = ((float)val/(float)1023+0.095)/0.009; //CHECK THE DATASHEET  
  //pAtm = kpa2atm*pkPa; //check the buildin function

  Serial.print(pkPa);
  Serial.print("kPa\n");
  Serial.print(pAtm);
  Serial.print("Atm\n");

  if (f)
    f.print("%f ", pkPa);
  //fprintf (f, "%f ", pkPa);
}

void GPS(){

  GPS.read();
  if (f)
    f.print("%.4f %.4f ",GPS.lattitude, GPS.longitude);
  //fprintf(f, "%.4f %.4f ",GPS.lattitude, GPS.longitude);
}

void temp(){ 
  voltage = getVoltage(temperaturePin);
  degreeC = (voltage - 0.5)*100.0;
  degreeF = degreeC*(9.0/5.0) + 32.0;

  Serial.print("Voltage: ");
  Serial.print(voltage);
  Serial.print("\ndeg C");
  Serial.print(degreeC);
  Serial.print("\ndegF: ");
  Serial.print(degreeF);

  if (f)
    f.print("%f ",degreeF);
  //fprintf(f, "%f ",degreeF);
}

void accel(){
  sensors_event_t accelEvent;
  accel.getEvent(&accelEvent);
  Serial.read();
  if (f)
    f.print("%f %f %f ", accelEvent.acceleration.x, accelEvent.acceleration.y, accelEvent.acceleration.z);
  //fprintf(f, "%f %f %f ", accelEvent.acceleration.x, accelEvent.acceleration.y, accelEvent.acceleration.z) ;
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
  barometer();
  GPS();
  temp();
  accel(); 
  if (f)
    f.print("\n");
  //fprintf(f, "\n");
  //sense = {pAtm, gps.location.lat(), gps.location.lng(), degreeC, T};
  
  //T = T + 0.1;
  send_dat();
  
  //close the file after 3 minutes from the rocket's launch
  if ((millis() - start_time)/1000 > 3*60 && f)
    f.close();
    
  delay(10);
}
