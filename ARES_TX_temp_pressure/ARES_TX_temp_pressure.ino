// Feather9x_TX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (transmitter)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example Feather9x_RX

#include <SPI.h>
#include <RH_RF95.h>
 
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0
 
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

//pressure
const int pressurePin = 0;
int val;
float pkPa; 
      //pAtm;

//temp
const int temperaturePin = 0;
float voltage, degreeC, degreeF;

//File
//File f;

float getVoltage(int pin)
{
  return (analogRead(pin)) * 0.004882814; // check the datasheet for the voltage conversion
}

void barometer(){
  val = analogRead(pressurePin);
  pkPa = ((float)val/(float)1023+0.095)/0.009; //CHECK THE DATASHEET  
  //pAtm = kpa2atm()*pkPa; //check the buildin function
  
  Serial.print("The pressure in Pa is: ");
  Serial.print(pkPa);
  Serial.print("kPa\n");
  
  //Serial.print(pAtm);
  //Serial.print("Atm\n");
/*
  if (f)
    f.print("%f ", pkPa);
  //fprintf (f, "%f ", pkPa);
*/
}

void temp(){ 
  voltage = getVoltage(temperaturePin);
  degreeC = (voltage - 0.5)*100.0;
  degreeF = degreeC*(9.0/5.0) + 32.0;

  //Serial.print("Voltage: ");
  //Serial.print(voltage);
  //Serial.print("\ndeg C: ");
  //Serial.print(degreeC);
  //Serial.print("\ndegF: ");
  //Serial.println(degreeF);

/*
  if (f)
    f.print("%f ",degreeF);
    */
  //fprintf(f, "%f ",degreeF);
}
 
void setup() 
{
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
 
  Serial.begin(115200);
  while (!Serial) {
    delay(1);
  }
 
  delay(100);
 
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
}
 
int16_t packetnum = 0;  // packet counter, we increment per xmission
//float num = 0.0;
 
void loop()
{
  delay(1000); // Wait 1 second between transmits, could also 'sleep' here!
  barometer();
  temp();
  Serial.println("Transmitting..."); // Send a message to rf95_server

  float radiopacket[20];
  radiopacket[0] = pkPa;
  radiopacket[1] = degreeF;
  //char radiopacket[20] = "Hello World #      ";
  /*
  itoa(f, string, 10);
  strcat(string, “.”);                   //append decimal point
  uint16_t i = (f – (int)f) * 1000;      //subtract to get the decimals, and multiply by 1000
  itoa(i, string2, 10);                  //convert to a second string
  strcat(string, string2);               //and append to the first
  //ftoa(num+=0.01, radiopacket+13, 3);
  */
  //itoa(packetnum++, radiopacket+13, 10);
  Serial.print("Sending "); 
  Serial.print(radiopacket[0]); Serial.print(radiopacket[1]);
  Serial.print("\n");
  radiopacket[19] = 0;
  
  Serial.println("Sending...");
  delay(10);
  rf95.send((uint8_t *)radiopacket, sizeof(radiopacket));
 
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
