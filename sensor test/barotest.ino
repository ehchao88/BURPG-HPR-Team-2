#include <Adafruit_BMP3XX.h>
#include <bmp3.h>
#include <bmp3_defs.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  while(!Serial);

  if(!bmp.begin()){
    Serial.println("not found");
    while(1);
  }

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);

}

void loop() {
  // put your main code here, to run repeatedly:

  if (!bmp.performReading()){
    Serial.println("failed reading");
    return;
  }

  Serial.print("pressure: "); Serial.print(bmp.pressure / 100.0); Serial.println("hPa");
  Serial.print("approx. alt: "); Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA)*3.28); Serial.println("ft");

  delay(100);

}
