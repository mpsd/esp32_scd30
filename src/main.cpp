#include <Arduino.h>

//////////////////////////////////////////////////////////////////////////
// set this to the path of https://github.com/paulvha/scd30             //
//                                                                      //
//////////////////////////////////////////////////////////////////////////
#include "../.pio/libdeps/esp32doit-devkit-v1/paulvha SCD30 Library/src/paulvha_SCD30.h"

//////////////////////////////////////////////////////////////////////////
// set SCD30 driver debug level (only NEEDED case of errors)            //
// Requires serial monitor (remove DTR-jumper before starting monitor)  //
// 0 : no messages                                                      //
// 1 : request sending and receiving                                    //
// 2 : request sending and receiving + show protocol errors             //
//////////////////////////////////////////////////////////////////////////
#define scd_debug 0

//////////////////////////////////////////////////////////////////////////
//                SELECT THE INTERFACES                                 //
//////////////////////////////////////////////////////////////////////////
#define SCD30WIRE Wire
#define ARDUINO_ARCH_ESP32

SCD30 airSensor;

// defined for EPS32 DevKit V1 board
#define I2C_SDA 21
#define I2C_SCL 22

#define MillisToWait 300000L // 5 minutes for stabilization

void setup()
{
  Serial.begin(115200);
  Serial.println("SCD30 Calibration");

  SCD30WIRE.begin(I2C_SDA, I2C_SCL);
  SCD30WIRE.setClock(50000L);
  SCD30WIRE.setClockStretchLimit(200000L);

  if (! airSensor.begin(SCD30WIRE, true))
  {
    Serial.println(F("The SCD30 did not respond. Please check wiring."));
    while(1);
  }

  if (airSensor.StopMeasurement()) { Serial.println("Continous Measurement stopped"); }

  airSensor.setDebug(scd_debug);
  airSensor.setAutoSelfCalibration(false);
  airSensor.setMeasurementInterval(5);    //Change number of seconds between measurements: 2 to 1800 (30 minutes)
  airSensor.setAltitudeCompensation(540); //Set altitude of the sensor in m

  if (airSensor.beginMeasuring(0)) { Serial.println("Continous Measurement started"); }  // Pressure in 700 - 1200 (0..no pressure compensation)

  // Read Information
  //
  uint8_t val8[2];
  char buf[50];

  if (airSensor.getSerialNumber(buf))
  {
   Serial.print(F("SCD30 serial number : "));
   Serial.println(buf);
  }

  // read Firmware level
  if ( airSensor.getFirmwareLevel(val8) ) {
    Serial.print("SCD30 Firmware level: ");
    Serial.print(val8[0]);
    Serial.print(".");
    Serial.println(val8[1]);
  }
  else {
    Serial.println("Could not obtain firmware level");
  }
  
  uint16_t TempOffset=400;
  if (airSensor.setTemperatureOffset(TempOffset)) {
      Serial.println("Starting Temperature Recalibration");
    }
    else {
      Serial.println("FAILED");
    }
}

unsigned long StartMillis=millis();
uint16_t val16;

void loop()
{
  
  if (airSensor.dataAvailable())
  {
    Serial.print("co2(ppm):");
    Serial.print(airSensor.getCO2());

    Serial.print(" temp(C):");
    Serial.print(airSensor.getTemperature(), 1);

    Serial.print(" humidity(%):");
    Serial.print(airSensor.getHumidity(), 1);

    Serial.print(" Temp Offset:");
    airSensor.getTemperatureOffset(&val16);
    Serial.print(val16, 1);

    Serial.println();
  }
  else
    Serial.print(".");

  if ( millis() - StartMillis > MillisToWait) {
    Serial.print("Starting SCD30 calibration: ");
    if (airSensor.setForceRecalibration(400)) {
      Serial.println("Starting Forced Recalibration");
    }
    else {
      Serial.println("FAILED");
    }

    delay(5000);
    StartMillis = millis();
  }

  delay(500);
}