// Copyright 2019 Greg Simon
// MIDI Conductor 3D
// Version 1.0

#include "Adafruit_VL53L0X.h"

// Rangefinders
Adafruit_VL53L0X sensorA = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorB = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorC = Adafruit_VL53L0X();


// the MIDI channel number to send messages
const int channel = 1;

// Wired to XSHUT of each rangefinder
const int pin_sensorA = 32;
const int pin_sensorB = 4;
const int pin_sensorC = 5;

const uint8_t addr_sensorA = 0x29;
const uint8_t addr_sensorB = 0x61;
const uint8_t addr_sensorC = 0x62;


// Since all rangefinders are on the I2C bus, we need to assign
// custom addresses to them. we'll do this by disabling them all,
// then one-by-one enabling them with specific addresses.
void setAddressesForRadar()
{
  // TODO
}

void onSystemExclusiveChunk(const uint8_t *data, uint16_t length, bool last)
{
  // TODO : adjust parameters.
}


void setup() {
  Serial.begin(9600);

  pinMode(32, INPUT_PULLDOWN);    // Interrupt SensorA
  pinMode(pin_sensorA, OUTPUT);   // XSHUT SensorA

  digitalWrite(pin_sensorA, 1);

  Serial.println(F("MIDI Conductor3"));
  if (!sensorA.begin(addr_sensorA, false, &Wire1)) {
    Serial.println(F("Failed to boot sensor A"));
    while(1);
  }  


  // USB MIDI seutp
  //usbMIDI.setHandleSystemExclusive(onSystemExclusiveChunk);
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;

  Serial.print("Reading a measurement... ");
  sensorA.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
  } else {
    Serial.println(" out of range ");
  }
    
  delay(100);

  //usbMIDI.read();
  //usbMIDI.sendControlChange(1, (1024-value)/8, channel);
  //usbMIDI.sendControlChange(2, value/8, channel);
  
  //while (usbMIDI.read()) {
    // ignore incoming messages
  //}
}