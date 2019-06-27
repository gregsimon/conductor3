// Copyright 2019 Greg Simon
// MIDI Conductor 3D
// Version 1.0

#include "Adafruit_VL53L0X.h"

// Rangefinders
Adafruit_VL53L0X sensorA = Adafruit_VL53L0X(); // SDA1
Adafruit_VL53L0X sensorB = Adafruit_VL53L0X(); // SDA0
Adafruit_VL53L0X sensorC = Adafruit_VL53L0X();


// the MIDI channel number to send messages
const int channel = 1;

bool use_serial = false;

// Wired to XSHUT of each rangefinder
const int pin_sensorA = 32;
const int pin_sensorB = 30;
const int pin_sensorC = 28;

const uint8_t addr_sensorA = 0x29;
const uint8_t addr_sensorB = 0x29;
const uint8_t addr_sensorC = 0x29;

int last_b = -1; // for detecting vibrato
uint32_t last_b_time = 0;


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
  if (use_serial) 
    Serial.begin(9600);

  pinMode(31, INPUT_PULLDOWN);    // Interrupt SensorA
  pinMode(pin_sensorA, OUTPUT);   // XSHUT SensorA
  digitalWrite(pin_sensorA, 1);

  pinMode(29, INPUT_PULLDOWN);    // Interrupt SensorA
  pinMode(pin_sensorB, OUTPUT);   // XSHUT SensorB
  digitalWrite(pin_sensorB, 1);

  pinMode(27, INPUT_PULLDOWN);    // Interrupt SensorA
  pinMode(pin_sensorC, OUTPUT);   // XSHUT SensorB
  digitalWrite(pin_sensorC, 1);


  if (use_serial)
    Serial.println(F("MIDI Conductor3"));

  if (!sensorA.begin(addr_sensorA, false, &Wire1)) {
    if (use_serial) Serial.println(F("Failed to boot sensor A"));
    while(1);
  }  
  if (!sensorB.begin(addr_sensorA, false, &Wire)) {
    if (use_serial) Serial.println(F("Failed to boot sensor B"));
    while(1);
  }  
  if (!sensorC.begin(addr_sensorC, false, &Wire2)) {
    if (use_serial) Serial.println(F("Failed to boot sensor C"));
    while(1);
  }  


  // USB MIDI seutp
  //usbMIDI.setHandleSystemExclusive(onSystemExclusiveChunk);
}

int range_clamp(int low, int high, int value) {
  if (value <= low)
    return low;
  if (value >= high)
    return high;
  return value;
}

void loop() {
  VL53L0X_RangingMeasurementData_t measureA;
  VL53L0X_RangingMeasurementData_t measureB;
  VL53L0X_RangingMeasurementData_t measureC;

  //Serial.print("Reading a measurement... ");
  sensorA.rangingTest(&measureA, false); // pass in 'true' to get debug data printout!
  sensorB.rangingTest(&measureB, false); // pass in 'true' to get debug data printout!
  sensorC.rangingTest(&measureC, false); // pass in 'true' to get debug data printout!

  // Data Mapping
  // Sensor B (X) 
  // Sensor C (Y) 40 .. 350 --> 0 .. 127

  // DYNAMICS
  // Sensor A (Z) 0 .. 400 --> 127 .. 0
  if (measureA.RangeStatus != 4) {
    int a = range_clamp(0, 500, measureA.RangeMilliMeter);
    a = 500 - a;
    a = (int)((float)a / 3.93);
    usbMIDI.sendControlChange(1, a, channel);
  }

  // Sensor B (X)
  // Centered is ~180  Look at ~150 off either side.
  if (measureB.RangeStatus != 4) {
    if (-1 == last_b) {
      last_b = measureB.RangeMilliMeter;
    } else {
      int b = measureB.RangeMilliMeter;

      //float value = 100.0 * abs(float(b - last_b) / float(time_stamp - last_b_time));
      // {
        //float v = abs(b - last_b) / 2.0;
      //if (use_serial) Serial.println(b);

      int value = 0;
      if (b < 120) {
        // range will be 30 .. 120, map to [127 .. 0]
        value = range_clamp(30, 120, b) - 30;
        value = (int)(1.4 * (float)(90.0-value));

        if (use_serial) Serial.println(value);
      }

      usbMIDI.sendControlChange(21, value, channel);

      last_b = b;
    }
  }

  // EXPRESSION
  // Sensor C (Y) 40 .. 350 --> 0 .. 127
  if (measureC.RangeStatus != 4) {
    int c = range_clamp(40, 250, measureC.RangeMilliMeter);
    c = c - 40;
    c = (int)((float)c / (210.0/127.0));
    usbMIDI.sendControlChange(11, c, channel);
  }


  if (use_serial) {
    if (measureA.RangeStatus != 4) {  // phase failures have incorrect data
      //Serial.print("Distance (mm): [A] "); Serial.println(measureA.RangeMilliMeter);
    } else {
      //Serial.println(" [A] out of range ");
    }
  
    if (measureB.RangeStatus != 4) {  // phase failures have incorrect data
      //Serial.print("Distance (mm): [B] "); Serial.println(measureB.RangeMilliMeter);
    } else {
      //Serial.println(" [B] out of range ");
    }
  
    if (measureC.RangeStatus != 4) {  // phase failures have incorrect data
      //Serial.print("Distance (mm): [C] "); Serial.println(measureC.RangeMilliMeter);
    } else {
      //Serial.println(" [C] out of range ");
    }
  }
  
  delay(10);

  //usbMIDI.read();
  //usbMIDI.sendControlChange(1, (1024-value)/8, channel);
  //usbMIDI.sendControlChange(2, value/8, channel);
  
  //while (usbMIDI.read()) {
    // ignore incoming messages
  //}
}
