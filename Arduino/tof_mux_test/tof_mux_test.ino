/*
This example shows how to take simple range measurements with the VL53L1X. The
range readings are in units of mm.
*/

#include <Wire.h>
#include <VL53L1X.h>

#define TCA9548A 0x70

VL53L1X sensor;

void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCA9548A);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void tofSensorSetup(uint8_t i) {
  tcaselect(i);

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.print(F("Failed to detect and initialize sensor "));
    Serial.print(i);
    Serial.println(F("!"));
    while (1);
  }

  // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
  // You can change these settings to adjust the performance of the sensor, but
  // the minimum timing budget is 20 ms for short distance mode and 33 ms for
  // medium and long distance modes. See the VL53L1X datasheet for more
  // information on range and timing limits.
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(50000);

  // Start continuous readings at a rate of one measurement every 50 ms (the
  // inter-measurement period). This period should be at least as long as the
  // timing budget.
  sensor.startContinuous(50);
}

void setup()
{
  while (!Serial) {}
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C
  
  tofSensorSetup(0);
  tofSensorSetup(1);
  tofSensorSetup(2);
  // tofSensorSetup(3);
}

void loop()
{
  int sensor0Val, sensor1Val, sensor2Val, sensor3Val;

  tcaselect(0);
  if (sensor.timeoutOccurred()) {
    Serial.print("Sensor 0 TIMEOUT"); 
  }
  sensor0Val = sensor.read();

  tcaselect(1);
  if (sensor.timeoutOccurred()) {
    Serial.print("Sensor 1 TIMEOUT"); 
  }
  sensor1Val = sensor.read();

  tcaselect(2);
  if (sensor.timeoutOccurred()) {
    Serial.print("Sensor 2 TIMEOUT"); 
  }
  sensor2Val = sensor.read();
  
  // tcaselect(3);
  // if (sensor.timeoutOccurred()) {
  //   Serial.print("Sensor 3 TIMEOUT"); 
  // }
  // sensor3Val = sensor.read();

  Serial.print("Sensor 0: ");
  Serial.print(sensor0Val);
  Serial.print(" | Sensor 1: ");
  Serial.print(sensor1Val);
  Serial.print(" | Sensor 2: ");
  Serial.print(sensor2Val);
  Serial.print(" | Sensor 3: ");
  Serial.println(sensor3Val);

  Serial.println();
}