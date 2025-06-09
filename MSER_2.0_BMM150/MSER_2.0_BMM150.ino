#include "DFRobot_BMM150.h"

// Use I2C address 0x13 (default)
DFRobot_BMM150_I2C bmm150(&Wire, I2C_ADDRESS_4);  // CS:1 SDO:1

void setup() {
  Serial.begin(115200);
  while (!Serial);

  while (bmm150.begin()) {
    Serial.println("BMM150 init failed. Please check wiring!");
    delay(1000);
  }
  Serial.println("BMM150 init success!");

  bmm150.setOperationMode(BMM150_POWERMODE_NORMAL);
  bmm150.setPresetMode(BMM150_PRESETMODE_HIGHACCURACY);
  bmm150.setRate(BMM150_DATA_RATE_10HZ);
  bmm150.setMeasurementXYZ();
}

String getCardinalDirection(float heading) {
  if (heading < 22.5 || heading >= 337.5) return "N";
  if (heading < 67.5) return "NE";
  if (heading < 112.5) return "E";
  if (heading < 157.5) return "SE";
  if (heading < 202.5) return "S";
  if (heading < 247.5) return "SW";
  if (heading < 292.5) return "W";
  return "NW";
}

void loop() {
  sBmm150MagData_t magData = bmm150.getGeomagneticData();

  // Compute heading
  float heading = bmm150.getCompassDegree();
  String direction = getCardinalDirection(heading);

  // Calculate magnetic field strength (magnitude)
  float strength = sqrt(pow(magData.x, 2) + pow(magData.y, 2) + pow(magData.z, 2));

  // Display all data
  Serial.println("======== Magnetometer Data ========");
  Serial.print("Heading: "); Serial.print(heading, 1); Serial.print("° ");
  Serial.println("(" + direction + ")");

  Serial.print("Magnetic Strength: "); Serial.print(strength, 1); Serial.println(" uT");

  Serial.print("X: "); Serial.print(magData.x); Serial.print(" uT\t");
  Serial.print("Y: "); Serial.print(magData.y); Serial.print(" uT\t");
  Serial.print("Z: "); Serial.print(magData.z); Serial.println(" uT");

  Serial.println("------------------------------------");
  delay(1000);
}

