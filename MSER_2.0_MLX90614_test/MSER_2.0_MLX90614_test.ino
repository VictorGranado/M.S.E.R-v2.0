#include <Wire.h>
#include <Adafruit_MLX90614.h>

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!mlx.begin()) {
    Serial.println("Sensor not found. Check wiring!");
    while (1);
  }
}

void loop() {
  Serial.println(mlx.readObjectTempC());
  delay(1000);  // 1 second delay
}
