#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);
Adafruit_BME280 bme;

float calculateDewPoint(float temp, float hum) {
  double a = 17.27;
  double b = 237.7;
  double alpha = ((a * temp) / (b + temp)) + log(hum / 100.0);
  return (b * alpha) / (a - alpha);
}

float calculateFeelsLike(float temp, float hum) {
  return temp + 0.33 * hum - 4.0;
}

void setup() {
  lcd.init();
  lcd.backlight();

  lcd.setCursor(0, 0);
  lcd.print("Testing BME280...");
  delay(1500);

  // Initialize BME280 at I2C address 0x76
  bme.begin(0x76);
}

void loop() {
  float temp = bme.readTemperature();
  float hum = bme.readHumidity();
  float press = bme.readPressure() / 100.0F;
  float alt = bme.readAltitude(1013.25);  // Sea-level pressure
  float dew = calculateDewPoint(temp, hum);
  float feels = calculateFeelsLike(temp, hum);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Ambi Temp/Hum/Press");

  lcd.setCursor(0, 1);
  lcd.print("Temp:");
  lcd.print(temp, 1);
  lcd.print("C DewPt:");
  lcd.print(dew, 1);

  lcd.setCursor(0, 2);
  lcd.print("Hum:");
  lcd.print(hum, 0);
  lcd.print("% Feels:");
  lcd.print(feels, 1);

  lcd.setCursor(0, 3);
  lcd.print("Prs:");
  lcd.print(press, 0);
  lcd.print("hPa Alt:");
  lcd.print(alt, 0);

  delay(2000);
}

