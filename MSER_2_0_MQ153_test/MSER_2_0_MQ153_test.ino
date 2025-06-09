#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);  // 20x4 LCD
const int mq135Pin = 34;  // ADC pin

int estimatePPM(int analogVal) {
  float voltage = analogVal * (3.3 / 4095.0);
  return voltage * 1000;
}

String classifyAirQuality(int ppm) {
  if (ppm < 400) return "Good";
  if (ppm < 1000) return "Moderate";
  if (ppm < 2000) return "Unhealthy";
  return "Dangerous";
}

void setup() {
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Air Conditions...");
  delay(1500);
}

void loop() {
  int analogVal = analogRead(mq135Pin);
  int ppm = estimatePPM(analogVal);
  String quality = classifyAirQuality(ppm);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Air Conditions");
  lcd.setCursor(0, 1);
  lcd.print("CO2: ");
  lcd.print(ppm);
  lcd.print(" ppm");

  lcd.setCursor(0, 2);
  lcd.print("Quality: ");
  lcd.print(quality);

  lcd.setCursor(0, 3);
  lcd.print("Warning: None");

  delay(2000);
}
