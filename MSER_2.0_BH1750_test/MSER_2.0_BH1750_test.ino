#include <Wire.h>
#include <BH1750.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);
BH1750 lightMeter(0x23);  // Explicit I2C address

String classifyLightLevel(float lux) {
  if (lux < 50) return "Dark";
  if (lux < 300) return "Dim";
  if (lux < 10000) return "Bright";
  return "Sunlight";
}

void setup() {
  lcd.init();
  lcd.backlight();

  lcd.setCursor(0, 0);
  lcd.print("Testing BH1750...");
  delay(1500);

  Wire.begin();  // SDA = GPIO21, SCL = GPIO22
  lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
}

void loop() {
  float lux = lightMeter.readLightLevel();
  String lightClass = classifyLightLevel(lux);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Localized Data");

  lcd.setCursor(0, 1);
  lcd.print("Light Intensity:");

  lcd.setCursor(0, 2);
  lcd.print(lux, 0);
  lcd.print(" lx");

  lcd.setCursor(0, 3);
  lcd.print("Level: ");
  lcd.print(lightClass);

  delay(2000);
}
