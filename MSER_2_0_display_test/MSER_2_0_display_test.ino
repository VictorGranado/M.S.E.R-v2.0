#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// LCD I2C address and dimensions
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Button configuration
const int buttonPin = 0;  // Change to any available GPIO

volatile bool buttonPressed = false;
int menuIndex = 0;
unsigned long lastPressTime = 0;
const unsigned long debounceDelay = 200;  // milliseconds

// ========== SENSOR MOCK FUNCTIONS ==========

float getTemperature() { return 24.7; }
float getHumidity() { return 55.0; }
float getPressure() { return 1012.0; }
float getAltitude() { return 150.0; }

float calculateDewPoint(float temp, float hum) {
  double a = 17.27;
  double b = 237.7;
  double alpha = ((a * temp) / (b + temp)) + log(hum / 100.0);
  return (b * alpha) / (a - alpha);
}

float calculateFeelsLike(float temp, float hum) {
  return temp + 0.33 * hum - 4.0;
}

int getCO2PPM() { return 750; }
String getAQILevel(int ppm) {
  if (ppm < 400) return "Good";
  if (ppm < 1000) return "Moderate";
  if (ppm < 2000) return "Unhealthy";
  return "Deadly";
}

float getMagneticField() { return 45.0; }
int getHeading() { return 270; }
float getLightLevel() { return 650.0; }
float getIRTemperature() { return 32.5; }
float getAmbientTemp() { return 24.7; }

int getSoundLevel() { return 68; }
int getDominantFreq() { return 500; }
int getAvgSoundLevel() { return 65; }
int getPeakSoundLevel() { return 72; }

String getCompassDirection(int degrees) {
  String directions[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
  int index = ((degrees + 22) / 45) % 8;
  return directions[index];
}

String getLightClassification(float lux) {
  if (lux < 100) return "Dark";
  if (lux < 300) return "Dim";
  if (lux < 10000) return "Bright";
  return "Sunlight";
}

String getSoundLevelClassification(int dB) {
  if (dB < 60) return "Safe";
  if (dB < 85) return "Moderate";
  return "Loud";
}

// ========== ISR for Button ==========

void IRAM_ATTR handleButtonPress() {
  buttonPressed = true;
}

// ========== SETUP ==========

void setup() {
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();

  pinMode(buttonPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(buttonPin), handleButtonPress, FALLING);

  lcd.clear();
  lcd.setCursor(4, 0);  // Line 0: "Hello user"
  lcd.print("Hello user");
  
  lcd.setCursor(4, 1);  // Line 1: "Welcome to"
  lcd.print("Welcome to");
  
  lcd.setCursor(3, 2);  // Line 2: "M.S.E.R v2.0"
  lcd.print("M.S.E.R v2.0");
  
  delay(3000);  // Hold message for 3 seconds
  lcd.clear();

  lcd.setCursor(2, 3);  // Optional: leave blank or add message
  lcd.print("Initializing...");
  
  delay(3000);  // Hold message for 3 seconds
  lcd.clear();
  showMenu(menuIndex);
}

// ========== MENU FUNCTIONS ==========

void showMenu(int index) {
  lcd.clear();
  switch (index) {
    case 0: showMenu0(); break;
    case 1: showMenu1(); break;
    case 2: showMenu2(); break;
    case 3: showMenu3(); break;
    case 4: showMenu4(); break;
  }
}

void showMenu0() {
  float temp = getTemperature();
  float hum = getHumidity();
  float press = getPressure();
  float alt = getAltitude();
  float dew = calculateDewPoint(temp, hum);
  float feels = calculateFeelsLike(temp, hum);

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
  lcd.print("Press:");
  lcd.print(press, 0);
  lcd.print("hPa Alt:");
  lcd.print(alt, 0);
}

void showMenu1() {
  int co2 = getCO2PPM();
  String quality = getAQILevel(co2);

  lcd.setCursor(0, 0);
  lcd.print("Air Conditions");
  lcd.setCursor(0, 1);
  lcd.print("Quality: ");
  lcd.print(quality);
  lcd.setCursor(0, 2);
  lcd.print("CO2: ");
  lcd.print(co2);
  lcd.print(" PPM");
  lcd.setCursor(0, 3);
  lcd.print("Warning: None");
}

void showMenu2() {
  float mag = getMagneticField();
  int head = getHeading();
  float light = getLightLevel();
  float irTemp = getIRTemperature();
  float ambient = getAmbientTemp();
  String compass = getCompassDirection(head);
  float diffTemp = irTemp - ambient;

  lcd.setCursor(0, 0);
  lcd.print("Localized Data");
  lcd.setCursor(0, 1);
  lcd.print("Head:");
  lcd.print(head);
  lcd.print((char)223); lcd.print("(");
  lcd.print(compass); lcd.print(")");
  lcd.setCursor(0, 2);
  lcd.print("Mag:");
  lcd.print(mag, 0);
  lcd.print("uT L:");
  lcd.print(light, 0);
  lcd.print("lx");
  lcd.setCursor(0, 3);
  lcd.print("FocT:");
  lcd.print(irTemp, 1);
  lcd.print("C D:");
  lcd.print(diffTemp, 1);
  lcd.print("C");
}

void showMenu3() {
  int level = getSoundLevel();
  int freq = getDominantFreq();
  int avg = getAvgSoundLevel();
  int peak = getPeakSoundLevel();
  String levelClass = getSoundLevelClassification(level);

  lcd.setCursor(0, 0);
  lcd.print("Sound Analysis");
  lcd.setCursor(0, 1);
  lcd.print("Lvl:");
  lcd.print(level);
  lcd.print("dB(");
  lcd.print(levelClass);
  lcd.print(")");
  lcd.setCursor(0, 2);
  lcd.print("Freq:");
  lcd.print(freq);
  lcd.print("Hz");
  lcd.setCursor(0, 3);
  lcd.print("Avg:");
  lcd.print(avg);
  lcd.print(" Pk:");
  lcd.print(peak);
}

void showMenu4() {
  lcd.setCursor(0, 0);
  lcd.print("Data Guidelines");
  lcd.setCursor(0, 1);
  lcd.print("Air:<400PPM Good");
  lcd.setCursor(0, 2);
  lcd.print("Temp:20-25 RH:30-60%");
  lcd.setCursor(0, 3);
  lcd.print("Sound:<70dB Light:300lx");
}

// ========== MAIN LOOP ==========

void loop() {
  if (buttonPressed && (millis() - lastPressTime > debounceDelay)) {
    buttonPressed = false;
    lastPressTime = millis();

    menuIndex = (menuIndex + 1) % 5;
    showMenu(menuIndex);
  }

  // Optional: slow update rate or future features
  delay(50);
}

