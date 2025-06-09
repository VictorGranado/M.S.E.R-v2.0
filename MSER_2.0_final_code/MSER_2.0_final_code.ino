// MSER v2.0 Final Integrated Code
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <DFRobot_BMM150.h>
#include <Adafruit_MLX90614.h>
#include <BH1750.h>

// LCD
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Button
const int buttonPin = 0;
volatile bool buttonPressed = false;
int menuIndex = 0;
unsigned long lastPressTime = 0;
const unsigned long debounceDelay = 200;

// BME280
Adafruit_BME280 bme;

// MQ135
const int mq135Pin = 34;

// BMM150
DFRobot_BMM150_I2C bmm150(&Wire, I2C_ADDRESS_4);

// MLX90614 IR Sensor
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

// Microphone
const int micPin = 35;
#define SAMPLES 200
int samples[SAMPLES];

// BH1750 Light Sensor
BH1750 lightMeter;

// --- Helper Functions ---
float calculateDewPoint(float temp, float hum) {
  double a = 17.27, b = 237.7;
  double alpha = ((a * temp) / (b + temp)) + log(hum / 100.0);
  return (b * alpha) / (a - alpha);
}
float calculateFeelsLike(float temp, float hum) {
  return temp + 0.33 * hum - 4.0;
}

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

String classifyDbLevel(float dB) {
  if (dB < 60) return "Safe";
  if (dB < 85) return "Moderate";
  return "Loud";
}

String classifyLightLevel(float lux) {
  if (lux < 100) return "Dark";
  if (lux < 300) return "Dim";
  if (lux < 10000) return "Bright";
  return "Sunlight";
}

// --- ISR ---
void IRAM_ATTR handleButtonPress() {
  buttonPressed = true;
}

// --- Setup ---
void setup() {
  lcd.init(); lcd.backlight();
  pinMode(buttonPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(buttonPin), handleButtonPress, FALLING);

  lcd.setCursor(4, 0); lcd.print("Hello user");
  lcd.setCursor(4, 1); lcd.print("Welcome to");
  lcd.setCursor(3, 2); lcd.print("M.S.E.R v2.0");
  delay(3000); lcd.clear();
  lcd.setCursor(2, 3); lcd.print("Initializing...");
  delay(3000); lcd.clear();

  bme.begin(0x76);
  bmm150.begin();
  bmm150.setOperationMode(BMM150_POWERMODE_NORMAL);
  bmm150.setPresetMode(BMM150_PRESETMODE_HIGHACCURACY);
  bmm150.setRate(BMM150_DATA_RATE_10HZ);
  bmm150.setMeasurementXYZ();
  mlx.begin();
  lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);

  showMenu(menuIndex);
}

// --- Menu Display ---
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
  float temp = bme.readTemperature();
  float hum = bme.readHumidity();
  float press = bme.readPressure() / 100.0F;
  float alt = bme.readAltitude(1013.25);
  float dew = calculateDewPoint(temp, hum);
  float feels = calculateFeelsLike(temp, hum);
  lcd.setCursor(0,0); lcd.print("Ambi Temp/Hum/Press");
  lcd.setCursor(0,1); lcd.print("Temp:"); lcd.print(temp,1); lcd.print(" Dew:"); lcd.print(dew,1);
  lcd.setCursor(0,2); lcd.print("Hum:"); lcd.print(hum,0); lcd.print("% Feel:"); lcd.print(feels,1);
  lcd.setCursor(0,3); lcd.print("Press:"); lcd.print(press,0); lcd.print(" Alt:"); lcd.print(alt,0);
}

void showMenu1() {
  int analogVal = analogRead(mq135Pin);
  int ppm = estimatePPM(analogVal);
  String quality = classifyAirQuality(ppm);
  lcd.setCursor(0,0); lcd.print("Air Conditions");
  lcd.setCursor(0,1); lcd.print("Quality: "); lcd.print(quality);
  lcd.setCursor(0,2); lcd.print("CO2: "); lcd.print(ppm); lcd.print(" PPM");
  lcd.setCursor(0,3); lcd.print("Warning: None");
}

void showMenu2() {
  sBmm150MagData_t mag = bmm150.getGeomagneticData();
  float heading = bmm150.getCompassDegree();
  float strength = sqrt(pow(mag.x,2) + pow(mag.y,2) + pow(mag.z,2));
  float ir = mlx.readObjectTempC();
  float ambient = mlx.readAmbientTempC();
  float lux = lightMeter.readLightLevel();
  lcd.setCursor(0,0); lcd.print("Localized Data");
  lcd.setCursor(0,1); lcd.print("Head:"); lcd.print((int)heading); lcd.print((char)223); lcd.print("("); lcd.print(getCardinalDirection(heading)); lcd.print(")");
  lcd.setCursor(0,2); lcd.print("Mag:"); lcd.print((int)strength); lcd.print("uT L:"); lcd.print((int)lux); lcd.print("lx");
  lcd.setCursor(0,3); lcd.print("FocT:"); lcd.print(ir,1); lcd.print(" D:"); lcd.print(ir-ambient,1);
}

void showMenu3() {
  unsigned long startMicros = micros();
  for (int i = 0; i < SAMPLES; i++) {
    samples[i] = analogRead(micPin);
    delayMicroseconds(100);
  }
  unsigned long endMicros = micros();

  int peak = 0; long sum = 0;
  for (int i = 0; i < SAMPLES; i++) {
    if (samples[i] > peak) peak = samples[i];
    sum += samples[i];
  }
  int average = sum / SAMPLES;
  float voltage = (peak - average) * 3.3 / 4095.0;
  float dB = voltage > 0 ? 20 * log10(voltage / 0.00631) : 0;
  dB = constrain(dB, 0, 120);

  int crossings = 0;
  for (int i = 1; i < SAMPLES; i++) {
    if ((samples[i - 1] < average && samples[i] >= average) || (samples[i - 1] > average && samples[i] <= average)) {
      crossings++;
    }
  }
  float durationSec = (endMicros - startMicros) / 1e6;
  float freq = (crossings / 2.0) / durationSec;

  lcd.setCursor(0,0); lcd.print("Sound Analysis");
  lcd.setCursor(0,1); lcd.print("Lvl:"); lcd.print((int)dB); lcd.print("dB("); lcd.print(classifyDbLevel(dB)); lcd.print(")");
  lcd.setCursor(0,2); lcd.print("Freq:"); lcd.print((int)freq); lcd.print("Hz");
  lcd.setCursor(0,3); lcd.print("Avg:"); lcd.print(average); lcd.print(" Pk:"); lcd.print(peak);
}

void showMenu4() {
  lcd.setCursor(0,0); lcd.print("Data Guidelines");
  lcd.setCursor(0,1); lcd.print("Air:<400PPM Good");
  lcd.setCursor(0,2); lcd.print("Temp:20-25 RH:30-60%");
  lcd.setCursor(0,3); lcd.print("Sound:<70dB Light:300lx");
}

// --- Main Loop ---
void loop() {
  if (buttonPressed && (millis() - lastPressTime > debounceDelay)) {
    buttonPressed = false;
    lastPressTime = millis();
    menuIndex = (menuIndex + 1) % 5;
    showMenu(menuIndex);
  }
  delay(50);
}
