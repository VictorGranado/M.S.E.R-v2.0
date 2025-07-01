// MSER v2.0 Final Integrated Code with BLE Export
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <DFRobot_BMM150.h>
#include <Adafruit_MLX90614.h>
#include <BH1750.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// LCD
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Button
const int buttonPin = 2;
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
const float V_REF = 3.3f;
const int ADC_MAX = 4095;
const float MIC_SENS = 0.00631f;
int samples[SAMPLES];

// BH1750 Light Sensor
BH1750 lightMeter;

// BLE
BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
unsigned long lastBLETime = 0;
const unsigned long bleInterval = 3000;  // 3 seconds

// --- Helper Functions ---
float calculateDewPoint(float temp, float hum) {
  double a = 17.27, b = 237.7;
  double alpha = ((a * temp) / (b + temp)) + log(hum / 100.0);
  return (b * alpha) / (a - alpha);
}
float calculateFeelsLike(float T, float RH) {
  float feelsLike = -8.784695 + 1.61139411*T + 2.338549*RH
                    - 0.14611605*T*RH - 0.01230809*pow(T,2)
                    - 0.01642482*pow(RH,2) + 0.00221173*pow(T,2)*RH
                    + 0.00072546*T*pow(RH,2) - 0.00000358*pow(T,2)*pow(RH,2);
  return feelsLike;
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
String classifyWarning(int ppm){
  if (ppm < 1000) return "None";
  if (ppm < 2000) return "Poor ventilation";
  return "Leave this area!";
}

float getCorrectedHeading() {
  sBmm150MagData_t mag = bmm150.getGeomagneticData();

  // Sensor mounted with Y facing up
  float heading = atan2(mag.x, mag.z) * 180.0 / PI;
  if (heading < 0) heading += 360.0;
  return heading;
}

String getCardinalDirection(float heading) {
  if (heading >= 337.5 || heading < 22.5)   return "N";
  else if (heading >= 22.5 && heading < 67.5)   return "NE";
  else if (heading >= 67.5 && heading < 112.5)  return "E";
  else if (heading >= 112.5 && heading < 157.5) return "SE";
  else if (heading >= 157.5 && heading < 202.5) return "S";
  else if (heading >= 202.5 && heading < 247.5) return "SW";
  else if (heading >= 247.5 && heading < 292.5) return "W";
  else if (heading >= 292.5 && heading < 337.5) return "NW";
  return "?"; // fallback
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

// BLE Callbacks
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  }
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

// --- ISR ---
void IRAM_ATTR handleButtonPress() {
  buttonPressed = true;
}

// --- Setup ---
void setup() {
  Serial.begin(115200);
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

  // BLE Init
  BLEDevice::init("MSER_v2");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
  pCharacteristic = pService->createCharacteristic(
                    "beb5483e-36e1-4688-b7f5-ea07361b26a8",
                    BLECharacteristic::PROPERTY_NOTIFY
                  );
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  pServer->getAdvertising()->start();

  showMenu(menuIndex);
}

// Format and send BLE data
String formatSensorData() {
  float temp = bme.readTemperature();
  float hum = bme.readHumidity();
  float press = bme.readPressure() / 100.0F;
  float alt = bme.readAltitude(1013.25);
  float dew = calculateDewPoint(temp, hum);
  float feels = calculateFeelsLike(temp, hum);
  int analogVal = analogRead(mq135Pin);
  int ppm = estimatePPM(analogVal);
  String airQual = classifyAirQuality(ppm);
  sBmm150MagData_t mag = bmm150.getGeomagneticData();
  float heading = bmm150.getCompassDegree();
  float strength = sqrt(pow(mag.x,2) + pow(mag.y,2) + pow(mag.z,2));
  float ir = mlx.readObjectTempC();
  float ambient = mlx.readAmbientTempC();
  float lux = lightMeter.readLightLevel();

  long sum = 0, sumSq = 0;
  int minVal = 4095, maxVal = 0;
  for (int i = 0; i < SAMPLES; i++) {
    samples[i] = analogRead(micPin);
    sum += samples[i];
    sumSq += long(samples[i]) * samples[i];
    minVal = min(minVal, samples[i]);
    maxVal = max(maxVal, samples[i]);
  }
  float avg = sum / float(SAMPLES);
  float var = (sumSq / float(SAMPLES)) - avg * avg;
  float rms = sqrt(max(0.0f, var));
  float vrms = rms * (V_REF / ADC_MAX);
  float dB = (vrms > 0) ? 20.0f * log10(vrms / MIC_SENS) : 0.0f;
  dB = constrain(dB, 0.0f, 120.0f);

  int crossings = 0;
  for (int i = 1; i < SAMPLES; i++) {
    if ((samples[i - 1] < avg && samples[i] >= avg) ||
        (samples[i - 1] > avg && samples[i] <= avg)) {
      crossings++;
    }
  }
  float freq = (crossings / 2.0f) / (SAMPLES * 0.001);
  if (freq < 20 || freq > 2000) freq = 0;

  String data = "# MSER v2.0 Report\n";
  data += "Temp: " + String(temp,1) + " C\n";
  data += "Humidity: " + String(hum,1) + " %\n";
  data += "Pressure: " + String(press,1) + " hPa\n";
  data += "Altitude: " + String(alt,1) + " m\n";
  data += "Feels Like: " + String(feels,1) + " C\n";
  data += "Dew Point: " + String(dew,1) + " C\n";
  data += "Air Quality: " + airQual + " (" + String(ppm) + " PPM)\n";
  data += "Heading: " + String(heading,1) + " deg\n";
  data += "Mag: " + String(strength,1) + " uT\n";
  data += "Lux: " + String(lux,1) + " lx\n";
  data += "IR Temp: " + String(ir,1) + " C\n";
  data += "dB: " + String(dB,1) + " dB\n";
  data += "Freq: " + String(freq,1) + " Hz\n";
  data += "Timestamp: " + String(millis()/1000) + " s\n\n";

  return data;
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
  lcd.setCursor(0,0); lcd.print(" Ambient Conditions");
  lcd.setCursor(0,1); lcd.print("Temp:"); lcd.print(temp,1); lcd.print(" Dew:"); lcd.print(dew,1);
  lcd.setCursor(0,2); lcd.print("Hum:"); lcd.print(hum,0); lcd.print("% Feel:"); lcd.print(feels,1);
  lcd.setCursor(0,3); lcd.print("Press:"); lcd.print(press,0); lcd.print(" Alt:"); lcd.print(alt,0);
}

void showMenu1() {
  int analogVal = analogRead(mq135Pin);
  int ppm = estimatePPM(analogVal);
  String warning = classifyWarning(ppm);
  String quality = classifyAirQuality(ppm);
  lcd.setCursor(0,0); lcd.print("   Air Conditions");
  lcd.setCursor(0,1); lcd.print("Quality: "); lcd.print(quality);
  lcd.setCursor(0,2); lcd.print("CO2: "); lcd.print(ppm); lcd.print(" PPM");
  lcd.setCursor(0,3); lcd.print("Warning: "); lcd.print(warning);
}

void showMenu2() {
  sBmm150MagData_t mag = bmm150.getGeomagneticData();
  float heading = getCorrectedHeading();
  float strength = sqrt(pow(mag.x,2) + pow(mag.y,2) + pow(mag.z,2));
  float ir = mlx.readObjectTempC();
  float ambient = mlx.readAmbientTempC();
  float lux = lightMeter.readLightLevel();

  lcd.setCursor(0,0); lcd.print("   Localized Data");
  lcd.setCursor(0,1); lcd.print("Head:"); lcd.print((int)heading); lcd.print((char)223); lcd.print("("); lcd.print(getCardinalDirection(heading)); lcd.print(")");
  lcd.setCursor(0,2); lcd.print("Mag:"); lcd.print((int)strength); lcd.print("uT Lux:"); lcd.print((int)lux); 
  lcd.setCursor(0,3); lcd.print("FTemp:"); lcd.print(ir,1); lcd.print(" TD:"); lcd.print(ir-ambient,1);
}



void showMenu3() {
  const float V_REF = 3.3f;
  const float MIC_SENS = 0.00631f;  // V/Pa (≈–44 dBV/Pa)
  const int DELAY_MS = 1;           // ~500 Hz sampling rate
  int minVal = 4095, maxVal = 0;
  long sumCounts = 0, sumSqCounts = 0;
  unsigned long startMicros = micros();

  for (int i = 0; i < SAMPLES; i++) {
    samples[i] = analogRead(micPin);
    sumCounts += samples[i];
    sumSqCounts += (long)samples[i] * samples[i];
    if (samples[i] < minVal) minVal = samples[i];
    if (samples[i] > maxVal) maxVal = samples[i];
    delay(DELAY_MS);
  }

  unsigned long endMicros = micros();
  float avg = sumCounts / float(SAMPLES);
  float meanSq = sumSqCounts / float(SAMPLES);
  float var = meanSq - avg * avg;
  float rmsCounts = sqrt(max(0.0f, var));
  float vrms = rmsCounts * (V_REF / 4095.0);
  float dB = (vrms > 0) ? 20.0f * log10(vrms / MIC_SENS) : 0.0f;
  dB = constrain(dB, 0.0f, 120.0f);

  // Frequency estimation via zero-crossing
  int crossings = 0;
  for (int i = 1; i < SAMPLES; i++) {
    if ((samples[i - 1] < avg && samples[i] >= avg) ||
        (samples[i - 1] > avg && samples[i] <= avg)) {
      crossings++;
    }
  }
  float durationSec = (endMicros - startMicros) / 1e6;
  float freq = (crossings / 2.0f) / durationSec;
  if (freq < 20 || freq > 2000) freq = 0;

  lcd.setCursor(0,0); lcd.print("   Sound Analysis");
  lcd.setCursor(0,1); lcd.print("Lvl:");
  lcd.print((int)dB); lcd.print("dB("); lcd.print(classifyDbLevel(dB)); lcd.print(")");

  lcd.setCursor(0,2); lcd.print("Freq:");
  lcd.print((int)freq); lcd.print("Hz");

  lcd.setCursor(0,3); lcd.print("Avg:");
  lcd.print((int)avg); lcd.print(" Pk:");
  lcd.print(maxVal);
}


void showMenu4() {
  lcd.setCursor(0,0); lcd.print("   Reference Data");
  lcd.setCursor(0,1); lcd.print("Air:<400PPM Good");
  lcd.setCursor(0,2); lcd.print("Temp:20-25 RH:30-60%");
  lcd.setCursor(0,3); lcd.print("Sound:<70dB Lux:300");
}

// --- Main Loop ---
void loop() {
  if (buttonPressed && (millis() - lastPressTime > debounceDelay)) {
    buttonPressed = false;
    lastPressTime = millis();
    menuIndex = (menuIndex + 1) % 5;
  }
  showMenu(menuIndex);

  if (deviceConnected && millis() - lastBLETime >= bleInterval) {
    String report = formatSensorData();
    pCharacteristic->setValue(report.c_str());
    pCharacteristic->notify();
    lastBLETime = millis();
  }
  delay(1000);
}

