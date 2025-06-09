#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);
const int micPin = 35;  // ADC pin

#define SAMPLES 200

int samples[SAMPLES];

String classifyDbLevel(float dB) {
  if (dB < 60) return "Safe";
  if (dB < 85) return "Moderate";
  return "Loud";
}

void setup() {
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Mic Analyzer Ready");
  delay(1500);
}

void loop() {
  // Sampling
  unsigned long startMicros = micros();
  for (int i = 0; i < SAMPLES; i++) {
    samples[i] = analogRead(micPin);
    delayMicroseconds(100);  // 10 kHz sampling
  }
  unsigned long endMicros = micros();

  // Peak & average
  int peak = 0;
  long sum = 0;
  for (int i = 0; i < SAMPLES; i++) {
    if (samples[i] > peak) peak = samples[i];
    sum += samples[i];
  }
  int average = sum / SAMPLES;

  // Estimate dB from peak voltage
  float voltage = (peak - average) * 3.3 / 4095.0;  // AC component
  float dB = voltage > 0 ? 20 * log10(voltage / 0.00631) : 0; // Ref ~0.00631V
  dB = constrain(dB, 0, 120);
  String levelClass = classifyDbLevel(dB);

  // Frequency estimate from zero crossings
  int crossings = 0;
  for (int i = 1; i < SAMPLES; i++) {
    if ((samples[i - 1] < average && samples[i] >= average) ||
        (samples[i - 1] > average && samples[i] <= average)) {
      crossings++;
    }
  }
  float durationSec = (endMicros - startMicros) / 1e6;
  float freq = (crossings / 2.0) / durationSec;

  // Display
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Lvl: ");
  lcd.print((int)dB);
  lcd.print(" dB (");
  lcd.print(levelClass);
  lcd.print(")");

  lcd.setCursor(0, 1);
  lcd.print("Freq: ");
  lcd.print((int)freq);
  lcd.print(" Hz");

  lcd.setCursor(0, 2);
  lcd.print("Peak: ");
  lcd.print(peak);

  lcd.setCursor(0, 3);
  lcd.print("Avg: ");
  lcd.print(average);

  delay(1000);
}

