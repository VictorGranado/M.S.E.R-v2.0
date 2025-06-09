# M.S.E.R-v2.0
The upgraded version of the M.S.E.R prototype. Hence, v2.0. 

## Project Overview

The Multi-Sensor Environmental Reader (MSER) v2.0 is a significant upgrade from the original prototype. Now running on the ESP32-WROOM-DA module, the system expands its capabilities with new sensors, modularity, and a dynamic UI. MSER v2.0 measures a broader range of environmental parameters, provides radiation detection, and displays real-time data on a 20x4 I2C LCD with user-controlled menu navigation. The system is ideal for environmental research, smart home integration, and portable monitoring solutions.

---

## Features

### Multi-Sensor Integration:

* **BME280**: Measures temperature, humidity, pressure, and calculates altitude and dew point (I2C).
* **BH1750**: Measures light intensity in lux (I2C).
* **MQ135**: Measures air quality index and gas concentration in ppm (Analog input).
* **BMM150 (Previously HMC5883L)**: Measures magnetic field strength and heading direction (I2C). 
* **MLX90614**: Measures non-contact infrared (IR) object temperature (I2C). 
* **Microphone Module**: Measures sound intensity in dB and basic frequency analysis (Analog input).
* **Geiger Counter Module**: Radiation detection via an Arduino-compatible sensor operating in parallel.

### Dynamic User Interface:

* 20x4 LCD with menu system navigated via a single button using interrupts.
* Menu Categories:

  * **Ambient Conditions**: Temp, Humidity, Pressure, Altitude, Dew Point, Feels Like Temp.
  * **Air Conditions**: Air Quality Classification, PPM, Hazard Warning.
  * **Localized Data**: Magnetic Field Strength, Heading, Lux, IR Temp, Temp Diff(Local - Ambient).
  * **Sound Analysis**: dB Level, Classification, Frequency, Peak & Avg Amplitude.
  * **Data Guidelines**: Static user information about each sensor category.

### Enhanced Design:

* **Microcontroller**: ESP32-WROOM-DA with dual antenna for robust wireless capabilities.
* **Display**: 20x4 I2C LCD.
* **Power**: 3.7V Li-ion rechargeable battery with USB-C charge controller and USB power option.
* **Navigation**: Button interrupt-driven UI menu system.
* **3D Printed Case**: Redesigned to securely house all components discreetly with external access to ports and button.
* **ESP32 Reprogram Port**: Allows the user to reflash firmware via USB connection for future upgrades or custom features.

---

## System Specifications

### Hardware Components:

* **Microcontroller**: ESP32-WROOM-DA
* **Display**: 20x4 LCD via I2C
* **Sensors**: BME280, BH1750, MQ135, BMM150, MLX90614, Microphone, Geiger Counter
* **Power Source**: USB and/or 3.7V Li-ion rechargeable battery with USB-C charge circuit

### Operating Constraints:

* **Sensor Calibration**: Compass, air quality, and Geiger sensors may require calibration.
* **Display Refresh**: Menu refresh is interrupt-driven and manually controlled.

### Known Limitations:

* Analog readings (e.g., MQ135, Microphone) are sensitive to noise and require filtering.

---

## Setup and Usage

### 1. Hardware Connections

* All I2C devices (BME280, BH1750, MLX90614, LCD, QMC5883L) connect to common SDA/SCL lines.
* Analog sensors (MQ135, Microphone) connect to separate ADC-capable pins on ESP32 an directly to battery (since they have higher current draw).
* Geiger Counter module connected in parallel to main circutry to the battery.
* Button connects to GPIO with a pull-down resistor for interrupt-based navigation.

### 2. Software Requirements

* Arduino IDE with ESP32 board support.
* Libraries: `Adafruit_BME280`, `BH1750`, `Adafruit_MLX90614`, `DFRobot_BMM150`, `LiquidCrystal_I2C`, etc.

### 3. Steps to Run the System

1. Connect all hardware components to the ESP32.
2. Upload code using Arduino IDE.
3. Power the device using USB or battery.
4. Use the button to cycle through sensor data menus.

---

## Test Plan and Results

### Test Cases

* **BME280**: Confirmed functional and accurate.
* **BH1750**: Provides responsive and realistic light intensity readings.
* **MQ135**: Shows ppm and air quality changes clearly.
* **Microphone**: Displays reasonable dB levels and reacts to sound.
* **BMM150**: Reads magnetic field and heading direction, though accuracy is pending calibration.
* **MLX90614**: Sensor gets hot; not functional at present.
* **Geiger Counter**: Verified power-up and baseline detection.

### Results

* LCD UI operates reliably.
* ADC and I2C data update accurately.
* Sensor switching via button works as intended.

---

## Future Enhancements

* Add Wi-Fi/Bluetooth integration for remote dashboard.
* Data logging to SD card.
* Sensor averaging and filtering for more stable readings.

---

## Updated Parts List

| Component             | Approx. Cost  |
| --------------------- | ------------- |
| ESP32-WROOM-DA        | \$5 - \$10    |
| BME280                | \$5 - \$8     |
| BH1750                | \$2 - \$5     |
| MQ135                 | \$5 - \$10    |
| QMC5883L              | \$3 - \$7     |
| MLX90614              | \$8 - \$15    |
| Microphone Module     | \$1 - \$3     |
| Geiger Counter Module | \$45          |
| 20x4 I2C LCD          | \$5 - \$10    |
| Button and wiring     | <\$2          |
| 3.7V Li-ion Battery   | \$5 - \$8     |
| USB-C Charge Circuit  | \$2 - \$4     |
| **Total**             | \~\$88 - \$127 |

---

## Contributors

* **Victor Stafussi Granado**: Embedded Systems Design, Firmware Development, System Integration

## License

This project is licensed under the MIT License. See the LICENSE file for details.

## Acknowledgments

* Sensor datasheets: BME280, BH1750, MQ135, QMC5883L, MLX90614
* Arduino Libraries from Adafruit, DFRobot, and community contributors
* ESP32 documentation and Arduino core support
* 3D modeling tools for enclosure design

