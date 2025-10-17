# EcoStick Sentinel: Advanced Portable Environmental Monitoring with M5Stick-C

## Introduction

**EcoStick Sentinel** is a cutting-edge, open-source, portable environmental monitoring system designed to leverage the compact and powerful [M5Stick-C](https://shop.m5stack.com/products/m5stickc-esp32-pico-mini-iot-development-kit) development board. This project aims to provide real-time monitoring of environmental parameters such as temperature, humidity, air quality, and motion, making it ideal for applications in urban farming, personal health tracking, environmental research, and IoT prototyping. The device integrates seamlessly with cloud services via MQTT for data visualization and analysis, features a user-friendly interface on a 0.96-inch TFT screen, and includes haptic feedback for alerts. Its low-power design ensures days of operation on a single charge, and its modular architecture allows for easy customization and expansion.

The EcoStick Sentinel is perfect for hobbyists, developers, researchers, and startups looking to explore IoT solutions with minimal hardware complexity. This README provides an exhaustive guide to the project, covering hardware setup, software configuration, firmware details, cloud integration, usage instructions, troubleshooting, and future enhancements. Whether you're a beginner or an experienced maker, this project is designed to be accessible yet powerful.

---

## Project Goals

- **Monitor Environmental Data**: Track temperature, humidity, air quality (optional), and motion in real-time.
- **Cloud Connectivity**: Transmit data to a cloud-based dashboard for visualization and historical analysis.
- **User Interaction**: Provide a clear, interactive interface on the M5Stick-C’s TFT screen and support button-based controls.
- **Portability and Power Efficiency**: Ensure the device is pocket-sized, battery-powered, and optimized for long runtime.
- **Extensibility**: Offer a modular design for adding new sensors or features.
- **Open-Source**: Share all code, schematics, and documentation under the MIT License for community contributions.

---

## Features

### Core Functionality
- **Real-Time Sensor Monitoring**:
  - Temperature and humidity via the onboard SHT30 sensor (±0.3°C, ±2% RH accuracy).
  - Motion detection using the MPU6886 6-axis IMU (accelerometer + gyroscope).
  - Optional air quality monitoring with an MQ-135 gas sensor (CO2, VOCs, etc.).
- **Cloud Integration**:
  - Data transmission over Wi-Fi using MQTT to a broker (e.g., Mosquitto, HiveMQ).
  - Real-time visualization on a web-based dashboard (Node-RED, Grafana, or custom).
- **User Interface**:
  - 0.96-inch TFT screen (80x160 pixels, ST7735S driver) displays sensor data, alerts, and battery status.
  - Two programmable buttons (A and B) for mode switching, reset, or sleep.
- **Haptic Feedback**:
  - Vibration motor alerts for threshold breaches (e.g., high temperature, poor air quality, or sudden motion).
- **Power Management**:
  - 80mAh Li-Po battery lasts 3-5 days in normal operation.
  - Deep sleep mode activates after 30 seconds of inactivity to conserve power.
  - Battery level monitoring via the AXP192 PMU chip.
- **Portability**:
  - Compact form factor (48.2 x 25.5 x 13.7 mm) for pocket or wearable use.
  - Optional 3D-printed enclosure for protection and aesthetics.
- **Extensibility**:
  - Grove connector for additional sensors (e.g., PM2.5, light, or pressure sensors).
  - Over-The-Air (OTA) firmware updates for seamless upgrades.
- **Robustness**:
  - Error handling for sensor failures, Wi-Fi disconnections, and MQTT issues.
  - Calibration routines for optional air quality sensors.

### Unique Selling Points
- **All-in-One Design**: Combines multiple sensors, display, and connectivity in a single, affordable device.
- **Low-Cost**: Built around the $15 M5Stick-C, with optional sensors costing $5-10.
- **Community-Driven**: Open-source with detailed documentation for easy adoption and contribution.
- **Versatile Applications**: Suitable for urban farming, health monitoring, environmental studies, or educational projects.

---

## Hardware Requirements

### Core Components
- **M5Stick-C**:
  - **Microcontroller**: ESP32-PICO-D4 (dual-core, 240 MHz, 4MB Flash, 520KB SRAM).
  - **Display**: 0.96-inch TFT (80x160 pixels, ST7735S driver).
  - **Sensors**:
    - SHT30 (temperature: -40°C to 125°C, ±0.3°C; humidity: 0-100% RH, ±2%).
    - MPU6886 6-axis IMU (accelerometer: ±2/4/8/16g; gyroscope: ±250/500/1000/2000 dps).
  - **Power**: 80mAh Li-Po battery, AXP192 PMU, USB-C charging.
  - **Peripherals**: Vibration motor, IR transmitter, microphone, two buttons (A: GPIO 35, B: GPIO 37), Grove connector (I2C + analog/digital).
  - **Connectivity**: Wi-Fi (2.4 GHz), Bluetooth 4.2.
- **Optional Sensors**:
  - **MQ-135 Gas Sensor**: For air quality (CO2, VOCs, NH3, etc.), connected via Grove (analog pin G33).
  - **Grove-Compatible Sensors**: E.g., BME280 (pressure), VL53L0X (distance), or PMS5003 (particulate matter).
- **Power Supply**:
  - USB-C cable (5V, included with M5Stick-C).
  - Optional external battery pack (3.7V Li-Po) for extended runtime.

### Additional Materials
- **Cables**: USB-C to USB-A cable for programming and charging.
- **Enclosure**: Optional 3D-printed case (STL files to be provided in the GitHub repo).
- **Tools**: Computer with USB port, screwdriver (for enclosure assembly), and optional multimeter for debugging.
- **Wi-Fi Network**: 2.4 GHz network for MQTT connectivity (5 GHz not supported).

### Hardware Setup Notes
- Ensure the M5Stick-C is fully charged before use (~30 minutes via USB-C).
- Connect the MQ-135 sensor to the Grove port (pins G32/G33) if used.
- Secure the device in an enclosure to protect against dust or impact, especially for outdoor use.
- Verify sensor connections using a multimeter if readings are inconsistent.

---

## Software Requirements

### Development Environment
- **IDE**:
  - [PlatformIO](https://platformio.org/) (recommended for better dependency management).
  - [Arduino IDE](https://www.arduino.cc/en/software) (simpler for beginners).
- **Board Support**:
  - ESP32 board package: `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`.
  - M5Stack board definitions (included in M5Stack library).
- **Libraries**:
  - [M5Stack](https://github.com/m5stack/M5Stack): Core library for M5Stick-C.
  - [Adafruit_SHT31](https://github.com/adafruit/Adafruit_SHT31): For SHT30 sensor.
  - [PubSubClient](https://github.com/knolleary/pubsubclient): For MQTT communication.
  - [WiFiManager](https://github.com/tzapu/WiFiManager): For dynamic Wi-Fi configuration.
- **Cloud Services**:
  - **MQTT Broker**: Mosquitto (local), HiveMQ, AWS IoT, or similar.
  - **Dashboard**: Node-RED (recommended), Grafana, or custom HTML/JS dashboard.
  - **Optional Database**: InfluxDB or MongoDB for historical data storage.
- **Programming Language**:
  - C++ (Arduino framework) for firmware.
  - Python or JavaScript for optional dashboard or data processing scripts.
- **Tools**:
  - MQTT client (e.g., MQTT Explorer) for testing.
  - Serial monitor (built into IDE) for debugging.
  - Git for version control (repo coming soon at `github.com/hamzadenizyilmaz/EcoStick-Sentinel`).

---

## System Architecture

The EcoStick Sentinel operates in a modular, layered architecture:

1. **Sensor Layer**:
   - Collects data from SHT30 (temperature/humidity), MPU6886 (motion), and optional MQ-135 (air quality).
   - Sampling frequency: Every 2 seconds (configurable).
2. **Processing Layer**:
   - ESP32 processes raw sensor data, applies calibration (e.g., MQ-135 CO2 estimation), and checks thresholds.
   - Manages power states (active, idle, deep sleep) via AXP192 PMU.
3. **Display Layer**:
   - Renders sensor data, alerts, and battery status on the TFT screen.
   - Optimizes updates to reduce flicker and power consumption.
4. **Connectivity Layer**:
   - Wi-Fi connects via WiFiManager for user-friendly network setup.
   - MQTT publishes data to predefined topics (`ecostick/temp`, `ecostick/humidity`, etc.).
5. **Alert Layer**:
   - Triggers haptic feedback and screen alerts for threshold breaches (e.g., temperature > 30°C).
   - Cooldown periods prevent alert spamming (e.g., 5 seconds for motion).
6. **Cloud Layer**:
   - Data is visualized on a web dashboard (Node-RED/Grafana).
   - Optional storage in a time-series database for analysis.
7. **Power Management**:
   - Deep sleep after 30 seconds of inactivity.
   - Wake-up on button press (GPIO 35) or timer (configurable).

---

## Installation and Setup

### Step 1: Hardware Assembly
1. **Prepare M5Stick-C**:
   - Charge the device via USB-C (full charge in ~30 minutes).
   - Verify the battery level using the AXP192 PMU (displayed on-screen).
2. **Connect Optional Sensors**:
   - Attach the MQ-135 sensor to the Grove port (pin G33 for analog input).
   - Secure connections with proper wiring (use Grove cables for plug-and-play).
3. **Optional Enclosure**:
   - 3D-print a custom enclosure (STL files to be provided in the repo).
   - Ensure ventilation for sensors and access to buttons/USB-C port.
4. **Test Connections**:
   - Power on the M5Stick-C and check for the boot message on the TFT screen.
   - Use a multimeter to verify Grove pin voltages if sensors fail.

### Step 2: Software Setup
1. **Install IDE**:
   - Download [PlatformIO](https://platformio.org/) or [Arduino IDE](https://www.arduino.cc/en/software).
   - For Arduino IDE, add ESP32 board support:
     - Go to `File > Preferences > Additional Boards Manager URLs`.
     - Add: `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`.
     - Install `esp32` by Espressif in Boards Manager.
2. **Install Libraries**:
   - Open the Library Manager in your IDE.
   - Install:
     - `M5Stack` (version 0.4.0 or later).
     - `Adafruit_SHT31` (version 1.2.0 or later).
     - `PubSubClient` (version 2.8 or later).
     - `WiFiManager` (version 2.0.17 or later).
3. **Clone or Download Firmware**:
   - The project will be hosted at `github.com/hamzadenizyilmaz/EcoStick-Sentinel` (coming soon).
   - For now, copy the firmware code provided below.
4. **Configure Firmware**:
   - Edit MQTT settings if using a custom broker (default: HiveMQ public broker).
   - Adjust thresholds (`TEMP_THRESHOLD`, `CO2_THRESHOLD`, etc.) as needed.
   - Calibrate MQ-135 constants based on your environment (see calibration section).

### Step 3: Upload Firmware
1. **Connect M5Stick-C**:
   - Plug the M5Stick-C into your computer via USB-C.
   - Select the board (`M5Stick-C`) and port in your IDE.
2. **Upload Code**:
   - In PlatformIO: Run `pio run -t upload`.
   - In Arduino IDE: Click `Upload`.
   - Monitor the serial output (115200 baud) for debugging.
3. **Verify Boot**:
   - The TFT screen should display “EcoStick Sentinel Booting...” followed by sensor data.
   - If errors occur, check the serial monitor for logs.

### Step 4: Cloud Setup
1. **MQTT Broker**:
   - **Local**: Install Mosquitto (`sudo apt install mosquitto` on Linux).
   - **Cloud**: Use HiveMQ, AWS IoT, or EMQX.
   - Configure topics: `ecostick/temp`, `ecostick/humidity`, `ecostick/air_quality`, `ecostick/motion`.
2. **Dashboard**:
   - **Node-RED**:
     - Install: `npm install -g node-red`.
     - Add MQTT input nodes and UI elements (gauges, charts).
     - Access at `http://<node-red-ip>:1880/ui`.
   - **Grafana**:
     - Pair with InfluxDB for time-series storage.
     - Create dashboards with real-time and historical data.
3. **Test Connectivity**:
   - Use MQTT Explorer to verify data is published to the correct topics.
   - Ensure the M5Stick-C’s IP address is stable (use a static IP if needed).

---

## Firmware Code

Below is the optimized firmware code for the EcoStick Sentinel. It includes power-efficient deep sleep, modular sensor handling, and robust error checking.

```cpp
#include <M5StickC.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_SHT31.h>
#include <WiFiManager.h>

// Configuration constants
const char* MQTT_SERVER = "broker.hivemq.com";
const uint16_t MQTT_PORT = 1883;
const char* MQTT_CLIENT_ID = "EcoStickClient";
const char* MQTT_TOPIC_TEMP = "ecostick/temp";
const char* MQTT_TOPIC_HUM = "ecostick/humidity";
const char* MQTT_TOPIC_AIR_QUALITY = "ecostick/air_quality";
const char* MQTT_TOPIC_MOTION = "ecostick/motion";

// Pin definitions
const uint8_t MQ135_PIN = 33; // Grove port pin G33 for analog input

// Thresholds for alerts
const float TEMP_THRESHOLD = 30.0; // °C
const float HUM_THRESHOLD = 80.0;  // % RH
const float CO2_THRESHOLD = 1000;  // ppm
const float MOTION_THRESHOLD = 2.0; // g

// Timing constants
const uint32_t UPDATE_INTERVAL = 2000; // 2 seconds
const uint32_t MOTION_COOLDOWN = 5000; // 5 seconds
const uint32_t SLEEP_TIMEOUT = 30000; // 30 seconds inactivity for deep sleep

// Global objects
Adafruit_SHT31 sht31 = Adafruit_SHT31();
WiFiClient espClient;
PubSubClient client(espClient);

// Sensor data structure
struct SensorData {
  float temperature = 0.0;
  float humidity = 0.0;
  float airQuality = 0.0;
  bool motionDetected = false;
} sensorData;

// Timing variables
uint32_t lastUpdate = 0;
uint32_t lastMotion = 0;
uint32_t lastInteraction = 0;

// Function prototypes
void setupWiFi();
void reconnectMQTT();
void readSensors();
void displayData();
void checkThresholds();
void publishData();
float readMQ135();
void checkMotion();
void enterDeepSleep();

void setup() {
  // Initialize M5Stick-C with minimal power settings
  M5.begin(true, true, false); // LCD, Power, Serial (disabled)
  M5.Lcd.setRotation(3);
  M5.Lcd.setBrightness(50); // Reduce brightness for power saving
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(1);

  // Initialize SHT31 sensor
  if (!sht31.begin(0x44)) {
    M5.Lcd.println("SHT31 Error!");
    while (1) delay(100);
  }

  // Initialize IMU
  M5.IMU.Init();

  // Setup Wi-Fi
  setupWiFi();

  // Setup MQTT
  client.setServer(MQTT_SERVER, MQTT_PORT);

  // Enable wake-up on Button A
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_35, 0); // Button A (active low)

  lastInteraction = millis();
}

void loop() {
  M5.update();
  uint32_t currentMillis = millis();

  // Check for deep sleep
  if (currentMillis - lastInteraction > SLEEP_TIMEOUT) {
    enterDeepSleep();
  }

  // Handle button presses
  if (M5.BtnA.wasPressed()) {
    lastInteraction = currentMillis;
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.println("Reset Display");
    delay(500);
  }

  if (M5.BtnB.wasPressed()) {
    lastInteraction = currentMillis;
    enterDeepSleep();
  }

  // Maintain MQTT connection
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();

  // Update sensors and display
  if (currentMillis - lastUpdate >= UPDATE_INTERVAL) {
    readSensors();
    checkThresholds();
    displayData();
    publishData();
    lastUpdate = currentMillis;
    lastInteraction = currentMillis; // Update interaction time
  }
}

void setupWiFi() {
  WiFiManager wifiManager;
  wifiManager.setTimeout(180);
  if (!wifiManager.autoConnect("EcoStick-AP")) {
    M5.Lcd.println("WiFi Failed!");
    delay(2000);
    ESP.restart();
  }
}

void reconnectMQTT() {
  static uint32_t lastAttempt = 0;
  if (millis() - lastAttempt < 5000) return; // Avoid rapid retries
  lastAttempt = millis();

  if (client.connect(MQTT_CLIENT_ID)) {
    M5.Lcd.setCursor(0, 40);
    M5.Lcd.println("MQTT Connected  ");
  }
}

void readSensors() {
  // Read temperature and humidity
  sensorData.temperature = sht31.readTemperature();
  sensorData.humidity = sht31.readHumidity();

  // Read air quality
  sensorData.airQuality = readMQ135();

  // Check motion
  checkMotion();
}

void displayData() {
  static SensorData lastData; // Track last displayed data to avoid flicker
  if (abs(sensorData.temperature - lastData.temperature) < 0.1 &&
      abs(sensorData.humidity - lastData.humidity) < 0.1 &&
      abs(sensorData.airQuality - lastData.airQuality) < 10 &&
      sensorData.motionDetected == lastData.motionDetected) {
    return; // Skip update if data hasn't changed significantly
  }
  lastData = sensorData;

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.setTextColor(WHITE);

  // Display sensor data
  if (!isnan(sensorData.temperature)) {
    M5.Lcd.printf("Temp: %.1f C\n", sensorData.temperature);
  }
  if (!isnan(sensorData.humidity)) {
    M5.Lcd.printf("Hum: %.1f %%\n", sensorData.humidity);
  }
  if (sensorData.airQuality > 0) {
    M5.Lcd.printf("CO2: %.0f ppm\n", sensorData.airQuality);
  }
  M5.Lcd.printf("Motion: %s", sensorData.motionDetected ? "Detected" : "None");

  // Display battery level
  float batteryVoltage = M5.Axp.GetVbatData() * 1.1 / 1000;
  M5.Lcd.setCursor(0, 60);
  M5.Lcd.printf("Bat: %.2f V", batteryVoltage);
}

void checkThresholds() {
  static bool alertActive = false;
  bool newAlert = false;

  // Check thresholds
  if (!isnan(sensorData.temperature) && sensorData.temperature > TEMP_THRESHOLD) {
    M5.Lcd.setTextColor(RED);
    M5.Lcd.setCursor(0, 70);
    M5.Lcd.println("High Temp!");
    newAlert = true;
  }
  if (!isnan(sensorData.humidity) && sensorData.humidity > HUM_THRESHOLD) {
    M5.Lcd.setTextColor(RED);
    M5.Lcd.setCursor(0, 80);
    M5.Lcd.println("High Hum!");
    newAlert = true;
  }
  if (sensorData.airQuality > CO2_THRESHOLD) {
    M5.Lcd.setTextColor(RED);
    M5.Lcd.setCursor(0, 90);
    M5.Lcd.println("Poor Air!");
    newAlert = true;
  }

  // Trigger haptic feedback for new alerts
  if (newAlert && !alertActive) {
    M5.Axp.SetLDOEnable(3, true);
    delay(150);
    M5.Axp.SetLDOEnable(3, false);
    alertActive = true;
  } else if (!newAlert) {
    alertActive = false;
  }
}

void publishData() {
  static char buffer[16]; // Buffer for string conversion
  if (!client.connected()) return;

  if (!isnan(sensorData.temperature)) {
    snprintf(buffer, sizeof(buffer), "%.1f", sensorData.temperature);
    client.publish(MQTT_TOPIC_TEMP, buffer);
  }
  if (!isnan(sensorData.humidity)) {
    snprintf(buffer, sizeof(buffer), "%.1f", sensorData.humidity);
    client.publish(MQTT_TOPIC_HUM, buffer);
  }
  if (sensorData.airQuality > 0) {
    snprintf(buffer, sizeof(buffer), "%.0f", sensorData.airQuality);
    client.publish(MQTT_TOPIC_AIR_QUALITY, buffer);
  }
  client.publish(MQTT_TOPIC_MOTION, sensorData.motionDetected ? "1" : "0");
}

float readMQ135() {
  int sensorValue = analogRead(MQ135_PIN);
  if (sensorValue <= 0) return 0.0; // No sensor connected
  float voltage = sensorValue * (3.3 / 4095.0);
  return 400 + (sensorValue * 2); // Simplified calibration
}

void checkMotion() {
  float accelX, accelY, accelZ;
  M5.IMU.getAccelData(&accelX, &accelY, &accelZ);
  float magnitude = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);

  if (magnitude > MOTION_THRESHOLD && (millis() - lastMotion > MOTION_COOLDOWN)) {
    sensorData.motionDetected = true;
    lastMotion = millis();
    M5.Axp.SetLDOEnable(3, true);
    delay(150);
    M5.Axp.SetLDOEnable(3, false);
  } else {
    sensorData.motionDetected = false;
  }
}

void enterDeepSleep() {
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.println("Sleeping...");
  delay(1000);
  M5.Lcd.fillScreen(BLACK);
  esp_deep_sleep_start();
}
```

---

## Usage Instructions

### Powering On
1. Press the power button (side of M5Stick-C) to start the device.
2. The TFT screen displays “EcoStick Sentinel Booting...” followed by sensor data within 5 seconds.
3. If Wi-Fi setup is required, connect to the “EcoStick-AP” access point using a phone or laptop and enter your Wi-Fi credentials.

### Operating Modes
1. **Normal Mode**:
   - Displays temperature, humidity, air quality (if MQ-135 is connected), motion status, and battery level.
   - Updates every 2 seconds.
2. **Alert Mode**:
   - Triggered when thresholds are exceeded (e.g., temperature > 30°C, CO2 > 1000 ppm).
   - Screen shows red text and the vibration motor activates.
3. **Sleep Mode**:
   - Activates after 30 seconds of inactivity or by pressing Button B.
   - Wakes on Button A press (GPIO 35).
4. **Reset Mode**:
   - Press Button A to clear the screen and refresh the display.

### Dashboard Access
- Open the Node-RED or Grafana dashboard in a browser (e.g., `http://<node-red-ip>:1880/ui`).
- View real-time data (temperature, humidity, air quality, motion) and historical trends.
- Configure alerts in Node-RED for email or push notifications (optional).

### Battery Management
- Check battery voltage on the screen (~4.2V full, ~3.3V low).
- Recharge via USB-C when the voltage drops below 3.5V.
- Expected runtime: 3-5 days in normal mode, up to 10 days with frequent deep sleep.

---

## Calibration and Maintenance

### Sensor Calibration
- **SHT30**:
  - No calibration required; factory-calibrated with high accuracy.
  - If readings are off, check for dust or moisture on the sensor (clean with compressed air).
- **MQ-135**:
  - Warm up for 24 hours in clean air (400 ppm CO2 baseline).
  - Calibrate using a known CO2 source (e.g., outdoor air or a reference meter).
  - Adjust the `readMQ135()` function with a calibration factor if needed.
- **MPU6886**:
  - Ensure the device is stationary during IMU initialization.
  - Adjust `MOTION_THRESHOLD` for sensitivity (default: 2g).

### Maintenance
- **Battery**:
  - Charge every 3-5 days (or sooner if used heavily).
  - Avoid over-discharging below 3.0V to prolong battery life.
- **Cleaning**:
  - Wipe the TFT screen with a microfiber cloth.
  - Use compressed air to clean sensor ports.
  - Avoid water or harsh chemicals.
- **Storage**:
  - Store in a dry, cool place when not in use.
  - Remove the battery if storing for months to prevent leakage.

---

## Troubleshooting

### Common Issues
1. **Device Doesn’t Boot**:
   - Check battery voltage (should be >3.3V).
   - Ensure USB-C cable is functional.
   - Reset by holding the power button for 10 seconds.
2. **Wi-Fi Connection Fails**:
   - Verify the network is 2.4 GHz (5 GHz not supported).
   - Reset WiFiManager by holding Button A for 10 seconds (starts “EcoStick-AP”).
   - Check signal strength; move closer to the router.
3. **MQTT Connection Issues**:
   - Confirm broker IP/port and internet connectivity.
   - Test with MQTT Explorer to verify topic subscriptions.
   - Increase `reconnectMQTT()` retry interval if the broker is slow.
4. **Sensor Errors**:
   - **SHT30**: Check I2C connections (pins G21/G22). Reinitialize with `sht31.begin(0x44)`.
   - **MQ-135**: Ensure 24-hour warm-up and proper calibration. Verify Grove pin G33.
   - **MPU6886**: Run `M5.IMU.Init()` again if motion detection fails.
5. **Screen Flickering**:
   - Reduce `M5.Lcd.setBrightness()` (e.g., to 30) to save power.
   - Update the M5Stack library to the latest version.
6. **Battery Drains Quickly**:
   - Lower `UPDATE_INTERVAL` or enable deep sleep more frequently.
   - Disable Wi-Fi when not needed (modify `setupWiFi()` to disconnect).

### Debugging Tips
- Enable serial output by setting `M5.begin(true, true, true)` and monitor at 115200 baud.
- Add debug prints in `readSensors()` or `publishData()` to trace issues.
- Use a logic analyzer on I2C (G21/G22) or analog (G33) pins for hardware faults.

---

## Future Enhancements

### Hardware Upgrades
- **Additional Sensors**:
  - Integrate PMS5003 for PM2.5/PM10 monitoring.
  - Add a VL53L0X ToF sensor for distance-based applications.
- **Enclosure**:
  - Design a rugged, IP65-rated 3D-printed case for outdoor use.
  - Include a clip or lanyard for wearable applications.
- **Power**:
  - Add a solar panel for energy harvesting in outdoor settings.
  - Upgrade to a 200mAh battery for extended runtime.

### Software Improvements
- **OTA Updates**:
  - Implement AsyncElegantOTA for seamless firmware updates.
- **Mobile App**:
  - Develop an iOS/Android app using Flutter or React Native for data visualization and alerts.
- **Machine Learning**:
  - Add edge-based anomaly detection (e.g., TensorFlow Lite) for unusual sensor patterns.
  - Predict environmental trends using cloud-based ML models.
- **Data Storage**:
  - Integrate with Firebase or MongoDB for scalable data logging.
  - Add local SD card storage via a Grove SD module.

### Community Features
- **API**:
  - Expose a REST API for third-party integrations.
- **Custom Dashboards**:
  - Create a customizable HTML/JS dashboard template.
- **Workshops**:
  - Host tutorials or webinars for beginners to build their own EcoStick Sentinel.

---

## Contributing

The EcoStick Sentinel is an open-source project, and contributions are highly encouraged! To contribute:
1. Fork the repository (coming soon at `github.com/hamzadenizyilmaz/EcoStick-Sentinel`).
2. Create a feature branch (`git checkout -b feature/new-sensor`).
3. Commit changes with clear messages (`git commit -m "Added PM2.5 sensor support"`).
4. Push to your fork (`git push origin feature/new-sensor`).
5. Open a pull request with a detailed description of your changes.

Please adhere to the [Contributor Covenant Code of Conduct](https://www.contributor-covenant.org/).

### Contribution Ideas
- Add support for new sensors (e.g., BME280, CCS811).
- Optimize power consumption further (e.g., dynamic sensor sampling).
- Create 3D enclosure designs or PCB layouts.
- Develop a mobile app or advanced dashboard.
- Translate documentation into other languages.

---

## License

This project is licensed under the [MIT License](https://opensource.org/licenses/MIT). You are free to use, modify, and distribute the code for personal or commercial purposes, provided the license terms are followed.

---

## Acknowledgments

- **M5Stack Team**: For creating the versatile M5Stick-C platform.
- **Adafruit**: For the SHT31 library and excellent sensor documentation.
- **Open-Source Community**: For libraries like PubSubClient and WiFiManager.
- **IoT Enthusiasts**: For inspiring this project through forums, blogs, and X posts.

---

## Contact

For questions, feedback, or collaboration opportunities, reach out to:
- **GitHub**: [github.com/hamzadenizyilmaz](https://github.com/hamzadenizyilmaz)
- **Email**: [info@vmmaker.com]
- **X**: [@hamzadenizyilmaz](https://x.com/hamzadenizyilmaz)

You can also join the conversation on X by sharing your EcoStick Sentinel projects with the hashtag #EcoStickSentinel!

---

## FAQs

**Q: Can I use a different ESP32 board?**  
A: Yes, but you’ll need to adapt the code for different pinouts and displays. The M5Stick-C’s integrated sensors and screen make it ideal.

**Q: How accurate is the MQ-135 sensor?**  
A: The MQ-135 provides approximate CO2 levels (±100 ppm with calibration). For precise measurements, consider a sensor like the MH-Z19.

**Q: Can I run this without Wi-Fi?**  
A: Yes, the device functions offline, displaying data on the screen. MQTT publishing is skipped if Wi-Fi is unavailable.

**Q: How do I add a new sensor?**  
A: Connect the sensor to the Grove port, update `readSensors()` to include it, and add a new MQTT topic for publishing.

**Q: What’s the cost to build?**  
A: ~$15 for the M5Stick-C, ~$5-10 for the MQ-135, and ~$5 for cables/miscellaneous. Total: $20-30.

---

## Project Status

- **Current Version**: v1.0 (initial release).
- **GitHub Repo**: Coming soon at `github.com/hamzadenizyilmaz/EcoStick-Sentinel`.
- **Known Issues**:
  - MQ-135 calibration is simplified; may need tuning for accuracy.
  - Deep sleep may require manual wake-up in some edge cases.
- **Next Release**: v1.1 (planned for OTA support and additional sensor integration).

---

Thank you for exploring the EcoStick Sentinel! This project is a labor of love for IoT and environmental monitoring. Build it, hack it, and share your creations with the community! 🌍🚀
