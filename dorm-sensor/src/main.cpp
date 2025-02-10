#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoHA.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_NeoPixel.h>
#include "ScioSense_ENS160.h"
#include "PMS.h"
// #include "driver/temperature_sensor.h"

const char* SSID     = "";
const char* PASSWORD = "";

const char* BROKER_ADDRESS  = "";
const uint16_t BROKER_PORT  =  99999;
const char* BROKER_USERNAME = "";
const char* BROKER_PASSWORD = "";
const char* CLIENT_ID       = "";
unsigned long lastStartConnect = 0;

const uint8_t LED_BUILDIN   = 8;

WiFiClient wifiClient;
HADevice device;
HAMqtt mqtt(wifiClient, device);

// HASensorNumber chipTemp("dorm_sensor_chip_temperature", HASensorNumber::PrecisionP1);
// temperature_sensor_handle_t temp_handle = NULL;
// temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
// unsigned long lastChipTempAt = 0;
// const unsigned long chipTempInterval = 5 * 60 * 1000; // 5 minutes
// const float chipTempDiff = 2;
// float chipTemperature;

HASensorNumber uptime("dorm_sensor_uptime");
unsigned long lastUptimeAt = 0;
const unsigned long uptimeInterval = 5 * 1000; // 5 seconds

// HASensorNumber dhtTemp("dorm_sensor_temperature_dht", HASensorNumber::PrecisionP1);
// HASensorNumber dhtHumid("dorm_sensor_humidity_dht", HASensorNumber::PrecisionP1);
// const uint8_t DHT_PIN = 2;
// const uint8_t DHT_TYPE = DHT22;
// DHT_Unified dht(DHT_PIN, DHT_TYPE);
// float dhtHumidity, dhtTemperature;
// unsigned long lastDhtAt = 0;
// unsigned long dhtDelay = 2000;
// const unsigned long dhtInterval = 10 * 60 * 1000; // 10 minute
// const float dhtTempDiff = 0.5;
// const float dhtHumidDiff = 2.0;

HASensorNumber bmeTemp("dorm_sensor_temperature_bme", HASensorNumber::PrecisionP1);
HASensorNumber bmeHumid("dorm_sensor_humidity_bme", HASensorNumber::PrecisionP1);
HASensorNumber bmePress("dorm_sensor_pressure_bme", HASensorNumber::PrecisionP1);
Adafruit_BME280 bme;
float bmeHumidity, bmeTemperature, bmePressure;
unsigned long lastBmeAt = 0;
unsigned long bmeDelay = 2000;
const unsigned long bmeInterval = 10 * 60 * 1000; // 10 minute
const float bmeTempDiff = 2.0;
const float bmeHumidDiff = 2.0;
const float bmePressureDiff = 2.0;

// Measure    Real
// 30.12 °C   24.4 °C    83.97 %    58.0 %
// 33.63 °C   28.1 °C    92.08 %    64.0 %
// HASensorNumber ahtTemp("dorm_sensor_temperature_aht", HASensorNumber::PrecisionP1);
// HASensorNumber ahtHumid("dorm_sensor_humidity_aht", HASensorNumber::PrecisionP1);
// Adafruit_AHTX0 aht;
// uint8_t ahtAvailable = 0;
// unsigned long lastAhtAt = 0;
// // const unsigned long ahtInterval = 1 * 60 * 60 * 1000; // 1 hour
// const unsigned long ahtInterval = 1 * 60 * 1000;
// float ahtHumidity, ahtTemperature;
// const float ahtTempDiff = 0.5;
// const float ahtHumidDiff = 2.0;
// double ahtTempFactor = 1.05413;
// double ahtTempOffset = -7.3504;
// double ahtHumidFactor = 0.739827;
// double ahtHumidOffset = -4.1233;

HASensorNumber ens160AQI("dorm_sensor_aqi");
HASensorNumber ens160TVOC("dorm_sensor_tvoc");
HASensorNumber ens160eCO2("dorm_sensor_eco2");
ScioSense_ENS160 ens160(ENS160_I2CADDR_1);
unsigned long lastEnsAt = 0;
const unsigned long ensWarmup = 3 * 60 * 1000; // 3 minutes
const unsigned long ensInterval = 5 * 60 * 1000; // 5 minutes
const uint16_t ens160TVOCDiff = 50;
const uint16_t ens160eCO2Diff = 100;
uint16_t AQI, TVOC, eCO2;

HASensorNumber pmsPM01("dorm_sensor_pm01");
HASensorNumber pmsPM25("dorm_sensor_pm25");
HASensorNumber pmsPM10("dorm_sensor_pm10");
PMS pms(Serial1);
PMS::DATA pmsData;
unsigned long lastPmsAt = 0;
const unsigned long pmsInterval = 5 * 60 * 1000; // 5 minutes
uint16_t pm01, pm25, pm10;

HASwitch relay1("dorm_sensor_relay1");
HASwitch relay2("dorm_sensor_relay2");
HASwitch relay_filter("dorm_sensor_filter");
const uint8_t relay1Pin = 7;
const uint8_t relay2Pin = 6;
const uint8_t relayFilterPin = 5;

const uint8_t detector_pin = 3;
const uint8_t response_led_pin = 2;
Adafruit_NeoPixel response_led(8, response_led_pin, NEO_GRB + NEO_KHZ800);

// HASwitch flashlight("dorm_sensor_flashlight");
HALight flashlight("dorm_sensor_flashlight", HALight::BrightnessFeature | HALight::ColorTemperatureFeature | HALight::RGBFeature);
uint8_t flashlightColor[3];
uint8_t flashlightBrightness;


void connectWiFi();
void onRelay1Command(bool state, HASwitch *sender);
void onRelay2Command(bool state, HASwitch *sender);
void onRelayFilterCommand(bool state, HASwitch *sender);
void onStateCommand(bool state, HALight *sender);
void onBrightnessCommand(uint8_t brightness, HALight *sender);
void onColorTemperatureCommand(uint16_t temperature, HALight *sender);
void onRGBColorCommand(HALight::RGBColor rgb, HALight *sender);
void flashlightSetColor(uint32_t color);
void updateLedStatus();
void scanI2C();

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600, SERIAL_8N1, 0, 1);

  pinMode(LED_BUILDIN, OUTPUT);
  digitalWrite(LED_BUILDIN, HIGH);

  for (uint8_t i = 10; i > 0; i--) {
    Serial.print("Connecting to WiFi in "); Serial.print(i); Serial.println("...");
    delay(1000);
  }

  connectWiFi();

  byte mac[6];
  WiFi.macAddress(mac);
  device.setName(CLIENT_ID);
  device.setUniqueId(mac, sizeof(mac));

  // chipTemp.setIcon("mdi:chip");
  // chipTemp.setName("Chip Temperature");
  // chipTemp.setUnitOfMeasurement("°C");
  // chipTemp.setDeviceClass("temperature");

  uptime.setIcon("mdi:clock");
  uptime.setName("Uptime");
  uptime.setUnitOfMeasurement("s");
  uptime.setDeviceClass("duration");

  // dhtTemp.setIcon("mdi:thermometer");
  // dhtTemp.setName("Temperature");
  // dhtTemp.setUnitOfMeasurement("°C");
  // dhtTemp.setDeviceClass("temperature");

  // dhtHumid.setIcon("mdi:water-percent");
  // dhtHumid.setName("Humidity");
  // dhtHumid.setUnitOfMeasurement("%");
  // dhtHumid.setDeviceClass("humidity");

  bmeTemp.setIcon("mdi:thermometer");
  bmeTemp.setName("Temperature");
  bmeTemp.setUnitOfMeasurement("°C");
  bmeTemp.setDeviceClass("temperature");

  bmeHumid.setIcon("mdi:water-percent");
  bmeHumid.setName("Humidity");
  bmeHumid.setUnitOfMeasurement("%");
  bmeHumid.setDeviceClass("humidity");

  bmePress.setIcon("mdi:gauge");
  bmePress.setName("Pressure");
  bmePress.setUnitOfMeasurement("hPa");
  bmePress.setDeviceClass("pressure");

  // ahtHumid.setIcon("mdi:water-percent");
  // ahtHumid.setName("Humidity");
  // ahtHumid.setUnitOfMeasurement("%");
  // ahtHumid.setDeviceClass("humidity");

  // ahtTemp.setIcon("mdi:thermometer");
  // ahtTemp.setName("Temperature");
  // ahtTemp.setUnitOfMeasurement("°C");
  // ahtTemp.setDeviceClass("temperature");

  ens160AQI.setIcon("mdi:air-filter");
  ens160AQI.setName("Air Quality Index");
  ens160AQI.setUnitOfMeasurement("aqi");

  ens160TVOC.setIcon("mdi:leaf-circle");
  ens160TVOC.setName("Total Volatile Organic Compounds");
  ens160TVOC.setUnitOfMeasurement("ppb");
  ens160TVOC.setDeviceClass("volatile_organic_compounds_parts");

  ens160eCO2.setIcon("mdi:molecule-co2");
  ens160eCO2.setName("Equivalent CO2");
  ens160eCO2.setUnitOfMeasurement("ppm");
  ens160eCO2.setDeviceClass("carbon_dioxide");

  pmsPM01.setIcon("mdi:chemical-weapon");
  pmsPM01.setName("PM 1.0");
  pmsPM01.setUnitOfMeasurement("µg/m³");
  pmsPM01.setDeviceClass("pm1");

  pmsPM25.setIcon("mdi:chemical-weapon");
  pmsPM25.setName("PM 2.5");
  pmsPM25.setUnitOfMeasurement("µg/m³");
  pmsPM25.setDeviceClass("pm25");

  pmsPM10.setIcon("mdi:chemical-weapon");
  pmsPM10.setName("PM 10");
  pmsPM10.setUnitOfMeasurement("µg/m³");
  pmsPM10.setDeviceClass("pm10");

  relay1.setIcon("mdi:light-switch");
  relay1.setName("Relay 1");
  relay1.setRetain(true);
  relay1.setDeviceClass("switch");
  relay1.onCommand(onRelay1Command);
  pinMode(relay1Pin, OUTPUT);

  relay2.setIcon("mdi:light-switch");
  relay2.setName("Relay 2");
  relay2.setRetain(true);
  relay2.setDeviceClass("switch");
  relay2.onCommand(onRelay2Command);
  pinMode(relay2Pin, OUTPUT);

  relay_filter.setIcon("mdi:light-switch");
  relay_filter.setName("Filter");
  relay_filter.setRetain(true);
  relay_filter.setDeviceClass("switch");
  relay_filter.onCommand(onRelayFilterCommand);
  pinMode(relayFilterPin, OUTPUT);

  flashlight.setIcon("mdi:flashlight");
  flashlight.setName("Flashlight");
  flashlight.setRetain(true);
  flashlight.onStateCommand(onStateCommand);
  flashlight.onBrightnessCommand(onBrightnessCommand);
  flashlight.onColorTemperatureCommand(onColorTemperatureCommand);
  flashlight.onRGBColorCommand(onRGBColorCommand);

  // temperature_sensor_install(&temp_sensor_config, &temp_handle);
  // temperature_sensor_enable(temp_handle);

  pinMode(detector_pin, INPUT);

  Wire.begin(9, 10);

  // dht.begin();

  // if (aht.begin()) {
  //   Serial.println("\tAHT21 found");
  //   ahtAvailable = 1;
  // } else {
  //   Serial.println("\tAHT21 not found");
  //   digitalWrite(LED_BUILDIN, LOW);
  // }

  // scanI2C();

  if (!bme.begin(0x76, &Wire)) {
    Serial.println("\tBME280 not found");
    digitalWrite(LED_BUILDIN, LOW);
  } else {
    Serial.println("\tBME280 found");
  }

  ens160.begin();
  if (ens160.available()) {
    Serial.println("\tENS160 found");
    Serial.print("Rev: "); Serial.print(ens160.getMajorRev());
    Serial.print("."); Serial.print(ens160.getMinorRev());
    Serial.print("."); Serial.println(ens160.getBuild());

    Serial.print("Setting mode to STD: ");
    Serial.println(ens160.setMode(ENS160_OPMODE_STD) ? "done." : "failed!" );
  } else {
    Serial.println("\tENS160 not found");
    digitalWrite(LED_BUILDIN, LOW);
  }

  pms.wakeUp();
  pms.passiveMode();

  // sensor_t sensor;
  // dht.temperature().getSensor(&sensor);
  // Serial.println(F("------------------------------------"));
  // Serial.println(F("Temperature Sensor"));
  // Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  // Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  // Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  // Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("°C"));
  // Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("°C"));
  // Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("°C"));
  // Serial.println(F("------------------------------------"));
  // // Print humidity sensor details.
  // dht.humidity().getSensor(&sensor);
  // Serial.println(F("Humidity Sensor"));
  // Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  // Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  // Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  // Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
  // Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
  // Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
  // Serial.println(F("------------------------------------"));

  // dhtDelay = sensor.min_delay / 1000;

  mqtt.begin(BROKER_ADDRESS, BROKER_PORT,BROKER_USERNAME, BROKER_PASSWORD);

  response_led.begin();
  response_led.clear();

  Serial.println("Setup done");
  Serial.println("\tStart");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }

  updateLedStatus();

  unsigned long now = millis();
  mqtt.loop();

  // if (true) {
  //   float temp = 0;
  //   temperature_sensor_get_celsius(temp_handle, &temp);

  //   uint8_t update = 0;
  //   if (now - lastChipTempAt >= chipTempInterval) update = 1;
  //   if (abs(temp - chipTemperature) >= chipTempDiff) update = 1;

  //   if (update) {
  //     chipTemperature = temp;
  //     chipTemp.setValue(chipTemperature);
  //     Serial.print("Chip Temperature: "); Serial.print(chipTemperature); Serial.println(" °C");
  //     lastChipTempAt = now;
  //   }
  // }

  if (now - lastUptimeAt >= uptimeInterval) {
    lastUptimeAt = now;
    uptime.setValue(now / 1000);
    Serial.print("Uptime: "); Serial.print(now / 1000); Serial.println(" s");
  }

  // if (now - lastDhtAt >= dhtDelay) {
  //   sensors_event_t eventT;
  //   sensors_event_t eventH;
  //   dht.temperature().getEvent(&eventT);
  //   dht.humidity().getEvent(&eventH);

  //   if (isnan(eventT.temperature) || isnan(eventH.relative_humidity)) {
  //     Serial.println("Failed to read from DHT sensor!");
  //   } else {
  //     float calcTemperature = eventT.temperature;
  //     float calcHumidity = eventH.relative_humidity;

  //     uint8_t update = 0;
  //     if (now - lastDhtAt >= dhtInterval) update = 1;
  //     if (abs(calcTemperature - dhtTemperature) >= dhtHumidDiff) update = 1;
  //     if (abs(calcHumidity - dhtHumidity) >= dhtTempDiff) update = 1;

  //     if (dhtTemperature < -40 || dhtHumidity > 80) update = 0;
  //     if (dhtHumidity < 0 || dhtHumidity > 100) update = 0;

  //     if (update) {
  //       dhtHumidity = calcHumidity;
  //       dhtTemperature = calcTemperature;
  //       dhtHumid.setValue(dhtHumidity);
  //       dhtTemp.setValue(dhtTemperature);
  //       Serial.print("Humidity DHT: "); Serial.print(dhtHumidity); Serial.println(" %");
  //       Serial.print("Temperature DHT: "); Serial.print(dhtTemperature); Serial.println(" °C");
  //       lastDhtAt = now;
  //     }
  //   }
  // }

  if (now - lastBmeAt >= bmeDelay) {
    float calcTemperature = bme.readTemperature();
    float calcHumidity = bme.readHumidity();
    float calcPressure = bme.readPressure() / 100.0F;

    uint8_t update = 0;
    if (now - lastBmeAt >= bmeInterval) update = 1;
    if (abs(calcTemperature - bmeTemperature) >= bmeTempDiff) update = 1;
    if (abs(calcHumidity - bmeHumidity) >= bmeHumidDiff) update = 1;
    if (abs(calcPressure - bmePressure) >= bmePressureDiff) update = 1;

    if (bmeTemperature < -40 || bmeTemperature > 80) update = 0;
    if (bmeHumidity < 0 || bmeHumidity > 100) update = 0;

    if (update) {
      bmeHumidity = calcHumidity;
      bmeTemperature = calcTemperature;
      bmePressure = calcPressure;
      bmeHumid.setValue(bmeHumidity);
      bmeTemp.setValue(bmeTemperature);
      bmePress.setValue(bmePressure);
      Serial.print("Humidity BME: "); Serial.print(bmeHumidity); Serial.println(" %");
      Serial.print("Temperature BME: "); Serial.print(bmeTemperature); Serial.println(" °C");
      Serial.print("Pressure BME: "); Serial.print(bmePressure); Serial.println(" hPa");
      lastBmeAt = now;
    }
  }

  // if (ahtAvailable && now - lastAhtAt >= 2000) {
  //   sensors_event_t sensorHumid, sensorTemp;
  //   aht.getEvent(&sensorHumid, &sensorTemp);

  //   float calcTemperature = (float)(ahtTempFactor * (double)sensorTemp.temperature + ahtTempOffset);
  //   float calcHumidity = (float)(ahtHumidFactor * (double)sensorHumid.relative_humidity + ahtHumidOffset);

  //   uint8_t update = 0;
  //   if (now - lastAhtAt >= ahtInterval) update = 1;
  //   if (abs(calcTemperature - ahtTemperature) >= ahtTempDiff) update = 1;
  //   if (abs(calcHumidity - ahtHumidity) >= ahtHumidDiff) update = 1;

  //   if (update) {
  //     ahtHumidity = calcHumidity;
  //     ahtTemperature = calcTemperature;
  //     // ahtHumid.setValue(humidity);
  //     // ahtTemp.setValue(temperature);
  //     Serial.print("Humidity AHT: "); Serial.print(ahtHumidity); Serial.println(" %");
  //     Serial.print("Temperature AHT: "); Serial.print(ahtTemperature); Serial.println(" °C");
  //     lastAhtAt = now;
  //   }
  // }

  if (now - ensWarmup >= 0 && ens160.available() && now - lastEnsAt >= 1000) {
    ens160.set_envdata(bmeTemperature, bmeHumidity);
    ens160.measure(true);

    uint8_t update = 0;
    if (now - lastEnsAt > ensInterval) update = 1;
    if (AQI != ens160.getAQI()) update = 1;
    if (abs(ens160.getTVOC() - TVOC) >= ens160TVOCDiff) update = 1;
    if (abs(ens160.geteCO2() - eCO2) >= ens160eCO2Diff) update = 1;

    if (ens160.getAQI() < 1 || ens160.getAQI() > 5) update = 0;
    if (ens160.getTVOC() < 0 || ens160.getTVOC() > 65000) update = 0;
    if (ens160.geteCO2() < 400 || ens160.geteCO2() > 65000) update = 0;

    if (update) {
      AQI = ens160.getAQI();
      TVOC = ens160.getTVOC();
      eCO2 = ens160.geteCO2();
      ens160AQI.setValue(AQI);
      ens160TVOC.setValue(TVOC);
      ens160eCO2.setValue(eCO2);
      Serial.print("AQI: "); Serial.println(AQI);
      Serial.print("TVOC: "); Serial.print(TVOC); Serial.println(" ppb");
      Serial.print("eCO2: "); Serial.print(eCO2); Serial.println(" ppm");
      lastEnsAt = now;
    }
  }

  if (now - lastPmsAt >= pmsInterval) {
    pms.requestRead();
    if (pms.readUntil(pmsData)) {
      pm01 = pmsData.PM_AE_UG_1_0;
      pm25 = pmsData.PM_AE_UG_2_5;
      pm10 = pmsData.PM_AE_UG_10_0;
      pmsPM01.setValue(pm01);
      pmsPM25.setValue(pm25);
      pmsPM10.setValue(pm10);
      Serial.print("PM 1.0: "); Serial.print(pm01); Serial.println(" µg/m³");
      Serial.print("PM 2.5: "); Serial.print(pm25); Serial.println(" µg/m³");
      Serial.print("PM 10: "); Serial.print(pm10); Serial.println(" µg/m³");
      lastPmsAt = now;
    }
  }
}

void connectWiFi() {
  Serial.print("Connecting to ");
  Serial.print(SSID);
  WiFi.begin(SSID, PASSWORD);
  lastStartConnect = millis();
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_BUILDIN, LOW);
    delay(500);
    Serial.print(".");

    if (millis() - lastStartConnect > 10000) {
      WiFi.begin(SSID, PASSWORD);
      lastStartConnect = millis();
    }
  }
  digitalWrite(LED_BUILDIN, HIGH);
  Serial.println(" connected");
}

void onRelay1Command(bool state, HASwitch *sender) {
  Serial.print("Relay 1: "); Serial.println(state);

  digitalWrite(relay1Pin, (state ? HIGH : LOW));
  sender->setState(state);
}

void onRelay2Command(bool state, HASwitch *sender) {
  Serial.print("Relay 2: "); Serial.println(state);

  digitalWrite(relay2Pin, (state ? HIGH : LOW));
  sender->setState(state);
}

void onRelayFilterCommand(bool state, HASwitch *sender) {
  Serial.print("Filter: "); Serial.println(state);

  digitalWrite(relayFilterPin, (state ? HIGH : LOW));
  sender->setState(state);
}

// void onFlashlightCommand(bool state, HASwitch *sender) {
//   Serial.print("Flashlight: "); Serial.println(state);

//   uint8_t brightness = 255 * (state ? 1 : 0);
//   uint32_t color = response_led.Color(brightness, brightness, brightness);
//   response_led.setPixelColor(0, color);
//   response_led.setPixelColor(1, color);
//   response_led.setPixelColor(2, color);
// }

void onStateCommand(bool state, HALight *sender) {
  Serial.print("Flashlight: "); Serial.println(state);
  uint32_t color = response_led.Color(flashlightColor[0] * (state ? (flashlightBrightness / 255.0) : 0), flashlightColor[1] * (state ? (flashlightBrightness / 255.0) : 0), flashlightColor[2] * (state ? (flashlightBrightness / 255.0) : 0));
  flashlightSetColor(color);

  sender->setState(state);
}

void onBrightnessCommand(uint8_t brightness, HALight *sender) {
  Serial.print("Brightness: "); Serial.println(brightness);
  flashlightBrightness = brightness;

  sender->setBrightness(brightness);
}

void onColorTemperatureCommand(uint16_t mired, HALight *sender) {
  uint16_t temperature = (mired > 0) ? (1000000 / mired) : 40000;
  Serial.print("Temperature: "); Serial.println(temperature);

  if (temperature < 1000)  temperature = 1000;
  if (temperature > 40000) temperature = 40000;

  float temp = temperature / 100.0; // Scale down for calculations
  float r, g, b;

  // Calculate Red
  if (temp <= 66) {
      r = 255;
  } else {
      r = 329.698727446 * pow(temp - 60, -0.1332047592);
      if (r < 0) r = 0;
      if (r > 255) r = 255;
  }

  // Calculate Green
  if (temp <= 66) {
      g = 99.4708025861 * log(temp) - 161.1195681661;
      if (g < 0) g = 0;
      if (g > 255) g = 255;
  } else {
      g = 288.1221695283 * pow(temp - 60, -0.0755148492);
      if (g < 0) g = 0;
      if (g > 255) g = 255;
  }

  // Calculate Blue
  if (temp >= 66) {
      b = 255;
  } else if (temp <= 19) {
      b = 0;
  } else {
      b = 138.5177312231 * log(temp - 10) - 305.0447927307;
      if (b < 0) b = 0;
      if (b > 255) b = 255;
  }

  flashlightColor[0] = r;
  flashlightColor[1] = g;
  flashlightColor[2] = b;

  sender->setColorTemperature(mired);
}

void onRGBColorCommand(HALight::RGBColor rgb, HALight *sender) {
  Serial.print("R: "); Serial.print(rgb.red);
  Serial.print(" G: "); Serial.print(rgb.green);
  Serial.print(" B: "); Serial.println(rgb.blue);

  flashlightColor[0] = rgb.red;
  flashlightColor[1] = rgb.green;
  flashlightColor[2] = rgb.blue;

  sender->setRGBColor(rgb);
}

void flashlightSetColor(uint32_t color) {
  response_led.setPixelColor(0, color);
  response_led.setPixelColor(1, color);
  response_led.setPixelColor(2, color);
}

void updateLedStatus() {
  float brightness = 10.0 / 1000.0;

  uint32_t pm01_color = 0;
  if (pm01 <= 12) {
    pm01_color = response_led.Color(0 * brightness, 228 * brightness, 0 * brightness);
  } else if (pm01 <= 35) {
    pm01_color = response_led.Color(255 * brightness, 255 * brightness, 0 * brightness);
  } else if (pm01 <= 55) {
    pm01_color = response_led.Color(255 * brightness, 126 * brightness, 0 * brightness);
  } else if (pm01 <= 150) {
    pm01_color = response_led.Color(255 * brightness, 0 * brightness, 0 * brightness);
  } else if (pm01 <= 250) {
    pm01_color = response_led.Color(143 * brightness, 63 * brightness, 151 * brightness);
  } else {
    pm01_color = response_led.Color(126 * brightness, 0 * brightness, 35 * brightness);
  }
  response_led.setPixelColor(7, pm01_color);

  uint32_t pm25_color = 0;
  if (pm25 <= 12) {
    pm25_color = response_led.Color(0 * brightness, 228 * brightness, 0 * brightness);
  } else if (pm25 <= 35) {
    pm25_color = response_led.Color(255 * brightness, 255 * brightness, 0 * brightness);
  } else if (pm25 <= 55) {
    pm25_color = response_led.Color(255 * brightness, 126 * brightness, 0 * brightness);
  } else if (pm25 <= 150) {
    pm25_color = response_led.Color(255 * brightness, 0 * brightness, 0 * brightness);
  } else if (pm25 <= 250) {
    pm25_color = response_led.Color(143 * brightness, 63 * brightness, 151 * brightness);
  } else {
    pm25_color = response_led.Color(126 * brightness, 0 * brightness, 35 * brightness);
  }
  response_led.setPixelColor(6, pm25_color);

  uint32_t pm10_color = 0;
  if (pm10 <= 12) {
    pm10_color = response_led.Color(0 * brightness, 228 * brightness, 0 * brightness);
  } else if (pm10 <= 35) {
    pm10_color = response_led.Color(255 * brightness, 255 * brightness, 0 * brightness);
  } else if (pm10 <= 55) {
    pm10_color = response_led.Color(255 * brightness, 126 * brightness, 0 * brightness);
  } else if (pm10 <= 150) {
    pm10_color = response_led.Color(255 * brightness, 0 * brightness, 0 * brightness);
  } else if (pm10 <= 250) {
    pm10_color = response_led.Color(143 * brightness, 63 * brightness, 151 * brightness);
  } else {
    pm10_color = response_led.Color(126 * brightness, 0 * brightness, 35 * brightness);
  }
  response_led.setPixelColor(5, pm10_color);

  uint32_t tvoc_color = 0;
  if (TVOC <= 220) {
    tvoc_color = response_led.Color(0 * brightness, 228 * brightness, 0 * brightness);
  } else if (TVOC <= 660) {
    tvoc_color = response_led.Color(255 * brightness, 255 * brightness, 0 * brightness);
  } else if (TVOC <= 2200) {
    tvoc_color = response_led.Color(255 * brightness, 126 * brightness, 0 * brightness);
  } else if (TVOC <= 5500) {
    tvoc_color = response_led.Color(255 * brightness, 0 * brightness, 0 * brightness);
  } else if (TVOC <= 11000) {
    tvoc_color = response_led.Color(143 * brightness, 63 * brightness, 151 * brightness);
  } else {
    tvoc_color = response_led.Color(126 * brightness, 0 * brightness, 35 * brightness);
  }
  response_led.setPixelColor(4, tvoc_color);

  uint32_t eCO2_color = 0;
  if (eCO2 <= 600) {
    eCO2_color = response_led.Color(0 * brightness, 228 * brightness, 0 * brightness);
  } else if (eCO2 <= 1000) {
    eCO2_color = response_led.Color(255 * brightness, 255 * brightness, 0 * brightness);
  } else if (eCO2 <= 1500) {
    eCO2_color = response_led.Color(255 * brightness, 126 * brightness, 0 * brightness);
  } else if (eCO2 <= 2000) {
    eCO2_color = response_led.Color(255 * brightness, 0 * brightness, 0 * brightness);
  } else if (eCO2 <= 5000) {
    eCO2_color = response_led.Color(143 * brightness, 63 * brightness, 151 * brightness);
  } else {
    eCO2_color = response_led.Color(126 * brightness, 0 * brightness, 35 * brightness);
  }
  response_led.setPixelColor(3, eCO2_color);

  response_led.show();
}

void scanI2C() {
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknow error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  }
  else {
    Serial.println("done\n");
  }
}