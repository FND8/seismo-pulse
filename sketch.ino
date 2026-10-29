#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ==========================================
// 1. SYSTEM & HARDWARE CONFIGURATION
// ==========================================
// Hardware Pins
const uint8_t PIN_LED_GREEN = 16;
const uint8_t PIN_LED_RED   = 4;
const uint8_t PIN_BUZZER    = 27;
const uint8_t PIN_BUTTON    = 13;
const uint8_t PIN_DHT       = 14;
const uint8_t PIN_I2C_SDA   = 21;
const uint8_t PIN_I2C_SCL   = 22;

// Thresholds & Parameters
const float VIB_WATCH_THRESHOLD     = 20.0;  // %
const float VIB_CRITICAL_THRESHOLD  = 50.0;  // %
const float TEMP_CRITICAL_THRESHOLD = 45.0;  // °C
const float HUM_WATCH_THRESHOLD     = 85.0;  // %
const uint8_t WATCH_CYCLES_REQUIRED = 3;
const float EMA_ALPHA               = 0.5;
const uint8_t BASELINE_SAMPLES      = 15;

// Baseline Sanity Limits
const float BASELINE_MAX_VALID      = 1.0;
const float BASELINE_MIN_VALID      = 0.001;
const float BASELINE_FALLBACK_HIGH  = 0.05;
const float BASELINE_FALLBACK_LOW   = 0.01;

// Network & MQTT
const char* WIFI_SSID     = "Wokwi-GUEST";
const char* WIFI_PASSWORD = "";
const char* MQTT_HOST     = "broker.hivemq.com";
const uint16_t MQTT_PORT  = 1883;

const char* TOPIC_TELEMETRY = "sic8/samsul/predictive/telemetry";
const char* TOPIC_ALERT     = "sic8/samsul/predictive/alert";
const char* TOPIC_COMMAND   = "sic8/samsul/predictive/command";

// ==========================================
// 2. OBJECTS & GLOBAL STATE
// ==========================================
Adafruit_MPU6050 mpu;
Adafruit_SSD1306 display(128, 64, &Wire, -1);
DHT dht(PIN_DHT, DHT22);

WiFiClient   espClient;
PubSubClient mqttClient(espClient);

enum SystemState { STATE_HEALTHY, STATE_WATCH, STATE_MAINTENANCE };
SystemState currentState  = STATE_HEALTHY;
SystemState previousState = STATE_HEALTHY;

// Sensor Data
float baselineVibration = 0.0;
float smoothedVibration = 0.0;
float deltaPercentage   = 0.0;
float currentTemp       = 0.0;
float currentHum        = 0.0;
float mpuTemp           = 0.0;

// System Logic State
float healthIndex       = 100.0;
uint8_t watchCyclesCount = 0;
bool alarmAcknowledged  = false;
bool buzzerActive       = false;
bool blinkState         = false;

// Timers
unsigned long lastSampleTime  = 0;
unsigned long lastDisplayTime = 0;
unsigned long lastBlinkTime   = 0;
unsigned long lastDhtTime     = 0;
unsigned long lastSerialTime  = 0;
unsigned long lastMqttTime    = 0;

// Button State
bool lastButtonReading  = HIGH;
bool currentButtonState = HIGH;
unsigned long lastDebounceTime = 0;

// ==========================================
// 3. HELPER FUNCTIONS
// ==========================================
String getStateString(SystemState state) {
  switch (state) {
    case STATE_HEALTHY:     return "HEALTHY";
    case STATE_WATCH:       return "WATCH";
    case STATE_MAINTENANCE: return "MAINTENANCE";
    default:                return "UNKNOWN";
  }
}

// ==========================================
// 4. SENSOR READING & PROCESSING
// ==========================================
float readVibrationRMS() {
  float sumOfSquares = 0;
  const uint8_t samples = 50;
  
  for (int i = 0; i < samples; i++) {
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);
    if (i == 0) mpuTemp = temp.temperature;

    float totalAcc = sqrt(
      (accel.acceleration.x * accel.acceleration.x) +
      (accel.acceleration.y * accel.acceleration.y) +
      (accel.acceleration.z * accel.acceleration.z)
    );
    float pureVibration = abs(totalAcc - 9.81);
    sumOfSquares += (pureVibration * pureVibration);

    unsigned long start = millis();
    while (millis() - start < 2) { yield(); }
  }
  return sqrt(sumOfSquares / samples);
}

float calculateHealthIndex() {
  float vibScore = constrain(100.0 - deltaPercentage, 0.0, 100.0);
  float thermalScore = constrain(100.0 - ((currentTemp - 25.0) * 4.0), 0.0, 100.0);
  return constrain((0.7 * vibScore) + (0.3 * thermalScore), 0.0, 100.0);
}

// ==========================================
// 5. LOGIC & STATE MACHINE
// ==========================================
void evaluateMachineState() {
  deltaPercentage = ((smoothedVibration - baselineVibration) / baselineVibration) * 100.0;

  // Priority 1: Critical Temperature
  if (currentTemp >= TEMP_CRITICAL_THRESHOLD) {
    currentState = STATE_MAINTENANCE;
    watchCyclesCount = 0;
    return;
  }

  // Priority 2: Critical Vibration
  if (deltaPercentage > VIB_CRITICAL_THRESHOLD) {
    currentState = STATE_MAINTENANCE;
    watchCyclesCount = 0;
    return;
  }

  // Priority 3: High Humidity (Watch state)
  if (currentHum >= HUM_WATCH_THRESHOLD) {
    currentState = STATE_WATCH;
    watchCyclesCount = 0;
    return;
  }

  // Priority 4: Vibration Trending (Watch state)
  if (deltaPercentage > VIB_WATCH_THRESHOLD) {
    watchCyclesCount++;
    if (watchCyclesCount >= WATCH_CYCLES_REQUIRED) {
      currentState = STATE_WATCH;
    }
  } 
  else {
    if (watchCyclesCount > 0) {
      watchCyclesCount--;
    } else {
      currentState = STATE_HEALTHY;
      alarmAcknowledged = false; // ACK reset hanya jika kembali normal
    }
  }
}

// ==========================================
// 6. INPUT, OUTPUT & DISPLAY
// ==========================================
void handleButtonPress() {
  bool reading = digitalRead(PIN_BUTTON);
  if (reading != lastButtonReading) lastDebounceTime = millis();

  if ((millis() - lastDebounceTime) > 50) {
    if (reading != currentButtonState) {
      currentButtonState = reading;
      if (currentButtonState == LOW && currentState == STATE_MAINTENANCE) {
        if (!alarmAcknowledged) {
          alarmAcknowledged = true;
          Serial.println("\n[AKSI] Operator menekan ACKNOWLEDGE (fisik).\n");
        }
      }
    }
  }
  lastButtonReading = reading;
}

void applyOutputs() {
  // Kontrol Buzzer secara terpisah (anti-glitch)
  bool shouldBuzzerSound = (currentState == STATE_MAINTENANCE && !alarmAcknowledged);
  
  if (shouldBuzzerSound && !buzzerActive) {
    tone(PIN_BUZZER, 1000);
    buzzerActive = true;
  } else if (!shouldBuzzerSound && buzzerActive) {
    noTone(PIN_BUZZER);
    buzzerActive = false;
  }

  // Kontrol LED
  switch (currentState) {
    case STATE_HEALTHY:
      digitalWrite(PIN_LED_GREEN, HIGH);
      digitalWrite(PIN_LED_RED, LOW);
      break;

    case STATE_WATCH:
      digitalWrite(PIN_LED_GREEN, LOW);
      if (millis() - lastBlinkTime > 300) {
        blinkState = !blinkState;
        digitalWrite(PIN_LED_RED, blinkState);
        lastBlinkTime = millis();
      }
      break;

    case STATE_MAINTENANCE:
      digitalWrite(PIN_LED_GREEN, LOW);
      digitalWrite(PIN_LED_RED, HIGH);
      break;
  }
}

void renderDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("PREDICTIVE MAINTENANCE");
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);

  display.setCursor(0, 16);
  display.print("Vib B:"); display.print(baselineVibration, 2);
  display.print(" C:"); display.println(smoothedVibration, 2);

  display.setCursor(0, 26);
  display.print("D:"); display.print(deltaPercentage, 1); display.print("% ");
  display.print("T:"); display.print(currentTemp, 0); display.print("C ");
  display.print("H:"); 
  if (currentHum >= HUM_WATCH_THRESHOLD) display.print("!");
  display.print(currentHum, 0); display.print("%");

  display.setCursor(0, 36);
  display.print("HI: "); display.print(healthIndex, 1); display.print("%");

  display.drawLine(0, 46, 128, 46, SSD1306_WHITE);

  display.setCursor(0, 48);
  display.print("STATUS: ");
  if (currentState == STATE_HEALTHY) display.println("HEALTHY");
  else if (currentState == STATE_WATCH) display.println("WATCH");
  else display.println("MAINTENANCE");

  if (currentState == STATE_MAINTENANCE) {
    display.setCursor(0, 56);
    display.print(alarmAcknowledged ? "ACKNOWLEDGED" : "PRESS BTN TO ACK");
  }

  display.display();
}

// ==========================================
// 7. NETWORK & MQTT COMMUNICATION
// ==========================================
void setupWiFi() {
  Serial.print("[WIFI] Connecting");
  WiFi.disconnect(true);
  delay(200);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  delay(100);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 60) {
    delay(500);
    Serial.print(".");
    if (attempts > 0 && attempts % 10 == 0) {
      WiFi.disconnect();
      delay(100);
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }
    attempts++;
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("[WIFI] OK: " + WiFi.localIP().toString());
  } else {
    Serial.println("[WIFI] FAILED");
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];

  StaticJsonDocument<128> doc;
  DeserializationError err = deserializeJson(doc, msg);
  if (err) return;

  const char* cmd = doc["cmd"] | "";
  if (strcmp(cmd, "ACK") == 0) {
    if (currentState == STATE_MAINTENANCE && !alarmAcknowledged) {
      alarmAcknowledged = true;
      Serial.println("[MQTT] Remote ACK applied.");
    }
  }
}

void reconnectMQTT() {
  int retries = 0;
  while (!mqttClient.connected() && retries < 5) {
    String clientId = "ESP32-Pred-" + String(random(0xffff), HEX);
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("[MQTT] Connected");
      mqttClient.subscribe(TOPIC_COMMAND);
    } else {
      Serial.printf("[MQTT] Failed rc=%d\n", mqttClient.state());
      delay(3000);
      retries++;
    }
  }
}

void publishTelemetry() {
  StaticJsonDocument<512> doc;
  doc["vib_raw"]  = smoothedVibration;
  doc["baseline"] = baselineVibration;
  doc["delta"]    = deltaPercentage;
  doc["temp"]     = currentTemp;
  doc["hum"]      = currentHum;
  doc["health"]   = healthIndex;
  doc["status"]   = getStateString(currentState);
  doc["ack"]      = alarmAcknowledged;
  doc["ts"]       = millis();

  char payload[512];
  serializeJson(doc, payload);

  if (!mqttClient.publish(TOPIC_TELEMETRY, payload)) {
    Serial.println("[MQTT] Publish FAILED");
  }

  if (currentState == STATE_MAINTENANCE && !alarmAcknowledged) {
    mqttClient.publish(TOPIC_ALERT, "{\"level\":\"CRITICAL\"}");
  }
}

// ==========================================
// 8. MAIN SETUP & LOOP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  dht.begin();

  if (!mpu.begin()) {
    Serial.println("[ERROR] MPU6050");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("[ERROR] OLED");
    while (1);
  }

  display.clearDisplay();
  display.setCursor(0, 20);
  display.println("Calibrating...");
  display.display();

  Serial.println("\n========================================================");
  Serial.println("       PREDICTIVE MAINTENANCE - STARTUP                ");
  Serial.println("========================================================");

  Serial.print("[KALIBRASI] Warm-up...");
  for (int i = 0; i < 10; i++) { readVibrationRMS(); delay(50); }
  Serial.println(" done.");

  Serial.print("[KALIBRASI] Progress: ");
  float sumBaseline = 0;
  for (int i = 0; i < BASELINE_SAMPLES; i++) {
    sumBaseline += readVibrationRMS();
    Serial.print("#");
    delay(100);
  }

  baselineVibration = sumBaseline / BASELINE_SAMPLES;
  Serial.printf("\n[KALIBRASI] Baseline mentah: %.4f m/s^2\n", baselineVibration);

  if (baselineVibration > BASELINE_MAX_VALID) {
    baselineVibration = BASELINE_FALLBACK_HIGH;
    Serial.println("[!] Warning: Baseline terlalu tinggi. Fallback ke 0.05.");
  } else if (baselineVibration < BASELINE_MIN_VALID) {
    baselineVibration = BASELINE_FALLBACK_LOW;
    Serial.println("[!] Warning: Baseline terlalu kecil. Fallback ke 0.01.");
  }

  smoothedVibration = baselineVibration;
  Serial.printf("[KALIBRASI] Baseline final: %.4f m/s^2\n", baselineVibration);

  display.clearDisplay();
  display.setCursor(0, 20);
  display.println("Baseline Ready!");
  display.display();
  delay(1500);

  setupWiFi();
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setKeepAlive(60);
  mqttClient.setSocketTimeout(10);
  mqttClient.setCallback(mqttCallback);
  reconnectMQTT();

  Serial.println("--------------------------------------------------------");
  Serial.println("Waktu\t| Base\t| Current\t| Delta\t\t| HI\t| DHT\t\t| STATUS");
  Serial.println("--------+-------+---------------+---------------+-------+---------------+--------");
}

void loop() {
  if (!mqttClient.connected()) reconnectMQTT();
  mqttClient.loop();

  handleButtonPress();

  if (millis() - lastDhtTime >= 2000) {
    lastDhtTime = millis();
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    if (!isnan(t) && !isnan(h)) {
      currentTemp = t;
      currentHum = h;
    }
  }

  if (millis() - lastSampleTime >= 100) {
    lastSampleTime = millis();

    float rawVibration = readVibrationRMS();
    smoothedVibration = (EMA_ALPHA * rawVibration) + ((1.0 - EMA_ALPHA) * smoothedVibration);

    evaluateMachineState();
    healthIndex = calculateHealthIndex();

    if (currentState != previousState) {
      Serial.println("\n>>> STATUS: " + getStateString(previousState) + " -> " + getStateString(currentState));
      
      if (currentState == STATE_MAINTENANCE) {
        if (currentTemp >= TEMP_CRITICAL_THRESHOLD) Serial.println("    [SEBAB] Thermal Overload");
        else Serial.println("    [SEBAB] Vibration Degradation");
      } else if (currentState == STATE_WATCH) {
        if (currentHum >= HUM_WATCH_THRESHOLD) Serial.println("    [SEBAB] Kelembaban tinggi!");
        else Serial.println("    [SEBAB] Getaran mulai naik.");
      }
      
      Serial.println("\nWaktu\t| Base\t| Current\t| Delta\t\t| HI\t| DHT\t\t| STATUS");
      Serial.println("--------+-------+---------------+---------------+-------+---------------+--------");
      previousState = currentState;
    }
  }

  if (millis() - lastSerialTime >= 500) {
    lastSerialTime = millis();
    char deltaStr[12];
    snprintf(deltaStr, sizeof(deltaStr), "%+.1f%%", deltaPercentage);

    Serial.printf("%lu ms\t| %.3f\t| %.3f\t\t| %7s\t| %.1f%%\t| %.1fC/%.0f%%\t| %s\n",
                  millis(), baselineVibration, smoothedVibration, deltaStr,
                  healthIndex, currentTemp, currentHum, getStateString(currentState).c_str());
  }

  applyOutputs();

  if (millis() - lastDisplayTime >= 500) {
    lastDisplayTime = millis();
    renderDisplay();
  }

  if (millis() - lastMqttTime >= 2000 && mqttClient.connected()) {
    lastMqttTime = millis();
    publishTelemetry();
  }
}
