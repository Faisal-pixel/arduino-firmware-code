// ============================================================================
//  SOLAR-POWERED IoT WATER LEVEL MONITOR
//  Firmware v3 - Firebase Realtime Database + on-device energy prediction
// ============================================================================
//  Final Year Project: Machine Learning-Based Energy Prediction and Adaptive
//  Power Scheduling for a Solar-Powered IoT Drainage Water Level Monitoring
//  Device
//
//  CHANGES FROM THE PREVIOUS (UBIDOTS) VERSION - all marked [NEW] below:
//   1. MQTT/Ubidots removed entirely, replaced with Firebase REST (HTTPS).
//   2. Added the trained linear regression model (predictEnergy) so the device
//      now predicts incoming solar power itself, instead of only measuring it.
//   3. Added NTP time sync, needed to supply "hour of day" to the model.
//   4. Payload now also carries "mode" and "predicted_power".
//   5. [v3.1] Mode switching is now prediction-driven, not just reactive to
//      battery voltage. See updatePublishInterval() and LOW_PREDICTED_POWER_W
//      below - if the model expects little incoming solar power, the device
//      now drops to POWER_SAVE early instead of waiting for the battery to
//      actually fall. This is the "adaptive, prediction-driven scheduling"
//      the project title refers to - previously only the voltage threshold
//      existed and the prediction was calculated but unused.
//   6. [v3.2] CRITICAL mode no longer goes permanently silent. It still
//      turns WiFi off immediately to save power, but now wakes briefly every
//      5 minutes (CRITICAL_WAKE_INTERVAL) to send one status update before
//      going back to WiFi-off. See criticalWakeAndPublish(). Previously the
//      dashboard would show nothing new for as long as the battery stayed
//      low, which defeats the purpose of a monitoring device during exactly
//      the period someone most needs to check on it.
//   7. [v3.3] FLOOD OVERRIDE. If the water level reaches FLOOD_THRESHOLD_CM,
//      the device forces NORMAL mode (5 s publishing, WiFi stays on) no
//      matter what the battery voltage or the model prediction say - the
//      battery-state and prediction checks are skipped entirely while the
//      water is high. Rationale: an active flood is a higher-priority
//      emergency than battery longevity, so the device deliberately trades
//      runtime for alerting. Payload now also carries "flood_alert".
//      When the water drops back below the threshold, normal battery- and
//      prediction-based scheduling resumes automatically on the next cycle.
//
//   8. [v3.4] FOUR BUG FIXES, all found by code review. See [FIX - v3.4]:
//      a) sendToFirebase() treated HTTP rejections as success. It checked
//         "code <= 0", but HTTPClient returns NEGATIVE values for connection
//         errors and POSITIVE values for HTTP errors (400/401/404/500). A
//         rejected write reported success, and the buffered copy was then
//         deleted. Now only HTTP 2xx counts, and the response body is logged.
//      b) publishPendingData() started with "if (criticalMode) return;", so
//         criticalWakeAndPublish() could never flush the buffer even though
//         its comment claimed it did. Guard removed, work capped instead.
//      c) criticalWakeAndPublish() printed "status sent" regardless of the
//         result and had no fallback, so failed readings vanished. It now
//         reports the true result and buffers on failure.
//      d) The offline buffer wrote one file per reading (one every 2 s in
//         CRITICAL, ~1,800 files/hour). It is now a single append-only
//         .jsonl file, uploaded at most MAX_BACKLOG_PER_PASS lines per pass
//         so catching up cannot stall the main loop.
//
//  ALL SENSOR READING LOGIC IS UNCHANGED - same pins, same dividers, same
//  calibration factors, same EmonLib CT setup, same ultrasonic logic.
// ============================================================================

#include <Wire.h>
#include "EmonLib.h"
#include <WiFi.h>
#include <HTTPClient.h>          // [NEW] replaces PubSubClient
#include <WiFiClientSecure.h>    // [NEW] Firebase requires HTTPS
#include <LittleFS.h>
#include <ArduinoJson.h>
#include "time.h"                // [NEW] for hour-of-day

// ============================================================================
// WIFI SETTINGS
// ============================================================================
const char* ssid     = "Oyenike";
const char* password = "@nurse22";

// ============================================================================
// [NEW] FIREBASE SETTINGS  (replaces the Ubidots MQTT block)
// ============================================================================
const char* firebaseHost = "https://final-year-project-cd7d4-default-rtdb.firebaseio.com";

// ============================================================================
// [NEW] NTP SETTINGS - Lagos is UTC+1 (WAT), no daylight saving
// ============================================================================
const char* ntpServer         = "pool.ntp.org";
const long  gmtOffset_sec     = 3600;
const int   daylightOffset_sec = 0;

// ============================================================================
// PIN DEFINITIONS (unchanged)
// ============================================================================
#define BATTERY_PIN       34
#define SOLAR_VOLTAGE_PIN 35
#define SOLAR_CURRENT_PIN 33
#define LDR_PIN           32
#define TRIG_PIN          22
#define ECHO_PIN          23
#define RELAY_PIN         21

// ============================================================================
// ADC SETTINGS (unchanged)
// ============================================================================
#define ADC_RESOLUTION  4096.0
#define ADC_REF_VOLTAGE 3.3

// ============================================================================
// VOLTAGE DIVIDER CONSTANTS (unchanged)
// ============================================================================
#define BATTERY_R1 2000.0
#define BATTERY_R2 3000.0
#define BATTERY_RATIO (BATTERY_R2 / (BATTERY_R1 + BATTERY_R2))
#define SOLAR_R1 100000.0
#define SOLAR_R2 10000.0
#define SOLAR_RATIO (SOLAR_R2 / (SOLAR_R1 + SOLAR_R2))

// ============================================================================
// CALIBRATION FACTORS (unchanged)
// ============================================================================
#define BATTERY_CALIBRATION 1.053
#define SOLAR_CALIBRATION 2.207
#define CT_CALIBRATION 0.1320

// ============================================================================
// BATTERY CONSTANTS (unchanged)
// ============================================================================
#define BATTERY_MAX_VOLTAGE 4.20
#define BATTERY_MIN_VOLTAGE 3.00
#define BATTERY_NORMAL_MIN 3.70
#define BATTERY_POWER_SAVE_MIN 3.30

// ============================================================================
// [NEW - v3.1] PREDICTION-DRIVEN SCHEDULING THRESHOLD
// ----------------------------------------------------------------------------
//  If the trained model predicts less than this much incoming solar power,
//  the device treats that as "not much sun coming soon" and pre-emptively
//  drops from NORMAL to POWER_SAVE, even if the battery voltage still looks
//  fine right now. This does NOT force CRITICAL mode - CRITICAL (WiFi off)
//  still only triggers on actually measured low voltage, as a hard safety
//  floor that a possibly-wrong prediction should never be able to trigger.
//  Tune this number against your panel's real output range if needed.
// ============================================================================
#define LOW_PREDICTED_POWER_W 0.5

// ============================================================================
// SOLAR / MOUNT CONSTANTS (unchanged)
// ============================================================================
#define MOUNT_HEIGHT 50.0

// ============================================================================
// [NEW - v3.3] FLOOD ALERT THRESHOLD
// ----------------------------------------------------------------------------
//  Water level (cm) at which the device declares a flood condition and forces
//  NORMAL mode regardless of battery state or model prediction.
//
//  KEEP IN SYNC WITH THE DASHBOARD. The same threshold is configured
//  separately in the dashboard as NEXT_PUBLIC_FLOOD_THRESHOLD_CM (Next.js
//  version, .env.local) and FLOOD_THRESHOLD_CM (standalone HTML version).
//  Changing it here alone will make the device and dashboard disagree.
// ============================================================================
#define FLOOD_THRESHOLD_CM 35.0

// ============================================================================
// RELAY CONSTANTS (unchanged)
// ============================================================================
#define RELAY_OFF HIGH
#define RELAY_ON  LOW

// ============================================================================
// [NEW] TRAINED ENERGY PREDICTION MODEL
// ----------------------------------------------------------------------------
//  Linear regression trained offline in Python (scikit-learn):
//      E_pred = W0 + (W1 * light) + (W2 * hour) + (W3 * battery_voltage)
//  Test-set performance: R2 = 0.9475, MAE = 0.109 W
// ============================================================================
#define MODEL_W0  5.152782f
#define MODEL_W1  0.00059438f
#define MODEL_W2  0.001284f
#define MODEL_W3 -1.246830f

float predictEnergy(float lightRawValue, float hourOfDayValue, float batteryVoltageValue) {
    float pred = MODEL_W0
               + (MODEL_W1 * lightRawValue)
               + (MODEL_W2 * hourOfDayValue)
               + (MODEL_W3 * batteryVoltageValue);
    if (pred < 0) pred = 0;   // predicted harvest can never be negative
    return pred;
}

// ============================================================================
// EMONLIB CURRENT SENSOR (unchanged)
// ============================================================================
EnergyMonitor emon1;
const int CURRENT_SAMPLES = 1480;

// ============================================================================
// PUBLISHING INTERVALS (unchanged)
// ============================================================================
const unsigned long PUBLISH_NORMAL     = 5000;   // 5 seconds
const unsigned long PUBLISH_POWER_SAVE = 15000;  // 15 seconds
const unsigned long PUBLISH_CRITICAL   = 0;      // WiFi OFF (save to LittleFS)

// ============================================================================
// VARIABLES
// ============================================================================
float batteryVoltage = 0.0;
float batterySOC     = 0.0;
float solarVoltage   = 0.0;
float solarCurrent   = 0.0;
float solarPower     = 0.0;
float waterLevel     = 0.0;
int   lightRaw       = 0;
float distance       = 0.0;
bool  relayState     = false;

float  hourOfDay    = 12.0;       // [NEW] fallback until NTP syncs
float  predictedPower = 0.0;      // [NEW] model output
String currentMode  = "NORMAL";   // [NEW] published mode label
bool   floodAlert   = false;      // [NEW - v3.3] true while water >= FLOOD_THRESHOLD_CM

unsigned long lastRead    = 0;
unsigned long lastPublish = 0;
unsigned long publishInterval = PUBLISH_NORMAL;
const unsigned long READ_INTERVAL = 2000;   // Read every 2 seconds
bool wifiConnected = false;
bool criticalMode  = false;
bool timeSynced    = false;       // [NEW]

// ============================================================================
// [NEW - v3.2] CRITICAL MODE PERIODIC WAKE
// ----------------------------------------------------------------------------
//  Without this, CRITICAL mode turns WiFi off and never turns it back on
//  until the battery recovers on its own - meaning the dashboard goes
//  completely silent for as long as the battery stays low, which is exactly
//  when you most want to know the device is still alive and what the water
//  level is doing. Instead, every CRITICAL_WAKE_INTERVAL the device briefly
//  reconnects, sends one status update, then goes back to WiFi-off.
//  5 minutes, matching what was already decided in the report.
// ============================================================================
const unsigned long CRITICAL_WAKE_INTERVAL = 300000;   // 5 minutes
unsigned long lastCriticalWake = 0;

// ============================================================================
// [NEW - v3.4] OFFLINE BUFFER
// ----------------------------------------------------------------------------
//  Previously every buffered reading was written as its own file
//  (/data_<millis>.json). In CRITICAL mode that produced one new file every
//  2 seconds, roughly 1,800 files per hour, and uploading them meant one
//  HTTPS connection per file. Now all buffered readings append as lines to a
//  single file, one JSON object per line (.jsonl format).
//
//  MAX_BACKLOG_PER_PASS bounds how many buffered lines are uploaded in one
//  call, so catching up after an outage never blocks the main loop for
//  minutes at a time. Whatever is left stays in the file for the next pass.
// ============================================================================
const char* OFFLINE_LOG = "/offline_data.jsonl";
const char* OFFLINE_TMP = "/offline_tmp.jsonl";
const int   MAX_BACKLOG_PER_PASS = 5;
const size_t MAX_OFFLINE_LOG_BYTES = 200000;   // ~200 KB cap on the buffer

// ============================================================================
// FUNCTION: Convert ADC to Voltage (unchanged)
// ============================================================================
float adcToVoltage(int adcValue) {
    return (adcValue / ADC_RESOLUTION) * ADC_REF_VOLTAGE;
}

// ============================================================================
// FUNCTION: Calculate Battery State of Charge (unchanged)
// ============================================================================
float calculateSOC(float voltage) {
    if (voltage >= BATTERY_MAX_VOLTAGE) return 100.0;
    if (voltage <= BATTERY_MIN_VOLTAGE) return 0.0;
    return ((voltage - BATTERY_MIN_VOLTAGE) / (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE)) * 100.0;
}

// ============================================================================
// FUNCTION: Determine Battery State (unchanged)
// ============================================================================
String getBatteryState(float voltage) {
    if (voltage >= BATTERY_NORMAL_MIN) return "NORMAL";
    else if (voltage >= BATTERY_POWER_SAVE_MIN) return "POWER_SAVE";
    else return "CRITICAL";
}

// ============================================================================
// [NEW] FUNCTION: Get hour of day (0.00 - 23.99) from NTP
// ============================================================================
float getHourOfDay() {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo, 500)) {
        return hourOfDay;   // keep last known value if NTP not ready yet
    }
    timeSynced = true;
    return timeinfo.tm_hour + (timeinfo.tm_min / 60.0);
}

// ============================================================================
// FUNCTION: Update Publish Interval Based on Battery
//           [NEW] also records the mode label for publishing
//           [NEW - v3.1] now also considers the model's prediction, so mode
//           switching is prediction-driven rather than purely reactive.
//           IMPORTANT: this must run AFTER predictedPower has been computed
//           for this read cycle (see loop() - order was changed for this).
// ============================================================================
void updatePublishInterval() {
    String state;

    // [NEW - v3.3] FLOOD OVERRIDE - checked first, beats everything else.
    // waterLevel is -1.0 when the ultrasonic reading failed, so the
    // >= 0 guard stops a sensor error from being read as a flood.
    floodAlert = (waterLevel >= 0) && (waterLevel >= FLOOD_THRESHOLD_CM);

    if (floodAlert) {
        // Skip the battery check AND the prediction check entirely. Forcing
        // NORMAL here also means the CRITICAL branch below never runs, so
        // WiFi is never switched off while the water is high - the device
        // keeps reporting every 5 s even on a nearly flat battery.
        state = "NORMAL";
        Serial.print("FLOOD OVERRIDE: water level ");
        Serial.print(waterLevel, 1);
        Serial.print(" cm >= ");
        Serial.print(FLOOD_THRESHOLD_CM, 1);
        Serial.println(" cm - forcing NORMAL mode, ignoring battery and prediction");
    } else {
        state = getBatteryState(batteryVoltage);

        // [NEW - v3.1] Prediction override: battery still looks NORMAL right
        // now, but the model expects little solar power coming in soon - switch
        // to POWER_SAVE early instead of waiting for the battery to actually
        // drop. This is the "throttle before the battery is endangered" idea
        // the project is built around.
        if (state == "NORMAL" && predictedPower < LOW_PREDICTED_POWER_W) {
            state = "POWER_SAVE";
            Serial.print("Prediction override: predicted ");
            Serial.print(predictedPower, 2);
            Serial.println(" W is below threshold - switching to POWER_SAVE early");
        }
    }

    currentMode = state;   // [NEW]

    if (state == "NORMAL") {
        publishInterval = PUBLISH_NORMAL;
        criticalMode = false;
        lastCriticalWake = 0;   // [NEW - v3.2] reset so the next CRITICAL episode wakes immediately
        Serial.println("Battery NORMAL - Publishing every 5s");
    } else if (state == "POWER_SAVE") {
        publishInterval = PUBLISH_POWER_SAVE;
        criticalMode = false;
        lastCriticalWake = 0;   // [NEW - v3.2]
        Serial.println("Battery POWER SAVE - Publishing every 15s");
    } else {
        publishInterval = PUBLISH_CRITICAL;
        criticalMode = true;
        Serial.println("Battery CRITICAL - WiFi OFF (except brief periodic wakes), saving to LittleFS");

        if (WiFi.status() == WL_CONNECTED) {
            WiFi.disconnect(true);
            WiFi.mode(WIFI_OFF);
            wifiConnected = false;
            Serial.println("WiFi Disconnected (Critical Mode)");
        }
    }
}

// ============================================================================
// [NEW - v3.2] FUNCTION: Briefly wake WiFi during CRITICAL mode to send one
// status update, then return to WiFi-off. Called from loop() on a timer -
// see CRITICAL_WAKE_INTERVAL above.
// ============================================================================
void criticalWakeAndPublish() {
    Serial.println("CRITICAL wake window - reconnecting briefly to send status");

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        wifiConnected = true;
        Serial.println("CRITICAL wake: connected, publishing status");
        publishPendingData();          // now actually flushes - see v3.4 fix
        String payload = buildPayload();

        // [FIX - v3.4] The old version ignored the return value and always
        // printed "status sent", even when Firebase rejected the write, and
        // had no fallback - so a failed critical reading was lost entirely.
        if (sendToFirebase(payload, true)) {
            Serial.println("CRITICAL wake: status sent successfully");
        } else {
            Serial.println("CRITICAL wake: Firebase upload failed - buffering instead");
            saveToLittleFS();
        }
    } else {
        Serial.println("CRITICAL wake: could not reconnect - saving to LittleFS instead");
        saveToLittleFS();
    }

    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    wifiConnected = false;
    Serial.println("CRITICAL wake window over - WiFi off again");
}

// ============================================================================
// SENSOR READ FUNCTIONS (all unchanged)
// ============================================================================
float readBatteryVoltage() {
    int raw = analogRead(BATTERY_PIN);
    float voltage = adcToVoltage(raw);
    return (voltage / BATTERY_RATIO) * BATTERY_CALIBRATION;
}

float readSolarVoltage() {
    int raw = analogRead(SOLAR_VOLTAGE_PIN);
    float voltage = adcToVoltage(raw);
    return (voltage / SOLAR_RATIO) * SOLAR_CALIBRATION;
}

float readSolarCurrent() {
    double temp = emon1.calcIrms(CURRENT_SAMPLES);   // first reading discarded
    delay(100);
    double current = emon1.calcIrms(CURRENT_SAMPLES);
    if (current < 0.05) current = 0.0;
    return current;
}

int readLightRaw() {
    return analogRead(LDR_PIN);
}

float readDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(20);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 30000);
    float dist = duration * 0.034 / 2;

    if (dist <= 0 || dist > 400) {
        dist = 999.0;
    }
    return dist;
}

float calculateWaterLevel(float dist) {
    if (dist >= 999.0) return -1.0;
    float level = MOUNT_HEIGHT - dist;
    if (level < 0) level = 0;
    if (level > MOUNT_HEIGHT) level = MOUNT_HEIGHT;
    return level;
}

float calculatePower(float voltage, float current) {
    return voltage * current;
}

// ============================================================================
// FUNCTION: Update Relay (unchanged)
// ============================================================================
void updateRelay() {
    if (batteryVoltage >= BATTERY_MAX_VOLTAGE) {
        digitalWrite(RELAY_PIN, RELAY_OFF);
        relayState = false;
        Serial.println("RELAY OFF (Battery fully charged)");
    } else {
        digitalWrite(RELAY_PIN, RELAY_ON);
        relayState = true;
        Serial.println("RELAY ON (Battery charging)");
    }
}

// ============================================================================
// FUNCTION: Connect to WiFi (unchanged, plus [NEW] NTP start on success)
// ============================================================================
bool connectToWiFi() {
    if (criticalMode) return false;

    if (WiFi.status() == WL_CONNECTED) {
        wifiConnected = true;
        return true;
    }

    Serial.print("Connecting to WiFi");
    WiFi.begin(ssid, password);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        wifiConnected = true;
        Serial.println();
        Serial.println("WiFi Connected");
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());

        // [NEW] start NTP so the model has a real hour-of-day
        configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
        return true;
    } else {
        wifiConnected = false;
        Serial.println();
        Serial.println("WiFi Connection Failed");
        return false;
    }
}

// ============================================================================
// [NEW] FUNCTION: Build the JSON payload (used by Firebase and LittleFS)
// ============================================================================
String buildPayload() {
    String payload = "{";
    payload += "\"battery_voltage\":" + String(batteryVoltage, 2) + ",";
    payload += "\"battery_soc\":"     + String(batterySOC, 0) + ",";
    payload += "\"solar_voltage\":"   + String(solarVoltage, 2) + ",";
    payload += "\"solar_current\":"   + String(solarCurrent, 3) + ",";
    payload += "\"solar_power\":"     + String(solarPower, 1) + ",";
    payload += "\"water_level\":"     + String(waterLevel, 1) + ",";
    payload += "\"light_raw\":"       + String(lightRaw) + ",";
    payload += "\"relay_state\":"     + String(relayState ? 1 : 0) + ",";
    payload += "\"mode\":\""          + currentMode + "\",";           // [NEW]
    payload += "\"flood_alert\":"     + String(floodAlert ? "true" : "false") + ","; // [NEW - v3.3]
    payload += "\"predicted_power\":" + String(predictedPower, 2) + ","; // [NEW]
    payload += "\"hour\":"            + String(hourOfDay, 2) + ",";      // [NEW]
    payload += "\"timestamp\":"       + String((unsigned long)time(nullptr)); // [NEW]
    payload += "}";
    return payload;
}

// ============================================================================
// FUNCTION: Save to LittleFS (offline storage) - unchanged behaviour,
//           now stores the same enriched payload
// ============================================================================
void saveToLittleFS() {
    // [FIX - v3.4] Appends one line to a single file instead of creating a
    // new file per reading. See the OFFLINE BUFFER block above for why.
    String jsonString = buildPayload();

    File file = LittleFS.open(OFFLINE_LOG, FILE_APPEND);
    if (file) {
        file.println(jsonString);
        size_t sz = file.size();
        file.close();
        Serial.printf("Buffered offline (log now %u bytes)\n", (unsigned)sz);
    } else {
        Serial.println("Failed to write offline buffer");
    }
}

// ============================================================================
// [NEW] FUNCTION: Send one JSON string to Firebase
//   - PUT  /current.json  overwrites the "latest reading" the dashboard shows
//   - POST /history.json  appends a new entry for the history chart
// ============================================================================
bool sendToFirebase(const String& jsonPayload, bool updateCurrent) {
    if (WiFi.status() != WL_CONNECTED) return false;

    WiFiClientSecure secureClient;
    secureClient.setInsecure();   // prototype: skip certificate pinning
    HTTPClient https;
    bool ok = true;

    if (updateCurrent) {
        String currentUrl = String(firebaseHost) + "/current.json";
        if (https.begin(secureClient, currentUrl)) {
            https.addHeader("Content-Type", "application/json");
            int code = https.PUT(jsonPayload);

            // [FIX - v3.4] Previously this was "if (code <= 0) ok = false;".
            // HTTPClient returns NEGATIVE numbers for connection errors but
            // POSITIVE numbers for HTTP rejections (400, 401, 404, 500...).
            // The old check therefore treated a rejected write as a success,
            // and publishPendingData() would then delete the buffered file.
            // Only 2xx counts as success.
            String body = https.getString();   // must be read BEFORE end()
            if (code < 200 || code >= 300) {
                ok = false;
                Serial.printf("Firebase PUT failed - HTTP %d: %s\n", code, body.c_str());
            } else {
                Serial.printf("Firebase PUT ok - HTTP %d\n", code);
            }
            https.end();
        } else {
            ok = false;
            Serial.println("Firebase PUT failed - could not open connection");
        }
    }

    String historyUrl = String(firebaseHost) + "/history.json";
    if (https.begin(secureClient, historyUrl)) {
        https.addHeader("Content-Type", "application/json");
        int code = https.POST(jsonPayload);

        String body = https.getString();       // [FIX - v3.4] before end()
        if (code < 200 || code >= 300) {       // [FIX - v3.4] was code <= 0
            ok = false;
            Serial.printf("Firebase POST failed - HTTP %d: %s\n", code, body.c_str());
        } else {
            Serial.printf("Firebase POST ok - HTTP %d\n", code);
        }
        https.end();
    } else {
        ok = false;
        Serial.println("Firebase POST failed - could not open connection");
    }

    return ok;
}

// ============================================================================
// FUNCTION: Publish current reading (replaces publishToUbidots)
// ============================================================================
bool publishToFirebase() {
    if (criticalMode) {
        saveToLittleFS();
        return false;
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi not connected - saving to LittleFS");
        saveToLittleFS();
        return false;
    }

    String payload = buildPayload();
    Serial.print("Publishing: ");
    Serial.println(payload);

    if (sendToFirebase(payload, true)) {
        Serial.println("Data published to Firebase");
        return true;
    } else {
        Serial.println("Firebase publish failed - saving to LittleFS");
        saveToLittleFS();
        return false;
    }
}

// ============================================================================
// FUNCTION: Publish Pending LittleFS Data (now targets Firebase history)
// ============================================================================
void publishPendingData() {
    // [FIX - v3.4] The old version began with "if (criticalMode) return;",
    // which meant criticalWakeAndPublish() could never actually flush the
    // buffer despite its comment saying it did. Buffered readings simply
    // accumulated. The guard is gone. Instead the work is CAPPED at
    // MAX_BACKLOG_PER_PASS uploads per call, so this can never block the
    // main loop (or drain a critically low battery) for minutes at a time.
    if (WiFi.status() != WL_CONNECTED) return;
    if (!LittleFS.exists(OFFLINE_LOG)) return;

    File in = LittleFS.open(OFFLINE_LOG, FILE_READ);
    if (!in) return;

    File out = LittleFS.open(OFFLINE_TMP, FILE_WRITE);
    if (!out) {
        in.close();
        return;
    }

    int sent = 0;
    int kept = 0;
    bool stopSending = false;   // set on first failure - retry next pass

    while (in.available()) {
        String line = in.readStringUntil('\n');
        line.trim();
        if (line.length() == 0) continue;

        if (!stopSending && sent < MAX_BACKLOG_PER_PASS) {
            if (sendToFirebase(line, false)) {   // history only, not current
                sent++;
                delay(50);
            } else {
                stopSending = true;   // network unhappy, keep the rest
                out.println(line);
                kept++;
            }
        } else {
            out.println(line);        // over the cap, or after a failure
            kept++;
        }
    }

    in.close();
    out.close();

    LittleFS.remove(OFFLINE_LOG);
    if (kept > 0) {
        LittleFS.rename(OFFLINE_TMP, OFFLINE_LOG);
    } else {
        LittleFS.remove(OFFLINE_TMP);   // buffer fully drained
    }

    if (sent > 0 || kept > 0) {
        Serial.printf("Offline buffer: %d uploaded, %d still queued\n", sent, kept);
    }
}

// ============================================================================
// FUNCTION: Check Storage and Rotate Logs (unchanged)
// ============================================================================
void checkStorage() {
    // [FIX - v3.4] Rewritten for the single .jsonl buffer. The old version
    // deleted /data_*.json files, which no longer exist. If the buffer grows
    // past MAX_OFFLINE_LOG_BYTES the OLDEST half is dropped and the newest
    // half kept, since recent readings matter more than stale ones.
    if (!LittleFS.exists(OFFLINE_LOG)) return;

    File f = LittleFS.open(OFFLINE_LOG, FILE_READ);
    if (!f) return;
    size_t sz = f.size();

    if (sz <= MAX_OFFLINE_LOG_BYTES) {
        f.close();
        return;
    }

    Serial.printf("Offline buffer %u bytes - trimming oldest half\n", (unsigned)sz);
    f.seek(sz / 2);
    f.readStringUntil('\n');   // discard the partial line at the seek point

    File out = LittleFS.open(OFFLINE_TMP, FILE_WRITE);
    if (!out) {
        f.close();
        return;
    }
    while (f.available()) {
        String line = f.readStringUntil('\n');
        line.trim();
        if (line.length() > 0) out.println(line);
    }
    f.close();
    out.close();

    LittleFS.remove(OFFLINE_LOG);
    LittleFS.rename(OFFLINE_TMP, OFFLINE_LOG);
    Serial.println("Offline buffer trimmed");
}

// ============================================================================
// [NEW - v3.4] FUNCTION: Delete legacy per-reading files from older firmware
// ----------------------------------------------------------------------------
//  Boards previously running v3.0-v3.3 may still hold hundreds or thousands
//  of /data_*.json files in flash. Nothing reads them any more, so they would
//  sit there occupying space forever. This clears them once at startup.
// ============================================================================
void cleanupLegacyFiles() {
    File root = LittleFS.open("/");
    if (!root) return;

    int deleted = 0;
    File file = root.openNextFile();
    while (file) {
        String filename = file.name();
        if (!filename.startsWith("/")) filename = "/" + filename;
        bool legacy = filename.startsWith("/data_") && filename.endsWith(".json");
        file.close();
        if (legacy) {
            LittleFS.remove(filename);
            deleted++;
        }
        file = root.openNextFile();
    }
    root.close();

    if (deleted > 0) {
        Serial.printf("Removed %d legacy /data_*.json files from older firmware\n", deleted);
    }
}

// ============================================================================
// FUNCTION: Print Serial Data (unchanged, plus mode + prediction lines)
// ============================================================================
void printSerialData() {
    Serial.println("========================================");
    Serial.println("     SOLAR WATER LEVEL MONITOR");
    Serial.println("========================================");
    Serial.print("Battery: ");
    Serial.print(batteryVoltage, 2);
    Serial.print(" V  (SOC: ");
    Serial.print(batterySOC, 0);
    Serial.print("%)  Mode: ");
    Serial.println(currentMode);
    Serial.print("Solar Voltage: ");
    Serial.print(solarVoltage, 2);
    Serial.println(" V");
    Serial.print("Solar Current: ");
    Serial.print(solarCurrent, 3);
    Serial.println(" A");
    Serial.print("Solar Power (measured): ");
    Serial.print(solarPower, 1);
    Serial.println(" W");
    Serial.print("Solar Power (predicted): ");        // [NEW]
    Serial.print(predictedPower, 2);
    Serial.println(" W");
    Serial.print("Hour of day: ");                     // [NEW]
    Serial.print(hourOfDay, 2);
    Serial.println(timeSynced ? " (NTP synced)" : " (fallback, NTP not ready)");
    Serial.print("Distance: ");
    Serial.print(distance, 1);
    Serial.println(" cm");
    Serial.print("Water Level: ");
    if (waterLevel >= 0) {
        Serial.print(waterLevel, 1);
        Serial.print(" cm");
        Serial.println(floodAlert ? "   *** FLOOD ALERT ***" : "");   // [NEW - v3.3]
    } else {
        Serial.println("ERROR");
    }
    Serial.print("Light Raw ADC: ");
    Serial.println(lightRaw);
    Serial.print("Relay: ");
    Serial.println(relayState ? "ON" : "OFF");
    Serial.print("WiFi: ");
    Serial.println(wifiConnected ? "ON" : "OFF");
    Serial.print("Interval: ");
    if (criticalMode) {
        Serial.println("OFF (Critical)");
    } else {
        Serial.print(publishInterval / 1000);
        Serial.println("s");
    }
    Serial.print("Storage: ");
    Serial.print(LittleFS.usedBytes());
    Serial.print("/");
    Serial.print(LittleFS.totalBytes());
    Serial.println(" bytes");
    Serial.println("========================================");
    Serial.println();
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n========================================");
    Serial.println("   SOLAR WATER LEVEL MONITOR");
    Serial.println("   Firebase + LittleFS + ML prediction");
    Serial.println("========================================\n");

    // Initialize LittleFS
    if (!LittleFS.begin()) {
        Serial.println("LittleFS Mount Failed! Formatting...");
        LittleFS.format();
        if (!LittleFS.begin()) {
            Serial.println("LittleFS Still Failed!");
        }
    }
    Serial.println("LittleFS Mounted Successfully");
    cleanupLegacyFiles();   // [NEW - v3.4] clear /data_*.json left by v3.0-v3.3

    // Initialize ADC
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    // Initialize pins
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(BATTERY_PIN, INPUT);
    pinMode(SOLAR_VOLTAGE_PIN, INPUT);
    pinMode(SOLAR_CURRENT_PIN, INPUT);
    pinMode(LDR_PIN, INPUT);
    pinMode(RELAY_PIN, OUTPUT);

    digitalWrite(RELAY_PIN, RELAY_OFF);
    relayState = false;
    digitalWrite(TRIG_PIN, LOW);

    // Initialize EmonLib
    emon1.current(SOLAR_CURRENT_PIN, CT_CALIBRATION);
    Serial.printf("EmonLib Initialized (Calibration: %.4f)\n", CT_CALIBRATION);
    delay(2000);

    // Connect to WiFi (also starts NTP)
    connectToWiFi();

    Serial.println("System Ready!");
    Serial.println("========================================\n");
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
    unsigned long currentMillis = millis();

    // ============================================================
    // READ SENSORS (Every 2 seconds)
    // ============================================================
    if (currentMillis - lastRead >= READ_INTERVAL) {
        batteryVoltage = readBatteryVoltage();
        batterySOC     = calculateSOC(batteryVoltage);

        lightRaw = readLightRaw();

        // [NEW] run the trained model on this reading
        // [NEW - v3.1] MOVED to run BEFORE updatePublishInterval(), because
        // the mode decision now needs predictedPower to already be known.
        hourOfDay      = getHourOfDay();
        predictedPower = predictEnergy((float)lightRaw, hourOfDay, batteryVoltage);

        // [NEW - v3.3] MOVED to run BEFORE updatePublishInterval() as well,
        // because the flood override needs waterLevel to already be known.
        distance   = readDistance();
        waterLevel = calculateWaterLevel(distance);

        updatePublishInterval();   // sets currentMode - flood- and prediction-aware

        if (criticalMode) {
            saveToLittleFS();
        }

        solarVoltage = readSolarVoltage();
        solarCurrent = readSolarCurrent();
        solarPower   = calculatePower(solarVoltage, solarCurrent);

        updateRelay();
        printSerialData();

        lastRead = currentMillis;
    }

    // ============================================================
    // MAINTAIN WIFI (if not critical)
    // ============================================================
    if (!criticalMode) {
        if (WiFi.status() != WL_CONNECTED) {
            connectToWiFi();
        }
    }

    // ============================================================
    // PUBLISH TO FIREBASE OR SAVE TO LITTLEFS
    // ============================================================
    if (!criticalMode && publishInterval > 0) {
        if (currentMillis - lastPublish >= publishInterval) {
            if (WiFi.status() == WL_CONNECTED) {
                publishPendingData();   // flush any offline backlog first
                publishToFirebase();
                checkStorage();
            } else {
                saveToLittleFS();
            }
            lastPublish = currentMillis;
        }
    } else if (criticalMode) {
        // [NEW - v3.2] Instead of staying silent for as long as the battery
        // stays low, wake briefly every CRITICAL_WAKE_INTERVAL and send one
        // status update so the dashboard never goes dark for more than that.
        if (lastCriticalWake == 0 || currentMillis - lastCriticalWake >= CRITICAL_WAKE_INTERVAL) {
            criticalWakeAndPublish();
            lastCriticalWake = currentMillis;
        }
    }

    delay(100);
}
