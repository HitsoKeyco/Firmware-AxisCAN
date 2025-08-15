// ===============================================================
// ESP32 Logger v3.4 - Corregido y Listo para Compilar
//
// CARACTERÍSTICAS:
// - Construye el objeto OBD en el momento, asignando 'null' a cada PID que falla.
// - La estructura del JSON es siempre consistente.
// - Mantiene la lógica de estados, recolección inteligente y reintentos.
// - Simulación actualizada para probar la nueva estructura de datos.
// ===============================================================

#include <SPI.h>
#include <SD.h>
#include <Adafruit_GPS.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <mcp_can.h>
#include <math.h>

// ---------- CONFIG PINS ----------
#define SD_CS       33
#define CAN_CS      14
#define CAN_INT     27

// ---> MODO DE SIMULACIÓN <---
// Usar 'true' para probar sin un coche. El código simulará datos o fallos.
// Usar 'false' para el despliegue final en el vehículo.
#define USE_SIMULATION_MODE true

#if USE_SIMULATION_MODE
  // Cambiar a 'true' para simular un fallo total (todos los PIDs serán 'null')
  // Cambiar a 'false' para simular datos OBD correctos (con valores aleatorios)
  #define SIMULATE_OBD_FAILURE true
#endif

// ---------- CONSTANTES DE FUNCIONAMIENTO ----------
const double MIN_MOVE_DISTANCE_M = 20.0;
const long FLUSH_RETRY_INTERVAL_MS = 30000;
const unsigned long OBD_PID_TIMEOUT_MS = 75; // Timeout por cada PID

// ---------- WIFI / SERVER ----------
const char* ssid     = "Base";
const char* password = "042116361";
const char* serverUrl = "http://localhost:port/api/v1/telemetria_axiscan";

// ---------- BASE (geocerca) ----------
const double BASE_LAT = -2.09407;
const double BASE_LON = -79.92911;
const double BASE_RADIUS_M = 60.0;

// ---------- MÁQUINA DE ESTADOS DEL SISTEMA ----------
enum SystemState { STATE_COLLECTING, STATE_FLUSHING_DATA, STATE_IN_BASE_IDLE };
SystemState currentState = STATE_COLLECTING;

// ---------- OBD/CAN ----------
MCP_CAN CAN(CAN_CS);
bool canReady = false;

// ---------- GPS, BATCH & SD ----------
Adafruit_GPS GPS(&Serial1);
uint32_t gpsTimer = 0;
const int BATCH_SIZE = 10;
StaticJsonDocument<4096> batchDoc;
JsonArray batchArray;
int readingCount = 0;
const char* LOG_PATH = "/telemetry.jsonl";
static double lastLoggedLat = 0.0;
static double lastLoggedLon = 0.0;
unsigned long flushRetryTimer = 0;

// --- Prototipos ---
void collectAndStoreData(double lat, double lon);
void buildObdJson(JsonObject obd);
void simulateObdJson(JsonObject obd);
bool leerRespuestaPID(uint8_t pid, float &valor);
void solicitarPID(uint8_t pid);
void csIdleAll();
bool initSD();
bool initCAN();
double deg2rad(double d);
double haversine_m(double lat1, double lon1, double lat2, double lon2);
bool isInBase(double lat, double lon);
bool ensureWiFi();
bool appendLineToSD(const String &line);
bool clearSDFile();
bool postJson(const String& jsonPayload);
bool flushBatchRAM();
bool flushSDFile();

// ===============================================================
// SETUP
// ===============================================================
void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("\n=== Logger v3.4 - Compilación Corregida ===");
  #if USE_SIMULATION_MODE
    Serial.println("!!! MODO SIMULACIÓN ACTIVADO !!!");
    #if SIMULATE_OBD_FAILURE
      Serial.println("--- Simulación de FALLO OBD activada (PIDs serán 'null') ---");
    #else
      Serial.println("--- Simulación de DATOS OBD activada ---");
    #endif
    randomSeed(analogRead(0));
  #endif
  
  csIdleAll(); SPI.begin(); delay(50); initSD(); initCAN();
  Serial1.begin(9600, SERIAL_8N1, 16, 17); GPS.begin(9600); GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); GPS.sendCommand(PGCMD_ANTENNA); delay(1000);
  batchArray = batchDoc.to<JsonArray>();
  Serial.println("Listo. Estado inicial: RECOLECTANDO");
}

// ===============================================================
// LOOP
// ===============================================================
void loop() {
  GPS.read();
  if (GPS.newNMEAreceived() && !GPS.parse(GPS.lastNMEA())) { return; }
  
  if (millis() - gpsTimer > 5000) {
    gpsTimer = millis();

    #if !USE_SIMULATION_MODE
      if (!GPS.fix) { Serial.println("Esperando fix de GPS..."); return; }
    #endif

    double lat = GPS.latitudeDegrees;
    double lon = GPS.longitudeDegrees;
    
    bool isCurrentlyInBase = isInBase(lat, lon);

    if (currentState == STATE_COLLECTING) {
      if (isCurrentlyInBase) {
        Serial.println("📍 Detectado DENTRO de la base. Iniciando proceso de descarga...");
        collectAndStoreData(lat, lon);
        currentState = STATE_FLUSHING_DATA;
        flushRetryTimer = millis() - FLUSH_RETRY_INTERVAL_MS - 1;
        Serial.println("--> Estado cambiado a: DESCARGANDO DATOS");
      } else {
        double distFromLast = (lastLoggedLat == 0.0) ? 999 : haversine_m(lat, lon, lastLoggedLat, lastLoggedLon);
        if (distFromLast >= MIN_MOVE_DISTANCE_M) {
          Serial.printf("🚗 Movimiento detectado (%.1f m). Recolectando datos...\n", distFromLast);
          collectAndStoreData(lat, lon);
          lastLoggedLat = lat; lastLoggedLon = lon;
        }
      }
    } 
    else if (currentState == STATE_FLUSHING_DATA) {
      if (!isCurrentlyInBase) {
        Serial.println("Vehículo ha salido de la base durante la descarga. Reanudando recolección...");
        currentState = STATE_COLLECTING;
        return;
      }
      if (millis() - flushRetryTimer > FLUSH_RETRY_INTERVAL_MS) {
        Serial.println("Intentando enviar datos pendientes desde la base...");
        if (flushBatchRAM() && flushSDFile()) {
          Serial.println("✅ Todos los datos han sido enviados exitosamente.");
          currentState = STATE_IN_BASE_IDLE;
          Serial.println("--> Estado cambiado a: EN BASE (IDLE)");
        } else {
          Serial.printf("⚠️ Falló el envío. Se reintentará en %ld segundos...\n", FLUSH_RETRY_INTERVAL_MS / 1000);
        }
        flushRetryTimer = millis();
      }
    }
    else if (currentState == STATE_IN_BASE_IDLE) {
      if (!isCurrentlyInBase) {
        Serial.println("🚀 Detectado FUERA de la base. Reanudando recolección de datos.");
        lastLoggedLat = 0.0; lastLoggedLon = 0.0;
        currentState = STATE_COLLECTING;
        Serial.println("--> Estado cambiado a: RECOLECTANDO");
      } else {
        Serial.println("🏠 En la base, en espera (sin datos pendientes).");
      }
    }
  }
}

// ===============================================================
// IMPLEMENTACIÓN DE FUNCIONES
// ===============================================================
void collectAndStoreData(double lat, double lon) {
  StaticJsonDocument<1024> item;
  JsonObject gps = item.createNestedObject("gps");
  gps["fix"] = true;
  gps["lat"] = lat;
  gps["lng"] = lon;
  gps["alt"] = GPS.altitude;
  gps["speed_kmh"] = GPS.speed * 1.852;
  gps["satellites"] = GPS.satellites;
  char ts[30]; sprintf(ts, "%04d-%02d-%02dT%02d:%02d:%02dZ", 2000 + GPS.year, GPS.month, GPS.day, GPS.hour, GPS.minute, GPS.seconds); gps["timestamp"] = ts;

  JsonObject obd = item.createNestedObject("obd");
  #if USE_SIMULATION_MODE
    simulateObdJson(obd);
  #else
    buildObdJson(obd);
  #endif

  item["ts"] = millis();
  String line;
  serializeJson(item, line);
  
  Serial.println("--- JSON Generado ---");
  Serial.println(line);
  Serial.println("---------------------");

  if (!appendLineToSD(line)) { Serial.println("❌ Error al escribir SD"); }
  if (readingCount < BATCH_SIZE) {
    batchArray.add(item);
    readingCount++;
    Serial.printf("Batch RAM %d/%d.\n", readingCount, BATCH_SIZE);
  }
}

void solicitarPID(uint8_t pid) {
  if (!canReady) return;
  byte solicitud[8] = {0x02, 0x01, pid, 0,0,0,0,0};
  digitalWrite(SD_CS, HIGH);
  CAN.sendMsgBuf(0x7DF, 0, 8, solicitud);
}

bool leerRespuestaPID(uint8_t pid, float &valor) {
  if (!canReady) return false;
  unsigned long startTime = millis();
  while (millis() - startTime < OBD_PID_TIMEOUT_MS) {
  
    if (CAN.checkReceive() == CAN_MSGAVAIL) { 
      unsigned long id; byte len; byte rx[8];
      CAN.readMsgBuf(&id, &len, rx);
      if (id >= 0x7E8 && id <= 0x7EF && rx[1] == 0x41 && rx[2] == pid) {
        if (pid == 0x0C)      valor = ((rx[3] * 256) + rx[4]) / 4.0;
        else if (pid == 0x0D) valor = rx[3];
        else if (pid == 0x05) valor = rx[3] - 40;
        else if (pid == 0x11) valor = (rx[3] * 100.0) / 255.0;
        else if (pid == 0x2F) valor = (rx[3] * 100.0) / 255.0;
        else if (pid == 0x42) valor = ((rx[3] * 256) + rx[4]) / 1000.0;
        else { valor = NAN; return false; }
        return true;
      }
    }
  }
  return false;
}

void buildObdJson(JsonObject obd) {
  float valor;
  uint8_t pids[] = {0x0C, 0x0D, 0x05, 0x11, 0x2F, 0x42};
  const char* names[] = {"rpm", "speed", "coolant", "tps", "fuel_level", "voltage"};
  
  for (int i = 0; i < sizeof(pids)/sizeof(pids[0]); i++) {
    solicitarPID(pids[i]);
    if (leerRespuestaPID(pids[i], valor)) {
      obd[names[i]] = valor;
    } else {
      obd[names[i]] = nullptr;
    }
  }
}

void simulateObdJson(JsonObject obd) {
  #if SIMULATE_OBD_FAILURE
    obd["rpm"] = nullptr;
    obd["speed"] = nullptr;
    obd["coolant"] = nullptr;
    obd["tps"] = nullptr;
    obd["fuel_level"] = nullptr;
    obd["voltage"] = nullptr;
  #else
    obd["rpm"] = random(800, 3500);
    obd["speed"] = random(40, 90);
    obd["coolant"] = random(85, 95);
    obd["tps"] = random(15, 60);
    obd["fuel_level"] = random(30, 80);
    obd["voltage"] = 13.5 + (random(0, 10) / 10.0);
  #endif
}

void csIdleAll() { pinMode(SD_CS, OUTPUT); digitalWrite(SD_CS, HIGH); pinMode(CAN_CS, OUTPUT); digitalWrite(CAN_CS, HIGH); }
double deg2rad(double d) { return d * (M_PI / 180.0); }
double haversine_m(double lat1, double lon1, double lat2, double lon2) { const double R = 6371000.0; double dLat = deg2rad(lat2 - lat1); double dLon = deg2rad(lon2 - lon1); double a = sin(dLat/2)*sin(dLat/2) + cos(deg2rad(lat1))*cos(deg2rad(lat2))*sin(dLon/2)*sin(dLon/2); double c = 2 * atan2(sqrt(a), sqrt(1 - a)); return R * c; }
bool isInBase(double lat, double lon) { double d = haversine_m(lat, lon, BASE_LAT, BASE_LON); return d <= BASE_RADIUS_M; }
bool ensureWiFi() { if (WiFi.status() == WL_CONNECTED) return true; Serial.printf("Conectando WiFi a %s ...", ssid); WiFi.begin(ssid, password); uint32_t t0 = millis(); while (WiFi.status() != WL_CONNECTED && (millis() - t0) < 10000) { delay(250); Serial.print("."); } Serial.println(); if (WiFi.status() == WL_CONNECTED) { Serial.print("WiFi OK. IP: "); Serial.println(WiFi.localIP()); return true; } Serial.println("WiFi NO conectado."); return false; }
bool initSD() { digitalWrite(SD_CS, HIGH); if (!SD.begin(SD_CS)) { Serial.println("❌ SD FAIL"); return false; } Serial.println("✅ SD OK"); return true; }
bool appendLineToSD(const String &line) { digitalWrite(CAN_CS, HIGH); File f = SD.open(LOG_PATH, FILE_APPEND); if (!f) return false; f.println(line); f.close(); return true; }
bool clearSDFile() { if (SD.exists(LOG_PATH)) { return SD.remove(LOG_PATH); } return true; }
bool initCAN() { digitalWrite(CAN_CS, HIGH); delay(50); if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK) { CAN.setMode(MCP_NORMAL); Serial.println("✅ CAN OK (MCP2515 16MHz @500kbps)"); canReady = true; } else { Serial.println("❌ CAN init FAILED"); canReady = false; } return canReady; }
bool postJson(const String& jsonPayload) { if (!ensureWiFi()) return false; HTTPClient http; http.begin(serverUrl); http.addHeader("Content-Type", "application/json"); int code = http.POST(jsonPayload); http.end(); Serial.printf("HTTP POST a servidor, código: %d\n", code); return (code == 200 || code == 201); }
bool flushBatchRAM() { if (readingCount == 0) return true; String payload; serializeJson(batchDoc, payload); bool ok = postJson(payload); if (ok) { batchDoc.clear(); batchArray = batchDoc.to<JsonArray>(); readingCount = 0; } return ok; }
bool flushSDFile() { if (!SD.exists(LOG_PATH)) return true; File f = SD.open(LOG_PATH, FILE_READ); if (!f) return false; const size_t CHUNK = 30; bool allOk = true; while (f.available()) { StaticJsonDocument<8192> doc; JsonArray arr = doc.to<JsonArray>(); size_t count = 0; while (count < CHUNK && f.available()) { String line = f.readStringUntil('\n'); line.trim(); if (line.length() > 0) { StaticJsonDocument<1024> one; if (deserializeJson(one, line) == DeserializationError::Ok) { arr.add(one); count++; } } } if (count > 0) { String payload; serializeJson(doc, payload); if (!postJson(payload)) { allOk = false; break; } } } f.close(); if (allOk) { if (!clearSDFile()) Serial.println("⚠️ No se pudo borrar archivo SD"); } return allOk; }