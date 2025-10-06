/*
  Proyecto: Medidor de Energía Eléctrica Inalámbrico
  Archivo: energy_meter_esp32.ino
  Autor: Juan Pablo Gamboa Durán
  Descripción:
  Firmware para el ESP32 que adquiere tensión y corriente RMS,
  las almacena en memoria y las transmite vía WebSocket al
  aplicativo web cuando este lo solicita.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <SPI.h>
#include <math.h>

// =========================
// === PARÁMETROS DEL ADC ===
// =========================
#define NUM_VALUES      100
#define SAMPLE_INTERVAL 1000   // 1 segundo
#define ADC_VREF        3.3
#define ADC_RESOLUTION  8388608.0

// Pines del ADC y SPI
#define CS_PIN    5
#define DRDY_PIN  32
#define SCLK_PIN  18
#define MISO_PIN  19
#define MOSI_PIN  23

// Pines de control y alimentación
#define POWER_CTRL  12
#define STATUS_LED  2

// ==========================
// === CONFIGURACIÓN Wi-Fi ===
// ==========================
const char* ssid = "EnergyMeter_AP";
const char* password = "12345678";
IPAddress local_ip(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

WebSocketsServer webSocket(81);

// ==========================
// === VARIABLES GLOBALES ===
// ==========================
float voltageBuffer[NUM_VALUES];
float currentBuffer[NUM_VALUES];
int bufferIndex = 0;
unsigned long lastSampleTime = 0;
bool dataRequested = false;

// ==========================
// === DECLARACIÓN FUNCIONES ===
// ==========================
float readADS1242Voltage();
float readADS1242Current();
void storeMeasurements(float voltage, float current);
void sendStoredData();
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length);

// ==========================
// === CONFIGURACIÓN INICIAL ===
// ==========================
void setup() {
  Serial.begin(115200);
  SPI.begin(SCLK_PIN, MISO_PIN, MOSI_PIN);

  pinMode(CS_PIN, OUTPUT);
  pinMode(DRDY_PIN, INPUT);
  pinMode(POWER_CTRL, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);

  digitalWrite(CS_PIN, HIGH);
  digitalWrite(POWER_CTRL, HIGH);
  digitalWrite(STATUS_LED, LOW);

  // Crear red Wi-Fi propia (modo Access Point)
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  WiFi.softAP(ssid, password);
  Serial.println("====================================");
  Serial.println("Red Wi-Fi creada exitosamente!");
  Serial.print("Nombre de red (SSID): ");
  Serial.println(ssid);
  Serial.print("Contraseña: ");
  Serial.println(password);
  Serial.print("IP del ESP32: ");
  Serial.println(WiFi.softAPIP());
  Serial.println("====================================");

  // Inicializar servidor WebSocket
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  Serial.println("Sistema inicializado correctamente.");
}

// ==========================
// === BUCLE PRINCIPAL ===
// ==========================
void loop() {
  webSocket.loop();

  unsigned long currentMillis = millis();
  if (currentMillis - lastSampleTime >= SAMPLE_INTERVAL) {
    lastSampleTime = currentMillis;

    if (digitalRead(DRDY_PIN) == LOW) {
      float vRMS = readADS1242Voltage();
      float iRMS = readADS1242Current();

      storeMeasurements(vRMS, iRMS);
      digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));

      Serial.printf("V: %.3f V | I: %.3f A\n", vRMS, iRMS);
    }
  }

  if (dataRequested) {
    sendStoredData();
    dataRequested = false;
  }
}

// ==========================
// === LECTURA DE ADC ===
// ==========================
float readADS1242Voltage() {
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(5);
  uint8_t msb = SPI.transfer(0x00);
  uint8_t mid = SPI.transfer(0x00);
  uint8_t lsb = SPI.transfer(0x00);
  digitalWrite(CS_PIN, HIGH);

  int32_t raw = ((int32_t)msb << 16) | ((int32_t)mid << 8) | lsb;
  if (raw & 0x800000) raw |= 0xFF000000;
  float voltage = (raw * ADC_VREF / ADC_RESOLUTION);
  return fabs(voltage) * 0.707; // Conversión RMS
}

float readADS1242Current() {
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(5);
  uint8_t msb = SPI.transfer(0x00);
  uint8_t mid = SPI.transfer(0x00);
  uint8_t lsb = SPI.transfer(0x00);
  digitalWrite(CS_PIN, HIGH);

  int32_t raw = ((int32_t)msb << 16) | ((int32_t)mid << 8) | lsb;
  if (raw & 0x800000) raw |= 0xFF000000;
  float current = (raw * ADC_VREF / ADC_RESOLUTION) / 0.01; // 0.01Ω shunt
  return fabs(current) * 0.707; // Conversión RMS
}

// ==========================
// === ALMACENAMIENTO ===
// ==========================
void storeMeasurements(float voltage, float current) {
  voltageBuffer[bufferIndex] = voltage;
  currentBuffer[bufferIndex] = current;
  bufferIndex = (bufferIndex + 1) % NUM_VALUES;
}

// ==========================
// === ENVÍO DE DATOS ===
// ==========================
void sendStoredData() {
  String json = "[";
  for (int i = 0; i < NUM_VALUES; i++) {
    json += "{\"V\":" + String(voltageBuffer[i], 3) +
            ",\"I\":" + String(currentBuffer[i], 3) + "}";
    if (i < NUM_VALUES - 1) json += ",";
  }
  json += "]";
  webSocket.broadcastTXT(json);
  Serial.println("Datos enviados al cliente.");
}

// ==========================
// === EVENTOS WEBSOCKET ===
// ==========================
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  if (type == WStype_CONNECTED) {
    Serial.println("Cliente conectado.");
  } 
  else if (type == WStype_TEXT) {
    String command = String((char *)payload);
    if (command == "REQUEST_DATA") {
      dataRequested = true;
    }
  }
}
