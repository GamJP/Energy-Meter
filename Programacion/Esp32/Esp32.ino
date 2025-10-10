#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <SPI.h>

// =========================
// --- CONFIGURACIÓN WIFI --
#define AP_SSID "EnergyMeter"
#define AP_PASS ""                // sin clave; pon una si lo deseas
const IPAddress AP_IP(192,168,4,1);
const IPAddress AP_GATE(192,168,4,1);
const IPAddress AP_MASK(255,255,255,0);

// =========================
// --- PINES (ajusta a tu PCB) ---
#define PIN_SPI_SCLK   18
#define PIN_SPI_MISO   19
#define PIN_SPI_MOSI   23
#define PIN_ADC_CS      5
#define PIN_ADC_DRDY   27
#define PIN_STATUS_LED  2

// =========================
// --- ADC / ESCALA ---
static const float ADC_VREF      = 3.300f;  // Vref del ADC (ajústalo si usas 2.5 V, etc.)
static const float ADC_MID       = 1.650f;  // Offset de media (1.65 V)
static const float CURR_NUM      = 100.0f;  // numerador de tu factor
static const float CURR_DEN      = 16.0f;   // denominador de tu factor
static const uint8_t ADC_BITS    = 24;      // ADS1243: 24-bit
static const bool    ADC_BIPOLAR = false;   // medimos 0..Vref (tenemos offset de 1.65 V en analógico)
static const uint8_t ADC_CMD_RESET = 0x06;  // Reset command (ADS124x)
static const uint8_t ADC_CMD_RDATA = 0x12;  // Read Data command (ADS124x)

// =========================
// --- TIEMPOS ---
static const uint32_t SAMPLE_MS = 1000; // 1 muestra por segundo (requisito)

// =========================
// --- WEBSOCKET ---
WebSocketsServer webSocket(81);

// =========================
// --- BUFFER DE 100 MUESTRAS ---
struct Sample { float V; float I; };
static const size_t BUF_N = 100;
Sample ringBuf[BUF_N];
size_t  ringIdx = 0;
bool    filled  = false;
volatile bool dataRequested = false;

// =========================
// --- PROTOS ---
void wsEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);
void sendBufferAsJson();
bool  adcWaitDRDY(uint32_t timeout_ms);
void  adcBegin();
float adcReadVoltage_once();

// =========================
// --- SETUP ---
void setup() {
  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, LOW);

  // SPI
  pinMode(PIN_ADC_CS, OUTPUT);
  digitalWrite(PIN_ADC_CS, HIGH);
  pinMode(PIN_ADC_DRDY, INPUT);

  SPI.begin(PIN_SPI_SCLK, PIN_SPI_MISO, PIN_SPI_MOSI);

  // Inicia ADC
  adcBegin();

  // SoftAP para que el front-end se conecte a 192.168.4.1:81
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_GATE, AP_MASK);
  WiFi.softAP(AP_SSID, AP_PASS);
  delay(100);

  // WebSocket
  webSocket.begin();
  webSocket.onEvent(wsEvent);

  Serial.begin(115200);
  Serial.println("\nESP32 listo. AP en 192.168.4.1, WS en :81");
}

// =========================
// --- LOOP ---
void loop() {
  static uint32_t t_sample = 0;
  static uint32_t t_window = 0;
  static const uint16_t N_SAMPLES = 500; // 500 muestras ≈ 0.5s si SAMPLE_MS=1ms

  static float sumV2 = 0, sumI2 = 0;
  static uint16_t count = 0;

  webSocket.loop();

  // muestreo a alta frecuencia (1 ms → 1 kHz)
  if (millis() - t_sample >= 1) {
    t_sample = millis();

    // 1. Lectura instantánea del ADC
    float vout = adcReadVoltage_once();
    float current = ((vout - ADC_MID) * CURR_NUM) / CURR_DEN;

    // 2. Acumulación para RMS
    sumV2 += vout * vout;
    sumI2 += current * current;
    count++;

    // 3. Cada N_SAMPLES calcular RMS
    if (count >= N_SAMPLES) {
      float Vrms = sqrt(sumV2 / count);
      float Irms = sqrt(sumI2 / count);

      // Reinicia acumuladores
      sumV2 = 0;
      sumI2 = 0;
      count = 0;

      // Guarda los RMS en buffer
      ringBuf[ringIdx] = { Vrms, Irms };
      ringIdx = (ringIdx + 1) % BUF_N;
      if (ringIdx == 0) filled = true;

      // LED parpadeo
      digitalWrite(PIN_STATUS_LED, !digitalRead(PIN_STATUS_LED));

      Serial.printf("Vrms=%.3f V, Irms=%.3f A\n", Vrms, Irms);
    }
  }

  // 4. Envía datos si la app lo pide
  if (dataRequested) {
    sendBufferAsJson();
    dataRequested = false;
  }
}

// =========================
// --- WEBSOCKET HANDLER ---
void wsEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_CONNECTED) {
    Serial.printf("WS: cliente #%u conectado\n", num);
  } else if (type == WStype_TEXT) {
    String cmd = String((char*)payload, length);
    if (cmd == "REQUEST_DATA") {
      dataRequested = true;
    }
  }
}

// =========================
// --- ENVÍO JSON (sin ArduinoJson) ---
void sendBufferAsJson() {
  String json = "[";
  size_t count = filled ? BUF_N : ringIdx;
  // Empezamos por la muestra más vieja para que queden ordenadas en el tiempo
  size_t start = filled ? ringIdx : 0;
  for (size_t k = 0; k < count; ++k) {
    size_t idx = (start + k) % BUF_N;
    // La UI espera {V:..., I:...}
    json += "{\"V\":";
    json += String(ringBuf[idx].V, 6);
    json += ",\"I\":";
    json += String(ringBuf[idx].I, 6);
    json += "}";
    if (k + 1 < count) json += ",";
  }
  json += "]";
  webSocket.broadcastTXT(json);
}

// =========================
// --- INICIALIZACIÓN DEL ADC ---
void adcBegin() {
  // Pulso de RESET al ADC
  digitalWrite(PIN_ADC_CS, LOW);
  SPI.transfer(ADC_CMD_RESET);
  digitalWrite(PIN_ADC_CS, HIGH);
  delay(5);
}

// =========================
// --- ESPERA DRDY ===
bool adcWaitDRDY(uint32_t timeout_ms) {
  uint32_t t0 = millis();
  while (digitalRead(PIN_ADC_DRDY) != LOW) {
    if (millis() - t0 > timeout_ms) return false;
    delayMicroseconds(50);
  }
  return true;
}

// =========================
// --- LECTURA ÚNICA DEL ADC EN VOLTIOS ---
float adcReadVoltage_once() {
  // Espera dato listo
  if (!adcWaitDRDY(100)) {
    // Si no hay dato listo a tiempo, devuelve la última estimación cerca del offset
    return ADC_MID;
  }

  // En ADS124x, RDATA (0x12) seguido de la lectura de 3 bytes (24-bit)
  digitalWrite(PIN_ADC_CS, LOW);
  SPI.transfer(ADC_CMD_RDATA);

  uint32_t b1 = SPI.transfer(0x00);
  uint32_t b2 = SPI.transfer(0x00);
  uint32_t b3 = SPI.transfer(0x00);
  digitalWrite(PIN_ADC_CS, HIGH);

  // 24-bit raw
  uint32_t raw24 = (b1 << 16) | (b2 << 8) | (b3);

  // Conversión a voltios:
  const float denom = (float)( (1UL << ADC_BITS) - 1UL );  // 2^24 - 1
  float v = ( (float)raw24 / denom ) * ADC_VREF;

  return v;
}
