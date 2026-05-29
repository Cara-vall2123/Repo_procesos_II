// ============================================================
//  ESP32B — RECEPTOR ECO
//  Recibe paquete, hace eco inmediato, guarda log en SD
//  Pines:
//    XBee RX → GPIO 16 (TX1)   XBee TX → GPIO 17 (RX1)
//    SD CS   → GPIO 5
// ============================================================

#include <SPI.h>
#include <SdFat.h>

#define XBEE_SERIAL   Serial1
#define XBEE_RX_PIN   17
#define XBEE_TX_PIN   16
#define SD_CS_PIN     5
#define LOG_FILE      "receptor.csv"

#pragma pack(push, 1)
typedef struct {
  uint8_t  hdr;
  uint16_t seq;
  uint8_t  payload_size;
  uint32_t t_emisor_us;
  uint8_t  payload[200];
  uint16_t crc;
} packet_t;
#pragma pack(pop)

SdFat sd;
bool     sdDisponible  = false;
uint32_t pktsRecibidos = 0;

uint16_t crc16(const uint8_t* d, size_t n) {
  uint16_t c = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    c ^= d[i];
    for (int b = 0; b < 8; b++) c = (c & 1) ? (c >> 1) ^ 0xA001 : c >> 1;
  }
  return c;
}

size_t wireLen(uint8_t ps) {
  return offsetof(packet_t, payload) + ps + sizeof(uint16_t);
}

// ── Modo AT ──────────────────────────────────────────────

bool xbeeEntrarModoAT() {
  XBEE_SERIAL.begin(115200, SERIAL_8N1, XBEE_RX_PIN, XBEE_TX_PIN);
  delay(1200);
  XBEE_SERIAL.print("+++");
  delay(1200);
  String resp = "";
  unsigned long t = millis();
  while (millis() - t < 1000) {
    if (XBEE_SERIAL.available()) resp += (char)XBEE_SERIAL.read();
  }
  resp.trim();
  if (resp.indexOf("OK") >= 0) {
    Serial.println("[XBEE] Modo AT → OK (115200)");
    return true;
  }

  Serial.println("[XBEE] Probando a 9600...");
  XBEE_SERIAL.begin(9600, SERIAL_8N1, XBEE_RX_PIN, XBEE_TX_PIN);
  delay(1200);
  XBEE_SERIAL.print("+++");
  delay(1200);
  resp = "";
  t = millis();
  while (millis() - t < 1000) {
    if (XBEE_SERIAL.available()) resp += (char)XBEE_SERIAL.read();
  }
  resp.trim();
  if (resp.indexOf("OK") >= 0) {
    Serial.println("[XBEE] Modo AT → OK (9600)");
    return true;
  }

  Serial.println("[XBEE] Modo AT → SIN RESPUESTA");
  return false;
}

void xbeeEnviarComando(const char* cmd) {
  XBEE_SERIAL.println(cmd);
  delay(800);
  String resp = "";
  unsigned long t = millis();
  while (millis() - t < 500) {
    if (XBEE_SERIAL.available()) resp += (char)XBEE_SERIAL.read();
  }
  resp.trim();
  Serial.println("  [AT] " + String(cmd) + " → " + resp);
}

void xbeeConfigurarRouter() {
  Serial.println("[XBEE] Configurando como Router...");
  if (!xbeeEntrarModoAT()) {
    Serial.println("[XBEE] ADVERTENCIA: No entró al modo AT.");
  }
  xbeeEnviarComando("ATRE");
  delay(2000);
  xbeeEntrarModoAT();
  xbeeEnviarComando("ATCE0");
  xbeeEnviarComando("ATID1234");
  xbeeEnviarComando("ATDH0");
  xbeeEnviarComando("ATDLFFFF");
  xbeeEnviarComando("ATBD7");
  xbeeEnviarComando("ATWR");
  xbeeEnviarComando("ATCN");
  XBEE_SERIAL.begin(115200, SERIAL_8N1, XBEE_RX_PIN, XBEE_TX_PIN);
  Serial.println("[XBEE] Router configurado a 115200 bps.");
}

// ── SD ───────────────────────────────────────────────────

void verificarCabecera() {
  if (!sd.exists(LOG_FILE)) {
    SdFile f;
    if (f.open(LOG_FILE, O_CREAT | O_WRITE)) {
      f.println("seq,payload_size,t_rx_us,crc_ok,pkts_acumulados");
      f.close();
      Serial.println("[SD] Archivo creado con cabecera.");
    }
  }
}

void guardarLog(uint16_t seq, uint8_t ps, uint32_t t_rx, bool crcOk) {
  if (!sdDisponible) return;
  SdFile f;
  if (f.open(LOG_FILE, O_CREAT | O_APPEND | O_WRITE)) {
    char linea[80];
    snprintf(linea, sizeof(linea), "%u,%u,%lu,%d,%lu",
             seq, ps, (unsigned long)t_rx, crcOk ? 1 : 0, pktsRecibidos);
    f.println(linea);
    f.close();
  }
}

// ── Setup ────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== ESP32B RECEPTOR ECO ===");

  xbeeConfigurarRouter();

  if (!sd.begin(SD_CS_PIN, SD_SCK_MHZ(4))) {
    Serial.println("[SD] SD no encontrada o error");
  } else {
    sdDisponible = true;
    Serial.println("[SD] Tarjeta SD OK");
    verificarCabecera();
  }

  Serial.println("\nSetup completo. Esperando paquetes...\n");
}

// ── Loop — ECO INMEDIATO ─────────────────────────────────

void loop() {
  if (!XBEE_SERIAL.available()) return;

  // Esperar hdr 0xA5
  uint8_t hdr = XBEE_SERIAL.read();
  if (hdr != 0xA5) return;

  // Leer cabecera: seq(2) + payload_size(1) + t_emisor_us(4) = 7 bytes
  uint8_t cabBuf[7];
  size_t leidos = 0;
  unsigned long t = millis();
  while (leidos < 7 && millis() - t < 50) {
    if (XBEE_SERIAL.available()) cabBuf[leidos++] = XBEE_SERIAL.read();
  }
  if (leidos < 7) {
    Serial.println("[ECO] Cabecera incompleta, descartando.");
    return;
  }

  uint16_t seq          = cabBuf[0] | ((uint16_t)cabBuf[1] << 8);
  uint8_t  payload_size = cabBuf[2];
  uint32_t t_emisor     = cabBuf[3] | ((uint32_t)cabBuf[4] << 8) |
                          ((uint32_t)cabBuf[5] << 16) | ((uint32_t)cabBuf[6] << 24);

  if (payload_size > 200) {
    Serial.println("[ECO] payload_size inválido, descartando.");
    return;
  }

  // Leer payload + CRC
  size_t  restante = payload_size + sizeof(uint16_t);
  uint8_t restBuf[202];
  leidos = 0;
  t = millis();
  while (leidos < restante && millis() - t < 200) {
    if (XBEE_SERIAL.available()) restBuf[leidos++] = XBEE_SERIAL.read();
  }
  if (leidos < restante) {
    Serial.printf("[ECO] Payload incompleto seq=%u leidos=%u esperados=%u\n",
                  seq, leidos, (unsigned)restante);
    return;
  }

  uint32_t t_rx = micros();

  // Reconstruir paquete campo por campo
  packet_t p;
  memset(&p, 0, sizeof(p));
  p.hdr          = 0xA5;
  p.seq          = seq;
  p.payload_size = payload_size;
  p.t_emisor_us  = t_emisor;
  memcpy(p.payload, restBuf, payload_size);

  size_t L = wireLen(payload_size);

  // CRC recibido y verificación
  uint16_t rxcrc;
  memcpy(&rxcrc, restBuf + payload_size, sizeof(uint16_t));
  uint16_t calc = crc16((uint8_t*)&p, L - sizeof(uint16_t));
  bool crcOk = (rxcrc == calc);

  // Colocar CRC en paquete para el eco
  memcpy((uint8_t*)&p + (L - sizeof(uint16_t)), &rxcrc, sizeof(uint16_t));

  // ECO INMEDIATO
  XBEE_SERIAL.write((uint8_t*)&p, L);
  pktsRecibidos++;

  Serial.printf("  [%03u] RECIBIDO  | ps=%u | rxcrc=%04X calc=%04X | CRC=%s | t_rx=%lu us\n",
                seq, payload_size, rxcrc, calc, crcOk ? "OK" : "ERR", t_rx);
  Serial.printf("  [%03u] ECO ENVIADO\n", seq);

  // Log en SD después del eco
  guardarLog(seq, payload_size, t_rx, crcOk);
}