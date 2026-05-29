// ============================================================
//  ESP32A — EMISOR RTT
//  Comando 'R' → barre 20, 50, 200 bytes (200 paquetes c/u)
//  Pines:
//    XBee RX → GPIO 16 (TX1)   XBee TX → GPIO 17 (RX1)
//    SD CS   → GPIO 5
//    SDA → GPIO 21   SCL → GPIO 22
// ============================================================

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <Wire.h>
#include <RTClib.h>
#include <SPI.h>
#include <SdFat.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define XBEE_SERIAL     Serial1
#define XBEE_RX_PIN     17
#define XBEE_TX_PIN     16
#define SD_CS_PIN       5
#define LOG_FILE        "rtt_log.csv"
#define N_PACKETS       200
#define TIMEOUT_MS      3000UL

const uint8_t PAYLOADS[] = {20, 50, 200};
const uint8_t N_PAYLOADS = 3;
const char*   TIPO[]     = {"telemetria", "lidar_chico", "lidar_grande"};

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

RTC_DS1307 rtc;
SdFat sd;
bool sdDisponible  = false;
bool rtcDisponible = false;

SemaphoreHandle_t semSD;
SemaphoreHandle_t semXBee;

volatile bool iniciarBarrido = false;

// ── CRC16 ────────────────────────────────────────────────

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

void xbeeConfigurarCoordinador() {
  Serial.println("[XBEE] Configurando como Coordinador...");
  if (!xbeeEntrarModoAT()) {
    Serial.println("[XBEE] ADVERTENCIA: No entró al modo AT.");
  }
  xbeeEnviarComando("ATRE");
  delay(2000);
  xbeeEntrarModoAT();
  xbeeEnviarComando("ATCE1");
  xbeeEnviarComando("ATID1234");
  xbeeEnviarComando("ATDH0");
  xbeeEnviarComando("ATDLFFFF");
  xbeeEnviarComando("ATBD7");
  xbeeEnviarComando("ATWR");
  xbeeEnviarComando("ATCN");
  XBEE_SERIAL.begin(115200, SERIAL_8N1, XBEE_RX_PIN, XBEE_TX_PIN);
  Serial.println("[XBEE] Coordinador configurado a 115200 bps.");
}

// ── Tiempo ───────────────────────────────────────────────

String formatoFecha(DateTime dt) {
  char buf[20];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
           dt.year(), dt.month(), dt.day(),
           dt.hour(), dt.minute(), dt.second());
  return String(buf);
}

// ── SD ───────────────────────────────────────────────────

void borrarDatos() {
  if (!sdDisponible) return;
  if (sd.exists(LOG_FILE)) sd.remove(LOG_FILE);
  Serial.println("[SD] Archivo anterior borrado.");
}

void verificarCabecera() {
  if (!sd.exists(LOG_FILE)) {
    SdFile f;
    if (f.open(LOG_FILE, O_CREAT | O_WRITE)) {
      f.println("seq,payload_size,tipo,t_envio_us,t_retorno_us,rtt_us,latencia_us,perdido,rtc_ts");
      f.close();
      Serial.println("[SD] Archivo creado con cabecera.");
    }
  }
}

void guardarResultado(uint16_t seq, uint8_t ps, const char* tipo,
                      uint32_t t0, uint32_t t1, bool perdido,
                      const String& rtcTs) {
  if (!sdDisponible) return;
  if (xSemaphoreTake(semSD, pdMS_TO_TICKS(1000)) == pdTRUE) {
    SdFile f;
    if (f.open(LOG_FILE, O_CREAT | O_APPEND | O_WRITE)) {
      long rtt      = perdido ? -1 : (long)(t1 - t0);
      long latencia = perdido ? -1 : rtt / 2;
      char linea[130];
      snprintf(linea, sizeof(linea), "%u,%u,%s,%lu,%lu,%ld,%ld,%d,%s",
               seq, ps, tipo,
               (unsigned long)t0,
               perdido ? 0UL : (unsigned long)t1,
               rtt, latencia,
               perdido ? 1 : 0,
               rtcTs.c_str());
      f.println(linea);
      f.close();
    } else {
      Serial.println("[SD] ERROR al guardar.");
    }
    xSemaphoreGive(semSD);
  }
}

// ── Barrido RTT ──────────────────────────────────────────

void ejecutarBarrido() {
  String rtcTs = rtcDisponible ? formatoFecha(rtc.now()) : "RTC_NO_DISP";
  Serial.println("\n========================================");
  Serial.println("[BARRIDO] INICIO — " + rtcTs);
  Serial.println("========================================");

  for (uint8_t pi = 0; pi < N_PAYLOADS; pi++) {
    uint8_t     ps   = PAYLOADS[pi];
    const char* tipo = TIPO[pi];

    Serial.println("\n----------------------------------------");
    Serial.printf("[BLOQUE] %s | %d bytes | %d paquetes\n", tipo, ps, N_PACKETS);
    Serial.println("----------------------------------------");

    int enviados = 0, recibidos = 0, perdidos = 0;

    for (uint16_t i = 0; i < N_PACKETS; i++) {

      // Construir paquete
      packet_t p;
      memset(&p, 0, sizeof(p));
      p.hdr          = 0xA5;
      p.seq          = i;
      p.payload_size = ps;
      randomSeed(i);
      for (uint8_t k = 0; k < ps; k++) p.payload[k] = random(0, 256);

      // Asignar tiempo ANTES de calcular CRC
      uint32_t t0 = micros();
      p.t_emisor_us = t0;

      // Calcular CRC con todos los campos correctos
      size_t L = wireLen(ps);
      uint16_t crcVal = crc16((uint8_t*)&p, L - sizeof(uint16_t));
      memcpy((uint8_t*)&p + (L - sizeof(uint16_t)), &crcVal, sizeof(uint16_t));

      if (xSemaphoreTake(semXBee, pdMS_TO_TICKS(2000)) == pdTRUE) {

        // Limpiar buffer y enviar
        while (XBEE_SERIAL.available()) XBEE_SERIAL.read();
        XBEE_SERIAL.write((uint8_t*)&p, L);
        enviados++;

        Serial.printf("  [%03u] ENVIADO  | t0=%lu us | %d bytes | crc=%04X\n",
                      i, t0, (int)L, crcVal);

        // Esperar eco byte a byte
        packet_t r;
        uint8_t* rBuf    = (uint8_t*)&r;
        size_t   rLeidos = 0;
        bool     got     = false;
        uint32_t t1      = 0;

        // Esperar hdr 0xA5
        unsigned long start = millis();
        while (millis() - start < TIMEOUT_MS) {
          if (XBEE_SERIAL.available()) {
            uint8_t b = XBEE_SERIAL.read();
            if (b == 0xA5) {
              rBuf[0] = 0xA5;
              rLeidos = 1;
              break;
            }
          }
        }

        // Leer resto del paquete
        if (rLeidos == 1) {
          unsigned long t2 = millis();
          while (rLeidos < L && millis() - t2 < 500) {
            if (XBEE_SERIAL.available()) {
              rBuf[rLeidos++] = XBEE_SERIAL.read();
            }
          }
          if (rLeidos == L && r.seq == i) {
            t1  = micros();
            got = true;
          } else {
            Serial.printf("  [%03u] ECO INCOMPLETO | leidos=%u esperados=%u\n",
                          i, rLeidos, (unsigned)L);
          }
        }

        xSemaphoreGive(semXBee);

        if (got) {
          recibidos++;
          long rtt      = (long)(t1 - t0);
          long latencia = rtt / 2;
          Serial.printf("  [%03u] ECO OK   | t1=%lu us | RTT=%ld us | Latencia=%ld us\n",
                        i, t1, rtt, latencia);
        } else {
          perdidos++;
          Serial.printf("  [%03u] PERDIDO  | sin eco en %lu ms\n", i, TIMEOUT_MS);
        }

        guardarResultado(i, ps, tipo, t0, t1, !got, rtcTs);
      }

      delay(50);
    }

    float pdr = (float)recibidos / enviados * 100.0f;
    Serial.println("----------------------------------------");
    Serial.printf("[BLOQUE FIN] %s → Enviados:%d | Recibidos:%d | Perdidos:%d | PDR:%.1f%%\n",
                  tipo, enviados, recibidos, perdidos, pdr);
    Serial.println("----------------------------------------");
    delay(500);
  }

  Serial.println("\n========================================");
  Serial.println("[BARRIDO] COMPLETO. Presiona 'R' para repetir.");
  Serial.println("========================================\n");
}

// ── Tareas FreeRTOS ──────────────────────────────────────

void taskComandos(void* param) {
  Serial.println("[CMD] Presiona 'R' para iniciar barrido RTT.");
  for (;;) {
    if (Serial.available()) {
      char cmd = Serial.read();
      while (Serial.available()) Serial.read();
      if (cmd == 'R' || cmd == 'r') {
        Serial.println("[CMD] Barrido iniciado.");
        iniciarBarrido = true;
      } else {
        Serial.println("[CMD] Comando desconocido. Usa 'R'.");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void taskBarrido(void* param) {
  for (;;) {
    if (iniciarBarrido) {
      iniciarBarrido = false;
      ejecutarBarrido();
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

// ── Setup ────────────────────────────────────────────────

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== ESP32A EMISOR RTT ===");

  Wire.begin();
  if (!rtc.begin()) {
    Serial.println("[RTC] DS1307 NO encontrado");
  } else {
    rtcDisponible = true;
    if (!rtc.isrunning()) {
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      Serial.println("[RTC] Hora ajustada a compilación.");
    }
    Serial.println("[RTC] Hora actual: " + formatoFecha(rtc.now()));
  }

  xbeeConfigurarCoordinador();

  if (!sd.begin(SD_CS_PIN, SD_SCK_MHZ(4))) {
    Serial.println("[SD] SD no encontrada o error");
  } else {
    sdDisponible = true;
    Serial.println("[SD] Tarjeta SD OK");
    borrarDatos();
    verificarCabecera();
  }

  semSD   = xSemaphoreCreateMutex();
  semXBee = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(taskComandos, "Comandos", 2048, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(taskBarrido,  "Barrido",  8192, NULL, 1, NULL, 1);

  Serial.println("\nSetup completo. Presiona 'R' para iniciar.\n");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}