// ============================================================
//  HELTEC V2 — EMISOR LoRa RTT
//  Comando 'R' → barre SF7, SF9, SF12 × 20, 50, 200 bytes
//  Pantalla OLED U8g2
// ============================================================

#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <math.h>

// Pines LoRa Heltec V2
#define LORA_SCK    5
#define LORA_MISO   19
#define LORA_MOSI   27
#define LORA_CS     18
#define LORA_RST    14
#define LORA_DIO0   26
#define LORA_FREQ   915E6

// OLED Heltec V2: SCL=15, SDA=4, RST=16
U8G2_SSD1306_128X64_NONAME_F_SW_I2C display(U8G2_R0, 15, 4, 16);

#define N_PACKETS   200
#define TIMEOUT_MS  5000UL

const uint8_t PAYLOADS[]  = {20, 50, 200};
const uint8_t N_PAYLOADS  = 3;
const char*   TIPO[]      = {"telemetria", "lidar_ch", "lidar_gr"};
const char*   TIPO_FULL[] = {"telemetria", "lidar_chico", "lidar_grande"};
const uint8_t SF_LIST[]   = {7};
const uint8_t N_SF        = 1;

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

uint8_t  dispSF       = 7;
char     dispTipo[12] = "---";
uint16_t dispPkt      = 0;
float    dispRTT      = 0;
float    dispPDR      = 0;

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

// ── Pantalla ─────────────────────────────────────────────

void pantallaInit() {
  pinMode(16, OUTPUT);
  digitalWrite(16, LOW);
  delay(50);
  digitalWrite(16, HIGH);
  delay(50);
  display.begin();
}

void pantallaBienvenida() {
  display.clearBuffer();
  display.setFont(u8g2_font_ncenB08_tr);
  display.drawStr(10, 14, "LORA EMISOR RTT");
  display.setFont(u8g2_font_6x10_tr);
  display.drawStr(5, 32, "915 MHz | Heltec V2");
  display.drawStr(15, 50, "Presiona 'R'...");
  display.sendBuffer();
}

void pantallaEstado(uint16_t i) {
  char buf[32];
  display.clearBuffer();
  display.setFont(u8g2_font_ncenB08_tr);

  snprintf(buf, sizeof(buf), "SF%d | %s", dispSF, dispTipo);
  display.drawStr(0, 12, buf);

  display.setFont(u8g2_font_6x10_tr);

  snprintf(buf, sizeof(buf), "Pkt: %03u / %d", i, N_PACKETS);
  display.drawStr(0, 26, buf);

  snprintf(buf, sizeof(buf), "RTT: %.1f ms", dispRTT);
  display.drawStr(0, 40, buf);

  snprintf(buf, sizeof(buf), "PDR: %.1f %%", dispPDR);
  display.drawStr(0, 54, buf);

  display.sendBuffer();
}

void pantallaFin() {
  display.clearBuffer();
  display.setFont(u8g2_font_ncenB08_tr);
  display.drawStr(20, 25, "BARRIDO");
  display.drawStr(20, 45, "COMPLETO");
  display.sendBuffer();
}

// ── LoRa ─────────────────────────────────────────────────

void loraConfigurar(uint8_t sf) {
  LoRa.idle();
  LoRa.setSpreadingFactor(sf);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();
  delay(100);
  Serial.printf("[LORA] SF=%d BW=125kHz CR=4/5\n", sf);
}

// ── Estadísticas ─────────────────────────────────────────

void imprimirEstadisticas(const char* tipo, uint8_t ps, uint8_t sf,
                           int enviados, int recibidos, int perdidos,
                           long* rtts, int rttCount) {
  float pdr = (float)recibidos / enviados * 100.0f;

  if (rttCount == 0) {
    Serial.println("\n[BLOQUE] Sin paquetes recibidos.");
    return;
  }

  float media = 0;
  long  rttMin = 999999999L;
  long  rttMax = 0;
  for (int j = 0; j < rttCount; j++) {
    media += rtts[j];
    if (rtts[j] < rttMin) rttMin = rtts[j];
    if (rtts[j] > rttMax) rttMax = rtts[j];
  }
  media /= rttCount;

  float varianza = 0;
  for (int j = 0; j < rttCount; j++) {
    float diff = rtts[j] - media;
    varianza += diff * diff;
  }
  float desv = sqrt(varianza / rttCount);

  for (int a = 0; a < rttCount - 1; a++) {
    for (int b = 0; b < rttCount - a - 1; b++) {
      if (rtts[b] > rtts[b + 1]) {
        long tmp = rtts[b]; rtts[b] = rtts[b+1]; rtts[b+1] = tmp;
      }
    }
  }
  int   idxP95     = (int)(rttCount * 0.95f);
  long  p95        = rtts[idxP95];
  float latMedia_s = (media / 2.0f) / 1000000.0f;
  float throughput = (latMedia_s > 0) ? (ps * 8.0f) / latMedia_s : 0;

  Serial.println("\n========== ESTADÍSTICAS DEL BLOQUE ==========");
  Serial.printf("  Tipo              : %s\n", tipo);
  Serial.printf("  Payload           : %d bytes\n", ps);
  Serial.printf("  SF                : %d\n", sf);
  Serial.printf("  Enviados          : %d\n", enviados);
  Serial.printf("  Recibidos         : %d\n", recibidos);
  Serial.printf("  Perdidos          : %d\n", perdidos);
  Serial.printf("  PDR               : %.1f %%\n", pdr);
  Serial.println("  --- RTT ---");
  Serial.printf("  Mínimo            : %.2f ms\n", rttMin / 1000.0f);
  Serial.printf("  Máximo            : %.2f ms\n", rttMax / 1000.0f);
  Serial.printf("  Media             : %.2f ms\n", media / 1000.0f);
  Serial.printf("  Desv. estándar    : %.2f ms  (jitter)\n", desv / 1000.0f);
  Serial.printf("  Percentil 95      : %.2f ms\n", p95 / 1000.0f);
  Serial.println("  --- Latencia (RTT/2) ---");
  Serial.printf("  Media             : %.2f ms\n", (media / 2.0f) / 1000.0f);
  Serial.printf("  Percentil 95      : %.2f ms\n", (p95 / 2.0f) / 1000.0f);
  Serial.printf("  Throughput ef.    : %.1f bps\n", throughput);
  Serial.println("==============================================\n");
}

// ── Barrido RTT ──────────────────────────────────────────

void ejecutarBarrido() {
  Serial.println("\n========================================");
  Serial.println("[BARRIDO LORA] INICIO");
  Serial.println("========================================");

  for (uint8_t si = 0; si < N_SF; si++) {
    uint8_t sf = SF_LIST[si];
    dispSF = sf;
    loraConfigurar(sf);

    for (uint8_t pi = 0; pi < N_PAYLOADS; pi++) {
      uint8_t     ps        = PAYLOADS[pi];
      const char* tipo      = TIPO_FULL[pi];
      const char* tipoCorto = TIPO[pi];

      strncpy(dispTipo, tipoCorto, sizeof(dispTipo));
      dispPkt = 0;
      dispRTT = 0;
      dispPDR = 0;

      Serial.println("\n----------------------------------------");
      Serial.printf("[BLOQUE] SF%d | %s | %d bytes | %d paquetes\n",
                    sf, tipo, ps, N_PACKETS);
      Serial.println("----------------------------------------");

      int  enviados  = 0, recibidos = 0, perdidos = 0;
      long rtts[N_PACKETS];
      int  rttCount  = 0;

      for (uint16_t i = 0; i < N_PACKETS; i++) {

        packet_t p;
        memset(&p, 0, sizeof(p));
        p.hdr          = 0xA5;
        p.seq          = i;
        p.payload_size = ps;
        randomSeed(i);
        for (uint8_t k = 0; k < ps; k++) p.payload[k] = random(0, 256);

        uint32_t t0 = micros();
        p.t_emisor_us = t0;

        size_t   L      = wireLen(ps);
        uint16_t crcVal = crc16((uint8_t*)&p, L - sizeof(uint16_t));
        memcpy((uint8_t*)&p + (L - sizeof(uint16_t)), &crcVal, sizeof(uint16_t));

        LoRa.beginPacket();
        LoRa.write((uint8_t*)&p, L);
        LoRa.endPacket();
        enviados++;

        Serial.printf("  [%03u] ENVIADO  | SF%d | %d bytes\n", i, sf, (int)L);

        packet_t r;
        bool     got   = false;
        uint32_t t1    = 0;
        unsigned long start = millis();

        LoRa.receive();
        while (millis() - start < TIMEOUT_MS) {
          int pktSize = LoRa.parsePacket();
          if (pktSize > 0) {
            uint8_t buf[sizeof(packet_t)];
            int n = 0;
            while (LoRa.available() && n < (int)sizeof(buf)) {
              buf[n++] = LoRa.read();
            }
            memcpy(&r, buf, n);
            if (r.hdr == 0xA5 && r.seq == i) {
              t1  = micros();
              got = true;
              break;
            }
          }
        }

        if (got) {
          recibidos++;
          long rtt      = (long)(t1 - t0);
          long latencia = rtt / 2;
          rtts[rttCount++] = rtt;
          dispRTT = rtt / 1000.0f;
          Serial.printf("  [%03u] ECO OK   | RTT=%.2f ms | Lat=%.2f ms | RSSI=%d\n",
                        i, rtt / 1000.0f, latencia / 1000.0f, LoRa.packetRssi());
        } else {
          perdidos++;
          Serial.printf("  [%03u] PERDIDO  | sin eco en %lu ms\n", i, TIMEOUT_MS);
        }

        dispPkt = i + 1;
        dispPDR = (float)recibidos / (i + 1) * 100.0f;
        pantallaEstado(i + 1);

        if      (sf == 7)  delay(200);
        else if (sf == 9)  delay(1000);
        else               delay(3000);
      }

      imprimirEstadisticas(tipo, ps, sf, enviados, recibidos,
                           perdidos, rtts, rttCount);
      delay(2000);
    }
  }

  pantallaFin();
  Serial.println("\n========================================");
  Serial.println("[BARRIDO] COMPLETO. Presiona 'R' para repetir.");
  Serial.println("========================================\n");
}

// ── Setup ────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(500);

  pantallaInit();
  pantallaBienvenida();

  Serial.println("\n=== HELTEC V2 EMISOR LORA RTT ===");

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("[LORA] ERROR: no se pudo iniciar.");
    display.clearBuffer();
    display.setFont(u8g2_font_ncenB08_tr);
    display.drawStr(0, 30, "LORA ERROR!");
    display.sendBuffer();
    while (1);
  }

  loraConfigurar(7);
  Serial.println("[LORA] Iniciado a 915 MHz OK");
  Serial.println("\nSetup completo. Presiona 'R' para iniciar.\n");
}

// ── Loop ─────────────────────────────────────────────────

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    while (Serial.available()) Serial.read();
    if (cmd == 'R' || cmd == 'r') {
      Serial.println("[CMD] Barrido iniciado.");
      ejecutarBarrido();
    } else {
      Serial.println("[CMD] Comando desconocido. Usa 'R'.");
    }
  }
}