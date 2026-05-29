// ============================================================
//  HELTEC V2 — RECEPTOR LoRa ECO
//  Hace eco inmediato, muestra estado en pantalla OLED
// ============================================================

#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <U8g2lib.h>

#define LORA_SCK    5
#define LORA_MISO   19
#define LORA_MOSI   27
#define LORA_CS     18
#define LORA_RST    14
#define LORA_DIO0   26
#define LORA_FREQ   915E6

// OLED Heltec V2: SCL=15, SDA=4, RST=16
U8G2_SSD1306_128X64_NONAME_F_SW_I2C display(U8G2_R0, 15, 4, 16);

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

uint32_t pktsRecibidos = 0;
int      ultimoRSSI    = 0;
float    ultimoSNR     = 0;
uint16_t ultimoSeq     = 0;
bool     ultimoCRC     = false;

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
  display.drawStr(10, 14, "LORA RECEPTOR");
  display.setFont(u8g2_font_6x10_tr);
  display.drawStr(5, 32, "915 MHz | Heltec V2");
  display.drawStr(20, 50, "Esperando...");
  display.sendBuffer();
}

void pantallaEstado() {
  char buf[32];
  display.clearBuffer();
  display.setFont(u8g2_font_ncenB08_tr);
  display.drawStr(0, 12, "LORA RECEPTOR ECO");
  display.setFont(u8g2_font_6x10_tr);

  snprintf(buf, sizeof(buf), "Pkts : %lu", pktsRecibidos);
  display.drawStr(0, 26, buf);

  snprintf(buf, sizeof(buf), "RSSI : %d dBm", ultimoRSSI);
  display.drawStr(0, 38, buf);

  snprintf(buf, sizeof(buf), "SNR  : %.1f dB", ultimoSNR);
  display.drawStr(0, 50, buf);

  snprintf(buf, sizeof(buf), "CRC:%s seq:%u",
           ultimoCRC ? "OK " : "ERR", ultimoSeq);
  display.drawStr(0, 62, buf);

  display.sendBuffer();
}

// ── Setup ────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(500);

  pantallaInit();
  pantallaBienvenida();

  Serial.println("\n=== HELTEC V2 RECEPTOR LORA ECO ===");

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

  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();
  LoRa.receive();

  Serial.println("[LORA] Iniciado a 915 MHz OK");
  Serial.println("\nSetup completo. Esperando paquetes...\n");
}

// ── Loop — ECO INMEDIATO ─────────────────────────────────

void loop() {
  int pktSize = LoRa.parsePacket();
  if (pktSize == 0) return;

  uint8_t buf[sizeof(packet_t)];
  int n = 0;
  while (LoRa.available() && n < (int)sizeof(buf)) {
    buf[n++] = LoRa.read();
  }

  if (n < 8) return;

  packet_t* p = (packet_t*)buf;
  if (p->hdr != 0xA5) return;
  if (p->payload_size > 200) return;

  ultimoRSSI = LoRa.packetRssi();
  ultimoSNR  = LoRa.packetSnr();
  ultimoSeq  = p->seq;

  size_t L = wireLen(p->payload_size);

  uint16_t rxcrc, calc;
  memcpy(&rxcrc, buf + (L - sizeof(uint16_t)), sizeof(uint16_t));
  calc = crc16(buf, L - sizeof(uint16_t));
  ultimoCRC = (rxcrc == calc);

  // ECO INMEDIATO
  LoRa.beginPacket();
  LoRa.write(buf, L);
  LoRa.endPacket();
  LoRa.receive();

  pktsRecibidos++;
  pantallaEstado();

  Serial.printf("  [%03u] RECIBIDO  | ps=%u | CRC=%s | RSSI=%d | SNR=%.1f\n",
                p->seq, p->payload_size,
                ultimoCRC ? "OK" : "ERR", ultimoRSSI, ultimoSNR);
  Serial.printf("  [%03u] ECO ENVIADO\n", p->seq);
}