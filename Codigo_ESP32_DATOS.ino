#define MIC_PIN            34
#define FILTRO_MANUAL_PIN  35
#define SAMPLE_RATE        6000
#define PRINT_RATE         6000

float alphaHigh, alphaLow;

// Etapa 1 del paso alto
volatile float emaHigh = 0.0f;

// Dos etapas en cascada para el paso bajo (orden 2)
volatile float emaLow1 = 0.0f;
volatile float emaLow2 = 0.0f;

// Dos etapas en cascada para el paso alto (orden 2)
volatile float emaHigh2 = 0.0f;

const float OFFSET_ADC     = 2048.0f;
volatile bool nuevaMuestra  = false;
volatile uint32_t contador  = 0;
const uint32_t IMPRIMIR_CADA = 1;

void IRAM_ATTR onTimer() {
  nuevaMuestra = true;
  contador++;
}

void setup() {
  Serial.begin(921600);
  analogSetAttenuation(ADC_11db);
  analogReadResolution(12);

  const float Ts = 1.0f / SAMPLE_RATE;
  const float wH = 2.0f * PI * 300.0f  * Ts;
  const float wL = 2.0f * PI * 3400.0f * Ts;
  alphaHigh = wH / (wH + 1.0f);
  alphaLow  = wL / (wL + 1.0f);

  // Warm-up
  float lecturaInicial = (float)analogRead(MIC_PIN) - OFFSET_ADC;
  emaHigh  = lecturaInicial;
  emaHigh2 = lecturaInicial;
  emaLow1  = 0.0f;
  emaLow2  = 0.0f;
  delay(300);

  hw_timer_t* timer = timerBegin(SAMPLE_RATE);
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, 1, true, 0);
}

void loop() {
  if (!nuevaMuestra) return;
  nuevaMuestra = false;

  float raw    = (float)analogRead(MIC_PIN);
  float manual = (float)analogRead(FILTRO_MANUAL_PIN);

  float rawCentrado    = raw    - OFFSET_ADC;
  float manualCentrado = manual - OFFSET_ADC;

  // ── Filtro EMA pasa-banda orden 2 ────────────────────────────────────────
  // Paso alto — etapa 1
  emaHigh += alphaHigh * (rawCentrado - (float)emaHigh);
  float highPassed1 = rawCentrado - (float)emaHigh;

  // Paso alto — etapa 2 (cascada → orden 2, pendiente -40 dB/década)
  emaHigh2 += alphaHigh * (highPassed1 - (float)emaHigh2);
  float highPassed2 = highPassed1 - (float)emaHigh2;

  // Paso bajo — etapa 1
  emaLow1 += alphaLow * (highPassed2 - (float)emaLow1);

  // Paso bajo — etapa 2 (cascada → orden 2, pendiente -40 dB/década)
  emaLow2 += alphaLow * ((float)emaLow1 - (float)emaLow2);

  float emaFinal = (float)emaLow2;

  // Enviar las 3 señales — Python hace el análisis
  Serial.print((int)rawCentrado);
  Serial.print(',');
  Serial.print((int)manualCentrado);
  Serial.print(',');
  Serial.println((int)emaFinal);
}