// ===== UNO R3 — Modbus RTU SLAVE via MAX485 =====
// RX  = D8 (AltSoftSerial RX)
// TX  = D9 (AltSoftSerial TX)
// RE+DE = D7 (Driver enable RS485)
// LED_RX = D4 (lampeggia su scrittura dal master)
// LED_TX = D5 (lampeggia quando aggiorno i registri ambiente)

#include <AltSoftSerial.h>
#include <ModbusRTU.h>

// ---------- Pin ----------
constexpr uint8_t PIN_RE_DE = 7;
constexpr uint8_t LED_RX    = 4;
constexpr uint8_t LED_TX    = 5;

// ---------- Parametri RTU ----------
constexpr uint32_t MB_BAUD    = 9600;
constexpr uint8_t  MB_SLAVEID = 1;     // deve combaciare con il JSON del gateway

// ---------- Indirizzi registri (coerenti con i JSON) ----------
constexpr uint16_t ADDR_ENV_BASE   = 0;   // temp float a 0..1, humidity a 2
constexpr uint16_t ADDR_FAN_BASE   = 20;  // fan_speed 20, fan_on 21

// ---------- Periodo aggiornamento "ambiente" ----------
constexpr uint32_t ENV_PERIOD_MS = 2000;

// ---------- Oggetti ----------
AltSoftSerial  ASerial;  // usa 8/9
ModbusRTU      mb;

// Buffer locale degli HREG che esponiamo (solo per confronto/LED)
uint16_t prevFanSpeed = 0;
uint16_t prevFanOn    = 0;

// Helpers
static void blink(uint8_t pin, uint16_t ms = 50) {
  digitalWrite(pin, HIGH);
  delay(ms);
  digitalWrite(pin, LOW);
}

// Converte float32 in due registri 16-bit (little-endian)
static void floatToRegsLE(float f, uint16_t& lo, uint16_t& hi) 
{
  union { float f; uint8_t b[4]; } u;
  u.f = f;
  // little endian → parola bassa = b0..b1, parola alta = b2..b3
  lo = (uint16_t) (u.b[0] | (uint16_t(u.b[1]) << 8));
  hi = (uint16_t) (u.b[2] | (uint16_t(u.b[3]) << 8));
}

// Per simulare dati "reali"
static float fakeTemperature() 
{
  // ~20.0 .. 30.0 °C con un po' di jitter
  return 20.0f + (millis() % 1000) / 100.0f + random(-5, 6) * 0.1f;
}
static uint16_t fakeHumidity() 
{
  // 40 .. 70 %RH
  return 40 + (millis()/1000) % 31;
}

void setup() 
{
  pinMode(PIN_RE_DE, OUTPUT);
  digitalWrite(PIN_RE_DE, LOW); // ricezione di default

  pinMode(LED_RX, OUTPUT);
  pinMode(LED_TX, OUTPUT);
  digitalWrite(LED_RX, LOW);
  digitalWrite(LED_TX, LOW);

  Serial.begin(115200);
  while (!Serial) {}
  Serial.println(F("\n[SLAVE] UNO R3 Modbus RTU via MAX485 (AltSoftSerial 8/9)"));

  // AltSoftSerial parte sempre a 8N1; ModbusRTU imposterà il formato
  ASerial.begin(MB_BAUD);

  // ModbusRTU: collega allo stream AltSoftSerial e pin RE/DE
  mb.begin(&ASerial, PIN_RE_DE);
  mb.slave(MB_SLAVEID);

  // Aggiungi Holding Registers come blocchi contigui
  // ENV: 3 registri da 0 a 2
  mb.addHreg(ADDR_ENV_BASE, 0, 3);
  // FAN_CMD: 2 registri 20..21
  mb.addHreg(ADDR_FAN_BASE, 0, 2);

  // Inizializza
  uint16_t lo, hi;
  floatToRegsLE(22.5f, lo, hi);
  mb.Hreg(ADDR_ENV_BASE + 0, lo);
  mb.Hreg(ADDR_ENV_BASE + 1, hi);
  mb.Hreg(ADDR_ENV_BASE + 2, 50);   // humidity 50 %

  mb.Hreg(ADDR_FAN_BASE + 0, 0);    // fan_speed
  mb.Hreg(ADDR_FAN_BASE + 1, 0);    // fan_on (0/1)

  prevFanSpeed = 0;
  prevFanOn    = 0;

  randomSeed(analogRead(A0));       // per la simulazione
  Serial.println(F("[SLAVE] pronto. Indirizzo=1, 9600 8N1"));
}

void loop() {
  // Servizio Modbus (risponde automaticamente a read/write del master)
  mb.task();

  // Rileva eventuali scritture del master sui comandi ventola (per LED RX/log)
  uint16_t curFanSpeed = mb.Hreg(ADDR_FAN_BASE + 0);
  uint16_t curFanOn    = mb.Hreg(ADDR_FAN_BASE + 1);
  if (curFanSpeed != prevFanSpeed || curFanOn != prevFanOn) 
  {
    // un WRITE del master ha cambiato i registri → LED_RX
    blink(LED_RX);
    prevFanSpeed = curFanSpeed;
    prevFanOn    = curFanOn;

    Serial.print(F("[SLAVE] FAN_CMD updated by master → speed="));
    Serial.print(curFanSpeed);
    Serial.print(F(" on="));
    Serial.println(curFanOn ? F("true") : F("false"));
  }

  // Aggiorna periodicamente i registri "ambiente"
  static uint32_t lastEnv = 0;
  uint32_t now = millis();

  if (now - lastEnv >= ENV_PERIOD_MS) 
  {
    lastEnv = now;

    float   t  = fakeTemperature();
    uint16_t h = fakeHumidity();

    uint16_t lo, hi;
    floatToRegsLE(t, lo, hi);
    mb.Hreg(ADDR_ENV_BASE + 0, lo);
    mb.Hreg(ADDR_ENV_BASE + 1, hi);
    mb.Hreg(ADDR_ENV_BASE + 2, h);

    // LED_TX per indicare che abbiamo “pronto” un nuovo dato da servire
    blink(LED_TX);

    Serial.print(F("[SLAVE] ENV update → T="));
    Serial.print(t, 2);
    Serial.print(F("C  H="));
    Serial.print(h);
    Serial.println(F("%"));
  }
}
