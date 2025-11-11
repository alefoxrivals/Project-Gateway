#include <Arduino.h>
#include <Arduino_CAN.h>
#include "utils.h"
#include "sd_manager.h"
#include "can_manager.h"
#include "modbus_manager.h"
#include "mapping.h"

// ===== SD paths =====
constexpr uint8_t PIN_SD_CS   = 10;
constexpr char    CAN_PATH[]  = "/CAN~1.JSO";
constexpr char    MB_PATH[]   = "/MODBUS~1.JSO";
constexpr char    MAP_PATH[]  = "/MAPPIN~1.JSO";

// ===== runtime config =====
long g_canBitrate = 500000;
std::vector<CanMessageSpec>     g_canMsgs;
ModbusRtuConfig                 g_rtu;
std::vector<ModbusResourceSpec> g_mbRes;
std::vector<MappingRule>        g_rules;

// Per il polling MB2CAN: manteniamo un last_ms per ogni risorsa coinvolta
struct PollState {
  const ModbusResourceSpec* res;
  uint32_t last_ms = 0;
};
std::vector<PollState> g_pollers;

static void buildPollers() {
  // Inserisce una voce per ogni risorsa Modbus usata in regole MB2CAN (una sola volta)
  for (auto& r : g_rules) 
  {
    if (r.dir != RuleDir::MB2CAN || !r.fromModbus) 
    {
      continue;
    }
    bool already=false;
    for (auto& p : g_pollers) 
    {
      if (p.res == r.fromModbus) 
      { 
        already=true; 
        break; 
      }
    }

    if (!already) 
    {
      g_pollers.push_back({ r.fromModbus, 0 });
    }
  }
}

void setup() 
{
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println(F("\n=== Gateway definitivo: CAN <-> Modbus via JSON ==="));

  // SD
  if (!SDM_begin(PIN_SD_CS)) 
  { 
    Serial.println(F("[SD] init FAIL")); 
    while(true){} 
  }

  // Load CAN
  String canJson;
  if (!SDM_readText(CAN_PATH, canJson)) 
  { 
    Serial.println(F("[SD] can.json missing")); 
    while(true){} 
  }
  if (!parseCanJson(canJson, g_canBitrate, g_canMsgs)) 
  { 
    Serial.println(F("[JSON] can FAIL")); 
    while(true){} 
  }
  Serial.print(F("[CFG] CAN bitrate=")); 
  Serial.println(g_canBitrate);

  // Load Modbus
  String mbJson;
  if (!SDM_readText(MB_PATH, mbJson)) 
  { 
    Serial.println(F("[SD] modbus.json missing")); 
    while(true){} 
  }

  if (!parseModbusJson(mbJson, g_rtu, g_mbRes)) 
  { 
    Serial.println(F("[JSON] modbus FAIL")); 
    while(true){} 
  }

  Serial.print(F("[CFG] MB RTU baud=")); 
  Serial.print(g_rtu.baud);
  Serial.print(F(" slave=")); 
  Serial.println(g_rtu.slave_id);

  // Load Mapping
  String mapJson;
  if (!SDM_readText(MAP_PATH, mapJson)) 
  { 
    Serial.println(F("[SD] mapping.json missing"));
    while(true){} 
  }
  if (!parseMappingJson(mapJson, g_mbRes, g_canMsgs, g_rules)) 
  { 
    Serial.println(F("[JSON] mapping FAIL")); 
    while(true){} 
  }
  Serial.print(F("[CFG] rules=")); 
  Serial.println((int)g_rules.size());

  // Init CAN
  if (!CANM::begin(g_canBitrate)) 
  { 
    Serial.println(F("[CAN] init FAIL")); 
    while(true){} 
  }
  Serial.println(F("[CAN] init OK"));

  // Init ModbusMaster (DE/RE su D7)
  if (!MBM::begin(g_rtu, 7)) 
  { 
    Serial.println(F("[MB] init FAIL")); 
    while(true){} 
  }
  Serial.println(F("[MB] init OK"));

  // Prepara pollers
  buildPollers();
}

// buffer temporanei per registri
static uint16_t regsBuf[16]; // sufficiente per i nostri esempi (aumenta se serve)

void loop() {
  // ========= RX CAN → Modbus (CAN2MB) =========
  if (CAN.available()) 
  {
    CanMsg rx = CAN.read();
    CANM::prettyPrintRx(g_canMsgs, rx);

    for (auto& rule : g_rules) 
    {
      if (rule.dir != RuleDir::CAN2MB || !rule.fromCan || !rule.toModbus) 
      {
        continue;
      }
      if (rule.fromCan->id != rx.id)
      {
        continue;
      }

      uint16_t outCount=0;
      if (extractModbusFromCan(rule, rx.data, rx.data_length, regsBuf, outCount)) 
      {
        if (!MBM::writeResource(*rule.toModbus, regsBuf, outCount)) 
        {
          Serial.println(F("[CAN->MB] writeResource FAIL"));
        } else 
        {
          Serial.print(F("[CAN->MB] write OK to ")); 
          Serial.print(rule.toModbus->name);
          Serial.print(F(" @addr=")); 
          Serial.println(rule.toModbus->address);
        }
      }
    }
  }

  // ========= Poll Modbus → CAN (MB2CAN) =========
  uint32_t now = millis();
  for (auto& p : g_pollers) 
  {
    const ModbusResourceSpec* res = p.res;
    if (!res || res->period_ms == 0) 
    {
      continue;
    }
    if (now - p.last_ms < res->period_ms) 
    {
      continue; // non ancora tempo
    }
    p.last_ms = now;

    // Leggi registri per la risorsa
    if (!MBM::readResource(*res, regsBuf)) 
    {
      Serial.print(F("[MB poll] read FAIL for ")); 
      Serial.println(res->name);
      continue;
    }

    // per ogni regola MB2CAN che usa questa risorsa, costruisci e invia il frame
    for (auto& rule : g_rules) 
    {
      if (rule.dir != RuleDir::MB2CAN || rule.fromModbus != res || !rule.toCan) 
      {
        continue;
      }

      uint32_t id; uint8_t dlc; uint8_t data[8];
      if (buildCanFromModbus(rule, regsBuf, res->count, id, dlc, data)) 
      {
        if (!CANM::sendRaw(id, dlc, data)) 
        {
          Serial.println(F("[MB->CAN] sendRaw FAIL (bus busy/no ACK)"));
        } else 
        {
          Serial.print(F("[MB->CAN] TX ")); Serial.print(rule.toCan->name);
          Serial.print(F(" id=0x")); Serial.print(id, HEX);
          Serial.print(F(" dlc=")); Serial.println(dlc);
        }
      }
    }
  }
}
