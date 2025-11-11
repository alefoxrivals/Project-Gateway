#include "modbus_manager.h"

static ModbusMaster g_mb;
static uint8_t g_deRePin = 7;
static bool    g_inited  = false;

static void preTransmission() 
{ 
  digitalWrite(g_deRePin, HIGH); 
}
static void postTransmission()
{ 
  digitalWrite(g_deRePin, LOW);
}

namespace MBM {

bool begin(const ModbusRtuConfig& cfg, uint8_t deRePin) 
{
  g_deRePin = deRePin;
  pinMode(g_deRePin, OUTPUT);
  digitalWrite(g_deRePin, LOW);

  g_mb.begin(cfg.slave_id, Serial1);
  Serial1.begin(cfg.baud, SERIAL_8N1); // per semplicità: 8N1 (parity/stop custom non supportati su R4 facilmente)
  g_mb.preTransmission(preTransmission);
  g_mb.postTransmission(postTransmission);

  g_inited = true;
  return true;
}

bool readResource(const ModbusResourceSpec& res, uint16_t* outRegs) 
{
  if (!g_inited) return false;
  if (res.fn != ModbusFn::ReadHolding) return false;

  uint8_t ec = g_mb.readHoldingRegisters(res.address, res.count);

  if (ec != g_mb.ku8MBSuccess) 
  {
    Serial.print(F("[MB] read ERR code=")); Serial.println(ec);
    return false;
  }
  for (uint16_t i=0;i<res.count;i++) 
  {
    outRegs[i] = g_mb.getResponseBuffer(i);
  }
  return true;
}

bool writeResource(const ModbusResourceSpec& res, const uint16_t* regs, uint16_t count) 
{
  if (!g_inited) return false;
  if (res.fn == ModbusFn::WriteSingle) 
  {
    if (count < 1) return false;

    uint8_t ec = g_mb.writeSingleRegister(res.address, regs[0]);

    if (ec != g_mb.ku8MBSuccess) 
    {
      Serial.print(F("[MB] writeSingle ERR code=")); 
      Serial.println(ec);
      return false;
    }
    return true;
  } else if (res.fn == ModbusFn::WriteMultiple) {

    if (count < res.count) return false;

    g_mb.clearTransmitBuffer();

    for (uint16_t i=0;i<res.count;i++)
    {
      g_mb.setTransmitBuffer(i, regs[i]);
    } 

    uint8_t ec = g_mb.writeMultipleRegisters(res.address, res.count);
    
    if (ec != g_mb.ku8MBSuccess) 
    {
      Serial.print(F("[MB] writeMultiple ERR code=")); 
      Serial.println(ec);
      return false;
    }
    return true;
  }
  return false;
}

ModbusMaster& client() 
{ 
  return g_mb; 
}

} // namespace
