#include "can_manager.h"

namespace CANM {

bool begin(long bitrate) 
{
  return CAN.begin(bitrate);
}

bool sendRaw(uint32_t id, uint8_t dlc, const uint8_t data[8]) 
{
  CanMsg m(CanStandardId(id), dlc, (uint8_t*)data);
  return CAN.write(m) >= 0;
}

static void printOneField(const FieldSpec& f, const uint8_t* p) 
{
  Serial.print(f.name); 
  Serial.print('=');
  switch (f.type) 
  {
    case FieldType::Bool: {
      uint8_t v = readValue<uint8_t>(p, f.endian, f.size);
      Serial.print(v ? F("true") : F("false"));
    } break;
    case FieldType::Uint16: {
      uint16_t u = readValue<uint16_t>(p, f.endian, f.size);
      if (f.scale != 1) Serial.print((double)u / f.scale);
      else Serial.print(u);
    } break;
    case FieldType::Int16: {
      int16_t s = readValue<int16_t>(p, f.endian, f.size);
      if (f.scale != 1) Serial.print((double)s / f.scale);
      else Serial.print(s);
    } break;
    case FieldType::Float32: {
      float fl = readValue<float>(p, f.endian, f.size);
      if (f.scale != 1) Serial.print((double)fl / f.scale);
      else Serial.print(fl, 3);
    } break;
    default: Serial.print('?'); break;
  }
}

void prettyPrintRx(const std::vector<CanMessageSpec>& specs, const CanMsg& rx) {
  // trova spec per id
  const CanMessageSpec* spec=nullptr;
  for (auto& m : specs) if (m.id == rx.id) { spec=&m; break; }

  Serial.print(F("[RX] id=0x")); Serial.print(rx.id, HEX);
  Serial.print(F(" dlc=")); Serial.print(rx.data_length);
  Serial.print(F(" data:"));
  for (uint8_t i=0;i<rx.data_length;i++){ Serial.print(' '); Serial.print(rx.data[i], HEX); }
  Serial.println();

  if (!spec) return; // nessuna spec → niente decode

  Serial.print(F("     ")); Serial.print(spec->name); Serial.print(F(" -> "));
  for (size_t i=0;i<spec->fields.size(); ++i) {
    const FieldSpec& f = spec->fields[i];
    if (f.offset + f.size <= rx.data_length) {
      const uint8_t* p = &rx.data[f.offset];
      printOneField(f, p);
      if (i+1 < spec->fields.size()) Serial.print(F(", "));
    }
  }
  Serial.println();
}

} // namespace
