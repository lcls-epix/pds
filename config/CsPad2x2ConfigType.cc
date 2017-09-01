#include "pds/config/CsPad2x2ConfigType.hh"

void Pds::CsPad2x2Config::setConfig(CsPad2x2ConfigType& c, 
                                    const Pds::CsPad::CsPadReadOnlyCfg& ro,
                                    unsigned concentratorVsn)
{
  const void* dst = reinterpret_cast<const void*>(&c.quad().ro());
  new(const_cast<void*>(dst)) Pds::CsPad::CsPadReadOnlyCfg(ro);
  *((uint32_t*)&c) = concentratorVsn;
}
