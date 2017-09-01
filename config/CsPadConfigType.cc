#include "pds/config/CsPadConfigType.hh"

void Pds::CsPadConfig::setConfig(CsPadConfigType& c, 
                                 const Pds::CsPad::CsPadReadOnlyCfg* ro,
                                 unsigned concentratorVsn)
{
  for(unsigned i=0; i<Pds::CsPad::MaxQuadsPerSensor; i++)
    if ((1<<i) &c.quadMask()) {
      const void* dst = reinterpret_cast<const void*>(&c.quads(i).ro());
      new(const_cast<void*>(dst)) Pds::CsPad::CsPadReadOnlyCfg(ro[i]);
    }
  *((uint32_t*)&c) = concentratorVsn;
}
