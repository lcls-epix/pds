#include "pds/config/EvrConfigType.hh"

void Pds::EvrConfig::clearMask(EventCodeType& tc,
                               unsigned maskType)
{
  new (&tc) EventCodeType(tc.code(), tc.isReadout(), tc.isCommand(), tc.isLatch(), 
                          tc.reportDelay(), tc.reportWidth(), 
                          (maskType&1) ? 0 : tc.maskTrigger(),
                          (maskType&2) ? 0 : tc.maskSet(),
                          (maskType&4) ? 0 : tc.maskClear(),
                          tc.desc(), tc.readoutGroup());
}

void Pds::EvrConfig::setMask(EventCodeType& tc,
                             unsigned maskType,
                             unsigned maskValue)
{
  new (&tc) EventCodeType(tc.code(), tc.isReadout(), tc.isCommand(), tc.isLatch(), 
                          tc.reportDelay(), tc.reportWidth(), 
                          tc.maskTrigger() | ((maskType&1) ? maskValue : 0),
                          tc.maskSet    () | ((maskType&2) ? maskValue : 0),
                          tc.maskClear  () | ((maskType&4) ? maskValue : 0),
                          tc.desc(), tc.readoutGroup());
}

unsigned Pds::EvrConfig::size(const EvrConfigType& c)
{
  return sizeof(c)
    + c.neventcodes()*sizeof(EventCodeType) 
    + c.npulses    ()*sizeof(PulseType) 
    + c.noutputs   ()*sizeof(OutputMapType) 
    + c.seq_config()._sizeof();
}

unsigned Pds::EvrConfig::map(const OutputMapType& m)
{
  enum { Pulse_Offset=0, 
         DBus_Offset=32, 
         Prescaler_Offset=40 }; 
  unsigned map=0;
  OutputMapType::Source src    = m.source();
  unsigned    src_id =m.source_id();
  switch(src) {
  case OutputMapType::Pulse     : map = (src_id + Pulse_Offset); break;
  case OutputMapType::DBus      : map = (src_id + DBus_Offset); break;
  case OutputMapType::Prescaler : map = (src_id + Prescaler_Offset); break;
  case OutputMapType::Force_High: map = 62; break;
  case OutputMapType::Force_Low : map = 63; break;
  }
  return map;
}
