#include "pds/config/PimaxConfigType.hh"

void Pds::PimaxConfig::setNumIntegrationShots(PimaxConfigType& c,
                                        unsigned n)
{
  new(&c) PimaxConfigType(c.width(),
                          c.height(),
                          c.orgX(),
                          c.orgY(),
                          c.binX(),
                          c.binY(),
                          c.exposureTime(),
                          c.coolingTemp(),
                          c.readoutSpeed(),
                          c.gainIndex(),
                          c.intensifierGain(),
                          c.gateDelay(),
                          c.gateWidth(),
                          c.maskedHeight(),
                          c.kineticHeight(),
                          c.vsSpeed(),
                          c.infoReportInterval(),
                          c.exposureEventCode(),
                          n);
}

void Pds::PimaxConfig::setSize(PimaxConfigType& c,
                               unsigned w,
                               unsigned h)
{
  new(&c) PimaxConfigType(w, h,
                          c.orgX(),
                          c.orgY(),
                          c.binX(),
                          c.binY(),
                          c.exposureTime(),
                          c.coolingTemp(),
                          c.readoutSpeed(),
                          c.gainIndex(),
                          c.intensifierGain(),
                          c.gateDelay(),
                          c.gateWidth(),
                          c.maskedHeight(),
                          c.kineticHeight(),
                          c.vsSpeed(),
                          c.infoReportInterval(),
                          c.exposureEventCode(),
                          c.numIntegrationShots());
}
