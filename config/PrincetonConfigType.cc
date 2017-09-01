#include "pds/config/PrincetonConfigType.hh"

void Pds::PrincetonConfig::setNumDelayShots(PrincetonConfigType& c,
                                            unsigned n)
{
  new(&c) PrincetonConfigType(c.width(),
                              c.height(),
                              c.orgX(),
                              c.orgY(),
                              c.binX(),
                              c.binY(),
                              c.exposureTime(),
                              c.coolingTemp(),
                              c.gainIndex(),
                              c.readoutSpeedIndex(),
                              c.maskedHeight(),
                              c.kineticHeight(),
                              c.vsSpeed(),
                              c.infoReportInterval(),
                              c.exposureEventCode(),
                              n);
}

void Pds::PrincetonConfig::setWidth        (PrincetonConfigType& c, 
                                            unsigned w)
{
  new(&c) PrincetonConfigType(w,c.height());
}

void Pds::PrincetonConfig::setHeight       (PrincetonConfigType& c, 
                                            unsigned h)
{
  new(&c) PrincetonConfigType(c.width(),h);
}

void Pds::PrincetonConfig::setSize         (PrincetonConfigType& c, 
                                            unsigned w, 
                                            unsigned h)
{
  new(&c) PrincetonConfigType(w,h);
}

void Pds::PrincetonConfig::setReadoutSpeedIndex(PrincetonConfigType& c, 
                                                unsigned index)
{
  new(&c) PrincetonConfigType(c.width(),
                              c.height(),
                              c.orgX(),
                              c.orgY(),
                              c.binX(),
                              c.binY(),
                              c.exposureTime(),
                              c.coolingTemp(),
                              c.gainIndex(),
                              index,
                              c.maskedHeight(),
                              c.kineticHeight(),
                              c.vsSpeed(),
                              c.infoReportInterval(),
                              c.exposureEventCode(),
                              c.numDelayShots());
}
