#include "pds/config/FliConfigType.hh"

void Pds::FliConfig::setNumDelayShots(FliConfigType& c,
                                      unsigned       n)
{
  new(&c) FliConfigType(c.width(),
                        c.height(),
                        c.orgX(),
                        c.orgY(),
                        c.binX(),
                        c.binY(),
                        c.exposureTime(),
                        c.coolingTemp(),
                        c.gainIndex(),
                        c.readoutSpeedIndex(),
                        c.exposureEventCode(),
                        n);
}

void Pds::FliConfig::setSize(FliConfigType& c,
                             unsigned w,
                             unsigned h)
{
  new(&c) FliConfigType(w, h,
                        c.orgX(),
                        c.orgY(),
                        c.binX(),
                        c.binY(),
                        c.exposureTime(),
                        c.coolingTemp(),
                        c.gainIndex(),
                        c.readoutSpeedIndex(),
                        c.exposureEventCode(),
                        c.numDelayShots());
}
