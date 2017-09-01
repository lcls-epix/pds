#include "pds/config/AndorConfigType.hh"

void Pds::AndorConfig::setNumDelayShots(AndorConfigType& c,
                                        unsigned n)
{
  new(&c) AndorConfigType(c.width(),
                          c.height(),
                          c.orgX(),
                          c.orgY(),
                          c.binX(),
                          c.binY(),
                          c.exposureTime(),
                          c.coolingTemp(),
                          c.fanMode(),
                          c.cropMode(),
                          c.baselineClamp(),
                          c.highCapacity(),
                          c.gainIndex(),
                          c.readoutSpeedIndex(),
                          c.exposureEventCode(),
                          n);
}

void Pds::AndorConfig::setSize(AndorConfigType& c,
                               unsigned w,
                               unsigned h)
{
  new(&c) AndorConfigType(w, h,
                          c.orgX(),
                          c.orgY(),
                          c.binX(),
                          c.binY(),
                          c.exposureTime(),
                          c.coolingTemp(),
                          c.fanMode(),
                          c.cropMode(),
                          c.baselineClamp(),
                          c.highCapacity(),
                          c.gainIndex(),
                          c.readoutSpeedIndex(),
                          c.exposureEventCode(),
                          c.numDelayShots());
}
