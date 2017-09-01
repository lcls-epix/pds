#include "pds/config/Andor3dConfigType.hh"

void Pds::Andor3dConfig::setNumDelayShots(Andor3dConfigType& c,
                                          unsigned n)
{
  new(&c) Andor3dConfigType(c.width(),
                            c.height(),
                            c.numSensors(),
                            c.orgX(),
                            c.orgY(),
                            c.binX(),
                            c.binY(),
                            c.exposureTime(),
                            c.coolingTemp(),
                            c.fanMode(),
                            c.baselineClamp(),
                            c.highCapacity(),
                            c.gainIndex(),
                            c.readoutSpeedIndex(),
                            c.exposureEventCode(),
                            c.exposureStartDelay(),
                            n);
}

void Pds::Andor3dConfig::setSize(Andor3dConfigType& c,
                                 unsigned w,
                                 unsigned h)
{
  new(&c) Andor3dConfigType(w, h,
                            c.numSensors(),
                            c.orgX(),
                            c.orgY(),
                            c.binX(),
                            c.binY(),
                            c.exposureTime(),
                            c.coolingTemp(),
                            c.fanMode(),
                            c.baselineClamp(),
                            c.highCapacity(),
                            c.gainIndex(),
                            c.readoutSpeedIndex(),
                            c.exposureEventCode(),
                            c.exposureStartDelay(),
                            c.numDelayShots());
}

void Pds::Andor3dConfig::setNumSensors(Andor3dConfigType& c,
                                       unsigned s)
{
  new(&c) Andor3dConfigType(c.width(),
                            c.height(),
                            s,
                            c.orgX(),
                            c.orgY(),
                            c.binX(),
                            c.binY(),
                            c.exposureTime(),
                            c.coolingTemp(),
                            c.fanMode(),
                            c.baselineClamp(),
                            c.highCapacity(),
                            c.gainIndex(),
                            c.readoutSpeedIndex(),
                            c.exposureEventCode(),
                            c.exposureStartDelay(),
                            c.numDelayShots());
}
