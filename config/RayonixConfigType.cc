#include "pds/config/RayonixConfigType.hh"

void Pds::RayonixConfig::setConfig(RayonixConfigType& c)
{
  new(&c) RayonixConfigType(c.binning_f(),
                            c.binning_s(),
                            c.testPattern(),
                            c.exposure(),
                            c.trigger(),
                            c.rawMode(),
                            c.darkFlag(),
                            c.readoutMode(),
                            c.deviceID());
}
