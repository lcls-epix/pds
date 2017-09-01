#include "pds/config/PimaxDataType.hh"

void Pds::PimaxData::setTemperature(PimaxDataType& d,
                                    float          t)
{
  new (&d) PimaxDataType(d.shotIdStart(),
                         d.readoutTime(),
                         t);
}
