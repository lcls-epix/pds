#include "pds/config/AndorDataType.hh"

void Pds::AndorData::setTemperature(AndorDataType& d,
                                    float          t)
{
  new (&d) AndorDataType(d.shotIdStart(),
                         d.readoutTime(),
                         t);
}
