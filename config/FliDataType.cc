#include "pds/config/FliDataType.hh"

void Pds::FliData::setTemperature(FliDataType& d,
                                  float        t)
{
  new(&d) FliDataType(d.shotIdStart(),
                      d.readoutTime(),
                      t);
}
