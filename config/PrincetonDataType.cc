#include "pds/config/PrincetonDataType.hh"

void Pds::PrincetonData::setTemperature(PrincetonDataType& d,
                                        float t)
{
  new(&d) PrincetonDataType(d.shotIdStart(),
                            d.readoutTime(),
                            t);
}
