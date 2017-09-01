#include "pds/config/JungfrauConfigType.hh"

void Pds::JungfrauConfig::setSize(JungfrauConfigType& c,
                                  unsigned modules,
                                  unsigned rows,
                                  unsigned columns)
{
  new(&c) JungfrauConfigType(modules,
                             rows,
                             columns,
                             c.biasVoltage(),
                             c.gainMode(),
                             c.speedMode(),
                             c.triggerDelay(),
                             c.exposureTime(),
                             c.exposurePeriod(),
                             c.vb_ds(),
                             c.vb_comp(),
                             c.vb_pixbuf(),
                             c.vref_ds(),
                             c.vref_comp(),
                             c.vref_prech(),
                             c.vin_com(),
                             c.vdd_prot());
}
