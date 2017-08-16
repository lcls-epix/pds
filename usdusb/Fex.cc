#include "Fex.hh"

#include "pds/xtc/InDatagram.hh"
#include "pds/config/CfgClientNfs.hh"
#include "pds/config/UsdUsbDataType.hh"
#include "pds/config/UsdUsbFexDataType.hh"

#include "pdsdata/xtc/DetInfo.hh"


#include <new>

using namespace Pds;
using namespace Pds::UsdUsb;

Fex::Fex(CfgClientNfs& cfg) :
  _cfg(cfg),
  _usdusb_config(new UsdUsbFexConfigType),
  _in(0) {}

Fex::~Fex() {
  delete _usdusb_config;
}

InDatagram* Fex::process(InDatagram* in) {
  _in = in;
  iterate(&in->datagram().xtc);
  return _in;
}

int Fex::process(Xtc* xtc) {
  if (xtc->contains.id()==TypeId::Id_Xtc)
    iterate(xtc);
  else if(xtc->contains.value() == _usdusbDataType.value()) {

    Xtc tc = Xtc(_usdusbFexDataType, xtc->src);
    tc.extent += sizeof(UsdUsbFexDataType);

    const UsdUsbDataType* data = reinterpret_cast<const UsdUsbDataType*>(xtc->payload());
    const ndarray<const int32_t, 1> encoder_count = data->encoder_count();

    const ndarray<const int32_t, 1> offset = _usdusb_config->offset();
    const ndarray<const double, 1> scale   = _usdusb_config->scale();

    double fex_values[FexConfigV1::NCHANNELS];

    for(int i=0; i<FexConfigV1::NCHANNELS; i++) {
      fex_values[i] = scale[i] * (encoder_count[i] + offset[i]);
    }
    
    UsdUsbFexDataType usdusb_fex(fex_values);
    _in->insert(tc, &usdusb_fex);

    return 0;
  }
  return 1;
}

bool Fex::configure(Transition& tr) {
  return _cfg.fetch(tr, _usdusbFexConfigType, _usdusb_config, sizeof(UsdUsbFexConfigType)) > 0;
}

void Fex::recordConfigure(InDatagram* dg) {
  Xtc tc = Xtc(_usdusbFexConfigType, _cfg.src());
  tc.extent += sizeof(UsdUsbFexConfigType);
  dg->insert(tc, _usdusb_config);
}
