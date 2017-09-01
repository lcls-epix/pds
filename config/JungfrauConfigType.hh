#ifndef JungfrauConfigType_hh
#define JungfrauConfigType_hh

#include "pdsdata/xtc/TypeId.hh"
#include "pdsdata/psddl/jungfrau.ddl.h"

typedef Pds::Jungfrau::ConfigV2 JungfrauConfigType;
typedef Pds::Jungfrau::ModuleInfoV1 JungfrauModInfoType;

static Pds::TypeId _jungfrauConfigType(Pds::TypeId::Id_JungfrauConfig,
				     JungfrauConfigType::Version);

namespace Pds {
  namespace JungfrauConfig {
    void setSize(JungfrauConfigType&,
                 unsigned modules,
                 unsigned rows,
                 unsigned columns);
  }
}

#endif
