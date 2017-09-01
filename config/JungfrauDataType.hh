#ifndef Pds_JungfrauDataType_hh
#define Pds_JungfrauDataType_hh

#include "pdsdata/xtc/TypeId.hh"
#include "pdsdata/psddl/jungfrau.ddl.h"

typedef Pds::Jungfrau::ElementV2 JungfrauDataType;
typedef Pds::Jungfrau::ModuleInfoV1 JungfrauModuleInfoType;

static Pds::TypeId _jungfrauDataType(Pds::TypeId::Id_JungfrauElement,
          JungfrauDataType::Version);

#endif
