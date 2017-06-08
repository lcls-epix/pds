#ifndef Pds_ZylaDataType_hh
#define Pds_ZylaDataType_hh

#include "pdsdata/xtc/TypeId.hh"
#include "pdsdata/psddl/zyla.ddl.h"

typedef Pds::Zyla::FrameV1 ZylaDataType;

static Pds::TypeId _zylaDataType(Pds::TypeId::Id_ZylaFrame,
          ZylaDataType::Version);

#endif
