#ifndef ZylaConfigType_hh
#define ZylaConfigType_hh

#include "pdsdata/xtc/TypeId.hh"
#include "pdsdata/psddl/zyla.ddl.h"

typedef Pds::Zyla::ConfigV1 ZylaConfigType;

static Pds::TypeId _zylaConfigType(Pds::TypeId::Id_ZylaConfig,
				     ZylaConfigType::Version);

#endif
