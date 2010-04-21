#ifndef Pds_FccdConfigType_hh
#define Pds_FccdConfigType_hh

#include "pdsdata/xtc/TypeId.hh"
#include "pdsdata/fccd/FccdConfigV1.hh"

typedef Pds::FCCD::FccdConfigV1 FccdConfigType;

static Pds::TypeId _fccdConfigType(Pds::TypeId::Id_FccdConfig,
				     FccdConfigType::Version);

#endif
