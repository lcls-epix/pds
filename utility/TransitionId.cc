#include "TransitionId.hh"

using namespace Pds;

const char* TransitionId::name(TransitionId::Value id)
{ 
  static const char* _names[] = {
    "Unknown",
    "Reset",
    "Map",
    "Unmap",
    "Configure",
    "Unconfigure",
    "BeginRun",
    "EndRun",
    "Pause",
    "Resume",
    "Enable",
    "Disable",
    "L1Accept"
  };
  return (id < TransitionId::NumberOf ? _names[id] : "-Invalid-");
};

