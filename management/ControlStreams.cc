#include "ControlStreams.hh"
#include "pds/utility/ToEventWire.hh"
#include "pds/utility/OpenOutlet.hh"
//#include "pds/utility/Occurrence.hh"
#include "pds/service/BitList.hh"
#include "pds/collection/Node.hh"
#include "pds/service/Task.hh"
#include "pds/service/VmonSourceId.hh"
#include "pds/management/PartitionMember.hh"
#include "EventBuilder.hh"
#include "pds/xtc/XtcType.hh"

using namespace Pds;

ControlStreams::ControlStreams(PartitionMember& cmgr) :
  WiredStreams(VmonSourceId(cmgr.header().level(), 0))
{
  //  VmonEb vmoneb(vmon());
  Level::Type level = cmgr.header().level();
  int ipaddress = cmgr.header().ip();
  unsigned eventpooldepth = 32;
  for (int s = 0; s < StreamParams::NumberOfStreams; s++) {
    _outlets[s] = new OpenOutlet(*stream(s)->outlet());
//     _outlets[s] = new ToEventWire(*stream(s)->outlet(),
//          cmgr,
//          ipaddress,
//          MaxSize*netbufdepth,
//          cmgr.occurrences());

    EventBuilder* eb =
      new EventBuilder(cmgr.header().procInfo(),
           _xtcType,
           level, *stream(s)->inlet(),
           *_outlets[s], s, ipaddress,
           MaxSize, eventpooldepth,
          cmgr.slowEb());
    //    vmoneb);

    eb->no_build(Sequence::Event,1<<TransitionId::L1Accept);

    _inlet_wires[s] = eb;
  }
}

ControlStreams::~ControlStreams()
{
  for (int s = 0; s < StreamParams::NumberOfStreams; s++) {
    delete _inlet_wires[s];
    delete _outlets[s];
  }
}

