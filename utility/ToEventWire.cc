#include "ToEventWire.hh"

#include "Mtu.hh"
#include "Transition.hh"
#include "Occurrence.hh"
#include "pds/collection/CollectionManager.hh"
#include "pds/xtc/Datagram.hh"
#include "pds/xtc/InDatagram.hh"

#include <errno.h>

using namespace Pds;

ToEventWire::ToEventWire(Outlet& outlet,
			 CollectionManager& collection,
			 int interface,
			 int maxbuf,
			 const Ins& occurrences) :
  OutletWire  (outlet),
  _collection (collection),
  _postman    (interface, Mtu::Size, 1 + maxbuf / Mtu::Size),
  _occurrences(occurrences)
{
}

ToEventWire::~ToEventWire()
{
}

Transition* ToEventWire::forward(Transition* tr)
{
  Ins dst(tr->reply_to());
  _collection.ucast(*tr,dst);
  return 0;
}

Occurrence* ToEventWire::forward(Occurrence* tr)
{
  if (tr->id() != OccurrenceId::EvrCommand)
    _collection.ucast(*tr,_occurrences);
  return 0;
}

InDatagram* ToEventWire::forward(InDatagram* dg)
{
  const Sequence& seq = dg->datagram().seq;
  int result;
  if (!_nodes.isempty()) {
    const Ins& dst = (seq.isEvent()) ? _nodes.lookup(seq)->ins() : _bcast;
    result = dg->send(_postman, dst);
  }
  else {
    result = EDESTADDRREQ;
  }
  if (result) _log(dg->datagram(), result);
  return 0;
}

void ToEventWire::bind(NamedConnection, const Ins& ins) 
{
  _bcast = ins;
}

void ToEventWire::bind(unsigned id, const Ins& node) 
{
  _nodes.insert(id, node);
}

void ToEventWire::unbind(unsigned id) 
{
  _nodes.remove(id);
}

void ToEventWire::dump(int detail)
{
  //  printf("Data flow throttle:\n");
  //  FlowThrottle::connect()->dump(detail);

  if (!_nodes.isempty()) {
    unsigned i=0;
    OutletWireIns* first = _nodes.lookup(i);
    OutletWireIns* current = first;
    do {
      printf(" Event id %i, port %i, address 0x%x\n",
	     current->id(),
	     current->ins().portId(),
	     current->ins().address());
      //      _ack_handler.dump(i, detail);
      current = _nodes.lookup(++i);
    } while (current != first);
  }
}

void ToEventWire::dumpHistograms(unsigned tag, const char* path)
{
  if (!_nodes.isempty()) {
    unsigned i=0;
    OutletWireIns* first = _nodes.lookup(i);
    OutletWireIns* current = first;
    do {
      //      _ack_handler.dumpHistograms(i, tag, path);
      current = _nodes.lookup(++i);
    } while (current != first);
  }
}

void ToEventWire::resetHistograms()
{
  if (!_nodes.isempty()) {
    unsigned i=0;
    OutletWireIns* first = _nodes.lookup(i);
    OutletWireIns* current = first;
    do {
      //      _ack_handler.resetHistograms(i);
      current = _nodes.lookup(++i);
    } while (current != first);
  }
}
