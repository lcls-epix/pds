#include "EbS.hh"

#include "EbEvent.hh"
#include "EbSequenceKey.hh"
#include "pds/vmon/VmonEb.hh"

using namespace Pds;

extern unsigned nEbPrints;

EbS::EbS(const Src& id,
   const TypeId& ctns,
   Level::Type level,
   Inlet& inlet,
   OutletWire& outlet,
   int stream,
   int ipaddress,
   unsigned eventsize,
   unsigned eventpooldepth,
   int slowEb,
   VmonEb* vmoneb) :
  Eb(id, ctns, level, inlet, outlet,
     stream, ipaddress,
     eventsize, eventpooldepth, slowEb, vmoneb),
  _keys( sizeof(EbSequenceKey), eventpooldepth )
{
  memset(_no_builds,0,sizeof(_no_builds));
}

EbS::~EbS()
{
}

void EbS::no_build(Sequence::Type type, unsigned mask)
{
  _no_builds[type] |= mask;
}

//
//  Allocate a new datagram buffer and copy payload into it (from a previously allocated buffer)
//
EbEventBase* EbS::_new_event(const EbBitMask& serverId, char* payload, unsigned sizeofPayload)
{
  CDatagram* datagram = new(&_datagrams) CDatagram(_ctns, _id);
  EbSequenceKey* key = new(&_keys) EbSequenceKey(datagram->dg());
  EbEvent* event = new(&_events) EbEvent(serverId, _clients, datagram, key);
  event->allocated().insert(serverId);
  event->recopy(payload, sizeofPayload, serverId);

  unsigned depth = _datagrams.depth();

  if (_vmoneb) _vmoneb->depth(depth);

  if (depth<=1 && _pending.forward()!=_pending.empty()) {
    if (nEbPrints) {
      printf("EbS::new_event claiming buffer for srv %08x payload %d\n",
             serverId.value(),sizeofPayload);
      nEbPrints--;
    }

    _post(_pending.forward());
  //    arm(_post(_pending.forward()));
  }
  return event;
}

//
//  Allocate a new datagram buffer
//
EbEventBase* EbS::_new_event(const EbBitMask& serverId)
{
  unsigned depth = _datagrams.depth();

  if (_vmoneb) _vmoneb->depth(depth);

  if (depth==1 && _pending.forward()!=_pending.empty()) { // keep one buffer for recopy possibility
    if (nEbPrints) {
      printf("EbS::new_event claiming buffer for srv %08x\n",
             serverId.value());
      nEbPrints--;
    }

    _post(_pending.forward());
  //    arm(_post(_pending.forward()));
  }

  CDatagram* datagram = new(&_datagrams) CDatagram(_ctns, _id);
  EbSequenceKey* key = new(&_keys) EbSequenceKey(datagram->dg());
  return new(&_events) EbEvent(serverId, _clients, datagram, key);
}

unsigned EbS::_fixup( EbEventBase* event, const Src& client, const EbBitMask& id )
{
  unsigned fixup = 0;
  const Sequence& seq = event->key().sequence();
  if (!(_no_builds[seq.type()] & (1<<seq.service()))) {
    EbEvent* ev = (EbEvent*)event;
    fixup = ev->fixup ( client, id );
  }
  return fixup;
}

EbBase::IsComplete EbS::_is_complete( EbEventBase* event,
              const EbBitMask& serverId)
{
  const Sequence& seq = event->key().sequence();
  if (_no_builds[seq.type()] & (1<<seq.service()))
    return NoBuild;
  return EbBase::_is_complete(event, serverId);
}
//
//  The following two routines override the base class, so that
//  these event builders will always have a full "select mask";
//  i.e. fetch will be called for all pending IO.  This overrides
//  the base class behavior which only selects upon servers that
//  haven't yet contributed to the current event under construction.
//  This specialization is useful where the server data has
//  limited buffering (segment levels) and needs better realtime
//  response.
//
int EbS::poll()
{
  if (_level == Level::Segment) {
    if(!ServerManager::poll()) return 0;
    if(active().isZero()) ServerManager::arm(managed());
    return 1;
  }
  else
    return EbBase::poll();
}

int EbS::processIo(Server* srv)
{
  if (_level == Level::Segment) {
    Eb::processIo(srv);
    return 1;
  }
  else
    return Eb::processIo(srv);
}
