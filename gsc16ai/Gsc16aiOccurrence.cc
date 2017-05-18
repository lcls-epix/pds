#include <stdlib.h>
#include <unistd.h>

#include "pds/utility/Occurrence.hh"
#include "pds/utility/OccurrenceId.hh"
#include "pds/service/GenericPool.hh"

#include "Gsc16aiManager.hh"
#include "Gsc16aiOccurrence.hh"

using namespace Pds;

Gsc16aiOccurrence::Gsc16aiOccurrence(Gsc16aiManager *mgr) :
  _mgr      (mgr),
  _outOfOrderPool   (new GenericPool(sizeof(Occurrence),1)),
  _userMessagePool  (new GenericPool(sizeof(UserMessage),1))
{
}

Gsc16aiOccurrence::~Gsc16aiOccurrence()
{
  delete _outOfOrderPool;
  delete _userMessagePool;
}

void Gsc16aiOccurrence::outOfOrder(void)
{
  // send occurrence: detector out-of-order
  Pds::Occurrence* occ = new (_outOfOrderPool)
  Pds::Occurrence(Pds::OccurrenceId::ClearReadout);
  _mgr->appliance().post(occ);
}

void Gsc16aiOccurrence::userMessage(const char *msgText)
{
  // send occurrence: user message
  UserMessage* msg = new (_userMessagePool) UserMessage;
  msg->append(msgText);
  _mgr->appliance().post(msg);
}
