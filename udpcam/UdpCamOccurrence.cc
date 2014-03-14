// $Id$
// Author: Chris Ford <caf@slac.stanford.edu>

#include <stdlib.h>
#include <unistd.h>

#include "pds/utility/Occurrence.hh"
#include "pds/utility/OccurrenceId.hh"
#include "pds/service/GenericPool.hh"

#include "UdpCamManager.hh"
#include "UdpCamOccurrence.hh"

using namespace Pds;

UdpCamOccurrence::UdpCamOccurrence(UdpCamManager *mgr) :
  _mgr      (mgr),
  _outOfOrderPool   (new GenericPool(sizeof(Occurrence),1)),
  _userMessagePool  (new GenericPool(sizeof(UserMessage),1))
{
}

UdpCamOccurrence::~UdpCamOccurrence()
{
  delete _outOfOrderPool;
  delete _userMessagePool;
}

void UdpCamOccurrence::outOfOrder(void)
{
  // send occurrence: detector out-of-order
  Pds::Occurrence* occ = new (_outOfOrderPool)
  Pds::Occurrence(Pds::OccurrenceId::ClearReadout);
  _mgr->appliance().post(occ);
}

void UdpCamOccurrence::userMessage(char *msgText)
{
  // send occurrence: user message
  UserMessage* msg = new (_userMessagePool) UserMessage;
  msg->append(msgText);
  _mgr->appliance().post(msg);
}
