#include "IpimbServer.hh"
#include "pds/xtc/CDatagram.hh"
#include "pds/config/IpimbDataType.hh"
#include "pdsdata/psddl/ipimb.ddl.h"

#include <unistd.h>
#include <sys/uio.h>
#include <string.h>
#include <errno.h>

static const int MaxPrints=20;
static int nPrints;

using namespace Pds;
	
IpimbServer::IpimbServer(const Src& client, const bool c01)
{
  _c01 = c01;
  _baselineSubtraction = 1;
  _polarity = -1;
  if (_c01) {
    _xtc = Xtc(oldIpimbDataType, client);
    _xtc.extent = sizeof(OldIpimbDataType)+sizeof(Xtc);
  } else {
    _xtc = Xtc(_ipimbDataType,client);
    _xtc.extent = sizeof(IpimbDataType)+sizeof(Xtc);
  }
}

unsigned IpimbServer::offset() const {
  return sizeof(Xtc);
}

void IpimbServer::dump(int detail) const
{
}

bool IpimbServer::isValued() const
{
  return true;
}

const Src& IpimbServer::client() const
{
  return _xtc.src; 
}

const Xtc& IpimbServer::xtc() const
{
  //  unsigned* junk = (unsigned*)(&_xtc);
  //  printf("xtc: ");
  //  for (unsigned i=0;i<5;i++) {printf("%8.8x ",junk[i]);}
  //  printf("\n");
  return _xtc;
}

int IpimbServer::pend(int flag) 
{
  return -1;
}

unsigned IpimbServer::length() const {
  return _xtc.extent;
}

int IpimbServer::fetch(char* payload, int flags)
{
  IpimBoardData data = _ipimBoard->WaitData();
  _xtc.damage = 0;
  if (_ipimBoard->dataDamage()) {
    if (nPrints) {
      printf("IpimBoard error: IpimbServer::fetch had problems getting data, fd %d, device %s or data had issues\n", fd(), _serialDevice);
      if ( --nPrints == 0 )
        printf("...throttling error messages\n");
    }
    //    data.dumpRaw(); // turned off for the moment for presampling
    _xtc.damage.increase(Pds::Damage::UserDefined);
  }
  //  double ch0 = data.GetCh0_V();
  unsigned long long ts = data.GetTriggerCounter();//_data->GetTimestamp_ticks();
  //  printf("data from fd %d has e.g. ch0=%fV, ts=%llu\n", fd(), ch0, ts);

  int payloadSize = sizeof(IpimbDataType);
  if (_c01) payloadSize = sizeof(Ipimb::DataV1);
  memcpy(payload, &_xtc, sizeof(Xtc));
  memcpy(payload+sizeof(Xtc), &data, payloadSize);
  _count = (unsigned) (0xFFFFFFFF&ts);
  return payloadSize+sizeof(Xtc);
}


unsigned IpimbServer::count() const
{
  return _count - 1; // "counting from" hack
}

void IpimbServer::setIpimb(IpimBoard* ipimb, char* portName, const int baselineSubtraction, const int polarity) {
  _ipimBoard = ipimb;
  _baselineSubtraction = baselineSubtraction;
  _polarity = polarity;
  fd(_ipimBoard->get_fd());
  _serialDevice = portName;
}

unsigned IpimbServer::configure(IpimbConfigType& config,
                                std::string&     msg) {
  nPrints = MaxPrints;
  printf("In IpimbServer, using baseline mode %d, polarity %d\n", _baselineSubtraction, _polarity);
  if (_c01) _ipimBoard->setOldVersion();
  _ipimBoard->setBaselineSubtraction(_baselineSubtraction, _polarity); // avoid updating config class; caveat user
  return _ipimBoard->configure(config,msg);
}

unsigned IpimbServer::unconfigure() {
  return _ipimBoard->unconfigure();
}
