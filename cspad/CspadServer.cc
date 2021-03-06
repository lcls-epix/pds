/*
 * CspadServer.cc
 *
 *  Created on: Nov 15, 2010
 *      Author: jackp
 */

#include "pds/cspad/CspadServer.hh"
#include "pds/cspad/CspadConfigurator.hh"
#include "pds/xtc/CDatagram.hh"
#include "pds/config/CsPadConfigType.hh"
#include "pds/pgp/DataImportFrame.hh"
#include "pds/pgp/RegisterSlaveExportFrame.hh"
#include "pds/service/Task.hh"
#include "pds/service/TaskObject.hh"
#include "pds/cspad/CspadDestination.hh"
#include "pds/cspad/Processor.hh"
#include "pgpcard/PgpCardMod.h"
#include <unistd.h>
#include <sys/uio.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <new>

using namespace Pds;
//using namespace Pds::CsPad;

CspadServer* CspadServer::_instance = 0;

class Task;
class TaskObject;

long long int timeDiff(timespec* end, timespec* start) {
  long long int diff;
  diff =  (end->tv_sec - start->tv_sec) * 1000000000LL;
  diff += end->tv_nsec;
  diff -= start->tv_nsec;
  return diff;
}

CspadServer::CspadServer( const Pds::Src& client, Pds::TypeId& myDataType, unsigned configMask )
   : _debug(0),
     _offset(0),
     _xtc( myDataType, client ),
     _cnfgrtr(0),
     _quads(0),
     _configMask(configMask),
     _configureResult(0xdead),
     _occPool(new GenericPool(sizeof(UserMessage),4)),
     _configured(false),
     _firstFetch(true),
     _ignoreFetch(true),
     _sequenceServer(false) {
  _histo = (unsigned*)calloc(sizeOfHisto, sizeof(unsigned));
  _rHisto = (unsigned*)calloc(sizeOfRHisto, sizeof(unsigned));
  _task = new Pds::Task(Pds::TaskObject("CSPADprocessor"));
  _dummy = (unsigned*)malloc(DummySize);
  strcpy(_runTimeConfigName, "");
  instance(this);
}

CspadServer::~CspadServer() { delete _occPool; }


unsigned CspadServer::configure(CsPadConfigType* config) {
  if (config == 0) {
    printf("CspadServer::configure was passed a nil config!\n");
    _configureResult = 0xdead;
  } else {
    if (_cnfgrtr == 0) {
      _cnfgrtr = new Pds::CsPad::CspadConfigurator(config, fd(), _debug);
      _cnfgrtr->runTimeConfigName(_runTimeConfigName);
      pgp(_cnfgrtr->pgp());
    } else {
      printf("CspadConfigurator already instantiated\n");
    }
    unsigned c = flushInputQueue(fd());
    if (c) printf("CspadServer::configure flushed %u event%s before configuration\n", c, c>1 ? "s" : "");
    if ((_configureResult = _cnfgrtr->configure(config, _configMask))) {
      printf("CspadServer::configure failed 0x%x\n", _configureResult);
      _cnfgrtr->dumpPgpCard();
    } else {
      _quads = 0;
      for (unsigned i=0; i<4; i++) if ((1<<i) & config->quadMask()) _quads += 1;
      _payloadSize = config->payloadSize();
      _xtc.extent = (_payloadSize * _quads) + sizeof(Xtc);
      printf("CspadServer::configure _quads(%u) _payloadSize(%u) _xtc.extent(%u)\n",
          _quads, _payloadSize, _xtc.extent);
    }
    _firstFetch = true;
    _fiducials = _count = _quadsThisCount = 0;
    _configured = _configureResult == 0;
    c = this->flushInputQueue(fd());
    if (c) printf("CspadServer::configure flushed %u event%s after confguration\n", c, c>1 ? "s" : "");
  }
  return _configureResult;
}

void Pds::CspadServer::die() {
  printf("CspadServer::die() has been called");
}

void Pds::CspadServer::printState() {
  printf("  CspadServer _quads(%u) _quadMask(%x) _count(%u) _fiducials(%x) _quadsThisCount(%x)\n",
      _quads, _quadMask, _count + _offset, _fiducials, _quadsThisCount);
}

void Pds::CspadServer::dumpFrontEnd() {
  if ((_configureResult != 0xdead) && (_cnfgrtr != 0)) {
    _cnfgrtr->dumpFrontEnd();
  } else {
    printf("CspadServer::dumpFrontEnd found nil configuration or configurator\n");
  }
}

void CspadServer::process() {
//  _ioIndex = ++_ioIndex % Pds::Xamps::numberOfFrames;
//  Pds::Routine* r = new Pds::Xamps::Processor(4, _ioIndex);
//  _task->call(r);
}

void Pds::CspadServer::enable() {
  _d.dest(Pds::CsPad::CspadDestination::CR);
  if (_configureResult != 0xdead) {
    _pgp->writeRegister(
        &_d,
        CsPad::CspadConfigurator::RunModeAddr,
        _cnfgrtr->configuration().activeRunMode());
    ::usleep(10000);
    _firstFetch = true;
    flushInputQueue(fd(), false);
    if (_debug & 0x20) printf("CspadServer::enable\n");
    _ignoreFetch = false;
  } else {
    printf("CspadServer::enable found nil configuration\n");
  }
}

void Pds::CspadServer::runTimeConfigName(char* name) {
  if (name) strcpy(_runTimeConfigName, name);
  printf("Pds::CspadServer::runTimeConfigName(%s)\n", name);
}

void Pds::CspadServer::disable(bool flush) {
  _ignoreFetch = true;
  if (_cnfgrtr && _pgp) {
    _d.dest(Pds::CsPad::CspadDestination::CR);
    if ((_configureResult != 0xdead) && (_cnfgrtr != 0)) {
      _pgp->writeRegister(
          &_d,
          CsPad::CspadConfigurator::RunModeAddr,
          _cnfgrtr->configuration().inactiveRunMode());
      ::usleep(10000);
      if (flush) flushInputQueue(fd(), false);
      if (_debug & 0x20) printf("CspadServer::disable\n");
    } else {
      printf("CspadServer::disable found nil configuration or configurator, so not disabling\n");
    }
  } else {
    printf("CspadServer::disable found nil objects, so not disabling\n");
  }
}

unsigned Pds::CspadServer::unconfigure(void) {
  unsigned c = flushInputQueue(fd());
  if (c) printf("CspadServer::unconfigure flushed %u event%s\n", c, c>1 ? "s" : "");
  return 0;
}

int Pds::CspadServer::fetch( char* payload, int flags ) {
   int ret = 0;
   PgpCardRx       pgpCardRx;
   unsigned        offset = 0;
   bool            exceptional = false;
   enum {Ignore=-1};

   if (_ignoreFetch) {
     unsigned c = this->flushInputQueue(fd(), false);
     if (_debug & 1) printf("CspadServer::fetch() ignored and flushed %u input buffer%s\n", c, c>1 ? "s" : "");
     return Ignore;
   }

   if (_configured == false)  {
     printf("CspadServer::fetch() called before configuration, configuration result 0x%x\n", _configureResult);
     unsigned c = this->flushInputQueue(fd());
     printf("\tWe flushed %u input buffer%s\n", c, c>1 ? "s" : "");
     return Ignore;
   }

   if (_debug & 1) printf("CspadServer::fetch called ");

   _xtc.damage = 0;

   _quadsThisCount %= _quads;

   if (!_quadsThisCount) {
     _quadMask = 0;
     memcpy( payload, &_xtc, sizeof(Xtc) );
     offset = sizeof(Xtc);
     if (_firstFetch) {
       _firstFetch = false;
       clock_gettime(CLOCK_REALTIME, &_lastTime);
     } else {
       clock_gettime(CLOCK_REALTIME, &_thisTime);
       long long int diff = timeDiff(&_thisTime, &_lastTime);
       if (diff > 0) {
    	   unsigned peak = 0;
    	   unsigned max = 0;
    	   unsigned count = 0;
    	   bool     doTest = false;
    	   diff += 500000;
    	   diff /= 1000000;
    	   if (diff > sizeOfHisto-1) diff = sizeOfHisto-1;
    	   _histo[diff] += 1;
    	   for (unsigned i=0; i<(sizeOfHisto); i++) {
    		   if (_histo[i]) {
    			   if (_histo[i] > max) {
    				   max = _histo[i];
    				   peak = i;
    			   }
    			   count += _histo[i];
    		   }
    		   if (count > 100) {
    		     doTest = true;
    		   }
    	   }
    	   if (doTest && ((diff >= (peak+2)) || (diff <= (peak-2)))) {
    		   printf("CspadServer::fetch exceptional period %3lld, not %3u ", diff, peak);
    		   exceptional = true;
    	   }
       } else {
    	   printf("CspadServer::fetch Clock backtrack %f ms\n", diff / 1000000.0);
       }
       memcpy(&_lastTime, &_thisTime, sizeof(timespec));
     }
   }

   pgpCardRx.model   = sizeof(&pgpCardRx);
   pgpCardRx.maxSize = _payloadSize / sizeof(__u32);
   pgpCardRx.data    = (__u32*)(payload + offset);
   Pds::Pgp::DataImportFrame* data = (Pds::Pgp::DataImportFrame*)(payload + offset);

   clock_gettime(CLOCK_REALTIME, &_readTime1);
   if ((ret = read(fd(), &pgpCardRx, sizeof(PgpCardRx))) < 0) {
     if (errno == ERESTART) {
       disable(false);
       char message[400];
       sprintf(message, "Pgpcard problem! Restart the DAQ system\nIf this does not work, Power cycle %s\n",
           getenv("HOSTNAME"));
       UserMessage* umsg = new (_occPool) UserMessage;
       umsg->append(message);
       umsg->append(DetInfo::name(static_cast<const DetInfo&>(_xtc.src)));
       _mgr->appliance().post(umsg);
       _ignoreFetch = true;
       printf("CspadServer::fetch exiting because of ERESTART\n");
       exit(-ERESTART);
     }
     perror ("CspadServer::fetch pgpCard read error");
     ret =  Ignore;
   } else ret *= sizeof(__u32);
   clock_gettime(CLOCK_REALTIME, &_readTime2);
   long long int diff = timeDiff(&_readTime2, &_readTime1);
   diff += 5000;
   diff /= 10000;
   if (diff > sizeOfRHisto-1) diff = sizeOfRHisto-1;
   _rHisto[diff] += 1;

   if ((ret > 0) && (ret < (int)_payloadSize)) {
     printf("CspadServer::fetch() returning Ignore, ret was %d, looking for %u, frame(%u) quad(%u) quadmask(%x) ",
         ret, _payloadSize, data->frameNumber() - 1, data->elementId(), _quadMask);
     if (_debug & 4 || ret < 0) printf("\n\topcode(0x%x) acqcount(0x%x) fiducials(0x%x) _count(%u) _quadsThisCount(%u) lane(%u) vc(%u)",
         data->second.opCode, data->acqCount(), data->fiducials(), _count, _quadsThisCount, pgpCardRx.pgpLane, pgpCardRx.pgpVc);
     ret = Ignore;
     //     printf("\n\t ");
     //     unsigned* u= (unsigned*)data;
     //     for (unsigned j=0; j<(sizeof(Pds::Pgp::DataImportFrame)/sizeof(unsigned)); j++) {
     //       printf(" 0x%x", u[j]);
     //     }
     printf("\n");
   }

   unsigned damageMask = 0;
   if (pgpCardRx.eofe)      damageMask |= 1;
   if (pgpCardRx.fifoErr)   damageMask |= 2;
   if (pgpCardRx.lengthErr) damageMask |= 4;
   if (damageMask) {
     printf("CsPadServer::fetch %s damageMask 0x%x, quad(%u) quadmask(%x)", ret>0 ? "setting user damage" : "ignoring wrong length", damageMask, data->elementId(), _quadMask);
     if (pgpCardRx.lengthErr) printf(" LENGTH_ERROR rxSize(%u)", (unsigned)pgpCardRx.rxSize);
     if (pgpCardRx.fifoErr) printf(" FIFO_ERROR");
     if (pgpCardRx.eofe) printf(" EOFE_ERROR");
     printf(" frame %u\n", data->frameNumber() - 1);
     if (ret > 0) {
       damageMask |= 0xe0;
       _xtc.damage.increase(Pds::Damage::UserDefined);
       _xtc.damage.userBits(damageMask);
     }
   } else {
     unsigned oldCount = _count;
     _count = data->frameNumber() - 1;  // cspad starts counting at 1, not zero
     _fiducials = data->fiducials();    // for the other event builder
     if (_debug & 4 || ret < 0) printf("\n\tquad(%u) opcode(0x%x) acqcount(0x%x) fiducials(0x%x) _oldCount(%u) _count(%u) _quadsThisCount(%u) lane(%u) vc(%u)\n",
         data->elementId(), data->second.opCode, data->acqCount(), data->fiducials(), oldCount, _count, _quadsThisCount, pgpCardRx.pgpLane, pgpCardRx.pgpVc);
     if ((_count != oldCount) && (_quadsThisCount) && (!_sequenceServer)) {
       if ((_count < oldCount) || (_count - oldCount > 10)) {
         printf("CsPadServer::fetch ignoring unreasonable frame number, %u followed %u, quadMask 0x%x, quad %u\n", _count, oldCount, _quadMask, data->elementId());
         ret = Ignore;
       } else {
         int missing = _quads;
         for(unsigned k=0; k<4; k++) { if (_quadMask & 1<<k) missing -= 1; }
         printf("CsPadServer::fetch detected missing %d quad%s in frame(%u) has %u quads,  quadMask %x, because quad %u,  frame %u arrived\n",
             missing, missing > 1 ? "s" : "", oldCount, _quadsThisCount, _quadMask, data->elementId(), _count);
         _quadsThisCount = 0;
         _quadMask = 1 << data->elementId();
         memcpy( payload, &_xtc, sizeof(Xtc) );
         ret = sizeof(Xtc);
       }
     }
     if (exceptional) {
       printf(" frame %u, fiducials 0x%x\n", _count, _fiducials);
     }
   }
   if (ret > 0) {
     _quadsThisCount += 1;
     ret += offset;
     if (_quadMask & (1 << data->elementId())) {
       printf("CsPadServer::fetch duplicate quad 0x%x with quadmask 0x%x\n", data->elementId(), _quadMask);
       damageMask |= 0xd0 | 1 << data->elementId();
       _xtc.damage.increase(Pds::Damage::UserDefined);
       _xtc.damage.userBits(damageMask);
     }
     _quadMask |= 1 << data->elementId();
   }
   if (_debug & 1) printf(" returned %d\n", ret);
   return ret;
}

bool CspadServer::more() const {
  bool ret = _quads > 1;
  if (_debug & 2) printf("CspadServer::more(%s)\n", ret ? "true" : "false");
  return ret;
}

unsigned CspadServer::offset() const {
  unsigned ret = _quadsThisCount == 1 ? 0 : sizeof(Xtc) + _payloadSize * (_quadsThisCount-1);
  if (_debug & 2) printf("CspadServer::offset(%u)\n", ret);
  return (ret);
}

unsigned CspadServerCount::count() const {
  if (_debug & 2) printf( "CspadServerCount::count(%u)\n", _count);
  return _count + _offset;
}

unsigned CspadServerSequence::fiducials() const {
  if (_debug & 2) printf( "CspadServerSequence::fiducials(0x%x)\n", _fiducials);
  return _fiducials;
}

unsigned CspadServer::flushInputQueue(int f, bool printFlag) {
  fd_set          fds;
  struct timeval  timeout;
  timeout.tv_sec  = 0;
  timeout.tv_usec = 2500;
  int ret;
  unsigned count = 0;
  PgpCardRx       pgpCardRx;
  pgpCardRx.model   = sizeof(&pgpCardRx);
  pgpCardRx.maxSize = DummySize;
  pgpCardRx.data    = _dummy;
  do {
    FD_ZERO(&fds);
    FD_SET(f,&fds);
    ret = select( f+1, &fds, NULL, NULL, &timeout);
    if (ret>0) {
      if (!count) {
        if (printFlag) printf("\tflushed lanes ");
      }
      count += 1;
      ::read(f, &pgpCardRx, sizeof(PgpCardRx));
      if (printFlag) printf("-%u-", pgpCardRx.pgpLane);
    }
  } while ((ret > 0) && (count < 100));
  if (count) {
    if (printFlag) printf("\n");
    if (count == 100) {
      printf("\tCspadServer::flushInputQueue: pgpCardRx lane(%u) vc(%u) rxSize(%u) eofe(%s) lengthErr(%s)\n",
          pgpCardRx.pgpLane, pgpCardRx.pgpVc, pgpCardRx.rxSize, pgpCardRx.eofe ? "true" : "false",
          pgpCardRx.lengthErr ? "true" : "false");
      printf("\t\t"); for (ret=0; ret<8; ret++) printf("%u ", _dummy[ret]);  printf("/n");
    }
  }
  return count;
}

void CspadServer::setCspad( int f ) {
  fd( f );
}

void CspadServer::printHisto(bool c) {
  printf("CspadServer event fetch periods\n");
  for (unsigned i=0; i<sizeOfHisto; i++) {
    if (_histo[i]) {
      printf("\t%3u ms   %8u\n", i, _histo[i]);
      if (c) _histo[i] = 0;
    }
  }
  printf("CspadServer read histo periods\n");
  for (unsigned i=0; i<sizeOfRHisto; i++) {
    if (_rHisto[i]) {
      printf("\t%3u0 us   %8u\n", i, _rHisto[i]);
      if (c) _rHisto[i] = 0;
    }
  }
}
