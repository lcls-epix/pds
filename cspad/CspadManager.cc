/*
 * CspadManager.cc
 *
 *  Created on: Nov 15, 2010
 *      Author: jackp
 */

/*
 *
 * Note by Tomy:
 *  
 * This CspadManager has support for Cspad Compression, but the function is turned off now.
 *
 * To enable the compression, find the following two lines in this file (at two places):
 *   //_bUseCompressor(server->xtc().contains.id() == TypeId::Id_CspadElement)
 *   _bUseCompressor(false)
 *
 *  Uncomment the first line, and comment the second line to enable the compression
 *
 */
#include <sys/mman.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <new>
#include <errno.h>
#include <math.h>

#include "pds/xtc/CDatagram.hh"
#include "pds/client/Fsm.hh"
#include "pds/client/Action.hh"
#include "pds/config/CsPadConfigType.hh"
#include "pds/cspad/CspadServer.hh"
#include "pds/pgp/DataImportFrame.hh"
#include "pdsdata/xtc/DetInfo.hh"
#include "pds/cspad/CspadManager.hh"
#include "pds/config/CfgClientNfs.hh"
#include "pds/config/CfgCache.hh"
#include "pds/utility/Occurrence.hh"
#include "pds/service/GenericPool.hh"

namespace Pds {
  class CspadConfigCache : public CfgCache {
    public:
      enum {StaticALlocationNumberOfConfigurationsForScanning=100};
      CspadConfigCache(const Src& src) :
        CfgCache(src, _CsPadConfigType, StaticALlocationNumberOfConfigurationsForScanning * sizeof(CsPadConfigType)) {}
    public:
     void printRO() {
       CsPadConfigType* cfg = (CsPadConfigType*)current();
       printf("CspadConfigCache::printRO concentrator config 0x%x\n", (unsigned)cfg->concentratorVersion());
     }
    private:
      int _size(void* tc) const { return sizeof(CsPadConfigType); }
  };
}


using namespace Pds;

class CspadAllocAction : public Action {

  public:
   CspadAllocAction(CspadConfigCache& cfg)
      : _cfg(cfg) {}

   Transition* fire(Transition* tr)
   {
      const Allocate& alloc = reinterpret_cast<const Allocate&>(*tr);
      _cfg.init(alloc.allocation());
      return tr;
   }

 private:
   CspadConfigCache& _cfg;
};

class CspadEnableAction : public Action {
  public:
    CspadEnableAction(CspadServer* s) : server(s) {};

    Transition* fire(Transition* tr) {
      printf("CspadEnableAction::fire(tr) enabling front end\n");
      server->enable();
      return tr;
    }

  private:
    CspadServer* server;
};

class CspadDisableAction : public Action {
  public:
    CspadDisableAction(CspadServer* s) : server(s) {};

    Transition* fire(Transition* tr) {
      printf("CspadDisableAction::fire(tr) disabling front end\n");
      server->disable();
      return tr;
    }

  private:
    CspadServer* server;
};

class CspadBeginRunAction : public Action {
  public:
    CspadBeginRunAction(CspadServer* s) : server(s) {};

    Transition* fire(Transition* tr) {
      printf("CspadBeginRunAction::fire(tr) enabling front end\n");
      server->enable();
      return tr;
    }

  private:
    CspadServer* server;
};

class CspadEndRunAction : public Action {
  public:
    CspadEndRunAction(CspadServer* s) : server(s) {};

    Transition* fire(Transition* tr) {
      printf("CspadEndRunAction::fire(tr) disabling front end\n");
      server->disable();
      return tr;
    }

  private:
    CspadServer* server;
};

class CspadUnmapAction : public Action {
  public:
    CspadUnmapAction(CspadServer* s) : server(s) {};

    Transition* fire(Transition* tr) {
      printf("CspadUnmapAction::fire(tr) disabling front end\n");
      server->disable();
      return tr;
    }

  private:
    CspadServer* server;
};

class CspadL1Action : public Action {
 public:
   CspadL1Action(CspadServer* svr, CspadCompressionProcessor& processor, bool compress=false);

   InDatagram* fire(InDatagram* in);
   void        reset(bool f=true);

   enum {FiducialErrorCountLimit=16};

private:
   inline InDatagram* postData(InDatagram* in);
   
   CspadServer* server;
   CspadCompressionProcessor& _processor;
   unsigned _lastMatchedFiducial;
   unsigned _lastMatchedFrameNumber;
   unsigned _lastMatchedAcqCount;
   unsigned _frameSyncErrorCount;
   unsigned _ioIndex;
   unsigned _resetCount;
   unsigned _damageCount;
   bool     _bUseCompressor;   
};

void CspadL1Action::reset(bool resetError) {
  _lastMatchedFiducial = 0xffffffff;
  _lastMatchedFrameNumber = 0xffffffff;
  if (resetError) _frameSyncErrorCount = 0;
}

CspadL1Action::CspadL1Action(CspadServer* svr, CspadCompressionProcessor& processor,  bool compress) :
    server(svr),
    _processor(processor),
    _lastMatchedFiducial(0xffffffff),
    _lastMatchedFrameNumber(0xffffffff),
    _frameSyncErrorCount(0),
    _resetCount(0),
    _damageCount(0),
    //_bUseCompressor(server->xtc().contains.id() == TypeId::Id_CspadElement)
    _bUseCompressor(compress)
    {}

inline InDatagram* CspadL1Action::postData(InDatagram* in)
{
  if (_bUseCompressor)
  {
    _processor.postData(*in); 
    return (InDatagram*) Appliance::DontDelete;            
  }
  
  return in;    
}

InDatagram* CspadL1Action::fire(InDatagram* in) {
  if (server->debug() & 8) printf("CspadL1Action::fire!\n");
  Datagram& dg = in->datagram();
  Xtc* xtc = &(dg.xtc);
  unsigned evrFiducials = dg.seq.stamp().fiducials();
  unsigned vector = dg.seq.stamp().vector();
  if (in->datagram().xtc.damage.value() != 0) {
    if (_damageCount++ < 32) {
      printf("CspadL1Action::fire damage(%x) fiducials(%x) vector(%x) lastMatchedFid(%x) lastMatchedFrame(%x)\n",
          in->datagram().xtc.damage.value(), evrFiducials, vector, _lastMatchedFiducial, _lastMatchedFrameNumber);
      server->printState();
    }
  } else {
    Pds::Pgp::DataImportFrame* data;
    unsigned frameError = 0;
    char*    payload;
    if (xtc->contains.id() == Pds::TypeId::Id_Xtc) {
      xtc = (Xtc*) xtc->payload();
      if ((xtc->contains.id() == Pds::TypeId::Id_CspadElement) ) {
        payload = xtc->payload();
      } else {
        printf("CspadL1Action::fire inner xtc not Id_Cspad Element, but %s!\n",
            xtc->contains.name(xtc->contains.id()));
            
        return postData(in);
      }
    } else {
      printf("CspadL1Action::fire outer xtc not Id_Xtc, but %s!\n",
          xtc->contains.name(xtc->contains.id()));
      return postData(in);
    }

    for (unsigned i=0; i<server->numberOfQuads(); i++) {
      data = (Pds::Pgp::DataImportFrame*) ( payload + (i * server->payloadSize()) );
      if (server->payloadSize() > 500000) {     // test if this is no the mini 2x2   !!!!!!!!!!!!!!!! KLUDGE ALERT
        if (evrFiducials != data->fiducials()) {
          frameError |= 1<<data->elementId();
          if (_frameSyncErrorCount < FiducialErrorCountLimit) {
            printf("CspadL1Action::fire(in) fiducial mismatch evr(0x%x) cspad(0x%x) in quad %u,%u lastMatchedFiducial(0x%x)\n\tframeNumber(0x%x), lastMatchedFrameNumber(0x%x), ",
                evrFiducials, data->fiducials(), data->elementId(), i, _lastMatchedFiducial, data->frameNumber(), _lastMatchedFrameNumber);
            printf("acqCount(0x%x), lastMatchedAcqCount(0x%x)\n", data->acqCount(), _lastMatchedAcqCount);
          }
        }
      }
      if (!frameError) {
        _lastMatchedFiducial = evrFiducials;
        _lastMatchedFrameNumber = data->frameNumber();
        _lastMatchedAcqCount = data->acqCount();
        _frameSyncErrorCount = 0;
      }
      if (server->debug() & 0x40) printf("L1 acq - frm# %d\n", data->acqCount() - data->frameNumber());
    }
    if (frameError) {
      dg.xtc.damage.increase(Pds::Damage::UserDefined);
      dg.xtc.damage.userBits(0xf0 | (frameError&0xf));
      if (_frameSyncErrorCount++ < FiducialErrorCountLimit) {
        printf("CspadL1Action setting user damage due to frame error in quads(0x%x)\n", frameError);
        if (!_frameSyncErrorCount) server->printHisto(false);
      } else {
        if (_frameSyncErrorCount == FiducialErrorCountLimit) {
          printf("CspadL1Action::fire(in) is turning of frame error reporting until we see a match\n");
        }
      }
    }
    if (!frameError) {
      server->process();
      
      if (xtc->contains.id() != Pds::TypeId::Id_CspadElement)
      {
        printf("CspadL1Action::fire(): Unsupported type: %s\n",
            TypeId::name(xtc->contains.id()));
      }
      else
      {
        if (!_bUseCompressor)
          return postData(in);           

        _processor.compressData(*in);
        
        // processor thread will post the compressed datagram
        return (InDatagram*) Appliance::DontDelete;
    }
  }
  } // if (in->datagram().xtc.damage.value() == 0)
  
  return postData(in);  
}

class CspadConfigAction : public Action {

  public:
    CspadConfigAction( Pds::CspadConfigCache& cfg, CspadServer* server, CspadCompressionProcessor& processor, bool compress)
    : _cfg( cfg ), _server(server), _occPool(new GenericPool(sizeof(UserMessage),4)),  _processor(processor), _result(0),
      //_bUseCompressor(_server->xtc().contains.id() == TypeId::Id_CspadElement)
      _bUseCompressor(compress)
      {}

    ~CspadConfigAction() {}

    Transition* fire(Transition* tr) {
      _result = 0;
      int i = _cfg.fetch(tr);
      printf("CspadConfigAction::fire(Transition) fetched %d\n", i);
      _server->resetOffset();
      if (_cfg.scanning() == false) {
        unsigned count = 0;
        while ((_result = _server->configure( (CsPadConfigType*)_cfg.current()))
            && _result != 0xdead
            && count++<5) {

          printf("\nCspadConfigAction::fire(tr) retrying config %u\n", count);
        };
        if (_result == 0 && _server->debug() & 0x10) _cfg.printRO();
      }
      return tr;
    }

    InDatagram* fire(InDatagram* in) {
      printf("CspadConfigAction::fire(InDatagram) recorded\n");
      _cfg.record(in);
      if (_result == 0 && _server->debug() & 0x10) _cfg.printRO();
      if( _result ) {
        printf( "*** CspadConfigAction found configuration errors _result(0x%x)\n", _result );
        if (in->datagram().xtc.damage.value() == 0) {
          in->datagram().xtc.damage.increase(Damage::UserDefined);
          in->datagram().xtc.damage.userBits(_result);
        }
        char message[400];
        sprintf(message, "Cspad  on host %s failed to configure!\n", getenv("HOSTNAME"));
        UserMessage* umsg = new (_occPool) UserMessage;
        umsg->append(message);
        umsg->append(DetInfo::name(static_cast<const DetInfo&>(_server->xtc().src)));
        umsg->append("\n");
        umsg->append(_server->pgp()->errorString());
        printf("CspadConfigAction %s%s", message, _server->pgp()->errorString());
        _server->pgp()->clearErrorString();
        _server->manager()->appliance().post(umsg);
      }
      
      if (in->datagram().xtc.damage.value() == 0 && _bUseCompressor) {
        _processor.readConfig(&(in->datagram().xtc));
      }
      return in;
    }

  private:
    CspadConfigCache&   _cfg;
    CspadServer*    _server;
    GenericPool*        _occPool;
    CspadCompressionProcessor&  _processor;
  unsigned       _result;
    bool                        _bUseCompressor;    
};

class CspadBeginCalibCycleAction : public Action {
  public:
    CspadBeginCalibCycleAction(CspadServer* s, CspadConfigCache& cfg, CspadL1Action& l1) : _server(s),
    _cfg(cfg), _l1(l1), _result(0), _occPool(new GenericPool(sizeof(UserMessage),4)) {};

    Transition* fire(Transition* tr) {
      printf("CspadBeginCalibCycleAction:;fire(Transition) payload size %u ", _server->payloadSize());
      _l1.reset();
      if (_cfg.scanning()) {
        if (_cfg.changed()) {
          printf("configured and \n");
          _server->offset(_server->offset()+_server->myCount()+1);
          unsigned count = 0;
          while ((_result = _server->configure( (CsPadConfigType*)_cfg.current())) && count++<5) {
            printf("\nCspadBeginCalibCycleAction::fire(tr) retrying config %u\n", count);
          };
          if (_server->debug() & 0x10) _cfg.printRO();
        }
      }
      printf("enabled\n");
      _server->enable();
      return tr;
    }

    InDatagram* fire(InDatagram* in) {
      printf("CspadBeginCalibCycleAction:;fire(InDatagram) payload size %u ", _server->payloadSize());
      if (_cfg.scanning() && _cfg.changed()) {
        printf(" recorded\n");
        _cfg.record(in);
        if (_server->debug() & 0x10) _cfg.printRO();
      } else printf("\n");
      if( _result ) {
        printf( "*** CspadBeginCalibCycleAction found configuration errors _result(0x%x)\n", _result );
        if (in->datagram().xtc.damage.value() == 0) {
          in->datagram().xtc.damage.increase(Damage::UserDefined);
          in->datagram().xtc.damage.userBits(_result);
        }
        char message[400];
        sprintf(message, "Cspad  on host %s failed to configure!\n", getenv("HOSTNAME"));
        UserMessage* umsg = new (_occPool) UserMessage;
        umsg->append(message);
        umsg->append(DetInfo::name(static_cast<const DetInfo&>(_server->xtc().src)));
        umsg->append("\n");
        umsg->append(_server->pgp()->errorString());
        printf("CspadBeginCalibCycleAction %s%s", message, _server->pgp()->errorString());
        _server->pgp()->clearErrorString();
        _server->manager()->appliance().post(umsg);
      }
      return in;
    }
  private:
    CspadServer*      _server;
    CspadConfigCache& _cfg;
    CspadL1Action&    _l1;
    unsigned          _result;
    GenericPool*        _occPool;
};


class CspadUnconfigAction : public Action {
 public:
   CspadUnconfigAction( CspadServer* server, CspadConfigCache& cfg ) : _server( server ), _cfg(cfg) {}
   ~CspadUnconfigAction() {}

   Transition* fire(Transition* tr) {
     printf("CspadUnconfigAction:;fire(Transition) unconfigured\n");
     _result = _server->unconfigure();
     if (!(_server->debug() & 0x2000)) _server->dumpFrontEnd();
     return tr;
   }

   InDatagram* fire(InDatagram* in) {
     printf("CspadUnconfigAction:;fire(InDatagram)\n");
      if( _result ) {
         printf( "*** Found %d cspad Unconfig errors\n", _result );
         in->datagram().xtc.damage.increase(Pds::Damage::UserDefined);
         in->datagram().xtc.damage.userBits(0xda);
      }
      return in;
   }

 private:
   CspadServer*      _server;
   CspadConfigCache& _cfg;
   unsigned          _result;
};

class CspadEndCalibCycleAction : public Action {
  public:
    CspadEndCalibCycleAction(CspadServer* s, CspadConfigCache& cfg) : _server(s), _cfg(cfg), _result(0) {};

    Transition* fire(Transition* tr) {
      printf("CspadEndCalibCycleAction:;fire(Transition) %p", _cfg.current());
      _cfg.next();
      printf(" %p\n", _cfg.current());
      _result = _server->unconfigure();
      if (_server->debug() & 0x2000) _server->dumpFrontEnd();
      _server->printHisto(true);
      return tr;
    }

    InDatagram* fire(InDatagram* in) {
      printf("CspadEndCalibCycleAction:;fire(InDatagram)\n");
      if( _result ) {
        printf( "*** CspadEndCalibCycleAction found configuration errors _result(0x%x)\n", _result );
        if (in->datagram().xtc.damage.value() == 0) {
          in->datagram().xtc.damage.increase(Damage::UserDefined);
          in->datagram().xtc.damage.userBits(_result);
        }
      }
      return in;
    }

  private:
    CspadServer*      _server;
    CspadConfigCache& _cfg;
    unsigned          _result;
};

class AppProcessor : public Appliance
{
public:  
  AppProcessor() {}
  virtual Transition* transitions(Transition* tr) {return tr;}
  virtual InDatagram* events     (InDatagram* in) {return in;}
};

CspadManager::CspadManager( CspadServer* server, int d, bool c) :
    _fsm(*new Fsm), _cfg(*new CspadConfigCache(server->client())),
    _appProcessor(*new AppProcessor()), _compressionProcessor(_appProcessor, 14, 32, server->debug())
{

   printf("CspadManager being initialized... " );

   server->manager(this);

   CspadL1Action* l1 = new CspadL1Action( server, _compressionProcessor, c );
   _fsm.callback( TransitionId::L1Accept, l1 );

   _fsm.callback( TransitionId::Map,             new CspadAllocAction( _cfg ) );
   _fsm.callback( TransitionId::Unmap,           new CspadUnmapAction( server ) );
   _fsm.callback( TransitionId::Configure,       new CspadConfigAction(_cfg, server, _compressionProcessor, c ) );
   _fsm.callback( TransitionId::Enable,          new CspadEnableAction( server ) );
   _fsm.callback( TransitionId::Disable,         new CspadDisableAction( server ) );
   _fsm.callback( TransitionId::BeginCalibCycle, new CspadBeginCalibCycleAction( server, _cfg, *l1 ) );
   _fsm.callback( TransitionId::EndCalibCycle,   new CspadEndCalibCycleAction( server, _cfg ) );
   _fsm.callback( TransitionId::Unconfigure,     new CspadUnconfigAction( server, _cfg ) );
   _fsm.callback( TransitionId::BeginRun,        new CspadBeginRunAction( server ) );
   _fsm.callback( TransitionId::EndRun,          new CspadEndRunAction( server ) );
}
