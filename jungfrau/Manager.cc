#include "pds/jungfrau/Manager.hh"
#include "pds/jungfrau/Driver.hh"
#include "pds/jungfrau/Server.hh"

#include "pds/config/JungfrauConfigType.hh"
#include "pds/config/JungfrauDataType.hh"
#include "pds/config/CfgClientNfs.hh"
#include "pds/client/Action.hh"
#include "pds/client/Fsm.hh"
#include "pds/utility/Appliance.hh"
#include "pds/service/GenericPool.hh"
#include "pds/service/Task.hh"
#include "pdsdata/xtc/XtcIterator.hh"

#include <errno.h>

namespace Pds {
  namespace Jungfrau {
    class FrameReader : public Routine {
    public:
      FrameReader(Driver& driver, Server& server, Task* task) :
        _task(task),
        _driver(driver),
        _server(server),
        _disable(true),
        _current_frame(-1),
        _last_frame(-1),
        _frame_sz(0),
        _buffer(0),
        _frame_ptr(0),
        _framenum_ptr(0) {}
      virtual ~FrameReader() {
        if (_buffer) delete[] _buffer;
      }
      void enable () { _disable=false; _task->call(this);}
      void disable() { _disable=true ; }
      void set_frame_sz(unsigned sz) {
        if (_buffer) {
          if (sz > _frame_sz) {
            delete[] _buffer;
            _buffer = new char[sizeof(JungfrauDataType) + sz];
          }
        } else {
          _buffer = new char[sizeof(JungfrauDataType) + sz];
        }
        _frame_sz = sz;
      }
      void routine() {
        if (_disable) {
          ;
        } else {
          _frame_ptr = (uint16_t*) (_buffer + sizeof(JungfrauDataType));
          _framenum_ptr = (uint32_t*) _buffer; 
          _current_frame = _driver.get_frame(_frame_ptr);
          if (_current_frame < 0) {
            fprintf(stderr, "Error: FrameReader failed to retrieve frame from Jungfrau receiver\n");
          } else {
            *_framenum_ptr = _current_frame;
            _server.post((char*) _buffer, sizeof(JungfrauDataType) + _frame_sz);
            if ((_last_frame >= 0) && (_current_frame != (_last_frame+1))) {
              fprintf(stderr, "Error: FrameReader frame out-of-order: got frame %d, but expected frame %d\n", _current_frame, _last_frame+1);
            }
            _last_frame = _current_frame;
          }
          _task->call(this);
        }
      }
    private:
      Task*       _task;
      Driver&     _driver;
      Server&     _server;
      bool        _disable;
      int32_t     _current_frame;
      int32_t     _last_frame;
      unsigned    _frame_sz;
      char*       _buffer;
      uint16_t*   _frame_ptr;
      uint32_t*   _framenum_ptr;
    };

    class AllocAction : public Action {
    public:
      AllocAction(CfgClientNfs& cfg) : _cfg(cfg) {}
      Transition* fire(Transition* tr) {
        const Allocate& alloc = reinterpret_cast<const Allocate&>(*tr);
        _cfg.initialize(alloc.allocation());
        return tr;
      }
    private:
      CfgClientNfs& _cfg;
    };

    class ConfigAction : public Action {
    public:
      ConfigAction(Manager& mgr, Driver& driver, Server& server, FrameReader& reader, CfgClientNfs& cfg) :
        _mgr(mgr),
        _driver(driver),
        _server(server),
        _reader(reader),
        _cfg(cfg),
        _cfgtc(_jungfrauConfigType,cfg.src()),
        _occPool(sizeof(UserMessage),1),
        _error(false) {}
      ~ConfigAction() {}
      InDatagram* fire(InDatagram* dg) {
        if (_error) {
          printf("*** Found configuration errors\n");
          dg->datagram().xtc.damage.increase(Pds::Damage::UserDefined);
        } else {
          // insert assumes we have enough space in the input datagram
          dg->insert(_cfgtc,    &_config);
        }
        return dg;
      }
      Transition* fire(Transition* tr) {
        _error = false;

        int len = _cfg.fetch( *tr, _jungfrauConfigType, &_config, sizeof(_config) );

        if (len <= 0) {
          _error = true;

          printf("ConfigAction: failed to retrieve configuration: (%d) %s.\n", errno, strerror(errno));

          UserMessage* msg = new (&_occPool) UserMessage("Jungfrau Config Error: failed to retrieve configuration.\n");
          _mgr.appliance().post(msg);
        } else {
          _cfgtc.extent = sizeof(Xtc) + sizeof(JungfrauConfigType);
          DacsConfig dacs_config(_config.vb_ds(), _config.vb_comp(), _config.vb_pixbuf(), _config.vref_ds(),
                                 _config.vref_comp(), _config.vref_prech(), _config.vin_com(), _config.vdd_prot());
          if(!_driver.configure(0, _config.gainMode(), _config.speedMode(), _config.triggerDelay(), _config.exposureTime(), _config.exposurePeriod(), _config.biasVoltage(), dacs_config) ||
              !_driver.check_size(_config.numberOfModules(), _config.numberOfRowsPerModule(), _config.numberOfColumnsPerModule())) {
            _error = true;

            printf("ConfigAction: failed to apply configuration.\n");

            UserMessage* msg = new (&_occPool) UserMessage("Jungfrau Config ");
            msg->append(_driver.error());
            _mgr.appliance().post(msg);
          }
          _server.resetCount();
          _server.set_frame_sz(_config.numberOfModules() * _config.numberOfRowsPerModule() * _config.numberOfColumnsPerModule() * sizeof(uint16_t));
          _reader.set_frame_sz(_config.numberOfModules() * _config.numberOfRowsPerModule() * _config.numberOfColumnsPerModule() * sizeof(uint16_t));
        }
        return tr;
      }
    private:
      Manager&            _mgr;
      Driver&             _driver;
      Server&             _server;
      FrameReader&        _reader;
      CfgClientNfs&       _cfg;
      JungfrauConfigType  _config;
      Xtc                 _cfgtc;
      GenericPool         _occPool;
      bool                _error;
    };

    class EnableAction : public Action {
    public:
      EnableAction(Driver& driver, FrameReader& reader): _driver(driver), _reader(reader), _error(false) { }
      ~EnableAction() { }
      InDatagram* fire(InDatagram* dg) {
        if (_error) {
          printf("EnableAction: failed to enable Jungfrau.\n");
          dg->datagram().xtc.damage.increase(Pds::Damage::UserDefined);
        }
        return dg;
      }
      Transition* fire(Transition* tr) {
        _reader.enable();
        _error = !_driver.start();  
        return tr;
      }
    private:
      Driver&       _driver;
      FrameReader&  _reader;
      bool          _error;
    };

    class DisableAction : public Action {
    public:
      DisableAction(Driver& driver, FrameReader& reader): _driver(driver), _reader(reader), _error(false) { }
      ~DisableAction() { }
      InDatagram* fire(InDatagram* dg) {
        if (_error) {
          printf("DisableAction: failed to disable Jungfrau.\n");
          dg->datagram().xtc.damage.increase(Pds::Damage::UserDefined);
        }
        return dg;
      }
      Transition* fire(Transition* tr) {
        _error = !_driver.stop();
        _reader.disable();
        return tr;
      }
    private:
      Driver&       _driver;
      FrameReader&  _reader;
      bool          _error;
    };

    class L1Action : public Action, public Pds::XtcIterator {
    public:
      L1Action() {}
      ~L1Action() {}
      InDatagram* fire(InDatagram* in) {
        _in = in;
        iterate(&in->datagram().xtc);

        return _in;
      }
      int process(Xtc* xtc) {
        if (xtc->contains.id()==TypeId::Id_Xtc)
          iterate(xtc);
        else if (xtc->contains.value() == _jungfrauDataType.value()) {
          uint32_t* ptr = (uint32_t*) xtc->payload();
          ptr[1] = _in->datagram().seq.stamp().ticks();
          ptr[2] = _in->datagram().seq.stamp().fiducials();
          return 0;
        }
        return 1;
      }
    private:
       InDatagram* _in;
    };
  }
}

using namespace Pds::Jungfrau;

Manager::Manager(Driver& driver, Server& server, CfgClientNfs& cfg) : _fsm(*new Pds::Fsm())
{
  Task* task = new Task(TaskObject("JungfrauReadout",35));
  FrameReader& reader = *new FrameReader(driver, server,task);

  _fsm.callback(Pds::TransitionId::Map, new AllocAction(cfg));
  _fsm.callback(Pds::TransitionId::Configure, new ConfigAction(*this, driver, server, reader, cfg));
  _fsm.callback(Pds::TransitionId::Enable   , new EnableAction(driver, reader));
  _fsm.callback(Pds::TransitionId::Disable  , new DisableAction(driver, reader));
  _fsm.callback(Pds::TransitionId::L1Accept , new L1Action());
}

Manager::~Manager() {}

Pds::Appliance& Manager::appliance() {return _fsm;}

