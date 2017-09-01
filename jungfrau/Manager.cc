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
#include "pds/service/RingPool.hh"
#include "pdsdata/xtc/XtcIterator.hh"

#include <vector>
#include <errno.h>
#include <math.h>

namespace Pds {
  namespace Jungfrau {
    class FrameReader : public Routine {
    public:
      FrameReader(Detector& detector, Server& server, Task* task) :
        _task(task),
        _detector(detector),
        _server(server),
        _disable(true),
        _current_frame(0),
        _last_frame(0),
        _first_frame(true),
        _header_sz(0),
        _frame_sz(0),
        _entry_sz(0),
        _buffer(0),
        _frame_ptr(0),
        _framenum_ptr(0),
        _metadata_ptr(0)
      {
        _header_sz = sizeof(JungfrauModuleInfoType) * _detector.get_num_modules();
        _frame_sz = _detector.get_frame_size();
        _entry_sz = sizeof(JungfrauDataType) + _header_sz + _frame_sz;
        _buffer = new char[_entry_sz];
        _server.set_frame_sz(_header_sz + _frame_sz);
      }
      virtual ~FrameReader() {
        if (_buffer) delete[] _buffer;
      }
      void enable () { _disable=false; _task->call(this);}
      void disable() { _disable=true ; }
      void routine() {
        if (_disable) {
          ;
        } else {
          _frame_ptr = (uint16_t*) (_buffer + sizeof(JungfrauDataType) + _header_sz);
          _framenum_ptr = (uint64_t*) _buffer;
          _metadata_ptr = (JungfrauModInfoType*) (_buffer + sizeof(JungfrauDataType));
          if (_detector.get_frame(&_current_frame, _metadata_ptr, _frame_ptr)) {
            *_framenum_ptr = _current_frame;
            _server.post((char*) _buffer, sizeof(JungfrauDataType) + _header_sz + _frame_sz);
            if (_first_frame) {
              _first_frame = false;
            } else if (_current_frame != (_last_frame+1)) {
              fprintf(stderr, "Error: FrameReader frame out-of-order: got frame %lu, but expected frame %lu\n", _current_frame, _last_frame+1);
            }
            _last_frame = _current_frame;
          } else {
            fprintf(stderr, "Error: FrameReader failed to retrieve frame from Jungfrau receiver\n");
          }
          _task->call(this);
        }
      }
    private:
      Task*     _task;
      Detector& _detector;
      Server&   _server;
      bool      _disable;
      uint64_t  _current_frame;
      uint64_t  _last_frame;
      bool      _first_frame;
      unsigned  _header_sz;
      unsigned  _frame_sz;
      unsigned  _entry_sz;
      char*     _buffer;
      uint16_t* _frame_ptr;
      uint64_t* _framenum_ptr;
      JungfrauModInfoType* _metadata_ptr;
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

    class L1Action : public Action, public Pds::XtcIterator {
    public:
      L1Action(unsigned num_modules) :
        _lreset(true),
        _synced(true),
        _num_modules(num_modules),
        _dgm_ts(0),
        _nfid(0),
        _mod_ts(new uint64_t[num_modules]) {}
      ~L1Action() {
        if (_mod_ts) delete[] _mod_ts;
      }
      InDatagram* fire(InDatagram* in) {
        unsigned dgm_ts = in->datagram().seq.stamp().fiducials();
        _nfid   = dgm_ts - _dgm_ts;
        if (((signed) _nfid) < 0)
          _nfid += Pds::TimeStamp::MaxFiducials;

        _in = in;
        iterate(&in->datagram().xtc);

        return _in;
      }
      void sync() { _synced=true; }
      void reset() { _lreset=true; }
      int process(Xtc* xtc) {
        if (xtc->contains.id()==TypeId::Id_Xtc)
          iterate(xtc);
        else if (xtc->contains.value() == _jungfrauDataType.value()) {
          uint32_t* ptr = (uint32_t*) (xtc->payload() + sizeof(uint64_t));
          JungfrauModInfoType* mod_info = (JungfrauModInfoType*) (xtc->payload() + sizeof(JungfrauDataType));
          ptr[0] = _in->datagram().seq.stamp().ticks();
          ptr[1] = _in->datagram().seq.stamp().fiducials();

          if (_lreset) {
            _lreset = false;
            _dgm_ts = _in->datagram().seq.stamp().fiducials();
            for (unsigned i=0; i<_num_modules; i++) {
              _mod_ts[i] = mod_info[i].timestamp();
            }
          } else {
            const double clkratio  = 360./10e6;
            const double tolerance = 0.003;  // AC line rate jitter and Jungfrau clock drift
            const unsigned maxdfid = 21600; // if there is more than 1 minute between triggers
            if (_nfid > 3) printf("nfid: %u\n", _nfid);

            for (unsigned i=0; i<_num_modules; i++) {
              double fdelta = double(mod_info[i].timestamp() - _mod_ts[i])*clkratio/double(_nfid) - 1;
              if (fabs(fdelta) > tolerance && (_nfid < maxdfid || !_synced)) {
                unsigned nfid = unsigned(double(mod_info[i].timestamp() - _mod_ts[i])*clkratio + 0.5);
                printf("  timestep error for module %u: fdelta %f  dfid %d  tds %lu,%lu [%d]\n", i, fdelta, _nfid, mod_info[i].timestamp(), _mod_ts[i], nfid);
                _synced = false;
              } else {
                _synced = true;
              }
            }

            if (!_synced) {
              _in->datagram().xtc.damage.increase(Pds::Damage::UserDefined);
            } else {
              _dgm_ts = _in->datagram().seq.stamp().fiducials();
              for (unsigned i=0; i<_num_modules; i++) {
                _mod_ts[i] = mod_info[i].timestamp();
              }
            }
          }
          return 0;
        }
        return 1;
      }
    private:
      bool        _lreset;
      bool        _synced;
      unsigned    _num_modules;
      unsigned    _dgm_ts;
      unsigned    _nfid;
      uint64_t*   _mod_ts;
      InDatagram* _in;
    };

    class ConfigAction : public Action {
    public:
      ConfigAction(Manager& mgr, Detector& detector, Server& server, FrameReader& reader, CfgClientNfs& cfg, L1Action& l1) :
        _mgr(mgr),
        _detector(detector),
        _server(server),
        _reader(reader),
        _cfg(cfg),
        _l1(l1),
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
          unsigned nrows = 0;
          unsigned ncols = 0;
          bool mod_size_set = false;

          for (unsigned i=0; i<_detector.get_num_modules(); i++) {
            if (!mod_size_set) {
              nrows = _detector.get_num_rows(i);
              ncols = _detector.get_num_columns(i);
              mod_size_set = true;
            } else if ((_detector.get_num_rows(i) != nrows) || (_detector.get_num_columns(i) != ncols)) {
              _error = true;
              printf("ConfigAction: detector modules with different shapes are not supported! (%u x %u) vs (%u x %u)\n", nrows, ncols, _detector.get_num_rows(i), _detector.get_num_columns(i));
              UserMessage* msg = new (&_occPool) UserMessage("Jungfrau Config Error: modules with different shapes are not supported!\n");
              _mgr.appliance().post(msg);
            }
          }

          if (!mod_size_set) {
            _error = true;
            printf("ConfigAction: detector seems to have no modules to configure!\n");
            UserMessage* msg = new (&_occPool) UserMessage("Jungfrau Config Error: detector seems to have no modules to configure!\n");
            _mgr.appliance().post(msg);
          }

          if (!_error) {
            JungfrauConfig::setSize(_config, _detector.get_num_modules(), nrows, ncols);
            DacsConfig dacs_config(_config.vb_ds(), _config.vb_comp(), _config.vb_pixbuf(), _config.vref_ds(),
                                   _config.vref_comp(), _config.vref_prech(), _config.vin_com(), _config.vdd_prot());
            if(!_detector.configure(0, _config.gainMode(), _config.speedMode(), _config.triggerDelay(), _config.exposureTime(), _config.exposurePeriod(), _config.biasVoltage(), dacs_config) ||
               !_detector.check_size(_config.numberOfModules(), _config.numberOfRowsPerModule(), _config.numberOfColumnsPerModule())) {
              _error = true;

              printf("ConfigAction: failed to apply configuration.\n");

              const char** errors = _detector.errors();
              for (unsigned i=0; i<_detector.get_num_modules(); i++) {
                if (strlen(errors[i])) {
                  UserMessage* msg = new (&_occPool) UserMessage("Jungfrau Config "); 
                  msg->append(errors[i]);
                  _mgr.appliance().post(msg);
                }
              }
              _detector.clear_errors();
            }
          }

          _l1.reset();
          _server.resetCount();
        }
        return tr;
      }
    private:
      Manager&            _mgr;
      Detector&           _detector;
      Server&             _server;
      FrameReader&        _reader;
      CfgClientNfs&       _cfg;
      L1Action&           _l1;
      JungfrauConfigType  _config;
      Xtc                 _cfgtc;
      GenericPool         _occPool;
      bool                _error;
    };

    class EnableAction : public Action {
    public:
      EnableAction(Detector& detector, FrameReader& reader, L1Action& l1): 
        _detector(detector),
        _reader(reader),
        _l1(l1),
        _error(false) { }
      ~EnableAction() { }
      InDatagram* fire(InDatagram* dg) {
        if (_error) {
          printf("EnableAction: failed to enable Jungfrau.\n");
          dg->datagram().xtc.damage.increase(Pds::Damage::UserDefined);
        }
        return dg;
      }
      Transition* fire(Transition* tr) {
        _l1.sync();
        _reader.enable();
        _error = !_detector.start();  
        return tr;
      }
    private:
      Detector&     _detector;
      FrameReader&  _reader;
      L1Action&     _l1;
      bool          _error;
    };

    class DisableAction : public Action {
    public:
      DisableAction(Detector& detector, FrameReader& reader): _detector(detector), _reader(reader), _error(false) { }
      ~DisableAction() { }
      InDatagram* fire(InDatagram* dg) {
        if (_error) {
          printf("DisableAction: failed to disable Jungfrau.\n");
          dg->datagram().xtc.damage.increase(Pds::Damage::UserDefined);
        }
        return dg;
      }
      Transition* fire(Transition* tr) {
        _error = !_detector.stop();
        _reader.disable();
        return tr;
      }
    private:
      Detector&     _detector;
      FrameReader&  _reader;
      bool          _error;
    };
  }
}

using namespace Pds::Jungfrau;

Manager::Manager(Detector& detector, Server& server, CfgClientNfs& cfg) : _fsm(*new Pds::Fsm())
{
  Task* task = new Task(TaskObject("JungfrauReadout",35));
  L1Action* l1 = new L1Action(detector.get_num_modules());
  FrameReader& reader = *new FrameReader(detector, server,task);

  _fsm.callback(Pds::TransitionId::Map, new AllocAction(cfg));
  _fsm.callback(Pds::TransitionId::Configure, new ConfigAction(*this, detector, server, reader, cfg, *l1));
  _fsm.callback(Pds::TransitionId::Enable   , new EnableAction(detector, reader, *l1));
  _fsm.callback(Pds::TransitionId::Disable  , new DisableAction(detector, reader));
  _fsm.callback(Pds::TransitionId::L1Accept , l1);
}

Manager::~Manager() {}

Pds::Appliance& Manager::appliance() {return _fsm;}

