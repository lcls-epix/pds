#include "pds/client/FrameCompApp.hh"

#include "pds/client/XtcStripper.hh"
#include "pds/xtc/InDatagram.hh"
#include "pds/service/Task.hh"
#include "pds/service/Timer.hh"

#include "pds/config/CsPad2x2DataType.hh"

#include "pds/config/CsPadConfigType.hh"
#include "pds/config/CsPadDataType.hh"
#include "pdsdata/psddl/cspad.ddl.h"

#include "pds/config/pnCCDConfigType.hh"
#include "pds/pnccd/FrameV0.hh"
#include "pdsdata/psddl/pnccd.ddl.h"

#include "pdsdata/compress/CompressedPayload.hh"
#include "pdsdata/compress/CompressedXtc.hh"
#include "pdsdata/psddl/camera.ddl.h"

#include "pdsdata/xtc/DetInfo.hh"

#include "pdsdata/compress/CompressedData.hh"
#include "pdsdata/compress/HistNEngine.hh"
#include "pdsdata/compress/Hist16Engine.hh"

#include "pds/vmon/VmonServerManager.hh"
#include "pds/mon/MonCds.hh"
#include "pds/mon/MonGroup.hh"
#include "pds/mon/MonEntryTH1F.hh"
#include "pds/mon/MonDescTH1F.hh"

#include <time.h>

#include <list>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

static bool lUseOMP=false;
static bool lVerbose=false;
static unsigned copyPresample=0;
static unsigned icopyPresample=0;

static double time_since(const timespec& now, const timespec& tv)
{
  double dt = double(now.tv_sec - tv.tv_sec)*1.e3;
  dt += (double(now.tv_nsec)-double(tv.tv_nsec))*1.e-6;
  return dt;
}

namespace Pds {
  namespace FCA {
    class Entry {
    public:
      Entry(Transition* tr) : _state(Completed), _type(TypeT), _ptr(tr), _copy(false), _insize(0)
      { clock_gettime(CLOCK_REALTIME,&_start); }
      Entry(InDatagram* in) : _state(in->datagram().seq.service()==TransitionId::L1Accept ||
                                     in->datagram().seq.service()==TransitionId::Configure ? Queued:Completed),
                              _type(TypeI), _ptr(in), _copy(false), _insize(in->datagram().xtc.sizeofPayload())
      { clock_gettime(CLOCK_REALTIME,&_start); 
        if (copyPresample!=0 && (++icopyPresample >= copyPresample)) {
          _copy=true;
          icopyPresample=0;
        }
      }
    public:
      bool is_complete  () const { return _state==Completed; }
      bool is_unassigned() const { return _state==Queued; }
    public:
      void assign  () { _state=Assigned; }
      void complete() { _state=Completed; clock_gettime(CLOCK_REALTIME,&_complete); }
      void post    (FrameCompApp& app) { 
        if (_type == TypeT) app.post((Transition*)_ptr);
        else                app.post((InDatagram*)_ptr);
      }
      void* ptr() { return _ptr; }
    public:
      double since_start (const timespec& now) const { return time_since(now,_start); }
      double start_to_complete() const               { return time_since(_complete,_start); }
      bool   copy() const { return _copy; }
      double compr_ratio () const {
        return _type==TypeT ? 1 : 
	  double(reinterpret_cast<InDatagram*>(_ptr)->datagram().xtc.sizeofPayload())/_insize;
      }
    private:
      enum { Queued, Assigned, Completed } _state;
      enum { TypeT, TypeI } _type;
      void* _ptr;
      bool  _copy;
      timespec _start;
      timespec _complete;
      double   _insize;
    };
    
    typedef std::list<Entry*> EList;

    class QueueTr : public Routine {
    public:
      QueueTr(Transition* tr, FrameCompApp& app) : _tr(tr), _app(app) {}
      void routine() { _app.queueTransition(_tr); delete this; }
    private:
      Transition*   _tr;
      FrameCompApp& _app;
    };

    class QueueEv : public Routine {
    public:
      QueueEv(InDatagram* in, FrameCompApp& app) : _in(in), _app(app) {}
      void routine() { _app.queueEvent(_in); delete this; }
    private:
      InDatagram*   _in;
      FrameCompApp& _app;
    };

    class ComplEv : public Routine {
    public:
      ComplEv(FCA::Entry* in,FrameCompApp& app,unsigned id) : _in(in), _app(app), _id(id) {}
      void routine() { _app.completeEntry(_in,_id); delete this; }
    private:
      FCA::Entry*   _in;
      FrameCompApp& _app;
      unsigned      _id;
    };

    class MyIter : public XtcStripper {
    public:
      enum Status {Stop, Continue};
      MyIter(Xtc* xtc, uint32_t*& p, char* obuff, size_t max_osize,bool cache) :
        XtcStripper(xtc, p), _obuff(obuff), _max_osize(max_osize), _cache(cache), _cached(false) {}
      ~MyIter() {}
      bool cached() const { return _cached; }
    protected:
      void process(Xtc*);
    private:
      char*  _obuff;
      size_t _max_osize;
      bool   _cache;
      bool   _cached;
    };

    class Task : public Routine {
    public:
      Task(unsigned id, FrameCompApp& app, size_t max_size) :
        _id(id), _app(app), 
        _task(new Pds::Task(TaskObject("FCAtsk"))),
        _entry(0),
        _obuff(new uint32_t[max_size>>2]),
        _max_size(max_size) {}
      ~Task() { _task->destroy(); delete[] _obuff; }
    public:
      void assign(FCA::Entry* e) { (_entry = e)->assign(); _task->call(this); }
      void unassign() { _entry = 0; }
      bool unassigned() const { return _entry==0; }
      void routine() {
        InDatagram* in = (InDatagram*)_entry->ptr();
        uint32_t* pdg = reinterpret_cast<uint32_t*>(&(in->datagram().xtc));
        MyIter iter(&in->datagram().xtc,pdg,(char*)_obuff,_max_size,_entry->copy());
        iter.iterate();
        if (iter.cached()) {
          Xtc* xtc = reinterpret_cast<Xtc*>(_obuff);
          in->insert(*xtc, xtc->payload());
        }
        _app.mgr_task().call(new ComplEv(_entry,_app,_id));
      }
      unsigned id() const { return _id; }
    private:
      unsigned      _id;
      FrameCompApp& _app;
      Pds::Task*    _task;
      FCA::Entry*   _entry;
      enum { MaxSize = 0x2000000 };
      uint32_t*     _obuff;
      size_t        _max_size;
    };

#ifdef _OPENMP
    class OMPCompressedXtc : public Pds::Xtc {
    public:
      enum { MaxThreads=4 };
      OMPCompressedXtc( Pds::Xtc&     xtc,
                        const std::list<unsigned>& headerOffsets,
                        unsigned headerSize,
                        unsigned depth,
                        Pds::CompressedPayload::Engine engine );
    };
#endif

    class Timer : public Pds::Timer {
    public:
      Timer(FrameCompApp& app, Pds::Task& task) : _app(app), _task(task) {}
    public:
      void expired() { _app.process(); _app.audit(); }
      Pds::Task* task  () { return &_task; }
      unsigned duration() const { return 200; }
      unsigned repetitive() const { return 1; }
    private:
      FrameCompApp& _app;
      Pds::Task&    _task;
    };
  };
};

using namespace Pds;

static std::vector<pnCCDConfigType> _configp;
static std::vector<DetInfo>         _infop;

static std::vector<CsPad::ConfigV4> _configv4;
static std::vector<DetInfo>         _infov4;

static std::vector<CsPadConfigType> _config;
static std::vector<DetInfo>         _info;

static const unsigned nbins = 64;
static const double ms_per_bin = 64./64.;
static const double rat_per_bin = 1.28/64.;
static const double evt_per_bin = 1.;

static MonEntryTH1F* _mon_entry_th1f( const char* name )
{
  MonDescTH1F desc(name,"[events]","", 32, 0., double(32)*evt_per_bin);
  return new MonEntryTH1F(desc);
}

FrameCompApp::FrameCompApp(size_t max_size, unsigned nthreads) :
  _mgr_task(new Task(TaskObject("FCAmgr"))),
  _tasks   (nthreads ? nthreads : 4),
  _timer   (new FCA::Timer(*this,*_mgr_task))
{
  MonGroup* group = new MonGroup("FCA");
  VmonServerManager::instance()->cds().add(group);

  MonDescTH1F start_to_complete("Start to Complete","[ms]", "", nbins, 0., double(nbins)*ms_per_bin);
  _start_to_complete = new MonEntryTH1F(start_to_complete);

  MonDescTH1F start_to_post("Start to Post","[ms]", "", nbins, 0., double(nbins)*ms_per_bin);
  _start_to_post = new MonEntryTH1F(start_to_post);

  MonDescTH1F compress_ratio("Compr Ratio","[fraction]", "", nbins, 0., double(nbins)*rat_per_bin);
  _compress_ratio = new MonEntryTH1F(compress_ratio);

  _queued   =_mon_entry_th1f("Queued");
  _assigned =_mon_entry_th1f("Assigned");
  _completed=_mon_entry_th1f("Completed");

  group->add(_queued);
  group->add(_assigned);
  group->add(_completed);
  group->add(_start_to_complete);
  group->add(_start_to_post);
  group->add(_compress_ratio);

  for(unsigned id=0; id<_tasks.size(); id++)
    _tasks[id] = new FCA::Task(id,*this,max_size);

  //  _timer->start();
}

FrameCompApp::~FrameCompApp()
{
  _mgr_task->destroy();
  for(unsigned id=0; id<_tasks.size(); id++)
    delete _tasks[id];
}

void FrameCompApp::useOMP(bool l) { lUseOMP=l; }
void FrameCompApp::setVerbose(bool l) { lVerbose=l; }

Transition* FrameCompApp::transitions(Transition* tr)
{
  _mgr_task->call(new FCA::QueueTr(tr,*this));
  return (Transition*)Appliance::DontDelete;
}

InDatagram* FrameCompApp::events(InDatagram* in)
{
  _mgr_task->call(new FCA::QueueEv(in,*this));
  return (InDatagram*)Appliance::DontDelete;
}

void FrameCompApp::queueTransition(Transition* tr)
{
  _list.push_back(new FCA::Entry(tr));
  process();
}

void FrameCompApp::queueEvent(InDatagram* in)
{
  if (in->datagram().seq.service()==TransitionId::Configure) {
    _config.clear();
    _info  .clear();
    _configv4.clear();
    _infov4  .clear();
    if (lVerbose)    printf("FCA::queue Configure\n");
  }
  _list.push_back(new FCA::Entry(in));
  process();
  audit();
}

void FrameCompApp::_post(FCA::Entry* e)
{
  timespec now;
  clock_gettime(CLOCK_REALTIME,&now);

  { unsigned bin = unsigned(e->start_to_complete()/ms_per_bin);
    if (bin < nbins)
      _start_to_complete->addcontent(1,bin);
    else
      _start_to_complete->addinfo(1,MonEntryTH1F::Overflow); 
  }

  { unsigned bin = unsigned(e->since_start(now)/ms_per_bin);
    if (bin < nbins)
      _start_to_post->addcontent(1,bin);
    else
      _start_to_post->addinfo(1,MonEntryTH1F::Overflow);
  }

  { unsigned bin = unsigned(e->compr_ratio()/rat_per_bin);
    if (bin < nbins)
      _compress_ratio->addcontent(1,bin);
    else
      _compress_ratio->addinfo(1,MonEntryTH1F::Overflow);
  }
    
  ClockTime time(now.tv_sec,now.tv_nsec);
  _start_to_complete ->time(time);
  _start_to_post     ->time(time);
  _compress_ratio    ->time(time);

  e->post(*this);
}

void FrameCompApp::completeEntry(FCA::Entry* e, unsigned id)
{
  e->complete();
  _tasks[id]->unassign();
  process();
}

void FrameCompApp::process()
{
  //  First post the entries in order that are complete
  while( !_list.empty() ) {
    FCA::Entry* e=_list.front();
    if (!e->is_complete()) break;

    _list.pop_front();
    _post(e);
    delete e;
  }

  //  Next assign entries in order that haven't yet been
  for(std::list<FCA::Entry*>::iterator it=_list.begin(); it!=_list.end(); it++) {
    if ((*it)->is_unassigned()) {
      bool lassign=false;
      for(unsigned id=0; id<_tasks.size(); id++)
        if (_tasks[id]->unassigned()) {
          _tasks[id]->assign(*it);
          lassign=true;
          break;
        }
      //  Too busy to process now
      //      if (!lassign) break;
      if (!lassign)
	(*it)->complete();
    }
  }
}

void FrameCompApp::audit()
{
  unsigned assigned=0, completed=0;
  //  Next assign entries in order that haven't yet been
  for(std::list<FCA::Entry*>::iterator it=_list.begin(); it!=_list.end(); it++) {
    if ((*it)->is_complete())
      completed++;
    else
      assigned++;
  }

  _queued   ->addcontent(1.,double(assigned+completed));
  _assigned ->addcontent(1.,double(assigned));
  _completed->addcontent(1.,double(completed));

  timespec now;
  clock_gettime(CLOCK_REALTIME,&now);
  ClockTime time(now.tv_sec,now.tv_nsec);

  _queued   ->time(time);
  _assigned ->time(time);
  _completed->time(time);
}



void FCA::MyIter::process(Xtc* xtc) 
{
  if (xtc->contains.id()==TypeId::Id_Xtc) {
    FCA::MyIter iter(xtc,_pwrite,_obuff,_max_osize,_cache);
    iter.iterate();
    _cached |= iter.cached();
    return;
  }

  std::list<unsigned> headerOffsets;
  unsigned headerSize = 0;
  int depth = 0;
  CompressedPayload::Engine engine = CompressedPayload::None;
  Xtc* mxtc = 0;
  
  const TypeId _FrameDataType(TypeId::Id_Frame,Camera::FrameV1::Version);
  const DetInfo& info = static_cast<const DetInfo&>(xtc->src);

  if (xtc->damage.value())
    ;
  else if (xtc->contains.value() == _FrameDataType.value()) {
    const Camera::FrameV1& frame = *reinterpret_cast<const Camera::FrameV1*>(xtc->payload());
    headerOffsets.push_back(0);
    headerSize = sizeof(Camera::FrameV1);
    depth      = (frame.depth()+7)/8;
    engine     = CompressedPayload::HistN;
  }
  else if (xtc->contains.value() == _CsPad2x2DataType.value()) {
    headerOffsets.push_back(0);
    headerSize = sizeof(CsPad2x2DataType);
    depth      = 2;
    engine     = CompressedPayload::Hist16;
  }
  else if (xtc->contains.id() == TypeId::Id_CspadElement &&
	   !xtc->contains.compressed()) {
    //
    //  We have to sparsify unwanted elements and change id
    //  from V1 to V2.
    //
    for(unsigned i=0; i<_info.size(); i++) {
      if (_info[i] == info) {
        const CsPadConfigType& cfg = _config[i];
        const unsigned quadMask = cfg.quadMask();
	bool v1 = (xtc->contains.version()==1);
	if (v1) {
	  xtc->contains = TypeId(xtc->contains.id(), 2); // change vsn
	  mxtc = reinterpret_cast<Xtc*>(_write(xtc,sizeof(*xtc)));
          const CsPad::DataV1& d = *reinterpret_cast<const CsPad::DataV1*>(xtc->payload());
	  const char* mhdr = mxtc->payload();
	  for(int q=0; q<d.quads_shape(cfg)[0]; q++) {
            const CsPad::ElementV1& e = d.quads(cfg,q);
	    if (quadMask & (1<<e.quad())) {
	      unsigned roiMask = cfg.asicMask()==1 ? 0x3 : 0xff;
	      unsigned tgtMask = cfg.roiMask(e.quad()) & roiMask;
	      if (tgtMask) {
		mhdr = (const char*)_write(&e, sizeof(e));
		headerOffsets.push_back( mhdr-mxtc->payload() );

                ndarray<const int16_t, 3> s = e.data(cfg);
                for(unsigned i=0; i<s.shape()[0]; i++) {
                  unsigned newMask = roiMask & (roiMask-1);
                  if ((roiMask ^ newMask) & tgtMask)
                    _write(&s[i][0][0],s.strides()[0]*sizeof(int16_t));
                  roiMask = newMask;
                }
		_write(s.data()+s.size(),sizeof(uint32_t));
              }
	    }
	  }
	  mxtc->extent = (const char*)_pwrite - (const char*)mxtc;
	}
	else {
          const CsPad::DataV2& d = *reinterpret_cast<const CsPad::DataV2*>(xtc->payload());
	  for(int q=0; q<d.quads_shape(cfg)[0]; q++) {
            const CsPad::ElementV2& e = d.quads(cfg,q);
	    if (quadMask & (1<<e.quad())) {
	      headerOffsets.push_back( reinterpret_cast<const char*>(&e) - xtc->payload() );
	    }
	  }
	}

        headerSize = sizeof(CsPad::ElementV2);
#ifdef _OPENMP
        depth      = lUseOMP ? -2 : 2;
#else
        depth      = 2;
#endif
        engine     = CompressedPayload::Hist16;
        break;
      }
    }

    //
    //  retry with ConfigV4
    //
    if (!depth)
      for(unsigned i=0; i<_infov4.size(); i++) {
        if (_infov4[i] == info) {
          const Pds::CsPad::ConfigV4& cfg = _configv4[i];
          const CsPad::DataV2& d = *reinterpret_cast<const CsPad::DataV2*>(xtc->payload());
          for(int q=0; q<d.quads_shape(cfg)[0]; q++) {
            const CsPad::ElementV2& e = d.quads(cfg,q);
            headerOffsets.push_back( reinterpret_cast<const char*>(&e)-xtc->payload() );
            headerSize = sizeof(CsPad::ElementV2);
#ifdef _OPENMP
            depth      = lUseOMP ? -2 : 2;
#else
            depth      = 2;
#endif
          }
          engine     = CompressedPayload::Hist16;
          break;
        }
      }

    if (!depth && lVerbose)      printf("Failed to lookup config for %08x.%08x:%sv%d\n",
                                        xtc->src.log(),xtc->src.phy(),
                                        TypeId::name(xtc->contains.id()),
                                        xtc->contains.version());
  }
  else if (xtc->contains.id() == TypeId::Id_pnCCDframe &&
           xtc->contains.version() == 0) {
    // shuffle
    for(unsigned i=0; i<_infop.size(); i++) {
      if (_infop[i] == info) {
        PNCCD::FrameV0* f = reinterpret_cast<PNCCD::FrameV0*>(xtc->payload());
        const pnCCDConfigType& cfg = _configp[i];
        headerSize = sizeof(*f);
        depth = 2;
        std::vector<PNCCD::ImageQuadrant*> p(cfg.numLinks());
        std::vector<PNCCD::ImageQuadrant*> q(cfg.numLinks());
        for (unsigned i=0;i<cfg.numLinks();i++) {
          headerOffsets.push_back( reinterpret_cast<const char*>(f)-xtc->payload() );
          unsigned iq = f->elementId();
          q[iq] = (PNCCD::ImageQuadrant*)f->data();
          p[iq] = new PNCCD::ImageQuadrant;
          f->shuffle(p[iq]);
          f->convertThisToFrameV1();
          f = f->next(cfg);
        }
        for (unsigned i=0;i<cfg.numLinks();i++) {
          memcpy(q[i], p[i], sizeof(PNCCD::ImageQuadrant));
          delete p[i];
        }
        break;
      }
    }   
  }
  else if (xtc->contains.value() == _pnCCDConfigType.value()) {
    _configp.push_back(*reinterpret_cast<const pnCCDConfigType*>(xtc->payload()));
    _infop  .push_back(static_cast<DetInfo&>(xtc->src));
  }
  else if (xtc->contains.value() == _CsPadConfigType.value()) {
    _config.push_back(*reinterpret_cast<const CsPadConfigType*>(xtc->payload()));
    _info  .push_back(static_cast<DetInfo&>(xtc->src));
    if (lVerbose)      printf("Registered config for %08x.%08x:%sv%d\n",
                              xtc->src.log(),xtc->src.phy(),
                              TypeId::name(xtc->contains.id()),
                              xtc->contains.version());
  }
  else if (xtc->contains.id() == TypeId::Id_CspadConfig && xtc->contains.version()==4) {
    _configv4.push_back(*reinterpret_cast<const CsPad::ConfigV4*>(xtc->payload()));
    _infov4  .push_back(static_cast<DetInfo&>(xtc->src));
    if (lVerbose)      printf("Registered config for %08x.%08x:%sv%d\n",
                              xtc->src.log(),xtc->src.phy(),
                              TypeId::name(xtc->contains.id()),
                              xtc->contains.version());
  }

  Xtc* cxtc = 0;
  if (depth > 0) {
    cxtc = new (_obuff) CompressedXtc(mxtc ? *mxtc : *xtc, 
                                      headerOffsets,
                                      headerSize,
                                      depth,
                                      engine);
  }
#ifdef _OPENMP
  else if (depth < 0) {
    cxtc = new (_obuff) OMPCompressedXtc(mxtc ? *mxtc : *xtc, 
                                         headerOffsets,
                                         headerSize,
                                         -depth,
                                         engine);
  }
#endif

  if (lVerbose) {
    if (cxtc) printf("Compressed %08x.%08x:%sv%d %d/%d\n",
                     xtc->src.log(),xtc->src.phy(),
                     TypeId::name(xtc->contains.id()),
                     xtc->contains.version(),
                     cxtc->sizeofPayload(),
                     xtc->sizeofPayload());
    else      printf("Did not compress %08x.%08x:%sv%d %d\n",
                     xtc->src.log(),xtc->src.phy(),
                     TypeId::name(xtc->contains.id()),
                     xtc->contains.version(),
                     xtc->sizeofPayload());
  }
  
  if (cxtc && cxtc->extent > _max_osize) {
    printf("FrameCompApp::MyIter compressed image exceeded buffer size [%d/%zd]\n",
	   cxtc->extent, _max_osize);
    cxtc = 0;
    abort();
  }
  
  if (cxtc && (cxtc->extent < xtc->extent)) {
    if (_cache) {  // keep uncompressed data
      _cached = true;
      if (!mxtc)
	_write(xtc, xtc->extent);
      return;
    }
    else {         // overwrite with compressed data
      if (mxtc) 
	_pwrite = (uint32_t*)mxtc; 
      _write(cxtc, cxtc->extent);
      return;
    }
  }
  else if (mxtc)
    return;
  
  XtcStripper::process(xtc);
}
  
#ifdef _OPENMP
FCA::OMPCompressedXtc::OMPCompressedXtc( Xtc&     xtc,
					 const std::list<unsigned>& headerOffsets,
					 unsigned headerSize,
					 unsigned depth,
					 CompressedPayload::Engine engine ) :
  Xtc( TypeId(xtc.contains.id(), xtc.contains.version(), true),
       xtc.src,
       xtc.damage )
{
  int n = headerOffsets.size();
  unsigned offsets[MaxThreads];
  char*    ibuff  [MaxThreads];
  char*    obuff  [MaxThreads];
  Compress::Hist16Engine::ImageParams img[MaxThreads];
  size_t csize[MaxThreads];
  int i=0;
  std::list<unsigned>::const_iterator it=headerOffsets.begin();
  while(it!=headerOffsets.end()) {
    offsets[i] = *it;
    ibuff  [i] = xtc.payload() + (*it) + headerSize;

    unsigned hoff  = *it;
    unsigned dsize = (++it == headerOffsets.end()) ? (char*)xtc.next()-ibuff[i] : (*it)-hoff-headerSize;

    obuff  [i] = (char*)next() + hoff + dsize*5/4;

    img[i].width  = dsize/depth;
    img[i].height = 1;
    img[i].depth  = depth;
    i++;
  }
#pragma omp parallel shared(offsets,ibuff,obuff,csize,engine) private(i) num_threads(MaxThreads)
  {
#pragma omp for schedule(dynamic,1)
    for(i=0; i<n; i++) {
      unsigned dsize = img[i].depth*img[i].width;

      if      (engine == CompressedPayload::HistN &&
	       Compress::HistNEngine().compress(ibuff[i],depth,dsize,obuff[i],csize[i]) == Compress::HistNEngine::Success)
	;
      else if (engine == CompressedPayload::Hist16 &&
	       Compress::Hist16Engine().compress(ibuff[i],img[i],obuff[i],csize[i]) == Compress::Hist16Engine::Success)
	;
      else {
	csize[i] = 0;
      }
    }
  }

  const unsigned align_mask = sizeof(uint32_t)-1;

  i = 0;
  it=headerOffsets.begin();
  while(it!=headerOffsets.end()) {
    //  copy the header
    new (alloc(sizeof(CompressedData))) CompressedData(headerSize);
    memcpy(alloc(headerSize), xtc.payload()+(*it), headerSize);
    //  copy the payload
    if (csize[i]==0) {
      unsigned dsize = img[i].depth*img[i].width;
      new (alloc(sizeof(CompressedPayload))) CompressedPayload(CompressedPayload::None,dsize,dsize);
      memcpy(alloc((dsize+align_mask)&~align_mask),ibuff[i],dsize);
    }
    else {
      unsigned dsize = img[i].depth*img[i].width;
      new (alloc(sizeof(CompressedPayload))) CompressedPayload(engine,dsize,csize[i]);
      memcpy(alloc((csize[i]+align_mask)&~align_mask),obuff[i],csize[i]);
    }
    i++;
    it++;
  }
}
#endif

void FrameCompApp::setCopyPresample(unsigned v) { copyPresample=v; }
