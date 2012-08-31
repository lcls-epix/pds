#ifndef Pds_EvrMasterFIFOHandler_hh
#define Pds_EvrMasterFIFOHandler_hh

#include <vector>

#include "pds/evgr/EvrFIFOHandler.hh"

/*
 * Signal handler, for processing the incoming event codes, and providing interfaces for
 *   retrieving L1 data from the L1Xmitter object
 * The Master EVR process is indicated by L1Xmitter::enable.  The master is responsible
 * for sending the EvrDatagram to the other segment levels, generating the sw triggers, 
 * adding the FIFO data to the L1Accept datagram, and counting events for calibration cycles.
 * All EVR processes configure the
 * EVRs to generate hardware triggers.  The slave EVR processes only need verify that
 * their FIFO data matches the timestamp of the L1Accept generated by the master.
 */

#include "pds/service/Client.hh"
#include "pds/service/Ins.hh"
#include "pds/utility/ToNetEb.hh"
#include "pds/service/GenericPool.hh"
#include "pds/evgr/EvrL1Data.hh"
#include "pds/evgr/EvrSync.hh"

namespace Pds {

  class Evr;
  class Appliance;
  class Src;
  class FIFOEvent;
  class InDatagram;
  class Task;
  class Transition;
  class EvrDataUtil;
  class DoneTimer;

  class EvrMasterFIFOHandler : public EvrFIFOHandler {
  public:
    enum { guNumTypeEventCode = 256 };
    enum { giMaxCommands      = 32 };
    enum { giMaxNumFifoEvent  = 32 };
    enum { giNumL1Buffers     = 32 };
    enum { TERMINATOR         = 1 };
  public:
    EvrMasterFIFOHandler(Evr&, 
       const Src&, 
       Appliance&, 
       unsigned partition,
       int      iMaxGroup,
       Task*    task);
    virtual ~EvrMasterFIFOHandler();
  public:
    virtual void        fifo_event  (const FIFOEvent&);  // formerly 'xmit'
    virtual InDatagram* l1accept    (InDatagram*);
    virtual Transition* enable      (Transition*);
    virtual Transition* disable     (Transition*);
    virtual void        set_config  (const EvrConfigType*);
    virtual Transition* config      (Transition*); // config action
    virtual Transition* endcalib    (Transition*);
    virtual void        get_sync    ();
    virtual void        release_sync();

  private:
    unsigned int          uFiducialPrev; // public data for checking fiducial increasing steps
    bool                  bShowFirstFiducial;
    bool                  bShowFiducial;
    unsigned int          uNumBeginCalibCycle;
    bool                  bEnabled;      // partition in Enabled state
    bool                  bShowFirst;
  private:
    struct EventCodeState
    {
      int  iReadout;
      bool bCommand;
      int  iDefReportDelay;
      int  iDefReportWidth;  
      int  iReportWidth;
      int  iReportDelayQ; // First-order  delay for Control-Transient events
      int  iReportDelay;  // Second-order delay for Control-Transient events; First-order delay for Control-Latch events
    };
  private:
    Evr &                 _er;
    Appliance&            _app;
    DoneTimer*            _done;
    Client                _outlet;
    std::vector<Ins>      _ldst;

    ToNetEb               _swtrig_out;
    Ins                   _swtrig_dst;
    const Src&            _src;

    GenericPool           _poolEvrData;    
    
    unsigned              _evtCounter;
    std::vector<unsigned> _lSegEvtCounter;
    unsigned              _evtStop;
    unsigned              _lastfid;
    int                   _iMaxGroup;
    unsigned              _uReadout;
    const EvrConfigType*  _pEvrConfig;
    EvrDataUtil&          _L1DataUpdated;     // codes that contribute to the coming L1Accept
    EvrDataUtil&          _L1DataLatchQ;      // codes that contribute to later L1Accepts. Holding first-order transient events.
    EvrDataUtil&          _L1DataLatch;       // codes that contribute to later L1Accepts. Holding second-order transient events and first-order latch events
    EventCodeState        _lEventCodeState[guNumTypeEventCode];
    bool                  _bEvrDataFullUpdated;
    unsigned              _ncommands;
    char                  _commands[giMaxCommands];
    EvrL1Data             _evrL1Data;

    unsigned              _lastFiducial;

    EvrSyncMaster         _sync;
    Transition*           _tr;

  private:
    void startL1Accept(const FIFOEvent& fe, bool bEvrDataIncomplete);

    int  getL1Data(int iTriggerCounter, const EvrDataUtil* & pEvrData, bool& bOutOfOrder);
    void releaseL1Data();

    // Add Fifo event to the evrData with boundary check
    int addFifoEventCheck( EvrDataUtil& evrData, const EvrDataType::FIFOEvent& fe );

    // Update Fifo event to the evrData with boundary check
    int updateFifoEventCheck( EvrDataUtil& evrData, const EvrDataType::FIFOEvent& fe );
  
    // Add a special event to the current evrData
    int addSpecialEvent( EvrDataUtil& evrData, const EvrDataType::FIFOEvent &feCur );
  
    void addCommand( const FIFOEvent& fe );

    void nextEnable();
    void clear();
    void reset();
  };
};

#endif
