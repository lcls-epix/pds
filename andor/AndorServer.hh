#ifndef ANDOR_SERVER_HH
#define ANDOR_SERVER_HH

#include <time.h>
#include <pthread.h>
#include <string>
#include <stdexcept>

#include "andor/include/atmcdLXd.h"
#include "pdsdata/xtc/DetInfo.hh"
#include "pds/config/AndorConfigType.hh"
#include "pds/xtc/InDatagram.hh"
#include "pds/xtc/Datagram.hh"
#include "pds/xtc/CDatagram.hh"
#include "pds/service/GenericPool.hh"
#include "pds/service/Routine.hh"
#include "pds/utility/EbTimeoutConstants.hh"
#include "AndorOccurrence.hh"

namespace Pds
{

class Task;
class AndorServer;

class AndorServer
{
public:
  AndorServer(int iCamera, bool bUseCaptureTask, bool bInitTest, const Src& src, std::string sConfigDb, int iSleepInt, int iDebugLevel);
  ~AndorServer();

  int   initSetup();
  int   map();
  int   config(AndorConfigType& config, std::string& sConfigWarning);
  int   unconfig();
  int   beginRun();
  int   endRun();
  int   beginCalibCycle();
  int   endCalibCycle();
  int   enable();
  int   disable();
  int   startExposure        ();
  int   getData              (InDatagram* in, InDatagram*& out);
  int   waitData             (InDatagram* in, InDatagram*& out);
  bool  isCapturingData      ();
  bool  inBeamRateMode       ();
  int   getDataInBeamRateMode(InDatagram* in, InDatagram*& out);
  AndorConfigType&
        config()  { return _config; }
  void setOccSend(AndorOccurrence* occSend);

  enum  ErrorCodeEnum
  {
    ERROR_INVALID_ARGUMENTS = 1,
    ERROR_FUNCTION_FAILURE  = 2,
    ERROR_INCORRECT_USAGE   = 3,
    ERROR_LOGICAL_FAILURE   = 4,
    ERROR_SDK_FUNC_FAIL     = 5,
    ERROR_SERVER_INIT_FAIL  = 6,
    ERROR_INVALID_CONFIG    = 7,
    ERROR_TEMPERATURE       = 8,
    ERROR_SEQUENCE_ERROR    = 9,
  };

private:
  /*
   * private enum
   */
  enum CaptureStateEnum
  {
    CAPTURE_STATE_IDLE        = 0,
    CAPTURE_STATE_RUN_TASK    = 1,
    CAPTURE_STATE_DATA_READY  = 2,
    CAPTURE_STATE_EXT_TRIGGER = 3,
  };

  /*
   * private static consts
   */
  static const int      _iMaxCoolingTime        = 100;                // in miliseconds
  static const int      _fTemperatureHiTol      = 5;                  // 5 degree Celcius
  static const int      _fTemperatureLoTol      = 200;                // 200 degree Celcius -> Do not use Low Tolerance now
  static const int      _iClockSavingExpTime    = 24*60*60*1000;      // 24 hours -> Long exposure time for clock saving
  static const int      _iFrameHeaderSize;                            // Buffer header used to store the CDatagram, Xtc and FrameV1 object
  static const int      _iMaxFrameDataSize;                           // Buffer for 4 Mega (image pixels) x 2 (bytes per pixel) +
                                                                      //   info size + header size
  static const int      _iPoolDataCount         = 120;                // to support beam rate mode
  static const int      _iMaxReadoutTime        = EB_TIMEOUT_SLOW_MS; // Max readout time
  static const int      _iMaxThreadEndTime      = EB_TIMEOUT_SLOW_MS; // Max thread terminating time (in ms)
  static const int      _iMaxLastEventTime      = EB_TIMEOUT_SLOW_MS; // Max readout time for the last event
  static const int      _iMaxEventReport        = 20;                 // Only report some statistics and non-critical errors in the first few L1 events
  static const float    _fEventDeltaTimeFactor;                 // Event delta time factor, for detecting sequence error

  /*
   * private classes
   */
  class CaptureRoutine : public Routine
  {
  public:
    CaptureRoutine(AndorServer& server);
    void routine(void);
  private:
    AndorServer& _server;
  };

  /*
   * private functions
   */

  /*
   * camera control functions
   */
  int   initDevice();
  int   deinit();
  int   printInfo();

  int   initCapture();
  int   startCapture();
  int   stopCapture();
  int   deinitCapture();

  int   initCaptureTask(); // for delay mode use only
  int   runCaptureTask();

  int   initCameraBeforeConfig();
  int   configCamera(AndorConfigType& config, std::string& sConfigWarning);

  int   initTest();
  int   setupROI();

  /*
   * Frame handling functions
   */
  int   setupFrame();
  int   waitForNewFrameAvailable();
  int   processFrame();
  int   resetFrameData(bool bDelOutDatagram);

  int   setupCooling(double fCoolingTemperature);
  int   updateTemperatureData();
  //int   checkSequence( const Datagram& datagram );

  /*
   * Initial settings
   */
  const int           _iCamera;
  const bool          _bDelayMode;
  const bool          _bInitTest;
  const Src           _src;
  const std::string   _sConfigDb;
  const int           _iSleepInt;
  const int           _iDebugLevel;

  /*
   * Camera basic status control
   */
  at_32               _hCam;
  bool                _bCameraInited;
  bool                _bCaptureInited;
  bool                _iTriggerMode; // 0: Normal (pre-open + N shot integration) 1: Ext trigger

  /*
   * Camera hardware settings
   */
  int                 _iDetectorWidth;
  int                 _iDetectorHeight;
  int                 _iImageWidth;
  int                 _iImageHeight;
  int                 _iADChannel;
  int                 _iReadoutPort;
  int                 _iMaxSpeedTableIndex;
  int                 _iMaxGainIndex;
  int                 _iTempMin;
  int                 _iTempMax;
  int                 _iFanModeNonAcq;

  /*
   * Event sequence/traffic control
   */
  float               _fPrevReadoutTime;// in seconds. Used to filter out events that are coming too fast
  bool                _bSequenceError;
  ClockTime           _clockPrevDatagram;
  int                 _iNumExposure;
  int                 _iNumAcq; // # of acquisition with trigger (reset on Enable/Disable)
  int                 _iReadoutWaitTime;

  /*
   * Config data
   */
  AndorConfigType _config;

  /*
   * Per-frame data
   */
  float               _fReadoutTime;    // in seconds

  /*
   * Buffer control
   */
  GenericPool         _poolFrameData;
  InDatagram*         _pDgOut;          // Datagram for outtputing to the Andor Manager

  /*
   * Occurrence support
   */
  AndorOccurrence*    _occSend;

  /*
   * Capture Task Control
   */
  CaptureStateEnum    _CaptureState;    // 0 -> idle, 1 -> start data polling/processing, 2 -> data ready
  Task*               _pTaskCapture;    // for delay mode use
  CaptureRoutine      _routineCapture;  // for delay mode use

  /*
   * Thread syncronization (lock/unlock) functions
   */
  inline static void lockCameraData(char* sDescription)
  {
    if ( pthread_mutex_lock(&_mutexPlFuncs) )
      printf( "AndorServer::lockCameraData(): pthread_mutex_timedlock() failed for %s\n", sDescription );
  }
  inline static void releaseLockCameraData()
  {
    pthread_mutex_unlock(&_mutexPlFuncs);
  }

  class LockCameraData
  {
  public:
    LockCameraData(char* sDescription)
    {
      lockCameraData(sDescription);
    }

    ~LockCameraData()
    {
      releaseLockCameraData();
    }
  };

  /*
   * private static data
   */
  static pthread_mutex_t _mutexPlFuncs;
};

class AndorServerException : public std::runtime_error
{
public:
  explicit AndorServerException( const std::string& sDescription ) :
    std::runtime_error( sDescription )
  {}
};

} //namespace Pds

#endif
