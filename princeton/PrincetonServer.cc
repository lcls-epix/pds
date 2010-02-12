#include "PrincetonServer.hh" 
#include "PrincetonUtils.hh"

#include "pds/xtc/Datagram.hh"
#include "pds/xtc/CDatagram.hh"

#include <errno.h>
#include <pthread.h> 
 
using std::string;

using PICAM::printPvError;

namespace Pds 
{

using namespace Princeton;

PrincetonServer::PrincetonServer(bool bUseCaptureThread, bool bStreamMode, std::string sFnOutput, const Src& src, int iDebugLevel) :
 _bUseCaptureThread(bUseCaptureThread), _bStreamMode(bStreamMode), _sFnOutput(sFnOutput), _src(src), _iDebugLevel(iDebugLevel),
 _hCam(-1), _bCameraInited(false), _bCaptureInited(false), _iThreadStatus(0), _iThreadCommand(0),
 _configCamera(), 
 _iCurShotId(-1), _fReadoutTime(0),  
  _poolEmptyData( sizeof(CDatagram), 1 ), _poolFrameData(_iMaxFrameDataSize, _iPoolDataCount), _pCurDatagram(0),
 _iEventCaptureEnd(0), _pDgOut(NULL)
{  
  if ( checkInitSettings() != 0 )    
    throw PrincetonServerException( "PrincetonServer::PrincetonServer(): Invalid initial settings" );
      
  if ( initCamera() != 0 )
    throw PrincetonServerException( "PrincetonServer::PrincetonServer(): initPrincetonCamera() failed" );    

  /*
   * Known issue:
   *
   * If initCaptureThread() is put here, occasionally we will miss some FRAME_COMPLETE event when we do the polling
   * Possible reason is that this function (constructor) is called by a different thread other than the polling thread.
   */    
}

PrincetonServer::~PrincetonServer()
{    
  /*
   * wait for montior and capture thread (if exists) to terminate
   */ 
  while ( _iThreadStatus > 0 )
  {
    static int        iTotalWaitTime  = 0;
    static const int  iSleepTime      = 10000; // 10 ms
    
    _iThreadCommand = 1; // notify monitor and capture threads to terminate
        
    timeval timeSleepMicro = {0, 10000}; 
    // use select() to simulate nanosleep(), because experimentally select() controls the sleeping time more precisely
    select( 0, NULL, NULL, NULL, &timeSleepMicro);    
    
    iTotalWaitTime += iSleepTime / 1000;
    if ( iTotalWaitTime >= _iMaxThreadEndTime )
    {
      printf( "PrincetonServer::~PrincetonServer(): timeout for waiting thread terminating\n" );
      break;
    }
  }
    
  deinitCamera();  
}

int PrincetonServer::initCamera()
{
  if ( _bCameraInited )
    return 0;
    
  /* Initialize the PVCam Library */
  if (!pl_pvcam_init())
  {
    printPvError("PrincetonServer::initCamera(): pl_pvcam_init() failed");
    return 1;
  }

  char strCamera[CAM_NAME_LEN];  /* camera name                    */
  
  /* Assume each machine only control one camera */
  if (!pl_cam_get_name(0, strCamera))
  {
    printPvError("PrincetonServer::initCamera(): pl_cam_get_name(0) failed");
    return 2;
  }
  
  if (!pl_cam_open(strCamera, &_hCam, OPEN_EXCLUSIVE))
  {
    printPvError("PrincetonServer::initCamera(): pl_cam_open() failed");
    return 3;
  }
  
  /* Initialize capture related functions */    
  if (!pl_exp_init_seq())
  {
    printPvError("PrincetonServer::initCamera(): pl_exp_init_seq() failed!\n");
    return 4; 
  }  
  
  printf( "Princeton Camera %s has been initialized\n", strCamera );
  
  _bCameraInited = true;    
  return 0;
}

int PrincetonServer::deinitCamera()
{  
  // If the camera has been init-ed before, and not deinit-ed yet  
  if ( _bCaptureInited )
    deinitCapture(); // deinit the camera explicitly    
  
  /* Uninit capture related functions */  
  if (!pl_exp_uninit_seq() ) 
  {
    printPvError("PrincetonServer::deinitCamera():pl_exp_uninit_seq() failed");
    return 1;
  }
    
  if (!pl_cam_close(_hCam))
    printPvError("PrincetonServer::deinitCamera(): pl_cam_close() failed"); // Don't return here; continue to uninit the library

  if (!pl_pvcam_uninit())
  {
    printPvError("PrincetonServer::deinitCamera(): pl_pvcam_uninit() failed");
    return 2;
  }
    
  printf( "Princeton Camera has been deinitialized\n" );
  
  return 0;
}

int PrincetonServer::mapCamera()
{
  /*
   * Thread Issue:
   *
   * initCaptureThread() need to be put here. See the comment in the constructor function.
   */      
  if ( initCaptureThread() != 0 ) // !! debug
    return 1;
  
  return 0;
}

int PrincetonServer::configCamera(Princeton::ConfigV1& config)
{
  // If the camera has been init-ed before, and not deinit-ed yet  
  if ( _bCaptureInited )
    deinitCapture(); // deinit the camera explicitly    

  //if ( initCamera() != 0 ) 
  //  return 1;
    
  if ( initCameraSettings(config) != 0 ) 
    return 2;
  
  _configCamera = config;
  FrameV1::initStatic( _configCamera ); // associate frame objects with this camera config
    
  if ( setupCooling() != 0 )
    return 3;  
  
  if ( initCapture() != 0 )
    return 4; 
    
  //if ( initCaptureThread() != 0 )
  //  return 5;
  
  return 0;
}

int PrincetonServer::unconfigCamera()
{
  return deinitCapture();
}

int PrincetonServer::beginRunCamera()
{
  /*
   * Camera control & DAQ issue:
   *
   * if we call initCapture() here, sometimes the camera will not be ready for the first few events
   * Hence we call initCapture() in configCamera() instead
   */    
  return 0;
}

int PrincetonServer::endRunCamera()
{
  return resetFrameData();
}

int PrincetonServer::initCapture()
{ 
  if ( _bCaptureInited )
    deinitCapture();
    
  lockCameraData("PrincetonServer::initCapture()");    
    
  rgn_type region;  
  setupROI(region);
  PICAM::printROI(1, &region);
 
  const int16 iExposureMode = _configCamera.exposureMode();
  
  /*
   * _configCamera.exposureTime() time is measured in seconds,
   *  while iExposureTime for pl_exp_setup() is measured in milliseconds
   *
   * Note: this value will be ignored by pl_exp_setup(), if exposure mode = BULB_MODE
   */
  const uns32 iExposureTime = (int) ( _configCamera.exposureTime() * 1000 ); 

  uns32 uFrameSize;
  rs_bool bStatus = 
   pl_exp_setup_seq(_hCam, 1, 1, &region, iExposureMode, iExposureTime, &uFrameSize);
  if (!bStatus)
  {
    printPvError("PrincetonServer::initCapture(): pl_exp_setup_seq() failed!\n");
    releaseLockCameraData();
    return 2;
  }
  printf( "Frame size for image capture = %lu\n", uFrameSize );  
    
  if ( (int)uFrameSize + _iFrameHeaderSize > _iMaxFrameDataSize )
  {
    printf( "PrincetonServer::initCapture():Frame size (%lu) + Frame header size (%d) "
     "is larger than internal data frame buffer size (%d)\n",
     uFrameSize, _iFrameHeaderSize, _iMaxFrameDataSize );
    releaseLockCameraData();
    return 3;    
  }
     
  _bCaptureInited = true;
  
  printf( "Capture initialized, exposure mode = %d, time = %lu\n",
    iExposureMode, iExposureTime );
  
  releaseLockCameraData();    
  return 0;
}

int PrincetonServer::deinitCapture()
{
  if ( !_bCaptureInited )
    return 0;

  resetFrameData();
    
  lockCameraData( "PrincetonServer::deinitCapture()" );
  _bCaptureInited = false;
  
  /* Stop the acquisition */
  rs_bool bStatus = 
   pl_exp_abort(_hCam, CCS_HALT);
  
  if (!bStatus)
  {
    printPvError("PrincetonServer::deinitCapture():pl_exp_abort() failed");
    releaseLockCameraData();  
    return 1;
  }    
    
  printf( "Capture deinitialized\n" );
  
  releaseLockCameraData();   
  return 0;
}

int PrincetonServer::startCapture()
{
  if ( _poolFrameData.numberOfFreeObjects() <= 0 )
  {
    printf( "PrincetonServer::startCapture(): Pool is full, and cannot provide buffer for new image data\n" );
    return 1;
  }
  
  _pCurDatagram = (unsigned char*) ( _poolFrameData.alloc(0) );
  Pool::free( _pCurDatagram );
  
  /* Start the acquisition */
  lockCameraData( "PrincetonServer::startCapture(): pl_exp_start_seq()" );
  rs_bool bStatus = 
   pl_exp_start_seq(_hCam, _pCurDatagram );
  releaseLockCameraData();
  if (!bStatus)
  {
    printPvError("PrincetonServer::startCapture():pl_exp_start_seq() failed");    
    _pCurDatagram = NULL;
    return 1;        
  }  
  
  return 0;
}

int PrincetonServer::initCameraSettings(Princeton::ConfigV1& config)
{
  if ( config.orgX() + config.width() > Princeton::ConfigV1::uPI_MTE_2048_Width 
    || config.orgY() + config.height() > Princeton::ConfigV1::uPI_MTE_2048_Height
    || config.binX() >= Princeton::ConfigV1::uPI_MTE_2048_Width / 32 
    || config.binY() >= Princeton::ConfigV1::uPI_MTE_2048_Height / 32 )
  {
    printf( "PrincetonServer::validateCameraSettings(): Config parameters are invalid\n" );
    return 1;
  }
  
  uns32 uTriggerEdge = EDGE_TRIG_POS;
  PICAM::setAnyParam(_hCam, PARAM_EDGE_TRIGGER, &uTriggerEdge );  
  
  using PICAM::displayParamIdInfo;
  displayParamIdInfo(_hCam, PARAM_EXPOSURE_MODE,    "Exposure Mode");
  displayParamIdInfo(_hCam, PARAM_CLEAR_MODE,       "Clear Mode");
  displayParamIdInfo(_hCam, PARAM_SHTR_OPEN_MODE,   "Shutter Open Mode");
  displayParamIdInfo(_hCam, PARAM_SHTR_OPEN_DELAY,  "Shutter Open Delay");
  displayParamIdInfo(_hCam, PARAM_SHTR_CLOSE_DELAY, "Shutter Close Delay");
    
  displayParamIdInfo(_hCam, PARAM_EXP_RES,          "Exposure Resolution");
  displayParamIdInfo(_hCam, PARAM_EXP_RES_INDEX,    "Exposure Resolution Index");
  
  displayParamIdInfo(_hCam, PARAM_EDGE_TRIGGER,     "Edge Trigger" );
  
  return 0;
}

int PrincetonServer::setupCooling()
{
  using namespace PICAM;
  // Cooling settings
  
  displayParamIdInfo(_hCam, PARAM_COOLING_MODE, "Cooling Mode");

  //displayParamIdInfo(_hCam, PARAM_TEMP_SETPOINT, "Set Cooling Temperature *Org*");  
  const int16 iCoolingTemp = _configCamera.coolingTemp();
  setAnyParam(_hCam, PARAM_TEMP_SETPOINT, &iCoolingTemp );
  displayParamIdInfo(_hCam, PARAM_TEMP_SETPOINT, "Set Cooling Temperature" );   

  displayParamIdInfo(_hCam, PARAM_TEMP, "Temperature Before Cooling" );  
  
  const static timeval timeSleepMicroOrg = {0, 1000}; // 1 millisecond    
  timespec timeVal1;
  clock_gettime( CLOCK_REALTIME, &timeVal1 );      
  
  int16 iTemperatureCurrent = -1;  
  int iNumLoop = 0;
  
  while (1)
  {  
    getAnyParam(_hCam, PARAM_TEMP, &iTemperatureCurrent );
    if ( iTemperatureCurrent <= iCoolingTemp ) break;
    
    if ( (iNumLoop+1) % 1000 == 0 )
      displayParamIdInfo(_hCam, PARAM_TEMP, "Temperature *Updating*" );
    
    if ( iNumLoop > _iMaxCoolingTime ) break;
    
    // This data will be modified by select(), so need to be reset
    timeval timeSleepMicro = timeSleepMicroOrg; 
    // use select() to simulate nanosleep(), because experimentally select() controls the sleeping time more precisely
    select( 0, NULL, NULL, NULL, &timeSleepMicro);    
    iNumLoop++;
  }
  
  timespec timeVal2;
  clock_gettime( CLOCK_REALTIME, &timeVal2 );
  double fCoolingTime = (timeVal2.tv_nsec - timeVal1.tv_nsec) * 1.e-6 + ( timeVal2.tv_sec - timeVal1.tv_sec ) * 1.e3;    
  printf("Cooling Time = %6.3lf ms\n", fCoolingTime);  
  
  displayParamIdInfo(_hCam, PARAM_TEMP, "Temperature After Cooling" );
  
  if ( iTemperatureCurrent > iCoolingTemp ) 
  {
    printf("PrincetonServer::setupCooling(): Cooling failed, final temperature = %lf degree Celcius", 
     (iTemperatureCurrent / 100.0) );
    return 1;
  }
  
  return 0;
}

int PrincetonServer::initCaptureThread()
{
  if ( _iThreadStatus > 0 )
    return 0;
    
  if ( ! _bUseCaptureThread )
  {
    _iThreadStatus = 0;
    return 0;
  }    
    
  pthread_attr_t threadAttr;
  pthread_attr_init(&threadAttr);
  pthread_attr_setdetachstate(&threadAttr, PTHREAD_CREATE_DETACHED);

  pthread_t threadControl;
      
  int iFail = 
   pthread_create(&threadControl, &threadAttr, &threadEntryCapture, this);   
  if (iFail != 0)
  {
    printf("PrincetonServer::initCaptureThread(): pthread_create(threadEntryCapture) failed, error code %d - %s\n",
       errno, strerror(errno));
    return 1;
  }  
  
  _iThreadStatus = 1;
  return 0;
}

int PrincetonServer::runCaptureThread()
{
  const static timeval timeSleepMicroOrg = {0, 1000}; // 1 milliseconds  
  
  while ( _iThreadCommand == 0 ) // if _iThreadCommand is set to non-zero value by other thread, this thread will be terminate
  { 
    // !! use condition variable
    if ( _iEventCaptureEnd == 0 )
    {
      // This data will be modified by select(), so need to be reset
      timeval timeSleepMicro = timeSleepMicroOrg; 
      // use select() to simulate nanosleep(), because experimentally select() controls the sleeping time more precisely
      select( 0, NULL, NULL, NULL, &timeSleepMicro);      
      continue;      
    }
        
    if ( !_bCameraInited )
      continue;    
    
    if ( _pDgOut != NULL )
    {
      printf( "PrincetonServer::runCaptureThread(): Last data has not been sent out, while new readout is requested\n" );
      
      /*
       * Delete the current datagram to prepare for the new image data
       *
       * Note that getDelayData() makes sure _pDgOut is set to NULL as soon as its data is being sent out,
       * so we don't worry if _pDgOut is being used by other thread.
       */
      lockCameraData( "PrincetonServer::runCaptureThread(): Check DgOut" );
      delete _pDgOut;
      _pDgOut = NULL;
      releaseLockCameraData();
    }
    
    if ( !_bCaptureInited )
    {
      _iEventCaptureEnd = 0;
      continue;
    }
        
    /*
     * Check if shot Id has been reset
     * 
     * This can happens if resetFrameData() is called
     */
    if ( _iCurShotId < 0 )
    {      
      _iEventCaptureEnd = 0;
      continue;
    }
 
    CDatagram*  pCdgEvent = new (&_poolEmptyData) CDatagram( TypeId(), Src() );
    InDatagram* in        = pCdgEvent;
    InDatagram* out       = in;
    
    int iFail;
    iFail =
     waitForNewFrameAvailable();
     
    if ( iFail == 0 )
    {
        iFail = 
         processFrame(in, out);       
    }    
          
    if ( out != in )
      delete in; // release the space of input datagram for next use
    
    /*
     * Set the damage bit if temperature status is not good
     */
    if ( iFail != 0 || checkTemperature() != 0 )
      out->datagram().xtc.damage.increase(Pds::Damage::UserDefined);      
    
    lockCameraData( "PrincetonServer::runCaptureThread(): Check capture event before outtputting" );
    if ( _iEventCaptureEnd == 0 )
    {
      delete out;
      _pDgOut =  NULL;
      releaseLockCameraData();
      continue;
    }
    
    if ( _bStreamMode )
      _pDgOut = out;
    else
    {
      writeFrameToFile( out->datagram() );      
      delete out;
    }
      
    releaseLockCameraData();
    
    /* 
     * Reset the frame data
     */
    _iCurShotId = -1;      
    
    _iEventCaptureEnd = 0;
  } //   while ( _iThreadCommand == 0 )   
  
  
  lockCameraData( "PrincetonServer::runCaptureThread(): Decrease thread end counter" );
  _iThreadStatus--;
  releaseLockCameraData();
  
  printf( "Capture thread terminated\n" );
  return 0;
}

int PrincetonServer::onEventReadoutPrompt(int iShotId, InDatagram* in, InDatagram*& out)
{
  if ( iShotId < 0 )
  {
    printf( "PrincetonServer::onEventReadoutPrompt(): Illegal input, shot id (%d) < 0\n", iShotId );
    return 1;
  }
      
  out = in; // default: return empty stream
  
  if ( startCapture() != 0 )
    return 2;  
      
  if ( waitForNewFrameAvailable() != 0 )
  {
    _iCurShotId = -1;
    return 3;
  }
      
  _iCurShotId  = iShotId;  
  if ( processFrame( in, out ) != 0 )
  {
    _iCurShotId = -1;    
    return 4;
  }

  /* 
   * Reset the frame data
   */
  _iCurShotId = -1;
    
  /*
   * Check temperature
   */  
  if ( checkTemperature() != 0 )  
    return 4;    
      
  return 0;
}

int PrincetonServer::onEventReadoutDelay(int iShotId)
{
  if ( iShotId < 0 )
  {
    printf( "PrincetonServer::onEventReadoutDelay(): Illegal input, shot id (%d) < 0\n", iShotId );
    return 1;
  }
  
  if ( _iThreadStatus == 0  )
  {
    printf( "PrincetonServer::onEventReadoutDelay(): No capture thread running for image acquisition\n" );    
    _iCurShotId = -1;
    return 2;
  }

  if ( _pDgOut != NULL )
  {
    // The data is ready, but has not been transfered out yet
    return 0;
  }
  
  if ( _iEventCaptureEnd == 1  )
  {         
    // The capture thread is still processing the data
    return 0;
  }
  
  if ( startCapture() != 0 )
    return 3;
    
  _iCurShotId       = iShotId;
  _iEventCaptureEnd = 1;  // Use event trigger "_iEventCaptureEnd" to notify the thread
  return 0;
}

int PrincetonServer::getDelayData(InDatagram* in, InDatagram*& out)
{
  out = in; // default: return empty stream

  /*
   * This function is only useful when _bUseCaptureThread = true and bStreamMode = true
   */
  if ( !_bUseCaptureThread || !_bStreamMode ) 
    return 1;  
    
  if ( _pDgOut == NULL )
    return 0;
    
  Datagram& dgIn    = in->datagram();
  Datagram& dgOut   = _pDgOut->datagram();
  
  /*
   * Backup the orignal Xtc data
   *
   * Note that it is not correct to use  Xtc xtc1 = xtc2, which means using the Xtc constructor,
   * instead of the copy operator to do the copy.
   */
  Xtc xtcOrg; 
  xtcOrg = dgOut.xtc;

  /*
   * Compose the datagram
   *
   *   1. Use the header from dgIn 
   *   2. Use the xtc (data header) from dgOut
   *   3. Use the data from dgOut (the data was located right after the xtc)
   */
  dgOut = dgIn;   
  dgOut.xtc = xtcOrg;
      
  out = _pDgOut;    
  
  // After sending out the data, set the _pDgOut pointer to NULL, so that the same data will never be sent out twice
  _pDgOut = NULL;  
  
  return 0;
}

int PrincetonServer::getLastDelayData(InDatagram* in, InDatagram*& out)
{
  out = in; // default: return empty stream

  /*
   * This function is only useful when _bUseCaptureThread = true and bStreamMode = true
   */
  if ( !_bUseCaptureThread || !_bStreamMode ) 
    return 1;  
  
  if ( _pDgOut == NULL && _iEventCaptureEnd == 0 )
    return 0;
  
  static timespec tsWaitStart;
  clock_gettime( CLOCK_REALTIME, &tsWaitStart );
  
  const static timeval timeSleepMicroOrg = {0, 1000}; // 1 milliseconds
  
  while (_pDgOut == NULL)
  {
    // This data will be modified by select(), so need to be reset
    timeval timeSleepMicro = timeSleepMicroOrg; 
    // use select() to simulate nanosleep(), because experimentally select() controls the sleeping time more precisely
    select( 0, NULL, NULL, NULL, &timeSleepMicro); 
        
          
    timespec tsCurrent;
    clock_gettime( CLOCK_REALTIME, &tsCurrent );
    
    int iWaitTime = (tsCurrent.tv_nsec - tsWaitStart.tv_nsec) / 1000000 + 
     ( tsCurrent.tv_sec - tsWaitStart.tv_sec ) * 1000; // in milliseconds
    if ( iWaitTime >= _iMaxLastEventTime )
    {
      printf( "PrincetonServer::getLastDelayData(): Waiting time is too long. Skip the final data\n" );          
      return 3;
    }
  } // while (1)
      
  Datagram& dgIn    = in->datagram();
  Datagram& dgOut   = _pDgOut->datagram();
  
  /*
   * Backup the orignal Xtc data
   *
   * Note that it is not correct to use  Xtc xtc1 = xtc2, which means using the Xtc constructor,
   * instead of the copy operator to do the copy.
   */
  Xtc xtcOrg; 
  xtcOrg = dgOut.xtc;

  /*
   * Compose the datagram
   *
   *   1. Use the header from dgIn 
   *   2. Use the xtc (data header) from dgOut
   *   3. Use the data from dgOut (the data was located right after the xtc)
   */
  dgOut = dgIn;   
  dgOut.xtc = xtcOrg;
      
  out = _pDgOut;    
  
  // After sending out the data, set the _pDgOut pointer to NULL, so that the same data will never be sent out twice
  _pDgOut = NULL;  
  
  return 0;
}

int PrincetonServer::waitForNewFrameAvailable()
{   
  static timespec tsWaitStart;
  clock_gettime( CLOCK_REALTIME, &tsWaitStart );
  
  const static timeval timeSleepMicroOrg = {0, 1000}; // 1 milliseconds
  
  while (1)
  {
    int16 iStatus = 0;
    uns32 uNumBytesTransfered;
    rs_bool bCheckOk;
    
    lockCameraData("PrincetonServer::waitForNewFrameAvailable():PrincetonServer::pl_exp_check_status()");  
    bCheckOk = 
     pl_exp_check_status(_hCam, &iStatus, &uNumBytesTransfered);
    releaseLockCameraData();  
    
    if (!bCheckOk)
    {
      if ( pl_error_code() == 0 )
        return 2;
      
      printPvError("PrincetonServer::waitForNewFrameAvailable(): pl_exp_check_status() failed\n");
      return 3;
    }
    
    /*
     * READOUT_COMPLETE=FRAME_AVAILABLE, according to PVCAM 2.7 library manual
     */
    if (iStatus == READOUT_COMPLETE) // New frame available
      break;

    if (iStatus == READOUT_FAILED)
    {
      printf("PrincetonServer::waitForNewFrameAvailable(): pl_exp_check_status() return status=READOUT_FAILED\n");      
      return 4;
    }

    if (iStatus == READOUT_NOT_ACTIVE) // idle state: camera controller won't transfer any data
    {
      printf("PrincetonServer::waitForNewFrameAvailable(): pl_exp_check_status() return status=READOUT_NOT_ACTIVE\n");      
      return 5;
    }
    
    /*
     * sleep and do the next iteration, for the cases:
     * 
     * READOUT_IN_PROGRESS, EXPOSURE_IN_PROGRESS, ACQUISITION_IN_PROGRESS
     */
     
    // This data will be modified by select(), so need to be reset
    timeval timeSleepMicro = timeSleepMicroOrg; 
    // use select() to simulate nanosleep(), because experimentally select() controls the sleeping time more precisely
    select( 0, NULL, NULL, NULL, &timeSleepMicro); 
        
          
    timespec tsCurrent;
    clock_gettime( CLOCK_REALTIME, &tsCurrent );
    
    int iWaitTime = (tsCurrent.tv_nsec - tsWaitStart.tv_nsec) / 1000000 + 
     ( tsCurrent.tv_sec - tsWaitStart.tv_sec ) * 1000; // in milliseconds
    if ( iWaitTime >= _iMaxReadoutTime )
    {
      printf( "PrincetonServer::waitForNewFrameAvailable(): Readout time is longer than %d miliseconds. Capture is stopped\n",
       _iMaxReadoutTime );    
      return 6;
    }     
  } // while (1)

  timespec tsWaitEnd;
  clock_gettime( CLOCK_REALTIME, &tsWaitEnd );
  
  _fReadoutTime = (tsWaitEnd.tv_nsec - tsWaitStart.tv_nsec) / 1.0e9 + ( tsWaitEnd.tv_sec - tsWaitStart.tv_sec ); // in seconds
    
  return 0;
}

int PrincetonServer::processFrame(InDatagram* in, InDatagram*& out)
{ 
  lockCameraData("PrincetonServer::processFrame()");  
  
  // default to return the empty (original) datagram without data
  out = in; 
  
  /*
   * Set frame object
   */    
  unsigned char*  pXtcHeader    = _pCurDatagram + sizeof(CDatagram);
  unsigned char*  pFrameHeader  = pXtcHeader + sizeof(Xtc);  
  int             iFrameSize    = FrameV1::size();

  if ( _poolFrameData.numberOfFreeObjects() <= 0 )
  {
    printf( "PrincetonServer::processFrame(): Pool is full, and cannot provide buffer for new datagram\n" );
    return 1;
  }
 
  out = 
   new ( &_poolFrameData ) CDatagram( in->datagram() ); 
  out->datagram().xtc.alloc( sizeof(Xtc) + iFrameSize );
  
  if ( (unsigned char*) out != _pCurDatagram )
  {
    printf( "PrincetonServer::processFrame(): Pool buffer inconsistency. The acquired image data isn't located in the correct buffer\n" );
    return 2;
  }
  
  TypeId typePrincetonFrame(TypeId::Id_PrincetonFrame, FrameV1::Version);
  Xtc* pXtcFrame = 
   new ((char*)pXtcHeader) Xtc(typePrincetonFrame, _src);
  pXtcFrame->alloc( iFrameSize );

  new (pFrameHeader) FrameV1(_iCurShotId, _iCurShotId, _fReadoutTime);
      
  releaseLockCameraData();    
  return 0;
}

int PrincetonServer::writeFrameToFile(const Datagram& dgOut)
{
  // File mode is not supported yet
  return 0;
}

int PrincetonServer::resetFrameData()
{
  lockCameraData("PrincetonServer::initCapture()");    
  
  /*
   * Reset per-frame data
   */
  _iCurShotId       = -1;
  _fReadoutTime     = 0;
    
  /*
   * Update Capture Thread Control and I/O variables
   */
  _iEventCaptureEnd = 0;
  
  delete _pDgOut; _pDgOut = NULL;   
  
  /*
   * Check buffer control buffer status
   */
  if ( _poolEmptyData.numberOfAllocatedObjects() > 0 )
    printf( "PrincetonServer::resetFrameData(): Memory usage issue. Empty Data Pool is not totally free.\n" );
  if ( _poolFrameData.numberOfAllocatedObjects() > 0 )
    printf( "PrincetonServer::resetFrameData(): Memory usage issue. Empty Data Pool is not totally free.\n" );
    
  releaseLockCameraData();  
  return 0;
}

int PrincetonServer::checkInitSettings()
{
  /* Check the input settings */
  if ( !_bStreamMode )
  {
    printf( "PrincetonServer::onEventShotIdEnd(): File mode is not supported.\n" );
    return 1;
  }
  
  if ( !_bStreamMode && !_bUseCaptureThread )  
  {
    printf( "PrincetonServer::onEventShotIdEnd(): Unsupported mode: File mode without capture thread.\n" );
    return 2;
  }
      
  return 0;
}

void PrincetonServer::setupROI(rgn_type& region)
{
  region.s1   = _configCamera.orgX();
  region.s2   = _configCamera.orgX() + _configCamera.width() - 1;
  region.sbin = _configCamera.binX();
  region.p1   = _configCamera.orgY();
  region.p2   = _configCamera.orgY() + _configCamera.height() - 1;
  region.pbin = _configCamera.binY();
}

int PrincetonServer::checkTemperature()
{
  const int16 iCoolingTemp = _configCamera.coolingTemp();
  int16 iTemperatureCurrent = -1;  
  
  lockCameraData("PrincetonServer::checkTemperature():PICAM::getAnyParam()");
  PICAM::getAnyParam(_hCam, PARAM_TEMP, &iTemperatureCurrent );
  releaseLockCameraData();
    
  if ( iTemperatureCurrent >= iCoolingTemp + _iTemperatureTolerance ) 
  {
    printf( "PrincetonServer::runMonitorThread(): Chip temperature (%lf) is higher than the settings (%lf)\n", 
     iTemperatureCurrent/100.0, iCoolingTemp/100.0 );
    return 1;
  }
  else
    return 0;
}

/*
 * Definition of private static consts
 */
const int       PrincetonServer::_iMaxCoolingTime;     
const int       PrincetonServer::_iTemperatureTolerance;
const int       PrincetonServer::_iFrameHeaderSize      = sizeof(CDatagram) + sizeof(Xtc) + sizeof(Princeton::FrameV1);
const int       PrincetonServer::_iMaxFrameDataSize     = 2048*2048*2 + _iFrameHeaderSize;
const int       PrincetonServer::_iPoolDataCount;
const int       PrincetonServer::_iMaxReadoutTime;
const int       PrincetonServer::_iMaxThreadEndTime;
const int       PrincetonServer::_iMaxLastEventTime;

void* PrincetonServer::threadEntryCapture(void * pServer)
{
  ((PrincetonServer*) pServer)->runCaptureThread();
  return NULL;
}

/*
 * Definition of private static data
 */
pthread_mutex_t PrincetonServer::_mutexPlFuncs = PTHREAD_MUTEX_INITIALIZER;    

} //namespace Pds 
