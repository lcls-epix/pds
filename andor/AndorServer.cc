#include <fcntl.h>
#include <math.h>
#include <errno.h>
#include <pthread.h> 

#include "pds/config/AndorDataType.hh"
#include "pds/xtc/Datagram.hh"
#include "pds/xtc/CDatagram.hh"
#include "pds/service/Task.hh"
#include "pds/service/Routine.hh"
#include "pds/config/EvrConfigType.hh"
#include "pds/config/CfgPath.hh"
#include "pds/andor/AndorErrorCodes.hh"
#include "pdsapp/config/Path.hh"
#include "pdsapp/config/Experiment.hh"
#include "pdsapp/config/Table.hh"

#include "AndorServer.hh" 

using std::string;

static inline bool isAndorFuncOk(int iError)
{
  return (iError == DRV_SUCCESS);
}

namespace Pds 
{

AndorServer::AndorServer(int iCamera, bool bDelayMode, bool bInitTest, const Src& src, string sConfigDb, int iSleepInt, int iDebugLevel) :
 _iCamera(iCamera), _bDelayMode(bDelayMode), _bInitTest(bInitTest), _src(src), 
 _sConfigDb(sConfigDb), _iSleepInt(iSleepInt), _iDebugLevel(iDebugLevel),
 _hCam(-1), _bCameraInited(false), _bCaptureInited(false), 
 _iDetectorWidth(-1), _iDetectorHeight(-1), _iImageWidth(-1), _iImageHeight(-1), 
 _iADChannel(-1), _iReadoutPort(-1), _iMaxSpeedTableIndex(-1), _iMaxGainIndex(-1), 
 _iTempMin(-1), _iTempMax(-1), _iFanModeNonAcq(-1),
 _fPrevReadoutTime(0), _bSequenceError(false), _clockPrevDatagram(0,0), _iNumExposure(0),
 _config(), 
 _fReadoutTime(0),  
 _poolFrameData(_iMaxFrameDataSize, _iPoolDataCount), _pDgOut(NULL),
 _CaptureState(CAPTURE_STATE_IDLE), _pTaskCapture(NULL), _routineCapture(*this)
{        
  if ( init() != 0 )
  {
    deinit();
    throw AndorServerException( "AndorServer::AndorServer(): initAndor() failed" );    
  }

  /*
   * Known issue:
   *
   * If initCaptureTask() is put here, occasionally we will miss some FRAME_COMPLETE event when we do the polling
   * Possible reason is that this function (constructor) is called by a different thread other than the polling thread.
   */    
}

AndorServer::~AndorServer()
{ 
  if ( _pTaskCapture != NULL )
    _pTaskCapture->destroy(); // task object will destroy the thread and release the object memory by itself

  if (_bDelayMode)
  {
    /*
     * Wait for capture thread (if exists) to terminate
     */ 
    while ( _CaptureState == CAPTURE_STATE_RUN_TASK )
    {
      printf( "AndorServer::~AndorServer(): Catpure task is running. Wait for it to terminate...\n" );
      
      static int        iTotalWaitTime  = 0;
      static const int  iSleepTime      = 10000; // 10 ms
      
      timeval timeSleepMicro = {0, 10000}; 
      // Use select() to simulate nanosleep(), because experimentally select() controls the sleeping time more precisely
      select( 0, NULL, NULL, NULL, &timeSleepMicro);    
      
      iTotalWaitTime += iSleepTime / 1000;
      if ( iTotalWaitTime >= _iMaxThreadEndTime )
      {
        printf( "AndorServer::~AndorServer(): timeout for waiting thread terminating\n" );
        break;
      }
    }
  }
  
  deinit();  
}

int AndorServer::init()
{
  if ( _bCameraInited )
    return 0;

  timespec timeVal0;
  clock_gettime( CLOCK_REALTIME, &timeVal0 );
 
  int  iError;
  
  at_32 iNumCamera;
  GetAvailableCameras(&iNumCamera);  
  printf("Found %d Andor Cameras\n", (int) iNumCamera);
  
  if (_iCamera < 0 || _iCamera >= iNumCamera)
  {
    printf("AndorServer::init(): Invalid Camera selection: %d (max %d)\n", _iCamera, (int) iNumCamera);
    return ERROR_INVALID_CONFIG;
  }
  
  GetCameraHandle(_iCamera, &_hCam);
  iError = SetCurrentCamera(_hCam);  
  if (!isAndorFuncOk(iError))
  {
    printf("AndorServer::init(): SetCurrentCamera() failed (hcam = %d): %s\n", (int) _hCam, AndorErrorCodes::name(iError));  
    _hCam = -1;
    return ERROR_SDK_FUNC_FAIL;        
  }
  
  iError = Initialize("/usr/local/etc/andor");
  if (!isAndorFuncOk(iError))
  {
    printf("AndorServer::init(): Initialize(): %s\n", AndorErrorCodes::name(iError));  
    return ERROR_SDK_FUNC_FAIL;
  }
  else
  {
    printf("Waiting for hardware to finish initializing...\n");
    sleep(2); //sleep to allow initialization to complete    
  }
  
  timespec timeVal1;  
  clock_gettime( CLOCK_REALTIME, &timeVal1 );
  
  printInfo();
        
  timespec timeVal2;  
  clock_gettime( CLOCK_REALTIME, &timeVal2 );
  
  double fOpenTime    = (timeVal1.tv_nsec - timeVal0.tv_nsec) * 1.e-6 + ( timeVal1.tv_sec - timeVal0.tv_sec ) * 1.e3;    
  double fReportTime  = (timeVal2.tv_nsec - timeVal1.tv_nsec) * 1.e-6 + ( timeVal2.tv_sec - timeVal1.tv_sec ) * 1.e3;    
  printf("Camera Open Time = %6.1lf Report Time = %6.1lf ms\n", fOpenTime, fReportTime);      
  
  //Get Detector dimensions
  iError = GetDetector(&_iDetectorWidth, &_iDetectorHeight);    
  if (!isAndorFuncOk(iError))
  {
    printf("AndorServer::init(): Cannot get detector size. GetDetector(): %s\n", AndorErrorCodes::name(iError));  
    return ERROR_SDK_FUNC_FAIL;
  }
  
  float fPixelWidth = -1, fPixelHeight = -1;
  iError = GetPixelSize(&fPixelWidth, &fPixelHeight);
  if (!isAndorFuncOk(iError))
    printf("AndorServer::init(): GetPixelSize(): %s\n", AndorErrorCodes::name(iError));  
    
  printf("Detector Width %d Height %d  Pixel Width (um) %.2f Height %.2f\n", 
    _iDetectorWidth, _iDetectorHeight, fPixelWidth, fPixelHeight);
    
  int   iVSRecIndex = -1;
  float fVSRecSpeed = -1;  
  iError = GetFastestRecommendedVSSpeed(&iVSRecIndex, &fVSRecSpeed);
  printf("VSSpeed Recommended Index [%d] Speed %f us/pixel\n", iVSRecIndex, fVSRecSpeed);
  if (!isAndorFuncOk(iError))
  {
    printf("AndorServer::init(): GetFastestRecommendedVSSpeed(): %s\n", AndorErrorCodes::name(iError));
    return ERROR_SDK_FUNC_FAIL;
  }
  
  iError = SetVSSpeed(iVSRecIndex);
  if (!isAndorFuncOk(iError))
  {
    printf("AndorServer::init(): SetVSSpeed(): %s\n", AndorErrorCodes::name(iError));
    return ERROR_SDK_FUNC_FAIL;
  }
  printf("Set VSSpeed to %d\n", iVSRecIndex);

  GetNumberPreAmpGains(&_iMaxGainIndex);
  --_iMaxGainIndex;
  printf("Max Gain Index: %d\n", _iMaxGainIndex);      
  
  _iADChannel = 0; // hard coded to use channel 0
  iError = SetADChannel(_iADChannel);
  if (!isAndorFuncOk(iError))
  {
    printf("AndorServer::init(): SetADChannel(): %s\n", AndorErrorCodes::name(iError));  
    return ERROR_SDK_FUNC_FAIL;
  }
  int iDepth = -1;
  GetBitDepth(_iADChannel, &iDepth);    
  printf("Set Channel to %d: depth %d\n", _iADChannel, iDepth);            
  
  _iReadoutPort = 0; // hard coded to use readout port (amplifier) 0
  int iNumAmp = -1;
  GetNumberAmp(& iNumAmp);
  if (_iReadoutPort < 0 || _iReadoutPort >= iNumAmp)
  {
    printf("AndorServer::init(): Readout Port %d out of range (max index %d)\n", _iReadoutPort, iNumAmp);
    return ERROR_SDK_FUNC_FAIL;
  }
  
  iError = SetOutputAmplifier(_iReadoutPort);
  if (!isAndorFuncOk(iError))
  {
    printf("AndorServer::init(): SetOutputAmplifier(): %s\n", AndorErrorCodes::name(iError));    
    return ERROR_SDK_FUNC_FAIL;
  }  
  printf("Set Readout Port (Output Amplifier) to %d\n", _iReadoutPort);

  _iMaxSpeedTableIndex = -1;
  GetNumberHSSpeeds(_iADChannel, _iReadoutPort, &_iMaxSpeedTableIndex);
  --_iMaxSpeedTableIndex;
  printf("Max Speed Table Index: %d\n", _iMaxSpeedTableIndex);

  int iTemperature = 999;
  iError = GetTemperature(&iTemperature);
  printf("Current Temperature %d C  Status %s\n", iTemperature, AndorErrorCodes::name(iError));
  
  printf( "Detector Width %d Height %d Max Speed %d Gain %d Temperature %d C\n", 
    _iDetectorWidth, _iDetectorHeight, _iMaxSpeedTableIndex, _iMaxGainIndex, iTemperature);  
    
  if (_bInitTest)
  {
    if (initTest() != 0)
      return ERROR_FUNCTION_FAILURE;
  }
  
  int iFail = initCameraBeforeConfig();
  if (iFail != 0)
  {
    printf("AndorServer::init(): initCameraBeforeConfig() failed!\n");
    return ERROR_FUNCTION_FAILURE; 
  }
  
  iTemperature = 999;
  iError = GetTemperature(&iTemperature);
  printf("Current Temperature %d C  Status %s\n", iTemperature, AndorErrorCodes::name(iError));
  
  printf( "Detector Width %d Height %d Max Speed %d Gain %d Temperature %d C\n", 
    _iDetectorWidth, _iDetectorHeight, _iMaxSpeedTableIndex, _iMaxGainIndex, iTemperature);  
  
  printf( "Andor Camera [%d] has been initialized\n", _iCamera );
  _bCameraInited = true;    
    
  return 0;
}

static int _printCaps(AndorCapabilities &caps);

int AndorServer::printInfo()
{
  int   iError;
  char  sVersionInfo[128];
  iError = GetVersionInfo(AT_SDKVersion, sVersionInfo, sizeof(sVersionInfo));    
  if (!isAndorFuncOk(iError))
    printf("AndorServer::printInfo(): GetVersionInfo(AT_SDKVersion): %s\n", AndorErrorCodes::name(iError));
  else
    printf("SDKVersion: %s\n", sVersionInfo);

  iError = GetVersionInfo(AT_DeviceDriverVersion, sVersionInfo, sizeof(sVersionInfo));  
  if (!isAndorFuncOk(iError))
    printf("AndorServer::printInfo(): DeviceDriverVersion: %s\n", sVersionInfo);
  else
    printf("GetVersionInfo(AT_DeviceDriverVersion): %s\n", AndorErrorCodes::name(iError));  
    
  unsigned int eprom    = 0;
  unsigned int coffile  = 0;
  unsigned int vxdrev   = 0;
  unsigned int vxdver   = 0;
  unsigned int dllrev   = 0;
  unsigned int dllver   = 0;
  iError = GetSoftwareVersion(&eprom, &coffile, &vxdrev, &vxdver, &dllrev, &dllver);    
  if (!isAndorFuncOk(iError))
    printf("AndorServer::printInfo(): GetSoftwareVersion(): %s\n", AndorErrorCodes::name(iError));  
  else
    printf("Software Version: eprom %d coffile %d vxdrev %d vxdver %d dllrev %d dllver %d\n",
      eprom, coffile, vxdrev, vxdver, dllrev, dllver);
    
  unsigned int iPCB     = 0;
  unsigned int iDecode  = 0;
  unsigned int iDummy1  = 0;
  unsigned int iDummy2  = 0;
  unsigned int iCameraFirmwareVersion = 0;
  unsigned int iCameraFirmwareBuild   = 0;
  iError = GetHardwareVersion(&iPCB, &iDecode, &iDummy1, &iDummy2, &iCameraFirmwareVersion, &iCameraFirmwareBuild);
  if (!isAndorFuncOk(iError))
    printf("AndorServer::printInfo(): GetHardwareVersion(): %s\n", AndorErrorCodes::name(iError));  
  else
    printf("Hardware Version: PCB %d Decode %d FirewareVer %d FirewareBuild %d\n",
      iPCB, iDecode, iCameraFirmwareVersion, iCameraFirmwareBuild);  
    
  int iSerialNumber = -1;
  iError = GetCameraSerialNumber(&iSerialNumber);
  if (!isAndorFuncOk(iError))
    printf("AndorServer::printInfo(): GetCameraSerialNumber(): %s\n", AndorErrorCodes::name(iError));  
  else
    printf("Camera serial number: %d\n", iSerialNumber);
    
  char sHeadModel[256];
  iError = GetHeadModel(sHeadModel);
  if (!isAndorFuncOk(iError))
    printf("AndorServer::printInfo(): GetHeadModel(): %s\n", AndorErrorCodes::name(iError));  
  else
    printf("Camera Head Model: %s\n", sHeadModel);
    
  AndorCapabilities caps;
  iError = GetCapabilities(&caps);
  if (!isAndorFuncOk(iError))
    printf("AndorServer::printInfo(): GetCapabilities(): %s\n", AndorErrorCodes::name(iError));      
  else
    _printCaps(caps);
    
  printf("Available Trigger Modes:\n");
  for (int iTriggerMode = 0; iTriggerMode < 13; ++iTriggerMode)
  {
    static const char* lsTriggerMode[] =
    { "Internal", //0
      "External", //1
      "", "", "", "", 
      "External Start", //6
      "External Exposure (Bulb)", //7
      "", 
      "External FVB EM", //9
      "Software Trigger", //10
      "",
      "External Charge Shifting", //12
    };
    iError = IsTriggerModeAvailable(iTriggerMode);
    if (isAndorFuncOk(iError))
      printf("  [%d] %s\n", iTriggerMode, lsTriggerMode[iTriggerMode]);
  }

  int iNumVSSpeed = -1;
  GetNumberVSSpeeds(&iNumVSSpeed);
  printf("VSSpeed Number: %d\n", iNumVSSpeed);
  
  for (int iVSSpeed = 0; iVSSpeed < iNumVSSpeed; ++iVSSpeed)
  {
    float fSpeed;
    GetVSSpeed(iVSSpeed, &fSpeed);
    printf("  VSSpeed[%d] : %f us/pixel\n", iVSSpeed, fSpeed);
  }  
  
  int iNumVSAmplitude = -1;
  GetNumberVSAmplitudes(&iNumVSAmplitude);
  printf("VSAmplitude Number: %d\n", iNumVSAmplitude);
  
  for (int iVSAmplitude = 0; iVSAmplitude < iNumVSAmplitude; ++iVSAmplitude)
  {
    int iAmplitudeValue = -1;
    GetVSAmplitudeValue(iAmplitudeValue, &iAmplitudeValue);
    
    char sAmplitude[32];
    sAmplitude[sizeof(sAmplitude)-1] = 0;
    GetVSAmplitudeString(iVSAmplitude, sAmplitude);    
    printf("  VSAmplitude[%d]: [%d] %s\n", iVSAmplitude, iAmplitudeValue, sAmplitude);    
  }        
  
  int iNumGain = -1;
  GetNumberPreAmpGains(&iNumGain);
  printf("Preamp Gain Number: %d\n", iNumGain);
  
  for (int iGain = 0; iGain < iNumGain; ++iGain)
  {
    float fGain = -1;
    GetPreAmpGain(iGain, &fGain);
    
    char sGainText[64];
    sGainText[sizeof(sGainText)-1] = 0;
    GetPreAmpGainText(iGain, sGainText, sizeof(sGainText));    
    printf("  Gain %d: %s\n", iGain, sGainText);    
  }   
  
  int iNumChannel = -1;
  GetNumberADChannels(&iNumChannel);
  printf("Channel Number: %d\n", iNumChannel);
    
  int iNumAmp = -1;
  GetNumberAmp(& iNumAmp);
  printf("Amp Number: %d\n", iNumAmp);
  
  for (int iChannel = 0; iChannel < iNumChannel; ++iChannel)
  {
    printf("  Channel[%d]\n", iChannel);
    
    int iDepth = -1;
    GetBitDepth(iChannel, &iDepth);    
    printf("    Depth %d\n", iDepth);
    
    for (int iAmp = 0; iAmp < iNumAmp; ++iAmp)
    {
      printf("    Amp[%d]\n", iAmp);
      int iNumHSSpeed = -1;
      GetNumberHSSpeeds(iChannel, iAmp, &iNumHSSpeed);
     
      for (int iSpeed = 0; iSpeed < iNumHSSpeed; ++iSpeed)
      {        
        float fSpeed = -1;
        GetHSSpeed(iChannel, iAmp, iSpeed, &fSpeed);
        printf("      Speed[%d]: %f MHz\n", iSpeed, fSpeed);
        
        for (int iGain = 0; iGain < iNumGain; ++iGain)
        {
          int iStatus = -1;
          IsPreAmpGainAvailable(iChannel, iAmp, iSpeed, iGain, &iStatus);
          printf("        Gain [%d]: %d\n", iGain, iStatus);
        }
      }
    }    
  }
      
  GetTemperatureRange(&_iTempMin, &_iTempMax);
  printf("Temperature Min %d Max %d\n", _iTempMin, _iTempMax);
  
  int iFrontEndStatus = -1;
  int iTECStatus      = -1;
  GetFrontEndStatus (&iFrontEndStatus);
  GetTECStatus      (&iTECStatus);
  printf("Overheat: FrontEnd %d TEC %d\n", iFrontEndStatus, iTECStatus);  
    
  int iCoolerStatus = -1;
  iError = IsCoolerOn(&iCoolerStatus);
  if (!isAndorFuncOk(iError))
    printf("IsCoolerOn(): %s\n", AndorErrorCodes::name(iError));  
    
  float fSensorTemp   = -1;
  float fTargetTemp   = -1;
  float fAmbientTemp  = -1;
  float fCoolerVolts  = -1;
  GetTemperatureStatus(&fSensorTemp, &fTargetTemp, &fAmbientTemp, &fCoolerVolts);
  printf("Advanced Temperature: Sensor %f Target %f Ambient %f CoolerVolts %f\n", fSensorTemp, fTargetTemp, fAmbientTemp, fCoolerVolts);
      
  return 0;
}

int AndorServer::deinit()
{  
  // If the camera has been init-ed before, and not deinit-ed yet  
  if ( _bCaptureInited )
    deinitCapture(); // deinit the camera explicitly    
  
  if (_hCam != -1)
  {
    int iTemperature  = 999;
    int iError        = GetTemperature(&iTemperature);
    printf("Current Temperature %d C  Status %s\n", iTemperature, AndorErrorCodes::name(iError));
    
    if ( iTemperature < 0 )
      printf("Warning: Temperature is still low (%d C). May results in fast warming.\n", iTemperature);
    else
    {
      iError = CoolerOFF();  
      if (!isAndorFuncOk(iError))
        printf("AndorServer::deinit():: CoolerOFF(): %s\n", AndorErrorCodes::name(iError));      
    }
    
    iError = ShutDown();    
    if (!isAndorFuncOk(iError))
      printf("AndorServer::deinit():: ShutDown(): %s\n", AndorErrorCodes::name(iError));      
  }

  _bCameraInited = false;
  
  printf( "Andor Camera [%d] has been deinitialized\n", _iCamera );  
  return 0;
}

int AndorServer::map()
{
  /*
   * Thread Issue:
   *
   * initCaptureTask() need to be put here. See the comment in the constructor function.
   */      
  if ( initCaptureTask() != 0 )
    return ERROR_SERVER_INIT_FAIL;
  
  return 0;
}

int AndorServer::config(AndorConfigType& config, std::string& sConfigWarning)
{        
  if ( configCamera(config, sConfigWarning) != 0 ) 
    return ERROR_SERVER_INIT_FAIL;
  
  if ( (int) config.width() > _iDetectorWidth || (int) config.height() > _iDetectorHeight)
  {
    char sMessage[128];    
    sprintf( sMessage, "!!! Andor %d ConfigSize (%d,%d) > CcdSize(%d,%d)\n", _iCamera, config.width(), config.height(), _iDetectorWidth, _iDetectorHeight);
    printf(sMessage);
    sConfigWarning += sMessage;
    config.setWidth (_iDetectorWidth);
    config.setHeight(_iDetectorHeight);
  }
        
  //Note: We don't send error for cooling incomplete
  setupCooling( (double) _config.coolingTemp() );
  //if ( setupCooling() != 0 )
    //return ERROR_SERVER_INIT_FAIL;  
  
  _config = config;  
      
  int iTemperature  = 999;
  int iError        = GetTemperature(&iTemperature);
  printf("Current Temperature %d C  Status %s\n", iTemperature, AndorErrorCodes::name(iError));
  
  printf( "Detector Width %d Height %d Speed %d/%d Gain %d/%d Temperature %d C\n", 
    _iDetectorWidth, _iDetectorHeight, _config.readoutSpeedIndex(), _iMaxSpeedTableIndex, _config.gainIndex(), _iMaxGainIndex, iTemperature);  
  
  return 0;
}

int AndorServer::unconfig()
{
  return 0;
}

int AndorServer::beginRun()
{
  /*
   * Check data pool status
   */  
  if ( _poolFrameData.numberOfAllocatedObjects() > 0 )
    printf( "AndorServer::enable(): Memory usage issue. Data Pool is not empty (%d/%d allocated).\n",
      _poolFrameData.numberOfAllocatedObjects(), _poolFrameData.numberofObjects() );
  else if ( _iDebugLevel >= 3 )
    printf( "BeginRun Pool status: %d/%d allocated.\n", _poolFrameData.numberOfAllocatedObjects(), _poolFrameData.numberofObjects() );
    
  /*
   * Reset the per-run data
   */
  _fPrevReadoutTime = 0;
  _bSequenceError   = false;
  _iNumExposure     = 0;
  
  return 0;
}

int AndorServer::endRun()
{  
  /*
   * Check data pool status
   */  
  if ( _poolFrameData.numberOfAllocatedObjects() > 0 )
    printf( "AndorServer::enable(): Memory usage issue. Data Pool is not empty (%d/%d allocated).\n",
      _poolFrameData.numberOfAllocatedObjects(), _poolFrameData.numberofObjects() );
  else if ( _iDebugLevel >= 3 )
    printf( "EndRun Pool status: %d/%d allocated.\n", _poolFrameData.numberOfAllocatedObjects(), _poolFrameData.numberofObjects() );
    
  return 0;
}

int AndorServer::beginCalibCycle()
{
  int iFail = initCapture();    
  if ( iFail != 0 )
    return ERROR_FUNCTION_FAILURE;
  
  return 0;
}

int AndorServer::endCalibCycle()
{
  int iFail  = deinitCapture();  
  if ( iFail != 0 )
    return ERROR_FUNCTION_FAILURE;
  
  return 0;
}

int AndorServer::enable()
{
  /*
   * Check data pool status
   */  
  if ( _poolFrameData.numberOfAllocatedObjects() > 0 )
    printf( "AndorServer::enable(): Memory usage issue. Data Pool is not empty (%d/%d allocated).\n",
      _poolFrameData.numberOfAllocatedObjects(), _poolFrameData.numberofObjects() );
  else if ( _iDebugLevel >= 3 )
    printf( "Enable Pool status: %d/%d allocated.\n", _poolFrameData.numberOfAllocatedObjects(), _poolFrameData.numberofObjects() );        
  
  return 0;
}

int AndorServer::disable()
{
  // Note: Here we don't worry if the data pool is not free, because the 
  //       traffic shaping thead may still holding the L1 Data
  if ( _iDebugLevel >= 3 )
    printf( "Disable Pool status: %d/%d allocated.\n", _poolFrameData.numberOfAllocatedObjects(), _poolFrameData.numberofObjects() );
      
  int iFail  = stopCapture();  
  if ( iFail != 0 )
    return ERROR_FUNCTION_FAILURE;
      
  return 0;
}

int AndorServer::initCapture()
{     
  if ( _bCaptureInited )
    deinitCapture();
    
  LockCameraData lockInitCapture("AndorServer::initCapture()");    
 
  printf("\nInit capture...\n");
  timespec timeVal0;
  clock_gettime( CLOCK_REALTIME, &timeVal0 );
  
  int iError = setupROI();
  if (iError != 0)
    return ERROR_FUNCTION_FAILURE;
  
  //Set initial exposure time
  iError = SetExposureTime(_config.exposureTime());
  if (!isAndorFuncOk(iError))
  {
    printf("AndorServer::initCapture(): SetExposureTime(): %s\n", AndorErrorCodes::name(iError));
    return ERROR_SDK_FUNC_FAIL;
  }
          
  if ( _config.frameSize() - (int) sizeof(AndorDataType) + _iFrameHeaderSize > _iMaxFrameDataSize )
  {
    printf( "AndorServer::initCapture(): Frame size (%i) + Frame header size (%d)"
     "is larger than internal data frame buffer size (%d)\n",
     _config.frameSize() - (int) sizeof(AndorDataType), _iFrameHeaderSize, _iMaxFrameDataSize );
    return ERROR_INVALID_CONFIG;    
  }
  
  float fTimeKeepClean = -1;
  GetKeepCleanTime(&fTimeKeepClean);
  printf("Keep clean time: %f s\n", fTimeKeepClean); 
 
  float fTimeExposure   = -1;
  float fTimeAccumulate = -1;
  float fTimeKinetic    = -1;
  GetAcquisitionTimings(&fTimeExposure, &fTimeAccumulate, &fTimeKinetic);
  printf("Exposure time: %f s  Accumulate time: %f s  Kinetic time: %f s\n", fTimeExposure, fTimeAccumulate, fTimeKinetic);

  float fTimeReadout = -1;
  GetReadOutTime(&fTimeReadout);  
  printf("Readout time: %f s\n", fTimeReadout);  
  
  iError = PrepareAcquisition();
  if (!isAndorFuncOk(iError))
  {
    printf("AndorServer::initCapture(): PrepareAcquisition(): %s\n", AndorErrorCodes::name(iError));
    return ERROR_SDK_FUNC_FAIL;
  }
  
  timespec timeVal1;
  clock_gettime( CLOCK_REALTIME, &timeVal1 );
  double fTimePreAcq    = (timeVal1.tv_nsec - timeVal0.tv_nsec) * 1.e-6 + ( timeVal1.tv_sec - timeVal0.tv_sec ) * 1.e3;    
  
  _bCaptureInited = true;  
  printf( "Capture initialized. Time = %6.1lf ms\n",  fTimePreAcq);

  if ( _iDebugLevel >= 2 )
    printf( "Frame size for image capture = %i. Exposure time = %f s\n",
     _config.frameSize(), _config.exposureTime());  
  
  return 0;
}

int AndorServer::stopCapture()
{
  LockCameraData lockDeinitCapture("AndorServer::stopCapture()");

  resetFrameData(true);

  int iError = AbortAcquisition();
  if (!isAndorFuncOk(iError) && iError != DRV_IDLE)
  {
    printf("AndorServer::deinitCapture(): AbortAcquisition() failed. %s\n", AndorErrorCodes::name(iError));
    return ERROR_SDK_FUNC_FAIL;
  }

  if (_config.fanMode() == (int) AndorConfigType::ENUM_FAN_ACQOFF)  
  {
    iError = SetFanMode((int) AndorConfigType::ENUM_FAN_FULL);
    if (!isAndorFuncOk(iError))
      printf("AndorServer::stopCapture(): SetFanMode(%d): %s\n", (int) AndorConfigType::ENUM_FAN_FULL, AndorErrorCodes::name(iError));
  }
  
  printf( "Capture stopped\n" );
  return 0;
}

int AndorServer::deinitCapture()
{
  if ( !_bCaptureInited )
    return 0;
    
  stopCapture();

  LockCameraData lockDeinitCapture("AndorServer::deinitCapture()");
  
  _bCaptureInited = false;    
  return 0;
}

int AndorServer::startCapture()
{
  if ( _pDgOut == NULL )
  {
    printf( "AndorServer::startCapture(): Datagram has not been allocated. No buffer to store the image data\n" );
    return ERROR_LOGICAL_FAILURE;
  }
    
  if (_config.fanMode() == (int) AndorConfigType::ENUM_FAN_ACQOFF)  
  {
    int iError = SetFanMode((int) AndorConfigType::ENUM_FAN_OFF);
    if (!isAndorFuncOk(iError))
      printf("AndorServer::startCapture(): SetFanMode(%d): %s\n", (int) AndorConfigType::ENUM_FAN_OFF, AndorErrorCodes::name(iError));
  }
  
  int iError = StartAcquisition();
  if (!isAndorFuncOk(iError))
  {
    printf("AndorServer::startCapture(): StartAcquisition() %s\n", AndorErrorCodes::name(iError));
    return ERROR_SDK_FUNC_FAIL;
  }
    
  return 0;
}

int AndorServer::configCamera(AndorConfigType& config, std::string& sConfigWarning)
{ 
  int   iError;    
  
  printf("\nConfiguring...\n");
  if (config.fanMode() == (int) AndorConfigType::ENUM_FAN_ACQOFF)
  {
    printf("Fan Mode: Acq Off\n");
    _iFanModeNonAcq = (int) AndorConfigType::ENUM_FAN_FULL;
  }
  else
    _iFanModeNonAcq = config.fanMode();
  iError = SetFanMode(_iFanModeNonAcq);
  if (!isAndorFuncOk(iError))
  {
    printf("AndorServer::configCamera(): SetFanMode(%d): %s\n", _iFanModeNonAcq, AndorErrorCodes::name(iError));  
    return ERROR_SDK_FUNC_FAIL;
  }
  else
    printf("Set Fan Mode      to %d\n", _iFanModeNonAcq);
    
  iError = SetBaselineClamp(config.baselineClamp());
  if (!isAndorFuncOk(iError))
  {
    printf("AndorServer::configCamera(): SetBaselineClamp(%d): %s\n", config.baselineClamp(), AndorErrorCodes::name(iError));  
    return ERROR_INVALID_CONFIG;
  }
  else
    printf("Set BaselineClamp to %d\n", config.baselineClamp());
  
  iError = SetHighCapacity(config.highCapacity());
  if (!isAndorFuncOk(iError))
  {
    printf("AndorServer::configCamera(): SetHighCapacity(%d): %s\n", config.highCapacity(), AndorErrorCodes::name(iError));  
    return ERROR_INVALID_CONFIG;
  }
  else
    printf("Set HighCapacity  to %d\n", config.highCapacity());    

  int iNumHSSpeed = -1;
  GetNumberHSSpeeds(_iADChannel, _iReadoutPort, &iNumHSSpeed);
  if (config.readoutSpeedIndex() >= iNumHSSpeed)
  {
    printf("AndorServer::configCamera(): Speed Index %d out of range (max index %d)\n", config.readoutSpeedIndex(), iNumHSSpeed);
    return ERROR_INVALID_CONFIG;
  }
    
  iError = SetHSSpeed(_iReadoutPort, config.readoutSpeedIndex());
  if (!isAndorFuncOk(iError))
  {
    printf("AndorServer::configCamera(): SetHSSpeed(%d,%d): %s\n", _iReadoutPort, config.readoutSpeedIndex(), AndorErrorCodes::name(iError));    
    return ERROR_INVALID_CONFIG;
  }
  
  float fSpeed = -1;
  GetHSSpeed(_iADChannel, _iReadoutPort, config.readoutSpeedIndex(), &fSpeed);
  printf("Set Speed Index to %d: %f MHz\n", config.readoutSpeedIndex(), fSpeed);      
  
  int iNumGain = -1;
  GetNumberPreAmpGains(&iNumGain);
  if (config.gainIndex() >= iNumGain)
  {
    printf("AndorServer::configCamera(): Gain Index %d out of range (max index %d)\n", config.gainIndex(), iNumGain);
    return ERROR_INVALID_CONFIG;
  }
  
  int iStatus = -1;
  IsPreAmpGainAvailable(_iADChannel, _iReadoutPort, config.readoutSpeedIndex(), config.gainIndex(), &iStatus);  
  if (iStatus != 1)
  {
    printf("AndorServer::configCamera(): Gain Index %d not supported for channel %d port %d speed %d\n", config.gainIndex(),
      _iADChannel, _iReadoutPort, config.readoutSpeedIndex());
    return ERROR_INVALID_CONFIG;
  }

  iError = SetPreAmpGain(config.gainIndex());
  if (!isAndorFuncOk(iError))
  {
    printf("AndorServer::configCamera(): SetGain(%d): %s\n", config.gainIndex(), AndorErrorCodes::name(iError));    
    return ERROR_INVALID_CONFIG;
  }
  
  float fGain = -1;
  GetPreAmpGain(config.gainIndex(), &fGain);
  
  char sGainText[64];
  sGainText[sizeof(sGainText)-1] = 0;
  GetPreAmpGainText(config.gainIndex(), sGainText, sizeof(sGainText));    
  printf("Set Gain Index to %d: %s\n", config.gainIndex(), sGainText);  
    
  return 0;
}

int AndorServer::initCameraBeforeConfig()
{
  if (_sConfigDb.empty())
    return 0;
    
  size_t iIndexComma = _sConfigDb.find_first_of(',');
  
  string sConfigPath, sConfigType;
  if (iIndexComma == string::npos)
  {
    sConfigPath = _sConfigDb;
    sConfigType = "PRINCETON_BURST";
  }
  else
  {
    sConfigPath = _sConfigDb.substr(0, iIndexComma);
    sConfigType = _sConfigDb.substr(iIndexComma+1, string::npos);
  }
  
  // Setup ConfigDB and Run Key 
  Pds_ConfigDb::Experiment expt((const Pds_ConfigDb::Path&)Pds_ConfigDb::Path(sConfigPath));
  expt.read();
  const Pds_ConfigDb::TableEntry* entry = expt.table().get_top_entry(sConfigType);
  if (entry == NULL)
  {
    printf("AndorServer::initCameraBeforeConfig(): Invalid config db path [%s] type [%s]\n",sConfigPath.c_str(), sConfigType.c_str());
    return ERROR_FUNCTION_FAILURE;    
  }
  
  int runKey = strtoul(entry->key().c_str(),NULL,16);        
  
  const TypeId typeAndorConfig = TypeId(TypeId::Id_AndorConfig, AndorConfigType::Version);    
  
  char strConfigPath[128];
  sprintf(strConfigPath,"%s/keys/%s",sConfigPath.c_str(),CfgPath::path(runKey,_src,typeAndorConfig).c_str());
  printf("Config Path: %s\n", strConfigPath);

  int fdConfig = open(strConfigPath, O_RDONLY);
  AndorConfigType config;
  int iSizeRead = read(fdConfig, &config, sizeof(config));
  if (iSizeRead != sizeof(config))
  {
    printf("AndorServer::initCameraBeforeConfig(): Read config data of incorrect size. Read size = %d (should be %d) bytes\n",
      iSizeRead, (int) sizeof(config));
    return ERROR_FUNCTION_FAILURE;
  }
  
  printf("Setting cooling temperature: %f\n", config.coolingTemp());
  setupCooling( (double) config.coolingTemp() );
  
  return 0;
}

int AndorServer::initTest()
{
  printf( "Running init test...\n" );

  int iError;  
  iError = SetImage(1, 1, 1, 128, 1, 128);
  if (!isAndorFuncOk(iError))
  {
    printf("AndorServer::initTest(): SetImage(): %s\n", AndorErrorCodes::name(iError));    
    return ERROR_SDK_FUNC_FAIL;
  }
          
  timespec timeVal1;
  clock_gettime( CLOCK_REALTIME, &timeVal1 );    
  
  iError = StartAcquisition();
  if (!isAndorFuncOk(iError))
  {
    printf("AndorServer::initTest(): StartAcquisition() %s\n", AndorErrorCodes::name(iError));
    return ERROR_SDK_FUNC_FAIL;
  }

  timespec timeVal2;
  clock_gettime( CLOCK_REALTIME, &timeVal2 );
  
  iError = WaitForAcquisitionTimeOut(_iMaxReadoutTime);
  if (!isAndorFuncOk(iError))
  {
    printf("AndorServer::waitForNewFrameAvailable(): WaitForAcquisitionTimeOut(): %s\n", AndorErrorCodes::name(iError));    
    return ERROR_SDK_FUNC_FAIL;
  }
  
  //while (true)
  //{
  //  //Loop until acquisition finished
  //  int status;    
  //  GetStatus(&status);
  //  if (status == DRV_IDLE)
  //    break;
  //  
  //  if (status != DRV_ACQUIRING) 
  //  {
  //    printf("AndorServer::initTest(): GetStatus(): %s\n", AndorErrorCodes::name(status));
  //    break;
  //  }
  //          
  //  timeval timeSleepMicro = {0, 1000}; // 1 ms
  //  // use select() to simulate nanosleep(), because experimentally select() controls the sleeping time more precisely
  //  select( 0, NULL, NULL, NULL, &timeSleepMicro);       
  //}
  
  timespec timeVal3;
  clock_gettime( CLOCK_REALTIME, &timeVal3 );
 
  double fStartupTime = (timeVal2.tv_nsec - timeVal1.tv_nsec) * 1.e-6 + ( timeVal2.tv_sec - timeVal1.tv_sec ) * 1.e3;    
  double fPollingTime = (timeVal3.tv_nsec - timeVal2.tv_nsec) * 1.e-6 + ( timeVal3.tv_sec - timeVal2.tv_sec ) * 1.e3;    
  double fSingleFrameTime = fStartupTime + fPollingTime;
  printf("  Capture Setup Time = %6.1lfms Total Time = %6.1lfms\n", 
    fStartupTime, fSingleFrameTime );        
    
  return 0;
}

int AndorServer::setupCooling(double fCoolingTemperature)
{
  int iError;
  int iTemperature  = 999;
  iError = GetTemperature(&iTemperature);
  printf("Temperature Before cooling: %d C  Status %s\n", iTemperature, AndorErrorCodes::name(iError));  
   
  if ( fCoolingTemperature < _iTempMin || fCoolingTemperature > _iTempMax )
  {
    printf("Cooling temperature %f out of range (min %d max %d)\n", fCoolingTemperature, _iTempMin, _iTempMax);
    return ERROR_COOLING_FAILURE;
  }
  else
  { 
    iError = SetTemperature((int)fCoolingTemperature);
    if (!isAndorFuncOk(iError))
    {
      printf("AndorServer::setupCooling(): SetTemperature(%d): %s\n", (int)fCoolingTemperature, AndorErrorCodes::name(iError));  
      return ERROR_SDK_FUNC_FAIL;
    }  
    iError = CoolerON();
    if (!isAndorFuncOk(iError))
    {
      printf("AndorServer::setupCooling(): CoolerON(): %s\n", AndorErrorCodes::name(iError));  
      return ERROR_SDK_FUNC_FAIL;
    }  
    printf("Set Temperature to %f C\n", fCoolingTemperature);
  }
    
  const static timeval timeSleepMicroOrg = {0, 5000}; // 5 millisecond    
  timespec timeVal1;
  clock_gettime( CLOCK_REALTIME, &timeVal1 );      
  
  int iNumLoop       = 0;
  int iNumRepateRead = 5;
  int iRead          = 0;
  
  while (1)
  {  
    iTemperature = 999;
    iError = GetTemperature(&iTemperature);
    
    if ( iTemperature <= fCoolingTemperature ) 
    {
      if ( ++iRead >= iNumRepateRead )      
        break;
    }
    else
      iRead = 0;      
    
    if ( (iNumLoop+1) % 200 == 0 )
      printf("Temperature *Updating*: %d C\n", iTemperature );
      
    timespec timeValCur;
    clock_gettime( CLOCK_REALTIME, &timeValCur );
    int iWaitTime = (timeValCur.tv_nsec - timeVal1.tv_nsec) / 1000000 + 
     ( timeValCur.tv_sec - timeVal1.tv_sec ) * 1000; // in milliseconds
    if ( iWaitTime > _iMaxCoolingTime ) break;
    
    // This data will be modified by select(), so need to be reset
    timeval timeSleepMicro = timeSleepMicroOrg; 
    // Use select() to simulate nanosleep(), because experimentally select() controls the sleeping time more precisely
    select( 0, NULL, NULL, NULL, &timeSleepMicro);    
    iNumLoop++;
  }
  
  timespec timeVal2;
  clock_gettime( CLOCK_REALTIME, &timeVal2 );
  double fCoolingTime = (timeVal2.tv_nsec - timeVal1.tv_nsec) * 1.e-6 + ( timeVal2.tv_sec - timeVal1.tv_sec ) * 1.e3;    
  printf("Cooling Time = %6.1lf ms\n", fCoolingTime);  

  
  int iCoolerStatus = -1;
  iError = IsCoolerOn(&iCoolerStatus);
  if (!isAndorFuncOk(iError))
    printf("AndorServer::setupCooling(): IsCoolerOn(): %s\n", AndorErrorCodes::name(iError));  
  
  iTemperature = 999;
  iError = GetTemperature(&iTemperature);
  printf("Temperature After cooling: %d C  Status %s Cooler %d\n", iTemperature, AndorErrorCodes::name(iError), iCoolerStatus);    
  
  if ( iTemperature > fCoolingTemperature ) 
  {
    printf("AndorServer::setupCooling(): Cooling temperature not reached yet; final temperature = %d C", 
     iTemperature );
    return ERROR_COOLING_FAILURE;
  }
  
  return 0;
}

int AndorServer::initCaptureTask()
{
  if ( _pTaskCapture != NULL )
    return 0;
    
  if ( ! _bDelayMode ) // Prompt mode doesn't need to initialize the capture task, because the task will be performed in the event handler    
    return 0;

  _pTaskCapture = new Task(TaskObject("AndorServer"));
  
  return 0;
}

int AndorServer::runCaptureTask()
{
  if ( _CaptureState != CAPTURE_STATE_RUN_TASK )
  {
    printf( "AndorServer::runCaptureTask(): _CaptureState = %d. Capture task is not initialized correctly\n", 
     _CaptureState );
    return ERROR_INCORRECT_USAGE;      
  }

  LockCameraData lockCaptureProcess("AndorServer::runCaptureTask(): Start data polling and processing" );
      
  /*
   * Check if current run is being reset or program is exiting
   */
  if ( !_bCaptureInited )
  {      
    resetFrameData(true);
    return 0;
  }
  
  /*
   * Check if the datagram and frame data have been properly set up
   *
   * Note: This condition should NOT happen normally. Here is a logical check
   * and will output warnings.
   */
  if ( _pDgOut == NULL )
  {
    printf( "AndorServer::runCaptureTask(): Datagram or frame data have not been properly set up before the capture task\n" );
    resetFrameData(true);
    return ERROR_INCORRECT_USAGE;
  }
  
  int iFail = 0;
  do 
  {    
    iFail  = waitForNewFrameAvailable();          
    
    // Even if waitForNewFrameAvailable() failed, we still fill in the frame data with ShotId information
    iFail |= processFrame();       
  }
  while (false);
  
  if ( iFail != 0 )
  {
    // set damage bit, and still keep the image data
    _pDgOut->datagram().xtc.damage.increase(Pds::Damage::UserDefined);      
    
    // // Old ways: delete image data and set the capture state to IDLE
    //resetFrameData(true);
    //return ERROR_FUNCTION_FAILURE;
  }
  
  /*
   * Set the damage bit if 
   *   1. temperature status is not good
   *   2. sequence error happened in the current run
   */
  // Note: Dont send damage when temperature is high
  updateTemperatureData();
  //if ( updateTemperatureData() != 0 )
  //  _pDgOut->datagram().xtc.damage.increase(Pds::Damage::UserDefined);           
  
  _CaptureState = CAPTURE_STATE_DATA_READY;  
  return 0;
}

int AndorServer::startExposure()
{
  ++_iNumExposure; // update event counter
  
  /*
   * Chkec if we are allowed to add a new catpure task
   */
  if ( _CaptureState != CAPTURE_STATE_IDLE  )
  {         
    
    /*
     * Chkec if the data is ready, but has not been transfered out
     *
     * Note: This should NOT happen normally, unless the L1Accept handler didn't get 
     * the data out when the previous data is ready. Here is a logical check
     * and will output warnings.
     */
    if ( _CaptureState == CAPTURE_STATE_DATA_READY )
    {
      printf( "AndorServer::startExposure(): Previous image data has not been sent out\n" );
      resetFrameData(true);
      return ERROR_INCORRECT_USAGE;
    }
   
    /*
     * Remaning case:  _CaptureState == CAPTURE_STATE_RUN_TASK
     * capture thread is still busy polling or processing the previous image data
     *
     * Note: Originally, this is a possible case for software adaptive mode, where the 
     * L1Accept event are raised no matter whether the polling thread is busy or not. 
     *
     * However, since current EVR implementation doesn't support software adaptive mode, 
     * this case is NOT a normal case.
     */    
    
    printf( "AndorServer::startExposure(): Capture task is running. It is impossible to start a new capture.\n" );
    
    /*
     * Here we don't reset the frame data, because the capture task is running and will use the data later
     */
    return ERROR_INCORRECT_USAGE; // No error for adaptive mode
  }
  
  int iFail = 0;
  do 
  {
    iFail = setupFrame();
    if ( iFail != 0 ) break;

    iFail = startCapture();
  }
  while (false);
    
  if ( iFail != 0 )
  {
    resetFrameData(true);
    return ERROR_FUNCTION_FAILURE;
  }    
    
  _CaptureState = CAPTURE_STATE_RUN_TASK;
  
  if (_bDelayMode)
    _pTaskCapture->call( &_routineCapture );
  else
    runCaptureTask();  
  
  return 0;
}

int AndorServer::getData(InDatagram* in, InDatagram*& out)
{
  out = in; // Default: return empty stream

  if ( _CaptureState != CAPTURE_STATE_DATA_READY )
    return 0;
  
  /*
   * Check if the datagram and frame data have been properly set up
   *
   * Note: This condition should NOT happen normally. Here is a logical check
   * and will output warnings.
   */
  if ( _pDgOut == NULL )
  {
    printf( "AndorServer::getData(): Datagram is not properly set up\n" );
    resetFrameData(true);
    return ERROR_LOGICAL_FAILURE;      
  }
    
    
  Datagram& dgIn  = in->datagram();
  Datagram& dgOut = _pDgOut->datagram();
  
  /*
   * Backup the orignal Xtc data
   *
   * Note that it is not correct to use  Xtc xtc1 = xtc2, which means using the Xtc constructor,
   * instead of the copy operator to do the copy. In this case, the xtc data size will be set to 0.
   */
  Xtc xtcOutBkp; 
  xtcOutBkp = dgOut.xtc;

  /*
   * Compose the datagram
   *
   *   1. Use the header from dgIn 
   *   2. Use the xtc (data header) from dgOut
   *   3. Use the data from dgOut (the data was located right after the xtc)
   */
  dgOut     = dgIn;   

  //dgOut.xtc = xtcOutBkp; // not okay for command
  dgOut.xtc.damage = xtcOutBkp.damage;
  dgOut.xtc.extent = xtcOutBkp.extent;

  unsigned char*  pFrameHeader  = (unsigned char*) _pDgOut + sizeof(CDatagram) + sizeof(Xtc);  
  new (pFrameHeader) AndorDataType(in->datagram().seq.stamp().fiducials(), _fReadoutTime);
      
  out       = _pDgOut;    
  
  /* 
   * Reset the frame data, without releasing the output data
   *
   * Note: 
   *   1. _pDgOut will be set to NULL, so that the same data will never be sent out twice
   *   2. _pDgOut will not be released, because the data need to be sent out for use.
   */
  resetFrameData(false);  

  // Delayed data sending for multiple andor cameras, to avoid creating a burst of traffic 
  timeval timeSleepMicro = {0, 1000 * _iSleepInt}; // (_iSleepInt) milliseconds
  select( 0, NULL, NULL, NULL, &timeSleepMicro);

  return 0;
}

int AndorServer::waitData(InDatagram* in, InDatagram*& out)
{
  out = in; // Default: return empty stream
  
  if ( _CaptureState == CAPTURE_STATE_IDLE )
    return 0;
  
  static timespec tsWaitStart;
  clock_gettime( CLOCK_REALTIME, &tsWaitStart );
  
  const static timeval timeSleepMicroOrg = {0, 1000}; // 1 milliseconds
  
  while (_CaptureState != CAPTURE_STATE_DATA_READY)
  {
    // This data will be modified by select(), so need to be reset
    timeval timeSleepMicro = timeSleepMicroOrg; 
    // Use select() to simulate nanosleep(), because experimentally select() controls the sleeping time more precisely
    select( 0, NULL, NULL, NULL, &timeSleepMicro); 
        
          
    timespec tsCurrent;
    clock_gettime( CLOCK_REALTIME, &tsCurrent );
    
    int iWaitTime = (tsCurrent.tv_nsec - tsWaitStart.tv_nsec) / 1000000 + 
     ( tsCurrent.tv_sec - tsWaitStart.tv_sec ) * 1000; // in milliseconds
    if ( iWaitTime >= _iMaxLastEventTime )
    {
      printf( "AndorServer::waitData(): Waiting time is too long. Skip the final data\n" );          
      return ERROR_FUNCTION_FAILURE;
    }
  } // while (1)  
  
  return getData(in, out);      
}

int AndorServer::waitForNewFrameAvailable()
{         
  static timespec tsWaitStart;
  clock_gettime( CLOCK_REALTIME, &tsWaitStart );
  
  int iError;
  iError = WaitForAcquisitionTimeOut(_iMaxReadoutTime);
    
  if (_config.fanMode() == (int) AndorConfigType::ENUM_FAN_ACQOFF)  
  {
    iError = SetFanMode((int) AndorConfigType::ENUM_FAN_FULL);
    if (!isAndorFuncOk(iError))
      printf("AndorServer::waitForNewFrameAvailable(): SetFanMode(%d): %s\n", (int) AndorConfigType::ENUM_FAN_FULL, AndorErrorCodes::name(iError));
  }
  
  if (!isAndorFuncOk(iError))
  {
    printf("AndorServer::waitForNewFrameAvailable(): WaitForAcquisitionTimeOut(): %s\n", AndorErrorCodes::name(iError));    
    
    timespec tsAcqComplete;
    clock_gettime( CLOCK_REALTIME, &tsAcqComplete );
    _fReadoutTime = (tsAcqComplete.tv_nsec - tsWaitStart.tv_nsec) / 1.0e9 + ( tsAcqComplete.tv_sec - tsWaitStart.tv_sec ); // in seconds
    
    return ERROR_SDK_FUNC_FAIL;
  }
  
  //while (true)
  //{
  //  int status;    
  //  GetStatus(&status);
  //  if (status == DRV_IDLE)
  //    break;
  //  
  //  if (status != DRV_ACQUIRING) 
  //  {
  //    printf("AndorServer::waitForNewFrameAvailable(): GetStatus(): %s\n", AndorErrorCodes::name(status));
  //    break;
  //  }
  //  timeval timeSleepMicro = {0, 1000}; // 1 ms
  //  // use select() to simulate nanosleep(), because experimentally select() controls the sleeping time more precisely
  //  select( 0, NULL, NULL, NULL, &timeSleepMicro);       
  //                  
  //  timespec tsCurrent;
  //  clock_gettime( CLOCK_REALTIME, &tsCurrent );
  //  
  //  int iWaitTime = (tsCurrent.tv_nsec - tsWaitStart.tv_nsec) / 1000000 + 
  //   ( tsCurrent.tv_sec - tsWaitStart.tv_sec ) * 1000; // in milliseconds
  //  if ( iWaitTime >= _iMaxReadoutTime )
  //  {
  //    printf( "AndorServer::waitForNewFrameAvailable(): Readout time is longer than %d miliseconds. Capture is stopped\n",
  //     _iMaxReadoutTime );    
  //     
  //    iError = AbortAcquisition();
  //    if (!isAndorFuncOk(iError) && iError != DRV_IDLE)
  //    {
  //      printf("AndorServer::waitForNewFrameAvailable(): AbortAcquisition(): %s\n", AndorErrorCodes::name(iError));
  //    }
  //     
  //    // The  readout time (with incomplete data) will be reported in the framedata
  //    _fReadoutTime = (tsCurrent.tv_nsec - tsWaitStart.tv_nsec) / 1.0e9 + ( tsCurrent.tv_sec - tsWaitStart.tv_sec ); // in seconds
  //    return 1;
  //  }     
  //} // while (true)
  
  uint8_t* pImage = (uint8_t*) _pDgOut + _iFrameHeaderSize;
  iError = GetAcquiredData16((uint16_t*)pImage, _iImageWidth*_iImageHeight);
  if (!isAndorFuncOk(iError))
  {
    printf("GetAcquiredData16(): %s\n", AndorErrorCodes::name(iError));
    return ERROR_SDK_FUNC_FAIL;
  }
  
  timespec tsWaitEnd;
  clock_gettime( CLOCK_REALTIME, &tsWaitEnd );  
  _fReadoutTime = (tsWaitEnd.tv_nsec - tsWaitStart.tv_nsec) / 1.0e9 + ( tsWaitEnd.tv_sec - tsWaitStart.tv_sec ); // in seconds
  
  // Report the readout time for the first few L1 events
  if ( _iNumExposure <= _iMaxEventReport )
    printf( "Readout time report [%d]: %.2f s  Non-exposure time %.2f s\n", _iNumExposure, _fReadoutTime,
      _fReadoutTime - _config.exposureTime());
    
  return 0;
}

int AndorServer::processFrame()
{   
  if ( _pDgOut == NULL )
  {
    printf( "AndorServer::startCapture(): Datagram has not been allocated. No buffer to store the image data\n" );
    return ERROR_LOGICAL_FAILURE;
  }
        
  if ( _iNumExposure <= _iMaxEventReport ||  _iDebugLevel >= 5 )
  {
    unsigned char*  pFrameHeader    = (unsigned char*) _pDgOut + sizeof(CDatagram) + sizeof(Xtc);  
    AndorDataType* pFrame           = (AndorDataType*) pFrameHeader;    
    const uint16_t*     pPixel      = pFrame->data();  
    //int                 iWidth   = (int) ( (_config.width()  + _config.binX() - 1 ) / _config.binX() );
    //int                 iHeight  = (int) ( (_config.height() + _config.binY() - 1 ) / _config.binY() );  
    const uint16_t*     pEnd        = (const uint16_t*) ( (unsigned char*) pFrame->data() + _config.frameSize() );
    const uint64_t      uNumPixels  = (uint64_t) (_config.frameSize() / sizeof(uint16_t) );
    
    uint64_t            uSum    = 0;
    uint64_t            uSumSq  = 0;
    for ( ; pPixel < pEnd; pPixel++ )
    {
      uSum   += *pPixel;
      uSumSq += ((uint32_t)*pPixel) * ((uint32_t)*pPixel);
    }
      
    printf( "Frame Avg Value = %.2lf  Std = %.2lf\n", 
      (double) uSum / (double) uNumPixels, 
      sqrt( (uNumPixels * uSumSq - uSum * uSum) / (double)(uNumPixels*uNumPixels)) );
  }  
        
  return 0;
}

int AndorServer::setupFrame()
{ 
  if ( _poolFrameData.numberOfFreeObjects() <= 0 )
  {
    printf( "AndorServer::setupFrame(): Pool is full, and cannot provide buffer for new datagram\n" );
    return ERROR_LOGICAL_FAILURE;
  }

  const int iFrameSize =_config.frameSize();
  
  /*
   * Set the output datagram pointer
   *
   * Note: This pointer will be used in processFrame(). In the case of delay mode,
   * processFrame() will be exectued in another thread.
   */
  InDatagram*& out = _pDgOut;
  //
  //  Fake a datagram header.  The real header will come with the L1Accept.
  //
  out = 
    new ( &_poolFrameData ) CDatagram( TypeId(TypeId::Any,0), DetInfo(0,DetInfo::NoDetector,0,DetInfo::NoDevice,0) );
  
  out->datagram().xtc.alloc( sizeof(Xtc) + iFrameSize ); 

  if ( _iDebugLevel >= 3 )
  {
    printf( "AndorServer::setupFrame(): pool status: %d/%d allocated, datagram: %p\n", 
     _poolFrameData.numberOfAllocatedObjects(), _poolFrameData.numberofObjects(), _pDgOut  );
  }
  
  /*
   * Set frame object
   */    
  unsigned char* pcXtcFrame = (unsigned char*) _pDgOut + sizeof(CDatagram);
     
  Xtc* pXtcFrame = 
   new ((char*)pcXtcFrame) Xtc(_andorDataType, _src);
  pXtcFrame->alloc( iFrameSize );

  return 0;
}

int AndorServer::resetFrameData(bool bDelOutDatagram)
{
  /*
   * Update Capture Thread Control and I/O variables
   */
  _CaptureState     = CAPTURE_STATE_IDLE;

  /*
   * Reset buffer data
   */
  if (bDelOutDatagram)  delete _pDgOut;
  _pDgOut = NULL;
  
  return 0;
}

int AndorServer::setupROI()
{
  printf("ROI (%d,%d) W %d/%d H %d/%d ", _config.orgX(), _config.orgY(), 
    _config.width(), _config.binX(), _config.height(), _config.binY());

  _iImageWidth  = _config.width()  / _config.binX();
  _iImageHeight = _config.height() / _config.binY();
  printf("image size: W %d H %d\n", _iImageWidth, _iImageHeight);

  int iError;  
  iError = SetImage(_config.binX(), _config.binY(), _config.orgX() + 1, _config.orgX() + _config.width(), 
    _config.orgY() + 1, _config.orgY() + _config.height());
  if (!isAndorFuncOk(iError))
  {
    printf("AndorServer::setupROI(): SetImage(): %s\n", AndorErrorCodes::name(iError));    
    return ERROR_SDK_FUNC_FAIL;
  }
    
  return 0;
}

int AndorServer::updateTemperatureData()
{
  int iError;
  int iTemperature = 999;
  iError = GetTemperature(&iTemperature);
    
  /*
   * Set Info object
   */
  printf( "Detector Temperature report [%d]: %d C\n", _iNumExposure, iTemperature );

  if ( _pDgOut == NULL )
  {
    printf( "AndorServer::updateTemperatureData(): Datagram has not been allocated. No buffer to store the info data\n" );
  }
  else
  {    
    AndorDataType*  pAndorData       = (AndorDataType*) ((unsigned char*) _pDgOut + sizeof(CDatagram) + sizeof(Xtc));  
    pAndorData->setTemperature( (float) iTemperature );    
  }
          
  if (  iTemperature >= _config.coolingTemp() + _fTemperatureHiTol ||  
        iTemperature <= _config.coolingTemp() - _fTemperatureLoTol ) 
  {
    printf( "** AndorServer::updateTemperatureData(): Detector temperature (%d C) is not fixed to the configuration (%.1f C)\n", 
      iTemperature, _config.coolingTemp() );
    return ERROR_TEMPERATURE;
  }
  
  return 0;
}

static int _printCaps(AndorCapabilities &caps)
{
  printf("Capabilities:\n");
  printf("  Size              : %d\n",   (int) caps.ulSize);
  printf("  AcqModes          : 0x%x\n", (int) caps.ulAcqModes);  
  printf("    AC_ACQMODE_SINGLE         : %d\n", (caps.ulAcqModes & AC_ACQMODE_SINGLE)? 1:0 );  
  printf("    AC_ACQMODE_VIDEO          : %d\n", (caps.ulAcqModes & AC_ACQMODE_VIDEO)? 1:0 );  
  printf("    AC_ACQMODE_ACCUMULATE     : %d\n", (caps.ulAcqModes & AC_ACQMODE_ACCUMULATE)? 1:0 );  
  printf("    AC_ACQMODE_KINETIC        : %d\n", (caps.ulAcqModes & AC_ACQMODE_KINETIC)? 1:0 );  
  printf("    AC_ACQMODE_FRAMETRANSFER  : %d\n", (caps.ulAcqModes & AC_ACQMODE_FRAMETRANSFER)? 1:0 );  
  printf("    AC_ACQMODE_FASTKINETICS   : %d\n", (caps.ulAcqModes & AC_ACQMODE_FASTKINETICS)? 1:0 );  
  printf("    AC_ACQMODE_OVERLAP  : %d\n", (caps.ulAcqModes & AC_ACQMODE_OVERLAP)? 1:0 );  
    
  printf("  ReadModes         : 0x%x\n", (int) caps.ulReadModes);  
  printf("    AC_READMODE_FULLIMAGE       : %d\n", (caps.ulReadModes & AC_READMODE_FULLIMAGE)? 1:0 );  
  printf("    AC_READMODE_SUBIMAGE        : %d\n", (caps.ulReadModes & AC_READMODE_SUBIMAGE)? 1:0 );  
  printf("    AC_READMODE_SINGLETRACK     : %d\n", (caps.ulReadModes & AC_READMODE_SINGLETRACK)? 1:0 );  
  printf("    AC_READMODE_FVB             : %d\n", (caps.ulReadModes & AC_READMODE_FVB)? 1:0 );  
  printf("    AC_READMODE_MULTITRACK      : %d\n", (caps.ulReadModes & AC_READMODE_MULTITRACK)? 1:0 );  
  printf("    AC_READMODE_RANDOMTRACK     : %d\n", (caps.ulReadModes & AC_READMODE_RANDOMTRACK)? 1:0 );  
  printf("    AC_READMODE_MULTITRACKSCAN  : %d\n", (caps.ulReadModes & AC_READMODE_MULTITRACKSCAN)? 1:0 );  
  
  printf("  TriggerModes      : 0x%x\n", (int) caps.ulTriggerModes);    
  printf("    AC_TRIGGERMODE_INTERNAL         : %d\n", (caps.ulTriggerModes & AC_TRIGGERMODE_INTERNAL)? 1:0 );  
  printf("    AC_TRIGGERMODE_EXTERNAL         : %d\n", (caps.ulTriggerModes & AC_TRIGGERMODE_EXTERNAL)? 1:0 );  
  printf("    AC_TRIGGERMODE_EXTERNAL_FVB_EM  : %d\n", (caps.ulTriggerModes & AC_TRIGGERMODE_EXTERNAL_FVB_EM)? 1:0 );  
  printf("    AC_TRIGGERMODE_CONTINUOUS       : %d\n", (caps.ulTriggerModes & AC_TRIGGERMODE_CONTINUOUS)? 1:0 );  
  printf("    AC_TRIGGERMODE_EXTERNALSTART    : %d\n", (caps.ulTriggerModes & AC_TRIGGERMODE_EXTERNALSTART)? 1:0 );  
  printf("    AC_TRIGGERMODE_EXTERNALEXPOSURE : %d\n", (caps.ulTriggerModes & AC_TRIGGERMODE_EXTERNALEXPOSURE)? 1:0 );  
  
  printf("  CameraType        : 0x%x\n", (int) caps.ulCameraType);
  printf("    AC_CAMERATYPE_IKON  : %d\n", (caps.ulCameraType == AC_CAMERATYPE_IKON)? 1:0 );  
  
  printf("  PixelMode         : 0x%x\n", (int) caps.ulPixelMode);
  printf("    AC_PIXELMODE_16BIT  : %d\n", (caps.ulPixelMode & AC_PIXELMODE_16BIT)? 1:0 );  
  printf("  SetFunctions      : 0x%x\n", (int) caps.ulSetFunctions);  
  printf("    AC_SETFUNCTION_VREADOUT           : %d\n", (caps.ulSetFunctions & AC_SETFUNCTION_VREADOUT)? 1:0 );  
  printf("    AC_SETFUNCTION_HREADOUT           : %d\n", (caps.ulSetFunctions & AC_SETFUNCTION_HREADOUT)? 1:0 );  
  printf("    AC_SETFUNCTION_TEMPERATURE        : %d\n", (caps.ulSetFunctions & AC_SETFUNCTION_TEMPERATURE)? 1:0 );  
  printf("    AC_SETFUNCTION_MCPGAIN            : %d\n", (caps.ulSetFunctions & AC_SETFUNCTION_MCPGAIN)? 1:0 );  
  printf("    AC_SETFUNCTION_EMCCDGAIN          : %d\n", (caps.ulSetFunctions & AC_SETFUNCTION_EMCCDGAIN)? 1:0 );  
  printf("    AC_SETFUNCTION_BASELINECLAMP      : %d\n", (caps.ulSetFunctions & AC_SETFUNCTION_BASELINECLAMP)? 1:0 );  
  printf("    AC_SETFUNCTION_VSAMPLITUDE        : %d\n", (caps.ulSetFunctions & AC_SETFUNCTION_VSAMPLITUDE)? 1:0 );  
  printf("    AC_SETFUNCTION_HIGHCAPACITY       : %d\n", (caps.ulSetFunctions & AC_SETFUNCTION_HIGHCAPACITY)? 1:0 );  
  printf("    AC_SETFUNCTION_BASELINEOFFSET     : %d\n", (caps.ulSetFunctions & AC_SETFUNCTION_BASELINEOFFSET)? 1:0 );  
  printf("    AC_SETFUNCTION_PREAMPGAIN         : %d\n", (caps.ulSetFunctions & AC_SETFUNCTION_PREAMPGAIN)? 1:0 );  
  printf("    AC_SETFUNCTION_CROPMODE           : %d\n", (caps.ulSetFunctions & AC_SETFUNCTION_CROPMODE)? 1:0 );  
  printf("    AC_SETFUNCTION_DMAPARAMETERS      : %d\n", (caps.ulSetFunctions & AC_SETFUNCTION_DMAPARAMETERS)? 1:0 );  
  printf("    AC_SETFUNCTION_HORIZONTALBIN      : %d\n", (caps.ulSetFunctions & AC_SETFUNCTION_HORIZONTALBIN)? 1:0 );  
  printf("    AC_SETFUNCTION_MULTITRACKHRANGE   : %d\n", (caps.ulSetFunctions & AC_SETFUNCTION_MULTITRACKHRANGE)? 1:0 );  
  printf("    AC_SETFUNCTION_RANDOMTRACKNOGAPS  : %d\n", (caps.ulSetFunctions & AC_SETFUNCTION_RANDOMTRACKNOGAPS)? 1:0 );  
  printf("    AC_SETFUNCTION_EMADVANCED         : %d\n", (caps.ulSetFunctions & AC_SETFUNCTION_EMADVANCED)? 1:0 );  
  printf("    AC_SETFUNCTION_GATEMODE           : %d\n", (caps.ulSetFunctions & AC_SETFUNCTION_GATEMODE)? 1:0 );  
  printf("    AC_SETFUNCTION_DDGTIMES           : %d\n", (caps.ulSetFunctions & AC_SETFUNCTION_DDGTIMES)? 1:0 );  
  printf("    AC_SETFUNCTION_IOC                : %d\n", (caps.ulSetFunctions & AC_SETFUNCTION_IOC)? 1:0 );  
  printf("    AC_SETFUNCTION_INTELLIGATE        : %d\n", (caps.ulSetFunctions & AC_SETFUNCTION_INTELLIGATE)? 1:0 );  
  printf("    AC_SETFUNCTION_INSERTION_DELAY    : %d\n", (caps.ulSetFunctions & AC_SETFUNCTION_INSERTION_DELAY)? 1:0 );  
  printf("    AC_SETFUNCTION_GATESTEP           : %d\n", (caps.ulSetFunctions & AC_SETFUNCTION_GATESTEP)? 1:0 );  
  printf("    AC_SETFUNCTION_TRIGGERTERMINATION : %d\n", (caps.ulSetFunctions & AC_SETFUNCTION_TRIGGERTERMINATION)? 1:0 );  
  printf("    AC_SETFUNCTION_EXTENDEDNIR        : %d\n", (caps.ulSetFunctions & AC_SETFUNCTION_EXTENDEDNIR)? 1:0 );  
  printf("    AC_SETFUNCTION_SPOOLTHREADCOUNT   : %d\n", (caps.ulSetFunctions & AC_SETFUNCTION_SPOOLTHREADCOUNT)? 1:0 );  

  printf("  GetFunctions      : 0x%x\n", (int) caps.ulGetFunctions);  
  printf("    AC_GETFUNCTION_TEMPERATURE        : %d\n", (caps.ulGetFunctions & AC_GETFUNCTION_TEMPERATURE)? 1:0 );  
  printf("    AC_GETFUNCTION_TARGETTEMPERATURE  : %d\n", (caps.ulGetFunctions & AC_GETFUNCTION_TARGETTEMPERATURE)? 1:0 );  
  printf("    AC_GETFUNCTION_TEMPERATURERANGE   : %d\n", (caps.ulGetFunctions & AC_GETFUNCTION_TEMPERATURERANGE)? 1:0 );  
  printf("    AC_GETFUNCTION_DETECTORSIZE       : %d\n", (caps.ulGetFunctions & AC_GETFUNCTION_DETECTORSIZE)? 1:0 );  
  printf("    AC_GETFUNCTION_MCPGAIN            : %d\n", (caps.ulGetFunctions & AC_GETFUNCTION_MCPGAIN)? 1:0 );  
  printf("    AC_GETFUNCTION_EMCCDGAIN          : %d\n", (caps.ulGetFunctions & AC_GETFUNCTION_EMCCDGAIN)? 1:0 );  
  printf("    AC_GETFUNCTION_HVFLAG             : %d\n", (caps.ulGetFunctions & AC_GETFUNCTION_HVFLAG)? 1:0 );  
  printf("    AC_GETFUNCTION_GATEMODE           : %d\n", (caps.ulGetFunctions & AC_GETFUNCTION_GATEMODE)? 1:0 );  
  printf("    AC_GETFUNCTION_DDGTIMES           : %d\n", (caps.ulGetFunctions & AC_GETFUNCTION_DDGTIMES)? 1:0 );  
  printf("    AC_GETFUNCTION_IOC                : %d\n", (caps.ulGetFunctions & AC_GETFUNCTION_IOC)? 1:0 );  
  printf("    AC_GETFUNCTION_INTELLIGATE        : %d\n", (caps.ulGetFunctions & AC_GETFUNCTION_INTELLIGATE)? 1:0 );  
  printf("    AC_GETFUNCTION_INSERTION_DELAY    : %d\n", (caps.ulGetFunctions & AC_GETFUNCTION_INSERTION_DELAY)? 1:0 );  
  printf("    AC_GETFUNCTION_GATESTEP           : %d\n", (caps.ulGetFunctions & AC_GETFUNCTION_GATESTEP)? 1:0 );  
  printf("    AC_GETFUNCTION_PHOSPHORSTATUS     : %d\n", (caps.ulGetFunctions & AC_GETFUNCTION_PHOSPHORSTATUS)? 1:0 );  
  printf("    AC_GETFUNCTION_MCPGAINTABLE       : %d\n", (caps.ulGetFunctions & AC_GETFUNCTION_MCPGAINTABLE)? 1:0 );  
  
  printf("  Features          : 0x%x\n", (int) caps.ulFeatures);  
  printf("    AC_FEATURES_POLLING                         : %d\n", (caps.ulFeatures & AC_FEATURES_POLLING)? 1:0 );  
  printf("    AC_FEATURES_EVENTS                          : %d\n", (caps.ulFeatures & AC_FEATURES_EVENTS)? 1:0 );  
  printf("    AC_FEATURES_SPOOLING                        : %d\n", (caps.ulFeatures & AC_FEATURES_SPOOLING)? 1:0 );  
  printf("    AC_FEATURES_SHUTTER                         : %d\n", (caps.ulFeatures & AC_FEATURES_SHUTTER)? 1:0 );  
  printf("    AC_FEATURES_SHUTTEREX                       : %d\n", (caps.ulFeatures & AC_FEATURES_SHUTTEREX)? 1:0 );  
  printf("    AC_FEATURES_EXTERNAL_I2C                    : %d\n", (caps.ulFeatures & AC_FEATURES_EXTERNAL_I2C)? 1:0 );  
  printf("    AC_FEATURES_SATURATIONEVENT                 : %d\n", (caps.ulFeatures & AC_FEATURES_SATURATIONEVENT)? 1:0 );  
  printf("    AC_FEATURES_FANCONTROL                      : %d\n", (caps.ulFeatures & AC_FEATURES_FANCONTROL)? 1:0 );  
  printf("    AC_FEATURES_MIDFANCONTROL                   : %d\n", (caps.ulFeatures & AC_FEATURES_MIDFANCONTROL)? 1:0 );  
  printf("    AC_FEATURES_TEMPERATUREDURINGACQUISITION    : %d\n", (caps.ulFeatures & AC_FEATURES_TEMPERATUREDURINGACQUISITION)? 1:0 );  
  printf("    AC_FEATURES_KEEPCLEANCONTROL                : %d\n", (caps.ulFeatures & AC_FEATURES_KEEPCLEANCONTROL)? 1:0 );  
  printf("    AC_FEATURES_DDGLITE                         : %d\n", (caps.ulFeatures & AC_FEATURES_DDGLITE)? 1:0 );  
  printf("    AC_FEATURES_FTEXTERNALEXPOSURE              : %d\n", (caps.ulFeatures & AC_FEATURES_FTEXTERNALEXPOSURE)? 1:0 );  
  printf("    AC_FEATURES_KINETICEXTERNALEXPOSURE         : %d\n", (caps.ulFeatures & AC_FEATURES_KINETICEXTERNALEXPOSURE)? 1:0 );  
  printf("    AC_FEATURES_DACCONTROL                      : %d\n", (caps.ulFeatures & AC_FEATURES_DACCONTROL)? 1:0 );  
  printf("    AC_FEATURES_METADATA                        : %d\n", (caps.ulFeatures & AC_FEATURES_METADATA)? 1:0 );  
  printf("    AC_FEATURES_IOCONTROL                       : %d\n", (caps.ulFeatures & AC_FEATURES_IOCONTROL)? 1:0 );  
  printf("    AC_FEATURES_PHOTONCOUNTING                  : %d\n", (caps.ulFeatures & AC_FEATURES_PHOTONCOUNTING)? 1:0 );  
  printf("    AC_FEATURES_COUNTCONVERT                    : %d\n", (caps.ulFeatures & AC_FEATURES_COUNTCONVERT)? 1:0 );  
  printf("    AC_FEATURES_DUALMODE                        : %d\n", (caps.ulFeatures & AC_FEATURES_DUALMODE)? 1:0 );  
  printf("    AC_FEATURES_OPTACQUIRE                      : %d\n", (caps.ulFeatures & AC_FEATURES_OPTACQUIRE)? 1:0 );  
  printf("    AC_FEATURES_REALTIMESPURIOUSNOISEFILTER     : %d\n", (caps.ulFeatures & AC_FEATURES_REALTIMESPURIOUSNOISEFILTER)? 1:0 );  
  printf("    AC_FEATURES_POSTPROCESSSPURIOUSNOISEFILTER  : %d\n", (caps.ulFeatures & AC_FEATURES_POSTPROCESSSPURIOUSNOISEFILTER)? 1:0 );  
  printf("    AC_FEATURES_DUALPREAMPGAIN                  : %d\n", (caps.ulFeatures & AC_FEATURES_DUALPREAMPGAIN)? 1:0 );  
  printf("    AC_FEATURES_DEFECT_CORRECTION               : %d\n", (caps.ulFeatures & AC_FEATURES_DEFECT_CORRECTION)? 1:0 );  
  printf("    AC_FEATURES_STARTOFEXPOSURE_EVENT           : %d\n", (caps.ulFeatures & AC_FEATURES_STARTOFEXPOSURE_EVENT)? 1:0 );  
  printf("    AC_FEATURES_ENDOFEXPOSURE_EVENT             : %d\n", (caps.ulFeatures & AC_FEATURES_ENDOFEXPOSURE_EVENT)? 1:0 );  
  printf("    AC_FEATURES_CAMERALINK                      : %d\n", (caps.ulFeatures & AC_FEATURES_CAMERALINK)? 1:0 );  

  printf("  PCICard           : 0x%x\n", (int) caps.ulPCICard);  
  printf("  EMGainCapability  : 0x%x\n", (int) caps.ulEMGainCapability);  
  printf("  FTReadModes       : 0x%x\n", (int) caps.ulFTReadModes);  
  printf("    AC_READMODE_FULLIMAGE       : %d\n", (caps.ulFTReadModes & AC_READMODE_FULLIMAGE)? 1:0 );  
  printf("    AC_READMODE_SUBIMAGE        : %d\n", (caps.ulFTReadModes & AC_READMODE_SUBIMAGE)? 1:0 );  
  printf("    AC_READMODE_SINGLETRACK     : %d\n", (caps.ulFTReadModes & AC_READMODE_SINGLETRACK)? 1:0 );  
  printf("    AC_READMODE_FVB             : %d\n", (caps.ulFTReadModes & AC_READMODE_FVB)? 1:0 );  
  printf("    AC_READMODE_MULTITRACK      : %d\n", (caps.ulFTReadModes & AC_READMODE_MULTITRACK)? 1:0 );  
  printf("    AC_READMODE_RANDOMTRACK     : %d\n", (caps.ulFTReadModes & AC_READMODE_RANDOMTRACK)? 1:0 );  
  printf("    AC_READMODE_MULTITRACKSCAN  : %d\n", (caps.ulFTReadModes & AC_READMODE_MULTITRACKSCAN)? 1:0 );    
  return 0;
}

/*
 * Definition of private static consts
 */
const int       AndorServer::_iMaxCoolingTime;  
const int       AndorServer::_fTemperatureHiTol;
const int       AndorServer::_fTemperatureLoTol;
const int       AndorServer::_iFrameHeaderSize      = sizeof(CDatagram) + sizeof(Xtc) + sizeof(AndorDataType);
const int       AndorServer::_iMaxFrameDataSize     = _iFrameHeaderSize + 2048*2048*2;
const int       AndorServer::_iPoolDataCount;
const int       AndorServer::_iMaxReadoutTime;
const int       AndorServer::_iMaxThreadEndTime;
const int       AndorServer::_iMaxLastEventTime;
const int       AndorServer::_iMaxEventReport;
const float     AndorServer::_fEventDeltaTimeFactor = 1.01f;  

/*
 * Definition of private static data
 */
pthread_mutex_t AndorServer::_mutexPlFuncs = PTHREAD_MUTEX_INITIALIZER;    


AndorServer::CaptureRoutine::CaptureRoutine(AndorServer& server) : _server(server)
{
}

void AndorServer::CaptureRoutine::routine(void)
{
  _server.runCaptureTask();
}

} //namespace Pds 
