#include <fcntl.h>
#include <math.h>
#include <errno.h>
#include <pthread.h>
#include <stdlib.h>

#include "pds/config/Andor3dDataType.hh"
#include "pds/xtc/Datagram.hh"
#include "pds/xtc/CDatagram.hh"
#include "pds/service/Task.hh"
#include "pds/service/Routine.hh"
#include "pds/config/EvrConfigType.hh"
#include "pds/config/CfgClientNfs.hh"
#include "pds/andor/AndorErrorCodes.hh"
#include "pdsapp/config/Experiment.hh"
#include "pdsapp/config/Table.hh"

#include "DualAndorServer.hh"

using std::string;

static inline bool isAndorFuncOk(int iError)
{
  return (iError == DRV_SUCCESS);
}

namespace Pds
{

DualAndorServer::DualAndorServer(int iCamera, bool bDelayMode, bool bInitTest, const Src& src, string sConfigDb, int iSleepInt, int iDebugLevel) :
 _iCamera(iCamera), _bDelayMode(bDelayMode), _bInitTest(bInitTest), _src(src),
 _sConfigDb(sConfigDb), _iSleepInt(iSleepInt), _iDebugLevel(iDebugLevel),
 _hCamMaster(-1), _hCamSlave(-1), _bCameraInited(false), _bCaptureInited(false),
 _iTriggerMode(0), _iDetectorWidth(-1), _iDetectorHeight(-1), _iDetectorSensor(_iMaxCamera),
 _iImageWidth(-1), _iImageHeight(-1), _iADChannel(-1), _iReadoutPort(-1), _iMaxSpeedTableIndex(-1),
 _iMaxGainIndex(-1), _iTempMin(-1), _iTempMax(-1), _iFanModeNonAcq(-1),
 _fPrevReadoutTime(0), _bSequenceError(false), _clockPrevDatagram(0,0), _iNumExposure(0), _iNumAcq(0),
 _iReadoutWaitTime(_iMaxReadoutTime),
 _config(),
 _fReadoutTime(0),
 _iTemperatureMaster(999), _iTemperatureSlave(999), _bCallShutdown(false),
 _poolFrameData(_iMaxFrameDataSize, _iPoolDataCount), _pDgOut(NULL),
 _CaptureState(CAPTURE_STATE_IDLE), _pTaskCapture(NULL), _routineCapture(*this)
{
  if ( initDevice() != 0 )
    throw DualAndorServerException( "DualAndorServer::DualAndorServer(): initAndor() failed" );

  /*
   * Known issue:
   *
   * If initCaptureTask() is put here, occasionally we will miss some FRAME_COMPLETE event when we do the polling
   * Possible reason is that this function (constructor) is called by a different thread other than the polling thread.
   */
}

DualAndorServer::~DualAndorServer()
{
  /*
   * Note: This function may be called by the signal handler, and need to be finished quickly
   */
  int iError;
  if (_hCamSlave != -1 && checkSlaveSelected())
  {
    iError = AbortAcquisition();
    if (!isAndorFuncOk(iError) && iError != DRV_IDLE)
      printf("DualAndorServer::~DualAndorServer(): AbortAcquisition() failed (hcam = %d). %s\n", (int) _hCamSlave, AndorErrorCodes::name(iError));
  }
  if (_hCamMaster != -1 && checkMasterSelected())
  {
    iError = AbortAcquisition();
    if (!isAndorFuncOk(iError) && iError != DRV_IDLE)
      printf("DualAndorServer::~DualAndorServer(): AbortAcquisition() failed (hcam = %d). %s\n", (int) _hCamMaster, AndorErrorCodes::name(iError));
  }

  deinit();

  if ( _pTaskCapture != NULL )
    _pTaskCapture->destroy(); // task object will destroy the thread and release the object memory by itself
}

int DualAndorServer::initDevice()
{
  if ( _bCameraInited )
    return 0;

  timespec timeVal0;
  clock_gettime( CLOCK_REALTIME, &timeVal0 );

  int  iError;

  at_32 iNumCamera;
  GetAvailableCameras(&iNumCamera);
  printf("Found %d Andor Cameras\n", (int) iNumCamera);

  if (iNumCamera != _iDetectorSensor)
  {
    printf("DualAndorServer::init(): Invalid Camera selection: found %d, expected %d\n", (int) iNumCamera, _iDetectorSensor);
    return ERROR_INVALID_CONFIG;
  }

  for (int iCamHandle = 0; iCamHandle < iNumCamera; ++iCamHandle)
  {
    at_32 hCam;
    GetCameraHandle(iCamHandle, &hCam);
    iError = SetCurrentCamera(hCam);
    if (!isAndorFuncOk(iError))
    {
      printf("DualAndorServer::init(): SetCurrentCamera() failed (hcam = %d): %s\n", (int) hCam, AndorErrorCodes::name(iError));
      hCam = -1;
      return ERROR_SDK_FUNC_FAIL;
    }

    iError = Initialize(const_cast<char*>("/usr/local/etc/andor"));
    if (!isAndorFuncOk(iError))
    {
      printf("DualAndorServer::init(): Initialize(): %s\n", AndorErrorCodes::name(iError));
      return ERROR_SDK_FUNC_FAIL;
    }
  }

  printf("Waiting for hardware to finish initializing...\n");
  sleep(2); //sleep to allow initialization to complete

  timespec timeVal1;
  clock_gettime( CLOCK_REALTIME, &timeVal1 );

  for (int iCamHandle = 0; iCamHandle < iNumCamera; ++iCamHandle)
  {
    at_32 hCam;
    GetCameraHandle(iCamHandle, &hCam);
    iError = SetCurrentCamera(hCam);
    if (!isAndorFuncOk(iError))
    {
      printf("DualAndorServer::init(): SetCurrentCamera() failed (hcam = %d): %s\n", (int) hCam, AndorErrorCodes::name(iError));
      hCam = -1;
      return ERROR_SDK_FUNC_FAIL;
    }

    printf("MultiAndorServer::init(): Printing info (hcam = %d)...\n", (int) hCam);
    printInfo();

    int iSerialNumber = -1;
    iError = GetCameraSerialNumber(&iSerialNumber);
    if (!isAndorFuncOk(iError))
    {
      printf("MultiAndorServer::init(): GetCameraSerialNumber() failed (hcam = %d): %s\n", (int) hCam, AndorErrorCodes::name(iError));
      return ERROR_SDK_FUNC_FAIL;
    }

    if (iSerialNumber != 0) {
      if (_hCamMaster == -1) {
        _hCamMaster = hCam;
      } else {
        printf("MultiAndorServer::init(): Invalid Camera selection: More than one master sensor found\n");
        return ERROR_INVALID_CONFIG;
      }
    } else {
      if (_hCamSlave == -1) {
        _hCamSlave = hCam;
      } else {
        printf("MultiAndorServer::init(): Invalid Camera selection: More than one slave sensor found\n");
        return ERROR_INVALID_CONFIG;
      }
    }
  }

  timespec timeVal2;
  clock_gettime( CLOCK_REALTIME, &timeVal2 );

  double fOpenTime    = (timeVal1.tv_nsec - timeVal0.tv_nsec) * 1.e-6 + ( timeVal1.tv_sec - timeVal0.tv_sec ) * 1.e3;
  double fReportTime  = (timeVal2.tv_nsec - timeVal1.tv_nsec) * 1.e-6 + ( timeVal2.tv_sec - timeVal1.tv_sec ) * 1.e3;
  printf("Camera Open Time = %6.1lf Report Time = %6.1lf ms\n", fOpenTime, fReportTime);

  return 0;
}

int DualAndorServer::initSetup()
{
  if ( _bCameraInited )
    return 0;

  // Read most cabilities from the master
  if (!checkMasterSelected())
  {
    printf("DualAndorServer::init(): Cannot get detector handle: (hcam = %d)\n", (int) _hCamMaster);
    return ERROR_SDK_FUNC_FAIL;
  }

  //Get Detector dimensions
  int iError = GetDetector(&_iDetectorWidth, &_iDetectorHeight);
  if (!isAndorFuncOk(iError))
  {
    printf("DualAndorServer::init(): Cannot get detector size. GetDetector(): %s\n", AndorErrorCodes::name(iError));
    return ERROR_SDK_FUNC_FAIL;
  }

  float fPixelWidth = -1, fPixelHeight = -1;
  iError = GetPixelSize(&fPixelWidth, &fPixelHeight);
  if (!isAndorFuncOk(iError))
    printf("DualAndorServer::init(): GetPixelSize(): %s\n", AndorErrorCodes::name(iError));

  printf("Detector Width %d Height %d  Pixel Width (um) %.2f Height %.2f\n",
    _iDetectorWidth, _iDetectorHeight, fPixelWidth, fPixelHeight);

  int   iVSRecIndex = -1;
  float fVSRecSpeed = -1;
  iError = GetFastestRecommendedVSSpeed(&iVSRecIndex, &fVSRecSpeed);
  printf("VSSpeed Recommended Index [%d] Speed %f us/pixel\n", iVSRecIndex, fVSRecSpeed);
  if (!isAndorFuncOk(iError))
  {
    printf("DualAndorServer::init(): GetFastestRecommendedVSSpeed(): %s\n", AndorErrorCodes::name(iError));
    return ERROR_SDK_FUNC_FAIL;
  }

  if (checkSlaveSelected()) {
    iError = SetVSSpeed(iVSRecIndex);
    if (!isAndorFuncOk(iError))
    {
      printf("DualAndorServer::init(): SetVSSpeed() (hcam = %d): %s\n", (int) _hCamSlave, AndorErrorCodes::name(iError));
      return ERROR_SDK_FUNC_FAIL;
    }
    printf("Set VSSpeed to %d (hcam = %d)\n", iVSRecIndex, (int) _hCamSlave);
  } else {
    return ERROR_SDK_FUNC_FAIL;
  }
  if (checkMasterSelected()) {
    iError = SetVSSpeed(iVSRecIndex);
    if (!isAndorFuncOk(iError))
    {
      printf("DualAndorServer::init(): SetVSSpeed() (hcam = %d): %s\n", (int) _hCamMaster, AndorErrorCodes::name(iError));
      return ERROR_SDK_FUNC_FAIL;
    }
    printf("Set VSSpeed to %d (hcam = %d)\n", iVSRecIndex, (int) _hCamMaster);
  } else {
    return ERROR_SDK_FUNC_FAIL;
  }

  GetNumberPreAmpGains(&_iMaxGainIndex);
  --_iMaxGainIndex;
  printf("Max Gain Index: %d\n", _iMaxGainIndex);

  _iADChannel = 0; // hard coded to use channel 0
  if (checkSlaveSelected()) {
    iError = SetADChannel(_iADChannel);
    if (!isAndorFuncOk(iError))
    {
      printf("DualAndorServer::init(): SetADChannel() (hcam = %d): %s\n", (int) _hCamSlave, AndorErrorCodes::name(iError));
      return ERROR_SDK_FUNC_FAIL;
    }
  } else {
    return ERROR_SDK_FUNC_FAIL;
  }
  if (checkMasterSelected()) {
    iError = SetADChannel(_iADChannel);
    if (!isAndorFuncOk(iError))
    {
      printf("DualAndorServer::init(): SetADChannel() (hcam = %d): %s\n", (int) _hCamMaster, AndorErrorCodes::name(iError));
      return ERROR_SDK_FUNC_FAIL;
    }
  } else {
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
    printf("DualAndorServer::init(): Readout Port %d out of range (max index %d)\n", _iReadoutPort, iNumAmp);
    return ERROR_SDK_FUNC_FAIL;
  }

  if (checkSlaveSelected()) {
    iError = SetOutputAmplifier(_iReadoutPort);
    if (!isAndorFuncOk(iError))
    {
      printf("DualAndorServer::init(): SetOutputAmplifier() (hcam = %d): %s\n", (int) _hCamSlave, AndorErrorCodes::name(iError));
      return ERROR_SDK_FUNC_FAIL;
    }
  } else {
    return ERROR_SDK_FUNC_FAIL;
  }
  if (checkMasterSelected()) {
    iError = SetOutputAmplifier(_iReadoutPort);
    if (!isAndorFuncOk(iError))
    {
      printf("DualAndorServer::init(): SetOutputAmplifier() (hcam = %d): %s\n", (int) _hCamMaster, AndorErrorCodes::name(iError));
      return ERROR_SDK_FUNC_FAIL;
    }
  } else {
    return ERROR_SDK_FUNC_FAIL;
  }
  printf("Set Readout Port (Output Amplifier) to %d\n", _iReadoutPort);

  _iMaxSpeedTableIndex = -1;
  GetNumberHSSpeeds(_iADChannel, _iReadoutPort, &_iMaxSpeedTableIndex);
  --_iMaxSpeedTableIndex;
  printf("Max Speed Table Index: %d\n", _iMaxSpeedTableIndex);

  if (checkMasterSelected()) {
    iError = GetTemperature(&_iTemperatureMaster);
    printf("Current Temperature %d C  Status %s\n", _iTemperatureMaster, AndorErrorCodes::name(iError));
  }
  if (checkSlaveSelected()) {
    iError = GetTemperature(&_iTemperatureSlave);
    printf("Current Temperature %d C  Status %s\n", _iTemperatureSlave, AndorErrorCodes::name(iError));
  }

  printf( "Detector Width %d Height %d Max Speed %d Gain %d Temperature %d C %d C\n",
    _iDetectorWidth, _iDetectorHeight, _iMaxSpeedTableIndex, _iMaxGainIndex, _iTemperatureMaster, _iTemperatureSlave);

  if (_bInitTest)
  {
    if (initTest() != 0)
      return ERROR_FUNCTION_FAILURE;
  }

  int iFail = initCameraBeforeConfig();
  if (iFail != 0)
  {
    printf("DualAndorServer::init(): initCameraBeforeConfig() failed!\n");
    return ERROR_FUNCTION_FAILURE;
  }

  if (checkMasterSelected()) {
    iError = GetTemperature(&_iTemperatureMaster);
    printf("Current Temperature %d C  Status %s\n", _iTemperatureMaster, AndorErrorCodes::name(iError));
  }
  if (checkSlaveSelected()) {
    iError = GetTemperature(&_iTemperatureSlave);
    printf("Current Temperature %d C  Status %s\n", _iTemperatureSlave, AndorErrorCodes::name(iError));
  }

  printf( "Detector Width %d Height %d Max Speed %d Gain %d Temperatures %d C %d C\n",
    _iDetectorWidth, _iDetectorHeight, _iMaxSpeedTableIndex, _iMaxGainIndex, _iTemperatureMaster, _iTemperatureSlave);

  const int iDevId = ((DetInfo&)_src).devId();
  printf( "Andor Camera [%d] (device %d) has been initialized\n", iDevId, _iCamera );
  _bCameraInited = true;

  return 0;
}

static int _printCaps(AndorCapabilities &caps);

int DualAndorServer::printInfo()
{
  int   iError;
  char  sVersionInfo[128];
  iError = GetVersionInfo(AT_SDKVersion, sVersionInfo, sizeof(sVersionInfo));
  if (!isAndorFuncOk(iError))
    printf("DualAndorServer::printInfo(): GetVersionInfo(AT_SDKVersion): %s\n", AndorErrorCodes::name(iError));
  else
    printf("SDKVersion: %s\n", sVersionInfo);

  iError = GetVersionInfo(AT_DeviceDriverVersion, sVersionInfo, sizeof(sVersionInfo));
  if (!isAndorFuncOk(iError))
    printf("DualAndorServer::printInfo(): DeviceDriverVersion: %s\n", sVersionInfo);
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
    printf("DualAndorServer::printInfo(): GetSoftwareVersion(): %s\n", AndorErrorCodes::name(iError));
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
    printf("DualAndorServer::printInfo(): GetHardwareVersion(): %s\n", AndorErrorCodes::name(iError));
  else
    printf("Hardware Version: PCB %d Decode %d FirewareVer %d FirewareBuild %d\n",
      iPCB, iDecode, iCameraFirmwareVersion, iCameraFirmwareBuild);

  int iSerialNumber = -1;
  iError = GetCameraSerialNumber(&iSerialNumber);
  if (!isAndorFuncOk(iError))
    printf("DualAndorServer::printInfo(): GetCameraSerialNumber(): %s\n", AndorErrorCodes::name(iError));
  else
    printf("Camera serial number: %d\n", iSerialNumber);

  char sHeadModel[256];
  iError = GetHeadModel(sHeadModel);
  if (!isAndorFuncOk(iError))
    printf("DualAndorServer::printInfo(): GetHeadModel(): %s\n", AndorErrorCodes::name(iError));
  else
    printf("Camera Head Model: %s\n", sHeadModel);

  AndorCapabilities caps;
  iError = GetCapabilities(&caps);
  if (!isAndorFuncOk(iError))
    printf("DualAndorServer::printInfo(): GetCapabilities(): %s\n", AndorErrorCodes::name(iError));
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

  int iMaxBinH = -1;
  int iMaxBinV = -1;
  GetMaximumBinning(4, 0, &iMaxBinH);
  GetMaximumBinning(4, 1, &iMaxBinV);
  printf("Max binning in Image mode: H %d V %d\n", iMaxBinH, iMaxBinV);

  float fTimeMaxExposure;
  GetMaximumExposure(&fTimeMaxExposure);
  printf("Max exposure time: %f s\n", fTimeMaxExposure);

  int iMinImageLen = -1;
  GetMinimumImageLength (&iMinImageLen);
  printf("MinImageLen: %d\n", iMinImageLen);

  return 0;
}

int DualAndorServer::deinit()
{
  // If the camera has been init-ed before, and not deinit-ed yet
  if ( _bCaptureInited )
    deinitCapture(); // deinit the camera explicitly

  int iError = resetCooling();
  if (iError != 0)
    printf("DualAndorServer::deinit(): resetCooling() non-zero exit code: %d\n", iError);

  if (_hCamSlave != -1 && checkSlaveSelected() && _bCallShutdown)
  {
    //iError = ShutDown();
    if (!isAndorFuncOk(iError))
      printf("DualAndorServer::deinit(): ShutDown() (hcam = %d): %s\n", (int) _hCamSlave, AndorErrorCodes::name(iError));
  }

  if (_hCamMaster != -1 && checkMasterSelected() && _bCallShutdown)
  {
    iError = ShutDown();
    if (!isAndorFuncOk(iError))
      printf("DualAndorServer::deinit(): ShutDown() (hcam = %d): %s\n", (int) _hCamMaster, AndorErrorCodes::name(iError));
  }

  _bCameraInited = false;

  const int iDevId = ((DetInfo&)_src).devId();
  printf( "Andor Camera [%d] (device %d) has been deinitialized\n", iDevId, _iCamera );
  return 0;
}

int DualAndorServer::map()
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

int DualAndorServer::config(Andor3dConfigType& config, std::string& sConfigWarning)
{
  printf("Configuring slave camera (hcam = %d)\n", (int) _hCamSlave);
  if ( !checkSlaveSelected() || configCamera(config, sConfigWarning) != 0 ) {
    if (_occSend != NULL) {
      // send occurrence
      _occSend->userMessage("Andor: camera configuration failed\n");
    }
    return ERROR_SERVER_INIT_FAIL;
  }

  printf("Configuring master camera (hcam = %d)\n", (int) _hCamMaster);
  if ( !checkMasterSelected() || configCamera(config, sConfigWarning) != 0 ) {
    if (_occSend != NULL) {
      // send occurrence
      _occSend->userMessage("Andor: camera configuration failed\n");
    }
    return ERROR_SERVER_INIT_FAIL;
  }

  // Check current config won't hit the event builder contribution timeout
  if (config.numDelayShots() > 1 && (config.exposureTime() * 1000) > _iMaxReadoutTime) {
    if (_occSend != NULL) {
      // send occurrence
      _occSend->userMessage("Andor: configured exposure time exceeds event builder timeout\n");
    }
    return ERROR_INVALID_CONFIG;
  }

  // If numDelayShots <= 1 then increase the readout wait time - allows long integrations with readout groups.
  if (config.numDelayShots() > 1)
    _iReadoutWaitTime = _iMaxReadoutTime;
  else
    _iReadoutWaitTime = _iMaxReadoutTime + (int) (config.exposureTime() * 1000);

  const int iDevId = ((DetInfo&)_src).devId();

  if ( (int) config.numSensors() != _iDetectorSensor)
  {
    char sMessage[128];
    sprintf( sMessage, "!!! Andor %d Config Sensors (%d) != Det Sensors (%d)\n", iDevId, config.numSensors(), _iDetectorSensor);
    printf(sMessage);
    sConfigWarning += sMessage;
    Pds::Andor3dConfig::setNumSensors(config,_iDetectorSensor);
  }

  if ( (int) config.width() > _iDetectorWidth || (int) config.height() > _iDetectorHeight || (int) config.numSensors() != _iDetectorSensor)
  {
    char sMessage[128];
    sprintf( sMessage, "!!! Andor %d ConfigSize (%d,%d) > Det Size(%d,%d)\n", iDevId, config.width(), config.height(), _iDetectorWidth, _iDetectorHeight);
    printf(sMessage);
    sConfigWarning += sMessage;
    Pds::Andor3dConfig::setSize(config,_iDetectorWidth,_iDetectorHeight);
  }

  if ( (int) (config.orgX() + config.width()) > _iDetectorWidth ||
       (int) (config.orgY() + config.height()) > _iDetectorHeight)
  {
    char sMessage[128];
    sprintf( sMessage, "!!! Andor %d ROI Boundary (%d,%d) > Det Size(%d,%d)\n", iDevId,
             config.orgX() + config.width(), config.orgY() + config.height(), _iDetectorWidth, _iDetectorHeight);
    printf(sMessage);
    sConfigWarning += sMessage;
    Pds::Andor3dConfig::setSize (config,
                               _iDetectorWidth - config.orgX(),
                               _iDetectorHeight - config.orgY());
  }

  _config = config;

  //Note: We don't send error for cooling incomplete
  setupCooling( (double) _config.coolingTemp() );
  //if ( setupCooling() != 0 )
    //return ERROR_SERVER_INIT_FAIL;

  if (_config.numDelayShots() == 0)
    _iTriggerMode = 1;
  else
    _iTriggerMode = 0;

  if (checkMasterSelected())
  {
    int iError = GetTemperature(&_iTemperatureMaster);
    printf("Current Temperature %d C  Status %s (hcam = %d)\n", _iTemperatureMaster, AndorErrorCodes::name(iError), (int) _hCamMaster);
  }
  if (checkSlaveSelected())
  {
    int iError = GetTemperature(&_iTemperatureSlave);
    printf("Current Temperature %d C  Status %s (hcam = %d)\n", _iTemperatureSlave, AndorErrorCodes::name(iError), (int) _hCamSlave);
  }

  printf( "Detector Width %d Height %d Speed %d/%d Gain %d/%d Temperature %d C and %d C\n",
    _iDetectorWidth, _iDetectorHeight, _config.readoutSpeedIndex(), _iMaxSpeedTableIndex, _config.gainIndex(), _iMaxGainIndex, _iTemperatureMaster, _iTemperatureSlave);

  return 0;
}

int DualAndorServer::unconfig()
{
  return 0;
}

int DualAndorServer::beginRun()
{
  /*
   * Check data pool status
   */
  if ( _poolFrameData.numberOfAllocatedObjects() > 0 )
    printf( "DualAndorServer::enable(): Memory usage issue. Data Pool is not empty (%d/%d allocated).\n",
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

int DualAndorServer::endRun()
{
  /*
   * Check data pool status
   */
  if ( _poolFrameData.numberOfAllocatedObjects() > 0 )
    printf( "DualAndorServer::enable(): Memory usage issue. Data Pool is not empty (%d/%d allocated).\n",
      _poolFrameData.numberOfAllocatedObjects(), _poolFrameData.numberofObjects() );
  else if ( _iDebugLevel >= 3 )
    printf( "EndRun Pool status: %d/%d allocated.\n", _poolFrameData.numberOfAllocatedObjects(), _poolFrameData.numberofObjects() );

  return 0;
}

int DualAndorServer::beginCalibCycle()
{
  int iFail = initCapture();
  if ( iFail != 0 )
    return ERROR_FUNCTION_FAILURE;

  return 0;
}

int DualAndorServer::endCalibCycle()
{
  int iFail  = deinitCapture();
  if ( iFail != 0 )
    return ERROR_FUNCTION_FAILURE;

  return 0;
}

#include <utility>
using namespace std;
static vector<pair<float, int> > vecReadoutTimeMaster; //!!!debug
static vector<pair<float, int> > vecReadoutTimeSlave; //!!!debug

int DualAndorServer::enable()
{
  /*
   * Check data pool status
   */
  if ( _poolFrameData.numberOfAllocatedObjects() > 0 )
    printf( "DualAndorServer::enable(): Memory usage issue. Data Pool is not empty (%d/%d allocated).\n",
      _poolFrameData.numberOfAllocatedObjects(), _poolFrameData.numberofObjects() );
  else if ( _iDebugLevel >= 3 )
    printf( "Enable Pool status: %d/%d allocated.\n", _poolFrameData.numberOfAllocatedObjects(), _poolFrameData.numberofObjects() );

  if (inBeamRateMode())
  {
    printf("Starting capture...\n");
    _iNumAcq = 0;
    vecReadoutTimeMaster.clear();
    vecReadoutTimeSlave.clear();
    if (startCapture() != 0)
      return ERROR_FUNCTION_FAILURE;

    _CaptureState = CAPTURE_STATE_EXT_TRIGGER;
    printf("Capture started in Enable transition.\n");
  }

  return 0;
}

int DualAndorServer::disable()
{
  // Note: Here we don't worry if the data pool is not free, because the
  //       traffic shaping thead may still holding the L1 Data
  if ( _iDebugLevel >= 3 )
    printf( "Disable Pool status: %d/%d allocated.\n", _poolFrameData.numberOfAllocatedObjects(), _poolFrameData.numberofObjects() );

  int iFail  = stopCapture();
  if ( iFail != 0 )
    return ERROR_FUNCTION_FAILURE;

  if (inBeamRateMode())
  {
    //!!!debug
    for (int i=0; i<(int)vecReadoutTimeMaster.size(); ++i)
      printf("  [%d] master cam readout time  %f s  acq %d\n", i, vecReadoutTimeMaster[i].first, vecReadoutTimeMaster[i].second);
    for (int i=0; i<(int)vecReadoutTimeSlave.size(); ++i)
      printf("  [%d] slave cam  readout time  %f s  acq %d\n", i, vecReadoutTimeSlave[i].first, vecReadoutTimeSlave[i].second);

    at_32 numAcqNew;
    int iError = GetTotalNumberImagesAcquired(&numAcqNew);
    if (!isAndorFuncOk(iError))
      printf("DualAndorServer::disable(): GetTotalNumberImagesAcquired(): %s\n", AndorErrorCodes::name(iError));

    if (numAcqNew != _iNumAcq)
    {
      printf("DualAndorServer::disable(): data not transferred out. acquired %d frames, but only transferred %d frames.\n",
        (int) numAcqNew, _iNumAcq);
      return ERROR_FUNCTION_FAILURE;
    }
  }

  return 0;
}

int DualAndorServer::initCapture()
{
  if ( _bCaptureInited )
    deinitCapture();

  LockCameraData lockInitCapture(const_cast<char*>("DualAndorServer::initCapture()"));

  printf("\nInit capture...\n");
  timespec timeVal0;
  clock_gettime( CLOCK_REALTIME, &timeVal0 );

  int iError = setupROI();
  if (iError != 0)
    return ERROR_FUNCTION_FAILURE;

  if (checkSlaveSelected())
  {
    printf(" Initializing capture on slave (hcam = %d)\n", (int) _hCamSlave);
    iError = configCapture();
    if (iError != 0)
      return iError;
  }
  else
    return ERROR_SDK_FUNC_FAIL;

  if (checkMasterSelected())
  {
    printf(" Initializing capture on master (hcam = %d)\n", (int) _hCamMaster);
    iError = configCapture();
    if (iError != 0)
      return iError;
  }
  else
    return ERROR_SDK_FUNC_FAIL;

  if (checkSlaveSelected())
  {
    iError = PrepareAcquisition();
    if (!isAndorFuncOk(iError))
    {
      printf("DualAndorServer::initCapture(): PrepareAcquisition() (hcam = %d): %s\n", (int) _hCamSlave, AndorErrorCodes::name(iError));
      return ERROR_SDK_FUNC_FAIL;
    }
  }
  else
    return ERROR_SDK_FUNC_FAIL;

  if (checkMasterSelected())
  {
    iError = PrepareAcquisition();
    if (!isAndorFuncOk(iError))
    {
      printf("DualAndorServer::initCapture(): PrepareAcquisition() (hcam = %d): %s\n", (int) _hCamMaster, AndorErrorCodes::name(iError));
      return ERROR_SDK_FUNC_FAIL;
    }
  }
  else
    return ERROR_SDK_FUNC_FAIL;

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

int DualAndorServer::stopCapture()
{
  if ( _CaptureState == CAPTURE_STATE_IDLE )
    return 0;

  LockCameraData lockDeinitCapture(const_cast<char*>("DualAndorServer::stopCapture()"));

  int iError;

  if(checkSlaveSelected()) {
    iError = AbortAcquisition();
    if (!isAndorFuncOk(iError) && iError != DRV_IDLE)
    {
      printf("DualAndorServer::deinitCapture(): AbortAcquisition() failed (hcam = %d). %s\n", (int) _hCamSlave, AndorErrorCodes::name(iError));
      return ERROR_SDK_FUNC_FAIL;
    }
  } else {
    printf("DualAndorServer::deinitCapture(): AbortAcquisition() failed (hcam = %d). Could not select camera\n", (int) _hCamSlave);
    return ERROR_SDK_FUNC_FAIL;
  }

  if(checkMasterSelected()) {
    iError = AbortAcquisition();
    if (!isAndorFuncOk(iError) && iError != DRV_IDLE)
    {
      printf("DualAndorServer::deinitCapture(): AbortAcquisition() failed (hcam = %d). %s\n", (int) _hCamMaster, AndorErrorCodes::name(iError));
      return ERROR_SDK_FUNC_FAIL;
    }
  } else {
    printf("DualAndorServer::deinitCapture(): AbortAcquisition() failed (hcam = %d). Could not select camera\n", (int) _hCamMaster);
    return ERROR_SDK_FUNC_FAIL;
  }

  resetFrameData(true);

  if (_config.fanMode() == (int) Andor3dConfigType::ENUM_FAN_ACQOFF && !inBeamRateMode())
  {
    if(checkSlaveSelected()) {
      iError = SetFanMode((int) Andor3dConfigType::ENUM_FAN_FULL);
      if (!isAndorFuncOk(iError))
        printf("DualAndorServer::stopCapture(): SetFanMode(%d) (hcam = %d): %s\n", (int) Andor3dConfigType::ENUM_FAN_FULL, (int) _hCamSlave, AndorErrorCodes::name(iError));
    } else {
      printf("DualAndorServer::stopCapture(): SetFanMode(%d) (hcam = %d): Could not select camera\n", (int) Andor3dConfigType::ENUM_FAN_FULL, (int) _hCamSlave);
    }

    if(checkMasterSelected()) {
      iError = SetFanMode((int) Andor3dConfigType::ENUM_FAN_FULL);
      if (!isAndorFuncOk(iError))
        printf("DualAndorServer::stopCapture(): SetFanMode(%d) (hcam = %d): %s\n", (int) Andor3dConfigType::ENUM_FAN_FULL, (int) _hCamMaster, AndorErrorCodes::name(iError));
    } else {
      printf("DualAndorServer::stopCapture(): SetFanMode(%d) (hcam = %d): Could not select camera\n", (int) Andor3dConfigType::ENUM_FAN_FULL, (int) _hCamMaster);
    }
  }

  _CaptureState = CAPTURE_STATE_IDLE;
  printf( "Capture stopped\n" );
  return 0;
}

int DualAndorServer::deinitCapture()
{
  if ( !_bCaptureInited )
    return 0;

  stopCapture();

  LockCameraData lockDeinitCapture(const_cast<char*>("DualAndorServer::deinitCapture()"));

  _bCaptureInited = false;
  return 0;
}

int DualAndorServer::startCapture()
{
  if (!inBeamRateMode())
  {
    if (_pDgOut == NULL )
    {
      printf( "DualAndorServer::startCapture(): Datagram has not been allocated. No buffer to store the image data\n" );
      return ERROR_LOGICAL_FAILURE;
    }

    if (_config.fanMode() == (int) Andor3dConfigType::ENUM_FAN_ACQOFF)
    {
      int iError;
      if (checkSlaveSelected()) {
        iError = SetFanMode((int) Andor3dConfigType::ENUM_FAN_OFF);
        if (!isAndorFuncOk(iError))
          printf("DualAndorServer::startCapture(): SetFanMode(%d) (hcam = %d): %s\n", (int) Andor3dConfigType::ENUM_FAN_OFF, (int) _hCamSlave, AndorErrorCodes::name(iError));
      }
      if (checkMasterSelected()) {
        iError = SetFanMode((int) Andor3dConfigType::ENUM_FAN_OFF);
        if (!isAndorFuncOk(iError))
          printf("DualAndorServer::startCapture(): SetFanMode(%d) (hcam = %d): %s\n", (int) Andor3dConfigType::ENUM_FAN_OFF, (int) _hCamMaster, AndorErrorCodes::name(iError));
      }
    }
  }

  if (checkSlaveSelected())
  {
    int iError = StartAcquisition();
    if (!isAndorFuncOk(iError))
    {
      printf("DualAndorServer::startCapture(): StartAcquisition() (hcam = %d): %s\n", (int) _hCamSlave, AndorErrorCodes::name(iError));
      return ERROR_SDK_FUNC_FAIL;
    }
  } 
  else
    return ERROR_SDK_FUNC_FAIL;

  // Sleep for configured time before starting acq on master
  timeval timeSleepMicro = {0, _config.exposureStartDelay()};
  // Use select() to simulate nanosleep(), because experimentally select() controls the sleeping time more precisely
  select( 0, NULL, NULL, NULL, &timeSleepMicro);

  if (checkMasterSelected())
  {
    int iError = StartAcquisition();
    if (!isAndorFuncOk(iError))
    {
      printf("DualAndorServer::startCapture(): StartAcquisition() (hcam = %d): %s\n", (int) _hCamMaster, AndorErrorCodes::name(iError));
      return ERROR_SDK_FUNC_FAIL;
    }
  }
  else
    return ERROR_SDK_FUNC_FAIL;

  return 0;
}

int DualAndorServer::configCapture()
{
  //Set initial exposure time
  int iError = SetExposureTime(_config.exposureTime());
  if (!isAndorFuncOk(iError))
  {
    printf("DualAndorServer::initCapture(): SetExposureTime(): %s\n", AndorErrorCodes::name(iError));
    return ERROR_SDK_FUNC_FAIL;
  }

  if ( int(_config.frameSize() - sizeof(Andor3dDataType) + _iFrameHeaderSize) > _iMaxFrameDataSize )
  {
    printf( "DualAndorServer::initCapture(): Frame size (%i) + Frame header size (%d)"
     "is larger than internal data frame buffer size (%d)\n",
     _config.frameSize() - (int) sizeof(Andor3dDataType), _iFrameHeaderSize, _iMaxFrameDataSize );
    return ERROR_INVALID_CONFIG;
  }

  //
  // Acquisition Mode: 1 Single Scan , 2 Accumulate , 3 Kinetics , 4 Fast Kinetics , 5 Run till abort
  //
  iError = SetAcquisitionMode(inBeamRateMode() ? 5 : 1);
  if (!isAndorFuncOk(iError))
  {
    printf("DualAndorServer::initCapture(): SetAcquisitionMode(): %s\n", AndorErrorCodes::name(iError));
    return ERROR_SDK_FUNC_FAIL;
  }

  //
  // Trigger Mode: 0 Internel , 1 External , 6 External Start
  //               7 External Exposure (Bulb) , 9 Ext FVB EM
  //               10 Software Trigger  12 Ext Charge Shifting
  //
  iError = SetTriggerMode(_iTriggerMode);
  if (!isAndorFuncOk(iError))
  {
    printf("DualAndorServer::initCapture(): SetTriggerMode(): %s\n", AndorErrorCodes::name(iError));
    return ERROR_SDK_FUNC_FAIL;
  }
  if (inBeamRateMode())
  {
    //iError = SetFastExtTrigger(1);
    //if (!isAndorFuncOk(iError))
    //{
    //  printf("DualAndorServer::initCapture(): SetFastExtTrigger(): %s\n", AndorErrorCodes::name(iError));
    //  return ERROR_SDK_FUNC_FAIL;
    //}
    //iError = SetAccumulationCycleTime(0);
    //if (!isAndorFuncOk(iError))
    //  printf("DualAndorServer::initCapture(): SetAccumulationCycleTime(): %s\n", AndorErrorCodes::name(iError));

    iError = SetKineticCycleTime(0);
    if (!isAndorFuncOk(iError))
      printf("DualAndorServer::initCapture(): SetKineticCycleTime(): %s\n", AndorErrorCodes::name(iError));

    at_32 sizeBuffer;
    iError = GetSizeOfCircularBuffer(&sizeBuffer);
    if (!isAndorFuncOk(iError))
      printf("DualAndorServer::initCapture(): GetSizeOfCircularBuffer(): %s\n", AndorErrorCodes::name(iError));
    else
      printf("Size of Circular Buffer: %d\n", (int) sizeBuffer);

    int iVSRecIndex = 0; //select the fastest VShift speed
    iError = SetVSSpeed(iVSRecIndex);
    if (!isAndorFuncOk(iError))
    {
      printf("DualAndorServer::init(): SetVSSpeed(): %s\n", AndorErrorCodes::name(iError));
      return ERROR_SDK_FUNC_FAIL;
    }
    printf("Set VSSpeed to %d\n", iVSRecIndex);

    int iKeepCleanMode = 0; // Off
    iError = EnableKeepCleans(iKeepCleanMode);
    if (!isAndorFuncOk(iError))
      printf("DualAndorServer::initCapture(): EnableKeepCleans(off): %s\n", AndorErrorCodes::name(iError));
  }

  //
  // SetShutter(int typ, int mode, int closingtime, int openingtime)
  //
  //  typ: 0 Output TTL low signal to open shutter 1 Output TTL high signal to open shutter
  // mode: 0 Auto , 1 Open , 2 Close
  iError = SetShutter(1, 0, 0, 0);
  if (!isAndorFuncOk(iError))
  {
    printf("DualAndorServer::initCapture(): SetShutter(): %s\n", AndorErrorCodes::name(iError));
    return ERROR_SDK_FUNC_FAIL;
  }

  //
  // SetShutterEx(int typ, int mode, int closingtime, int openingtime, int extmode)
  //
  //          typ: 0 Output TTL low signal to open shutter 1 Output TTL high signal to open shutter
  // mode/extmode: 0 Auto , 1 Open , 2 Close
  iError = SetShutterEx(1, 0, 0, 0, 0);
  if (!isAndorFuncOk(iError))
  {
    printf("DualAndorServer::initCapture(): SetShutterEx(): %s\n", AndorErrorCodes::name(iError));
    return ERROR_SDK_FUNC_FAIL;
  }

  iError = SetDMAParameters(1, 0.001);
  if (!isAndorFuncOk(iError))
    printf("DualAndorServer::initCapture(): SetDMAParameters(): %s\n", AndorErrorCodes::name(iError));

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
  printf("Estimated Readout time: %f s\n", fTimeReadout);

  return 0;
}

int DualAndorServer::configCamera(Andor3dConfigType& config, std::string& sConfigWarning)
{
  int   iError;

  printf("\nConfiguring...\n");
  if (config.fanMode() == (int) Andor3dConfigType::ENUM_FAN_ACQOFF)
  {
    printf("Fan Mode: Acq Off\n");
    _iFanModeNonAcq = (int) Andor3dConfigType::ENUM_FAN_FULL;
  }
  else
    _iFanModeNonAcq = config.fanMode();
  iError = SetFanMode(_iFanModeNonAcq);
  if (!isAndorFuncOk(iError))
  {
    printf("DualAndorServer::configCamera(): SetFanMode(%d): %s\n", _iFanModeNonAcq, AndorErrorCodes::name(iError));
    return ERROR_SDK_FUNC_FAIL;
  }
  else
    printf("Set Fan Mode      to %d\n", _iFanModeNonAcq);

  iError = SetBaselineClamp(config.baselineClamp());
  if (!isAndorFuncOk(iError))
  {
    printf("DualAndorServer::configCamera(): SetBaselineClamp(%d): %s\n", config.baselineClamp(), AndorErrorCodes::name(iError));
    return ERROR_INVALID_CONFIG;
  }
  else
    printf("Set BaselineClamp to %d\n", config.baselineClamp());

  iError = SetHighCapacity(config.highCapacity());
  if (!isAndorFuncOk(iError))
  {
    printf("DualAndorServer::configCamera(): SetHighCapacity(%d): %s\n", config.highCapacity(), AndorErrorCodes::name(iError));
    return ERROR_INVALID_CONFIG;
  }
  else
    printf("Set HighCapacity  to %d\n", config.highCapacity());

  int iNumHSSpeed = -1;
  GetNumberHSSpeeds(_iADChannel, _iReadoutPort, &iNumHSSpeed);
  if (config.readoutSpeedIndex() >= iNumHSSpeed)
  {
    printf("DualAndorServer::configCamera(): Speed Index %d out of range (max index %d)\n", config.readoutSpeedIndex(), iNumHSSpeed);
    return ERROR_INVALID_CONFIG;
  }

  iError = SetHSSpeed(_iReadoutPort, config.readoutSpeedIndex());
  if (!isAndorFuncOk(iError))
  {
    printf("DualAndorServer::configCamera(): SetHSSpeed(%d,%d): %s\n", _iReadoutPort, config.readoutSpeedIndex(), AndorErrorCodes::name(iError));
    return ERROR_INVALID_CONFIG;
  }

  float fSpeed = -1;
  GetHSSpeed(_iADChannel, _iReadoutPort, config.readoutSpeedIndex(), &fSpeed);
  printf("Set Speed Index to %d: %f MHz\n", config.readoutSpeedIndex(), fSpeed);

  int iNumGain = -1;
  GetNumberPreAmpGains(&iNumGain);
  if (config.gainIndex() >= iNumGain)
  {
    printf("DualAndorServer::configCamera(): Gain Index %d out of range (max index %d)\n", config.gainIndex(), iNumGain);
    return ERROR_INVALID_CONFIG;
  }

  int iStatus = -1;
  IsPreAmpGainAvailable(_iADChannel, _iReadoutPort, config.readoutSpeedIndex(), config.gainIndex(), &iStatus);
  if (iStatus != 1)
  {
    printf("DualAndorServer::configCamera(): Gain Index %d not supported for channel %d port %d speed %d\n", config.gainIndex(),
      _iADChannel, _iReadoutPort, config.readoutSpeedIndex());
    return ERROR_INVALID_CONFIG;
  }

  iError = SetPreAmpGain(config.gainIndex());
  if (!isAndorFuncOk(iError))
  {
    printf("DualAndorServer::configCamera(): SetGain(%d): %s\n", config.gainIndex(), AndorErrorCodes::name(iError));
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

int DualAndorServer::initCameraBeforeConfig()
{
  if (_sConfigDb.empty())
    return 0;

  size_t iIndexComma = _sConfigDb.find_first_of(',');

  string sConfigPath, sConfigType;
  if (iIndexComma == string::npos)
  {
    sConfigPath = _sConfigDb;
    sConfigType = "BEAM";
  }
  else
  {
    sConfigPath = _sConfigDb.substr(0, iIndexComma);
    sConfigType = _sConfigDb.substr(iIndexComma+1, string::npos);
  }

  // Setup ConfigDB and Run Key
  int runKey;
  { Pds_ConfigDb::Experiment expt(sConfigPath.c_str(),
                                  Pds_ConfigDb::Experiment::NoLock);
    expt.read();
    const Pds_ConfigDb::TableEntry* entry = expt.table().get_top_entry(sConfigType);
    if (entry == NULL)
      {
        printf("DualAndorServer::initCameraBeforeConfig(): Invalid config db path [%s] type [%s]\n",sConfigPath.c_str(), sConfigType.c_str());
        return ERROR_FUNCTION_FAILURE;
      }
    runKey = strtoul(entry->key().c_str(),NULL,16);
  }

  const TypeId typeAndor3dConfig = TypeId(TypeId::Id_Andor3dConfig, Andor3dConfigType::Version);

  CfgClientNfs client(_src);
  client.initialize(Allocation("none",sConfigPath.c_str(),0));
  Transition tr(TransitionId::BeginRun, Env(runKey));
  Andor3dConfigType config;
  int iSizeRead=client.fetch(tr, typeAndor3dConfig, &config, sizeof(config));
  if (iSizeRead != sizeof(config)) {
    printf("DualAndorServer::initCameraBeforeConfig(): Read config data of incorrect size. Read size = %d (should be %d) bytes\n",
      iSizeRead, (int) sizeof(config));
    return ERROR_INVALID_CONFIG;
  }

  printf("Setting cooling temperature: %f\n", config.coolingTemp());
  setupCooling( (double) config.coolingTemp() );

  return 0;
}

int DualAndorServer::initTest()
{
  printf( "Running init test...\n" );

  timespec timeVal0;
  clock_gettime( CLOCK_REALTIME, &timeVal0 );

  int iError;

  if (checkSlaveSelected()) {
    iError = SetHSSpeed(_iReadoutPort, 0);
    if (!isAndorFuncOk(iError))
    {
      printf("DualAndorServer::initTest(): SetHSSpeed(%d,%d) (hcam = %d): %s\n", _iReadoutPort, 0, (int) _hCamSlave, AndorErrorCodes::name(iError));
      return ERROR_SDK_FUNC_FAIL;
    }

    iError = SetExposureTime(0.001);
    if (!isAndorFuncOk(iError))
    {
      printf("DualAndorServer::initTest(): SetExposureTime() (hcam = %d): %s\n", (int) _hCamSlave, AndorErrorCodes::name(iError));
      return ERROR_SDK_FUNC_FAIL;
    }

    iError = SetImage(1, 1, 1, 128, 1, 128);
    if (!isAndorFuncOk(iError))
    {
      printf("DualAndorServer::initTest(): SetImage() (hcam = %d): %s\n", (int) _hCamSlave, AndorErrorCodes::name(iError));
      return ERROR_SDK_FUNC_FAIL;
    }
  } else {
    return ERROR_SDK_FUNC_FAIL;
  }

  if (checkMasterSelected()) {
    iError = SetHSSpeed(_iReadoutPort, 0);
    if (!isAndorFuncOk(iError))
    {
      printf("DualAndorServer::initTest(): SetHSSpeed(%d,%d) (hcam = %d): %s\n", _iReadoutPort, 0, (int) _hCamMaster, AndorErrorCodes::name(iError));
      return ERROR_SDK_FUNC_FAIL;
    }

    iError = SetExposureTime(0.001);
    if (!isAndorFuncOk(iError))
    {
      printf("DualAndorServer::initTest(): SetExposureTime() (hcam = %d): %s\n", (int) _hCamMaster, AndorErrorCodes::name(iError));
      return ERROR_SDK_FUNC_FAIL;
    }

    iError = SetImage(1, 1, 1, 128, 1, 128);
    if (!isAndorFuncOk(iError))
    {
      printf("DualAndorServer::initTest(): SetImage() (hcam = %d): %s\n", (int) _hCamMaster, AndorErrorCodes::name(iError));
      return ERROR_SDK_FUNC_FAIL;
    }
  } else {
    return ERROR_SDK_FUNC_FAIL;
  }

  float fTimeReadout = -1;
  GetReadOutTime(&fTimeReadout);
  printf("Estimated Readout time: %f s\n", fTimeReadout);

  timespec timeVal1;
  clock_gettime( CLOCK_REALTIME, &timeVal1 );

  if (checkSlaveSelected()) {
    iError = StartAcquisition();
    if (!isAndorFuncOk(iError))
    {
      printf("DualAndorServer::initTest(): StartAcquisition() (hcam = %d): %s\n", (int) _hCamSlave, AndorErrorCodes::name(iError));
      return ERROR_SDK_FUNC_FAIL;
    }
  } else {
    return ERROR_SDK_FUNC_FAIL;
  }

  // Sleep for configured time before starting acq on master
  timeval timeSleepMicro = {0, _iTestExposureStartDelay};
  // Use select() to simulate nanosleep(), because experimentally select() controls the sleeping time more precisely
  select( 0, NULL, NULL, NULL, &timeSleepMicro);

  if (checkMasterSelected()) {
    iError = StartAcquisition();
    if (!isAndorFuncOk(iError))
    {
      printf("DualAndorServer::initTest(): StartAcquisition() (hcam = %d): %s\n", (int) _hCamMaster, AndorErrorCodes::name(iError));
      return ERROR_SDK_FUNC_FAIL;
    }
  } else {
    return ERROR_SDK_FUNC_FAIL;
  }

  timespec timeVal2;
  clock_gettime( CLOCK_REALTIME, &timeVal2 );

  if (checkMasterSelected()) {
    iError = WaitForAcquisitionTimeOut(_iReadoutWaitTime);
    if (!isAndorFuncOk(iError))
    {
      printf("DualAndorServer::waitForNewFrameAvailable(): WaitForAcquisitionTimeOut() (hcam = %d): %s\n", (int) _hCamMaster, AndorErrorCodes::name(iError));
      return ERROR_SDK_FUNC_FAIL;
    }
  } else {
    return ERROR_SDK_FUNC_FAIL;
  }

  if (checkSlaveSelected()) {
    iError = WaitForAcquisitionTimeOut(_iReadoutWaitTime);
    if (!isAndorFuncOk(iError))
    {
      printf("DualAndorServer::waitForNewFrameAvailable(): WaitForAcquisitionTimeOut() (hcam = %d): %s\n", (int) _hCamSlave, AndorErrorCodes::name(iError));
      return ERROR_SDK_FUNC_FAIL;
    }
  } else {
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
  //    printf("DualAndorServer::initTest(): GetStatus(): %s\n", AndorErrorCodes::name(status));
  //    break;
  //  }
  //
  //  timeval timeSleepMicro = {0, 1000}; // 1 ms
  //  // use select() to simulate nanosleep(), because experimentally select() controls the sleeping time more precisely
  //  select( 0, NULL, NULL, NULL, &timeSleepMicro);
  //}

  timespec timeVal3;
  clock_gettime( CLOCK_REALTIME, &timeVal3 );

  double fInitTime    = (timeVal1.tv_nsec - timeVal0.tv_nsec) * 1.e-6 + ( timeVal1.tv_sec - timeVal0.tv_sec ) * 1.e3;
  double fStartupTime = (timeVal2.tv_nsec - timeVal1.tv_nsec) * 1.e-6 + ( timeVal2.tv_sec - timeVal1.tv_sec ) * 1.e3;
  double fPollingTime = (timeVal3.tv_nsec - timeVal2.tv_nsec) * 1.e-6 + ( timeVal3.tv_sec - timeVal2.tv_sec ) * 1.e3;
  double fSingleFrameTime = fStartupTime + fPollingTime;
  printf("Capture Init Time = %6.1lfms Setup Time = %6.1lfms Total Time = %6.1lfms\n",
    fInitTime, fStartupTime, fSingleFrameTime );

  return 0;
}

int DualAndorServer::resetCooling()
{
  int iError;
  int iStatus = 0;

  if (_hCamSlave != -1 && checkSlaveSelected())
  {
    iError        = GetTemperature(&_iTemperatureSlave);
    printf("Current Temperature %d C  Status %s (hcam = %d)\n", _iTemperatureSlave, AndorErrorCodes::name(iError), (int) _hCamSlave);

    if ( _iTemperatureSlave < 0 )
      printf("Warning: Temperature is still low (%d C). May results in fast warming.\n", _iTemperatureSlave);
    else
    {
      iError = CoolerOFF();
      if (!isAndorFuncOk(iError)) {
        printf("DualAndorServer::deinit():: CoolerOFF() (hcam = %d): %s\n", (int) _hCamSlave, AndorErrorCodes::name(iError));
        iStatus = ERROR_SDK_FUNC_FAIL;
      }
    }
  }

  if (_hCamMaster != -1 && checkMasterSelected())
  {
    iError        = GetTemperature(&_iTemperatureMaster);
    printf("Current Temperature %d C  Status %s (hcam = %d)\n", _iTemperatureMaster, AndorErrorCodes::name(iError), (int) _hCamMaster);

    if ( _iTemperatureMaster < 0 )
      printf("Warning: Temperature is still low (%d C). May results in fast warming.\n", _iTemperatureMaster);
    else
    {
      iError = CoolerOFF();
      if (!isAndorFuncOk(iError)) {
        printf("DualAndorServer::deinit():: CoolerOFF() (hcam = %d): %s\n", (int) _hCamMaster, AndorErrorCodes::name(iError));
        iStatus = ERROR_SDK_FUNC_FAIL;
      }
    }
  }

  return iStatus;
}

int DualAndorServer::setupCooling(double fCoolingTemperature)
{
  int iErrorMaster;
  int iErrorSlave;

  if (checkMasterSelected()) {
    iErrorMaster = GetTemperature(&_iTemperatureMaster);
    printf("Temperature Before cooling: %d C  Status %s\n", _iTemperatureMaster, AndorErrorCodes::name(iErrorMaster));
  } else {
    return ERROR_SDK_FUNC_FAIL;
  }
  if (checkSlaveSelected()) {
    iErrorSlave = GetTemperature(&_iTemperatureSlave);
    printf("Temperature Before cooling: %d C  Status %s\n", _iTemperatureSlave, AndorErrorCodes::name(iErrorSlave));
  } else {
    return ERROR_SDK_FUNC_FAIL;
  }

  if (fCoolingTemperature > _iTempMax) {
    printf("Cooling temperature %f above valid range (max %d).  Turning cooler OFF.\n", fCoolingTemperature, _iTempMax);
    if (!checkMasterSelected())
      return ERROR_SDK_FUNC_FAIL;
    iErrorMaster = CoolerOFF();
    if (!checkSlaveSelected())
      return ERROR_SDK_FUNC_FAIL;
    iErrorSlave = CoolerOFF();
    if (!isAndorFuncOk(iErrorMaster) || !isAndorFuncOk(iErrorSlave)) {
      if (!isAndorFuncOk(iErrorMaster))
        printf("DualAndorServer::setupCooling():: CoolerOFF() (hcam = %d): %s\n", (int) _hCamMaster, AndorErrorCodes::name(iErrorMaster));
      if (!isAndorFuncOk(iErrorSlave))
        printf("DualAndorServer::setupCooling():: CoolerOFF() (hcam = %d): %s\n", (int) _hCamSlave, AndorErrorCodes::name(iErrorSlave));
      if (_occSend != NULL) {
        // send occurrence
        _occSend->userMessage("Andor: requested cooling temperature above valid range, turning cooler off FAILED\n");
      }
      return ERROR_SDK_FUNC_FAIL;
    } else {
      if (_occSend != NULL) {
        // send occurrence
        _occSend->userMessage("Andor: requested cooling temperature above valid range, cooler turned OFF\n");
      }
      return 0;
    }
  }

  if (fCoolingTemperature < _iTempMin)
  {
    printf("Cooling temperature %f below valid range (min %d)\n", fCoolingTemperature, _iTempMin);
    if (_occSend != NULL) {
      // send occurrence
      _occSend->userMessage("Andor: requested cooling temperature below valid range\n");
    }
    return ERROR_TEMPERATURE;
  }
  else
  {
    if (!checkMasterSelected())
      return ERROR_SDK_FUNC_FAIL;
    iErrorMaster = SetTemperature((int)fCoolingTemperature);
    if (!checkSlaveSelected())
      return ERROR_SDK_FUNC_FAIL;
    iErrorSlave  = SetTemperature((int)fCoolingTemperature);
    if (!isAndorFuncOk(iErrorMaster) || !isAndorFuncOk(iErrorSlave))
    {
      if (!isAndorFuncOk(iErrorMaster))
        printf("DualAndorServer::setupCooling(): SetTemperature(%d): %s\n", (int)fCoolingTemperature, AndorErrorCodes::name(iErrorMaster));
      if (!isAndorFuncOk(iErrorSlave))
        printf("DualAndorServer::setupCooling(): SetTemperature(%d): %s\n", (int)fCoolingTemperature, AndorErrorCodes::name(iErrorSlave));
      return ERROR_SDK_FUNC_FAIL;
    }
    if (!checkMasterSelected())
      return ERROR_SDK_FUNC_FAIL;
    iErrorMaster = CoolerON();
    if (!checkSlaveSelected())
      return ERROR_SDK_FUNC_FAIL;
    iErrorSlave  = CoolerON();
    if (!isAndorFuncOk(iErrorMaster) || !isAndorFuncOk(iErrorSlave))
    {
      if (!isAndorFuncOk(iErrorMaster))
        printf("DualAndorServer::setupCooling(): CoolerON(): %s\n", AndorErrorCodes::name(iErrorMaster));
      if (!isAndorFuncOk(iErrorSlave))
        printf("DualAndorServer::setupCooling(): CoolerON(): %s\n", AndorErrorCodes::name(iErrorSlave));
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
    if (checkMasterSelected())
      iErrorMaster = GetTemperature(&_iTemperatureMaster);
    if (checkSlaveSelected())
      iErrorSlave  = GetTemperature(&_iTemperatureSlave);

    if ( _iTemperatureMaster <= fCoolingTemperature && _iTemperatureSlave <= fCoolingTemperature)
    {
      if ( ++iRead >= iNumRepateRead )
        break;
    }
    else
      iRead = 0;

    if ( (iNumLoop+1) % 200 == 0 )
      printf("Temperature *Updating*: %d C, %d C\n", _iTemperatureMaster, _iTemperatureSlave );

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

  int iCoolerStatusMaster = -1;
  int iCoolerStatusSlave = -1;

  if (checkMasterSelected()) {
    iErrorMaster = IsCoolerOn(&iCoolerStatusMaster);
    if (!isAndorFuncOk(iErrorMaster))
      printf("DualAndorServer::setupCooling(): IsCoolerOn(): %s\n", AndorErrorCodes::name(iErrorMaster));
    iErrorMaster = GetTemperature(&_iTemperatureMaster);
    printf("Temperature After cooling: %d C  Status %s Cooler %d\n", _iTemperatureMaster, AndorErrorCodes::name(iErrorMaster), iCoolerStatusMaster);
  }
  if (checkSlaveSelected()) {
    iErrorSlave = IsCoolerOn(&iCoolerStatusSlave);
    if (!isAndorFuncOk(iErrorSlave))
      printf("DualAndorServer::setupCooling(): IsCoolerOn(): %s\n", AndorErrorCodes::name(iErrorSlave));
    iErrorSlave = GetTemperature(&_iTemperatureSlave);
    printf("Temperature After cooling: %d C  Status %s Cooler %d\n", _iTemperatureSlave, AndorErrorCodes::name(iErrorSlave), iCoolerStatusSlave);
  }

  if ( _iTemperatureMaster > fCoolingTemperature || _iTemperatureSlave > fCoolingTemperature )
  {
    printf("DualAndorServer::setupCooling(): Cooling temperature not reached yet; final temperatures = %d C, %d C",
     _iTemperatureMaster, _iTemperatureSlave );
    return ERROR_TEMPERATURE;
  }

  return 0;
}

int DualAndorServer::initCaptureTask()
{
  if ( _pTaskCapture != NULL )
    return 0;

  if ( ! _bDelayMode ) // Prompt mode doesn't need to initialize the capture task, because the task will be performed in the event handler
    return 0;

  _pTaskCapture = new Task(TaskObject("DualAndorServer"));

  return 0;
}

int DualAndorServer::runCaptureTask()
{
  if ( _CaptureState != CAPTURE_STATE_RUN_TASK )
  {
    printf( "DualAndorServer::runCaptureTask(): _CaptureState = %d. Capture task is not initialized correctly\n",
     _CaptureState );
    return ERROR_INCORRECT_USAGE;
  }

  LockCameraData lockCaptureProcess(const_cast<char*>("DualAndorServer::runCaptureTask(): Start data polling and processing" ));

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
    printf( "DualAndorServer::runCaptureTask(): Datagram or frame data have not been properly set up before the capture task\n" );
    resetFrameData(true);
    return ERROR_INCORRECT_USAGE;
  }

  //!!!debug
  static const char sTimeFormat[40] = "%02d_%02H:%02M:%02S"; /* Time format string */
  char      sTimeText[40];
  timespec  timeCurrent;
  time_t    timeSeconds ;

  int iFail = 0;
  do
  {
    ////!!!debug
    //clock_gettime( CLOCK_REALTIME, &timeCurrent );
    //timeSeconds = timeCurrent.tv_sec;
    //strftime(sTimeText, sizeof(sTimeText), sTimeFormat, localtime(&timeSeconds));
    //printf("DualAndorServer::runCaptureTask(): Before waitForNewFrameAvailable(): Local Time: %s.%09ld\n", sTimeText, timeCurrent.tv_nsec);

    iFail  = waitForNewFrameAvailable();

    ////!!!debug
    //clock_gettime( CLOCK_REALTIME, &timeCurrent );
    //timeSeconds = timeCurrent.tv_sec;
    //strftime(sTimeText, sizeof(sTimeText), sTimeFormat, localtime(&timeSeconds));
    //printf("DualAndorServer::runCaptureTask(): After waitForNewFrameAvailable(): Local Time: %s.%09ld\n", sTimeText, timeCurrent.tv_nsec);

    // Even if waitForNewFrameAvailable() failed, we still fill in the frame data with ShotId information
    iFail |= processFrame();

    //!!!debug
    clock_gettime( CLOCK_REALTIME, &timeCurrent );
    timeSeconds = timeCurrent.tv_sec;
    strftime(sTimeText, sizeof(sTimeText), sTimeFormat, localtime(&timeSeconds));
    printf("DualAndorServer::runCaptureTask(): After processFrame(): Local Time: %s.%09ld\n", sTimeText, timeCurrent.tv_nsec);
    
    updateTemperatureData();
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
  //JBT  updateTemperatureData();
  //if ( updateTemperatureData() != 0 )
  //  _pDgOut->datagram().xtc.damage.increase(Pds::Damage::UserDefined);

  _CaptureState = CAPTURE_STATE_DATA_READY;

  //!!!debug
  clock_gettime( CLOCK_REALTIME, &timeCurrent );
  timeSeconds = timeCurrent.tv_sec;
  strftime(sTimeText, sizeof(sTimeText), sTimeFormat, localtime(&timeSeconds));
  printf("After capture: Local Time: %s.%09ld\n", sTimeText, timeCurrent.tv_nsec);

  return 0;
}

int DualAndorServer::startExposure()
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
      printf( "DualAndorServer::startExposure(): Previous image data has not been sent out\n" );
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

    printf( "DualAndorServer::startExposure(): Capture task is running. It is impossible to start a new capture.\n" );

    /*
     * Here we don't reset the frame data, because the capture task is running and will use the data later
     */
    return ERROR_INCORRECT_USAGE; // No error for adaptive mode
  }

  //!!!debug
  static const char sTimeFormat[40] = "%02d_%02H:%02M:%02S"; /* Time format string */
  char      sTimeText[40];
  timespec  timeCurrent;
  time_t    timeSeconds ;

  ////!!!debug
  //clock_gettime( CLOCK_REALTIME, &timeCurrent );
  //timeSeconds = timeCurrent.tv_sec;
  //strftime(sTimeText, sizeof(sTimeText), sTimeFormat, localtime(&timeSeconds));
  //printf("DualAndorServer::startExposure(): Before setupFrame(): Local Time: %s.%09ld\n", sTimeText, timeCurrent.tv_nsec);

  int iFail = 0;
  do
  {
    iFail = setupFrame();
    if ( iFail != 0 ) break;

    ////!!!debug
    //clock_gettime( CLOCK_REALTIME, &timeCurrent );
    //timeSeconds = timeCurrent.tv_sec;
    //strftime(sTimeText, sizeof(sTimeText), sTimeFormat, localtime(&timeSeconds));
    //printf("DualAndorServer::startExposure(): After setupFrame(): Local Time: %s.%09ld\n", sTimeText, timeCurrent.tv_nsec);

    iFail = startCapture();
    if ( iFail != 0 ) break;

    //!!!debug
    clock_gettime( CLOCK_REALTIME, &timeCurrent );
    timeSeconds = timeCurrent.tv_sec;
    strftime(sTimeText, sizeof(sTimeText), sTimeFormat, localtime(&timeSeconds));
    printf("DualAndorServer::startExposure(): After startCapture(): Local Time: %s.%09ld\n", sTimeText, timeCurrent.tv_nsec);
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

int DualAndorServer::getData(InDatagram* in, InDatagram*& out)
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
    printf( "DualAndorServer::getData(): Datagram is not properly set up\n" );
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
  new (pFrameHeader) Andor3dDataType(in->datagram().seq.stamp().fiducials(), _fReadoutTime);

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

int DualAndorServer::waitData(InDatagram* in, InDatagram*& out)
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
      printf( "DualAndorServer::waitData(): Waiting time is too long. Skip the final data\n" );
      return ERROR_FUNCTION_FAILURE;
    }
  } // while (1)

  return getData(in, out);
}

bool DualAndorServer::isCapturingData()
{
  return ( _CaptureState != CAPTURE_STATE_IDLE );
}

bool DualAndorServer::inBeamRateMode()
{
  return ( _iTriggerMode == 1 );
}

int DualAndorServer::getTemperatureMaster(bool bForceUpdate)
{
  if (bForceUpdate) {
    if (checkMasterSelected())
      GetTemperature(&_iTemperatureMaster);
  }

  return _iTemperatureMaster;
}

int DualAndorServer::getTemperatureSlave(bool bForceUpdate)
{
  if (bForceUpdate) {
    if (checkSlaveSelected())
      GetTemperature(&_iTemperatureSlave);
  }

  return _iTemperatureSlave;
}

int DualAndorServer::getDataInBeamRateMode(InDatagram* in, InDatagram*& out)
{
  out = in; // Default: return empty stream

  //LockCameraData lockGetDataInBeamRateMode(const_cast<char*>("DualAndorServer::getDataInBeamRateMode()"));

  if ( _CaptureState != CAPTURE_STATE_EXT_TRIGGER )
    return 0;

  if ( _poolFrameData.numberOfFreeObjects() <= 0 )
  {
    printf( "DualAndorServer::getDataInBeamRateMode(): Pool is full, and cannot provide buffer for new datagram\n" );
    return ERROR_LOGICAL_FAILURE;
  }

  out =
    new ( &_poolFrameData ) CDatagram( TypeId(TypeId::Any,0), DetInfo(0,DetInfo::NoDetector,0,DetInfo::NoDevice,0) );
  out->datagram().xtc.alloc( sizeof(Xtc) + _config.frameSize() );

  static timespec tsWaitStart;
  clock_gettime( CLOCK_REALTIME, &tsWaitStart );

  int       iWaitTime       = 0;
  const int iWaitPerIter    = 4; // 1milliseconds
  at_32     numAcqNewMaster = _iNumAcq;
  at_32     numAcqNewSlave  = _iNumAcq;
  static const int  iMaxReadoutTime = 2000;
  static bool       bPrevCatpureFailed;
  if (_iNumAcq == 0)
    bPrevCatpureFailed = false;

  while ((numAcqNewMaster == _iNumAcq || numAcqNewSlave == _iNumAcq) && iWaitTime < iMaxReadoutTime)
  {
    if (checkMasterSelected()) {
      int iError = GetTotalNumberImagesAcquired(&numAcqNewMaster);
      if (!isAndorFuncOk(iError))
      {
        printf("DualAndorServer::getDataInBeamRateMode(): GetTotalNumberImagesAcquired() (hcam = %d): %s\n", (int) _hCamMaster, AndorErrorCodes::name(iError));
        break;
      }
    } else {
      printf("DualAndorServer::getDataInBeamRateMode(): GetTotalNumberImagesAcquired() (hcam = %d): failed to get cam handle\n", (int) _hCamMaster);
      break;
    }

    if (checkSlaveSelected()) {
      int iError = GetTotalNumberImagesAcquired(&numAcqNewSlave);
      if (!isAndorFuncOk(iError))
      {
        printf("DualAndorServer::getDataInBeamRateMode(): GetTotalNumberImagesAcquired() (hcam = %d): %s\n", (int) _hCamSlave, AndorErrorCodes::name(iError));
        break;
      }
    } else {
      printf("DualAndorServer::getDataInBeamRateMode(): GetTotalNumberImagesAcquired() (hcam = %d): failed to get cam handle\n", (int) _hCamSlave);
      break;
    }

    //if (numAcqNew != numAcq)
    //  printf("Num Image Acquired: %d\n", (int) numAcqNew);

    timeval timeSleepMicro = {0, iWaitPerIter * 1000}; // in milliseconds
    // use select() to simulate nanosleep(), because experimentally select() controls the sleeping time more precisely
    select( 0, NULL, NULL, NULL, &timeSleepMicro);

    iWaitTime += iWaitPerIter;
  }

  bool bFrameError = false;
  if (numAcqNewMaster == _iNumAcq || numAcqNewSlave == _iNumAcq)
  {
    if (!bPrevCatpureFailed)
      printf("DualAndorServer::getDataInBeamRateMode(): Wait image data %d failed\n", _iNumAcq);
    bFrameError = true;
    bPrevCatpureFailed  = true;//!!!debug
  }
  else
  {
    ++_iNumAcq;
    bPrevCatpureFailed  = false;//!!!debug

    uint8_t* pImageSlave  = (uint8_t*) out + _iFrameHeaderSize + _iDetectorSensor*sizeof(float);
    uint8_t* pImageMaster = pImageSlave + _iImageWidth*_iImageHeight*sizeof(uint16_t);
    if (checkMasterSelected()) {
      int iError = getOldestFrameWithRetry((uint16_t*)pImageMaster, _iImageWidth*_iImageHeight);
      if (!isAndorFuncOk(iError))
      {
        printf("DualAndorServer::getDataInBeamRateMode(): GetOldestImage16(): %s\n", AndorErrorCodes::name(iError));
        bFrameError = true;
      }
    } else {
      printf("DualAndorServer::getDataInBeamRateMode(): GetOldestImage16(): failed to get cam handle\n");
      bFrameError = true;
    }
    if (checkSlaveSelected()) {
      int iError = getOldestFrameWithRetry((uint16_t*)pImageSlave, _iImageWidth*_iImageHeight);
      if (!isAndorFuncOk(iError))
      {
        printf("DualAndorServer::getDataInBeamRateMode(): GetOldestImage16(): %s\n", AndorErrorCodes::name(iError));
        bFrameError = true;
      }
    } else {
      printf("DualAndorServer::getDataInBeamRateMode(): GetOldestImage16(): failed to get cam handle\n");
      bFrameError = true;
    }
  }

  timespec tsWaitEnd;
  clock_gettime( CLOCK_REALTIME, &tsWaitEnd );
  _fReadoutTime = (tsWaitEnd.tv_nsec - tsWaitStart.tv_nsec) / 1.0e9 + ( tsWaitEnd.tv_sec - tsWaitStart.tv_sec ); // in seconds

  ////!!!debug
  if (vecReadoutTimeMaster.size() < 20)
    vecReadoutTimeMaster.push_back(make_pair(_fReadoutTime, numAcqNewMaster));
  if (vecReadoutTimeSlave.size() < 20)
    vecReadoutTimeSlave.push_back(make_pair(_fReadoutTime, numAcqNewSlave));
  static int iSlowThreshold = 1;
  if ( _fReadoutTime > 1.0 )
    printf("  *** get %d / %d master cam image out, time = %f sec\n", _iNumAcq, (int) numAcqNewMaster, _fReadoutTime);
    printf("  *** get %d / %d slave cam  image out, time = %f sec\n", _iNumAcq, (int) numAcqNewSlave, _fReadoutTime);
  if (numAcqNewMaster > _iNumAcq+iSlowThreshold || numAcqNewSlave > _iNumAcq+iSlowThreshold) {
    printf("  ^^^ get %d / %d master cam image out, time = %f sec\n", _iNumAcq, (int) numAcqNewMaster, _fReadoutTime);
    printf("  ^^^ get %d / %d slave cam  image out, time = %f sec\n", _iNumAcq, (int) numAcqNewSlave, _fReadoutTime);
    iSlowThreshold <<= 1;
  }

  /*
   * Set frame object
   */
  unsigned char* pcXtcFrame = (unsigned char*) out + sizeof(CDatagram);
  Xtc* pXtcFrame            = new ((char*)pcXtcFrame) Xtc(_andor3dDataType, _src);
  pXtcFrame->alloc( _config.frameSize() );


  Datagram& dgIn  = in->datagram();
  Datagram& dgOut = out->datagram();

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

  unsigned char*  pFrameHeader  = (unsigned char*) out + sizeof(CDatagram) + sizeof(Xtc);
  new (pFrameHeader) Andor3dDataType(dgIn.seq.stamp().fiducials(), _fReadoutTime);

  if (bFrameError)
    // set damage bit, and still keep the image data
    dgOut.xtc.damage.increase(Pds::Damage::UserDefined);

  return 0;
}

int DualAndorServer::waitForNewFrameAvailable()
{
  static timespec tsWaitStart;
  clock_gettime( CLOCK_REALTIME, &tsWaitStart );

  int iErrorMaster = DRV_SUCCESS;
  int iErrorSlave  = DRV_SUCCESS;
  if (checkMasterSelected())
  {
    iErrorMaster = WaitForAcquisitionTimeOut(_iReadoutWaitTime);
    
    if (_config.fanMode() == (int) Andor3dConfigType::ENUM_FAN_ACQOFF)
    {
      iErrorMaster = SetFanMode((int) Andor3dConfigType::ENUM_FAN_FULL);
      if (!isAndorFuncOk(iErrorMaster))
        printf("DualAndorServer::waitForNewFrameAvailable(): SetFanMode(%d): %s\n", (int) Andor3dConfigType::ENUM_FAN_FULL, AndorErrorCodes::name(iErrorMaster));
    }
  }
  if (checkSlaveSelected())
  {
    iErrorSlave  = WaitForAcquisitionTimeOut(_iReadoutWaitTime);

    if (_config.fanMode() == (int) Andor3dConfigType::ENUM_FAN_ACQOFF)
    {
      iErrorSlave = SetFanMode((int) Andor3dConfigType::ENUM_FAN_FULL);
      if (!isAndorFuncOk(iErrorSlave))
        printf("DualAndorServer::waitForNewFrameAvailable(): SetFanMode(%d): %s\n", (int) Andor3dConfigType::ENUM_FAN_FULL, AndorErrorCodes::name(iErrorSlave));
    }
  }

  if (!isAndorFuncOk(iErrorMaster) || !isAndorFuncOk(iErrorSlave))
  {
    if (!isAndorFuncOk(iErrorMaster))
      printf("DualAndorServer::waitForNewFrameAvailable(): WaitForAcquisitionTimeOut(): %s\n", AndorErrorCodes::name(iErrorMaster));
    if (!isAndorFuncOk(iErrorSlave))
      printf("DualAndorServer::waitForNewFrameAvailable(): WaitForAcquisitionTimeOut(): %s\n", AndorErrorCodes::name(iErrorSlave));

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
  //    printf("DualAndorServer::waitForNewFrameAvailable(): GetStatus(): %s\n", AndorErrorCodes::name(status));
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
  //  if ( iWaitTime >= _iReadoutWaitTime )
  //  {
  //    printf( "DualAndorServer::waitForNewFrameAvailable(): Readout time is longer than %d miliseconds. Capture is stopped\n",
  //     _iReadoutWaitTime );
  //
  //    iError = AbortAcquisition();
  //    if (!isAndorFuncOk(iError) && iError != DRV_IDLE)
  //    {
  //      printf("DualAndorServer::waitForNewFrameAvailable(): AbortAcquisition(): %s\n", AndorErrorCodes::name(iError));
  //    }
  //
  //    // The  readout time (with incomplete data) will be reported in the framedata
  //    _fReadoutTime = (tsCurrent.tv_nsec - tsWaitStart.tv_nsec) / 1.0e9 + ( tsCurrent.tv_sec - tsWaitStart.tv_sec ); // in seconds
  //    return 1;
  //  }
  //} // while (true)

  uint8_t* pImageSlave = (uint8_t*) _pDgOut + _iFrameHeaderSize + _iDetectorSensor*sizeof(float);
  uint8_t* pImageMaster  = pImageSlave + _iImageWidth*_iImageHeight*sizeof(uint16_t);
  if (checkMasterSelected()) {
    iErrorMaster = getAcqFrameWithRetry((uint16_t*)pImageMaster, _iImageWidth*_iImageHeight);
    if (!isAndorFuncOk(iErrorMaster))
    {
      printf("GetAcquiredData16() (hcam = %d): %s\n", (int) _hCamMaster, AndorErrorCodes::name(iErrorMaster));
      return ERROR_SDK_FUNC_FAIL;
    }
  } else {
    printf("GetAcquiredData16() couldn't get camera handle: (hcam = %d)\n", (int) _hCamMaster);
    return ERROR_SDK_FUNC_FAIL;
  }
  if (checkSlaveSelected()) {
    iErrorSlave = getAcqFrameWithRetry((uint16_t*)pImageSlave, _iImageWidth*_iImageHeight);
    if (!isAndorFuncOk(iErrorSlave))
    {
      printf("GetAcquiredData16() (hcam = %d): %s\n", (int) _hCamSlave, AndorErrorCodes::name(iErrorSlave));
      return ERROR_SDK_FUNC_FAIL;
    }
  } else {
    printf("GetAcquiredData16() couldn't get camera handle: (hcam = %d)\n", (int) _hCamSlave);
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

int DualAndorServer::processFrame()
{
  if ( _pDgOut == NULL )
  {
    printf( "DualAndorServer::startCapture(): Datagram has not been allocated. No buffer to store the image data\n" );
    return ERROR_LOGICAL_FAILURE;
  }

  if ( _iNumExposure <= _iMaxEventReport ||  _iDebugLevel >= 5 )
  {
    unsigned char*  pFrameHeader    = (unsigned char*) _pDgOut + sizeof(CDatagram) + sizeof(Xtc);
    Andor3dDataType* pFrame           = (Andor3dDataType*) pFrameHeader;
    const uint16_t*     pPixel      = pFrame->data(_config).data();
    //int                 iWidth   = (int) ( (_config.width()  + _config.binX() - 1 ) / _config.binX() );
    //int                 iHeight  = (int) ( (_config.height() + _config.binY() - 1 ) / _config.binY() );
    const uint16_t*     pEnd        = (const uint16_t*) ( (unsigned char*) pFrame->data(_config).data() + _config.frameSize() );
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

int DualAndorServer::setupFrame()
{
  if ( _poolFrameData.numberOfFreeObjects() <= 0 )
  {
    printf( "DualAndorServer::setupFrame(): Pool is full, and cannot provide buffer for new datagram\n" );
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
    printf( "DualAndorServer::setupFrame(): pool status: %d/%d allocated, datagram: %p\n",
     _poolFrameData.numberOfAllocatedObjects(), _poolFrameData.numberofObjects(), _pDgOut  );
  }

  /*
   * Set frame object
   */
  unsigned char* pcXtcFrame = (unsigned char*) _pDgOut + sizeof(CDatagram);

  Xtc* pXtcFrame =
   new ((char*)pcXtcFrame) Xtc(_andor3dDataType, _src);
  pXtcFrame->alloc( iFrameSize );

  return 0;
}

int DualAndorServer::resetFrameData(bool bDelOutDatagram)
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

int DualAndorServer::setupROI()
{
  printf("ROI (%d,%d) W %d/%d H %d/%d ", _config.orgX(), _config.orgY(),
    _config.width(), _config.binX(), _config.height(), _config.binY());

  _iImageWidth  = _config.width()  / _config.binX();
  _iImageHeight = _config.height() / _config.binY();
  printf("Image size: W %d H %d\n", _iImageWidth, _iImageHeight);

  /*
   * Read Mode:
   *   0: Full Vertical Binning 1: MultiTrack 2: Random Track
   *   3: Single Track 4: Image
   */
  int iReadMode = 4;
  if ( _iImageWidth == _iDetectorWidth && _iImageHeight == 1)
  {
    if (_config.orgY() == 0 && _config.height() == (unsigned) _iDetectorHeight)
      iReadMode = 0;
    else
      iReadMode = 3;
  }

  printf("DualAndorServer::setupROI(): Configuring ROI of slave (hcam = %d) for readmode: %d\n", (int) _hCamSlave, iReadMode);
  if(checkSlaveSelected())
  {
    int iError = configureROI(iReadMode);
    if (iError != 0)
      return iError;
  }
  else
    return ERROR_SDK_FUNC_FAIL;

  printf("DualAndorServer::setupROI(): Configuring ROI of master (hcam = %d) for readmode: %d\n", (int) _hCamMaster, iReadMode);
  if(checkMasterSelected())
  {
    int iError = configureROI(iReadMode);
    if (iError != 0)
      return iError;
  }
  else
    return ERROR_SDK_FUNC_FAIL;

  return 0;
}

int DualAndorServer::configureROI(int iReadMode)
{
  SetReadMode(iReadMode);
  printf("Read mode: %d\n", iReadMode);

  if (iReadMode == 4)
  {
    int iError;
    iError = SetImage(_config.binX(), _config.binY(), _config.orgX() + 1, _config.orgX() + _config.width(),
      _config.orgY() + 1, _config.orgY() + _config.height());
    if (!isAndorFuncOk(iError))
    {
      printf("DualAndorServer::setupROI(): SetImage(): %s\n", AndorErrorCodes::name(iError));
      return ERROR_SDK_FUNC_FAIL;
    }
  }
  else if (iReadMode == 3) // single track
  {
    //Setup Image dimensions
    printf("Setting Single Track center %d height %d\n", _config.orgY() + _config.height()/2, _config.height());
    int  iError = SetSingleTrack( 1 + _config.orgY() + _config.height()/2, _config.height());
    if (!isAndorFuncOk(iError))
    {
      printf("DualAndorServer::setupROI():SetSingleTrack(): %s\n", AndorErrorCodes::name(iError));
      return ERROR_SDK_FUNC_FAIL;
    }
  }

  return 0;
}

bool DualAndorServer::checkMasterSelected()
{
  at_32 hCam;
  GetCurrentCamera(&hCam);
  if (hCam == _hCamMaster)
    return true;
  else
    return isAndorFuncOk(SetCurrentCamera(_hCamMaster));
}

bool DualAndorServer::checkSlaveSelected()
{
  at_32 hCam;
  GetCurrentCamera(&hCam);
  if (hCam == _hCamSlave)
    return true;
  else
    return isAndorFuncOk(SetCurrentCamera(_hCamSlave));
}

int DualAndorServer::updateTemperatureData()
{
  if (checkMasterSelected())
    GetTemperature(&_iTemperatureMaster);
  if (checkSlaveSelected())
    GetTemperature(&_iTemperatureSlave);

  /*
   * Set Info object
   */
  printf( "Detector Temperature report [%d]: %d C and %d C\n", _iNumExposure, _iTemperatureMaster, _iTemperatureSlave );

  if ( _pDgOut == NULL )
  {
    printf( "DualAndorServer::updateTemperatureData(): Datagram has not been allocated. No buffer to store the info data\n" );
  }
  else
  {
    unsigned char*  pTemperatureHeader = (unsigned char*) _pDgOut + _iFrameHeaderSize;
    float* pTempData = (float*) pTemperatureHeader;
    pTempData[0] = _iTemperatureSlave;
    pTempData[1] = _iTemperatureMaster;
  }

  if (  _iTemperatureMaster >= _config.coolingTemp() + _fTemperatureHiTol ||
        _iTemperatureMaster <= _config.coolingTemp() - _fTemperatureLoTol ||
        _iTemperatureSlave  >= _config.coolingTemp() + _fTemperatureHiTol ||
        _iTemperatureSlave  <= _config.coolingTemp() - _fTemperatureLoTol)
  {
    printf( "** DualAndorServer::updateTemperatureData(): Detector temperatures (%d C and %d C) are not fixed to the configuration (%.1f C)\n",
      _iTemperatureMaster, _iTemperatureSlave, _config.coolingTemp() );
    return ERROR_TEMPERATURE;
  }

  return 0;
}

int DualAndorServer::getAcqFrameWithRetry(uint16_t* liImageData, int iImageSize)
{
  int iError = 0;
  int iAttempts = 0;
  while (iAttempts < _iMaxReadoutAttempts) {
    iError = GetAcquiredData16(liImageData, iImageSize);
    if (isAndorFuncOk(iError)) {
      break;
    }
    iAttempts++;
    timeval timeSleepMicro = {0, iAttempts * _iReadoutBackoffTime * 1000}; // in milliseconds
    // use select() to simulate nanosleep(), because experimentally select() controls the sleeping time more precisely
    select( 0, NULL, NULL, NULL, &timeSleepMicro);
  }

  return iError;
}

int DualAndorServer::getOldestFrameWithRetry(uint16_t* liImageData, int iImageSize)
{
  int iError = 0;
  int iAttempts = 0;
  while (iAttempts < _iMaxReadoutAttempts) {
    iError = GetOldestImage16(liImageData, iImageSize);
    if (isAndorFuncOk(iError)) {
      break;
    }
    iAttempts++;
    timeval timeSleepMicro = {0, iAttempts * _iReadoutBackoffTime * 1000}; // in milliseconds
    // use select() to simulate nanosleep(), because experimentally select() controls the sleeping time more precisely
    select( 0, NULL, NULL, NULL, &timeSleepMicro);
  }

  return iError;
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
const int       DualAndorServer::_iMaxCamera;
const int       DualAndorServer::_iMaxCoolingTime;
const int       DualAndorServer::_fTemperatureHiTol;
const int       DualAndorServer::_fTemperatureLoTol;
const int       DualAndorServer::_iFrameHeaderSize      = sizeof(CDatagram) + sizeof(Xtc) + sizeof(Andor3dDataType);
const int       DualAndorServer::_iMaxFrameDataSize     = _iFrameHeaderSize + _iMaxCamera*sizeof(float) + _iMaxCamera*2048*2048*sizeof(uint16_t);
const int       DualAndorServer::_iPoolDataCount;
const int       DualAndorServer::_iMaxReadoutTime;
const int       DualAndorServer::_iMaxThreadEndTime;
const int       DualAndorServer::_iMaxLastEventTime;
const int       DualAndorServer::_iMaxEventReport;
const int       DualAndorServer::_iReadoutBackoffTime;
const int       DualAndorServer::_iMaxReadoutAttempts;
const int       DualAndorServer::_iTestExposureStartDelay;
const float     DualAndorServer::_fEventDeltaTimeFactor = 1.01f;

/*
 * Definition of private static data
 */
pthread_mutex_t DualAndorServer::_mutexPlFuncs = PTHREAD_MUTEX_INITIALIZER;


DualAndorServer::CaptureRoutine::CaptureRoutine(DualAndorServer& server) : _server(server)
{
}

void DualAndorServer::CaptureRoutine::routine(void)
{
  _server.runCaptureTask();
}

void DualAndorServer::setOccSend(DualAndorOccurrence* occSend)
{
  _occSend = occSend;
}

} //namespace Pds
