#ifndef Pds_Zyla_Features_hh
#define Pds_Zyla_Features_hh

#include "andor3/include/atcore.h"

namespace Pds {
  namespace Zyla {
    static const AT_WC* AT3_AOI_H_BIN = L"AOIHBin";
    static const AT_WC* AT3_AOI_HEIGHT = L"AOIHeight";
    static const AT_WC* AT3_AOI_LEFT = L"AOILeft";
    static const AT_WC* AT3_AOI_STRIDE = L"AOIStride";
    static const AT_WC* AT3_AOI_TOP = L"AOITop";
    static const AT_WC* AT3_AOI_V_BIN = L"AOIVBin";
    static const AT_WC* AT3_AOI_WIDTH = L"AOIWidth";
    static const AT_WC* AT3_AOI_FAST_FRAME = L"FastAOIFrameRateEnable";
    static const AT_WC* AT3_SENSOR_COOLING = L"SensorCooling";
    static const AT_WC* AT3_TEMPERATURE_CONTROL = L"TemperatureControl";
    static const AT_WC* AT3_TEMPERATURE_STATUS = L"TemperatureStatus";
    static const AT_WC* AT3_SENSOR_TEMPERATURE = L"SensorTemperature";
    static const AT_WC* AT3_FAN_SPEED = L"FanSpeed";
    static const AT_WC* AT3_ACQUISITION_START = L"AcquisitionStart";
    static const AT_WC* AT3_ACQUISITION_STOP = L"AcquisitionStop";
    static const AT_WC* AT3_TRIGGER_MODE = L"TriggerMode";
    static const AT_WC* AT3_EXTERN_TRIGGER_DELAY = L"ExternalTriggerDelay";
    static const AT_WC* AT3_EXPOSURE_TIME = L"ExposureTime";
    static const AT_WC* AT3_OVERLAP = L"Overlap";
    static const AT_WC* AT3_PIXEL_READOUT_RATE = L"PixelReadoutRate";
    static const AT_WC* AT3_READOUT_TIME = L"ReadoutTime";
    static const AT_WC* AT3_SHUTTERING_MODE = L"ElectronicShutteringMode";
    static const AT_WC* AT3_PREAMP_GAIN_MODE = L"SimplePreAmpGainControl";
    static const AT_WC* AT3_PIXEL_ENCODING = L"PixelEncoding";
    static const AT_WC* AT3_NOISE_FILTER = L"SpuriousNoiseFilter";
    static const AT_WC* AT3_BLEMISH_CORRECTION = L"StaticBlemishCorrection";
    static const AT_WC* AT3_IMAGE_SIZE_BYTES = L"ImageSizeBytes";
    static const AT_WC* AT3_FRAME_COUNT = L"FrameCount";
    static const AT_WC* AT3_FRAME_RATE = L"FrameRate";
    static const AT_WC* AT3_CYCLE_MODE = L"CycleMode";
    static const AT_WC* AT3_SENSOR_HEIGHT  = L"SensorHeight";
    static const AT_WC* AT3_SENSOR_WIDTH = L"SensorWidth";
    static const AT_WC* AT3_BASELINE = L"Baseline";
    static const AT_WC* AT3_PIXEL_HEIGHT = L"PixelHeight";
    static const AT_WC* AT3_PIXEL_WIDTH = L"PixelWidth";
    static const AT_WC* AT3_CAMERA_MODEL = L"CameraModel";
    static const AT_WC* AT3_CAMERA_NAME = L"CameraName";
    static const AT_WC* AT3_CAMERA_FAMILY = L"CameraFamily";
    static const AT_WC* AT3_FIRMWARE_VERSION = L"FirmwareVersion";
    static const AT_WC* AT3_INTERFACE_TYPE = L"InterfaceType";
    static const AT_WC* AT3_SERIAL_NUMBER = L"SerialNumber";
    static const AT_WC* AT3_SOFTWARE_VERSION = L"SoftwareVersion";
    static const AT_WC* AT3_CAMERA_PRESENT = L"CameraPresent";
    static const AT_WC* AT3_METADATA_ENABLE = L"MetadataEnable";
    static const AT_WC* AT3_METADATA_TIMESTAMP = L"MetadataTimestamp";
    static const AT_WC* AT3_TIMESTAMP_CLOCK = L"TimestampClock";
    static const AT_WC* AT3_TIMESTAMP_FREQUENCY = L"TimestampClockFrequency";
  }
}

#endif
