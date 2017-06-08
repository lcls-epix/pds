#include "Driver.hh"
#include "Features.hh"
#include "Errors.hh"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>

#define LENGTH_FIELD_SIZE 4
#define CID_FIELD_SIZE 4

#define set_config_bool(feature, value)                                     \
  if (!at_set_bool(feature, value)) {                                       \
    fprintf(stderr, "Failure setting %ls feature of the camera\n", feature);\
    return false; }

#define set_config_int(feature, value)                            \
  if (!at_set_int(feature, value)) {                              \
    fprintf(stderr,                                               \
            "Failure setting %ls feature of the camera to %lld\n",\
            feature, value);                                      \
    return false; }

#define set_config_float(feature, value)                        \
  if (!at_set_float(feature, value)) {                          \
    fprintf(stderr,                                             \
            "Failure setting %ls feature of the camera to %G\n",\
            feature, value);                                    \
    return false; }

#define set_enum_case(feature, enum_case, value)                            \
  case enum_case:                                                           \
    if (!at_set_enum(feature, value)) {                                     \
      fprintf(stderr, "Failure setting %ls feature of the camera to %ls\n", \
      feature, value);                                                      \
      return false;                                                         \
    }                                                                       \
    break;


namespace Pds {
  namespace Zyla {
    static const AT_WC* AT3_NOT_STABILISED = L"Not Stabilised";
    static const AT_WC* AT3_STABILISED = L"Stabilised";
    static const AT_WC* AT3_COOLER_OFF = L"Cooler Off";
    static const AT_WC* AT3_PIXEL_MONO_16 = L"Mono16";
  }
}

using namespace Pds::Zyla;


Driver::Driver(AT_H cam) :
  _cam(cam),
  _open(cam!=AT_HANDLE_UNINITIALISED),
  _buffer_size(0),
  _data_buffer(0),
  _stride(0),
  _width(0),
  _height(0)
{
}

Driver::~Driver()
{
  if (_open) AT_Close(_cam);
  if (_data_buffer) delete[] _data_buffer;
}

bool Driver::set_image(AT_64 width, AT_64 height, AT_64 orgX, AT_64 orgY, AT_64 binX, AT_64 binY, bool noise_filter, bool blemish_correction, bool fast_frame)
{
  // Setup the image size
  set_config_int(AT3_AOI_WIDTH, width);
  set_config_int(AT3_AOI_LEFT, orgX);
  set_config_int(AT3_AOI_H_BIN, binX);
  set_config_int(AT3_AOI_HEIGHT, height);
  set_config_int(AT3_AOI_TOP, orgY);
  set_config_int(AT3_AOI_V_BIN, binY);
  // Enable/Disable the on FPGA image corrections
  set_config_bool(AT3_NOISE_FILTER, noise_filter);
  set_config_bool(AT3_BLEMISH_CORRECTION, blemish_correction);
  set_config_bool(AT3_AOI_FAST_FRAME, fast_frame);
  // If we got here this is done! :)
  return true;
}

bool Driver::set_cooling(bool enable, ZylaConfigType::CoolingSetpoint setpoint, ZylaConfigType::FanSpeed fan_speed)
{
  // Enable cooling
  set_config_bool(AT3_SENSOR_COOLING, enable);
  // Set Cooling setpoint
  if (at_check_write(AT3_TEMPERATURE_CONTROL)) {
    switch(setpoint) {
      set_enum_case(AT3_TEMPERATURE_CONTROL, ZylaConfigType::Temp_0C,     L"0.00");
      set_enum_case(AT3_TEMPERATURE_CONTROL, ZylaConfigType::Temp_Neg5C,  L"-5.00");
      set_enum_case(AT3_TEMPERATURE_CONTROL, ZylaConfigType::Temp_Neg10C, L"-10.00");
      set_enum_case(AT3_TEMPERATURE_CONTROL, ZylaConfigType::Temp_Neg15C, L"-15.00");
      set_enum_case(AT3_TEMPERATURE_CONTROL, ZylaConfigType::Temp_Neg20C, L"-20.00");
      set_enum_case(AT3_TEMPERATURE_CONTROL, ZylaConfigType::Temp_Neg25C, L"-25.00");
      set_enum_case(AT3_TEMPERATURE_CONTROL, ZylaConfigType::Temp_Neg30C, L"-30.00");
      set_enum_case(AT3_TEMPERATURE_CONTROL, ZylaConfigType::Temp_Neg35C, L"-35.00");
      set_enum_case(AT3_TEMPERATURE_CONTROL, ZylaConfigType::Temp_Neg40C, L"-40.00");
      default:
        fprintf(stderr, "Unknown cooling setpoint value: %d\n", setpoint);
        return false;
    }
  }
  // Set Fan speed
  switch(fan_speed) {
    set_enum_case(AT3_FAN_SPEED, ZylaConfigType::Off, L"Off");
    set_enum_case(AT3_FAN_SPEED, ZylaConfigType::Low, L"Low");
    set_enum_case(AT3_FAN_SPEED, ZylaConfigType::On,  L"On");
    default:
      fprintf(stderr, "Unknown fan_speed value: %d\n", fan_speed);
      return false;
  }
  return true;
}

bool Driver::set_trigger(ZylaConfigType::TriggerMode trigger, double trigger_delay, double exposure_time, bool overlap)
{
  // Set the trigger mode
  switch(trigger) {
    set_enum_case(AT3_TRIGGER_MODE, ZylaConfigType::Internal,                 L"Internal");
    set_enum_case(AT3_TRIGGER_MODE, ZylaConfigType::ExternalLevelTransition,  L"External Level Transition");
    set_enum_case(AT3_TRIGGER_MODE, ZylaConfigType::ExternalStart,            L"External Start");
    set_enum_case(AT3_TRIGGER_MODE, ZylaConfigType::ExternalExposure,         L"External Exposure");
    set_enum_case(AT3_TRIGGER_MODE, ZylaConfigType::Software,                 L"Software");
    set_enum_case(AT3_TRIGGER_MODE, ZylaConfigType::Advanced,                 L"Advanced");
    set_enum_case(AT3_TRIGGER_MODE, ZylaConfigType::External,                 L"External");
    default:
      fprintf(stderr, "Unknown tigger setting value: %d\n", trigger);
      return false;
  }
  // Set readout overlap mode (if enabled next acquistion can start while camera is reading out last frame)
  if (at_check_write(AT3_OVERLAP)) {
    set_config_bool(AT3_OVERLAP, overlap);
  }
  // Set the internal trigger delay
  if (at_check_write(AT3_EXTERN_TRIGGER_DELAY)) {
    set_config_float(AT3_EXTERN_TRIGGER_DELAY, trigger_delay);
  }
  // Set the camera exposure time
  if (at_check_write(AT3_EXPOSURE_TIME)) {
    set_config_float(AT3_EXPOSURE_TIME, exposure_time);
  }

  return true;
}

bool Driver::set_readout(ZylaConfigType::ShutteringMode shutter, ZylaConfigType::ReadoutRate readout_rate, ZylaConfigType::GainMode gain)
{
  if (!at_set_enum(AT3_PIXEL_ENCODING, AT3_PIXEL_MONO_16)) {
    fprintf(stderr, "Unable to set the pixel encoding of the camera to %ls!\n", AT3_PIXEL_MONO_16);
    return false;
  }
  // Set the camera electronic shuttering mode
  switch(shutter) {
    set_enum_case(AT3_SHUTTERING_MODE, ZylaConfigType::Rolling, L"Rolling");
    set_enum_case(AT3_SHUTTERING_MODE, ZylaConfigType::Global,  L"Global");
    default:
      fprintf(stderr, "Unknown shutter mode value: %d\n", shutter);
      return false;
  }
  // Set the pixel readout rate
  switch(readout_rate) {
    set_enum_case(AT3_PIXEL_READOUT_RATE, ZylaConfigType::Rate280MHz, L"280 MHz");
    set_enum_case(AT3_PIXEL_READOUT_RATE, ZylaConfigType::Rate200MHz, L"200 MHz");
    set_enum_case(AT3_PIXEL_READOUT_RATE, ZylaConfigType::Rate100MHz, L"100 MHz");
    set_enum_case(AT3_PIXEL_READOUT_RATE, ZylaConfigType::Rate10MHz,  L"10 MHz");
    default:
      fprintf(stderr, "Unknown readout rate value: %d\n", readout_rate);
      return false;
  }
  // Set the gain mode
  switch(gain) {
    set_enum_case(AT3_PREAMP_GAIN_MODE, ZylaConfigType::HighWellCap12Bit,         L"12-bit (high well capacity)");
    set_enum_case(AT3_PREAMP_GAIN_MODE, ZylaConfigType::LowNoise12Bit,            L"12-bit (low noise)");
    set_enum_case(AT3_PREAMP_GAIN_MODE, ZylaConfigType::LowNoiseHighWellCap16Bit, L"16-bit (low noise & high well capacity)");
    default:
      fprintf(stderr, "Unknown gain mode value: %d\n", gain);
      return false;
  }

  return true;
}

bool Driver::configure(const AT_64 nframes)
{
  AT_64 img_size_bytes;
  int old_buffer_size = _buffer_size;

  // set metadata settings
  set_config_bool(AT3_METADATA_ENABLE, true);
  set_config_bool(AT3_METADATA_TIMESTAMP, true);

  if (nframes > 0) {
    // set camera to acquire requested number of frames
    set_config_int(AT3_FRAME_COUNT, nframes);
    if (!at_set_enum(AT3_CYCLE_MODE, L"Fixed")) {
      fprintf(stderr, "Unable to set %ls to %ls", AT3_CYCLE_MODE, L"Fixed");
      return false;
    }
  } else {
    if (!at_set_enum(AT3_CYCLE_MODE, L"Continuous")) {
      fprintf(stderr, "Unable to set %ls to %ls", AT3_CYCLE_MODE, L"Continuous");
      return false;
    }
  }

  // Figure out if the camera is going to use weird padding at the end of rows. Thanks Andor....
  if (AT_GetInt(_cam, AT3_AOI_STRIDE, &_stride) != AT_SUCCESS) {
    fprintf(stderr, "Failure reading back %ls from the camera!\n", AT3_AOI_STRIDE);
  }
  // Figure out the height and width of the camera image we will get
  if (AT_GetInt(_cam, AT3_AOI_WIDTH, &_width) != AT_SUCCESS) {
    fprintf(stderr, "Failure reading back %ls from the camera!\n", AT3_AOI_WIDTH);
  }
  if (AT_GetInt(_cam, AT3_AOI_HEIGHT, &_height) != AT_SUCCESS) {
    fprintf(stderr, "Failure reading back %ls from the camera!\n", AT3_AOI_HEIGHT);
  }
  // Figure out the size of the total data the camera will send back - frame + metadata
  if (AT_GetInt(_cam, AT3_IMAGE_SIZE_BYTES, &img_size_bytes) == AT_SUCCESS) {
    _buffer_size = static_cast<int>(img_size_bytes);
    if(_data_buffer) {
      if (_buffer_size > old_buffer_size) {
        delete[] _data_buffer;
        _data_buffer = new unsigned char[_buffer_size];
      }
    } else {
      _data_buffer = new unsigned char[_buffer_size];
    }
    return true;
  } else {
    fprintf(stderr, "Failed to retrieve image buffer size from camera!\n");
    return false;
  }
}

bool Driver::start()
{
  if (AT_QueueBuffer(_cam, _data_buffer, _buffer_size) != AT_SUCCESS) {
    fprintf(stderr, "Failed adding image buffer to queue! - abort acquistion start\n");
    return false;
  }
  return (AT_Command(_cam, AT3_ACQUISITION_START) == AT_SUCCESS);
}

bool Driver::stop()
{
  if (AT_Command(_cam, AT3_ACQUISITION_STOP) != AT_SUCCESS) {
    fprintf(stderr, "Stop acquistion command failed!\n");
  }
  return (AT_Flush(_cam) == AT_SUCCESS);
}

bool Driver::close()
{
  if (_open) {
    _open = false;
    int retcode = AT_Close(_cam);
    _cam = AT_HANDLE_UNINITIALISED;
    return (retcode == AT_SUCCESS);
  } else {
    return true;
  }
}

bool Driver::is_present() const
{
  AT_BOOL camera_pres;
  AT_GetBool(_cam, AT3_CAMERA_PRESENT, &camera_pres);
  return (camera_pres == AT_TRUE);
}

size_t Driver::frame_size() const
{
  return (size_t) _width * _height;
}

bool Driver::get_frame(AT_64& timestamp, uint16_t* data)
{
  int retcode;
  AT_64 width;
  AT_64 height;
  AT_64 stride;
  unsigned char* buffer;

  retcode = AT_WaitBuffer(_cam, &buffer, &_buffer_size, AT_INFINITE);
  if (retcode == AT_SUCCESS) {
    bool success = true;

    if (AT_GetTimeStampFromMetadata(buffer, _buffer_size, timestamp) != AT_SUCCESS) {
      fprintf(stderr, "Failure retrieving timestamp from frame metadata\n");
      success = false;
    }
    if (AT_GetWidthFromMetadata(buffer, _buffer_size, width) != AT_SUCCESS) {
      fprintf(stderr, "Failure retrieving timestamp from frame metadata\n");
      success = false;
    }
    if (AT_GetHeightFromMetadata(buffer, _buffer_size, height) != AT_SUCCESS) {
      fprintf(stderr, "Failure retrieving timestamp from frame metadata\n");
      success = false;
    }
    if (AT_GetStrideFromMetadata(buffer, _buffer_size, stride) != AT_SUCCESS) {
      fprintf(stderr, "Failure retrieving timestamp from frame metadata\n");
      success = false;
    }
    // Check if the metadata matches with the expected frame size
    if ((width != _width) || (height != _height) || (stride != _stride)) {
      fprintf(stderr,
              "Unexpected frame size returned by camera: width (%lld vs %lld), height (%lld vs %lld), stride (%lld vs %lld)\n",
              width,
              _width,
              height,
              _height,
              stride,
              _stride);
      success = false;
    }
  
    // If the metadata looks good convert the buffer to a usable image an return it
    if (success) {
      retcode = AT_ConvertBufferUsingMetadata(buffer, reinterpret_cast<unsigned char*>(data), _buffer_size, AT3_PIXEL_MONO_16);
      if (retcode != AT_SUCCESS) {
        fprintf(stderr, "Failure converting data buffer to image using metadata: %s\n", ErrorCodes::name(retcode));
        success = false;
      }
    }

    // Reuse the buffer for the next frame
    AT_QueueBuffer(_cam, _data_buffer, _buffer_size);

    return success;
  } else {
    // Acquistion failed flush the buffer before re-adding it to the queue
    if (retcode == AT_ERR_NODATA) {
      printf("Aquistion ended - stopping buffer wait\n");
    } else {
      fprintf(stderr, "Failure waiting for buffer callback from camera: %s\n", ErrorCodes::name(retcode));
    }
    return false;
  }
}

AT_64 Driver::sensor_width() const
{
  return at_get_int(AT3_SENSOR_WIDTH);
}

AT_64 Driver::sensor_height() const
{
  return at_get_int(AT3_SENSOR_HEIGHT);
}

AT_64 Driver::baseline() const {
  return at_get_int(AT3_BASELINE);
}

AT_64 Driver::clock() const {
  return at_get_int(AT3_TIMESTAMP_CLOCK);
}

AT_64 Driver::clock_rate() const {
  return at_get_int(AT3_TIMESTAMP_FREQUENCY);
}

AT_64 Driver::image_stride() const {
  return at_get_int(AT3_AOI_STRIDE);
}

AT_64 Driver::image_width() const {
  return at_get_int(AT3_AOI_WIDTH);
}

AT_64 Driver::image_height() const {
  return at_get_int(AT3_AOI_HEIGHT);
}

double Driver::readout_time() const
{
  return at_get_float(AT3_READOUT_TIME);
}

double Driver::frame_rate() const
{
  return at_get_float(AT3_FRAME_RATE);
}
      
double Driver::pixel_height() const
{
  return at_get_float(AT3_PIXEL_HEIGHT);
}

double Driver::pixel_width() const
{
  return at_get_float(AT3_PIXEL_WIDTH);
}

double Driver::temperature() const
{
  return at_get_float(AT3_SENSOR_TEMPERATURE);
}

double Driver::exposure() const
{
  return at_get_float(AT3_EXPOSURE_TIME);
}

bool Driver::cooling_on() const
{
  AT_BOOL cooling_on;
  AT_GetBool(_cam, AT3_SENSOR_COOLING, &cooling_on);
  return (cooling_on == AT_TRUE);
}

bool Driver::check_cooling(bool is_stable) const
{
  int retcode;
  int temp_idx;
  AT_WC temp_buf[128];
  
  retcode = AT_GetEnumIndex(_cam, AT3_TEMPERATURE_STATUS, &temp_idx);
  if (retcode != AT_SUCCESS) {
    fprintf(stderr, "Failed to retrieve %ls from camera!\n", AT3_TEMPERATURE_STATUS);
    return false;
  }

  retcode = AT_GetEnumStringByIndex(_cam, AT3_TEMPERATURE_STATUS, temp_idx, temp_buf, 128);
  if (retcode != AT_SUCCESS) {
    fprintf(stderr, "Failed to retrieve %ls from camera!\n", AT3_TEMPERATURE_STATUS);
    return false;
  }

  if (wcscmp(AT3_STABILISED, temp_buf) == 0) {
    return true;
  } else if(!is_stable && (wcscmp(AT3_NOT_STABILISED, temp_buf) == 0)) {
    return true;
  } else {
    return false;
  }
}

bool Driver::wait_cooling(unsigned timeout, bool is_stable) const
{
  int retcode;
  bool ready = false;
  int temp_idx;
  double temp;
  int count = 0;
  time_t start = time(0);
  AT_WC temp_buf[128];

  while((time(0) - start < timeout) || (timeout == 0)) {
    retcode = AT_GetEnumIndex(_cam, AT3_TEMPERATURE_STATUS, &temp_idx);
    if (retcode != AT_SUCCESS) {
      fprintf(stderr, "Failed to retrieve %ls from camera!\n", AT3_TEMPERATURE_STATUS);
      break;
    }
    retcode = AT_GetEnumStringByIndex(_cam, AT3_TEMPERATURE_STATUS, temp_idx, temp_buf, 128);
    if (retcode != AT_SUCCESS) {
      fprintf(stderr, "Failed to retrieve %ls from camera!\n", AT3_TEMPERATURE_STATUS);
      break;
    }
    retcode = AT_GetFloat(_cam, AT3_SENSOR_TEMPERATURE, &temp);
    if (retcode != AT_SUCCESS) {
      fprintf(stderr, "Failed to retrieve %ls from camera!\n", AT3_SENSOR_TEMPERATURE);
      break;
    }
    if (count % 5 == 0) 
      printf("Temperature status: %ls (%G C)\n", temp_buf, temp);
    if(wcscmp(AT3_STABILISED, temp_buf) == 0 || wcscmp(AT3_COOLER_OFF, temp_buf) == 0) {
      ready = true;
      break;
    }
    if(!is_stable && wcscmp(AT3_NOT_STABILISED, temp_buf) == 0) {
      ready = true;
      break;
    }
    count++;
    sleep(1);
  }

  return ready;
}

bool Driver::get_cooling_status(AT_WC* buffer, int buffer_size) const
{
  int retcode;
  int temp_idx;

  retcode = AT_GetEnumIndex(_cam, AT3_TEMPERATURE_STATUS, &temp_idx);
  if (retcode == AT_SUCCESS) {
    retcode = AT_GetEnumStringByIndex(_cam, AT3_TEMPERATURE_STATUS, temp_idx, buffer, buffer_size);
  }
  
  return retcode;
} 

bool Driver::get_name(AT_WC* buffer, int buffer_size) const
{
  return at_get_string(AT3_CAMERA_NAME, buffer, buffer_size);
}

bool Driver::get_model(AT_WC* buffer, int buffer_size) const
{
  return at_get_string(AT3_CAMERA_MODEL, buffer, buffer_size);
}

bool Driver::get_family(AT_WC* buffer, int buffer_size) const
{
  return at_get_string(AT3_CAMERA_FAMILY, buffer, buffer_size);
}

bool Driver::get_serial(AT_WC* buffer, int buffer_size) const
{
  return at_get_string(AT3_SERIAL_NUMBER, buffer, buffer_size);
}

bool Driver::get_firmware(AT_WC* buffer, int buffer_size) const
{
  return at_get_string(AT3_FIRMWARE_VERSION, buffer, buffer_size);
}

bool Driver::get_interface_type(AT_WC* buffer, int buffer_size) const
{
  return at_get_string(AT3_INTERFACE_TYPE, buffer, buffer_size);
}

bool Driver::get_sdk_version(AT_WC* buffer, int buffer_size) const
{
  return at_get_string(AT3_SOFTWARE_VERSION, buffer, buffer_size);
}

bool Driver::at_check_write(const AT_WC* feature) const
{
  AT_BOOL writeable;
  if (AT_IsWritable(_cam, feature, &writeable) != AT_SUCCESS) {
    fprintf(stderr, "Unable to get the write status of %ls from the camera\n", feature);
    return false;
  }
  return writeable;
}

bool Driver::at_get_string(const AT_WC* feature, AT_WC* buffer, int buffer_size) const
{
  return (AT_GetString(_cam, feature, buffer, buffer_size) == AT_SUCCESS);
}

AT_64 Driver::at_get_int(const AT_WC* feature) const
{
  AT_64 value = 0;
  if (AT_GetInt(_cam, feature, &value) != AT_SUCCESS)
    fprintf(stderr, "Failed to retrieve %ls from camera!\n", feature);
  return value;
}

double Driver::at_get_float(const AT_WC* feature) const
{
  double value = NAN;
  if (AT_GetFloat(_cam, feature, &value) != AT_SUCCESS)
    fprintf(stderr, "Failed to retrieve %ls from camera!\n", feature);
  return value;
}

bool Driver::at_set_int(const AT_WC* feature, const AT_64 value)
{
  AT_64 limit;
  if ((AT_GetIntMin(_cam, feature, &limit) != AT_SUCCESS) || (value < limit)) {
    printf("Unable to set %ls to %lld: value is below lower range of %lld\n", feature, value, limit);
    return false;
  }
  if ((AT_GetIntMax(_cam, feature, &limit) != AT_SUCCESS) || (value > limit)) {
    printf("Unable to set %ls to %lld: value is above upper range of %lld\n", feature, value, limit);
    return false;
  }

  int retcode = AT_SetInt(_cam, feature, value);
  if (retcode != AT_SUCCESS)
    fprintf(stderr, "Failed to set integer feature %ls: %s\n", feature, ErrorCodes::name(retcode));

  return (retcode == AT_SUCCESS);
}

bool Driver::at_set_float(const AT_WC* feature, const double value)
{
  double limit;
  if ((AT_GetFloatMin(_cam, feature, &limit) != AT_SUCCESS) || (value < limit)) {
    printf("Unable to set %ls to %G: value is below lower range of %G\n", feature, value, limit);
    return false;
  }
  if ((AT_GetFloatMax(_cam, feature, &limit) != AT_SUCCESS) || (value > limit)) {
    printf("Unable to set %ls to %G: value is above upper range of %G\n", feature, value, limit);
    return false;
  }

  int retcode = AT_SetFloat(_cam, feature, value);
  if (retcode != AT_SUCCESS)
    fprintf(stderr, "Failed to set float feature %ls: %s\n", feature, ErrorCodes::name(retcode));

  return (retcode == AT_SUCCESS);
}

bool Driver::at_set_enum(const AT_WC* feature, const AT_WC* value)
{
  int retcode = AT_SetEnumString(_cam, feature, value);

  if (retcode != AT_SUCCESS)
    fprintf(stderr, "Failed to set enum feature %ls: %s\n", feature, ErrorCodes::name(retcode));

  return (retcode == AT_SUCCESS);
}

bool Driver::at_set_bool(const AT_WC* feature, bool value)
{
  int retcode;
  if (value) retcode = AT_SetBool(_cam, feature, AT_TRUE);
  else retcode = AT_SetBool(_cam, feature, AT_FALSE);

  if (retcode != AT_SUCCESS)
    fprintf(stderr, "Failed to set boolean feature %ls: %s\n", feature, ErrorCodes::name(retcode));

  return (retcode == AT_SUCCESS);
}

#undef LENGTH_FIELD_SIZE
#undef CID_FIELD_SIZE
#undef set_config_bool
#undef set_config_int
#undef set_config_float
#undef set_enum_case
