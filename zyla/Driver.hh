#ifndef Pds_Zyla_Driver_hh
#define Pds_Zyla_Driver_hh

#include "pds/config/ZylaConfigType.hh"
#include "andor3/include/atutility.h"

#include <stdint.h>

namespace Pds {
  namespace Zyla {
    class Driver {
      public:
        Driver(AT_H cam);
        ~Driver();
      public:
        bool set_image(AT_64 width, AT_64 height, AT_64 orgX, AT_64 orgY, AT_64 binX, AT_64 binY, bool noise_filter, bool blemish_correction, bool fast_frame=true);
        bool set_cooling(bool enable, ZylaConfigType::CoolingSetpoint setpoint, ZylaConfigType::FanSpeed fan_speed);
        bool set_trigger(ZylaConfigType::TriggerMode trigger, double trigger_delay, double exposure_time, bool overlap);
        bool set_readout(ZylaConfigType::ShutteringMode shutter, ZylaConfigType::ReadoutRate readout_rate, ZylaConfigType::GainMode gain);
        bool configure(const AT_64 nframes=0);
        bool start();
        bool stop(bool flush_buffers=true);
        bool flush();
        bool close();
        bool is_present() const;
        size_t frame_size() const;
        bool get_frame(AT_64& timestamp, uint16_t* data);
      public:
        AT_64 sensor_width() const;
        AT_64 sensor_height() const;
        AT_64 baseline() const;
        AT_64 clock() const;
        AT_64 clock_rate() const;
        AT_64 image_stride() const;
        AT_64 image_width() const;
        AT_64 image_height() const;
        AT_64 image_orgX() const;
        AT_64 image_orgY() const;
        AT_64 image_binX() const;
        AT_64 image_binY() const;
        double readout_time() const;
        double frame_rate() const;
        double pixel_height() const;
        double pixel_width() const;
        double temperature() const;
        double exposure() const;
        bool overlap_mode() const;
        bool cooling_on() const;
        bool check_cooling(bool is_stable=true) const;
        bool wait_cooling(unsigned timeout, bool is_stable=true) const;

        bool get_cooling_status(AT_WC* buffer, int buffer_size) const;
        bool get_shutter_mode(AT_WC* buffer, int buffer_size) const;
        bool get_trigger_mode(AT_WC* buffer, int buffer_size) const;
        bool get_gain_mode(AT_WC* buffer, int buffer_size) const;
        bool get_readout_rate(AT_WC* buffer, int buffer_size) const;
        bool get_name(AT_WC* buffer, int buffer_size) const;
        bool get_model(AT_WC* buffer, int buffer_size) const;
        bool get_family(AT_WC* buffer, int buffer_size) const;
        bool get_serial(AT_WC* buffer, int buffer_size) const;
        bool get_firmware(AT_WC* buffer, int buffer_size) const;
        bool get_interface_type(AT_WC* buffer, int buffer_size) const;
        bool get_sdk_version(AT_WC* buffer, int buffer_size) const;
      private:
        bool at_check_write(const AT_WC* feature) const;
        bool at_get_string(const AT_WC* feature, AT_WC* buffer, int buffer_size) const;
        AT_64 at_get_int(const AT_WC* feature) const;
        double at_get_float(const AT_WC* feature) const;
        bool at_get_enum(const AT_WC* feature, AT_WC* buffer, int buffer_size) const;
        bool at_set_int(const AT_WC* feature, const AT_64 value);
        bool at_set_float(const AT_WC* feature, const double value);
        bool at_set_enum(const AT_WC* feature, const AT_WC* value);
        bool at_set_bool(const AT_WC* feature, bool value);
      private:
        AT_H            _cam;
        bool            _open;
        bool            _queued;
        int             _buffer_size;
        unsigned char*  _data_buffer;
        AT_64           _stride;
        AT_64           _width;
        AT_64           _height;
    };
  }
}

#endif
