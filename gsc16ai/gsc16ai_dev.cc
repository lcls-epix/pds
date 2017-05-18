#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>

#include "pdsdata/psddl/gsc16ai.ddl.h"
#include "gsc16ai_dev.hh"

using namespace Pds;

gsc16ai_dev::gsc16ai_dev(const char* devName) :
  _devName(devName),
  _fd(0),
  _isOpen(false) {
}

//
// open
//
// Returns zero on success, nonzero otherwise.
//
int gsc16ai_dev::open(void) {
  int rv = 1;
  if (!_isOpen) {
    // not already open...
    int tmpfd = ::open(_devName, O_RDWR);
    if (tmpfd >= 0) {
      // basic initialization
      int set;
      set = AI32SSC_QUERY_CHANNEL_QTY;
      if (ai32ssc_dsl_ioctl(tmpfd, AI32SSC_IOCTL_QUERY, &set)) {
        fprintf(stderr, "Error: device %s does not respond to AI32SSC_IOCTL_QUERY\n", _devName);
        (void)::close(tmpfd);
        return (1); // ERROR
      } else {
        _nChan = set;
      }
      // open() successful
      rv = 0;
      _isOpen = true;
      _fd = tmpfd;
      // initialize both hardware based settings and software based settings
      if (ai32ssc_dsl_ioctl(_fd, AI32SSC_IOCTL_INITIALIZE, NULL)) {
        fprintf(stderr, "Error: AI32SSC_IOCTL_INITIALIZE\n");
      }
      // use PIO mode
      set = GSC_IO_MODE_PIO;
      if (ai32ssc_dsl_ioctl(_fd, AI32SSC_IOCTL_RX_IO_MODE, &set)) {
        fprintf(stderr, "Error: AI32SSC_IOCTL_RX_IO_MODE\n");
      }
      // check for overflow during read
      set = AI32SSC_IO_OVERFLOW_CHECK;
      if (ai32ssc_dsl_ioctl(_fd, AI32SSC_IOCTL_RX_IO_OVERFLOW, &set)) {
        fprintf(stderr, "Error: AI32SSC_IOCTL_RX_IO_OVERFLOW\n");
      }
      // do not sleep in order to wait for more sample data
      set = 0;
      if (ai32ssc_dsl_ioctl(_fd, AI32SSC_IOCTL_RX_IO_TIMEOUT, &set)) {
        fprintf(stderr, "Error: AI32SSC_IOCTL_RX_IO_TIMEOUT\n");
      }
      // check for underflow during read
      set = AI32SSC_IO_UNDERFLOW_CHECK;
      if (ai32ssc_dsl_ioctl(_fd, AI32SSC_IOCTL_RX_IO_UNDERFLOW, &set)) {
        fprintf(stderr, "Error: AI32SSC_IOCTL_RX_IO_UNDERFLOW\n");
      }
      // ADC off
      if (unconfigure()) {
        fprintf(stderr, "Error: unconfigure() failed\n");
      }
    } else {
      perror("gsc16ai_dev::open");
    }
  } else {
    // already open...
    printf("%s: device already open\n", __PRETTY_FUNCTION__);
  }
  return (rv);
}

//
// close
//
// Returns zero on success, nonzero otherwise.
//
int gsc16ai_dev::close(void) {
  int rv = 1;
  if (_isOpen && (_fd > 0) && (::close(_fd)) == 0) {
    // close() successful
    rv = 0;
    _isOpen = false;
    _fd = 0;
  }
  return (rv);
}

//
// read
//
// Returns number of samples on success, -1 otherwise.
//
int gsc16ai_dev::read(uint32_t *buf, size_t samples) {
  size_t bytes = samples * 4;
  int status = ::read(_fd, buf, bytes);
  if (status == -1) {
    perror("gsc16ai_dev::read");
  } else if ((status > 0) && (status % 4)) {
    printf("gsc16ai_dev::read: unexpected read() return value: %d\n", status);
  }

  return ((status > 0) ? (status / 4) : status);
}

//
// configure
//
int gsc16ai_dev::configure(const Pds::Gsc16ai::ConfigV1& config) {
  int numErrs = 0;
  int set;

  if (!_fd || !_isOpen) {
    fprintf(stderr, "Error: gsc16ai device not open in %s\n", __FUNCTION__);
    return (1);
  }

  // Sanity check the config
  if (config.triggerMode() == Gsc16ai::ConfigV1::TriggerMode_IntClk) {
    if ((config.fps() < Gsc16ai::ConfigV1::LowestFps) ||
        (config.fps() > Gsc16ai::ConfigV1::HighestFps)) {
    fprintf(stderr, "Error: invalid fps value (%d) in %s\n", config.fps(), __FUNCTION__);
    return (1);
    }
  }

  // General settings
  set = 15; numErrs += ai32ssc_dsl_ioctl(_fd, AI32SSC_IOCTL_AIN_BUF_THR_LVL, &set); // num chan minus 1
  set = 1; numErrs += ai32ssc_dsl_ioctl(_fd, AI32SSC_IOCTL_BURST_SIZE, &set);
  set = AI32SSC_BURST_SYNC_DISABLE; numErrs += ai32ssc_dsl_ioctl(_fd, AI32SSC_IOCTL_BURST_SYNC, &set);
  set = AI32SSC_CHAN_ACTIVE_RANGE; numErrs += ai32ssc_dsl_ioctl(_fd, AI32SSC_IOCTL_CHAN_ACTIVE, &set);
  set = 0; numErrs += ai32ssc_dsl_ioctl(_fd, AI32SSC_IOCTL_CHAN_SINGLE, &set);
  set = AI32SSC_DATA_PACKING_DISABLE; numErrs += ai32ssc_dsl_ioctl(_fd, AI32SSC_IOCTL_DATA_PACKING, &set);
  set = AI32SSC_SCAN_MARKER_DISABLE; numErrs += ai32ssc_dsl_ioctl(_fd, AI32SSC_IOCTL_SCAN_MARKER, &set);
  set = AI32SSC_IRQ0_INIT_DONE; numErrs += ai32ssc_dsl_ioctl(_fd, AI32SSC_IOCTL_IRQ0_SEL, &set);
  set = AI32SSC_IRQ1_IN_BUF_THR_L2H; numErrs += ai32ssc_dsl_ioctl(_fd, AI32SSC_IOCTL_IRQ1_SEL, &set);

  if (numErrs > 0) {
    fprintf(stderr, "Error: %d general settings errors (%s)\n", numErrs, __FUNCTION__);
    return (1);
  }

  // Voltage range
  switch (config.voltageRange()) {
    case Gsc16ai::ConfigV1::VoltageRange_10V:
      set = AI32SSC_AIN_RANGE_10V;
      if (ai32ssc_dsl_ioctl(_fd, AI32SSC_IOCTL_AIN_RANGE, &set)) {
        ++numErrs;
      }
      break;
    case Gsc16ai::ConfigV1::VoltageRange_5V:
      set = AI32SSC_AIN_RANGE_5V;
      if (ai32ssc_dsl_ioctl(_fd, AI32SSC_IOCTL_AIN_RANGE, &set)) {
        ++numErrs;
      }
      break;
    case Gsc16ai::ConfigV1::VoltageRange_2_5V:
      set = AI32SSC_AIN_RANGE_2_5V;
      if (ai32ssc_dsl_ioctl(_fd, AI32SSC_IOCTL_AIN_RANGE, &set)) {
        ++numErrs;
      }
      break;
    default:
      fprintf(stderr, "Error: voltage range %hu not recognized\n",
              config.voltageRange());
      ++numErrs;
      break;
  }

  // Channel range
  set = 0;
  if (ai32ssc_dsl_ioctl(_fd, AI32SSC_IOCTL_CHAN_FIRST, &set)) {
    ++numErrs;
  }
  set = Gsc16ai::NumChannels - 1;
  if (ai32ssc_dsl_ioctl(_fd, AI32SSC_IOCTL_CHAN_LAST, &set)) {
    ++numErrs;
  }

  // Input mode
  switch (config.inputMode()) {
    case Gsc16ai::ConfigV1::InputMode_Differential:
      set = AI32SSC_AIN_MODE_DIFF;
      if (ai32ssc_dsl_ioctl(_fd, AI32SSC_IOCTL_AIN_MODE, &set)) {
        ++numErrs;
      }
      break;
    case Gsc16ai::ConfigV1::InputMode_Zero:
      set = AI32SSC_AIN_MODE_ZERO;
      if (ai32ssc_dsl_ioctl(_fd, AI32SSC_IOCTL_AIN_MODE, &set)) {
        ++numErrs;
      }
      break;
    case Gsc16ai::ConfigV1::InputMode_Vref:
      set = AI32SSC_AIN_MODE_VREF;
      if (ai32ssc_dsl_ioctl(_fd, AI32SSC_IOCTL_AIN_MODE, &set)) {
        ++numErrs;
      }
      break;
    default:
      fprintf(stderr, "Error: input mode %hu not recognized\n",
              config.inputMode());
      ++numErrs;
      break;
  }

  // Trigger mode
  switch (config.triggerMode()) {
    case Gsc16ai::ConfigV1::TriggerMode_ExtPos:
      set = AI32SSC_ADC_CLK_SRC_EXT;
      if (ai32ssc_dsl_ioctl(_fd, AI32SSC_IOCTL_ADC_CLK_SRC, &set)) {
        ++numErrs;
      }
      set = AI32SSC_IO_INV_HIGH;
      if (ai32ssc_dsl_ioctl(_fd, AI32SSC_IOCTL_IO_INV, &set)) {
        ++numErrs;
      }
      break;
    case Gsc16ai::ConfigV1::TriggerMode_ExtNeg:
      set = AI32SSC_ADC_CLK_SRC_EXT;
      if (ai32ssc_dsl_ioctl(_fd, AI32SSC_IOCTL_ADC_CLK_SRC, &set)) {
        ++numErrs;
      }
      set = AI32SSC_IO_INV_LOW;
      if (ai32ssc_dsl_ioctl(_fd, AI32SSC_IOCTL_IO_INV, &set)) {
        ++numErrs;
      }
      break;
    case Gsc16ai::ConfigV1::TriggerMode_IntClk:
      set = AI32SSC_ADC_CLK_SRC_RBG;
      if (ai32ssc_dsl_ioctl(_fd, AI32SSC_IOCTL_ADC_CLK_SRC, &set)) {
        ++numErrs;
      }
      // Frames per second
      set = 868;
      if (ai32ssc_dsl_ioctl(_fd, AI32SSC_IOCTL_RAG_NRATE, &set)) {
        ++numErrs;
      }

      set = AI32SSC_GEN_ENABLE_YES;
      if (ai32ssc_dsl_ioctl(_fd, AI32SSC_IOCTL_RAG_ENABLE, &set)) {
        ++numErrs;
      }
      set = 57600 / config.fps();
      if (ai32ssc_dsl_ioctl(_fd, AI32SSC_IOCTL_RBG_NRATE, &set)) {
        ++numErrs;
      }
      set = AI32SSC_RBG_CLK_SRC_RAG;
      if (ai32ssc_dsl_ioctl(_fd, AI32SSC_IOCTL_RBG_CLK_SRC, &set)) {
        ++numErrs;
      }
      set = AI32SSC_GEN_ENABLE_YES;
      if (ai32ssc_dsl_ioctl(_fd, AI32SSC_IOCTL_RBG_ENABLE, &set)) {
        ++numErrs;
      }
      break;
    default:
      fprintf(stderr, "Error: trigger mode %hu not recognized\n",
              config.triggerMode());
      ++numErrs;
      break;
  }

  // Data format
  set = AI32SSC_DATA_PACKING_DISABLE;
  if (ai32ssc_dsl_ioctl(_fd, AI32SSC_IOCTL_DATA_PACKING, &set)) {
    ++numErrs;
  }
  switch (config.dataFormat()) {
    case Gsc16ai::ConfigV1::DataFormat_TwosComplement:
      set = AI32SSC_DATA_FORMAT_2S_COMP;
      if (ai32ssc_dsl_ioctl(_fd, AI32SSC_IOCTL_DATA_FORMAT, &set)) {
        ++numErrs;
      }
      break;
    case Gsc16ai::ConfigV1::DataFormat_OffsetBinary:
      set = AI32SSC_DATA_FORMAT_OFF_BIN;
      if (ai32ssc_dsl_ioctl(_fd, AI32SSC_IOCTL_DATA_FORMAT, &set)) {
        ++numErrs;
      }
      break;
    default:
      fprintf(stderr, "Error: data format %hu not recognized\n",
              config.dataFormat());
      ++numErrs;
      break;
  }

  // Time tagging
  if (config.timeTagEnable()) {
    fprintf(stderr, "Error: time tag feature not supported\n");
    ++numErrs;
  }

  // Buffer clear
  if (ai32ssc_dsl_ioctl(_fd, AI32SSC_IOCTL_AIN_BUF_CLEAR, &set)) {
    fprintf(stderr, "Error: AI32SSC_IOCTL_AIN_BUF_CLEAR\n");
    ++numErrs;
  }

  // ADC on
  set = AI32SSC_ADC_ENABLE_YES;
  if (ai32ssc_dsl_ioctl(_fd, AI32SSC_IOCTL_ADC_ENABLE, &set)) {
    fprintf(stderr, "Error: AI32SSC_IOCTL_ADC_ENABLE\n");
    ++numErrs;
  }

  return (numErrs);
}

//
// unconfigure
//
// Returns zero on success, otherwise the number of errors.
//
int gsc16ai_dev::unconfigure(void) {
  int numErrs = 0;
  int set;

  if (_fd > 0) {
    // ADC off
    set = AI32SSC_ADC_ENABLE_NO;
    if (ai32ssc_dsl_ioctl(_fd, AI32SSC_IOCTL_ADC_ENABLE, &set)) {
      fprintf(stderr, "Error: AI32SSC_IOCTL_ADC_ENABLE\n");
      numErrs ++;
    }
    // Buffer clear
    if (ai32ssc_dsl_ioctl(_fd, AI32SSC_IOCTL_AIN_BUF_CLEAR, &set)) {
      fprintf(stderr, "Error: AI32SSC_IOCTL_AIN_BUF_CLEAR\n");
      numErrs ++;
    }
  }
  return (numErrs);
}

//
// get_bufLevel
//
// Returns number of samples in ADC FIFO on success, otherwise -1.
//
int gsc16ai_dev::get_bufLevel(void) {
  int rv = -1;
  if (_isOpen && (_fd > 0)) {
    if (ai32ssc_dsl_ioctl(_fd, AI32SSC_IOCTL_AIN_BUF_LEVEL, &rv)) {
      fprintf(stderr, "Error: ioctl AI32SSC_IOCTL_AIN_BUF_LEVEL\n");
      rv = -1;
    } 
  }
  return (rv);
}

//
// calibrate - initiate an auto-calibration cycle
//
// Returns O on success, otherwise -1.
//
int gsc16ai_dev::calibrate() {
  int rv = -1;
  if (_isOpen && (_fd > 0)) {
    if (ai32ssc_dsl_ioctl(_fd, AI32SSC_IOCTL_AUTO_CALIBRATE, NULL) == 0) {
      rv = 0;
    }
  }
  return (rv);
}

//
// waitEventInBufThrL2H -
//
int gsc16ai_dev::waitEventInBufThrL2H(int timeout_ms)
{
  int rv = gsc16ai_dev::waitEventError;
  gsc_wait_t  gWait = {0, 0, AI32SSC_WAIT_GSC_IN_BUF_THR_L2H, 0, 0, (unsigned) timeout_ms, 0};

  if (ai32ssc_dsl_ioctl(_fd, AI32SSC_IOCTL_WAIT_EVENT, &gWait)) {
    fprintf(stderr, "Error: ioctl AI32SSC_IOCTL_WAIT_EVENT\n");
  } else if (gWait.flags == GSC_WAIT_FLAG_TIMEOUT) {
    // timed out
    rv = gsc16ai_dev::waitEventTimeout;
  } else if (gWait.flags == GSC_WAIT_FLAG_DONE) {
    // detected event
    rv = gsc16ai_dev::waitEventReady;
  }

  return (rv);
}
