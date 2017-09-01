#include "Driver.hh"
#include "slsDetectorUsers.h"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#define EVENTS_TO_BUFFER 10
#define PACKET_NUM 128
#define DATA_ELEM 4096
#define CMD_LEN 128
#define MSG_LEN 256

// Temporary fixed sizes :(
#define NUM_ROWS 512
#define NUM_COLUMNS 1024

#define put_command_print(cmd, val, ...) \
  { printf("cmd_put %s: %s\n", cmd, put_command(cmd, val, ##__VA_ARGS__).c_str()); }

#define put_register_print(cmd, val, ...) \
  { printf("reg_put %#x - %#x: %s\n", cmd, val, put_register(cmd, val, ##__VA_ARGS__).c_str()); }

#define put_adcreg_print(cmd, val, ...) \
  { printf("adc_put %#x - %#x: %s\n", cmd, val, put_adcreg(cmd, val, ##__VA_ARGS__).c_str()); }

#define setbit_print(cmd, val, ...) \
  { printf("setbit %#x - %d: %s\n", cmd, val, setbit(cmd, val, ##__VA_ARGS__).c_str()); }

#define clearbit_print(cmd, val, ...) \
  { printf("clearbit %#x - %d: %s\n", cmd, val, clearbit(cmd, val, ##__VA_ARGS__).c_str()); }

#define error_print(fmt, ...) \
  { snprintf(_msgbuf, MSG_LEN, fmt, ##__VA_ARGS__); fprintf(stderr, _msgbuf); }

using namespace Pds::Jungfrau;

static const char* ADCREG   = "adcreg";
static const char* REG      = "reg";
static const char* SETBIT   = "setbit";
static const char* CLEARBIT = "clearbit";
static struct sockaddr_in clientaddr;
static socklen_t clientaddrlen = sizeof(clientaddr);

#pragma pack(push)
#pragma pack(2)
  struct jungfrau_dgram {
    char emptyheader[6];
    uint64_t framenum;
    uint32_t exptime;
    uint32_t packetnum;
    uint64_t bunchid;
    uint64_t timestamp;
    uint16_t moduleID;
    uint16_t xCoord;
    uint16_t yCoord;
    uint16_t zCoord;
    uint32_t debug;
    uint16_t roundRobin;
    uint8_t detectortype;
    uint8_t headerVersion;
    uint16_t data[DATA_ELEM];
  };
#pragma pack(pop)

struct frame_thread_args {
  uint64_t* frame;
  uint16_t* data;
  JungfrauModInfoType* metadata; 
  Module* module;
  bool status;
};

static void* frame_thread(void* args_ptr)
{
  frame_thread_args* args = (frame_thread_args*) args_ptr;
   args->status = args->module->get_frame(args->frame, args->metadata, args->data);

  return NULL;
}

DacsConfig::DacsConfig(uint16_t vb_ds, uint16_t vb_comp, uint16_t vb_pixbuf, uint16_t vref_ds,
                       uint16_t vref_comp, uint16_t vref_prech, uint16_t vin_com, uint16_t  vdd_prot):
  _vb_ds(vb_ds),
  _vb_comp(vb_comp),
  _vb_pixbuf(vb_pixbuf),
  _vref_ds(vref_ds),
  _vref_comp(vref_comp),
  _vref_prech(vref_prech),
  _vin_com(vin_com),
  _vdd_prot(vdd_prot)
{}

DacsConfig::~DacsConfig() {}

uint16_t DacsConfig::vb_ds() const      { return _vb_ds; }
uint16_t DacsConfig::vb_comp() const    { return _vb_comp; }
uint16_t DacsConfig::vb_pixbuf() const  { return _vb_pixbuf; }
uint16_t DacsConfig::vref_ds() const    { return _vref_ds; }
uint16_t DacsConfig::vref_comp() const  { return _vref_comp; }
uint16_t DacsConfig::vref_prech() const { return _vref_prech; }
uint16_t DacsConfig::vin_com() const    { return _vin_com; }
uint16_t DacsConfig::vdd_prot() const   { return _vdd_prot; }

Module::Module(const int id, const char* control, const char* host, unsigned port, const char* mac, const char* det_ip, bool config_det_ip) :
  _id(id), _control(control), _host(host), _port(port), _mac(mac), _det_ip(det_ip),
  _socket(-1), _connected(false), _boot(true),
  _sockbuf_sz(sizeof(jungfrau_dgram)*PACKET_NUM*EVENTS_TO_BUFFER), _readbuf_sz(sizeof(jungfrau_dgram)), _frame_sz(DATA_ELEM * sizeof(uint16_t)), _frame_elem(DATA_ELEM),
  _speed(JungfrauConfigType::Quarter)
{
  _readbuf = new char[_readbuf_sz];
  _msgbuf  = new char[MSG_LEN];
  for (int i=0; i<MAX_JUNGFRAU_CMDS; i++) {
    _cmdbuf[i] = new char[CMD_LEN];
  }
  _socket = ::socket(AF_INET, SOCK_DGRAM, 0);
  ::setsockopt(_socket, SOL_SOCKET, SO_RCVBUF, &_sockbuf_sz, sizeof(unsigned));

  _det = new slsDetectorUsers(_id);
  std::string reply = put_command("hostname", _control);
  std::string type  = get_command("type");

  bool needs_config = false;
  bool detmac_status = false;
  std::string rx_ip  = get_command("rx_udpip");
  std::string rx_mac = get_command("rx_udpmac");

  if (!strcmp(rx_ip.c_str(), "none") || !strcmp(rx_mac.c_str(), "none")) {
    printf("detector udp_rx interface appears to be unset\n");
    needs_config = true;
  }

  if (config_det_ip || needs_config) {
    printf("setting up detector udp_rx interface\n");
    put_command_print("rx_udpport", _port);
    put_command_print("rx_udpip",   _host);
    put_command_print("rx_udpmac", _mac);
    put_command_print("detectorip", _det_ip);
    put_command_print("detectormac", "00:aa:bb:cc:dd:ee");
    std::string cfgmac_reply = put_command("configuremac", 0);
    printf("cmd_put configuremac: %s\n", cfgmac_reply.c_str());
    if (!strcmp(cfgmac_reply.c_str(), "3") || !strcmp(cfgmac_reply.c_str(), "0")) {
      detmac_status = true;
      printf("detector udp_rx interface is up\n");
    } else {
      error_print("Error: detector udp_rx interface did not come up\n");
    }
  } else {
    printf("detector udp_rx interface is being set externally\n");
    detmac_status = true;
  }

  hostent* entries = gethostbyname(host);
  if (entries) {
    unsigned addr = htonl(*(in_addr_t*)entries->h_addr_list[0]);

    sockaddr_in sa;
    sa.sin_family = AF_INET;
    sa.sin_addr.s_addr = htonl(addr);
    sa.sin_port        = htons(port);

    int nb = ::bind(_socket, (sockaddr*)&sa, sizeof(sa));
    if (nb<0) {
      error_print("Error: failed to bind to Jungfrau data receiver at %s on port %d: %s\n", host, port, strerror(errno));
    } else if (strncmp(control, reply.c_str(), strlen(control)) != 0) {
      error_print("Error: failed to connect to Jungfrau control interface at %s: %s\n", control, reply.c_str());
    } else if (strcmp(type.c_str(), "Jungfrau+") !=0) {
      error_print("Error: detector at %s on port %d is not a Jungfrau: %s\n", host, port, type.c_str());
    } else {
      _connected=detmac_status;
    }
  }
}

Module::~Module()
{
  if (_socket >= 0) {
    ::close(_socket);
  }
  if (_readbuf) {
    delete[] _readbuf;
  }
  if (_msgbuf) {
    delete[] _msgbuf;
  }
  if (_det) {
    put_command("free", "0");
    delete _det;
  }
  for (int i=0; i<MAX_JUNGFRAU_CMDS; i++) {
    if (_cmdbuf[i]) delete[] _cmdbuf[i];
  }
}

bool Module::configure(uint64_t nframes, JungfrauConfigType::GainMode gain, JungfrauConfigType::SpeedMode speed, double trig_delay, double exposure_time, double exposure_period, uint32_t bias, const DacsConfig& dac_config)
{
  if (!_connected) {
    error_print("Error: failed to connect to Jungfrau at address %s\n", _control);
    return false;
  }

  // stop the detector if not!
  if (status() == RUNNING) {
    if (!stop()) {
      error_print("Error: can't configure when the detector is in a running state\n");
      return false;
    }
  }

  if (_boot) {
    // Setting Dacs 12bit on 2.5V  (i.e. 2.5v=4096)
    printf("Setting Dacs:\n");
  
    // setting vb_ds
    printf("setting vb_ds to %hu\n", dac_config.vb_ds());
    put_command_print("dac:5", dac_config.vb_ds());

    // setting vb_comp
    printf("setting vb_comp to %hu\n", dac_config.vb_comp());
    put_command_print("dac:0", dac_config.vb_comp());

    // setting vb_pixbuf
    printf("setting vb_pixbuf to %hu\n", dac_config.vb_pixbuf());
    put_command_print("dac:4", dac_config.vb_pixbuf());

    // setting vref_ds
    printf("setting vref_ds to %hu\n", dac_config.vref_ds());
    put_command_print("dac:6", dac_config.vref_ds());

    // setting vref_comp
    printf("setting vref_comp to %hu\n", dac_config.vref_comp());
    put_command_print("dac:7", dac_config.vref_comp());

    // setting vref_prech
    printf("setting vref_prech to %hu\n", dac_config.vref_prech());  
    put_command_print("dac:3", dac_config.vref_prech());

    // setting vin_com
    printf("setting vin_com to %hu\n", dac_config.vin_com());
    put_command_print("dac:2", dac_config.vin_com());

    // setting vdd_prot
    printf("setting vdd_prot to %hu\n", dac_config.vdd_prot());
    put_command_print("dac:1", dac_config.vdd_prot());

    // power on the chips
    printf("powering on the chip\n");
    put_register_print(0x5e, 0x1);

    // reset adc
    printf("resetting the adc\n");
    put_adcreg_print(0x08, 0x3);
    put_adcreg_print(0x08, 0x0);
    put_adcreg_print(0x14, 0x40);
    put_adcreg_print(0x4, 0xf);
    put_adcreg_print(0x5, 0x3f);
    put_adcreg_print(0x18, 0x2);
    put_register_print(0x43, 0x453b2a9c);
  }

  if (_boot || speed != _speed) {
    // set speed
    switch (speed) {
    case JungfrauConfigType::Quarter:
      printf("setting detector to quarter speed\n");
      put_register_print(0x59, 0x2810);
      put_register_print(0x4d, 0x00000000);
      put_register_print(0x42, 0x0f);
      put_register_print(0x5d, 0x00000f00);
      put_command_print("adcphase", 25);
      break;
    case JungfrauConfigType::Half:
      printf("setting detector to half speed\n");
      put_register_print(0x59, 0x1000);
      put_register_print(0x4d, 0x00100000);
      put_register_print(0x42, 0x20);
      put_register_print(0x5d, 0x00000f00);
      put_command_print("adcphase", 65);
      break;
    default:
      error_print("Error: invalid clock speed setting for the camera %d\n", speed);
      return false;
    } 
    _speed = speed;
    sleep(1);
  }

  reset();

  printf("setting trigger delay to %.6f\n", trig_delay);
  put_command("delay", trig_delay);

  if (!nframes) {
    printf("configuring triggered mode\n");
    put_register_print(0x4e, 0x3333);
    put_command_print("cycles", 10000000000);
    put_command_print("frames", 1);
    put_command_print("period", exposure_period);
  } else {
    printf("configuring for free run\n");
    put_register_print(0x4e, 0x0000);
    put_command_print("cycles", 1);
    put_command_print("frames", nframes);
    put_command_print("period", exposure_period);
  }

  printf("setting exposure time to %.6f seconds\n", exposure_time);
  put_command_print("exptime", 0.000010);

  printf("setting bias voltage to %d volts\n", bias);
  put_command_print("vhighvoltage", bias);

  printf("setting gain mode %d\n", gain);
  // Clear all the bits
  clearbit_print(0x5d, 0);
  clearbit_print(0x5d, 1);
  clearbit_print(0x5d, 2);
  clearbit_print(0x5d, 12);
  clearbit_print(0x5d, 13);
  // Now set the ones of the gain we want
  switch(gain) {
    case JungfrauConfigType::Normal:
      break;
    case JungfrauConfigType::FixedGain1:
      setbit_print(0x5d, 1);
      break;
    case JungfrauConfigType::FixedGain2:
      setbit_print(0x5d, 1);
      setbit_print(0x5d, 2);
      break;
    case JungfrauConfigType::ForcedGain1:
      setbit_print(0x5d, 12);
      break;
    case JungfrauConfigType::ForcedGain2:
      setbit_print(0x5d, 12);
      setbit_print(0x5d, 13);
      break;
    case JungfrauConfigType::HighGain0:
      setbit_print(0x5d, 0);
      break;
  }

  _boot = false;

  return status() != ERROR;
}

bool Module::check_size(uint32_t num_rows, uint32_t num_columns) const
{
  return (num_rows == NUM_ROWS) && (num_columns == NUM_COLUMNS);
}

std::string Module::put_command(const char* cmd, const char* value, int pos)
{
  if (strlen(cmd) >= CMD_LEN || strlen(value) >= CMD_LEN) {
    return "invalid command or value length";
  } 
  strcpy(_cmdbuf[0], cmd);
  strcpy(_cmdbuf[1], value);
  if (value) {
    return _det->putCommand(2, _cmdbuf, pos);
  } else {
    return _det->putCommand(1, _cmdbuf, pos);
  }
}

std::string Module::put_command(const char* cmd, const short value, int pos)
{
  if (strlen(cmd) >= CMD_LEN) {
    return "invalid command or value length";
  }
  strcpy(_cmdbuf[0], cmd);
  sprintf(_cmdbuf[1], "%hd", value);
  return _det->putCommand(2, _cmdbuf, pos);
}

std::string Module::put_command(const char* cmd, const unsigned short value, int pos)
{
  if (strlen(cmd) >= CMD_LEN) {
    return "invalid command or value length";
  }
  strcpy(_cmdbuf[0], cmd);
  sprintf(_cmdbuf[1], "%hu", value);
  return _det->putCommand(2, _cmdbuf, pos);
}

std::string Module::put_command(const char* cmd, const int value, int pos)
{
  if (strlen(cmd) >= CMD_LEN) {
    return "invalid command or value length";
  }
  strcpy(_cmdbuf[0], cmd);
  sprintf(_cmdbuf[1], "%d", value);
  return _det->putCommand(2, _cmdbuf, pos); 
}

std::string Module::put_command(const char* cmd, const unsigned int value, int pos)
{
  if (strlen(cmd) >= CMD_LEN) {
    return "invalid command or value length";
  }
  strcpy(_cmdbuf[0], cmd);
  sprintf(_cmdbuf[1], "%u", value);
  return _det->putCommand(2, _cmdbuf, pos);
}

std::string Module::put_command(const char* cmd, const long value, int pos)
{
  if (strlen(cmd) >= CMD_LEN) {
    return "invalid command or value length";
  }
  strcpy(_cmdbuf[0], cmd);
  sprintf(_cmdbuf[1], "%ld", value);
  return _det->putCommand(2, _cmdbuf, pos);
}

std::string Module::put_command(const char* cmd, const unsigned long value, int pos)
{
  if (strlen(cmd) >= CMD_LEN) {
    return "invalid command or value length";
  }
  strcpy(_cmdbuf[0], cmd);
  sprintf(_cmdbuf[1], "%lu", value);
  return _det->putCommand(2, _cmdbuf, pos);
}

std::string Module::put_command(const char* cmd, const long long value, int pos)
{
  if (strlen(cmd) >= CMD_LEN) {
    return "invalid command or value length";
  }
  strcpy(_cmdbuf[0], cmd);
  sprintf(_cmdbuf[1], "%lld", value);
  return _det->putCommand(2, _cmdbuf, pos);
}

std::string Module::put_command(const char* cmd, const unsigned long long value, int pos)
{
  if (strlen(cmd) >= CMD_LEN) {
    return "invalid command or value length";
  }
  strcpy(_cmdbuf[0], cmd);
  sprintf(_cmdbuf[1], "%llu", value);
  return _det->putCommand(2, _cmdbuf, pos);
}

std::string Module::put_command(const char* cmd, const double value, int pos)
{
  if (strlen(cmd) >= CMD_LEN) {
    return "invalid command or value length";
  }
  strcpy(_cmdbuf[0], cmd);
  sprintf(_cmdbuf[1], "%f", value);
  return _det->putCommand(2, _cmdbuf, pos);
}

std::string Module::get_command(const char* cmd, int pos)
{
  if (strlen(cmd) >= CMD_LEN) {
    return "invalid command or value length";
  }
  strcpy(_cmdbuf[0], cmd);
  return _det->getCommand(1, _cmdbuf, pos);
}

std::string Module::put_register(const int reg, const int value, int pos)
{
  strcpy(_cmdbuf[0], REG);
  sprintf(_cmdbuf[1], "%#x", reg);
  sprintf(_cmdbuf[2], "%#x", value);
  return _det->putCommand(3, _cmdbuf, pos);
}

std::string Module::put_adcreg(const int reg, const int value, int pos)
{
  strcpy(_cmdbuf[0], ADCREG);
  sprintf(_cmdbuf[1], "%#x", reg);
  sprintf(_cmdbuf[2], "%#x", value);
  return _det->putCommand(3, _cmdbuf, pos);
}

std::string Module::setbit(const int reg, const int bit, int pos)
{
  strcpy(_cmdbuf[0], SETBIT);
  sprintf(_cmdbuf[1], "%#x", reg);
  sprintf(_cmdbuf[2],  "%d", bit);
  return _det->putCommand(3, _cmdbuf, pos);
}

std::string Module::clearbit(const int reg, const int bit, int pos)
{
  strcpy(_cmdbuf[0], CLEARBIT);
  sprintf(_cmdbuf[1], "%#x", reg);
  sprintf(_cmdbuf[2],  "%d", bit);  
  return _det->putCommand(3, _cmdbuf, pos);
}

uint64_t Module::nframes()
{
  std::string reply = get_command("nframes");
  return strtoull(reply.c_str(), NULL, 10);
}

Module::Status Module::status(const std::string& reply)
{
  if (!strcmp(reply.c_str(), "waiting"))
    return WAIT;
  else if (!strcmp(reply.c_str(), "idle"))
    return IDLE;
  else if (!strcmp(reply.c_str(), "running"))
    return RUNNING;
  else if (!strcmp(reply.c_str(), "data"))
    return DATA;
  else
    return ERROR;
}

Module::Status Module::status()
{
  std::string reply = get_command("status");
  return status(reply);
}

std::string Module::status_str()
{
  return get_command("status");
}

bool Module::start()
{
  std::string reply = put_command("status", "start");
  printf("starting detector: %s\n", reply.c_str());
  return status(reply) == RUNNING || status(reply) == WAIT;
}

bool Module::stop()
{
  std::string reply = put_command("status", "stop");
  if (!strcmp(reply.c_str(), "error")) {
    printf("stopping detector:  done\n");
    reset();
  } else {
    printf("stopping detector: %s\n", reply.c_str());
  }
  return status(reply) != RUNNING;
}

void Module::reset()
{
  printf("reseting run control ...");
  // reset mem machine fifos fifos
  put_register(0x4f, 0x4000);
  put_register(0x4f, 0x0);
  // reset run control
  put_register(0x4f, 0x0400);
  put_register(0x4f, 0x0);
  printf(" done\n");
}

bool Module::get_frame(uint64_t* frame, uint16_t* data)
{
  return get_frame(frame, NULL, data);
}

bool Module::get_frame(uint64_t* frame, JungfrauModInfoType* metadata, uint16_t* data)
{
  int nb;
  unsigned npackets = 0;
  uint64_t cur_frame = 0;
  bool first_packet = true;
  bool last_packet = false;
  jungfrau_dgram* packet = NULL;

  while (!last_packet) {
    nb = ::recvfrom(_socket, _readbuf, sizeof(jungfrau_dgram), 0, (struct sockaddr *)&clientaddr, &clientaddrlen);
    if (nb<0) {
      fprintf(stderr,"Error: failure receiving packet from Jungfru at %s on port %d: %s\n", _host, _port, strerror(errno));
      return false;
    }

    packet = (jungfrau_dgram*) _readbuf;

    if (first_packet) {
      cur_frame = packet->framenum;
      first_packet = false;
    } else if(cur_frame != packet->framenum) {
      fprintf(stderr,"Error: data out-of-order got data for frame %lu, but was expecting frame %lu\n", packet->framenum, cur_frame);
      return false;
    }
    
    memcpy(&data[_frame_elem*packet->packetnum], packet->data, _frame_sz);
    last_packet = (packet->packetnum == (PACKET_NUM -1));
    npackets++;
  }

  if ((npackets == PACKET_NUM) && packet) {
    *frame = cur_frame;
    if (metadata) new(metadata) JungfrauModInfoType(packet->timestamp, packet->exptime, packet->moduleID, packet->xCoord, packet->yCoord, packet->zCoord);
  }
 
  return (npackets == PACKET_NUM);
}

const char* Module::error()
{
  return _msgbuf;
}

void Module::clear_error()
{
  _msgbuf[0] = 0;
}

unsigned Module::get_num_rows() const
{
  return NUM_ROWS;
}

unsigned Module::get_num_columns() const
{
  return NUM_COLUMNS;
}

unsigned Module::get_num_pixels() const
{
  return NUM_ROWS * NUM_COLUMNS;
}

unsigned Module::get_frame_size() const
{
  return NUM_ROWS * NUM_COLUMNS * sizeof(uint16_t);
}

Detector::Detector(std::vector<Module*>& modules) :
  _threads(new pthread_t[modules.size()]),
  _num_modules(modules.size()),
  _module_frames(new uint64_t[modules.size()]),
  _modules(modules),
  _msgbuf(new const char*[modules.size()]) {}

Detector::~Detector()
{
  for (unsigned i=0; i<_num_modules; i++) {
    delete _modules[i];
  }
  _modules.clear();
  if (_threads) delete[] _threads;
  if (_module_frames) delete[] _module_frames;
  if (_msgbuf) delete[] _msgbuf;
}

bool Detector::check_size(uint32_t num_modules, uint32_t num_rows, uint32_t num_columns) const
{
  for (unsigned i=0; i<_num_modules; i++) {
    if (!_modules[i]->check_size(num_rows, num_columns))
      return false;
  }

  return (num_modules == _num_modules);
}

bool Detector::configure(uint64_t nframes, JungfrauConfigType::GainMode gain, JungfrauConfigType::SpeedMode speed, double trig_delay, double exposure_time, double exposure_period, uint32_t bias, const DacsConfig& dac_config)
{
  bool success = true;

  for (unsigned i=0; i<_num_modules; i++) {
    if (!_modules[i]->configure(nframes, gain, speed, trig_delay, exposure_time, exposure_period, bias, dac_config)) {
      fprintf(stderr,"Error: module %u failed to configure!\n", i);
      success = false;
    }
  }

  return success;
}

bool Detector::get_frame(uint64_t* frame, uint16_t* data)
{
  return get_frame(frame, NULL, data);
}

bool Detector::get_frame(uint64_t* frame, JungfrauModInfoType* metadata, uint16_t* data)
{
  bool drop_frame = false;
  bool frame_unset = true;
  frame_thread_args args[_num_modules];

  // Start frame reading threads
  for (unsigned i=0; i<_num_modules; i++) {
    args[i].frame = &_module_frames[i];
    args[i].data = data;
    args[i].metadata = metadata;
    args[i].module = _modules[i];
    pthread_create(&_threads[i], NULL, frame_thread, &args);

    data += _modules[i]->get_num_pixels();
    if(metadata) metadata++;
  }

  // Wait for the threads to finish
  for (unsigned i=0; i<_num_modules; i++) {
    pthread_join(_threads[i], NULL);
    if (!args[i].status) {
      fprintf(stderr,"Error: module %u failed to return a frame!\n", i);
      drop_frame = true;
    }
  }

  if (!drop_frame) {
    // Check that we got a consistent frame
    for (unsigned i=0; i<_num_modules; i++) {
      if (frame_unset) {
        *frame = _module_frames[i];
        frame_unset = false;
      } else {
        if (*frame != _module_frames[i]) {
          fprintf(stderr,"Error: data out-of-order got data for module %u got frame %lu, but was expecting frame %lu\n", i, _module_frames[i], *frame);
          drop_frame = true;
        }
      }
    }
  }

  return !drop_frame;
}

bool Detector::start()
{
  bool success = true;
  for (unsigned i=0; i<_num_modules; i++) {
    if (!_modules[i]->start()) {
      fprintf(stderr,"Error: failure starting acquistion of module %u of the detector!\n", i);
      success = false;
    }
  }
  return success;
}

bool Detector::stop()
{
  bool success = true;
  for (unsigned i=0; i<_num_modules; i++) {
    if (!_modules[i]->stop()) {
      fprintf(stderr,"Error: failure stopping acquistion of module %u of the detector!\n", i);
      success = false;
    }
  }
  return success;
}

unsigned Detector::get_num_rows(unsigned module) const
{
  if (module < _num_modules) {
    return _modules[module]->get_num_rows();
  } else {
    fprintf(stderr,"Error: detector does not contain module %u - only %u modules in detector!\n", module, _num_modules);
    return 0;
  }
}

unsigned Detector::get_num_columns(unsigned module) const
{
  if (module < _num_modules) {
    return _modules[module]->get_num_columns();
  } else {
    fprintf(stderr,"Error: detector does not contain module %u - only %u modules in detector!\n", module, _num_modules);
    return 0;
  }
}

unsigned Detector::get_num_modules() const
{
  return _num_modules;
}

unsigned Detector::get_frame_size() const
{
  unsigned total_frame_size = 0;
  for (unsigned i=0; i<_num_modules; i++) {
    total_frame_size += _modules[i]->get_frame_size();
  }
  return total_frame_size;
}

const char** Detector::errors() 
{
  for (unsigned i=0; i<_num_modules; i++) {
    _msgbuf[i] = _modules[i]->error();
  }
  return _msgbuf;
}

void Detector::clear_errors()
{
  for (unsigned i=0; i<_num_modules; i++) {
    _modules[i]->clear_error();
  }
}

#undef EVENTS_TO_BUFFER
#undef PACKET_NUM
#undef DATA_ELEM
#undef CMD_LEN
#undef MSG_LEN
#undef NUM_ROWS
#undef NUM_COLUMNS
#undef put_command_print
#undef put_register_print
#undef put_adcreg_print
#undef setbit_print
#undef clearbit_print
#undef error_print
