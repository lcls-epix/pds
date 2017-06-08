#include "Server.hh"
#include "pds/config/ZylaDataType.hh"

#include <unistd.h>
#include <sys/uio.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>

using namespace Pds::Zyla;

Server::Server( const Src& client )
  : _xtc( _zylaDataType, client ), _count(0), _framesz(0), _last_frame(0)
{
  _xtc.extent = sizeof(ZylaDataType) + sizeof(Xtc);
  int err = ::pipe(_pfd);
  if (err) {
    fprintf(stderr, "%s pipe error: %s\n", __FUNCTION__, strerror(errno));
  } else {
    fd(_pfd[0]);
  }
}

int Server::fetch( char* payload, int flags )
{
  Xtc& xtc = *reinterpret_cast<Xtc*>(payload);
  memcpy(payload, &_xtc, sizeof(Xtc));

  int len = ::read(_pfd[0], xtc.payload(), sizeof(ZylaDataType) + _framesz);
  if (len != (int) (sizeof(ZylaDataType) + _framesz)) {
    fprintf(stderr, "Error: read() returned %d in %s\n", len, __FUNCTION__);
    return -1;
  }

  uint64_t current_frame = *(uint64_t*) xtc.payload();
  // Check that the frame count is sequential
  /*if (_last_frame == 0) {
    _last_frame = current_frame;
    _count++;
  } else {
    if (current_frame - _last_frame > 1) {
      fprintf(stderr, "Error: expected frame %d instead of %d - it appears that %d frames have been dropped\n", _last_frame+1, current_frame, (current_frame - _last_frame - 1));
    }
    _count+=(current_frame-_last_frame);
  }*/
  _count++;

  _last_frame = current_frame;

  return xtc.extent;
}

unsigned Server::count() const
{
  return _count - 1;
}

void Server::resetCount()
{
  _count = 0;
  _last_frame = 0;
}

void Server::post(char* payload, unsigned sz)
{
  ::write(_pfd[1], payload, sz);
}

void Server::set_frame_sz(size_t sz)
{
  _framesz = sz;
  _xtc.extent = sizeof(ZylaDataType) + sizeof(Xtc) + sz;
}
