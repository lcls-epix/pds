#include "pds/camera/FexFrameServer.hh"

#include "pdsdata/camera/FrameFexConfigV1.hh"

#include "pds/camera/DmaSplice.hh"
#include "pds/camera/Frame.hh"
#include "pds/camera/FrameHandle.hh"
#include "pds/camera/FrameServerMsg.hh"
#include "pds/camera/TwoDGaussian.hh"

#include "pds/service/ZcpFragment.hh"

#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sys/uio.h>
#include <new>

using namespace Pds;

typedef unsigned short pixel_type;

FexFrameServer::FexFrameServer(const Src& src,
			       DmaSplice& splice) :
  _splice(splice),
  _more  (false),
  _xtc   (TypeId::Id_Xtc, src),
  _config(0)
{
  int err = ::pipe(_fd);
  if (err)
    printf("Error opening FexFrameServer pipe: %s\n",strerror(errno));
  fd(_fd[0]);
}

FexFrameServer::~FexFrameServer()
{
  ::close(_fd[0]);
  ::close(_fd[1]);
}

void FexFrameServer::Config(const Camera::FrameFexConfigV1& cfg,
			    unsigned camera_offset)
{
  _config = &cfg;
  _camera_offset = camera_offset;
  _nposts = 0;
}

void FexFrameServer::post(FrameServerMsg* msg)
{
  msg->connect(_msg_queue.reverse());
  ::write(_fd[1],&msg,sizeof(msg));
  if (msg->count != _nposts)
    printf("Camera frame number(%d) != Server number(%d)\n",
	  msg->count, _nposts);
  _nposts++;
}

void FexFrameServer::dump(int detail) const
{
}

bool FexFrameServer::isValued() const
{
  return true;
}

const Src& FexFrameServer::client() const
{
  return _xtc.src;
}

const Xtc& FexFrameServer::xtc() const
{
  return _xtc;
}

//
//  Fragment information
//
bool FexFrameServer::more() const
{
  return _more;
}

unsigned FexFrameServer::length() const
{
  return _xtc.extent;
}

unsigned FexFrameServer::offset() const
{
  return _offset;
}

int FexFrameServer::pend(int flag)
{
  return 0;
}

//
//  Apply feature extraction to the input frame and
//  provide the result to the event builder
//
int FexFrameServer::fetch(char* payload, int flags)
{
  FrameServerMsg* msg;
  int length = ::read(_fd[0],&msg,sizeof(msg));
  if (length >= 0) {
    FrameServerMsg* fmsg = _msg_queue.remove();
    _count = fmsg->count;

    //  Is pipe write/read good enough?
    if (msg != fmsg) printf("Overlapping events %d/%d\n",msg->count,fmsg->count);

    Frame frame(*fmsg->handle, _camera_offset);
    const unsigned short* frame_data = 
      reinterpret_cast<const unsigned short*>(fmsg->handle->data);
    delete fmsg;

    if (_config->forwarding(Camera::FrameFexConfigV1::Summary)) {
      if (_config->forwarding(Camera::FrameFexConfigV1::FullFrame) ||
	  _config->forwarding(Camera::FrameFexConfigV1::RegionOfInterest)) {
	Xtc* xtc = new (payload) Xtc(TypeId::Id_Xtc,_xtc.src);
	xtc->extent += _post_fex  (xtc->next(), frame, frame_data);
	xtc->extent += _post_frame(xtc->next(), frame, frame_data);
	return xtc->extent;
      }
      else
	return _post_fex( payload, frame, frame_data );
    }
    else if (_config->forwarding(Camera::FrameFexConfigV1::FullFrame) ||
	     _config->forwarding(Camera::FrameFexConfigV1::RegionOfInterest))
      return _post_frame( payload, frame, frame_data );
    else
      return 0;
  }
  else
    printf("FexFrameServer::fetch error: %s\n",strerror(errno));

  return length;
}

//
//  Apply feature extraction to the input frame and
//  provide the result to the (zero-copy) event builder
//
int FexFrameServer::fetch(ZcpFragment& zfo, int flags)
{
  _more = false;
  int length = 0;
  FrameServerMsg* fmsg;

  fmsg = _msg_queue.remove();
  _count = fmsg->count;

  FrameServerMsg* msg;
  length = ::read(_fd[0],&msg,sizeof(msg));
  if (length < 0) throw length;
  
  Frame frame(*fmsg->handle, _camera_offset);
  const unsigned short* frame_data = reinterpret_cast<const unsigned short*>(fmsg->handle->data);
  
  if (fmsg->type == FrameServerMsg::NewFrame) {
    
    if (_config->forwarding(Camera::FrameFexConfigV1::Summary)) {
      if (_config->forwarding(Camera::FrameFexConfigV1::FullFrame)) {
	Frame zcpFrame(frame.width(), frame.height(), 
		       frame.depth(), frame.offset(),
		       *fmsg->handle, _splice);
	length = _queue_fex_and_frame( _feature_extract(frame,frame_data), zcpFrame, fmsg, zfo );
      }
      else if (_config->forwarding(Camera::FrameFexConfigV1::RegionOfInterest)) {
	Frame zcpFrame(_config->roiBegin().column,
		       _config->roiEnd  ().column,
		       _config->roiBegin().row,
		       _config->roiEnd  ().row,
		       frame.width(), frame.height(), 
		       frame.depth(), frame.offset(),
		       *fmsg->handle,_splice);
	length = _queue_fex_and_frame( _feature_extract(frame,frame_data), zcpFrame, fmsg, zfo );
      }
      else
	length = _queue_fex( _feature_extract(frame,frame_data), fmsg, zfo );
    }
    else if (_config->forwarding(Camera::FrameFexConfigV1::FullFrame)) {
      Frame zcpFrame(frame.width(), frame.height(), 
		     frame.depth(), frame.offset(),
		     *fmsg->handle, _splice);
      length = _queue_frame( zcpFrame, fmsg, zfo );
    }
    else if (_config->forwarding(Camera::FrameFexConfigV1::RegionOfInterest)) {
      Frame zcpFrame(_config->roiBegin().column,
		     _config->roiEnd  ().column,
		     _config->roiBegin().row,
		     _config->roiEnd  ().row,
		     frame.width(), frame.height(),
		     frame.depth(), frame.offset(),
		     *fmsg->handle,_splice);
      length = _queue_frame( zcpFrame, fmsg, zfo );
    }
    else {
      length = 0;
    }
  }
  else { // Post_Fragment
    _more       = true;
    _xtc.extent = fmsg->extent;
    _offset     = fmsg->offset;
    int remaining = fmsg->extent - fmsg->offset;

    try {
      if ((length = zfo.kinsert( _splice.fd(), remaining)) < 0) throw length;
      if (length != remaining) {
	fmsg->offset += length;
	fmsg->connect(reinterpret_cast<FrameServerMsg*>(&_msg_queue));
	::write(_fd[1],&fmsg,sizeof(fmsg));
      }
      else
	delete fmsg;
    }
    catch (int err) {
      printf("FexFrameServer::fetchz error: %s : %d\n",strerror(errno),length);
      delete fmsg;
      return -1;
    }
  }
  return length;
}

unsigned FexFrameServer::count() const
{
  return _count;
}

unsigned FexFrameServer::_post_fex(void* xtc, 
				   const Frame& frame, const unsigned short* frame_data) const
{
  Xtc& fexXtc = *new((char*)xtc) Xtc(TypeId::Id_TwoDGaussian, _xtc.src);
  new(fexXtc.alloc(sizeof(TwoDGaussian))) TwoDGaussian(_feature_extract(frame,frame_data));
  return fexXtc.extent;
}

unsigned FexFrameServer::_post_frame(void* xtc, 
				     const Frame& frame, const unsigned short* frame_data) const
{
  Xtc& frameXtc = *new((char*)xtc) Xtc(TypeId::Id_Frame, _xtc.src);
  Frame* fp;
  if (_config->forwarding(Camera::FrameFexConfigV1::FullFrame))
    fp=new(frameXtc.alloc(sizeof(Frame))) Frame(frame.width(), frame.height(), 
						frame.depth(), frame.offset(),
						frame_data);
  else
    fp=new(frameXtc.alloc(sizeof(Frame))) Frame (_config->roiBegin().column,
						 _config->roiEnd  ().column,
						 _config->roiBegin().row,
						 _config->roiEnd  ().row,
						 frame.width(), frame.height(), 
						 frame.depth(), frame.offset(),
						 frame_data);
  frameXtc.extent += fp->data_size();
  return frameXtc.extent;
}

//
//  Zero copy helper functions
//
int FexFrameServer::_queue_frame( const Frame& frame, 
				  FrameServerMsg* fmsg,
				  ZcpFragment& zfo )
{
  int frame_extent = frame.data_size();
  _xtc.contains = TypeId::Id_Frame;
  _xtc.extent = sizeof(Xtc) + sizeof(Frame) + frame_extent;

  try {
    int err;
    if ((err=zfo.uinsert( &_xtc, sizeof(Xtc))) != sizeof(Xtc)) throw err;
    
    if ((err=zfo.uinsert( &frame, sizeof(Frame))) != sizeof(Frame)) throw err;
    
    int length;
    if ((length = zfo.kinsert( _splice.fd(), frame_extent)) < 0) throw length;
    if (length != frame_extent) { // fragmentation
      _more   = true;
      _offset = 0;
      fmsg->type   = FrameServerMsg::Fragment;
      fmsg->offset = length + sizeof(Frame) + sizeof(Xtc);
      fmsg->extent = _xtc.extent;
      fmsg->connect(reinterpret_cast<FrameServerMsg*>(&_msg_queue));
      ::write(_fd[1],&fmsg,sizeof(fmsg));
    }
    else
      delete fmsg;

    return length + sizeof(Frame) + sizeof(Xtc);
  }
  catch (int err) {
    printf("FexFrameServer::_queue_frame error: %s : %d\n",strerror(errno),err);
    delete fmsg;
    return -1;
  }
}

TwoDMoments FexFrameServer::_feature_extract(const Frame&          frame,
					     const unsigned short* frame_data) const
{
  //
  // perform the feature extraction here
  //
  switch(_config->processing()) {
  case Camera::FrameFexConfigV1::None:
    return TwoDMoments();
  case Camera::FrameFexConfigV1::GssFullFrame:
    return TwoDMoments(frame.width(), frame.height(), 
		       frame.offset(), frame_data);
  case Camera::FrameFexConfigV1::GssRegionOfInterest:
    return TwoDMoments(frame.width(),
		       _config->roiBegin().column,
		       _config->roiEnd  ().column,
		       _config->roiBegin().row,
		       _config->roiEnd  ().row,
		       frame.offset(),
		       frame_data);
  case Camera::FrameFexConfigV1::GssThreshold:
    return TwoDMoments(frame.width(),
		       frame.height(),
		       _config->threshold(),
		       frame.offset(),
		       frame_data);
  }
  return TwoDMoments();
}

int FexFrameServer::_queue_fex( const TwoDMoments& moments,
				FrameServerMsg* fmsg,
				ZcpFragment& zfo )
{	
  delete fmsg;

  _xtc.contains = TypeId::Id_TwoDGaussian;
  _xtc.extent   = sizeof(Xtc) + sizeof(TwoDGaussian);

  TwoDGaussian payload(moments);

  try {
    int err;
    if ((err=zfo.uinsert( &_xtc, sizeof(Xtc))) < 0) throw err;
    
    if ((err=zfo.uinsert( &payload, sizeof(payload))) < 0) throw err;

    return _xtc.extent;
  }
  catch (int err) {
    printf("FexFrameServer::_queue_frame error: %s : %d\n",strerror(errno),err);
    return -1;
  }
}

int FexFrameServer::_queue_fex_and_frame( const TwoDMoments& moments,
					  const Frame& frame,
					  FrameServerMsg* fmsg,
					  ZcpFragment& zfo )
{	

  Xtc fexxtc(TypeId::Id_TwoDGaussian, _xtc.src);
  fexxtc.extent += sizeof(TwoDGaussian);

  Xtc frmxtc(TypeId::Id_Frame, _xtc.src);
  frmxtc.extent += sizeof(Frame);
  frmxtc.extent += frame.data_size();

  _xtc.contains = TypeId::Id_Xtc;
  _xtc.extent   = sizeof(Xtc) + fexxtc.extent + frmxtc.extent;

  TwoDGaussian fex(moments);

  try {
    int err;
    if ((err=zfo.uinsert( &_xtc  , sizeof(Xtc  ))) < 0) throw err;

    if ((err=zfo.uinsert( &fexxtc, sizeof(Xtc  ))) < 0) throw err;
    if ((err=zfo.uinsert( &fex   , sizeof(fex  ))) < 0) throw err;
    
    if ((err=zfo.uinsert( &frmxtc, sizeof(Xtc  ))) < 0) throw err;
    if ((err=zfo.uinsert( &frame , sizeof(Frame))) < 0) throw err;

    int length;
    int frame_extent = frame.data_size();
    if ((length = zfo.kinsert( _splice.fd(), frame_extent)) < 0) 
      throw length;

    if (length != frame_extent) { // fragmentation
      _more   = true;
      _offset = 0;
      fmsg->type   = FrameServerMsg::Fragment;
      fmsg->offset = _xtc.extent - frame_extent + length;
      fmsg->extent = _xtc.extent;
      fmsg->connect(reinterpret_cast<FrameServerMsg*>(&_msg_queue));
      ::write(_fd[1],&fmsg,sizeof(fmsg));
    }
    else
      delete fmsg;

    return _xtc.extent + length - frame_extent;
  }
  catch (int err) {
    printf("FexFrameServer::_queue_frame error: %s : %d\n",strerror(errno),err);
    delete fmsg;
    return -1;
  }
}
