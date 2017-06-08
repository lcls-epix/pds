#ifndef Pds_Zyla_Server_hh
#define Pds_Zyla_Server_hh

#include "pds/utility/EbServer.hh"
#include "pds/utility/EbCountSrv.hh"
#include "pds/utility/EbEventKey.hh"

namespace Pds {
  namespace Zyla {
    class Server : public EbServer,
       public EbCountSrv {

    public:
      Server( const Src& client );
      virtual ~Server() {}

      //  Eb interface
      void       dump ( int detail ) const {}
      bool       isValued( void ) const    { return true; }
      const Src& client( void ) const      { return _xtc.src; }

      //  EbSegment interface
      const Xtc& xtc( void ) const    { return _xtc; }
      unsigned   offset( void ) const { return sizeof(Xtc); }
      unsigned   length() const       { return _xtc.extent; }

      //  Eb-key interface
      EbServerDeclare;

      //  Server interface
      int pend( int flag = 0 ) { return -1; }
      int fetch( char* payload, int flags );

      unsigned count() const;
      void resetCount();

      void post(char*, unsigned);

      void set_frame_sz(size_t);

    private:
      Xtc _xtc;
      unsigned  _count;
      unsigned  _framesz;
      uint64_t  _last_frame;
      int _pfd[2];
    };
  }
}

#endif
