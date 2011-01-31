#ifndef PDS_ENCODERSERVER
#define PDS_ENCODERSERVER

#include "pds/utility/EbServer.hh"
#include "pds/utility/EbCountSrv.hh"
#include "pds/utility/EbEventKey.hh"
#include "pds/config/CsPadConfigType.hh"
#include "pdsdata/cspad/ElementHeader.hh"
#include "pds/csPad/CspadConfigurator.hh"
#include "pdsdata/xtc/Xtc.hh"
#include <fcntl.h>

namespace Pds
{
   class CspadServer;
}

class Pds::CspadServer
   : public EbServer,
     public EbCountSrv
{
 public:
   CspadServer( const Src& client, unsigned configMask=0 );
   virtual ~CspadServer() {}
    
   //  Eb interface
   void       dump ( int detail ) const {}
   bool       isValued( void ) const    { return true; }
   const Src& client( void ) const      { return _xtc.src; }

   //  EbSegment interface
   const Xtc& xtc( void ) const    { return _xtc; }
   unsigned   offset( void ) const;
   unsigned   length() const       { return _xtc.extent; }

   //  Eb-key interface
   EbServerDeclare;

   //  Server interface
   int pend( int flag = 0 ) { return -1; }
   int fetch( ZcpFragment& , int flags ) { return 0; }
   int fetch( char* payload, int flags );
   bool more() const;

   unsigned count() const;
   void setCspad( int fd );

   unsigned configure(CsPadConfigType*);
   unsigned unconfigure(void);

   unsigned payloadSize(void)   { return _payloadSize; }
   unsigned numberOfQuads(void) { return _quads; }
   unsigned flushInputQueue(int);
   void     enable();
   void     disable();
   void     die();
   void     debug(unsigned d) { _debug = d; }
   unsigned debug() { return _debug; }
   void     offset(unsigned c) { _offset = c; }
   unsigned offset() { return _offset; }
   void     resetOffset() { _offset = 0; _count = 0xffffffff; }
   unsigned myCount() { return _count; }
   void     dumpFrontEnd();

 public:
   static CspadServer* instance() { return _instance; }

 private:
   static CspadServer*            _instance;
   static void instance(CspadServer* s) { _instance = s; }

 private:
   Xtc                            _xtc;
   Pds::CsPad::CspadConfigurator* _cnfgrtr;
   unsigned                       _quads;
   unsigned                       _count;
   unsigned                       _quadsThisCount;
   unsigned                       _payloadSize;
   unsigned                       _configMask;
   unsigned                       _configureResult;
   unsigned                       _debug;
   unsigned                       _offset;
   bool                           _configured;
};

#endif