#ifndef PDS_TOEB
#define PDS_TOEB

//
//  class ToEb
//
//  This class is used by an appliance stream outlet to send datagrams
//  directly to the event builder of another appliance stream through 
//  the use of a unix pipe.  It differs from other outlet clients in
//  that it does not reproduce the Xtc from the datagram into the payload.
//  This allows contributions to the first stream's event builder to
//  appear at the same level as contribution's to the second stream's
//  event builder.
//

#include "EbServer.hh"
#include "EbSequenceSrv.hh"
#include "EbEventKey.hh"

#include "pds/xtc/Datagram.hh"

namespace Pds {

  class CDatagram;

  class ToEb : public EbServer, public EbSequenceSrv {
  public:
    ToEb(const Src& client);
    virtual ~ToEb() {}
    
    int  send(const CDatagram*  );
  public:
    //  Eb interface
    void        dump    (int detail)   const;
    bool        isValued()             const;
    const Src&  client  ()             const;
    //  EbSegment interface
    const Xtc&   xtc   () const;
    bool           more  () const { return _more; }
    unsigned       length() const { return _datagram.xtc.sizeofPayload(); }
    unsigned       offset() const { return _offset; }
  public:
    //  Eb-key interface
    EbServerDeclare;
  public:
    //  Server interface
    int      pend        (int flag = 0);
    int      fetch       (char* payload, int flags);
  public:
    const Sequence& sequence() const;
    const Env&      env()      const;
  private:
    int      _pipefd[2];
    Src      _client;
    Datagram _datagram;
    bool     _more;
    unsigned _offset;
    unsigned _next;
  };
}
#endif
