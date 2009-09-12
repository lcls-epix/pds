#ifndef Pds_ControlEb_hh
#define Pds_ControlEb_hh

#include "pds/service/Timer.hh"

#include "pds/service/BitMaskArray.hh"
#include "pds/service/GenericPool.hh"
#include "pds/utility/Transition.hh"

namespace Pds {

  class Node;
  class Transition;

  class ControlEb : public Timer {
  public:
    ControlEb (const Node&, Routine* =0);
    ~ControlEb();
  public:
    void        reset(const Allocation& alloc);
    Transition* build(const Node&, 
		      const Transition&);
    Allocation  remaining() const;
  public:
    void expired();
    Task* task();
    unsigned duration() const;
    unsigned repetitive() const;
  private:
    const Node&       _hdr;
    Routine*          _timeout;
    Task*             _task;
    const Allocation* _alloc;
    GenericPool       _buffer;
    Transition*       _pending;
    enum { MAX_CONTRIBUTORS=128 };
    BitMaskArray<(MAX_CONTRIBUTORS>>5)> _remaining;
  };

};

#endif
    