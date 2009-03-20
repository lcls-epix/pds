#ifndef PDSTRANSITION_HH
#define PDSTRANSITION_HH

#include "pds/collection/Message.hh"
#include "pds/collection/Node.hh"
#include "pds/service/Pool.hh"
#include "pdsdata/xtc/Sequence.hh"
#include "pdsdata/xtc/TransitionId.hh"

namespace Pds {

  class Transition : public Message {
  public:
    enum Phase { Execute, Record };
    static const char* name(TransitionId::Value id);

  public:
    Transition(TransitionId::Value id,
	       Phase           phase,
	       const Sequence& sequence,
	       unsigned        env, 
	       unsigned        size=sizeof(Transition));

    Transition(TransitionId::Value id,
	       unsigned        env, 
	       unsigned        size=sizeof(Transition));
    Transition(const Transition&);

    TransitionId::Value id      () const;
    Phase           phase   () const;
    const Sequence& sequence() const;
    unsigned        env     () const;

    void* operator new(size_t size, Pool* pool);
    void* operator new(size_t size);
    void  operator delete(void* buffer);

  private:
    TransitionId::Value _id;
    Phase        _phase;
    Sequence     _sequence;
    unsigned     _env;
  };

  class Allocation {
  public:
    Allocation();
    Allocation(const char* partition,
	       const char* dbpath,
	       unsigned    partitionid);
    Allocation(const char* partition,
	       const char* dbpath,
	       unsigned    partitionid,
	       const Sequence&);

    bool add(const Node& node);

    unsigned    nnodes() const;
    const Node* node(unsigned n) const;
    const char* partition() const;
    const char* dbpath() const;
    unsigned    partitionid() const;

    unsigned    size() const;
  private:
    static const unsigned MaxNodes=128;
    static const unsigned MaxName=64;
    static const unsigned MaxDbPath=64;
    char _partition[MaxName];
    char _dbpath   [MaxDbPath];
    unsigned _partitionid;
    unsigned _nnodes;
    Node _nodes[MaxNodes];
  };

  class Allocate : public Transition {
  public:
    Allocate(const Allocation&);
    Allocate(const Allocation&,
	     const Sequence&);
  public:
    const Allocation& allocation() const;
  private:
    Allocation _allocation;
  };

  class Kill : public Transition {
  public:
    Kill(const Node& allocator);

    const Node& allocator() const;
  
  private:
    Node _allocator;
  };

}

#endif
