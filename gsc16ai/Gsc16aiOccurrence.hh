#ifndef __GSC16AIOCCURRENCE_HH
#define __GSC16AIOCCURRENCE_HH

namespace Pds {
   class Gsc16aiOccurrence;
   class Gsc16aiManager;
   class GenericPool;
}

class Pds::Gsc16aiOccurrence {
  public:
    Gsc16aiOccurrence(Gsc16aiManager *mgr);
    ~Gsc16aiOccurrence();
    void outOfOrder(void);
    void userMessage(const char *msgText);

  private:
    Gsc16aiManager* _mgr;
    GenericPool* _outOfOrderPool;
    GenericPool* _userMessagePool;
};

#endif
