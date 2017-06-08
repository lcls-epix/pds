#ifndef Pds_Zyla_Manager_hh
#define Pds_Zyla_Manager_hh

namespace Pds {
  class Appliance;
  class CfgClientNfs;
  class Fsm;
  namespace Zyla {
    class Driver;
    class Server;
    class Manager {
    public:
      Manager (Driver&, Server&, CfgClientNfs&, bool);
      ~Manager();
    public:
      Appliance& appliance();
    private:
      Fsm& _fsm;
    };
  }
}

#endif
