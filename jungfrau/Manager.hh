#ifndef Pds_Jungfrau_Manager_hh
#define Pds_Jungfrau_Manager_hh

namespace Pds {
  class Appliance;
  class CfgClientNfs;
  class Fsm;
  namespace Jungfrau {
    class Detector;
    class Server;
    class Manager {
    public:
      Manager (Detector&, Server&, CfgClientNfs&);
      ~Manager();
    public:
      Appliance& appliance();
    private:
      Fsm& _fsm;
    };
  }
}

#endif
