#ifndef PDSIPIMBMANAGER_HH
#define PDSIPIMBMANAGER_HH

namespace Pds {

  class Fsm;
  class Appliance;
  class IpimbServer;
  class CfgClientNfs;
  class IpimBoard;
  class IpimbFex;

  class IpimbManager {
  public:
    IpimbManager(IpimbServer* server[], unsigned nServers, CfgClientNfs** cfg, char* portName[16], const int baselineMode, const int polarities[16], IpimbFex&);
    Appliance& appliance();
  private:
    Fsm& _fsm;
    static const char* _calibPath;
    unsigned _nServers;
    IpimbServer* _server[];
  };
}
#endif
