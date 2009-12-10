/*
 * ProxyMsg.hh
 *
 *  Created on: Sep 5, 2009
 *      Author: jackp
 */

#ifndef PROXYMSG_HH_
#define PROXYMSG_HH_

#include <netinet/in.h>
#include <arpa/inet.h>
#include "pdsdata/xtc/Src.hh"
#include "pdsdata/xtc/Damage.hh"

struct mcaddress {
    in_addr_t mcaddr;
    uint32_t  mcport;
};

namespace RceFBld {

  class ProxyMsg {
    public:
      ProxyMsg() : numberOfEventLevels(0) {};
      ~ProxyMsg() {};

    public:
      enum {MaxEventLevelServers=64, ProxyPort=5000};
      bool        byteOrderIsBigEndian;
      uint32_t    numberOfEventLevels;
      mcaddress   mcAddrs[MaxEventLevelServers];
      mcaddress   evrMcAddr;
      uint32_t    payloadSizePerLink;
      uint32_t    numberOfLinks;
      Pds::Src    procInfoSrc;
      Pds::Src    detInfoSrc;
      Pds::TypeId contains;
  };

  class ProxyReplyMsg {

    public:
      ProxyReplyMsg() : damage(Pds::Damage(0)) {};
      ~ProxyReplyMsg() {};

    public:
      Pds::Damage   damage;
  };

}

#endif /* PROXYMSG_HH_ */
