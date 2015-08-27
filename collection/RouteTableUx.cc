#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <net/if.h>

#ifdef __linux__
#include <sys/ioctl.h>
#else
#include <sys/sockio.h>
#include <net/if_arp.h>
#endif

#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#include <net/route.h>

#include "RouteTable.hh"

using namespace Pds;

#define TABLSIZ 4096

#pragma pack(2)
// Structure for sending the request
typedef struct
{
  struct nlmsghdr nlMsgHdr;
  struct rtmsg rtMsg;
  char buf[TABLSIZ];
} route_request;

RouteTable::RouteTable() :
  _found(0)
{
  int fd;  
  if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) return;

  struct ifreq ifrarray[MaxRoutes];
  memset(ifrarray, 0, sizeof(ifreq)*MaxRoutes);

  struct ifconf ifc;
  ifc.ifc_len = MaxRoutes * sizeof(struct ifreq);
  ifc.ifc_buf = (char*)ifrarray;

  if (ioctl(fd, SIOCGIFCONF, &ifc) < 0) {
    printf("*** RouteTable unable to config interfaces: too many? "
	   "(current limit is %d)\n", MaxRoutes);
    close(fd);
    return;
  }
  
  for(unsigned i=0; i<MaxRoutes; i++) {
    struct ifreq* ifr = ifrarray+i;
    if (!ifr || !(((sockaddr_in&)ifr->ifr_addr).sin_addr.s_addr))
      break;

    if (ioctl (fd, SIOCGIFINDEX, ifr) < 0) {
      perror("Failed to fetch ifIndex");
    }
  }

  memset(_name,    0, sizeof(char)*MaxRoutes*MaxNameLen);
  memset(_dst  ,   0, sizeof(int)*MaxRoutes);
  memset(_iface,   0, sizeof(int)*MaxRoutes);
  memset(_netma,   0, sizeof(int)*MaxRoutes);
  memset(_bcast,   0, sizeof(int)*MaxRoutes);

  int route_sock = ::socket(AF_NETLINK, SOCK_DGRAM, NETLINK_ROUTE);
  if (route_sock < 0) {
    perror("Failed to open socket");
    exit(1);
  }

  int retValue;
  route_request *request = (route_request *)malloc(sizeof(route_request));
  memset(request, 0, sizeof(*request));

  // Fill in the NETLINK header
  request->nlMsgHdr.nlmsg_len = NLMSG_LENGTH(sizeof(struct rtmsg));
  request->nlMsgHdr.nlmsg_type = RTM_GETROUTE;
  request->nlMsgHdr.nlmsg_flags = NLM_F_REQUEST | NLM_F_DUMP;
  
  // set the routing message header
  request->rtMsg.rtm_family = AF_INET;
  request->rtMsg.rtm_table = 254;
  
  // Send routing request
  if ((retValue = send(route_sock, request, sizeof(route_request), 0)) < 0) {
    perror("send");
    exit(1);
  }

  int reply_len = 0;
  size_t counter = TABLSIZ;
  char reply_ptr[TABLSIZ];
  struct nlmsghdr *nlp;
  char* buf = reply_ptr;
  unsigned long bufsize ;

  for(;;) {
    if( counter < sizeof( struct nlmsghdr)) {
      printf("Routing table is bigger than msgsiz %zu/%zu\n",
             counter,sizeof(struct nlmsghdr));
      exit(1);
    }

    int nbytes = ::recv(route_sock, &reply_ptr[reply_len], counter, 0);

    if(nbytes < 0 ) {
      printf("Error in recv\n");
      break;
    }

    if(nbytes == 0)
      printf("EOF in netlink\n");

    nlp = (struct nlmsghdr*)(&reply_ptr[reply_len]);

    if (nlp->nlmsg_type == NLMSG_DONE) {
      // All data has been received.
      // Truncate the reply to exclude this message,
      // i.e. do not increase reply_len.
      break;
    }

    if (nlp->nlmsg_type == NLMSG_ERROR)  {
      printf("Error in msg\n");
      exit(1);
    }

    reply_len += nbytes;
    counter -= nbytes;
  }  
  
  nlp = (struct nlmsghdr *) buf;
  bufsize = reply_len;

  for(; NLMSG_OK(nlp, bufsize); nlp=NLMSG_NEXT(nlp, bufsize)) {
    // get route entry header
    struct rtmsg *rtp = (struct rtmsg *) NLMSG_DATA(nlp);
    // we are only concerned about the
    // tableId route table
    if(rtp->rtm_table != 254)
      continue;

    int dev=-1, dst=0, netma=0, prefsrc=0;

    // inner loop: loop thru all the attributes of
    // one route entry
    struct rtattr *rtap = (struct rtattr *) RTM_RTA(rtp);
    int rtl = RTM_PAYLOAD(nlp);
    for( ; RTA_OK(rtap, rtl); rtap = RTA_NEXT(rtap, rtl))
     {
       switch(rtap->rta_type)
         {
           // destination IPv4 address
         case RTA_DST:
           { int count = 32 - rtp->rtm_dst_len;
             dst = ntohl(*(int *) RTA_DATA(rtap));

             unsigned mask = 0xffffffff;
             for (; count!=0 ;count--)
               mask = mask << 1;
             netma = mask;
           } break;
         case RTA_GATEWAY:
           { // in_addr gateWay;
           // gateWay.s_addr = *(unsigned long *) RTA_DATA(rtap);
             // printf("gw:%s\t",inet_ntoa(gateWay));
           } break;
         case RTA_PREFSRC:
           { // in_addr srcAddr;
             // srcAddr.s_addr = *(unsigned long *) RTA_DATA(rtap);
             // printf("src:%s\t", inet_ntoa(srcAddr));
             prefsrc=1;
           } break;
           // unique ID associated with the network
           // interface
         case RTA_OIF:
           { // char ifName[64];
             //             ifname(*((int *) RTA_DATA(rtap)),ifName);
             //             printf( "ifname %s\n", ifName);
             dev = *((int *) RTA_DATA(rtap));
             // printf("dev: %x\n",*((int *) RTA_DATA(rtap)));
           } break;
         default:
           break;
         }

     }

    //
    //  Require an interface index and destination IP address for this route
    //
    if (dev >= 0 && prefsrc) {
      for(unsigned i=0; i<MaxRoutes; i++) {
        struct ifreq* ifr = ifrarray+i;
        if (!ifr || !(((sockaddr_in&)ifr->ifr_addr).sin_addr.s_addr)) {
          printf("*** RouteTable failed to find device %d/%i\n",dev,i);
          break;
        }

        if (ifr->ifr_ifindex != dev)
          continue;

        strncpy(_name+_found*MaxNameLen, ifr->ifr_name, MaxNameLen-1);

        struct ifreq ifreq_flags = *ifr;
        if (ioctl(fd, SIOCGIFFLAGS, &ifreq_flags) < 0) {
          printf("*** RouteTable error getting flags for '%s': %s\n",
                 name(_found), strerror(errno));
        }
        int flags = ifreq_flags.ifr_flags;
      
        if ((flags & IFF_UP) && (flags & IFF_BROADCAST)) {
          struct ifreq ifreq_hwaddr = *ifr;
          if (ioctl(fd, SIOCGIFHWADDR, &ifreq_hwaddr) < 0)  {
            printf("*** RouteTable error getting hwaddr for '%s': %s\n",
                   name(_found), strerror(errno));
          }

          _iface[_found] = ntohl((((sockaddr_in&)ifr->ifr_addr).sin_addr.s_addr));
          _dst  [_found] = dst;
          _netma[_found] = netma;
          _bcast[_found] = dst | ~_netma[_found];
          _ether[_found] = Ether((unsigned char*)ifreq_hwaddr.ifr_hwaddr.sa_data);

          ++_found;
          break;
        }
      }      
    }
  }

  free(request);
  close(route_sock);
  close(fd);
}

