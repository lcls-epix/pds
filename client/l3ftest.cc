#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <dlfcn.h>
#include <errno.h>

#include "pds/client/L3FilterDriver.hh"
#include "pds/client/L3FilterThreads.hh"
#include "pds/utility/Inlet.hh"
#include "pds/utility/Outlet.hh"
#include "pds/utility/OpenOutlet.hh"
#include "pds/service/GenericPoolW.hh"
#include "pds/xtc/CDatagram.hh"
#include "pdsdata/app/L3FilterModule.hh"
#include "pdsdata/xtc/XtcFileIterator.hh"
#include "pdsdata/xtc/ProcInfo.hh"
#include "pdsdata/xtc/L1AcceptEnv.hh"

namespace Pds {
  class MyOutlet : public Appliance {
  public:
    MyOutlet(Semaphore& sem) : _sem(sem), _pass(0), _fail(0) {}
  public:
    Transition* transitions(Transition* in) { return 0; }
    InDatagram* events     (InDatagram* in) {
      CDatagram* cdg = reinterpret_cast<CDatagram*>(in);
      Datagram* dg = &cdg->datagram();
      if (dg->seq.service()==TransitionId::L1Accept) {
        L1AcceptEnv* env = reinterpret_cast<L1AcceptEnv*>(&dg->env);
        if (env->l3t_result()==L1AcceptEnv::Pass)
          _pass++;
        if (env->l3t_result()==L1AcceptEnv::Fail)
          _fail++;
      }
      else
        printf("%s transition: time %08x/%08x  stamp %08x/%08x, dmg %08x, payloadSize 0x%x  pass:fail=%d:%d\n",
               TransitionId::name(dg->seq.service()),
               dg->seq.clock().seconds(),dg->seq.clock().nanoseconds(),
               dg->seq.stamp().fiducials(),dg->seq.stamp().ticks(),
               dg->xtc.damage.value(),
               dg->xtc.sizeofPayload(),
               _pass,_fail);
      _sem.give();
      return in;
    }
  private:
    Semaphore& _sem;
    unsigned _pass,_fail;
  };
};

using namespace Pds;

static void load_filter(char*       arg,
                        Appliance*& apps)
{
  {
    std::string p(arg);

    unsigned n=0;
    size_t posn = p.find(",");
    if (posn!=std::string::npos) {
      n = strtoul(p.substr(posn+1).c_str(),NULL,0);
      p = p.substr(0,posn);
    }
        
    printf("dlopen %s\n",p.c_str());

    void* handle = dlopen(p.c_str(), RTLD_LAZY);
    if (!handle) {
      printf("dlopen failed : %s\n",dlerror());
    }
    else {
      // reset errors
      const char* dlsym_error;
      dlerror();

      // load the symbols
      create_m* c_user = (create_m*) dlsym(handle, "create");
      if ((dlsym_error = dlerror())) {
        fprintf(stderr,"Cannot load symbol create: %s\n",dlsym_error);
      }
      else {
        L3FilterThreads* driver = new L3FilterThreads(c_user, n);
        if (apps != NULL)
          driver->connect(apps);
        else
          apps = driver;
      }
    }
  }
}

void usage(char* progname) {
  fprintf(stderr,"Usage: %s -x <xtcname> [-f <l3fname>] [-F <filter>] [-d <damage mask>] [-h]\n", progname);
}

int main(int argc, char* argv[]) {
  int c;
  char* xtcname=0;
  char* l3fname=0;
  int parseErr = 0;
  unsigned damage = -1;
  Appliance*  apps = 0;
  char *endPtr;
  extern char* optarg;

  while ((c = getopt(argc, argv, "hf:F:x:d:")) != -1) {
    switch (c) {
    case 'h':
      usage(argv[0]);
      exit(0);
    case 'x':
      xtcname = optarg;
      break;
    case 'f':
      l3fname = optarg;
      break;
    case 'F':
      load_filter(optarg,apps);
      break;
    case 'd':
      errno = 0;
      endPtr = NULL;
      damage = strtoul(optarg, &endPtr, 0);
      if (errno || (endPtr == NULL) || (*endPtr != '\0')) {
        printf("Error: failed to parse damage mask\n");
        damage = -1;
      }
      break;
    default:
      parseErr++;
    }
  }
  
  if (!xtcname) {
    usage(argv[0]);
    exit(2);
  }

  int fd = open(xtcname,O_RDONLY | O_LARGEFILE);
  if (fd < 0) {
    printf("Unable to open file %s\n",xtcname);
    exit(2);
  }

  GenericPoolW pool(0x900000,4);

  Semaphore sem(Semaphore::EMPTY);

  Inlet* inlet = new Inlet;
  Outlet* outlet = new Outlet;
  new OpenOutlet(*outlet);
  outlet->connect(inlet);
  (new MyOutlet(sem))->connect(inlet);

  if (apps)
    apps->connect(inlet);

  Allocation alloc("l3ftest",
                   "nodb",
                   0);
  alloc.set_l3t(l3fname);

  Allocate* a = new(&pool) Allocate(alloc);
  inlet->post(a);

  XtcFileIterator iter(fd,0x900000);
  Dgram* dg;
  //  unsigned long long bytes=0;
  unsigned events=0;
  //  unsigned damage=0;
  while ((dg = iter.next())) {

    Datagram& ddg = *reinterpret_cast<Datagram*>(dg);
    CDatagram* odg = new(&pool) CDatagram(ddg,ddg.xtc);

    events++;
    inlet->post(odg);
    sem.take();
  }

  close(fd);
  return 0;
}
