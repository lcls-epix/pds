#include "pds/tprds/Module.hh"

#include <stdio.h>

using namespace Pds::TprDS;

void TprBase::dump() const {
  static const unsigned NChan=12;
  printf("irqEnable [%p]: %08x\n",&irqEnable,irqEnable);
  printf("irqStatus [%p]: %08x\n",&irqStatus,irqStatus);
  printf("gtxDebug  [%p]: %08x\n",&gtxDebug  ,gtxDebug);
  printf("trigSel   [%p]: %08x\n",&dmaFullThr,dmaFullThr);
  printf("channel0  [%p]\n",&channel[0].control);
  printf("control : ");
  for(unsigned i=0; i<NChan; i++)      printf("%08x ",channel[i].control);
  printf("\nevtCount: ");
  for(unsigned i=0; i<NChan; i++)      printf("%08x ",channel[i].evtCount);
  printf("\nevtSel  : ");
  for(unsigned i=0; i<NChan; i++)      printf("%08x ",channel[i].evtSel);
  printf("\ntestData: ");
  for(unsigned i=0; i<NChan; i++)      printf("%08x ",channel[i].testData);
  printf("\nframeCnt: %08x\n",frameCount);
  printf("pauseCnt: %08x\n",pauseCount);
  printf("ovfloCnt: %08x\n",overflowCount);
  printf("idleCnt : %08x\n",idleCount);
}

void TprBase::setupRate   (unsigned i,
                           unsigned rate,
                           unsigned dataLength) 
{
  channel[i].evtSel   = (1<<30) | (0<<11) | rate; // 
  channel[i].testData = dataLength;
  channel[i].control  = 5;
}

void TprBase::setupDaq    (unsigned i,
                           unsigned partition,
                           unsigned dataLength) 
{
  channel[i].evtSel   = (1<<30) | (3<<11) | partition; // 
  channel[i].testData = dataLength;
  channel[i].control  = 5;
}
