#include "pds/client/L3TIterator.hh"
#include "pds/xtc/SummaryDg.hh"

#include <stdio.h>

using namespace Pds;

int L3TIterator::process(Xtc* xtc)
{
  if (xtc->contains.id()==TypeId::Id_Xtc) {
    iterate(xtc);
    return 1;
  }
  if (xtc->contains.value()==SummaryDg::Xtc::typeId().value()) {
    const SummaryDg::Xtc& s = *static_cast<SummaryDg::Xtc*>(xtc);
    _found = (s.l3tresult()!=L1AcceptEnv::None);
    _pass  = (s.l3tresult()==L1AcceptEnv::Pass);
  }
  return 0;
}
