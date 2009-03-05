#include "pds/mon/MonStats1D.hh"

#include <math.h>

using namespace Pds;

MonStats1D::MonStats1D() {}
MonStats1D::~MonStats1D() {}

void MonStats1D::stats(unsigned nbins, 
		       float xlo, float xup, 
		       const double* con)
{
  _sumw   = 0; 
  _sumw2  = 0; 
  _sumwx  = 0; 
  _sumwx2 = 0;
  _under  = *(con+nbins);
  _over   = *(con+nbins+1);
  const double* end = con+nbins;
  float dx = (xup-xlo)/nbins;
  float x  = xlo+0.5*dx;
  do {
    double w = *con++;
    _sumw   += w;
    _sumw2  += w*w;
    _sumwx  += w*x;
    _sumwx2 += w*x*x;
    x += dx;
  } while (con < end);
}

double MonStats1D::sum() const 
{
  return _sumw;
}

double MonStats1D::mean() const
{
  return _sumw ? _sumwx/_sumw : 0;
}
 
double MonStats1D::rms() const
{
  if (_sumw) {
    double mean = _sumwx/_sumw;
    return sqrt(_sumwx2/_sumw-mean*mean);
  } else {
    return 0;
  }
}

double MonStats1D::under() const 
{
  return _under;
}

double MonStats1D::over() const
{
  return _over;
}