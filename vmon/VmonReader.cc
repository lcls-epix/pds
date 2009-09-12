#include "VmonReader.hh"

#include "pds/vmon/VmonRecord.hh"
#include "pds/mon/MonGroup.hh"
#include "pds/mon/MonEntry.hh"
#include "pds/mon/MonDescEntry.hh"
#include "pds/mon/MonUsage.hh"
#include "pds/mon/MonStats1D.hh"
#include "pds/mon/MonStats2D.hh"

using namespace Pds;

VmonReader::VmonReader(const char* name) :
  _buff(new char[0x10000])
{
  _file = fopen(name,"r");

  fread(_buff,sizeof(VmonRecord),1,_file);
  VmonRecord* record = new (_buff) VmonRecord;
  unsigned size = record->len()-sizeof(VmonRecord);
  fread(record+1,size,1,_file);

  _seek_pos = record->len();
  _len = record->extract(_src, _cds, _offsets);

  fread(_buff,sizeof(VmonRecord),1,_file);
  _begin = record->time();

  fseek(_file, -_len, SEEK_END);
  fread(_buff,sizeof(VmonRecord),1,_file);
  _end = record->time();
}
  
VmonReader::~VmonReader()
{
  reset();

  for(vector<MonCds*>::iterator it = _cds.begin(); it!=_cds.end(); it++)
    delete *it;

  for(vector<int*>::iterator it = _offsets.begin(); it!=_offsets.end(); it++)
    delete[] *it;

  for(vector<int*>::iterator it = _req_off.begin(); it!=_req_off.end(); it++)
    delete[] *it;
}

const vector<Src>& VmonReader::sources() const { return _src; }

const MonCds* VmonReader::cds(const Src& src) const
{
  int i=0;
  for(vector<Src>::const_iterator it=_src.begin(); it!=_src.end(); it++, i++)
    if (src == *it) 
      return _cds[i];
  return 0;
}

const ClockTime& VmonReader::begin() const { return _begin; }
const ClockTime& VmonReader::end  () const { return _end  ; }

void VmonReader::reset()
{
  for(vector<int*>::iterator it=_req_off.begin(); it!=_req_off.end(); it++)
    delete *it;
  _req_off.clear();
  _req_use.clear();
  _req_src.clear();
}

void VmonReader::use(const Src& src, const MonUsage& usage)
{
  if (!usage.used()) return;

  int i=0;
  for(vector<Src>::iterator it=_src.begin(); it!=_src.end(); it++, i++) {
    if (*it == src) {
      const MonCds* cds = _cds[i];
      int* off = new int[usage.used()];
      for(unsigned short u=0; u<usage.used(); u++) {
	int s = usage.signature(u);
	int n=0;
	for(unsigned short g = 0; g < (s>>16); g++)  
	  n += cds->group(g)->nentries();
	n += s & 0xffff;
	off[u] = (_offsets[i])[n];
      } 
    
      _req_src.push_back(src);
      _req_use.push_back(&usage);
      _req_off.push_back(off);
    }
  }
}

void VmonReader::process(VmonReaderCallback& callback)
{
  ClockTime begin(0,0);
  ClockTime end  (-1UL,-1UL);
  process(callback,begin,end);
}

void VmonReader::process(VmonReaderCallback& callback,
			 const ClockTime& begin,
			 const ClockTime& end)
{
  fseek(_file, _seek_pos, SEEK_SET);
  while( !feof(_file) ) {
    fread(_buff, sizeof(VmonRecord), 1, _file);
    VmonRecord& record = *new(_buff) VmonRecord;
    int remaining = record.len() - sizeof(record);
    if (record.time() > end)
      break;
    if (begin > record.time()) {
      fseek(_file, remaining, SEEK_CUR);
      continue;
    }
    fread(&record+1, remaining, 1, _file);
    int i=0;
    for(vector<Src>::iterator it=_req_src.begin(); it!=_req_src.end(); it++,i++) {
      const MonCds& cds = *_cds[i];
      const MonUsage& usage = *_req_use[i];
      for(unsigned short u = 0; u < usage.used(); u++)
	switch(cds.entry(usage.signature(u))->desc().type()) {
	case MonDescEntry::TH1F:
	  callback.process(record.time(), *it, usage.signature(u), 
			   *reinterpret_cast<const MonStats1D*>(_buff + _req_off[i][u]));
	  break;
	case MonDescEntry::TH2F:
	  callback.process(record.time(), *it, usage.signature(u), 
			   *reinterpret_cast<const MonStats2D*>(_buff + _req_off[i][u]));
	  break;
	default:
	  break;
	}
    }
  }
}