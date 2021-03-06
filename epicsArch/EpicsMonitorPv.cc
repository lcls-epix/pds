#include "EpicsMonitorPv.hh"

#define epicsAlarmGLOBAL
#include <alarm.h>
#undef epicsAlarmGLOBAL


namespace Pds
{

  int EpicsMonitorPv::init(int iPvId, 
    const std::string & sPvName, const std::string & sPvDescription,
    float fUpdateInterval, int iNumEventNode)
  {
    release();

    _iPvId            = iPvId;
    _sPvName          = sPvName;
    _sPvDescription   = sPvDescription;
    _fUpdateInterval  = fUpdateInterval;
    _iNumEventNode    = iNumEventNode;
    _tsLastUpdate.tv_sec = _tsLastUpdate.tv_nsec = 0;
    _iCaStatus = ca_create_channel(_sPvName.c_str(), caConnectionHandler, // event handler
           this, _iCaChannelPriority, &_chidPv);
    if (_iCaStatus != ECA_NORMAL)
    {
      printf("EpicsMonitorPv()::init()::ca_create_channel(%s) failed, CA errmsg: %s\n",
        _sPvName.c_str(), ca_message(_iCaStatus));
      return 1;
    }

    _u64MaskEventNode = ( ((uint64_t)1)<<iNumEventNode) - 1;    
    return 0;
  }

  int EpicsMonitorPv::reconnect()
  {
    release();
    _tsLastUpdate.tv_sec = _tsLastUpdate.tv_nsec = 0;
    _iCaStatus = ca_create_channel(_sPvName.c_str(), caConnectionHandler, // event handler
                                   this, _iCaChannelPriority, &_chidPv);
    if (_iCaStatus != ECA_NORMAL)
    {
      printf("EpicsMonitorPv()::init()::ca_create_channel(%s) failed, CA errmsg: %s\n",
        _sPvName.c_str(), ca_message(_iCaStatus));
      return 1;
    }

    _u64MaskEventNode = ( ((uint64_t)1)<<_iNumEventNode) - 1;    
    return 0;
  }

  EpicsMonitorPv::~EpicsMonitorPv()
  {
    release();
  }

  void EpicsMonitorPv::resetUpdates(int iNumEventNode)
  {
    _iNumEventNode    = iNumEventNode;
    _u64MaskEventNode = ( ((uint64_t)1)<<iNumEventNode) - 1;    
  }

  int EpicsMonitorPv::release()
  {
    if (!_bConnected)
      return 0;

    _bConnected = false;
    _iPvId = -1;
    ca_clear_channel(_chidPv);
    _chidPv = NULL;

    free(_pTimeValue);
    _pTimeValue = NULL;

    free(_pCtrlValue);
    _pCtrlValue = NULL;

    _evidCtrl = NULL;
    _evidTime = NULL;

    _bTimeValueUpdated = false;
    _bCtrlValueUpdated = false;
    _bCtrlValueWritten = false;
    _lDbrLastUpdateType = -1;

    return 0;
  }

  int EpicsMonitorPv::onCaChannelConnected()
  {
    // Check if the connction has been established before
    // if yes, it means the connection was lost before, but is automatically
    // recovered by channel access library now
    if (_evidTime != NULL)
    {
      // reset the flag to true to enable the processing
      _bConnected = true;
      return 0;
    }

    /* Get type and array count */
    _ulNumElems = ca_element_count(_chidPv);
    _lDbfType = ca_field_type(_chidPv);

    _lDbrTimeType = dbf_type_to_DBR_TIME(_lDbfType);
    _lDbrCtrlType = dbf_type_to_DBR_CTRL(_lDbfType);
    if (_lDbfType == DBF_STRING)  // string doesn't have ctrl type
      _lDbrCtrlType = DBR_STS_STRING; // use status type instead

    _bConnected = true;
    _bTimeValueUpdated = false;
    _bCtrlValueUpdated = false;
    _lDbrLastUpdateType = -1;

    /* allocate value buffers */
    if (!_pTimeValue)
    {
      /* 
       * the value will be initialzed to all 0's
       */
      _pTimeValue = calloc(1, dbr_size_n(_lDbrTimeType, _ulNumElems));
      if (!_pTimeValue)
      {
        printf("EpicsMonitorPv::onCaChannelConnected()::calloc() failed\n");
        return 1;
      }
    }
    if (!_pCtrlValue)
    {
      /* 
       * the value will be initialzed to all 0's
       */
      _pCtrlValue = calloc(1, dbr_size_n(_lDbrCtrlType, _ulNumElems));
      if (!_pCtrlValue)
      {
        printf("EpicsMonitorPv::onCaChannelConnected()::calloc() failed\n");
        return 1;
      }
    }

    /* install monitors: */
    /*   1. PV with control values */
    const unsigned long ulEventMask = DBE_VALUE | DBE_ALARM;  /* Event mask used */

    _iCaStatus = ca_create_subscription(_lDbrCtrlType,
          _ulNumElems,
          _chidPv,
          ulEventMask,
          caSubscriptionHandler,
          this, &_evidCtrl);
    if (_iCaStatus != ECA_NORMAL)
    {
      free(_pCtrlValue);
      _pCtrlValue = NULL;
      printf("EpicsMonitorPv::onCaChannelConnected()::ca_create_subscription(CTRL) failed, Pv %s CA errmsg: %s\n",
        _sPvName.c_str(), ca_message(_iCaStatus));
      return 2;
    }

    /*   2. PV with time values */
    _iCaStatus = ca_create_subscription(_lDbrTimeType,
          _ulNumElems,
          _chidPv,
          ulEventMask,
          caSubscriptionHandler,
          this, &_evidTime);
    if (_iCaStatus != ECA_NORMAL)
    {
      free(_pTimeValue);
      _pTimeValue = NULL;
      printf("EpicsMonitorPv::onCaChannelConnected()::ca_create_subscription(TIME) failed, Pv %s CA errmsg: %s\n",
        _sPvName.c_str(), ca_message(_iCaStatus));
      return 2;
    }

    return 0;
  }

  int EpicsMonitorPv::onCaChannelDisconnected()
  {
    // The channel might be just temporarily reset
    // so here we only set the flag to be false, and wait for it to come back in the future 
    _bConnected = false;
    return 0;
  }

  void EpicsMonitorPv::onSubscriptionUpdate(const evargs & args)
  {
    _iCaStatus = args.status;
    if (_iCaStatus != ECA_NORMAL)
    {
      printf("EpicsMonitorPv::onSubscriptionUpdate() status not okay, Pv %s CA errmsg: %s\n",
        _sPvName.c_str(), ca_message(_iCaStatus));
      return;
    }

    if (args.count != (int) _ulNumElems)
    {
      printf("EpicsMonitorPv::onSubscriptionUpdate(): Inconsistent Pv Element Count, Type %ld Count %ld (Prev Count %d)\n",
        args.type, args.count, (int) _ulNumElems);
      return;
    }

    if (args.type == _lDbrTimeType)
    {
      _bTimeValueUpdated = true;
      memcpy(_pTimeValue, args.dbr, dbr_size_n(args.type, args.count));
    }
    else if (args.type == _lDbrCtrlType)
    {
      _bCtrlValueUpdated = true;
      memcpy(_pCtrlValue, args.dbr, dbr_size_n(args.type, args.count));

      _iCaStatus = ca_clear_subscription(_evidCtrl);
      _evidCtrl = NULL;
      if (_iCaStatus != ECA_NORMAL)
      {
        printf("EpicsMonitorPv::onSubscriptionUpdate()::ca_clear_subscription() Failed for Pv %s CA errmsg: %s\n",
          _sPvName.c_str(), ca_message(_iCaStatus));
        return;
      }
    }
    else
    {
      printf("EpicsMonitorPv::onSubscriptionUpdate(): Incorrect Pv Data Type, Type %ld Count %ld\n",
        args.type, args.count);
      return;
    }

    _lDbrLastUpdateType = args.type;
  }

  const EpicsMonitorPv::TPrintPvFuncPointer EpicsMonitorPv::
    lfuncPrintPvFunctionTable[] = {
    &EpicsMonitorPv::printPvByDbrId < DBR_STRING >,
      &EpicsMonitorPv::printPvByDbrId < DBR_SHORT >,
    &EpicsMonitorPv::printPvByDbrId < DBR_FLOAT >,
      &EpicsMonitorPv::printPvByDbrId < DBR_ENUM >,
    &EpicsMonitorPv::printPvByDbrId < DBR_CHAR >,
      &EpicsMonitorPv::printPvByDbrId < DBR_LONG >,
    &EpicsMonitorPv::printPvByDbrId < DBR_DOUBLE >
  };

  int EpicsMonitorPv::printPv() const
  {
    if (!_bConnected || _lDbrLastUpdateType == -1)
    {
      printf("EpicsMonitorPv::printPv(): Pv %s not Connected\n",
       _sPvName.c_str());
      return 1;
    }

    if (_lDbfType < 0 || _lDbfType >= _iSizeBasicDbrTypes)
    {
      printf("EpicsMonitorPv::printPv(): Unknown data type %ld\n", _lDbfType);
      return 2;
    }

    printf("\n> PV %s\n", _sPvName.c_str());

    (this->*lfuncPrintPvFunctionTable[_lDbfType]) ();

    return 0;
  }

  template<int T>
  int EpicsMonitorPv::writeXtcCtrlValueByDbrId(char*& p, int &isize) const 
  {
    typedef typename EpicsDbrTools::DbrTypeTraits<T>::TPdsCtrl TPdsCtrl;
    typedef typename EpicsDbrTools::DbrTypeTraits<T>::TDbrCtrl TDbrCtrl;
    typedef typename EpicsDbrTools::DbrTypeTraits<T>::TDbrOrgP TDbrOrgP;

    const TDbrCtrl* v = reinterpret_cast<const TDbrCtrl*>(_pCtrlValue);
    TPdsCtrl* q = new(p) TPdsCtrl(_iPvId, 
                                  EpicsDbrTools::DbrTypeTraits<T>::iDbrCtrlType, 
                                 _ulNumElems,
                                  _sPvName.c_str(),
                                  *v, reinterpret_cast<TDbrOrgP>(v+1));

    isize = q->_sizeof();
    return 0;
  }

  const EpicsMonitorPv::TWriteXtcValueFuncPointer EpicsMonitorPv::
    lfuncWriteXtcCtrlValueFunctionTable[] = {
    &EpicsMonitorPv::writeXtcCtrlValueByDbrId < DBR_STRING >,
    &EpicsMonitorPv::writeXtcCtrlValueByDbrId < DBR_SHORT  >,
    &EpicsMonitorPv::writeXtcCtrlValueByDbrId < DBR_FLOAT  >,
    &EpicsMonitorPv::writeXtcCtrlValueByDbrId < DBR_ENUM   >,
    &EpicsMonitorPv::writeXtcCtrlValueByDbrId < DBR_CHAR   >,
    &EpicsMonitorPv::writeXtcCtrlValueByDbrId < DBR_LONG   >,
    &EpicsMonitorPv::writeXtcCtrlValueByDbrId < DBR_DOUBLE >
  };

  template <int T>
  int EpicsMonitorPv::writeXtcTimeValueByDbrId(char*& p, int &isize) const 
  {
    typedef typename EpicsDbrTools::DbrTypeTraits<T>::TPdsTime TPdsTime;
    typedef typename EpicsDbrTools::DbrTypeTraits<T>::TDbrTime TDbrTime;
    typedef typename EpicsDbrTools::DbrTypeTraits<T>::TDbrOrgP TDbrOrgP;

    const TDbrTime* v = reinterpret_cast<const TDbrTime*>(_pTimeValue);
    TPdsTime* q = new(p) TPdsTime(_iPvId, EpicsDbrTools::DbrTypeTraits<T>::iDbrTimeType, _ulNumElems,
                                  *v, reinterpret_cast<const TDbrOrgP>(v+1));

    isize = q->_sizeof();
    return 0;
  }

  const EpicsMonitorPv::TWriteXtcValueFuncPointer EpicsMonitorPv::
    lfuncWriteXtcTimeValueFunctionTable[] = {
    &EpicsMonitorPv::writeXtcTimeValueByDbrId < DBR_STRING >,
    &EpicsMonitorPv::writeXtcTimeValueByDbrId < DBR_SHORT  >,
    &EpicsMonitorPv::writeXtcTimeValueByDbrId < DBR_FLOAT  >,
    &EpicsMonitorPv::writeXtcTimeValueByDbrId < DBR_ENUM   >,
    &EpicsMonitorPv::writeXtcTimeValueByDbrId < DBR_CHAR   >,
    &EpicsMonitorPv::writeXtcTimeValueByDbrId < DBR_LONG   >,
    &EpicsMonitorPv::writeXtcTimeValueByDbrId < DBR_DOUBLE >
  };

  bool EpicsMonitorPv::checkWriteEvent(const struct timespec& tsCurrent, unsigned int uVectorCur)
  {    
    double fTimeElapsed = (tsCurrent.tv_sec - _tsLastUpdate.tv_sec) + (tsCurrent.tv_nsec - _tsLastUpdate.tv_nsec) * 1e-9;
    if (fTimeElapsed >= _fUpdateInterval)
    {     
      _tsLastUpdate     = tsCurrent;
      _u64MaskEventNode = ( ((uint64_t)1) << _iNumEventNode) - 1;    
    }
    
    if ( _u64MaskEventNode == 0 )
    {
      _bWriteEvent = false;
      return false;
    }

    uint64_t uEventBit = (uint64_t)1 << ( uVectorCur % _iNumEventNode );
    
    if ( _u64MaskEventNode & uEventBit )
    {
      _u64MaskEventNode ^= uEventBit;
      _bWriteEvent      =  true;
      return true;
    }
        
    _bWriteEvent = false;
    return false;
  }
  
  int EpicsMonitorPv::writeXtc(char *pcXtcMem, bool bCtrlValue, int &iSizeXtc)
  {
    if (pcXtcMem == NULL)
      return 1;
      
    if (!_bConnected || _lDbrLastUpdateType == -1)
    {
      if (_iNumReportForNoConnection < _iMaxNumReportForNoConnection)
      {
        printf("EpicsMonitorPv::writeXtc(): Pv %s not Connected\n", _sPvName.c_str());
        _iNumReportForNoConnection++;
      }
      return 2;     // This error code (2) is a special case, and will be checked by the caller function.
    }

    if (_lDbfType < 0 || _lDbfType >= _iSizeBasicDbrTypes)
    {
      if (_iNumReportForNoConnection < _iMaxNumReportForNoConnection)
      {
        printf("EpicsMonitorPv::writeXtc(): Unknown data type %ld\n", _lDbfType);
        _iNumReportForNoConnection++;
      }
      return 3;
    }

    if (bCtrlValue)
    {
      if (!_bCtrlValueUpdated)
      {
        printf("EpicsMonitorPv::writeXtc(): Pv %s Ctrl Value has not been updated\n", _sPvName.c_str());
        return 4;
      }

      int iFail =
        (this->*lfuncWriteXtcCtrlValueFunctionTable[_lDbfType]) (pcXtcMem, iSizeXtc);
      if (iFail != 0)
      {
        printf("EpicsMonitorPv::writeXtc(): writeXtcCtrlValue Failed\n");
        return iFail;
      }

      _bCtrlValueWritten = true;
      return 0;
    }
        
    {
      if (!_bTimeValueUpdated)
      {
        printf("EpicsMonitorPv::writeXtc(): Pv %s Time Value has not been updated\n",
          _sPvName.c_str());
        return 5;
      }

      return (this->*
        lfuncWriteXtcTimeValueFunctionTable[_lDbfType]) (pcXtcMem,
                     iSizeXtc);
    }
  }

/**
 * CA connection handler 
 */
  void EpicsMonitorPv::
    caConnectionHandler(struct connection_handler_args args)
  {
    EpicsMonitorPv & epicsPvCur = *(EpicsMonitorPv *) ca_puser(args.chid);

    if (args.op == CA_OP_CONN_UP)
    {
      epicsPvCur.onCaChannelConnected();
    }
    else if (args.op == CA_OP_CONN_DOWN)
    {
      epicsPvCur.onCaChannelDisconnected();
    }
  }

/**
 * CA subscription handler
 */
  void EpicsMonitorPv::caSubscriptionHandler(evargs args)
  {
    EpicsMonitorPv & epicsPvCur = *(EpicsMonitorPv *) ca_puser(args.chid);
    epicsPvCur.onSubscriptionUpdate(args);
  }
}       // namespace Pds
