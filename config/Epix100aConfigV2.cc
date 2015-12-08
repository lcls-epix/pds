/*
 * Epix100aConfigV2.cc
 *
 *  Created on: 2014.4.17
 *      Author: jackp
 */

#include <stdio.h>
#include "pds/config/Epix100aConfigV2.hh"
#include "pds/config/Epix100aASICConfigV1.hh"
#include "pds/config/EpixConfigType.hh"

namespace Pds {
  namespace Epix100aConfig {

    class  Register {
      public:
        uint32_t        offset;
        uint32_t        shift;
        uint32_t        mask;
        uint32_t        defaultValue;
        uint32_t        readOnly;
        ConfigV2::types type;
        uint32_t        selectionIndex;
    } ;
//    R0Mode = 0x1
//    asicPpmatControl = 0x0
//    AsicRoClkHalfT = 0xA

    static uint32_t _regsfoo[ConfigV2::NumberOfRegisters][7] = {
    //offset shift mask default readOnly type selectionIndex
        {  0,  0, 0x7fffffff, 0,      1, 1, 0},    //  Version,
        {  1,  0, 0x7fffffff, 0,      0, 0, 0},    //  RunTrigDelay,
        {  2,  0, 0x7fffffff, 0x4e2,  2, 0, 0},    //  DaqTrigDelay,
        {  3,  0, 0xffff,     0x4e20, 0, 1, 0},    //  DacSetting,
        //  pin states(4) and controls(5)
        {  4,  0, 1, 1, 0, 0, 0},    //  asicGR,
        {  5,  0, 1, 1, 0, 0, 0},    //  asicGRControl,
        {  4,  1, 1, 0, 0, 0, 0},    //  asicAcq,
        {  5,  1, 1, 0, 0, 0, 0},    //  asicAcqControl,
        {  4,  2, 1, 1, 0, 0, 0},    //  asicR0,
        {  5,  2, 1, 0, 0, 0, 0},    //  asicR0Control,
        {  4,  3, 1, 1, 0, 0, 0},    //  asicPpmat,
        {  5,  3, 1, 0, 0, 0, 0},    //  asicPpmatControl,
        {  4,  4, 1, 0, 0, 0, 0},    //  asicPpbe,
        {  5,  4, 1, 0, 0, 0, 0},    //  asicPpbeControl,
        {  4,  5, 1, 0, 0, 0, 0},    //  asicRoClk,
        {  5,  5, 1, 0, 0, 0, 0},    //  asicR0ClkControl,
        {  5,  6, 1, 1, 0, 0, 0},    //  prepulseR0En,
        {  5,  7, 1, 0, 0, 0, 0},    //  adcStreamMode,
        {  5,  8, 1, 0, 0, 0, 0},    //  testPatternEnable,
        {  5,  9, 3, 0, 0, 0, 0},    //  SyncMode
        {  5, 11, 1, 1, 0, 0, 0},    //  R0Mode
        //
        {  6,  0, 0x7fffffff,  3000,      0, 0, 0},    //  AcqToAsicR0Delay,
        {  7,  0, 0x7fffffff,  3200,      0, 0, 0},    //  AsicR0ToAsicAcq,
        {  8,  0, 0x7fffffff,  3200,      0, 0, 0},    //  AsicAcqWidth,
        {  9,  0, 0x7fffffff,  200,       0, 0, 0},    //  AsicAcqLToPPmatL,
        { 10,  0, 0x7fffffff,  11200,     0, 0, 0},    //  AsicPPmatToReadout,
        { 11,  0, 0x7fffffff, 0xa,        0, 0, 0},    //  AsicRoClkHalfT,
        { 12,  0, 3,          1,          0, 0, 0},    //  AdcReadsPerPixel,
        { 13,  0, 0x7fffffff, 1,          0, 0, 0},    //  AdcClkHalfT,
        { 14,  0, 0x7fffffff,   30,       0, 0, 0},    //  AsicR0Width,
        { 15,  0, 0x7fffffff,   31,       0, 0, 0},    //  AdcPipelineDelay,
        { 16,  0, 0xffff,       30,       0, 0, 0},    //  SnycWidth,
        { 16, 16, 0xffff,       30,       0, 0, 0},    //  SyncDelay,
        { 17,  0, 0x7fffffff,   30,       0, 0, 0},    //  PrepulseR0Width,
        { 18,  0, 0x7fffffff,  32000,     0, 0, 0},    //  PrepulseR0Delay,
        { 19,  0, 0x7fffffff, 0,          1, 0, 0},    //  DigitalCardId0,
        { 20,  0, 0x7fffffff, 0,          1, 0, 0},    //  DigitalCardId1,
        { 21,  0, 0x7fffffff, 0,          1, 0, 0},    //  AnalogCardId0,
        { 22,  0, 0x7fffffff, 0,          1, 0, 0},    //  AnalogCardId1,
        { 23,  0, 0x7fffffff, 0,          1, 0, 0},    //  CarrierId0,
        { 24,  0, 0x7fffffff, 0,          1, 0, 0},    //  CarrierId1,
        { 25,  0, 0xf,        2,          2, 0, 0},    //  NumberOfAsicsPerRow,
        { 26,  0, 0xf,        2,          2, 0, 0},    //  NumberOfAsicsPerColumn,
        { 27,  0, 0x1ff,      RowsPerASIC,2, 0, 0},    //  NumberOfRowsPerAsic,
        { 28,  0, 0x1ff,      RowsPerASIC,2, 0, 0},    //  NumberOfReadableRowsPerAsic,
        { 29,  0, 0x1ff,      ColsPerASIC,2, 0, 0},    //  NumberOfPixelsPerAsicRow,
        { 30,  0, 0x1ff,      2,          2, 0, 0},    //  CalibrationRowCountPerASIC,
        { 31,  0, 0x1ff,      1,          2, 0, 0},    //  EnvironmentalRowCountPerASIC,
        { 32,  0, 0x7fffffff, 0,          1, 0, 0},    //  BaseClockFrequency,
        { 33,  0, 0xffff,     0xf,        0, 1, 0},    //  AsicMask,
        { 34,  0, 1,          0,          0, 0, 0},    //  EnableAutomaticRunTrigger,
        { 35,  0, 0x7fffffff,  833334,    0, 2, 0},    //  NumbClockTicksPerRunTrigger,
        { 36,  0, 1,          0,          0, 0, 0},    //  ScopeEnable,
        { 36,  1, 1,          1,          0, 0, 0},    //  ScopeTrigEdge,
        { 36,  2, 0xf,        6,          0, 0, 0},    //  ScopeTrigCh,
        { 36,  6, 3,          2,          0, 0, 0},    //  ScopeArmMode,
        { 36, 16, 0xffff,     0,          0, 1, 0},    //  ScopeAdcThresh,
        { 37,  0, 0x1fff,     0,          0, 0, 0},    //  ScopeHoldoff,
        { 37, 13, 0x1fff,     0xf,        0, 0, 0},    //  ScopeOffset,
        { 38,  0, 0x1fff,     0x1000,     0, 0, 0},    //  ScopeTraceLength,
        { 38, 13, 0x1fff,     0,          0, 0, 0},    //  ScopeSkipSamples,
        { 39,  0, 0x1f,       0,          0, 0, 0},    //  ScopeInputA,
        { 39,  5, 0x1f,       4,          0, 0, 0},    //  ScopeInputB,
    };

    static uint32_t _regSelect0[7] = {
    		6,
    		0xFE503,
    		0x1fca05,
    		0x3f940b,
    		0xbebc20,
    		0x17d7840,
    		0x7735940
    };

    static uint32_t* _regSelects[ConfigV2::NumberOfSelects] = {
    		_regSelect0
    };

    static char _regNames[ConfigV2::NumberOfRegisters+1][120] = {
        {"Version"},
        {"RunTrigDelay"},
        {"DaqTrigDelay"},
        {"DacSetting"},
        {"asicGR"},
        {"asicGRControl"},
        {"asicAcq"},
        {"asicAcqControl"},
        {"asicR0"},
        {"asicR0Control"},
        {"asicPpmat"},
        {"asicPpmatControl"},
        {"asicPpbe"},
        {"asicPpbeControl"},
        {"asicRoClk"},
        {"asicRoClkControl"},
        {"prepulseR0En"},
        {"adcStreamMode"},
        {"testPatternEnable"},
        {"SyncMode"},
        {"R0Mode"},
        {"AcqToAsicR0Delay"},
        {"AsicR0ToAsicAcq"},
        {"AsicAcqWidth"},
        {"AsicAcqLToPPmatL"},
        {"AsicPPmatToReadout"},
        {"AsicRoClkHalfT"},
        {"AdcReadsPerPixel"},
        {"AdcClkHalfT"},
        {"AsicR0Width"},
        {"AdcPipelineDelay"},
        {"SyncWidth"},
        {"SyncDelay"},
        {"PrepulseR0Width"},
        {"PrepulseR0Delay"},
        {"DigitalCardId0"},
        {"DigitalCardId1"},
        {"AnalogCardId0"},
        {"AnalogCardId1"},
        {"CarrierId0"},
        {"CarrierId1"},
        {"NumberOfAsicsPerRow"},
        {"NumberOfAsicsPerColumn"},
        {"NumberOfRowsPerAsic"},
        {"NumberOfReadableRowsPerAsic"},
        {"NumberOfPixelsPerAsicRow"},
        {"CalibrationRowCountPerASIC"},
        {"EnvironmentalRowCountPerASIC"},
        {"BaseClockFrequency"},
        {"AsicMask"},
        {"EnableAutomaticRunTrigger"},
        {"NumbClockTicksPerRunTrigger"},
        {"ScopeEnable"},
        {"ScopeTrigEdge"},
        {"ScopeTrigCh"},
        {"ScopeArmMode"},
        {"ScopeAdcThresh"},
        {"ScopeHoldoff"},
        {"ScopeOffset"},
        {"ScopeTraceLength"},
        {"ScopeSkipSamples"},
        {"ScopeInputA"},
        {"ScopeInputB"},
        {"-------INVALID------------"}//NumberOfRegisters
    };

    static Register* _regs = (Register*) _regsfoo;
    static bool namesAreInitialized = false;

    ConfigV2::ConfigV2(bool init) {
      if (init) 
	clear();
    }

    ConfigV2::~ConfigV2() {}

    unsigned   ConfigV2::get (Registers r) const {
      if (r >= NumberOfRegisters) {
        printf("ConfigV2::get parameter out of range!! %u %u\n", r, NumberOfRegisters);
        return 0;
      }
      return ((_values[_regs[r].offset] >> _regs[r].shift) & _regs[r].mask);
    }

    void   ConfigV2::set (Registers r, unsigned v) {
      if (r >= NumberOfRegisters) {
        printf("ConfigV2::set parameter out of range!! %u %u\n", r, NumberOfRegisters);
        return;
      }
      _values[_regs[r].offset] &= ~(_regs[r].mask << _regs[r].shift);
      _values[_regs[r].offset] |= (_regs[r].mask & v) << _regs[r].shift;
      return;
    }

    void   ConfigV2::clear() {
      memset(_values, 0, NumberOfValues*sizeof(uint32_t));
      for(unsigned i=0; i<(defaultValue(NumberOfAsicsPerRow)*defaultValue(NumberOfAsicsPerColumn)); i++)
	      asics()[i].clear();
    }

    uint32_t ConfigV2::offset(Registers r) {
      if (r >= NumberOfRegisters) {
        printf("ConfigV2::set parameter out of range!! %u %u\n", r, NumberOfRegisters);
        return 0x7fffffff;
      }
      return _regs[r].offset;
    }

    uint32_t ConfigV2::numberOfAsics() const {
      return get(NumberOfAsicsPerRow) * get(NumberOfAsicsPerColumn);
    }

    uint32_t   ConfigV2::rangeHigh(Registers r) {
      uint32_t ret = _regs[r].mask;
      if (r >= NumberOfRegisters) {
        printf("ConfigV2::rangeHigh parameter out of range!! %u %u\n", r, NumberOfRegisters);
        return 0;
      }
      switch (r) {
        case AdcClkHalfT :
          ret = 400;
          break;
        case NumberOfRowsPerAsic :
        case NumberOfReadableRowsPerAsic :
          ret = RowsPerASIC;
          break;
        case NumberOfPixelsPerAsicRow :
          ret = ColsPerASIC;
          break;
        case NumberOfAsicsPerRow :
          ret = AsicsPerRow;
          break;
        case NumberOfAsicsPerColumn :
          ret = AsicsPerCol;
          break;
        case DaqTrigDelay :
          ret = 1250;
          break;
        case CalibrationRowCountPerASIC :
          ret = 2;
          break;
        case EnvironmentalRowCountPerASIC :
          ret = 1;
          break;
        case AsicMask :
          ret = 15;
          break;
        default:
          break;
      }
      return ret;
    }

    uint32_t   ConfigV2::rangeLow(Registers r) {
      uint32_t ret = 0;
      switch (r) {
        case AdcReadsPerPixel:
        case AsicMask :
          ret = 1;
          break;
        case NumberOfRowsPerAsic :
          ret = RowsPerASIC;
          break;
        case NumberOfPixelsPerAsicRow :
          ret = ColsPerASIC;
          break;
        case AdcClkHalfT :
          ret = 1;
          break;
        case NumberOfAsicsPerRow :
          ret = AsicsPerRow;
          break;
        case NumberOfAsicsPerColumn :
          ret = AsicsPerCol;
          break;
        case DaqTrigDelay :
          ret = 1250;
          break;
        case CalibrationRowCountPerASIC :
          ret = 2;
          break;
        case EnvironmentalRowCountPerASIC :
          ret = 1;
          break;
        default:
          break;
      }
      return ret;
    }

    uint32_t   ConfigV2::defaultValue(Registers r) {
      if (r >= NumberOfRegisters) {
        printf("ConfigV2::defaultValue parameter out of range!! %u %u\n", r, NumberOfRegisters);
        return 0;
      }
      return _regs[r].defaultValue & _regs[r].mask;
    }

    char* ConfigV2::name(Registers r) {
      return r < NumberOfRegisters ? _regNames[r] : _regNames[NumberOfRegisters];
    }

    void ConfigV2::initNames() {
      static char range[60];
      if (namesAreInitialized == false) {
        for (unsigned i=0; i<NumberOfRegisters; i++) {
          Registers r = (Registers) i;
          types t = (types) type(r);
          if (t==hex) sprintf(range, "  (0x%x->0x%x)    ", rangeLow(r), rangeHigh(r));
          else if (t==decimal) sprintf(range, "  (%u->%u)    ", rangeLow(r), rangeHigh(r));
          if ((_regs[r].readOnly != ReadOnly) && (t != selection)) {
            strncat(_regNames[r], range, 40);
//            printf("ConfigV2::initNames %u %s %s\n", i, range, _regNames[r]);
          }
        }
        ASIC_ConfigV1::initNames();
      } else {
//        printf("ASIC_ConfigV1::initNames namesAreInitialized=%s\n", namesAreInitialized ? "true" : "false");
      }
      namesAreInitialized = true;
    }

    unsigned ConfigV2::readOnly(Registers r) {
      if (r >= NumberOfRegisters) {
        printf("Epix100aConfigV2::readOnly parameter out of range!! %u %u\n", r, NumberOfRegisters);
        return 400;
      }
      return (unsigned)_regs[r].readOnly;
    }

    unsigned ConfigV2::type(Registers r) {
        if (r >= NumberOfRegisters) {
          printf("Epix100aConfigV2::type parameter out of range!! %u, numregs %u\n", r, NumberOfRegisters);
          return 400;
        }
        return (unsigned)_regs[r].type;
    }

    unsigned ConfigV2::numberOfSelections(Registers r) {
        if (r >= NumberOfRegisters) {
          printf("Epix100aConfigV2::numberOfselections parameter out of range!! %u %u\n", r, NumberOfRegisters);
          return 0;
        }
        if ((_regs[r].type == ConfigV2::selection) && (_regs[r].selectionIndex < NumberOfSelects)) {
        	return (unsigned)(_regSelects[_regs[r].selectionIndex][0]);
        }
        return 0;
    }

    int ConfigV2::select(Registers r, uint32_t s) {
    	unsigned n = ConfigV2::numberOfSelections(r);
    	if (n) {
    		return (int) _regSelects[_regs[r].selectionIndex][s+1];
    	}
    	return -1;
    }

    int ConfigV2::indexOfSelection(Registers r, uint32_t s) {
        unsigned i = 0;
    	unsigned n = ConfigV2::numberOfSelections(r);
    	if (n) {
    		do {
    			if (s == _regSelects[_regs[r].selectionIndex][i+1]) return i;
    		} while (i++ < n);
    	}
    	return -1;
    }

  } /* namespace Epix100aConfig */
} /* namespace Pds */
