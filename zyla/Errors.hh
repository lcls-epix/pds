#ifndef Pds_Zyla_Errors_hh
#define Pds_Zyla_Errors_hh

namespace Pds {
  namespace Zyla {
    class ErrorCodes
    {
      public:
        ErrorCodes() {}
        ~ErrorCodes() {}

        static const char *name(unsigned id);
    };
  }
}

#endif
