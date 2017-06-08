# List of packages (low level first)
packages := service collection xtc
packages += config confignfs configsql mon vmon
packages += utility management client offlineclient
packages += ipimb camera evgr epicstools pgp imp
packages += pnccd epicsArch
packages += oceanoptics fli andor usdusb
packages += cspad cspad2x2
packages += ioc
packages += rayonix
packages += epixSampler
packages += epix
packages += epix10k
packages += epix100a
packages += genericpgp
packages += udpcam
packages += lecroy
packages += pvdaq
packages += monreq
packages += archon
packages += gsc16ai

ifneq ($(findstring x86_64,$(tgt_arch)),)
  packages += firewire jungfrau
##  No DDL
#  packages += phasics
else
  packages += encoder acqiris \
              princeton
endif
##  No DDL
#  packages += xamps
#  packages += fexamp

ifneq ($(findstring x86_64-rhel6,$(tgt_arch)),)
  packages += pimax
endif

ifneq ($(findstring x86_64-rhel7,$(tgt_arch)),)
  packages += zyla
endif



