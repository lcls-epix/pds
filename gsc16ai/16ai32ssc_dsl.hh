// $URL: http://subversion:8080/svn/gsc/trunk/drivers/LINUX/16AI32SSC/docsrc/16ai32ssc_dsl.h $
// $Rev$
// $Date$

#ifndef __16AI32SSC_DSL_HH
#define __16AI32SSC_DSL_HH

#include "gsc16ai/include/16ai32ssc.h"

#define AI32SSC_DEV_BASE_NAME   "/dev/" AI32SSC_BASE_NAME

// prototypes	***************************************************************

#ifdef __cplusplus
extern "C" {
#endif

// int	ai32ssc_dsl_close(int fd);
int	ai32ssc_dsl_ioctl(int fd, int request, void *arg);
// int	ai32ssc_dsl_open(unsigned int board);
int	ai32ssc_dsl_read(int fd, __u32 *buf, size_t samples);

#ifdef __cplusplus
}
#endif

#endif
