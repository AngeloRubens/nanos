/* nanos runtime headers have no include guards (deliberate), so pull
 * <runtime.h> exactly once behind this guard; every shim includes this
 * instead of <runtime.h> directly. */
#ifndef GVE_HARNESS_RUNTIME_INCLUDED
#define GVE_HARNESS_RUNTIME_INCLUDED
#ifndef NULL
#define NULL ((void *)0)   /* the kernel build gets this elsewhere */
#endif
#include <runtime.h>
#endif
