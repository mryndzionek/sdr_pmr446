#ifndef __SHARED_H__
#define __SHARED_H__

#include <stdbool.h>

#ifdef APP_SDR_PMR446
#include "sdr_pmr446.h"
#elif APP_DSD_IN
#include "dsd_in.h"
#else
#error "The app identifier is not defined!"
#endif

typedef struct _proc_chain_t proc_chain_t;

bool init_soapy(proc_chain_t *chain);
void destroy_soapy(proc_chain_t *chain);

#endif // __SHARED_H__