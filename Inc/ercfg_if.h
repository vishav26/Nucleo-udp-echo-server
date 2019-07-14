#ifndef __CFG_IF_H__
#define __CFG_IF_H__

#include "lwip/def.h"
#include "lwip/err.h"
struct cfgif
{

	/** maximum transfer unit (in bytes) */
	  u16_t mtu;
};




err_t cfg_init(struct cfgif *cfgif);

u16_t getconfig_mtu(struct cfgif *cfgif);

u16_t setconfig_mtu(struct cfgif *cfgif, u16_t new_mtu);

#endif
