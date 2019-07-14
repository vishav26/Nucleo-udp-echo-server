#ifndef __ECHORINGIF_H__
#define __ECHORINGIF_H__

#include "lwip/err.h"
#include "lwip/netif.h"
#include "cmsis_os.h"

/* Exported functions ------------------------------------------------------- */
err_t echoringif_init(struct netif *netif);
void ethernetif_input( void const * argument );
void echoringif_input( void const * argument );

u32_t sys_now(void);

#endif
