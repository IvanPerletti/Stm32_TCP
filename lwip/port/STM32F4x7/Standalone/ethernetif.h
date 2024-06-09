#ifndef __ETHERNETIF_H__
#define __ETHERNETIF_H__


#include "lwip/err.h"
#include "lwip/netif.h"

#define DEST_PORT       7

/* MAC ADDRESS: MAC_ADDR0:MAC_ADDR1:MAC_ADDR2:MAC_ADDR3:MAC_ADDR4:MAC_ADDR5 */
#define MAC_ADDR0   0x00
#define MAC_ADDR1   0x80
#define MAC_ADDR2   0xE1
#define MAC_ADDR3   0x01
#define MAC_ADDR4   0x02
#define MAC_ADDR5   0x03

err_t ethernetif_init(struct netif *netif);
err_t ethernetif_input(struct netif *netif);

#endif
