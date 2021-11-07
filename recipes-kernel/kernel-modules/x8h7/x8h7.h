/**
 * Portenta X8H7
 */
#ifndef __X8H7_H
#define __X8H7_H

#define X8H7_RX_TIMEOUT (HZ/10)

typedef void (*x8h7_hook_t)(void *priv, x8h7_pkt_t *pkt);

int x8h7_pkt_enq(uint8_t peripheral, uint8_t opcode, uint16_t size, void *data);
int x8h7_pkt_send(void);
int x8h7_hook_set(uint8_t idx, x8h7_hook_t hook, void *priv);

#endif  /* __X8H7_H */
