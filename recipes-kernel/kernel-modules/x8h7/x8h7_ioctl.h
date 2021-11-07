/**
 */

#ifndef __X8H7_IOCTL_H
#define __X8H7_IOCTL_H

#include <linux/types.h>
#include <linux/ioctl.h>

#define X8H7_PKT_SIZE   1024

typedef struct {
  uint8_t   peripheral;
  uint8_t   opcode;
  uint16_t  size;
  uint8_t   data[X8H7_PKT_SIZE];
} x8h7_pkt_t;


#define X8H7_IOCTL_MAGIC     0xB5

#define X8H7_IOCTL_PKT_INIT      _IO   (X8H7_IOCTL_MAGIC, 0)
#define X8H7_IOCTL_PKT_SEND      _IO   (X8H7_IOCTL_MAGIC, 1)
#define X8H7_IOCTL_PKT_ENQ       _IOW  (X8H7_IOCTL_MAGIC, 2, x8h7_pkt_t*)

#define X8H7_IOCTL_MAXNR     2

#endif  /* __X8H7_IOCTL_H */
