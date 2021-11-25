/**
 */

#ifndef __X8H7_IOCTL_H
#define __X8H7_IOCTL_H

#include <linux/types.h>
#include <linux/ioctl.h>

#define X8H7_MODE_DEBUG   0x00000001

#define X8H7_IOCTL_MAGIC     0xB5

#define X8H7_IOCTL_MODE_SET      _IOW  (X8H7_IOCTL_MAGIC, 0, uint32_t*)
#define X8H7_IOCTL_MODE_GET      _IOR  (X8H7_IOCTL_MAGIC, 1, uint32_t*)
#define X8H7_IOCTL_FW_VER        _IOR  (X8H7_IOCTL_MAGIC, 2, x8h7_pkt_t*)
#define X8H7_IOCTL_PKT_INIT      _IO   (X8H7_IOCTL_MAGIC, 3)
#define X8H7_IOCTL_PKT_ENQ       _IOW  (X8H7_IOCTL_MAGIC, 4, x8h7_pkt_t*)
#define X8H7_IOCTL_PKT_SEND      _IO   (X8H7_IOCTL_MAGIC, 5)

#define X8H7_IOCTL_MAXNR     5

#endif  /* __X8H7_IOCTL_H */
