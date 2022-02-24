/**
 */
#ifndef __DEBUG_H
#define __DEBUG_H

#ifdef DEBUG
  #warning "Verbose debug messages will be activated for this module!"
  #define DBG_PRINT(fmt, ...)   { \
    char  __ctx[7]; \
    __ctx[0] = in_irq()            ? 'H': '-'; \
    __ctx[1] = in_softirq()        ? 'S': '-'; \
    __ctx[2] = in_interrupt()      ? 'I': '-'; \
    __ctx[3] = in_serving_softirq()? 'V': '-'; \
    __ctx[4] = in_nmi()            ? 'N': '-'; \
    __ctx[5] = in_atomic()         ? 'A': '-'; \
    __ctx[6] = 0; \
    printk("%s:%s:%s:"fmt, DRIVER_NAME, __ctx, __func__, ##__VA_ARGS__); \
  }

  #define DBG_ERROR(fmt, ...)  { \
    char  __ctx[7]; \
    __ctx[0] = in_irq()            ? 'H': '-'; \
    __ctx[1] = in_softirq()        ? 'S': '-'; \
    __ctx[2] = in_interrupt()      ? 'I': '-'; \
    __ctx[3] = in_serving_softirq()? 'V': '-'; \
    __ctx[4] = in_nmi()            ? 'N': '-'; \
    __ctx[5] = in_atomic()         ? 'A': '-'; \
    __ctx[6] = 0; \
    printk("%s:%s:%s:ERROR:"fmt, DRIVER_NAME, __ctx, __func__, ##__VA_ARGS__); \
  }
#else
  #define DBG_PRINT(fmt, ...)
  #define DBG_ERROR(fmt, ...)
#endif

#endif  /* __DEBUG_H */
