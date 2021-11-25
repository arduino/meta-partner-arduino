obj-m += x8h7_drv.o
obj-m += x8h7_adc.o
obj-m += x8h7_gpio.o
obj-m += x8h7_pwm.o
obj-m += x8h7_rtc.o
obj-m += x8h7_can.o
obj-m += x8h7_uart.o
obj-m += x8h7_ui.o
obj-m += x8h7_h7.o

SRC := $(shell pwd)

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install

clean:
	rm -f *.o *~ core .depend .*.cmd *.ko *.mod.c
	rm -f Module.markers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symvers
