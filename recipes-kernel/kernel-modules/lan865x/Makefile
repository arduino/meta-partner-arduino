obj-m += microchip_t1s.o
microchip_t1s-y := src/microchip_t1s.o
obj-m += lan865x_t1s.o
lan865x_t1s-y := src/lan865x.o src/oa_tc6.o

SRC := $(shell pwd)

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install

clean:
	rm -f *.o *~ core .depend .*.cmd *.ko *.mod.c
	rm -f Module.markers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symvers