mi2-objs := anx7625_driver.o \
            anx7625_private_interface.o \
            anx7625_public_interface.o \
            Flash.o \
            display.o \
            anx7625_display.o \
            anx7625_display_table.o \
            anx7625_pd30_message.o \
            anx7625_pd_fw_update.o

#For other platform, parse EDID.
#mi2-objs += display_drv.o display_platform.o display_edid_3d.o

obj-m += mi2.o



SRC := $(shell pwd)

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install

clean:
	rm -f *.o *~ core .depend .*.cmd *.ko *.mod.c
	rm -f Module.markers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symvers
