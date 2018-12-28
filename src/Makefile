obj-m = l3gd20.o
KERN_DIR = ~/Documents/Sensor/kernel-3.4.39
PWD = $(shell pwd)

all:
	make -C $(KERN_DIR) SUBDIRS=$(PWD) modules

.PHONY: clean cp
clean:
	rm *.o *.ko *.symvers *.order *.mod.c
cp:
	cp l3gd20.c $(KERN_DIR)/drivers/hwmon
