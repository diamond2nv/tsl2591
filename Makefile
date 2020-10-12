KDIR := /Development/education/bbb/linux
CROSS_COMPILER := arm-linux-gnueabi-
ARCH := arm

obj-m += tsl2591.o

all:
	make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILER) -C $(KDIR) M=$(PWD) modules
clean:
	rm -rf *.o *.ko *.mod.* *.cmd .module* modules* Module* .*.cmd .tmp*
