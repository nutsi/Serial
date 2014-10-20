KERNEL_SOURCE := ../linux
PWD := $(shell pwd)

obj-m :=	serial.o

default:
	${MAKE} -C ${KERNEL_SOURCE} SUBDIRS=${PWD} modules
install:
	sudo rmmod 8250_pnp && sudo rmmod 8250 && sudo insmod driver.ko
reinstall:
	sudo rmmod driver && sudo insmod driver.ko
clean:
	${MAKE} -C ${KERNEL_SOURCE} SUBDIRS=${PWD} clean
