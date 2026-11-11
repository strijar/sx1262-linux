# SPDX-License-Identifier: GPL-2.0-only

ifneq ($(KERNELRELEASE),)
obj-m := sx1262.o
sx1262-y := sx1262_main.o sx1262_radio.o
ccflags-y += -I$(src)/include/uapi
else
KDIR ?= /lib/modules/$(shell uname -r)/build
.PHONY: all modules modules_install clean test tools
all: modules
modules modules_install clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) $@
test:
	$(HOSTCC) -std=c11 -Wall -Wextra -Werror -Iinclude/uapi tests/logic_test.c -o tests/logic_test
	./tests/logic_test
HOSTCC ?= cc
USERCC ?= $(CROSS_COMPILE)gcc
tools:
	$(USERCC) -std=c11 -O2 -Wall -Wextra -Werror -Iinclude/uapi tools/sx1262-test.c -o tools/sx1262-test
endif
