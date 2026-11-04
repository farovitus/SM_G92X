# SPDX-License-Identifier: GPL-2.0-only

obj-$(CONFIG_TRUSTONIC_MOBICORE) += mcDrvModule.o

mcDrvModule-objs := MobiCoreDriver/logging.o \
		    MobiCoreDriver/ops.o \
		    MobiCoreDriver/mem.o \
		    MobiCoreDriver/api.o \
		    MobiCoreDriver/main.o

ccflags-y += -DNDEBUG -I$(srctree)/$(src)/MobiCoreDriver/public
