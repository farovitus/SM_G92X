# SPDX-License-Identifier: GPL-2.0-only
obj-$(CONFIG_BCM4773) += bcm4773.o

bcm4773-y := core.o engine.o ssi.o \
	     transport/bbd_bridge_c.o \
	     transport/bbd_packet_layer_c.o \
	     transport/transport_layer_c.o \
	     utils/crc8bits_c.o \
	     utils/stream_codec_c.o

# Broadcom's transport code, kept close to upstream.
ccflags-y += -Wno-unused-variable -Wno-unused-function
