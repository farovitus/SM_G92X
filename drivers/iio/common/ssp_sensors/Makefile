# SPDX-License-Identifier: GPL-2.0-only
#
# Makefile for SSP sensor drivers and commons.
#

sensorhub-y				:= ssp_dev.o ssp_spi.o
sensorhub-$(CONFIG_IIO_SSP_SENSORHUB_BCM4773)	+= ssp_bbd.o
obj-$(CONFIG_IIO_SSP_SENSORHUB)		+= sensorhub.o

obj-$(CONFIG_IIO_SSP_SENSORS_COMMONS) 	+= ssp_iio.o
obj-$(CONFIG_IIO_SSP_HUB_SENSORS)	+= ssp_hub_sensors.o
