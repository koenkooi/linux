# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the orinoco wireless device drivers.
#
# hermes.o is the chip transport and stays a module of its own, because the
# bus glue calls hermes_struct_init() directly. orinoco.o is the
# bus-independent core. kbuild cannot build a composite that contains a
# member with its own basename, so orinoco_cs.o stays a single object.

obj-$(CONFIG_HERMES)		+= hermes.o orinoco.o
obj-$(CONFIG_PCMCIA_HERMES)	+= orinoco_cs.o

orinoco-y := core.o cfg.o scan.o hw.o ccmp.o
orinoco-$(CONFIG_HERMES_PRISM_FWDL) += fw_prism.o
