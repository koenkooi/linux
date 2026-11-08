# SPDX-License-Identifier: GPL-2.0

veo_sdiocam-y := veo-sdiocam-core.o veo-sdiocam-v4l2.o veo-sdiocam-ctrl.o \
		 veo-sdiocam-jpeg.o veo-sdiocam-tables.o

veo_sdiocam-$(CONFIG_VIDEO_VEO_SDIOCAM_KUNIT_TEST) += veo-sdiocam-jpeg-test.o

obj-$(CONFIG_VIDEO_VEO_SDIOCAM) += veo_sdiocam.o
