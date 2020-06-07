# SPDX-License-Identifier: GPL-2.0
dtb-$(CONFIG_ARCH_FSL_IMX8QM) += armstonemx8m.dtb \
								armstonemx8m-dcss-sn65dsi84-g070y2-l01.dtb \
								armstonemx8m-dcss-sn65dsi84-g101ice-l01.dtb \
								armstonemx8m-lcdif-sn65dsi84-g070y2-l01.dtb \
								armstonemx8m-lcdif-sn65dsi84-g101ice-l01.dtb \
								armstonemx8m-dcss-hdmi.dtb

dtb-$(CONFIG_ARCH_FSL_IMX8MM) += picocoremx8mm.dtb \
								picocoremx8mm-bbdsi.dtb \
								picocoremx8mm-lcdif-tc358764-g070y2-l01.dtb \
								picocoremx8mm-lcdif-tc358764-j070wvtc0211.dtb \
								picocoremx8mm-lcdif-g050tan01.dtb \
								picocoremx8mm-lcdif-adp-mipi2lvds1-j070wvtc0211-single-display.dtb \
								picocoremx8mm-lcdif-rf3500d-ayw-mng1.dtb 

dtb-$(CONFIG_ARCH_FSL_IMX8QXP) += efusmx8x.dtb

dtb-$(CONFIG_ARCH_FSL_IMX8MN) += picocoremx8mn.dtb \
				picocoremx8mn-bbdsi.dtb \
				picocoremx8mn-lcdif-tc358764-g070y2-l01.dtb \
                picocoremx8mn-lcdif-tc358764-j070wvtc0211.dtb

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb
