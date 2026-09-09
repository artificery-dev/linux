# SPDX-License-Identifier: GPL-2.0

ccflags-y += -DMT6627_FM
ccflags-y += -I$(src)/inc -I$(src)/mt6627/inc
ccflags-y += -I$(src)/../common/include -I$(src)/../common/core/include
ccflags-y += -I$(src)/../common/linux/include -I$(src)/../common/linux/pri/include
ccflags-y += -I$(src)/../detect -I$(src)/../plat/include
ccflags-y += -Wno-unused-variable -Wno-unused-function -Wno-missing-prototypes
ccflags-y += -Wno-missing-declarations -Wno-misleading-indentation
ccflags-y += -Wno-undef -Wno-format

obj-$(CONFIG_MTK_CONSYS_FM) += mtk_fm_drv.o

mtk_fm_drv-y := \
	core/fm_module.o \
	core/fm_main.o \
	core/fm_config.o \
	core/fm_rds_parser.o \
	core/fm_patch.o \
	core/fm_utils.o \
	core/fm_link.o \
	core/fm_eint.o \
	mt6627/pub/mt6627_fm_lib.o \
	mt6627/pub/mt6627_fm_rds.o \
	mt6627/pub/mt6627_fm_cmd.o \
	mt6627/pub/mt6627_fm_config.o
