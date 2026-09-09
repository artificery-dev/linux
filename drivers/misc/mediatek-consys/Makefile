# SPDX-License-Identifier: GPL-2.0
subdir-ccflags-y += -I$(src)/plat/include
subdir-ccflags-y += -I$(src)/btif/inc -I$(src)/btif/hal_inc -I$(src)/btif/mt6582/inc
subdir-ccflags-y += -I$(src)/common/include -I$(src)/common/core/include
subdir-ccflags-y += -I$(src)/common/linux/include -I$(src)/common/linux/pri/include
subdir-ccflags-y += -I$(src)/detect -I$(src)/detect/drv_init/inc
subdir-ccflags-y += -DWMT_PLAT_ALPS=1 -DMTK_WCN_SOC_CHIP_SUPPORT -DWMT_CREATE_NODE_DYNAMIC=1
subdir-ccflags-y += -Wno-unused-variable -Wno-unused-function -Wno-format -Wno-undef -Wno-old-style-declaration -Wno-missing-prototypes -Wno-missing-declarations -Wno-ignored-qualifiers -Wno-enum-conversion

obj-$(CONFIG_MTK_CONSYS) += plat/ btif/ common/ detect/
obj-$(CONFIG_MTK_CONSYS_BT) += bt/
obj-$(CONFIG_MTK_CONSYS) += wifi/
obj-$(CONFIG_MTK_CONSYS_FM) += fmradio/
