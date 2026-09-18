global-incdirs-y += .

srcs-$(CFG_SCMI_MSG_DRIVERS) += scmi_server.c
srcs-y += main.c
srcs-y += stm32mp_pm.c
subdirs-y += drivers
