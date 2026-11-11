################################################################################
# sx1262-linux (local standalone BR2_EXTERNAL tree)
################################################################################

SX1262_LINUX_VERSION = 0.1.0
SX1262_LINUX_SITE = $(BR2_EXTERNAL_SX1262_PATH)/..
SX1262_LINUX_SITE_METHOD = local
SX1262_LINUX_LICENSE = GPL-2.0-only
SX1262_LINUX_LICENSE_FILES = COPYING
SX1262_LINUX_INSTALL_STAGING = YES

define SX1262_LINUX_INSTALL_STAGING_CMDS
	$(INSTALL) -D -m 0644 $(@D)/include/uapi/linux/sx1262.h \
		$(STAGING_DIR)/usr/include/linux/sx1262.h
endef

$(eval $(kernel-module))
$(eval $(generic-package))
