DESCRIPTION = "Preempt-RT kernel for Arduino Portenta MX8MM platform"

include recipes-kernel/linux/linux-lmp-fslc-imx_git.bb

KERNEL_REPO = "git://github.com/foundriesio/linux.git"
LINUX_VERSION = "5.10.90"
KERNEL_BRANCH = "5.10-2.1.x-imx-rt"

SRCREV_machine = "fcae15dfd58d61c6587298cf4456358d201fab1b"
LINUX_KERNEL_TYPE = "preempt-rt"
