# LMP partner specific customizations (either replace or extend options as defined by meta-lmp)

PACKAGECONFIG_append_pn-networkmanager_portenta-mx8mm = " modemmanager"

# drop idle timeout in weston
PACKAGECONFIG_append_pn-weston-init_portenta-mx8mm = " no-idle-timeout"
