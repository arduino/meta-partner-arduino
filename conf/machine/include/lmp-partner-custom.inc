# LMP partner specific customizations (either replace or extend options as defined by meta-lmp)

PACKAGECONFIG_append_pn-networkmanager_portenta-m8 = " modemmanager"

# drop idle timeout in weston
PACKAGECONFIG_append_pn-weston-init_portenta-m8 = " no-idle-timeout"

# Use legacy kernel fit-image node separator so current devices won't break
FIT_NODE_SEPARATOR_portenta-m8 = "@"
