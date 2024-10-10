![Microchip logo](https://raw.githubusercontent.com/wiki/Microchip-MPLAB-Harmony/Microchip-MPLAB-Harmony.github.io/images/microchip_logo.png)

# LAN865x 10BASE-T1S MAC-PHY Ethernet Linux Driver Release Notes
This document describes the release notes of LAN865x Linux driver.
## LAN865x 10BASE-T1S MAC-PHY Ethernet Linux Driver Release v0.2
Added support for LAN865x Rev.B1. The below version v0.1 already supports for Rev.B0.
## LAN865x 10BASE-T1S MAC-PHY Ethernet Linux Driver Release v0.1
## New Features
**LAN865x Linux driver with generic OPEN Alliance TC6 Protocol implementation**
- **New device support** -
  This release introduces support for

    | Device    | Description             |
    | ------    | ------                  |
    | LAN865x   | 10BASE-T1S Ethernet MAC-PHY |

- **Preparation scripts support** -
  The following table provides a list of scripts for installing and configuring LAN865x Linux driver in Raspberry Pi

| Name                    | Comments                                                |
| ---                     | ---                                                     |
| [prepare_lan865x_cli.sh](prepare_lan865x_cli.sh)  | LAN865x preparation script with command line interface for Raspberry Pi   |
| [prepare_lan865x_tui.sh](prepare_lan865x_tui.sh)  | LAN865x preparation script with terminal user interface for Raspberry Pi |
