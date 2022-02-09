#!/bin/sh

export BOARD=$(fw_printenv board | cut -d '=' -f 2)
export IS_ON_CARRIER=$(fw_printenv is_on_carrier | cut -d '=' -f 2)
export CARRIER_NAME=$(fw_printenv carrier_name | cut -d '=' -f 2)
