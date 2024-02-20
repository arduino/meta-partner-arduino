#!/bin/sh
# See companion modem_on.sh

ENV_FILE="/var/run/arduino_hw_info.env"

if [ -s $ENV_FILE ]; then
    . $ENV_FILE
else
    echo "Env file $ENV_FILE not found"
    exit 1
fi

if [ "$IS_ON_CARRIER" = "yes" ]; then
    if [ "$CARRIER_NAME" = "max" ]; then
        echo "Power on SARA-R4 usb modem on $CARRIER_NAME"
        gpioset gpiochip5 4=0 # PWR_ON=High
        gpioset gpiochip5 2=1 # RST=Low

        echo "Power off pcie usb modem on $CARRIER_NAME"
        gpioset gpiochip5 29=0 # PCIE 3V3 BUCK EN (stm32h7 PWM6)
    elif [ "$CARRIER_NAME" = "mid" ]; then
        echo "Power off pcie usb modem on $CARRIER_NAME"
        gpioset gpiochip5 5=0 # # PCIE 3V3 BUCK EN (stm32h7 PE10)
    fi
    exit 0
else
    echo "Not on a carrier board"
    exit 0
fi
exit 0
