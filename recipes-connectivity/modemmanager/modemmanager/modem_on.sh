#!/bin/sh

ENV_FILE="/var/run/arduino_hw_info.env"

if [ -s $ENV_FILE ]; then
    . $ENV_FILE
else
    echo "Env file $ENV_FILE not found"
    exit 1
fi

if [ "$IS_ON_CARRIER"=="yes" ]; then
    # @TODO: we expect to fail if pin owned by kernel (pcie_mini ov)
    # would be more elegant a u-boot env variable to configure usb modems
    if [ "$CARRIER_NAME"=="max" ]; then
        echo "Power on SARA-R4 usb modem on $CARRIER_NAME"
        # Note: we're driving open-drain n-mos
        # pull-ups are provided internally by modem
        gpioset gpiochip5 4=1 # PWR_ON=Low
        gpioset gpiochip5 2=0 # RST=High
        gpioset gpiochip5 2=1 # RST=Low
        sleep 1
        gpioset gpiochip5 2=0 # RST=High

        echo "Power on pcie usb modem on $CARRIER_NAME"
        gpioset gpiochip5 29=1 # PCIE 3V3 BUCK EN (stm32h7 PWM6)
        sleep 1
    elif [ "$CARRIER_NAME"=="mid" ]; then
        echo "Power on pcie usb modem on $CARRIER_NAME"
        gpioset gpiochip5 5=1 # # PCIE 3V3 BUCK EN (stm32h7 PE10)
        sleep 1
    fi
    exit 0
else
    echo "Not on a carrier board"
    exit 0
fi
exit 0
