#!/bin/sh

ENV_FILE="/var/run/arduino_hw_info.env"

if [ -s $ENV_FILE ]; then
    . $ENV_FILE
else
    echo "Env file $ENV_FILE not found"
    exit 1
fi

if [ "$IS_ON_CARRIER"=="yes" ]; then
    if ! [ "$CARRIER_NAME"=="max" ]; then
        echo "Not on Max Carrier"
        exit 1
    fi
else
    echo "Not on a carrier board"
    exit 1
fi

echo 162 > /sys/class/gpio/export
echo 164 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio162/direction
echo out > /sys/class/gpio/gpio164/direction
echo 1 > /sys/class/gpio/gpio164/value
echo 0 > /sys/class/gpio/gpio162/value
echo 1 > /sys/class/gpio/gpio162/value
sleep 1
echo 0 > /sys/class/gpio/gpio162/value

exit 0
