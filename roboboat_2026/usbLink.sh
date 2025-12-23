#!/bin/bash

for sysdevpath in $(find /sys/bus/usb/devices/usb*/ -name dev); do
(
    syspath="$(dirname "$sysdevpath")"
    devname="$(udevadm info -q name -p "$syspath")"


    [[ "$devname" == "bus/"* ]] && exit

    # echo $devname
    # echo $syspath
    eval "$(udevadm info -q property --export -p "$syspath")"

    # echo $DEVPATH
    # echo $DEVNAME
    [[ -z "$DEVPATH" ]] && exit
    [[ -z "$DEVNAME" ]] && exit

    echo "/dev/$devname - $DEVPATH - $DEVNAME"
)
done