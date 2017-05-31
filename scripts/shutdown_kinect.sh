#!/bin/bash
echo '1-1' > /sys/bus/usb/drivers/usb/unbind; sleep 2.5; echo '1-1' > /sys/bus/usb/drivers/usb/unbind
