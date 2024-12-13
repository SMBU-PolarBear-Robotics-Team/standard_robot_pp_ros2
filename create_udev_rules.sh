#!/bin/bash

set -e

echo ""
echo "This script copies a udev rule to /etc/udev/rules.d to facilitate bringing up the ttyACM0 usb connection as /dev/ttyACM0"
echo ""

UDEV_RULES_FILE="/etc/udev/rules.d/99-RoboMaster_C_Board.rules"
UDEV_RULE='SUBSYSTEMS=="usb", KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", SYMLINK+="ttyACM0", MODE="0777"'

echo "Setting udev rules..."
echo "$UDEV_RULE" | sudo tee $UDEV_RULES_FILE > /dev/null

echo "Restarting udev..."
sudo service udev reload
sleep 2
sudo service udev restart

echo -e "\e[32mUdev rules have been set and restarted successfully.\e[0m"

CURRENT_USER=$(whoami)

echo ""
echo "Adding user $CURRENT_USER to dialout group..."
sudo usermod -aG dialout $CURRENT_USER

echo -e "\e[32mUser '$CURRENT_USER' has been added to the dialout group successfully\e[0m"