#!/bin/bash

# Copy our service to systemd
cp systemd/nrc.service /etc/systemd/system/

# Add executable status to our startup just in case
chmod +x systemd/nrc_start.sh

# Enable our service to run on boot
systemctl enable nrc