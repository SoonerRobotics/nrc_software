# nrc_software
Software for the NRC AVC 2019/20 robot

# Setup

## Dependencies

This setup assumes and has only been tested on a Raspberry Pi 3 running Ubuntu 18.04.3 with [ROS Melodic](http://wiki.ros.org/melodic) installed.

**You must clone this repo into /home/nrc/ for the setup to work. The scripts assume that /home/nrc/nrc_software exists. This will (hopefully but probably not) be improved in the future.**

## Run on boot

To setup running on boot, run `sudo ./setup` from within the `setup` directory. This will copy `setup/systemd/nrc.service` into the `/etc/systemd/system/` directory to create the `nrc` service. It will also run `systemctl enable nrc` which enables the `nrc` service to run on startup.

## Configuring startup

On boot, `setup/systemd/nrc_start.sh` will be ran. Modify this as needed.

## The `nrc` service

The `nrc` service is a [systemd](https://www.freedesktop.org/software/systemd/man/systemd.service.html) service unit so it can controlled using `service` or `systemctl`.

Common use cases:

`sudo service nrc start` to start the service.

`sudo service nrc stop` to stop the service.

`sudo service nrc restart` to restart the service.

`sudo service nrc status` to view the active status of the service.

`sudo systemctl enable nrc` to enable the service running on startup.

`sudo systemctl disable nrc` to disable the service running on startup.

`sudo journalctl -e -t nrc_logs` to view logs.