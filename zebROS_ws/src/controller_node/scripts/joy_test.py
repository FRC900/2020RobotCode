#!/usr/bin/env python

import os
import psutil
import subprocess
import syslog
import sys

def x_joystick_plugged():
    """                         
    @return: True if joystick's mountpoint found.
    @rtype: bool
    """
    PATH_JOYSTICK = sys.argv[1]
    # https://stackoverflow.com/a/39124749/577001
    mount_points = {el.mountpoint: el for el in psutil.disk_partitions(all=True)}
    try:
        partition_joystick = mount_points[PATH_JOYSTICK][0]
        print("Joystick found at {}".format(PATH_JOYSTICK))
    except KeyError as e:
        syslog.syslog("x joystick's mount point {} not found.".format(PATH_JOYSTICK))
        syslog.syslog(str(e))
        print("Joystick not found at {}".format(PATH_JOYSTICK))

if __name__ == "__main__":
    x_joystick_plugged()
