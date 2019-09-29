#!/bin/bash
./zenith &
sleep 15
pkill zenith
kill -9 $(pidof hddldaemon autoboot)
