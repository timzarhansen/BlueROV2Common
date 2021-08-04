#!/bin/bash
until mavlink-routerd /dev/ttyACM0:1500000 ; do
echo "Server 'myserver' crashed with exit code $?.  Respawning.." >&2
sleep 1
done