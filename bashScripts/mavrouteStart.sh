#!/bin/bash
until mavlink-routerd ; do
echo "Server 'myserver' crashed with exit code $?.  Respawning.." >&2
sleep 1
done