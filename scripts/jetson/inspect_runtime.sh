#!/usr/bin/env bash
set -euo pipefail
export LC_ALL=C.UTF-8
echo 'ROSBRIDGE LAUNCH'
cat /opt/ros/melodic/share/rosbridge_server/launch/rosbridge_websocket.launch
echo 'BINDING IMPLEMENTATION'
grep -nE 'address|listenWS|listenTCP' /opt/ros/melodic/lib/rosbridge_server/rosbridge_websocket
echo 'CURRENT ROBONIX API'
grep -nE 'def mcp|mcp =|def __init__|def provides_mcp' "$HOME/refs/robonix/pylib/robonix-api/robonix_api/capability.py"
echo 'COMPILE STATE'
ps -eo pid,etime,args | grep -E '[c]argo build|[r]ustc --crate-name' | head -8
