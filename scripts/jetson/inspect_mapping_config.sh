#!/usr/bin/env bash
cat "$HOME/yahboomcar_ws/src/yahboomcar_nav/launch/yahboomcar_map.launch"
cat "$HOME/yahboomcar_ws/src/yahboomcar_nav/launch/yahboomcar_navigation.launch"
find "$HOME/yahboomcar_ws/src" -iname '*gmapping*' -o -iname '*cartographer*' -o -iname '*hector*'
ls "$HOME/miniforge3/envs/robonix/bin/python"
