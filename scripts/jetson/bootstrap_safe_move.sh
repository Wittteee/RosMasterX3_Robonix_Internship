#!/usr/bin/env bash
set -euo pipefail
export LC_ALL=C.UTF-8
task_cache="$HOME/.cache/x3_safe_move_setup"
mkdir -p "$task_cache"

# The 2023 installer supports this Jetson's Ubuntu 18.04 / glibc 2.27.
if [[ ! -x "$HOME/miniforge3/bin/conda" ]]; then
    task_asset=Miniforge3-23.3.1-0-Linux-aarch64.sh
    task_url="https://github.com/conda-forge/miniforge/releases/download/23.3.1-0/$task_asset"
    curl -fL --retry 2 --connect-timeout 15 "$task_url" -o "$task_cache/$task_asset"
    curl -fL --retry 2 --connect-timeout 15 "$task_url.sha256" -o "$task_cache/$task_asset.sha256"
    (cd "$task_cache" && sha256sum -c "$task_asset.sha256")
    bash "$task_cache/$task_asset" -b -p "$HOME/miniforge3"
fi
if [[ ! -x "$HOME/miniforge3/envs/robonix/bin/python" ]]; then
    "$HOME/miniforge3/bin/mamba" create -y -n robonix python=3.10 pip
fi
if [[ ! -d "$HOME/refs/robonix/.git" ]]; then
    git clone --depth 1 https://github.com/syswonder/robonix.git "$HOME/refs/robonix"
fi
git -C "$HOME/refs/robonix" submodule update --init --recursive -- capabilities/lib/common_interfaces capabilities/lib/rcl_interfaces
if git -C "$HOME/refs/robonix" ls-files --stage capabilities/lib/unique_identifier_msgs | grep -q '^160000'; then
    git -C "$HOME/refs/robonix" submodule update --init --recursive -- capabilities/lib/unique_identifier_msgs
fi
if [[ ! -x "$HOME/.cargo/bin/cargo" ]]; then
    curl --proto '=https' --tlsv1.2 -fL --retry 2 https://sh.rustup.rs -o "$task_cache/rustup-init.sh"
    bash "$task_cache/rustup-init.sh" -y --profile minimal --no-modify-path
fi
"$HOME/miniforge3/envs/robonix/bin/python" --version
"$HOME/.cargo/bin/cargo" --version
git -C "$HOME/refs/robonix" rev-parse HEAD
