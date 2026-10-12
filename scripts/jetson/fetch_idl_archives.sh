#!/usr/bin/env bash
set -euo pipefail
export LC_ALL=C.UTF-8
task_repo="$HOME/refs/robonix"

# Stop only the submodule downloads started by this deployment attempt.
while read -r task_pid task_command; do
    case "$task_command" in
        'git submodule update --init --recursive -- capabilities/lib/common_interfaces capabilities/lib/rcl_interfaces'|"/usr/lib/git-core/git clone --no-checkout --separate-git-dir $task_repo/.git/modules/"*|'/usr/lib/git-core/git-remote-https origin https://github.com/enkerewpo/common_interfaces'|'/usr/lib/git-core/git-remote-https origin https://github.com/enkerewpo/rcl_interfaces')
            kill -TERM "$task_pid" 2>/dev/null || true ;;
    esac
done < <(ps -u "$(id -u)" -o pid=,args=)

task_cache="$HOME/.cache/x3_safe_move_setup"
mkdir -p "$task_cache"
cd "$task_repo"
for task_spec in 'enkerewpo/common_interfaces common_interfaces' 'enkerewpo/rcl_interfaces rcl_interfaces' 'ros2/unique_identifier_msgs unique_identifier_msgs'; do
    read -r task_upstream task_name <<< "$task_spec"
    task_commit="$(git ls-tree HEAD "capabilities/lib/$task_name" | awk '{print $3}')"
    [[ -n "$task_commit" ]] || continue
    task_archive="$task_cache/$task_name-$task_commit.tar.gz"
    curl -fsSL --retry 2 --connect-timeout 15 --max-time 120 "https://codeload.github.com/$task_upstream/tar.gz/$task_commit" -o "$task_archive"
    tar -tzf "$task_archive" >/dev/null
    mkdir -p "capabilities/lib/$task_name"
    tar -xzf "$task_archive" --strip-components=1 -C "capabilities/lib/$task_name"
    echo "IDL source: $task_name $task_commit"
done
test -f capabilities/lib/common_interfaces/geometry_msgs/msg/Pose.msg
test -f capabilities/lib/common_interfaces/std_msgs/msg/String.msg
