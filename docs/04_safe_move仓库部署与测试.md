# Robonix short-distance safe_move：拉仓、上车、测试

本文保留首次检查时的状态。后续部署与目标检测接入说明见
[05_官方导航与目标检测接入.md](05_官方导航与目标检测接入.md)。
系统 Python 仍为 ROS 原有版本，导航 Provider 已使用独立 Python 3.10 环境。

参考：

- [syswonder/robonix](https://github.com/syswonder/robonix)
- [luoyg0831-a11y/robonix-rosmaster-x3-safe-move](https://github.com/luoyg0831-a11y/robonix-rosmaster-x3-safe-move)
- Catalog：[robonix.robot.yahboom.rosmaster_x3](https://packages.robonix.ai/robots/robonix.robot.yahboom.rosmaster_x3/)

本机已下载到：

```text
refs/robonix/
refs/robonix-rosmaster-x3-safe-move/
```

小车上已克隆到：

```text
/home/jetson/refs/robonix-rosmaster-x3-safe-move/
```

---

## 1. 这个仓库在干什么

`safe_move` 不是亚博 bringup 本身，而是 **Robonix 侧短距安全导航 primitive**：

```text
move_base CLI
  → preview_move_options（AMCL + costmap + make_plan，最多 5 个候选，≤0.80 m，前方 ±30°）
  → prepare_selected_move（一次性令牌）
  → 人工输入「确认执行」
  → execute_prepared_navigation（再验一次，只发一次 PoseStamped）
  → X3Bridge（rosbridge）→ ROS1 move_base
```

红线：上层 **不发 `/cmd_vel`**；速度只由车上 `move_base` 产生。

核心代码：

| 路径 | 作用 |
|---|---|
| `jetson/main.py` | Robonix capability / Provider |
| `jetson/x3_bridge.py` | rosbridge 读写、发 goal、监控、cancel |
| `jetson/scripts/move_base_cli.py` | 交互 CLI |
| `jetson/scripts/move_base` | 激活 `robonix` conda 后启动 CLI |

官方期望的车上目录（见 `docs/deployment-map.md`）：

```text
/home/jetson/rosmaster_x3_deploy/
/home/jetson/x3_rosbridge_adapter/
/home/jetson/miniforge3/   # conda env: robonix (Python 3.10)
```

---

## 2. 当前这台车（10.206.9.102）差距

| 项 | 状态 |
|---|---|
| 仓库源码 | 已在 `~/refs/robonix-rosmaster-x3-safe-move` |
| `/dev/myserial` | 曾出现（底盘 CH340） |
| 亚博 `yahboomcar_ws` + 导航 launch | 有 |
| `ros-melodic-rosbridge-server` | 已安装 |
| `~/rosmaster_x3_deploy` | **无** |
| `~/x3_rosbridge_adapter` | **无** |
| `miniforge3` / `robonix` conda / `rbnx` | **无** |
| 系统 Python | **3.6.9**（跑不动官方 `from __future__ import annotations` 代码） |
| `roscore` / `9090` | 探测时 **未在跑** |

结论：不能直接敲官方的 `move_base`；要先补 **ROS 导航栈 + rosbridge**，再二选一接 Provider。

---

## 3. 推荐落地路径（两条）

### 路径 A（实习最快）：本仓 `x3_nav` + 车上原生 ROS

与 `safe_move` 同构：预览 → 令牌 → 确认 → goal/cancel。

**车上（SSH）：**

```bash
# 终端 1：底盘
source /opt/ros/melodic/setup.bash
source ~/yahboomcar_ws/devel/setup.bash
export ROS_MASTER_URI=http://10.206.9.102:11311
export ROS_IP=10.206.9.102
roslaunch yahboomcar_bringup bringup.launch

# 终端 2：导航（按实验室地图改 map 参数）
roslaunch yahboomcar_nav yahboomcar_navigation.launch

# 终端 3：rosbridge
roslaunch rosbridge_server rosbridge_websocket.launch port:=9090
```

**本机：**

```powershell
ssh -L 9090:127.0.0.1:9090 jetson@10.206.9.102
conda activate D:\RosMasterX3_Robonix_Internship\conda_env
python scripts\check_bridge.py
python scripts\check_topics.py
python scripts\run_mission.py                 # 停在准备阶段
python scripts\run_mission.py --confirm --dispatch   # 实车，需清场
```

### 路径 B（对齐 Catalog 官方）：在车上装 Robonix 再跑 `move_base`

需要在 Jetson 上额外安装（工作量大，需指导教师确认）：

1. Miniforge + Python **3.10** 环境名 `robonix`
2. 安装 `rbnx` / Robonix API（见 [book.robonix.ai](https://book.robonix.ai/)）
3. 按 `deployment-map.md` 摆放：
   - `~/rosmaster_x3_deploy/primitives/rosmaster_x3_bridge/` ← `jetson/main.py` 等
   - `~/x3_rosbridge_adapter/scripts/x3_bridge.py`
   - `~/rosmaster_x3_deploy/scripts/move_base` + `move_base_cli.py`
4. `RBNX_PACKAGE_ROOT=... bash jetson/scripts/build.sh`（`rbnx codegen --mcp`）
5. 先起路径 A 的 ROS + rosbridge，再：

```bash
source ~/miniforge3/etc/profile.d/conda.sh
conda activate robonix
move_base --dry-run    # 无运动：列出候选
move_base              # 选编号 → 输入「确认执行」
```

---

## 4. 现在就能做的测试

### 4.1 本机离线单测（已验证）

```powershell
cd D:\RosMasterX3_Robonix_Internship\refs\robonix-rosmaster-x3-safe-move\jetson
python tests\move_base_cli_unit_test.py
# 输出：move_base_cli_unit_test: PASS
```

（`stage1_unit_test.py` 在 Windows/Py3.9 上可能有计时断言差异；官方 CI 以 Linux 为准。）

### 4.2 小车上（当前 Python 3.6 **跑不了**官方 CLI）

需先装 Python 3.10 / `robonix` 环境，否则会：

```text
SyntaxError: future feature annotations is not defined
```

### 4.3 联调验收清单（路径 A）

1. `rostopic list` 有 `/scan` `/odom` `/amcl_pose` `/move_base/status`
2. `ss -lntp | grep 9090` 有监听
3. 本机 `check_bridge` / `check_topics` 通过
4. `run_mission` 无 `--dispatch`：得到令牌、车不动
5. `--confirm --dispatch`：短距到点 `SUCCEEDED`；或 `--auto-preempt` 得 `PREEMPTED`

---

## 5. 和课题「目标检测驱动导航」的关系

`safe_move` 只负责 **安全短距导航**。检测驱动应叠在上面：

```text
检测结果 → 生成 ≤0.80 m 候选点 → 再走 preview/prepare/confirm/execute
```

本仓已有 Mock 版：`python scripts\run_detect_nav.py --mock`（见 `docs/03_目标检测驱动导航.md`）。
