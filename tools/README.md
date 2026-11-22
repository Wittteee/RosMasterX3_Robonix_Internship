# 避障部署与实车验证辅助脚本

归档自本地工作区 `D:/Projects/Car/tools`，用于复核 2026-11-21 的功能1/功能2部署与测试流程。历史脚本保留当时的路径、阈值、校验和、姿态种子和试验编号，不是可按文件名依次执行的自动安装程序。

本次归档时合并了远端 `79af814` 等更新，当前主线已改为30cm触发侧绕。历史5cm启动器的校验和与阈值断言针对旧版本，不能直接用于当前主线；独立轮式航向 launch 只覆盖 EKF 航向输入，不覆盖当前主线的避障触发距离。复核旧试验代码请使用 `d2d7366`，不要把本目录名称视为当前运行阈值。

## SSH 工具

本机 Python 3 安装 `paramiko` 后，在仓库根目录使用：

```powershell
python tools/car_remote.py --help
python tools/car_remote.py run tools/car_brief_status.sh --timeout 30
```

默认连接 `jetson@10.206.170.35`，交互输入密码；也可通过 `CAR_SSH_HOST`、`CAR_SSH_USER`、`CAR_SSH_PASSWORD` 环境变量提供连接信息。仓库版本不引用本地 `ssh_probe.py`，不包含密码。主机首次连接使用 Paramiko AutoAddPolicy，与原辅助工具一致。

## 上次 5cm 实测涉及的文件

- 构建：`car_prepare_bypass.sh`、`car_build_bypass.sh`。
- 测试：`car_test_bypass.sh`、`car_verify_5cm_results.sh`。
- 预览：`car_start_5cm_preview.sh`、`car_check_5cm_preview.sh`。
- 实测：`car_preflight_5cm_motion.sh`、`car_arm_5cm_motion.sh`、`car_run_5cm_motion.sh`、`car_bypass_live_trial.py`、`car_finish_5cm_motion.sh`。
- 原始记录：`../validation_artifacts/obstacle2_20261121/hardware_5cm_01/`。

`run` 通过 Bash 在小车执行 shell 脚本；`python` 子命令使用远端 Python 3。ROS Melodic 的 rospy 监督脚本应由相应 shell 启动器用 Python 2 执行。脚本默认工作区为 `/home/jetson/rosmaster_x3_runtime/obstacle2_20261121`；部分启动器依赖已上传的 `live_trial.py`、`readiness.py` 等远端文件，使用前须按启动器引用的名字部署。

## 临时轮式航向配置

`car_start_5cm_preview.sh` 生成预览 launch，`car_arm_5cm_motion.sh` 基于它生成实测 launch。两者保留原历史生成过程。`trial_configs/wheel_heading_5cm.launch` 提取了相同的 EKF 覆盖配置：`imu0_config` 的15项全部为 false，保留原底盘配置中的轮式里程计测量。它是试验用 IMU 漂移规避措施，IMU 偏置尚未标定。

该独立 launch 的初始 x/y/yaw 必须显式提供，默认 `execute=false`。它是根据本地历史脚本整理的等价配置，不是从小车取回的原始 `trial.launch`；本次 SSH 连接超时，未取回该原始文件。

历史启动器含旧位姿种子、`execute=true` 的启动步骤，以及通过 confirm 服务实际发车的监督脚本。重复运行前需核对现场定位与障碍布局，换用新的试验目录，避免覆盖历史日志。仅运行构建或查看帮助不会发车。

上次5cm实测完成朝水瓶前进、近障停车和选择右绕；侧移后因 `BYPASS_SENSOR_UNAVAILABLE` 停止，尚未通过完整绕行验收。仓库仍需外部 ROS/厂商驱动及 YOLOX 模型，模型未包含在此次归档中。
