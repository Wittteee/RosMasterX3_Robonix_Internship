# ROSMaster X3 短距安全导航与视觉候选

业务主机通过 rosbridge 与车载 ROS Melodic 通信。一次任务遵循：

```text
候选 → 预览距离校验 → 短窗口健康检查 → 一次性令牌
     → 人工确认 → 单次下发 → 到达 / 取消 / 失败 → JSON 日志
```

本项目不实现速度控制器，不重写 bringup、地图、AMCL 或 move_base。
业务侧只允许发布配置的 goal/cancel，禁止 `/cmd_vel` 及其命名空间变体。
**预览、确认和下发分离；未确认不会下发，默认不下发。**

## 环境与安装

- 业务主机：Python **3.10** + Conda，Windows/Linux。也支持 venv。
- 车载：Jetson、Ubuntu 18.04、系统 ROS Melodic，保留系统 ROS Python。
- 官方二维码执行路径额外要求 safe_move Provider、Robonix API 与生成绑定已安装。
- 离线流程不需要 ROS 安装、机器人或网络连接；pip 依赖需要预先安装。

在本仓库根目录执行：

```bash
conda create -n x3-nav python=3.10 pip -y
conda activate x3-nav
python -X utf8 -m pip install -r requirements-dev.txt
python -X utf8 -m pytest -q
python -X utf8 scripts/self_check.py
```

`requirements.txt` 是基础导航依赖；`requirements-vision.txt` 增加 OpenCV/NumPy；
`requirements-dev.txt` 再增加 pytest。只运行基础导航不必安装视觉依赖。

本机已有 `.venv` 时，Windows 可直接用：

```powershell
.\.venv\Scripts\python.exe -X utf8 -m pytest -q
.\.venv\Scripts\python.exe -X utf8 scripts\self_check.py
```

当前开发验证环境是 Windows/Python 3.12。Python 3.10 的运行测试由 CI 配置覆盖，
只有 CI 实际执行通过后才能认定该环境已验证；本轮未在 Jetson 上运行。

## 真实目标检测（YOLOX）

已接入 OpenCV DNN + YOLOX-s，支持照片/摄像头中的 COCO 80 类目标。
检测默认只输出标注与 JSON；摄像头目标方向可进入已有官方候选与人工确认流程。
安装、下载权重及运行方法见 [目标检测说明](docs/object_detection.md)。

```bash
python scripts/download_yolox.py
python scripts/run_object_detection.py --image logs/demo_20261028/camera_current.jpg
```

## 无实车完整演练

```bash
# 准备令牌后退出：PREPARE，没有任何目标发布
python scripts/run_mission.py --dry-run --distance 0.30
# 仅确认：CONFIRM，令牌尚未消费，不发布目标
python scripts/run_mission.py --dry-run --distance 0.30 --confirm
# 完整模拟到达：SUCCEEDED，向内存桥接发布一次目标
python scripts/run_mission.py --dry-run --distance 0.30 --confirm --dispatch
# 主动中止实验：达到 8 cm 模拟位移后取消，等待 PREEMPTED
python scripts/run_mission.py --dry-run --distance 0.30 --confirm --dispatch --auto-preempt
# 检测框候选：默认离线，--mock 明确标记模拟检测
python scripts/run_detect_nav.py --mock --offset-x 0.15 --size 0.22 --confirm --dispatch
# 二维码解码和官方协议模拟；execute 仍然不会连接实车
python scripts/run_visual_nav.py --demo
python scripts/run_visual_nav.py --demo --execute
# 仅模拟取消消息
python scripts/run_cancel.py --dry-run
```

二维码完整模拟需要输入“确认执行”。`--demo` 不能与真实 `--provider-dir` 同用。
`--dispatch` 不带 `--confirm` 会在连接之前被拒绝。
每个 runner/controller 只处理一个任务，不允许复用令牌或在同一个实例中切换 dry/live。
`--auto-preempt` 是主动中止测试，与到点实验分别运行；不要打开它测量完整移动距离。

真实生成的模拟样例见 [examples](examples/README.md)。所有模拟记录标注 dry-run/simulation，
不作为实车成功或物理停车的证据。

## 配置

默认 `configs/robot.yaml`，支持入口参数 `--config path/to/robot.yaml`。

| 字段 | 默认 | 单位 / 作用 |
|---|---|---|
| robot.host / rosbridge_port | 127.0.0.1 / 9090 | WebSocket 连接；支持 SSH 隧道 |
| topics.scan / odom / amcl_pose | /scan / /odom / /amcl_pose | 必需数据源 |
| topics.move_base_status | /move_base/status | 状态，健康检查时可暂时缺失 |
| topics.move_base_goal | /move_base_simple/goal | 唯一目标发布口，PoseStamped |
| topics.move_base_cancel | /move_base/cancel | 唯一取消口，空 id GoalID |
| safety.candidate_distance_m | 0.80 | 米；可下调，不可超过硬上限 |
| safety.preempt_trigger_displacement_m | 0.080 | 米；只在 --auto-preempt 时使用 |
| safety.forbidden_publish | [cmd_vel] | 配置必须包含；首尾斜杠等价 |
| mission.token_ttl_sec | 60 | 秒；单调时钟验证令牌期限 |
| mission.default_timeout_sec | 120 | 秒；监控超时会请求取消并 FAILED |
| mission.frame_id | map | 候选与定位必须同坐标系 |
| mission.health_duration_sec | 2 | 秒；统计传感器消息数量 |
| mission.pose_max_age_sec | 2 | 秒；按本机收到有效位姿的时间计算 |
| mission.preview_motion_tolerance_m | 0.03 | 米；预览前后位置偏移拒绝阈值 |
| mission.cancel_timeout_sec | 3 | 秒；等待可归属的终态 |
| mission.poll_interval_sec | 0.1 | 秒 |
| vision.target_label / min_confidence | person / 0.50 | 检测筛选 |
| vision.min_approach_m / max_approach_m | 0.20 / 0.80 | 米；启发式候选步长范围 |
| vision.max_yaw_deg | 30 | 度；候选偏航夹紧 |
| vision.detections_topic | /darknet_ros/bounding_boxes | 真实检测订阅扩展点，当前未实现 YOLO 订阅 |

缺少必填段、必需话题、cmd_vel 黑名单或非法数值时启动失败；视觉段可缺省。
黑名单之外还限制发布白名单与消息类型，不能通过重命名 goal 为 cmd_vel 绕过。
业务配置不可关闭人工确认或一次性令牌。不要把候选位移解释为物体距离。

## 实车连接与手工候选

先确认当前地图、初始化位姿、底盘通信、雷达、TF 和车载导航正常。
本程序不替你启动底盘或建图，不使用历史旧地图推断现场。

```bash
# 独立终端，按实际用户名和地址替换
ssh -N -L 9090:127.0.0.1:9090 jetson@JETSON_IP
# 车载已启动 ROS 和导航时，启动 rosbridge（按现场部署）
roslaunch rosbridge_server rosbridge_websocket.launch port:=9090
```

业务主机另一个终端：

```bash
python scripts/check_bridge.py
python scripts/check_topics.py --duration 3
# 当前定位前方 30 cm，仅准备
python scripts/run_mission.py --distance 0.30
# 只确认，仍不下发
python scripts/run_mission.py --distance 0.30 --confirm
# 明确允许实际执行（现场看护与急停就绪后）
python scripts/run_mission.py --distance 0.30 --confirm --dispatch
# 也可用同坐标系绝对候选；x/y/yaw 是目标，不是位移
python scripts/run_mission.py --x 1.2 --y 0.4 --yaw 0 --confirm
# 取消当前 move_base 任务；仅请求取消，不等于已物理停止
python scripts/run_cancel.py
```

`--distance` 使用本次实际位姿和朝向；缺少位姿时拒绝，不默认为地图原点。
只在同坐标系内比较距离；不会把 odom 坐标混入 map。
空 id cancel 会影响当前 move_base 任务，因此每辆车只运行一个任务客户端。
运行中 Ctrl+C 请求取消，观察到 PREEMPTED/RECALLED 后才记为 PREEMPTED；
没有取消确认或连接断开则 FAILED，并保留取消错误。

## 视觉两条路径

**A：检测框提出短步候选。** `Detection` 经过标签、置信度、有限值检查；
图像右方映射 ROS 负偏航；相对大小映射启发式步长，再由公共 MissionRunner 复核距离。
只计算候选，不宣称二维码/物体的世界坐标。没有 YOLO 模型下载或实时检测话题订阅。
CLI 注入的现场观察可显式执行：

```bash
python scripts/run_detect_nav.py --live --label person --offset-x 0.1 --size 0.5 --confidence 0.9 --confirm
# 审阅并具备实车条件后才加 --dispatch
```

这条路径采用本机授权语义，不提供官方服务端的 costmap/make_plan 预览证明。
它与原先被禁用的视觉模块直接执行入口不同：执行统一由 mission 层负责。
Mock 与 live 是互斥模式，不自动把模拟检测换成真实下发。

**B：二维码筛选官方候选。** 保留所有同标签检测；需要一个目标方向时，重复标记会拒绝。
使用实际相机标定和新帧，调用官方预览、prepare 与 execute；服务不可用直接失败，
不退回路径 A 冒充官方安全候选。官方执行流程也使用公共 SafetyController 管理本机授权。

部署生成绑定与完整命令见 [官方运行说明](docs/06_视觉导航运行与验收.md)。
真实入口新增 `--config` 配置 rosbridge 地址。参考 Provider 固定 map 与标准 ROS 话题，
不支持的改名会明确拒绝，不静默忽略。执行前复查目标；运动中由官方 Provider 监控导航，
不是连续视觉跟随器。

合并远程主分支后，也保留 `--observation-file path/to/observation.json` 入口：
读取真实相机观测文件，只把唯一、居中的新鲜二维码作为前方候选触发条件；每次复查重新读取文件。
此模式仍需 `--provider-dir`，不把居中判定当成标定测距；过期观测明确拒绝。
远程新增的停车候选、底盘映射脚本及 `logs/demo_20261028/` 实车记录保持保留，
本次合并未运行这些车载运动脚本。

## 结果、日志与能力边界

- `PREPARE`：健康检查通过并签发令牌；`CONFIRM`：进入人工确认阶段，可已确认但未下发。
- `SUCCEEDED`：观察到关联任务的导航成功；不等于独立尺量或物理停止验证。
- `PREEMPTED`：观察到关联任务取消终态；保护性中止不是到点成功。
- `FAILED`：拒绝、异常、超时、终态未确认等；超时不会只退出而不请求取消。
- JSON 在 `logs/`，包含 mode、health、snapshot、状态名、detail、log_path、取消结果。
  不保存明文令牌。文件名唯一，不因同秒多次运行覆盖。

短窗口健康检查只统计 count/hz；不能证明传感器数值正确、长期定位不漂移或 TF 完整。
预览前后检查位移，但不能保证其他程序没有驱动车辆。
PoseStamped 不携带客户端 action goal_id：本程序绑定 dispatch 后出现的新 PENDING/ACTIVE id，
忽略旧 id 与未观察到接受阶段的终态；多个新任务时失败并取消。必须保证单客户端。
如果 ROS 丢失接受阶段消息，程序宁可超时失败，不用旧 SUCCEEDED 假报成功。

## 故障排查

- **30 cm 只走 2 cm**：先看终态、reason、取消原因、是否启用 auto-preempt；
  若报告 SUCCEEDED 再查局部规划器到点容差及里程计/AMCL 实际变化。不要单纯放大目标补偿。
- **底盘默认版本 -1 / 电压 0**：检查供电、控制板接线、端口和协议；驱动发布零里程计不代表正常。
- **串口被占用**：MakerControl、厂商驱动与调试脚本不能同时占用控制板。
- **Python 版本分裂**：ROS Melodic 保留系统 Python；本业务和官方绑定使用独立 3.10 环境，
  不将 ROS 系统包或二进制 OpenCV 复制进 Conda。
- **多源抢速度/任务**：停用手柄或其他导航客户端，明确唯一导航控制源；本项目不会发布零速度抢控制。
- **rosbridge 连接失败**：检查 SSH 转发、端口、车上节点；连接等待限时约 10 s。
- **目标未到就结束**：查看新日志中的目标 id、原始失败状态 `status_before_cancel`、取消确认。
- **官方 Provider 导入失败**：检查独立环境和生成绑定 PYTHONPATH；不自动安装或执行车上脚本。

## 目录与验收

```text
src/x3_nav/
  config.py       配置校验
  safety/         纯 Python 状态机，无通信依赖
  bridge/         受限 ROS 发布、官方适配、纯内存模拟
  nav/            消息编码、同坐标系位姿与状态监视
  validation/     短窗口话题统计
  mission/        手工、检测与二维码任务编排
  vision/         检测/二维码、方向与候选筛选，不执行运动
scripts/          参数解析与调用
 tests/           pytest 与兼容 unittest 用例
examples/         模拟日志与自检文本
```

[设计逐条对照](docs/08_完整授权链设计对照.md) · [本轮验证记录](docs/09_完整授权链验证记录.md)

`docs/00`–`07` 是早期阶段记录；以本 README 与 08/09 为当前实现说明。
实车位置精度、停止距离、底盘恢复、地图、实际相机和重复实验仍需现场验收。
