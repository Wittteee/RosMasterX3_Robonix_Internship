# ROSMaster X3 短距安全导航与 RGB-D 目标导航

面向 Yahboom ROSMASTER X3 的导航实验项目，包含业务主机侧 Python 授权流程、
YOLOX/二维码视觉候选，以及 Jetson 上的 ROS1 RGB-D 静态目标导航。
Python 业务主机通过 rosbridge 与车载 ROS Melodic 通信，一次任务遵循：

```text
候选 → 预览距离校验 → 短窗口健康检查 → 一次性令牌
     → 人工确认 → 单次下发 → 到达 / 取消 / 失败 → JSON 日志
```

Python 业务侧只允许发布配置的 goal/cancel，禁止 `/cmd_vel` 及其命名空间变体。
ROS1 包另有直线接近控制器，经 `/target_navigation/approach_velocity` 和带运动许可
心跳的底盘驱动执行；绕行模式结合 move_base 规划。仓库也包含车载启动配置和底盘修正脚本，
因此业务侧的发布限制不能理解为整个仓库都不产生速度指令。
**预览、确认和下发分离；未确认不会下发，默认不下发。**

## 功能与入口

| 使用场景 | 入口 | 默认行为 / 执行条件 |
|---|---|---|
| 无机器人演练授权流程 | `scripts/run_mission.py --dry-run` | 内存模拟；`--confirm --dispatch` 也不连接实车 |
| rosbridge 手工短距导航 | `scripts/run_mission.py` | 仅准备；实际下发需 `--confirm --dispatch` |
| 检测框启发式候选 | `scripts/run_detect_nav.py` | 默认离线；实车需显式 `--live` 及确认、下发参数 |
| 照片 / 摄像头目标检测 | `scripts/run_object_detection.py` | 输出标注和 JSON，不驱动车辆 |
| 二维码 / YOLOX 官方候选 | `scripts/run_visual_nav.py` | 依赖官方 Provider；执行需 `--execute` 和交互确认 |
| RGB-D 直线接近、遇障停车（功能 1） | `target_navigation.launch` | 默认预览；`execute:=true` 后还需确认服务 |
| RGB-D 局部遮挡绕行（功能 2） | `bottle_bypass.launch` | 默认预览；停车后规划侧移、通过和接近路径 |

Python 命令均从仓库根目录运行；ROS launch 需先在车载 Catkin 工作空间构建并 source。
RGB-D 路径使用 ROS 服务确认，不使用 Python CLI 的 `--confirm` / `--dispatch` 参数。

## 软件包目录

按 [软件包目录发布流程](https://book.robonix.ai/integration-guide/package-catalog)
登记为技能软件包 `robonix.skill.rgbd_target_navigation`。
目录只读取默认分支根目录的 `package_manifest.yaml`。
`scripts/build.sh` 跑离线自检；`scripts/start.sh` 只做不下发的 dry-run。
实车 RGB-D 入口仍是 `ros1/x3_target_navigation` 的 launch，且默认不执行。

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

也可以在 Windows PowerShell 中创建独立 venv，无需激活脚本：

```powershell
py -3.10 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements-dev.txt
.\.venv\Scripts\python.exe -X utf8 scripts\run_mission.py --dry-run --distance 0.30
.\.venv\Scripts\python.exe -X utf8 -m pytest -q
.\.venv\Scripts\python.exe -X utf8 scripts\self_check.py
```

上述命令要求已安装 Python 3.10。Linux 的 venv 可用 `python3.10 -m venv .venv`，
随后使用 `.venv/bin/python` 执行同样的命令。CLI 会自行加载仓库内的 `src`，无需 `pip install -e .`。

`requirements.txt` 是基础导航依赖；`requirements-vision.txt` 增加 OpenCV/NumPy；
`requirements-dev.txt` 再增加 pytest；`requirements-model-tools.txt` 仅用于生成
Jetson OpenCV 4.1 兼容的 ONNX 副本。只运行基础导航不必安装视觉依赖。

本机已有 `.venv` 时，Windows 可直接用：

```powershell
.\.venv\Scripts\python.exe -X utf8 -m pytest -q
.\.venv\Scripts\python.exe -X utf8 scripts\self_check.py
```

CI 配置见 [offline-tests](.github/workflows/tests.yml)，在 Ubuntu 上覆盖 Python 3.10 和 3.12，
执行 pytest 与离线自检。配置存在不代表当前提交已通过 CI；历史验证结果见对应日期的文档。
Python 离线测试不替代 Jetson Catkin 构建、ROS 集成测试或实车验收。

## 真实目标检测（YOLOX）

已接入 OpenCV DNN + YOLOX-s，支持照片/摄像头中的 COCO 80 类目标。
检测默认只输出标注与 JSON；摄像头目标方向可进入已有官方候选与人工确认流程。
安装、下载权重及运行方法见 [目标检测说明](docs/object_detection.md)。

```bash
python scripts/download_yolox.py
python scripts/run_object_detection.py --image logs/demo_20261028/camera_current.jpg
```

## ROS1 RGB-D 静态目标导航

[`ros1/x3_target_navigation`](ros1/x3_target_navigation/README.md) 是 C++ Catkin 包，
面向 Jetson ROS Melodic 与 Astra Pro。它同步 RGB、注册深度和相机内参，
将唯一 YOLOX 目标按图像时间戳转换到 `map`，经多帧融合后生成安全停车点。
只支持静态、唯一的同类目标；到达后需要新鲜 RGB-D 观测复核。

| 模式 | 选择方式 | 控制与遇障行为 |
|---|---|---|
| 功能 1：直线接近 | `target_navigation.launch` 默认 `obstacle_stop_approach:=true` | 自有低速接近控制器；路径内障碍实测前向间隙 ≤ 5 cm 时发零速并锁止，确认停止后报告 `FAILED: BLOCKED_NO_SAFE_PATH` |
| 功能 2：局部遮挡绕行 | `bottle_bypass.launch` | 同一 5 cm 触发后先停车；经停止确认和路径校验后，借助 move_base 分段绕过，仍朝向原先冻结的瓶子目标 |
| 普通导航 | `target_navigation.launch obstacle_stop_approach:=false` | 使用 move_base；障碍可能使停车候选保守截断，受阻任务不报告成功 |

功能 1 不启动 move_base；功能 2 将其速度输出先交给任务管理器检查，再转发到接近控制话题。
5 cm 是**发出停车指令的测量阈值**，不保证最终静止间隙；采样、制动滑行和激光盲区仍需实测。
功能 1 的遇障锁止不会因障碍移走自动恢复。功能 2 对全宽封堵、方向不明确或无安全路径保持停止。

先按 [ROS1 包说明](ros1/x3_target_navigation/README.md) 构建、准备兼容模型及设备配置，
再在 Jetson 上启动预览。下面的地图名为历史演示地图，使用前须替换为现场有效地图；
三个 `start_*` 参数会启动相应设备或导航节点，应确保没有重复驱动：

```bash
source /opt/ros/melodic/setup.bash
source ~/software/library_ws/devel/setup.bash
source ~/yahboomcar_ws/devel/setup.bash
export ROBOT_TYPE=X3
roslaunch x3_target_navigation target_navigation.launch \
  start_bringup:=true start_astra:=true start_navigation:=true \
  map:=final_site_20261107 target_label:=bottle execute:=false
```

另一个已加载 ROS 环境的终端可查看：

```bash
rostopic echo /target_navigation/state
rostopic echo /target_navigation/preview_goal
```

预览模式拒绝确认服务。完成现场校验后，以 `execute:=true` 重新启动，审阅新预览，
再调用 `rosservice call /target_navigation/confirm`；取消使用
`rosservice call /target_navigation/cancel`。模型路径、内参、设备别名和 TF 等部署要求
见包内说明。旧 Provider/二维码入口保持可用。

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

Python 业务流程默认使用 `configs/robot.yaml`，支持入口参数 `--config path/to/robot.yaml`。
ROS1 RGB-D 使用独立的 [`astra_pro.yaml`](ros1/x3_target_navigation/config/astra_pro.yaml)，
由 launch 的 `config` 参数加载；下表不适用于 ROS1 的接近或绕行控制器。

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
此入口只计算候选，不宣称二维码/物体的世界坐标，也不自动加载 YOLO 或订阅实时检测话题。
独立的 YOLOX 检测入口及模型下载工具见上文“真实目标检测”。
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

二维码流程也支持 `--observation-file path/to/observation.json` 入口：
读取真实相机观测文件，只把唯一、居中的新鲜二维码作为前方候选触发条件；每次复查重新读取文件。
此模式仍需 `--provider-dir`，不把居中判定当成标定测距；过期观测明确拒绝。
停车候选、底盘映射与车载诊断脚本位于 `scripts/jetson/`；
`logs/demo_20261028/` 保存历史实车记录。该目录包含可实际驱动车辆的实验脚本，
使用时应阅读各脚本参数和对应实验记录，不能将其视为离线自检入口。

## 结果、日志与能力边界

以下状态与 JSON 字段描述 Python 业务流程；ROS1 状态通过 `/target_navigation/state`
发布，还包含 `NAVIGATING`、`STOPPING` 等阶段，详见包内说明。

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
- **多源抢速度/任务**：停用手柄或其他导航客户端，明确唯一导航控制源；Python 业务侧不会发布零速度抢控制，ROS1 接近模式则要求底盘输入和运动许可按 launch 配套配置。
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
ros1/x3_target_navigation/  C++ RGB-D、融合、任务管理、launch 与 ROS 测试
scripts/          Python CLI、模型下载、转换与离线自检
  jetson/         车载部署、底盘诊断、标定与运动实验
  ros1/           RGB-D 标定、指标采集与验收辅助脚本
configs/          Python 业务配置、SSH 配置示例与底盘标定
tests/            pytest、unittest 与 C++ 几何回归用例
examples/         模拟日志与自检文本
docs/             设计说明、部署指南及按日期保存的实验记录
logs/             历史实验记录与运行生成日志
validation_artifacts/  验证产物
package_manifest.yaml  Robonix 软件包登记与构建 / 启动入口
```

## 文档与验证

| 文档 | 内容 |
|---|---|
| [完整授权链设计](docs/08_完整授权链设计对照.md) | Python 业务侧的预览、授权、下发与取消约束 |
| [授权链验证记录](docs/09_完整授权链验证记录.md) | 对应阶段的离线验证证据 |
| [YOLOX 目标检测](docs/object_detection.md) | 权重下载、图片检测、相机标定与官方候选接入 |
| [官方视觉导航](docs/06_视觉导航运行与验收.md) | Provider、生成绑定及二维码运行流程 |
| [ROS1 包说明](ros1/x3_target_navigation/README.md) | 当前 RGB-D 模式、控制话题、绕行和 ROS 集成测试 |
| [2026-11-15 部署记录](docs/17_20261115_ROS1_RGBD目标导航部署与验收.md) | 当日 Jetson、相机、地图和只读验收记录 |
| [模拟样例](examples/README.md) | 不连接实车的运行输出 |

历史文档中的“本轮”、测试数量、IP、地图和设备参数仅对应其记录时点。
当前入口与默认值应结合本 README、ROS1 包说明及实际配置阅读。
实车位置精度、停止距离、底盘恢复、相机标定及左右绕行能力仍需现场重复验收，
不能由离线测试通过推断。

## 许可证

本项目采用 [Apache License 2.0](LICENSE)。YOLOX 权重来源与校验方式见
[目标检测说明](docs/object_detection.md)，模型需单独下载。
