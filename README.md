# ROSMASTER X3 × Robonix 安全导航（本机开发仓库）

## 当前入口（2026-10-20）

按阶段汇报补充了二维码驱动的官方安全导航流程：
**底盘回读检查 → 相机识别 → 服务端候选推荐 → 服务端令牌 → 人工确认 → 再检查 → 执行结果 / 取消请求**。
默认只预览；所有模拟输出标记为 `simulation`。实车底盘、当前地图、相机标定和到点结果尚待现场验收。

Windows 本机快速运行（在本仓库目录）：

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -X utf8 -m pip install -r requirements-vision.txt
.\.venv\Scripts\python.exe -X utf8 -m unittest discover -s tests -v
.\.venv\Scripts\python.exe -X utf8 scripts\run_visual_nav.py --demo
# 完整模拟交互：输入“确认执行”，只调用模拟 Provider，不连接机器人
.\.venv\Scripts\python.exe -X utf8 scripts\run_visual_nav.py --demo --execute
```

真实相机和官方 Provider 的启动方式、异常处理、验收表见
[06_视觉导航运行与验收.md](docs/06_视觉导航运行与验收.md)。
新入口是 `scripts/run_visual_nav.py`；旧 `run_detect_nav.py` 仅保留离线演示，禁止 `--dispatch`。
下文和 `00`–`05` 文档保留早期开发记录，其中旧的本地坐标/令牌方案不是最终视觉课题入口。

东华大学专业实习：本机用 **Conda** 写代码，SSH 连 Jetson，经 rosbridge 完成带安全管控的点位导航联调。

## 安装环境（Conda）

```powershell
cd D:\RosMasterX3_Robonix_Internship

conda create -p .\conda_env python=3.10 pip -y `
  -c https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/main `
  -c https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/r `
  --override-channels

conda activate D:\RosMasterX3_Robonix_Internship\conda_env
python -m pip install -r requirements.txt
python scripts\self_check.py
```

详细说明见：[docs/01_环境与SSH说明.md](docs/01_环境与SSH说明.md)

## 快速联调

```powershell
conda activate D:\RosMasterX3_Robonix_Internship\conda_env
# 另开终端：ssh -L 9090:127.0.0.1:9090 user@jetson-ip
python scripts\check_bridge.py
python scripts\check_topics.py
python scripts\run_mission.py
# 拓展课题：检测驱动短距导航（默认 Mock 干跑）
python scripts\run_detect_nav.py --mock
```

## 文档

- [总体实验与主题排序](docs/00_总体实验与主题排序.md)
- [环境与 SSH（含 Conda 安装命令）](docs/01_环境与SSH说明.md)
- [主实验验收清单](docs/02_主实验验收清单.md)
- [目标检测驱动导航（拓展）](docs/03_目标检测驱动导航.md)
- [safe_move 仓库部署与测试](docs/04_safe_move仓库部署与测试.md)

## 早期代码结构

```text
src/x3_nav/
  bridge/      rosbridge 连接 + 禁止 /cmd_vel
  safety/      预览→令牌→确认→执行→PREEMPTED/SUCCEEDED
  nav/         goal/cancel + status 监控
  validation/  话题频率/健康检查
  mission/     完整任务编排（主实验闭环）
  vision/      检测→短距候选点（拓展课题）
scripts/
  self_check.py      不连车自检
  check_bridge.py    连通性
  check_topics.py    数据有效性
  run_preview.py     状态机干跑
  run_mission.py     完整任务
  run_cancel.py      安全取消
  run_detect_nav.py  检测驱动导航（Mock/实车）
```

## 安全红线

Provider / 本机业务：**只发 goal / cancel，只读状态；禁止发 `/cmd_vel`。**
