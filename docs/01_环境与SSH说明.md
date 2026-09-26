# 环境配置（Conda）与 SSH 说明

## 角色分工

| 位置 | 你做什么 |
|---|---|
| **本机（Windows，本仓库）** | 用 **Conda** 写/跑 Python 联调代码；**不部署完整 ROS** |
| **车上 Jetson（Ubuntu 18.04 + Melodic）** | 跑 `roscore` / bringup / 导航栈 / rosbridge |

结论：本机只装 Conda 环境即可写代码；实车验收通过 **SSH + rosbridge** 联调。

---

## 一、本机 Conda 环境安装（必做）

工程路径：`D:\RosMasterX3_Robonix_Internship`  
环境安装在工程内：`D:\RosMasterX3_Robonix_Internship\conda_env`（Python 3.10，对齐 PPT 上层要求）

### 方式 A：推荐（国内镜像，更快更稳）

在 **Anaconda Prompt** 或已初始化 conda 的 PowerShell 中执行：

```powershell
cd D:\RosMasterX3_Robonix_Internship

conda create -p .\conda_env python=3.10 pip -y `
  -c https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/main `
  -c https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/r `
  --override-channels

conda activate D:\RosMasterX3_Robonix_Internship\conda_env
python -m pip install -U pip
python -m pip install -r requirements.txt

python -c "import sys, roslibpy, yaml; print(sys.version); print('roslibpy', roslibpy.__version__)"
python scripts\self_check.py
```

### 方式 B：environment.yml

```powershell
cd D:\RosMasterX3_Robonix_Internship
conda env create -f environment.yml -p .\conda_env
conda activate D:\RosMasterX3_Robonix_Internship\conda_env
python scripts\self_check.py
```

> 若 `conda env create` 卡在 `Collecting package metadata`，改用方式 A（清华镜像）。

### 日常激活 / 退出

```powershell
conda activate D:\RosMasterX3_Robonix_Internship\conda_env
# ... 写代码 / 跑脚本 ...
conda deactivate
```

### 更新依赖

```powershell
conda activate D:\RosMasterX3_Robonix_Internship\conda_env
python -m pip install -r requirements.txt
# 或重建：
# conda env update -f environment.yml -p .\conda_env --prune
```

### 删除环境（需要重建时）

```powershell
conda activate base
conda env remove -p D:\RosMasterX3_Robonix_Internship\conda_env
```

> 说明：本仓库**不再使用** `.venv`。请统一用上面的 `conda_env`。

---

## 二、SSH 连接（本机 → Jetson）

1. 复制并填写：

```powershell
copy configs\ssh.env.example configs\ssh.env
```

2. 登录：

```powershell
ssh <user>@<jetson-ip>
```

3. **推荐端口转发**（本机 `127.0.0.1:9090` → 车上 rosbridge）：

```powershell
ssh -L 9090:127.0.0.1:9090 <user>@<jetson-ip>
```

保持该窗口不关，再开一个已 `conda activate` 的终端跑脚本。

---

## 三、联调命令（conda 激活后）

```powershell
conda activate D:\RosMasterX3_Robonix_Internship\conda_env
cd D:\RosMasterX3_Robonix_Internship

# 不连车：红线 + 状态机自检
python scripts\self_check.py
python scripts\run_preview.py
python scripts\run_preview.py --confirm

# 已 SSH 转发且车上 rosbridge 已开：
python scripts\check_bridge.py
python scripts\check_topics.py

# 完整任务：默认只到「准备」；真下发必须同时 --confirm --dispatch
python scripts\run_mission.py
python scripts\run_mission.py --confirm --dispatch
python scripts\run_mission.py --confirm --dispatch --auto-preempt

# 手动取消
python scripts\run_cancel.py
```

---

## 四、车上启动顺序（验收时）

1. `roscore`  
2. 底盘 + 传感器 `bringup`  
3. `map_server` + AMCL + `move_base`  
4. `rosbridge`（`9090`）  
5. （可选）部署 Provider / Robonix  

## 红线

- 只发 goal / cancel，只读状态  
- **禁止**发布 `/cmd_vel`
