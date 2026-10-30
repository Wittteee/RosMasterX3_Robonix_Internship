# YOLOX 目标检测接入

使用 OpenCV DNN 在 CPU 上运行 YOLOX-s ONNX，无需 PyTorch。支持 COCO 80 类英文标签，例如 `person`、`bottle`、`chair`。自定义类别需要另行训练并适配，不能直接把二维码文本当作检测类别。

## 安装与模型

在仓库根目录，使用业务主机 Python 环境执行：

```bash
python -m pip install -r requirements-vision.txt
python scripts/download_yolox.py
```

模型来源：[OpenCV Zoo YOLOX](https://github.com/opencv/opencv_zoo/tree/main/models/object_detection_yolox)，该目录按 Apache-2.0 发布。预处理和输出格式遵循官方模型接口；实现使用 RGB、640×640 左上对齐、右下填充 114、原始像素值、stride 8/16/32 解码以及按类别 NMS。

下载脚本固定文件 `object_detection_yolox_2022nov.onnx`，校验大小 35,858,002 字节及 SHA-256 `c5c2d13e59ae883e6af3b45daea64af4833a4951c92d116ec270d9ddbe998063`。校验失败不会覆盖已有模型。权重不提交 Git；离线主机可先下载再复制到 `models/`。推理入口不会自动联网下载。

## 只检测，不连接机器人

```bash
python scripts/run_object_detection.py --image logs/demo_20261028/camera_current.jpg
python scripts/run_object_detection.py --camera 0 --label bottle --confidence 0.5
```

每次采集/处理一帧后退出，输出全部检测框、类别、置信度、推理耗时，并在 `logs/detection/` 保存 JSON 和标注图片。`--label` 要求该类恰好一个目标，否则返回失败和原因。未指定标签时，零检测框也是有效检测结果。`candidate_detection` 可供现有候选模块使用，但它的 `size` 仅为框占图像面积比例，不是米制距离。

## 摄像头接入现有官方候选流程

以下命令中的相机内参、分辨率与 Provider 路径必须替换为实际标定值。示例数字仅展示参数格式：

```bash
python scripts/run_visual_nav.py --detector yolox --camera 0 --label bottle --provider-dir /path/to/provider --fx 500 --cx 320 --camera-yaw-deg 0 --image-width 640 --image-height 480 --max-radius 0.2
```

默认停在准备阶段，不下发移动。额外指定 `--execute` 后仍需在终端输入“确认执行”，并通过已有健康检查、候选有效期和一次性令牌验证。原有 `--detector qr`（默认值）保持可用。

每次候选准备前、人工确认后重新打开本机摄像头采集并检测。支持数字设备索引或 Linux `/dev/videoN`；实车导航不接受录像文件。单次采集加推理超过 2 秒、画面尺寸与标定不符、目标消失或同类目标不唯一，都会拒绝继续。若主机推理过慢，应先优化部署性能并验证延迟，而不是直接放宽时效限制。

检测框中心结合 `fx/cx` 与相机安装偏航角计算目标方向，再从 Provider 返回的安全短距候选中选择；不会用检测框估算目标实际距离，也不会自动连续追踪或保证最终停车距离。目标身份没有跨帧跟踪，适用于静态、唯一的目标。导航日志保留最近一次检测框及任务状态，存放在 `logs/vision/`。

部署验证顺序：先用照片验收类别与标注，再用本机摄像头核对画面和延迟，最后在底盘回读、定位与距离标定正常后做带人工确认的短距实车验收。目标检测接入本身不会修复底盘“30 cm 指令只走 2 cm”的问题。

## 测试

```bash
python -m pytest -q
python scripts/self_check.py
```

测试覆盖解码、坐标还原、RGB/填充、按类别去重、目标歧义、重复采集、超时、标定尺寸和权重校验。单元测试不需要下载权重；真实模型推理需要单独执行上述图片命令。

2026-10-30 本机 Windows/Python 3.12/OpenCV 5.0 验证：150 项测试、5 个子测试及 9 项自检通过。官方权重校验通过；对仓库 `logs/demo_20261028/camera_current.jpg` 实际推理检测到 2 个 `person`，置信度约 0.904/0.744，首次推理约 105 ms（仅本机该图片测量，不代表 Jetson 帧率）。标注框已人工查看；画面是强曝光、低机位场景，不能据此认定其他类别或场景的准确率。实物摄像头采集、Jetson 部署和移动执行本轮未验证。
