# 智能网联汽车赛项：路径跟踪参考

本目录从 `pb2025_sentry_nav` 的路径跟踪控制器中选取了**前视距离与前视点选择**的思路，整理成不依赖 ROS 的 Python 参考代码。原控制器属于 `pb_omni_pid_pursuit_controller` 子模块，面向全向移动的哨兵机器人；这里新增的汽车自行车模型前轮转角计算不来自原控制器。

来源：

- `pb2025_sentry_nav`：https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav
- 原控制器提交：`b542c5794b91fba2bb3fe2cbdc72fcf854fedb1a`
- 原代码：`src/omni_pid_pursuit_controller.cpp` 中的 `getLookAheadDistance`、`getLookAheadPoint`、`circleSegmentIntersection`
- 原作者版权：Copyright 2025 Lihan Chen；原控制器采用 Apache License 2.0，许可文本见本目录 `LICENSE`。

Python 与 C++ 两个实现分开放置，算法功能目标相同，均为不依赖 ROS/SimOne 的正向路径跟踪离线参考：

- **Python 版本**：本目录根层的 `path_tracking.py`、`controller.py`、`offline_demo.py`；运行 `python -B competition/icv/offline_demo.py`。
- **C++ 版本**：`cpp/` 子目录，含独立的控制器、离线示例和 CMake 工程；其源码不会覆盖或调用 Python 文件。构建和运行方式见 [`cpp/README.md`](cpp/README.md)。

两个版本都把车辆状态和路线作为内部输入，输出前轮转角（弧度）、归一化油门和制动（0–1）。它们都只支持**正向行驶**，未实现避障、信号灯、倒车泊车或完整赛题，也没有接入赛事平台。实际使用前须按赛事 API、车辆轴距、转向范围及场景规则适配并验证。不要把这个目录接入现有室内人体跟随启动链。

## 第一个离线闭环

Python 离线闭环由 `controller.py` 和 `offline_demo.py` 提供，可运行直道与半径 50 m 弯道示例：

```powershell
python -B competition/icv/offline_demo.py
```

该示例仅用于算法开发，不能代表正式场景得分。赛事字段、内部约定与所需平台资料见 [PLATFORM_GAP.md](PLATFORM_GAP.md)。

