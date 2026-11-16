# 智能网联汽车赛项：路径跟踪参考

本目录从 `pb2025_sentry_nav` 的路径跟踪控制器中选取了**前视距离与前视点选择**的思路，整理成不依赖 ROS 的 Python 参考代码 `path_tracking.py`。原控制器属于 `pb_omni_pid_pursuit_controller` 子模块，面向全向移动的哨兵机器人；这里新增的 `forward_steering_angle` 使用汽车自行车模型计算前轮转角，不来自原控制器。

来源：

- `pb2025_sentry_nav`：https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav
- 原控制器提交：`b542c5794b91fba2bb3fe2cbdc72fcf854fedb1a`
- 原代码：`src/omni_pid_pursuit_controller.cpp` 中的 `getLookAheadDistance`、`getLookAheadPoint`、`circleSegmentIntersection`
- 原作者版权：Copyright 2025 Lihan Chen；原控制器采用 Apache License 2.0，许可文本见本目录 `LICENSE`。

输入的路径应按行驶方向排序，且已经转换到车体坐标系（x 向前、y 向左，单位为米）。这份代码只给出前视点和**正向行驶**的转向角参考，尚未接入赛事仿真平台，也没有完成油门、制动、避障、信号灯、泊车或完整赛题。实际使用前，须按赛事 API、车辆轴距、转向范围及场景规则适配并验证。不要把这个目录接入现有室内人体跟随启动链。
