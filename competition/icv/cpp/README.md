# C++ 版离线路径跟踪

这是与 `competition/icv/` 根目录 Python 版**分开放置**的 C++17 实现。它不读取 Python 文件，也不依赖 ROS 或 SimOne；两个版本实现相同的前向路径跟踪闭环和简化自行车模型演示。Python 源码保持原样。前视点选择方法沿用本仓库 Python 参考中的改写思路；原始思路来源及 Apache 2.0 许可见 [`../README.md`](../README.md) 和 [`../LICENSE`](../LICENSE)。

## 文件

- `controller.hpp` / `controller.cpp`：车辆状态、路径前视、汽车自行车模型转向、纵向速度控制。
- `offline_demo.cpp`：简化车辆模型下的直道与半径 50 m 弯道演示。
- `CMakeLists.txt`：独立 CMake 工程。

## 构建和运行

从仓库根目录执行：

```powershell
cmake -S competition/icv/cpp -B build/icv-cpp
cmake --build build/icv-cpp --config Release
.
build/icv-cpp/Release/icv_offline_demo.exe
```

在单配置生成器（如 Linux Makefiles/Ninja）下，运行 `build/icv-cpp/icv_offline_demo`。

## 内部接口与边界

- 路线坐标为世界坐标，单位米，按行驶方向排序。
- 状态为世界坐标位置（米）、航向角（弧度）、正向车速（米/秒）。
- 控制输出为前轮转角（弧度，左转为正）以及范围 `[0, 1]` 的油门和制动。
- 当前只支持正向行驶。坐标/单位、轮距轴距参数、转向比例及控制范围必须在接入赛事 SDK 前按官方接口重新核对。
- 演示仅为简化离线模型，不包含赛事 SDK、感知、交通参与者、碰撞判定或官方评分，不能视作赛题场景通过。
