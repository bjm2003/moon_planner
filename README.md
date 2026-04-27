# 月球车月面路径规划项目 README

本文档用于记录从现有任务文档提取出的路径规划需求、规划模块输入设计、软件顶层架构、拟定代码文件结构和后续实施计划。后续开发应优先阅读本文档，再开始实现代码。

## 1. 文档来源与已确认约束

当前参考文件：

- `任务书.pdf`：扫描版任务书，已人工识读关键页。
- `技术要点清单（总）(改)(6).docx`：规划模块技术要点、软件设计清单、验证实验清单。
- `基于状态栅格图搜索的月球车路径规划技术方案.docx`：状态栅格图搜索、运动基元、代价地图、工程实现方案。

已确认规划相关约束：

- 测试环境：Ubuntu 20.04。
- 主要开发语言：C++。
- 目标部署方向：华为昇腾 Atlas 200i DK A2，后续需考虑 NPU 加速、轻量化、内存控制。
- ROS 依赖：任务书要求分析算法从 ROS 系统剥离的可行性及方案，因此核心规划算法不得强绑定 ROS。
- 路径规划结果输出频率：优于 1Hz。
- 路径规划成功率：不低于 99%。
- 车辆自动行驶速度：不小于 5km/h。
- 算法程序 CPU/ARM 占用率：小于 40%。
- 感知距离：不低于 15m。
- 最小识别障碍：200mm @ 15m。
- 障碍尺寸识别精度：优于 30mm，3 sigma。
- 障碍位置识别精度：200mm @ 15m，3 sigma。
- 障碍识别实时性：优于 1s。
- 坡度大于 10 度时，坡识别精度优于 1 度 @ 15m，3 sigma。
- 障碍目标感知识别虚警率：不高于 5%，漏警率不高于 3%。
- 感知检测结果输出频率：2Hz。
- 规划需考虑自身动力学约束、安全性约束、规划时间约束、路径最短约束。
- 规划需利用前方区域障碍信息与坡度信息综合研判，输出适合车辆行驶的期望速度、航向角、期望轨迹点。
- 规划需分析导航地图分辨率、范围、存储大小对路径规划的影响。
- 规划需分析视场外、已路过、历史障碍物对路径的影响。
- 规划需有近障循环恢复逻辑，处理接近障碍后陷入循环的情况。

当前设计假设：

- 先实现独立 C++17 核心库和命令行测试程序，后续再增加 ROS/Atlas 适配层。
- 核心规划算法采用状态栅格图搜索：运动基元库 + 通行性代价地图 + Hybrid A*/A*。
- 初版优先实现二维局部规划状态 `[x, y, theta]`，地图保留高程、坡度、粗糙度等通道；后续扩展到 `[x, y, z, theta, pitch]` 或速度维。
- 初版地图采用局部滑动窗口，典型感知/规划范围不低于 15m。
- 初版地图分辨率以 0.1m 为基准，后续实验比较 0.05m、0.1m、0.2m 等分辨率。

## 2. 规划模块应具备的输入

路径规划模块不能只接收起点、终点和障碍物。根据任务要求，输入应分为任务输入、车辆状态、地图环境、车辆模型、算法配置和历史信息六类。

### 2.1 任务与目标输入

- `start_state`：起始状态，至少包含 `x, y, z, yaw, pitch, roll, v, omega, timestamp, frame_id`。
- `goal`：目标点或兴趣点，至少包含 `x, y, z, yaw_optional, tolerance_xy, tolerance_yaw, frame_id`。
- `waypoints`：公里级路径节点或阶段目标序列，用于全局到局部分段规划。
- `mission_mode`：任务模式，例如 `global_to_waypoint`、`local_avoidance`、`near_obstacle_recovery`、`emergency_stop`。
- `planning_horizon`：局部规划范围和时间窗。
- `speed_policy`：期望速度范围、是否允许倒车、是否优先安全或优先效率。

### 2.2 车辆实时状态输入

- `pose`：车辆在导航坐标系下的位置与姿态。
- `twist`：线速度、角速度。
- `localization_covariance`：定位不确定性，用于安全裕度膨胀。
- `health_state`：定位、感知、执行机构状态，用于降级和异常处理。
- `last_executed_trajectory`：上一周期已执行轨迹，用于重规划连续性判断。

### 2.3 感知与地图输入

- `obstacle_list`：障碍物列表，包含类型、位置、尺寸、置信度和时间戳。
- `positive_obstacles`：凸起岩石、高度障碍等正障碍。
- `negative_obstacles`：凹坑、沟槽、陨石坑等负障碍。
- `slope_regions`：坡区位置、坡度、坡向、置信度。
- `roughness_map`：地形粗糙度或平整度通道。
- `local_cost_map`：感知模块或建图模块输出的局部代价地图。
- `dem_grid`：数字高程地图，用于坡度、俯仰角、可通行性评估。
- `occupancy_grid_2d`：二维占据栅格，供状态栅格搜索直接使用。
- `occupancy_grid_3d`：三维占据栅格，供障碍体素、车体包络碰撞检测使用。
- `navigation_map`：上注或先验导航地图，包含分辨率、范围、存储大小、原点、坐标系和数据通道。
- `sensor_fov`：当前感知视场，用于处理视场外历史障碍。

### 2.4 历史与记忆输入

- `historical_obstacles`：已观测但当前不在视场内的障碍。
- `traversed_cells`：车辆已路过区域，用于闭环、绕障恢复和重复路径惩罚。
- `blocked_regions`：近期规划失败或执行失败区域。
- `planner_history`：历史规划路径、失败原因、重规划次数、循环检测状态。

### 2.5 车辆模型与动力学约束输入

- `vehicle_geometry`：车体长宽高、轮距、轴距、离地间隙、碰撞外廓、安全裕度。
- `motion_limits`：最大线速度、最小线速度、最大角速度、最大加速度、最大减速度、最大曲率、最小转弯半径。
- `terrain_limits`：最大可通行坡度、最大侧倾限制、最大台阶高度、最大沟壑宽度。
- `skid_steer_model`：滑移转向模型参数、滑移系数、不同坡度/月壤条件下的修正参数。
- `control_limits`：速度档位、角速度档位、控制周期、是否允许原地转向或倒车。

### 2.6 算法配置输入

- `grid_resolution`：位置栅格分辨率。
- `heading_bins`：航向离散数量，例如 16 或 32。
- `primitive_library_path`：运动基元库文件路径。
- `cost_weights`：长度、转向、坡度、粗糙度、安全距离、历史障碍、目标距离等代价权重。
- `collision_config`：车体膨胀半径、轨迹采样间隔、碰撞检测层级。
- `search_limits`：最大扩展节点数、最大规划时间、内存池大小、开放列表容量。
- `recovery_config`：近障恢复阈值、循环检测窗口、失败重试策略。
- `runtime_config`：线程数、日志级别、性能统计开关、是否启用 Atlas/NPU 加速路径。

## 3. 规划模块输出

- `trajectory`：连续轨迹点序列，每点包含 `x, y, z, yaw, v, omega, t`。
- `path_nodes`：离散状态栅格节点序列，用于调试与复现。
- `control_reference`：期望线速度、角速度、航向角。
- `planner_status`：成功、目标不可达、超时、地图无效、碰撞风险、近障恢复、紧急停车等状态。
- `diagnostics`：规划耗时、扩展节点数、路径长度、最小障碍距离、最大坡度、总代价、各代价分量。
- `debug_layers`：可选输出代价地图、碰撞膨胀层、搜索访问层、历史障碍层。

## 4. 顶层软件架构

推荐采用分层架构，核心算法库不依赖 ROS。

```text
应用层
  命令行规划器 / 离线回放 / 基元生成工具 / Benchmark 工具 / 后续 ROS 或 Atlas 适配

运行时层
  参数加载 / 线程调度 / 日志 / 性能统计 / 异常管理 / 内存池

规划算法层
  状态栅格搜索 / 混合 A* / 局部避障 / 近障恢复 / 路径提取

地图与风险层
  2D/3D 占据栅格 / DEM / 坡度图 / 粗糙度图 / 通行性代价图 / 历史障碍融合

运动模型层
  滑移转向运动学 / 动力学约束 / 运动基元生成、加载、查询

基础数据层
  坐标、状态、轨迹、障碍、栅格、配置、错误码
```

关键工程原则：

- 核心库只依赖 C++17 STL，必要时引入 Eigen、yaml-cpp、GoogleTest；不得在核心库中直接依赖 ROS。
- 输入输出接口采用明确结构体和文件格式，先支持离线场景回放，再接入实时系统。
- 所有在线规划路径避免频繁动态分配，搜索节点、开放列表、闭集优先使用预分配或内存池。
- 地图、基元、碰撞模板必须有可序列化格式，便于复现实验。
- 所有算法必须输出诊断信息，便于达成 1Hz、99% 成功率和 CPU/ARM 占用率指标。

## 5. 拟定代码文件结构

以下是正式代码实现前的目标文件结构。后续实现时应尽量按此结构推进，除非有明确架构原因需要调整。

```text
moon_planner/
  README.md
  CMakeLists.txt
  cmake/
    ProjectOptions.cmake
    Sanitizers.cmake
  config/
    planner_default.yaml
    vehicle_default.yaml
    cost_weights_default.yaml
    primitive_default.yaml
  include/
    moon_planner/
      core/
        types.hpp
        geometry.hpp
        math_utils.hpp
        status.hpp
        time.hpp
      config/
        planner_config.hpp
        vehicle_config.hpp
        cost_config.hpp
      model/
        skid_steer_model.hpp
        motion_constraints.hpp
      primitives/
        motion_primitive.hpp
        primitive_library.hpp
        primitive_generator.hpp
      map/
        grid_index.hpp
        occupancy_grid.hpp
        elevation_grid.hpp
        cost_map.hpp
        map_fusion.hpp
        history_layer.hpp
      collision/
        footprint.hpp
        collision_checker.hpp
      cost/
        traversability_evaluator.hpp
        primitive_cost.hpp
        heuristic.hpp
      search/
        state_lattice.hpp
        search_node.hpp
        open_list.hpp
        hybrid_astar.hpp
      planner/
        planner_interface.hpp
        lattice_planner.hpp
        local_planner.hpp
        recovery_planner.hpp
      trajectory/
        path_extractor.hpp
        trajectory_generator.hpp
        trajectory_smoother.hpp
      runtime/
        memory_pool.hpp
        planner_context.hpp
        scheduler.hpp
        diagnostics.hpp
      io/
        yaml_loader.hpp
        map_reader.hpp
        map_writer.hpp
        primitive_io.hpp
        scenario_reader.hpp
        trajectory_writer.hpp
  src/
    core/
      geometry.cpp
    config/
      planner_config.cpp
      vehicle_config.cpp
      cost_config.cpp
    model/
      skid_steer_model.cpp
      motion_constraints.cpp
    primitives/
      primitive_library.cpp
      primitive_generator.cpp
    map/
      grid_index.cpp
      occupancy_grid.cpp
      elevation_grid.cpp
      cost_map.cpp
      map_fusion.cpp
      history_layer.cpp
    collision/
      footprint.cpp
      collision_checker.cpp
    cost/
      traversability_evaluator.cpp
      primitive_cost.cpp
      heuristic.cpp
    search/
      state_lattice.cpp
      open_list.cpp
      hybrid_astar.cpp
    planner/
      lattice_planner.cpp
      local_planner.cpp
      recovery_planner.cpp
    trajectory/
      path_extractor.cpp
      trajectory_generator.cpp
      trajectory_smoother.cpp
    runtime/
      memory_pool.cpp
      planner_context.cpp
      scheduler.cpp
      diagnostics.cpp
    io/
      yaml_loader.cpp
      map_reader.cpp
      map_writer.cpp
      primitive_io.cpp
      scenario_reader.cpp
      trajectory_writer.cpp
    apps/
      moon_planner_cli.cpp
      generate_primitives.cpp
      replay_scenario.cpp
      benchmark_planner.cpp
      inspect_map.cpp
  tests/
    unit/
      test_grid_index.cpp
      test_skid_steer_model.cpp
      test_primitive_generator.cpp
      test_collision_checker.cpp
      test_cost_map.cpp
      test_hybrid_astar.cpp
      test_path_extractor.cpp
    integration/
      test_planner_flat_map.cpp
      test_planner_dense_obstacles.cpp
      test_planner_slope_map.cpp
      test_near_obstacle_recovery.cpp
      test_history_obstacles.cpp
  scenarios/
    README.md
    flat_empty.yaml
    flat_obstacles.yaml
    dense_obstacles.yaml
    slope_region.yaml
    negative_obstacles.yaml
  tools/
    plot_trajectory.py
    summarize_benchmark.py
    convert_map.py
  docs/
    architecture.md
    data_formats.md
    test_plan.md
```

### 5.1 根目录与构建文件

- `CMakeLists.txt`：项目主构建入口，定义核心库、工具程序和测试目标。
- `cmake/ProjectOptions.cmake`：统一编译选项，Ubuntu 20.04 默认使用 C++17。
- `cmake/Sanitizers.cmake`：调试阶段启用 AddressSanitizer/UndefinedBehaviorSanitizer。

### 5.2 配置文件

- `config/planner_default.yaml`：规划器默认配置，包含栅格分辨率、航向离散数、搜索时间上限、线程配置。
- `config/vehicle_default.yaml`：车辆几何、速度、角速度、加速度、坡度、曲率、滑移参数。
- `config/cost_weights_default.yaml`：路径长度、转向、坡度、粗糙度、安全距离、历史障碍等代价权重。
- `config/primitive_default.yaml`：运动基元生成参数，包含速度档位、角速度档位、积分步长、基元持续时间。

### 5.3 核心基础文件

- `include/moon_planner/core/types.hpp`：核心数据类型，定义 `Pose2D`、`Pose3D`、`State`、`TrajectoryPoint`、`Obstacle` 等。
- `include/moon_planner/core/geometry.hpp` / `src/core/geometry.cpp`：几何计算，如角度归一化、点到线距离、多边形变换。
- `include/moon_planner/core/math_utils.hpp`：轻量数学工具函数，尽量 header-only。
- `include/moon_planner/core/status.hpp`：错误码、规划状态码和返回结果枚举。
- `include/moon_planner/core/time.hpp`：时间戳、耗时统计工具。

### 5.4 配置加载文件

- `include/moon_planner/config/planner_config.hpp` / `src/config/planner_config.cpp`：规划器配置结构体和校验逻辑。
- `include/moon_planner/config/vehicle_config.hpp` / `src/config/vehicle_config.cpp`：车辆参数结构体和合法性检查。
- `include/moon_planner/config/cost_config.hpp` / `src/config/cost_config.cpp`：代价权重配置与范围检查。

### 5.5 车辆模型与运动基元

- `include/moon_planner/model/skid_steer_model.hpp` / `src/model/skid_steer_model.cpp`：滑移转向运动学模型，提供状态积分函数。
- `include/moon_planner/model/motion_constraints.hpp` / `src/model/motion_constraints.cpp`：速度、角速度、加速度、曲率、坡度等约束检查。
- `include/moon_planner/primitives/motion_primitive.hpp`：运动基元数据结构，包含起终状态、轨迹点、控制输入和预估代价。
- `include/moon_planner/primitives/primitive_library.hpp` / `src/primitives/primitive_library.cpp`：运动基元库加载、索引、按航向查询。
- `include/moon_planner/primitives/primitive_generator.hpp` / `src/primitives/primitive_generator.cpp`：离线运动基元生成，初期采用前向积分，后续可扩展 BVP/优化生成。

### 5.6 地图与历史层

- `include/moon_planner/map/grid_index.hpp` / `src/map/grid_index.cpp`：连续坐标与栅格索引转换。
- `include/moon_planner/map/occupancy_grid.hpp` / `src/map/occupancy_grid.cpp`：二维/三维占据栅格基础结构。
- `include/moon_planner/map/elevation_grid.hpp` / `src/map/elevation_grid.cpp`：DEM、高程、坡度和坡向计算。
- `include/moon_planner/map/cost_map.hpp` / `src/map/cost_map.cpp`：通行性代价地图，融合障碍、坡度、粗糙度、安全距离。
- `include/moon_planner/map/map_fusion.hpp` / `src/map/map_fusion.cpp`：感知输入、先验地图、历史层融合。
- `include/moon_planner/map/history_layer.hpp` / `src/map/history_layer.cpp`：视场外、已路过、历史障碍维护与衰减策略。

### 5.7 碰撞与风险代价

- `include/moon_planner/collision/footprint.hpp` / `src/collision/footprint.cpp`：车体足迹、多边形外廓、安全膨胀。
- `include/moon_planner/collision/collision_checker.hpp` / `src/collision/collision_checker.cpp`：状态点和运动基元碰撞检测。
- `include/moon_planner/cost/traversability_evaluator.hpp` / `src/cost/traversability_evaluator.cpp`：坡度、粗糙度、障碍距离等通行性判断。
- `include/moon_planner/cost/primitive_cost.hpp` / `src/cost/primitive_cost.cpp`：单条基元总代价计算。
- `include/moon_planner/cost/heuristic.hpp` / `src/cost/heuristic.cpp`：A*/Hybrid A* 启发式，初期欧氏距离，后续扩展 HLUT/Reeds-Shepp。

### 5.8 搜索与规划器

- `include/moon_planner/search/state_lattice.hpp` / `src/search/state_lattice.cpp`：状态栅格图，管理状态离散化和后继生成。
- `include/moon_planner/search/search_node.hpp`：搜索节点结构，包含状态索引、g/h/f、父节点、基元 ID。
- `include/moon_planner/search/open_list.hpp` / `src/search/open_list.cpp`：开放列表优先队列封装，后续可替换为更高性能结构。
- `include/moon_planner/search/hybrid_astar.hpp` / `src/search/hybrid_astar.cpp`：Hybrid A*/状态栅格搜索主算法。
- `include/moon_planner/planner/planner_interface.hpp`：规划器统一接口，隔离应用层与算法实现。
- `include/moon_planner/planner/lattice_planner.hpp` / `src/planner/lattice_planner.cpp`：基于状态栅格的主规划器。
- `include/moon_planner/planner/local_planner.hpp` / `src/planner/local_planner.cpp`：局部避障规划器，处理短距离局部目标。
- `include/moon_planner/planner/recovery_planner.hpp` / `src/planner/recovery_planner.cpp`：近障恢复、循环检测、降级策略。

### 5.9 路径和轨迹

- `include/moon_planner/trajectory/path_extractor.hpp` / `src/trajectory/path_extractor.cpp`：从搜索父节点链提取离散路径。
- `include/moon_planner/trajectory/trajectory_generator.hpp` / `src/trajectory/trajectory_generator.cpp`：拼接运动基元生成连续轨迹和速度指令。
- `include/moon_planner/trajectory/trajectory_smoother.hpp` / `src/trajectory/trajectory_smoother.cpp`：轨迹平滑和速度连续性处理，初期可实现为可选后处理。

### 5.10 运行时与 I/O

- `include/moon_planner/runtime/memory_pool.hpp` / `src/runtime/memory_pool.cpp`：搜索节点、轨迹点等对象预分配。
- `include/moon_planner/runtime/planner_context.hpp` / `src/runtime/planner_context.cpp`：单次规划上下文，封装地图、配置、状态和诊断。
- `include/moon_planner/runtime/scheduler.hpp` / `src/runtime/scheduler.cpp`：规划线程调度和周期控制。
- `include/moon_planner/runtime/diagnostics.hpp` / `src/runtime/diagnostics.cpp`：规划耗时、内存、节点扩展数、状态统计。
- `include/moon_planner/io/yaml_loader.hpp` / `src/io/yaml_loader.cpp`：YAML 配置读取。
- `include/moon_planner/io/map_reader.hpp` / `src/io/map_reader.cpp`：导航地图、DEM、代价图读取。
- `include/moon_planner/io/map_writer.hpp` / `src/io/map_writer.cpp`：调试地图和结果地图输出。
- `include/moon_planner/io/primitive_io.hpp` / `src/io/primitive_io.cpp`：运动基元库序列化和反序列化。
- `include/moon_planner/io/scenario_reader.hpp` / `src/io/scenario_reader.cpp`：离线测试场景读取。
- `include/moon_planner/io/trajectory_writer.hpp` / `src/io/trajectory_writer.cpp`：轨迹、诊断、benchmark 结果输出。

### 5.11 应用程序

- `src/apps/moon_planner_cli.cpp`：命令行单次规划入口，输入场景文件，输出轨迹和诊断。
- `src/apps/generate_primitives.cpp`：离线生成运动基元库。
- `src/apps/replay_scenario.cpp`：按时间序列回放感知/地图输入，测试重规划。
- `src/apps/benchmark_planner.cpp`：批量运行场景库，统计成功率、耗时、路径长度、安全距离。
- `src/apps/inspect_map.cpp`：检查地图文件、分辨率、范围、障碍统计和内存估算。

### 5.12 测试文件

- `tests/unit/test_grid_index.cpp`：坐标与栅格索引转换测试。
- `tests/unit/test_skid_steer_model.cpp`：滑移转向积分与约束测试。
- `tests/unit/test_primitive_generator.cpp`：运动基元生成、闭合性和约束测试。
- `tests/unit/test_collision_checker.cpp`：车体足迹和基元碰撞检测测试。
- `tests/unit/test_cost_map.cpp`：通行性代价融合测试。
- `tests/unit/test_hybrid_astar.cpp`：搜索算法基础可达性和最短性测试。
- `tests/unit/test_path_extractor.cpp`：路径回溯与轨迹拼接测试。
- `tests/integration/test_planner_flat_map.cpp`：平坦无障碍场景。
- `tests/integration/test_planner_dense_obstacles.cpp`：密集障碍场景。
- `tests/integration/test_planner_slope_map.cpp`：大坡度起伏场景。
- `tests/integration/test_near_obstacle_recovery.cpp`：近障循环恢复场景。
- `tests/integration/test_history_obstacles.cpp`：视场外/已路过/历史障碍影响场景。

### 5.13 工具与文档

- `scenarios/README.md`：场景文件格式说明。
- `tools/plot_trajectory.py`：绘制地图、搜索路径、轨迹和障碍。
- `tools/summarize_benchmark.py`：汇总 benchmark 结果，输出均值、标准差、最大值、最小值。
- `tools/convert_map.py`：转换 DEM/栅格/CSV/图像等离线地图格式。
- `docs/architecture.md`：后续扩展的详细架构说明。
- `docs/data_formats.md`：地图、障碍、场景、轨迹、基元库格式。
- `docs/test_plan.md`：测试计划、指标、场景库和验收标准。

## 6. 初版实现路线

### 阶段 0：工程骨架

- 创建 CMake 工程、核心 include/src 目录、配置文件和基础测试框架。
- 明确 YAML 场景格式、轨迹 CSV/JSON 输出格式。
- 建立 `moon_planner_cli`，即使初期只做参数读取和空结果返回。

### 阶段 1：基础数据结构与地图

- 实现核心类型、栅格索引、占据栅格、DEM、高程转坡度。
- 实现代价地图生成：障碍代价、坡度代价、粗糙度代价、安全距离代价。
- 完成平坦地图、障碍地图、坡地图的单元测试。

### 阶段 2：运动模型与运动基元

- 实现滑移转向运动学积分。
- 实现运动约束检查。
- 实现离线运动基元生成工具。
- 初始推荐参数：位置分辨率 0.1m，航向 16 档，积分步长 0.1s，基元时长 1-3s，具体按车辆速度和转弯半径修正。

### 阶段 3：状态栅格搜索

- 实现状态离散化、开放列表、闭集、父节点回溯。
- 实现 Hybrid A*/状态栅格搜索主流程。
- 实现基元碰撞检测和基元代价计算。
- 先用欧氏距离启发式跑通，再评估 HLUT 或更强启发式。

### 阶段 4：轨迹输出与诊断

- 拼接基元轨迹，输出连续轨迹点。
- 输出期望线速度、角速度、航向角。
- 输出规划耗时、扩展节点数、路径长度、最小安全距离、最大坡度、总代价。

### 阶段 5：局部避障、历史障碍与恢复

- 加入历史障碍层和视场外障碍衰减机制。
- 加入已路过区域和循环检测。
- 加入近障恢复策略：减速、后退、侧向绕行、切换局部目标、返回最近可达点。

### 阶段 6：性能与部署

- 引入内存池和预分配开放列表/闭集。
- 建立 benchmark 场景库，每场景重复至少 10 次，统计均值、标准差、最大值、最小值。
- 控制规划周期优于 1Hz，逐步向 CPU/ARM 占用率小于 40% 收敛。
- 分析地图分辨率、范围、存储大小对实时性、成功率、路径质量的影响。
- 分析从 ROS 剥离后的接口方案，预留 Atlas 200i DK A2 适配层。

## 7. 验证实验清单

必须覆盖以下场景：

- 平坦区域。
- 密集障碍区。
- 大坡度起伏区。
- 负障碍物集中区。
- 视场外/已路过/历史障碍场景。
- 极近距离障碍物场景。
- 公里级路径节点约束场景。

必须统计以下指标：

- 路径规划实时性。
- 路径规划成功率。
- 避障成功率。
- 近障恢复率。
- 路径长度偏差。
- 轨迹跟踪误差。
- 安全距离均值和最小值。
- 资源占用超标率。
- 异常发生率。
- 每种场景重复次数不小于 10 次，并统计均值、标准差、最大值、最小值。

## 8. 后续 Codex 执行注意事项

- 不要直接把 ROS 节点作为核心实现入口；先实现纯 C++ 核心库。
- 不要在未确认车辆实参前写死车体尺寸、速度、转弯半径和坡度极限；应全部从配置读取。
- 不要只实现普通 2D A*；必须保留状态栅格、航向离散和运动基元机制。
- 不要只关注可达路径；必须从第一版就输出诊断信息，否则后续难以证明 1Hz、99% 成功率和资源占用指标。
- 初期不需要立即做 NPU 加速；但数据结构和模块边界要避免后续无法迁移。
- 如果后续要创建代码，先按本文第 5 节创建工程骨架，再实现第 6 节阶段 1。

## 9. 当前工程进展

已完成第一版可编译工程骨架：

- CMake/C++17 核心库 `moon_planner_core`。
- 基础类型、状态码、几何工具、耗时统计。
- 规划、车辆、代价和运动基元配置结构体，以及默认 YAML 配置加载。
- 栅格索引、二维占据栅格、高程栅格、通行性代价图、基础历史层和地图融合。
- 滑移转向运动学积分、运动约束、运动基元生成和基元库查询。
- 多边形车体足迹碰撞检测、基元代价、欧氏距离启发式。
- 占据障碍、安全距离膨胀和坡度代价地图。
- 最小状态栅格/Hybrid A* 搜索链路。
- `LatticePlanner` 统一规划器入口。
- `LocalPlanner` 局部规划入口，支持按 `planning_horizon_m` 裁剪远距离目标。
- 轨迹生成、CSV 输出、诊断格式化。
- CLI：`moon_planner_cli`，支持默认场景和 `scenarios/*.yaml` 输入。
- 工具：`generate_primitives`、`benchmark_planner`。
- 测试：`test_grid_index`、`test_skid_steer_model`、`test_collision_checker`、`test_cost_map`、`test_local_planner`、`test_yaml_loader`、`test_scenario_reader`、`test_planner_flat_map`、`test_planner_dense_obstacles`、`test_planner_slope_map`。

当前验证命令：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j2
ctest --test-dir build --output-on-failure
./build/moon_planner_cli
./build/moon_planner_cli scenarios/dense_obstacles.yaml /tmp/moon_dense_trajectory.csv
./build/moon_planner_cli scenarios/slope_region.yaml /tmp/moon_slope_trajectory.csv
./build/generate_primitives /tmp/moon_primitives_summary.txt
./build/benchmark_planner
```

最近一次验证结果：

- `ctest`：10/10 通过。
- `moon_planner_cli` 默认场景：返回 `status=success`，扩展节点 100，轨迹点 20。
- `moon_planner_cli scenarios/dense_obstacles.yaml /tmp/moon_dense_trajectory.csv`：返回 `status=success`，扩展节点 100，轨迹点 20。
- `moon_planner_cli scenarios/slope_region.yaml /tmp/moon_slope_trajectory.csv`：返回 `status=success`，扩展节点 100，轨迹点 20。
- `generate_primitives`：生成 16 个航向组，共 256 条运动基元。
- `benchmark_planner`：10 次平坦场景运行，成功率 1.0。

当前限制：

- 配置文件已接入 CLI 和工具；当前 YAML loader 只支持顶层键值、bool、数字和数字数组，不支持嵌套 YAML。
- 碰撞检测已采用多边形车体足迹，但尚未加入更完整的分离轴边交检测和动态安全裕度。
- 搜索启发式当前为欧氏距离，后续应加入航向代价、HLUT 或 Reeds-Shepp/Dubins 近似。
- `LocalPlanner` 目前实现局部目标裁剪，尚未加入动态障碍局部避让和恢复动作。
- 近障恢复、历史障碍衰减和多分辨率实验仍待实现。

总体路线复盘和后续阶段规划见 `docs/roadmap.md`。

下一阶段计划：

- 扩展 `HistoryLayer`，支持历史障碍和已访问区域衰减。
- 将历史层接入 `MapFusion`，让搜索能避开近期失败/重复区域。
- 为 `RecoveryPlanner` 增加近障恢复入口：检测局部目标失败后生成后退或侧向重试请求。
