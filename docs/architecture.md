# 架构说明

核心库 `moon_planner_core` 不依赖 ROS。应用层通过 CLI、回放程序或后续适配层调用 `PlannerInterface`，输入为车辆状态、目标和地图，输出为轨迹、状态码和诊断信息。
