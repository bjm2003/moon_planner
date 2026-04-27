# 数据格式

当前阶段已支持轻量 YAML 场景、默认 YAML 配置和 CSV 轨迹输出。

配置 YAML 当前支持顶层键值、bool、数字和数字数组，已用于 `config/planner_default.yaml`、`config/vehicle_default.yaml`、`config/cost_weights_default.yaml` 和 `config/primitive_default.yaml`。

场景 YAML 字段：

- `name`：场景名。
- `map.width`、`map.height`、`map.resolution_m`：二维占据栅格定义。
- `start: {x, y, yaw}`：起始状态。
- `goal: {x, y, yaw}`：目标状态。
- `obstacle_cells`：占据栅格矩形闭区间列表，格式为 `{x0, y0, x1, y1}`。
- `slope_regions`：坡度矩形闭区间列表，格式为 `{x0, y0, x1, y1, slope_rad, axis}`，当前用于生成简化高程栅格，坡度超过车辆限制的区域会进入 lethal cost。

轨迹 CSV 字段：

```text
t,x,y,z,yaw,v,omega
```

后续需要补充完整 DEM 文件、历史障碍层、运动基元库二进制格式和 benchmark 结果格式。
