# 数据格式

当前阶段已支持轻量 YAML 场景和 CSV 轨迹输出。

场景 YAML 字段：

- `name`：场景名。
- `map.width`、`map.height`、`map.resolution_m`：二维占据栅格定义。
- `start: {x, y, yaw}`：起始状态。
- `goal: {x, y, yaw}`：目标状态。
- `obstacle_cells`：占据栅格矩形闭区间列表，格式为 `{x0, y0, x1, y1}`。

轨迹 CSV 字段：

```text
t,x,y,z,yaw,v,omega
```

后续需要补充 DEM/坡度图、历史障碍层、运动基元库二进制格式和 benchmark 结果格式。
