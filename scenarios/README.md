# 场景目录

当前场景文件使用项目内置的轻量 YAML 子集，暂不依赖 `yaml-cpp`。

支持字段：

- `name`：场景名。
- `map.width`、`map.height`、`map.resolution_m`：地图尺寸和分辨率。
- `start: {x, y, yaw}`：起点。
- `goal: {x, y, yaw}`：终点。
- `obstacle_cells`：占据栅格矩形列表，每项为 `{x0, y0, x1, y1}`，闭区间。
- `slope_regions`：坡度矩形列表，每项为 `{x0, y0, x1, y1, slope_rad, axis}`，`axis` 可选 `x` 或 `y`，用于生成简化 DEM 坡面。

后续需要扩展负障碍物、历史障碍、感知视场和动态回放时间序列。
