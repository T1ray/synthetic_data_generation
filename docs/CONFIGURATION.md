# Настройка Scenario YAML

## Выбор версии

- Используйте v1 только для совместимости со старой командой одного box.
- Используйте v2 для нескольких объектов и RViz без сенсорных эффектов.
- Используйте v3 для noise/dropout/intensity, zero slots и temporal placement.

Готовые примеры находятся в `config/scenarios`.

## Общие поля

```yaml
schema_version: 3
scenario_id: unique_run_name
seed: 17421

source:
  pointcloud_topic: /lidar_points

frames:
  start_index: 0
  end_index: 100
```

Диапазон включительный. Индекс увеличивается только для выбранного
PointCloud2-топика.

## Visualisation

```yaml
visualization:
  enabled: true
  marker_topic: /synthetic/markers
  point_size_m: 0.06
  marker_lifetime_sec: 0.25
  show_geometry: true
  show_modified_points: true
  show_text: true
  show_bounding_box: true
```

`marker_lifetime_sec` должен быть положительным. Marker topic не должен
совпадать с PointCloud2-топиком или существующим входным topic.

## Цвет объекта

```yaml
visualization:
  color_rgba: [1.0, 0.2, 0.1, 0.9]
```

Каждая компонента находится в `[0, 1]`, alpha строго больше нуля.

## Геометрии

### Box

```yaml
geometry:
  type: box
  dimensions_m: [0.6, 0.4, 0.3]
```

### Cylinder

```yaml
geometry:
  type: cylinder
  radius_m: 0.25
  height_m: 1.2
  radial_segments: 32
```

`radial_segments` находится в `[8, 256]`.

### Human mesh

```yaml
geometry:
  type: human_mesh
  path: ../../meshes/humans/person.obj
  units: meters
  scale: [1.0, 1.0, 1.0]
  origin: base_center
  mesh_resource_uri: package://synthetic_data_generation/meshes/humans/person.obj
```

Поддерживаются `.obj` и `.stl`. `path` разрешается относительно YAML. Units:
`meters`, `centimeters`, `millimeters`. `base_center` переносит min Z в ноль и
центрирует XY bounds. Внешние mesh должны иметь проверенную лицензию.

### Cable

```yaml
geometry:
  type: cable
  radius_m: 0.015
  radial_segments: 12
  control_points_m:
    - [0.0, -1.0, 0.0]
    - [0.0,  0.0, 0.3]
    - [0.0,  1.0, 0.0]
```

Нужно минимум две различные последовательные точки.

## Lidar-relative placement

```yaml
placement:
  type: lidar_relative
  xyz_m: [10.0, 0.0, -1.0]
  rpy_deg: [0.0, 0.0, 15.0]
```

Этот режим не требует trajectory. В schema v3 он поддерживает только static
motion.

## Track-relative placement

```yaml
placement:
  type: track_relative
  longitudinal_m: 35.0
  lateral_m: 0.7
  height_m: 0.15
  rpy_track_deg: [0.0, 0.0, 10.0]
```

Требует одновременно `track` и `ego_motion` с одинаковым reference frame.

## Active interval и motion

```yaml
temporal:
  active_frames:
    start_index: 20
    end_index: 80
  motion:
    type: static
```

или:

```yaml
motion:
  type: constant_track_velocity
  longitudinal_velocity_mps: 0.5
  lateral_velocity_mps: 0.0
  vertical_velocity_mps: 0.0
  yaw_rate_deg_s: 2.0
```

Active interval включительный и должен быть внутри `frames` сценария.

## Material

```yaml
material:
  reflectivity: 0.45
  return_probability_scale: 0.9
```

Оба значения находятся в `[0, 1]`.

## Zero-slot recovery

```yaml
zero_slot_recovery:
  enabled: true
  ring_field: ring
  timestamp_field: timestamp
  timestamp_unit: microseconds
  min_valid_samples_per_ring: 8
  interpolation: linear
  allow_extrapolation: false
  max_angular_error_deg: 0.25
```

`timestamp_unit: auto` не поддерживается намеренно. Если layout не содержит
ring/timestamp, отключите recovery или укажите правильные имена полей.

## Range noise

```yaml
range_noise:
  enabled: true
  model: gaussian
  base_sigma_m: 0.008
  sigma_per_meter: 0.0002
  incidence_sigma_scale: 0.01
  min_range_m: 0.2
  max_resample_attempts: 8
```

Все sigma-компоненты неотрицательные.

## Dropout

```yaml
dropout:
  enabled: true
  base_return_probability: 0.96
  distance_reference_m: 30.0
  distance_exponent: 1.0
  incidence_exponent: 1.5
  min_return_probability: 0.05
```

No-return текущей версии:

```yaml
no_return_encoding: zero_xyz
```

## Intensity

```yaml
intensity:
  enabled: true
  model: empirical
  field_name: intensity
  range_bin_m: 5.0
  min_samples_per_bin: 32
  incidence_exponent: 1.0
  additive_sigma: 1.5
  fallback: preserve_original
  on_missing: error
```

Допустимы `fallback: constant` с полем `constant` и
`on_missing: skip_with_warning`.

## Track polyline

```yaml
track:
  reference_frame: map
  model:
    type: polyline
    points_m:
      - [0.0, 0.0, 0.0]
      - [20.0, 0.0, 0.0]
      - [40.0, 2.0, 0.2]
  up_hint: [0.0, 0.0, 1.0]
```

Нулевые сегменты запрещены. `longitudinal_m` измеряется по длине полилинии.

## Ego trajectory

```yaml
ego_motion:
  source: trajectory_csv
  path: ../../trajectories/run.csv
  reference_frame: map
  lidar_frame: lidar
  timestamp_unit: nanoseconds
  max_interpolation_gap_ms: 100
```

CSV:

```text
timestamp_ns,x_m,y_m,z_m,qx,qy,qz,qw
```

Timestamp должен строго возрастать. Экстраполяция запрещена.

## Полное отключение шагов 8–10

Используйте schema v2 либо в v3:

```yaml
zero_slot_recovery:
  enabled: false

sensor_effects:
  range_noise: {enabled: false, ...}
  dropout: {enabled: false, ...}
  intensity: {enabled: false, ...}
```

Для v3 все параметры всё равно задаются и валидируются, даже если эффект
выключен. Это делает сценарий самодостаточным и воспроизводимым.
