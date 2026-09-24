# Генерация синтетических LiDAR-данных в ROS 2 bag

Пакет читает ROS 2 bag, внедряет синтетические препятствия в выбранные
`sensor_msgs/msg/PointCloud2`, сохраняет новый bag, JSONL ground truth и, при
необходимости, `visualization_msgs/msg/MarkerArray` для RViz2. Исходный bag и
существующие output-пути никогда автоматически не удаляются и не
перезаписываются.

## Возможности

- безопасный deserialize/process/serialize roundtrip всех исходных сообщений;
- сохранение topic, порядка, bag timestamp и бинарной раскладки PointCloud2;
- включительный диапазон кадров, считаемый только на выбранном LiDAR-топике;
- единая Open3D-сцена для нескольких `box`, `cylinder`, `human_mesh` и `cable`;
- взаимная окклюзия: каждому лучу назначается только ближайший синтетический объект;
- JSONL со статистикой каждого объекта и каждого обработанного кадра;
- `/synthetic/markers` с геометрией, реально заменёнными точками, текстом и AABB;
- schema v1 для совместимости, v2 для нескольких объектов и v3 для сенсорной
  модели, zero slots и temporal placement;
- детерминированные независимые RNG-потоки для noise, dropout и intensity.

## Подробная документация

- [Индекс документации](docs/README.md) — рекомендуемый порядок чтения.
- [Архитектура системы](docs/ARCHITECTURE.md) — слои, поток данных,
  координатные соглашения и архитектурные инварианты.
- [Карта исходного кода](docs/CODE_GUIDE.md) — основные файлы, функции, классы,
  структуры и их взаимодействие.
- [Установка и окружение](docs/INSTALLATION.md) — ROS 2, WSL, virtualenv,
  colcon, зависимости и проверка установки.
- [Настройка Scenario YAML](docs/CONFIGURATION.md) — schema v1–v3, геометрии,
  визуализация, сенсорная модель, trajectory и track.
- [Работа с ROS 2 bags](docs/BAG_WORKFLOW.md) — identity roundtrip, изменение
  исходных записей, выходные артефакты и проверка результата.
- [Настройка RViz2](docs/RVIZ_GUIDE.md) — Fixed Frame, PointCloud2,
  MarkerArray, цвета, namespace и lifetime.
- [Тестирование](docs/TESTING.md) — unit/integration/regression тесты,
  воспроизводимость и проверка реального bag.
- [Диагностика](docs/TROUBLESHOOTING.md) — типичные ошибки конфигурации,
  зависимостей, trajectory, zero slots и RViz.

## Окружение и установка

Поддерживаемая конфигурация: Ubuntu 22.04, ROS 2 Humble, Python 3.10. Нужны
`rosbag2_py`, NumPy, PyYAML, Open3D, Trimesh, `visualization_msgs`,
`geometry_msgs`, `std_msgs`, launch и RViz2.

```bash
source /opt/ros/humble/setup.bash
cd /path/to/HackathonLoDT/synthetic_data_generation
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python -m pip install --editable .
```

Сборка ROS-пакета:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --base-paths synthetic_data_generation \
  --packages-select synthetic_data_generation
source install/setup.bash
```

## Запуск генерации

```bash
python -m synthetic_data_generation.bag_roundtrip \
  --input /path/to/input_bag \
  --output /path/to/new_output_bag \
  --scenario config/scenarios/diag_mixed_geometries.yaml
```

Выходной путь должен быть новым каталогом. Рядом создаётся
`<output>.annotations.jsonl`; этот файл также не должен существовать.

`frames.start_index` и `frames.end_index` включаются в диапазон. Нумерация
начинается с нуля и увеличивается только для сообщений `source.pointcloud_topic`.
Если хотя бы одного индекса нет, программа завершается ошибкой.

## Система координат и transform

Все значения задаются в метрах в `PointCloud2.header.frame_id`:

- X — вперёд;
- Y — влево;
- Z — вверх;
- начало лучей LiDAR — `(0, 0, 0)`.

`rpy_deg` означает roll вокруг X, pitch вокруг Y и yaw вокруг Z. Вращение:

```text
R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
```

Для mesh порядок преобразований:

```text
v_lidar = T_pose @ T_normalization @ S @ v_local
```

Масштаб применяется к вершинам, но не к мировому переносу `pose.xyz_m`.

Schema v1 сохраняет прежнюю семантику box: `xyz_m` — центр box, вращение равно
нулю, визуализация выключена. В schema v2 `box`, `cylinder` и `human_mesh` с
`origin: base_center` используют `xyz_m` как центр основания.

## Scenario YAML v2

Полный пример: `config/scenarios/diag_mixed_geometries.yaml`.

Общая визуализация:

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

Lifetime обязан быть положительным, поэтому маркеры не остаются в RViz
навсегда. При `enabled: false` marker topic и дополнительные сообщения не
создаются; PointCloud2 и JSONL от этого не меняются.

### Box

```yaml
geometry:
  type: box
  dimensions_m: [0.6, 0.4, 0.3]
```

Локальный origin schema v2 — центр основания, локальный объём по Z: `[0, size_z]`.

### Cylinder

```yaml
geometry:
  type: cylinder
  radius_m: 0.25
  height_m: 1.2
  radial_segments: 32
```

Локальная ось — `+Z`, origin — центр основания, торцы закрыты.
`radial_segments` допускается в диапазоне `[8, 256]`.

### Human mesh

```yaml
geometry:
  type: human_mesh
  path: ../../meshes/humans/synthetic_person.obj
  units: meters
  scale: [1.0, 1.0, 1.0]
  origin: base_center
  mesh_resource_uri: package://synthetic_data_generation/meshes/humans/synthetic_person.obj
```

Поддерживаются `.obj` и `.stl`. Относительный `path` разрешается относительно
YAML. `units`: `meters`, `centimeters` или `millimeters`. При `base_center`
минимальный Z становится нулём, а центр XY bounds — `(0, 0)`. Для RViz
предпочтителен переносимый `package://` URI; без него используется
`TRIANGLE_LIST`, а для слишком большого mesh — AABB fallback. Production-модели
нужно добавлять только после проверки лицензии.

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

Для ray casting строятся закрытые цилиндрические сегменты и сферические стыки;
RViz показывает ту же преобразованную полилинию как `LINE_STRIP` толщиной
`2 * radius_m`. Последовательные совпадающие точки запрещены.

## Окклюзия и разметка

Все mesh одного кадра добавляются в одну `RaycastingScene` в порядке YAML.
Open3D возвращает ближайшие `t_hit` и `geometry_id`; программа явно связывает
каждый ID с `object_id`. Замена выполняется только при:

```text
finite(t_hit) and epsilon < t_hit < original_range - epsilon
```

Дальний синтетический объект не получает visible/modified point, если его
закрыл ближний. Объект за реальным фоном также не меняет точку.

В каждой JSONL-строке присутствуют все объекты в порядке YAML, включая объекты
с нулевой видимостью. Записываются pose, bounds, intersections, visible points
и modified slots; для mesh также path и SHA-256. Сумма object
`modified_slot_count` равна общему числу изменённых слотов кадра.

## MarkerArray и цвета

После каждого обработанного облака в bag сразу записывается один MarkerArray с
тем же bag timestamp. Каждый marker копирует `header.frame_id` и `header.stamp`
облака. На объект создаются включённые в YAML представления:

- `synthetic/<object_id>/geometry`, ID `0`;
- `synthetic/<object_id>/points`, ID `1`;
- `synthetic/<object_id>/text`, ID `2`;
- `synthetic/<object_id>/bbox`, ID `3`.

`POINTS` содержит только новые координаты реально заменённых слотов данного
объекта. Цвет берётся из `objects[].visualization.color_rgba`, а не из
нестабильного Python `hash()`.

## Schema v3: сенсорная модель и временная согласованность

Полный пример находится в `config/scenarios/diag_track_and_sensor_model.yaml`, а
синтетическая trajectory — в `trajectories/synthetic_run.csv`. Порядок обработки
одного кадра фиксирован:

```text
decode → valid/zero classification → direction recovery → rays
→ pose текущего кадра → ray casting → background occlusion
→ range noise → dropout → intensity → encode → MarkerArray → JSONL
```

Чтобы полностью отключить шаги 8–10, используйте schema v2 либо schema v3 с
`zero_slot_recovery.enabled: false`, всеми тремя sensor effects `enabled: false`
и только `placement.type: lidar_relative` со `motion.type: static`.

### Range noise

Шум применяется только к идеальным синтетическим hit после окклюзии:

```text
sigma = base_sigma_m
      + sigma_per_meter * r_hit
      + incidence_sigma_scale * (1 - incidence_cosine)
r_noisy = r_hit + Normal(0, sigma²)
```

`incidence_cosine = abs(dot(-ray_direction, normalized_primitive_normal))`.
Некорректная нормаль получает документированный fallback `1.0` и учитывается
счётчиком. Выборка повторяется до `max_resample_attempts`; после этого дальность
clamp-ится между `min_range_m` и известным реальным фоном. Для восстановленного
нулевого слота фон равен бесконечности.

### Dropout и no-return

Вероятность возврата:

```text
distance_factor = min(1, (distance_reference_m / range) ^ distance_exponent)
incidence_factor = incidence_cosine ^ incidence_exponent
p_return = clamp(base_probability * distance_factor * incidence_factor
                 * material.return_probability_scale,
                 min_return_probability, 1)
```

Dropout не возвращает закрытый реальный фон. Стратегия `zero_xyz` записывает
`x=y=z=0` и `intensity=0`, сохраняя ring, timestamp, остальные поля, layout и
число слотов.

### Empirical intensity

Intensity генерируется только для synthetic returns, переживших dropout.
Донор выбирается воспроизводимо из текущего реального кадра по range bin с
fallback к ближайшему bin, затем:

```text
I = I_sample * material.reflectivity
  * incidence_cosine ^ incidence_exponent
  + Normal(0, additive_sigma²)
```

Значение ограничивается диапазоном фактического datatype `PointField`. Поле не
добавляется автоматически. `on_missing` задаёт `error` или
`skip_with_warning`; `fallback` — `preserve_original` или `constant`.

### Независимые RNG-потоки

Глобальный `np.random` и Python `hash()` не используются. Ключ объекта — первые
32 бита SHA-256 от UTF-8 `object_id`. Каждый поток строится как:

```text
SeedSequence([scenario_seed, frame_index, stable_object_key, effect_code])
```

Noise, dropout и intensity имеют разные постоянные `effect_code`. Поэтому
включение intensity не меняет dropout mask, а изменение одного объекта не
сдвигает случайные числа другого.

## Восстановление нулевых слотов

Слоты классифицируются отдельно:

- valid return — конечный XYZ с range больше epsilon;
- zero slot — конечный XYZ с range не больше epsilon;
- invalid nonzero — NaN/Inf, который не восстанавливается.

Для каждого ring по валидным точкам берётся median elevation. Azimuth
`atan2(y,x)` сортируется по timestamp, проходит `unwrap`, после чего линейно
интерполируется на timestamp zero slot. Направление используется только при
достаточном числе образцов, допустимом timestamp и p95 angular error не выше
`max_angular_error_deg`.

`timestamp_unit` задаётся явно: `seconds`, `milliseconds`, `microseconds`,
`nanoseconds` или `relative_ticks`. `auto` отклоняется как неоднозначный. Для
интерполяции важен относительный порядок; trajectory всегда использует
абсолютные `timestamp_ns`.

Если облако не сохраняет no-return слоты вообще, отсутствующие лучи не
выдумываются. Ring и timestamp восстановленного слота не изменяются. JSONL
содержит zero/recovered counts, причины отказов и median/p95/max angular error.

## Ego motion, track и placement

Матрица `T_A_B` преобразует координаты из B в A:

```text
p_A = T_A_B @ p_B
T_lidar_object = inverse(T_reference_lidar) @ T_reference_object
```

Надёжный источник pose текущей версии — `trajectory_csv`:

```text
timestamp_ns,x_m,y_m,z_m,qx,qy,qz,qw
```

Translation интерполируется линейно, rotation — quaternion SLERP. Quaternion
нормализуется; экстраполяция и интервалы больше `max_interpolation_gap_ms`
запрещены. TF-provider пока не реализован и identity transform для production
track-relative режима не подставляется.

Track задаётся полилинией в reference frame. `longitudinal_m` — arc length, а
не X и не индекс вершины. Базис:

```text
T = normalized tangent
L = normalize(up_hint × T)       # влево
U = normalize(T × L)             # вверх
```

Положение центра основания:

```text
P_reference = C(s) + lateral_m * L(s) + height_m * U(s)
R_reference_object = [T L U] @ Rz(yaw) @ Ry(pitch) @ Rx(roll)
```

`static` сохраняет pose объекта в reference frame. Для
`constant_track_velocity` состояние вычисляется от timestamp первого активного
кадра:

```text
s=s0+v_longitudinal*dt; l=l0+v_lateral*dt; h=h0+v_vertical*dt
yaw=yaw0+yaw_rate*dt
```

`active_frames` имеет включительные границы. Неактивный объект не попадает в
сцену, помечается `active: false` в JSONL, а его RViz marker исчезает по lifetime.

Весь PointCloud2 обрабатывается на одном reference timestamp: сначала
`header.stamp`, при нуле — bag timestamp. Timestamp отдельных точек используется
для восстановления направления, но внутрискановое движение и deskew не
моделируются.

## Просмотр в RViz2

```bash
ros2 bag play /path/to/new_output_bag
ros2 launch synthetic_data_generation preview.launch.py
```

Готовая конфигурация добавляет `/lidar_points` и `/synthetic/markers`. Если
реальный топик называется иначе, измените его в RViz. `Fixed Frame` должен быть
равен `PointCloud2.header.frame_id`. Пакет не создаёт TF и не скрывает его
отсутствие.

## Тесты

```bash
source /opt/ros/humble/setup.bash
source .venv/bin/activate
python -m pytest -q
```

Тесты программно создают маленькие PointCloud2 и временные SQLite3 bags. Реальные
данные хакатона не используются. Mesh fixture `synthetic_person.obj` создан в
проекте и имеет CC0-декларацию.

## Ограничения

- только система координат LiDAR, без track/map transform;
- lidar-relative pose фиксирован; track-relative pose следует заданной временной модели;
- moving pose поддерживается только простой track-velocity моделью;
- TF-provider и intra-scan motion compensation пока не реализованы;
- не создаются новые угловые слоты LiDAR — заменяются существующие возвраты;
- human mesh должен быть доступен локально и иметь треугольные грани;
- лицензии внешних production mesh проверяются отдельно;
- big-endian PointCloud2 отклоняется;
- при ошибке частично созданный новый output не удаляется автоматически.
