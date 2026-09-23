# Настройка сценария в YAML

Сценарий описывает, **в какие кадры** и **куда** поместить объекты, как изменять
LiDAR-возвраты и какие маркеры записать для RViz2. Готовые полные файлы
находятся в `config/scenarios/`. Скопируйте подходящий пример под новым именем
и меняйте значения в копии.

## Как записывать YAML

```yaml
schema_version: 2
source:
  pointcloud_topic: /lidar_points
frames: {start_index: 80, end_index: 100}
objects:
  - id: box_1
    class_name: obstacle
    geometry: {type: box, dimensions_m: [1.0, 1.0, 2.0]}
    pose: {coordinate_system: lidar, xyz_m: [8.0, 0.0, -1.0], rpy_deg: [0.0, 0.0, 0.0]}
```

- Отступы задают вложенность; используйте пробелы, не табуляцию. Записи
  `frames: {start_index: 80, end_index: 100}` и многострочная форма равнозначны.
- `-` начинает элемент списка; `objects` содержит один или несколько объектов.
- `[x, y, z]` — список из трёх чисел именно в этом порядке. `true` и `false`
  пишутся без кавычек. Десятичный разделитель — точка.
- Имена ключей и значения вроде `lidar_relative` пишутся точно как в
  примерах. Парсер отклоняет неизвестные и отсутствующие обязательные поля.
- Суффиксы `_m`, `_mps`, `_deg`, `_s`, `_ms`, `_ns` означают метры, метры в
  секунду, градусы, секунды, миллисекунды и наносекунды соответственно.
- Относительные пути к mesh и CSV отсчитываются **от каталога YAML-файла**,
  а не от текущего каталога терминала.
- После изменения YAML заново сгенерируйте bag: `ros2 bag play` уже готового
  bag не перечитывает сценарий.

Полный сценарий содержит `schema_version`, `scenario_id`, `seed`, `source`,
`frames`, `objects`, `sensor_effects`. Для v3 также обязательны
`visualization` и `zero_slot_recovery`; `track` и `ego_motion` нужны при
`track_relative`. Фрагмент выше показывает только синтаксис. Полный пример
v2 — `config/scenarios/visible_objects_lidar.yaml`, v3 —
`config/scenarios/track_sensor_model.yaml`.

## Версии и выбор режима

| Версия | Назначение | Положение объекта | Сенсорная модель |
| --- | --- | --- | --- |
| v1 | Совместимость со старым одиночным box | `pose`, без поворота | Выключена |
| v2 | Несколько объектов и цветные маркеры | `pose` в LiDAR frame | Выключена |
| v3 | Восстановление лучей, эффекты и движение | `placement` + `temporal` | Настраивается |

Для первого визуального прогона выбирайте v2. `track_relative` в v3 требует
отдельно подготовленной траектории; если её нет, используйте
`lidar_relative`.

## Системы координат, повороты и время

**LiDAR frame** — система координат облака: `X` вперёд, `Y` влево, `Z` вверх.
Проверьте фактический `PointCloud2.header.frame_id` своего bag. Положение
`[10, 0, -1]` означает 10 м впереди, без бокового смещения, на 1 м ниже
начала координат LiDAR. Для `pose.coordinate_system: lidar` и
`placement.type: lidar_relative` оно повторяется в каждом кадре.
Это соглашение сценария, а не автоматическое преобразование входного облака:
генератор использует XYZ из bag как есть. Проверяйте фактический сектор лучей
своего датасета. Например, у `roundT_doubleT` ненулевые точки лежат при
`Y < 0`, поэтому объект на `[10, 0, z]` не пересекает существующие лучи.

**Track frame** строится по полилинии: продольная ось направлена от первой
точки к последней, боковая — влево относительно направления пути при
`up_hint: [0, 0, 1]`, вертикальная — вверх. Положительный `lateral_m`
смещает объект влево, положительный `height_m` — вверх. `track` и позы из
`ego_motion` должны быть в одном `reference_frame`, например `map`.
Рельсы автоматически не распознаются: полилинию задаёт пользователь.

`rpy_deg` и `rpy_track_deg` идут в порядке **[roll, pitch, yaw]** в градусах.
Положительный поворот следует правилу правой руки: roll вокруг X, pitch
вокруг Y, yaw вокруг Z. Поворот составляется как `Rz(yaw) @ Ry(pitch) @
Rx(roll)`. Например, `[0, 0, 90]` поворачивает локальную ось X объекта к
положительной Y. В `rpy_track_deg` оси принадлежат track frame.

В v3 время кадра берётся из `PointCloud2.header.stamp`; **только при нулевом
header stamp** используется bag timestamp. Это время сравнивается с
`ego_motion` CSV. Время отдельных точек применяется только при
`zero_slot_recovery`; компенсации движения внутри скана нет. Номер кадра
считается отдельно от времени.

## Общие поля

```yaml
schema_version: 2
scenario_id: visible_objects_lidar
seed: 17421
source:
  pointcloud_topic: /lidar_points
frames:
  start_index: 80
  end_index: 100
```

| Параметр | Что делает |
| --- | --- |
| `schema_version` | Выбирает формат 1, 2 или 3. Поля разных версий нельзя смешивать. |
| `scenario_id` | Непустое имя сценария для вывода и JSONL; на положение объекта не влияет. |
| `seed` | Целое число от 0 до 2³²−1 для воспроизводимых случайных эффектов. |
| `source.pointcloud_topic` | Имя существующего топика `sensor_msgs/msg/PointCloud2`, который изменяется. Остальные входные топики проходят без изменений. |
| `frames.start_index`, `frames.end_index` | Первый и последний кадры **включительно**. Индексация с нуля и только среди сообщений выбранного PointCloud2-топика; `/tf`, IMU и камера индекс не увеличивают. |

## `visualization` — маркеры для RViz2

Блок создаёт дополнительный топик `visualization_msgs/msg/MarkerArray`.
Цвета из YAML относятся к маркерам объектов, **не окрашивают PointCloud2**.
В RViz2 нужны два display: `PointCloud2` на исходном топике и `MarkerArray`
на `marker_topic`. `Fixed Frame` должен совпадать с `header.frame_id`
облака либо иметь доступный TF. Генератор TF не создаёт.

```yaml
visualization:
  enabled: true
  marker_topic: /synthetic/markers
  marker_mode: timed
  point_size_m: 0.06
  marker_lifetime_sec: 0.25
  show_geometry: true
  show_modified_points: true
  show_text: true
  show_bounding_box: true
```

| Параметр | Что делает |
| --- | --- |
| `enabled` | `true` записывает один MarkerArray на обработанный кадр; `false` не создаёт топик маркеров. На изменение PointCloud2 не влияет. |
| `marker_topic` | Имя нового топика маркеров; не должно совпадать с входным топиком или другим топиком bag. |
| `marker_mode` | `timed` (по умолчанию) удаляет маркеры по времени. `frame` удаляет предыдущие маркеры при следующем кадре и сохраняет текущие на паузе. |
| `point_size_m` | Размер маркера `POINTS` для возвращённых синтетических точек, в метрах. Не меняет размер точек display PointCloud2; > 0. |
| `marker_lifetime_sec` | В режиме `timed` — время жизни маркера в RViz в секундах. В режиме `frame` игнорируется, но остаётся обязательным положительным полем YAML. |
| `show_geometry` | Показывает поверхность или форму активного объекта: CUBE, CYLINDER, mesh или линию cable. Может быть видна и при нуле добавленных точек. |
| `show_modified_points` | Показывает цветные `POINTS` только для действительно записанных синтетических возвратов. При нуле изменённых слотов маркер пустой. |
| `show_text` | Показывает над объектом ID, класс, тип геометрии и число изменённых слотов. |
| `show_bounding_box` | Показывает осевой ограничивающий параллелепипед в LiDAR frame. |

В v2 блок `visualization` можно опустить — тогда он выключен. В v3 блок
обязателен; четыре флага `show_*` можно опустить, тогда каждый равен `true`.
В режиме `frame` генератор добавляет `DELETEALL` перед маркерами каждого
обрабатываемого кадра и ещё одно очищающее сообщение на первом кадре после
диапазона `frames`, если такой кадр есть в bag. Это предотвращает наложение
рамок разных объектов. `DELETEALL` действует на весь выделенный marker topic;
не публикуйте на нём чужие маркеры.

## `objects` — список объектов

Каждый элемент списка — отдельный объект. `id` должен быть уникальным и
непустым; он используется в статистике, JSONL и namespace маркеров.
`class_name` — текстовая метка, например `obstacle` или `person`; сама по
себе геометрию и отражение не меняет.

### Цвет одного объекта

```yaml
objects:
  - id: box_1
    class_name: obstacle
    geometry: {type: box, dimensions_m: [1.0, 1.0, 2.0]}
    pose: {coordinate_system: lidar, xyz_m: [8.0, 0.0, -1.0], rpy_deg: [0, 0, 0]}
    visualization:
      color_rgba: [1.0, 0.2, 0.1, 0.9]
```

`objects[].visualization.color_rgba` — `[red, green, blue, alpha]`, четыре
числа от 0 до 1. `alpha` задаёт непрозрачность: 1 — непрозрачный маркер;
должен быть больше нуля. Цвет виден только в display `MarkerArray` и не
записывается как RGB в PointCloud2. В v2 цвет можно опустить, тогда
используется `[1.0, 0.2, 0.1, 0.8]`; в v3 он обязателен.

### Геометрия: `box`

```yaml
geometry:
  type: box
  dimensions_m: [0.6, 0.4, 0.3]
```

`type` выбирает параллелепипед. `dimensions_m` — размеры вдоль **локальных
осей объекта** `[X, Y, Z]` в метрах; все > 0. В v2/v3 локальное начало
находится **в центре нижней грани**: при `pose.xyz_m: [8, 0, -1]` и высоте
2 м объект занимает Z от −1 до +1 м до поворота. В v1 старый box
центрирован по всем трём осям; это важно при переносе сценария.

### Геометрия: `cylinder`

```yaml
geometry:
  type: cylinder
  radius_m: 0.25
  height_m: 1.2
  radial_segments: 32
```

`radius_m` — радиус в метрах, `height_m` — высота вдоль локальной +Z от
нижнего основания; оба > 0. `radial_segments` — число граней аппроксимации
окружности, целое от 8 до 256; большее число делает цилиндр глаже.

### Геометрия: `human_mesh`

```yaml
geometry:
  type: human_mesh
  path: ../../meshes/humans/low_poly_person.obj
  units: meters
  scale: [1.0, 1.0, 1.0]
  origin: base_center
  # mesh_resource_uri: package://synthetic_data_generation/meshes/humans/low_poly_person.obj
```

| Параметр | Что делает |
| --- | --- |
| `path` | Путь к локальному `.obj` или `.stl` для ray casting, относительно YAML; файл должен существовать. Загрузка требует `trimesh`. |
| `units` | Единицы координат исходного mesh: `meters`, `centimeters` или `millimeters`; приводятся к метрам. |
| `scale` | Дополнительные положительные масштабы по локальным X/Y/Z после учёта `units`. `[1, 1, 1]` не меняет размеры. |
| `origin` | `base_center` переносит центр XY-габарита в ноль и нижнюю точку на Z=0; `mesh_origin` сохраняет исходное начало координат mesh. |
| `mesh_resource_uri` | Необязательный `package://` URI для отображения mesh в RViz. Ray casting всё равно использует `path`. Без URI используется маркер из треугольников или box для очень большого mesh. |

### Геометрия: `cable`

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

`control_points_m` — упорядоченные точки в **локальной** системе объекта,
в метрах; нужны минимум две, соседние не должны совпадать. Между точками
строятся цилиндрические участки со сферическими стыками. `radius_m` — радиус
этих участков, > 0; `radial_segments` — число граней поперечного сечения,
от 8 до 256. RViz показывает осевую линию, ray casting — объёмную поверхность.

## Размещение в v1/v2: `pose`

```yaml
pose:
  coordinate_system: lidar
  xyz_m: [12.0, 0.5, -1.2]
  rpy_deg: [0.0, 0.0, 15.0]
```

`coordinate_system` принимает только `lidar`. `xyz_m` — положение локального
начала геометрии в LiDAR frame: X вперёд, Y влево, Z вверх, в метрах.
Для box v2 и cylinder это **центр основания**, а не центр объёма.
`rpy_deg` — поворот вокруг локального начала в градусах; порядок и знаки
описаны выше. В v1 все углы обязаны быть нулевыми. В v2 pose неизменна
от кадра к кадру: объект словно прикреплён к LiDAR.

## Размещение в v3: `placement`

### `lidar_relative` — относительно LiDAR

```yaml
placement:
  type: lidar_relative
  xyz_m: [10.0, 0.0, -1.0]
  rpy_deg: [0.0, 0.0, 15.0]
```

`type` выбирает режим, `xyz_m` и `rpy_deg` имеют тот же смысл, что и в
`pose`. Не нужны `track` и `ego_motion`. В текущей v3 поддерживается только
`temporal.motion.type: static`; объект остаётся на одном месте относительно
LiDAR даже при движении машины.

### `track_relative` — относительно пути

```yaml
placement:
  type: track_relative
  longitudinal_m: 35.0
  lateral_m: 0.7
  height_m: 0.15
  rpy_track_deg: [0.0, 0.0, 10.0]
```

`longitudinal_m` — расстояние **вдоль всей полилинии** от её первой точки
до объекта, в метрах; должно оставаться в пределах длины пути. Это не
координата X в `map`. `lateral_m` — смещение влево от направления пути,
`height_m` — вверх от линии пути, оба в метрах; отрицательные значения
направлены вправо/вниз. `rpy_track_deg` поворачивает объект в осях пути,
в градусах. Режим требует `track` и `ego_motion` в одной системе координат.

## `temporal` — время активности и движение объекта (v3)

```yaml
temporal:
  active_frames: {start_index: 20, end_index: 80}
  motion: {type: static}
```

`active_frames.start_index` и `end_index` включительны и обязаны лежать
внутри глобального `frames`. Вне интервала объект не добавляется в сцену
и не имеет маркера. `motion.type: static` сохраняет параметры размещения
постоянными: для `track_relative` это неподвижное место в `reference_frame`,
хотя координаты в LiDAR frame меняются при движении LiDAR.

```yaml
motion:
  type: constant_track_velocity
  longitudinal_velocity_mps: 0.5
  lateral_velocity_mps: 0.0
  vertical_velocity_mps: 0.0
  yaw_rate_deg_s: 2.0
```

`constant_track_velocity` доступен только для `track_relative`.
Значения прибавляются к исходным координатам или yaw с момента **первого
активного кадра**: скорость вдоль пути, боковая скорость, вертикальная
скорость (м/с), скорость yaw (градусы/с). Например,
`longitudinal_m(t) = longitudinal_m(0) + longitudinal_velocity_mps × t`.
Положительные боковая и вертикальная скорости направлены влево и вверх.
Roll и pitch не меняются. Весь активный интервал должен оставаться
внутри длины пути.

## `material` — свойства синтетического возврата (v3)

```yaml
material:
  reflectivity: 0.45
  return_probability_scale: 0.9
```

Оба параметра в `[0, 1]`. `reflectivity` умножает вычисленную синтетическую
интенсивность, **только если включён** `sensor_effects.intensity`;
не влияет на цвет маркера или пересечение луча. `return_probability_scale`
умножает вероятность получить возврат, **только если включён** dropout;
само по себе объект невидимым не делает.

## `zero_slot_recovery` — восстановление лучей из пустых слотов (v3)

Некоторые LiDAR сохраняют слот луча с `x=y=z=0`, когда отражения не было.
Из нулевого XYZ нельзя узнать направление луча напрямую. Этот блок
пытается восстановить его по **кольцу сканирования (`ring`)** и времени
отдельной точки. После восстановления в пустой слот можно записать
синтетическое попадание. Без восстановления обрабатываются только лучи,
для которых во входном облаке уже есть ненулевая конечная точка.

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

| Параметр | Что делает |
| --- | --- |
| `enabled` | Включает восстановление нулевых слотов. При `false` нули остаются без направлений; остальные параметры всё равно обязательны в v3. |
| `ring_field` | Имя **существующего числового** поля номера лазерного канала в PointCloud2, например `ring`. |
| `timestamp_field` | Имя **существующего числового** поля времени отдельной точки, например `timestamp`. Это не `header.stamp` кадра. |
| `timestamp_unit` | Объявленные единицы поля: `seconds`, `milliseconds`, `microseconds`, `nanoseconds` или `relative_ticks`. `auto` запрещён. Текущий алгоритм использует сырые значения для порядка и линейной интерполяции, без перевода в секунды. |
| `min_valid_samples_per_ring` | Минимум ненулевых точек того же ring для построения модели направления; целое ≥ 2. |
| `interpolation` | Только `linear`: азимут интерполируется по времени точки; угол возвышения берётся как медиана валидных точек ring. |
| `allow_extrapolation` | При `false` отбрасывает нулевые слоты за временным диапазоном известных точек ring. При `true` допускает такие слоты; текущая реализация удерживает ближайшее граничное значение азимута, а не продолжает наклон. |
| `max_angular_error_deg` | Допустимый 95-й процентиль ошибки направления модели на валидных точках ring, в градусах; при превышении весь ring не восстанавливается. Значение > 0. |

Требуемые поля автоматически не добавляются. Если bag не хранит нулевые
слоты или поля `ring`/времени отсутствуют, оставьте `enabled: false`.
Ненулевые точки с NaN/Inf не восстанавливаются. В JSONL есть число
восстановленных направлений и причины отказов.

## `sensor_effects` — модель измерения LiDAR

Геометрия сначала даёт идеальное синтетическое пересечение. Затем v3
последовательно применяет ошибку дальности, потерю возврата и интенсивность.
Эффекты меняют только синтетические попадания. Цвет маркеров задаётся
`objects[].visualization.color_rgba` отдельно.

### v1/v2: эффекты отключены

```yaml
sensor_effects:
  range_noise: false
  dropout: false
  modify_intensity: false
```

Все три ключа обязательны и в v1/v2 принимают только `false`.

### v3: общий блок и отсутствие возврата

```yaml
sensor_effects:
  no_return_encoding: zero_xyz
  range_noise: {enabled: false, model: gaussian, base_sigma_m: 0.0, sigma_per_meter: 0.0, incidence_sigma_scale: 0.0, min_range_m: 0.2, max_resample_attempts: 8}
  dropout: {enabled: false, base_return_probability: 1.0, distance_reference_m: 30.0, distance_exponent: 1.0, incidence_exponent: 1.0, min_return_probability: 0.0}
  intensity: {enabled: false, model: empirical, field_name: intensity, range_bin_m: 5.0, min_samples_per_bin: 32, incidence_exponent: 1.0, additive_sigma: 0.0, fallback: preserve_original, on_missing: error}
```

`no_return_encoding: zero_xyz` — единственный поддерживаемый формат для
потерянного синтетического возврата: XYZ соответствующего слота заменяется
нулями. Если поле интенсивности есть, интенсивность такого слота также
обнуляется. Все вложенные параметры обязательны и проверяются даже при
`enabled: false`.

Следующие три примера показывают вложенные блоки внутри `sensor_effects`;
при записи полного сценария сохраняйте эту вложенность.

### `range_noise` — шум дальности

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

`enabled` включает эффект; `model` сейчас только `gaussian`. Шум
прибавляется **вдоль исходного луча**, а не сдвигает точку вбок.
Стандартное отклонение в метрах:

```text
sigma = base_sigma_m + sigma_per_meter × range_m
        + incidence_sigma_scale × (1 − cos_incidence)
```

`base_sigma_m` — постоянная часть (м); `sigma_per_meter` — добавка на метр
дальности; `incidence_sigma_scale` — добавка при скользящем падении луча
(м). Все три ≥ 0. `min_range_m` — минимально допустимая дальность, > 0.
`max_resample_attempts` — число повторных попыток, если точка получилась
слишком близко или за исходным фоном; целое 0–100. Затем дальность
ограничивается допустимыми границами.

### `dropout` — потеря отражения

```yaml
dropout:
  enabled: true
  base_return_probability: 0.96
  distance_reference_m: 30.0
  distance_exponent: 1.0
  incidence_exponent: 1.5
  min_return_probability: 0.05
```

`enabled` включает случайное удаление синтетических попаданий.
`base_return_probability` — вероятность до поправок, `[0, 1]`.
`distance_reference_m` — дальность, после которой вероятность уменьшается,
> 0 м; `distance_exponent` задаёт силу уменьшения, ≥ 0.
`incidence_exponent` уменьшает вероятность при скользящем угле, ≥ 0.
`min_return_probability` — нижняя граница после множителей, `[0, 1]`.
`material.return_probability_scale` объекта — дополнительный множитель.
При `dropout.enabled: false` эти параметры не удаляют точки.

### `intensity` — интенсивность синтетического возврата

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
  # constant: 50.0  # для fallback: constant
```

| Параметр | Что делает |
| --- | --- |
| `enabled` | Записывает синтетическую интенсивность для возвратов, переживших dropout. |
| `model` | Сейчас только `empirical`: значение выбирается из интенсивностей реальных точек входного кадра. |
| `field_name` | Имя существующего скалярного поля PointCloud2, например `intensity`; новое поле не создаётся. |
| `range_bin_m` | Ширина группы по дальности, > 0 м; доноры сначала ищутся в той же группе, что и синтетическая точка. |
| `min_samples_per_bin` | Минимум доноров в той же группе, целое ≥ 1; иначе выбирается ближайшая непустая группа. |
| `incidence_exponent` | Степень множителя `cos_incidence` для интенсивности, ≥ 0: скользящие лучи становятся слабее. |
| `additive_sigma` | Стандартное отклонение добавочного гауссова шума в единицах поля, ≥ 0. |
| `fallback` | Если **во всём кадре нет доноров**, `preserve_original` берёт прежнее значение слота, `constant` — число из `constant`. |
| `constant` | Необязательное число для `fallback: constant`; если опущено, равно 0. |
| `on_missing` | Если поля `field_name` нет: `error` завершает обработку, `skip_with_warning` оставляет интенсивность без генерации. |

Выбранная интенсивность умножается на `material.reflectivity` и поправку
на угол падения, затем добавляется шум. Результат ограничивается диапазоном
типа поля. Интенсивность **не является RGB-цветом** в RViz2.

## `track` — геометрия пути (v3, для `track_relative`)

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

`reference_frame` — имя системы, в которой заданы точки пути; должно
совпадать с `ego_motion.reference_frame`. `model.type` только `polyline`.
`points_m` — минимум две точки `[x, y, z]` в **reference frame**, в метрах
и в порядке движения вдоль пути; одинаковые соседние точки запрещены.
`up_hint` — ненулевой ориентир направления вверх для построения локальных
осей; не обязан быть нормированным и не должен быть параллелен участку пути.
Длина пути — сумма длин его отрезков.

## `ego_motion` — движение LiDAR (v3, для `track_relative`)

```yaml
ego_motion:
  source: trajectory_csv
  path: ../../trajectories/run.csv
  reference_frame: map
  lidar_frame: lidar
  timestamp_unit: nanoseconds
  max_interpolation_gap_ms: 100
```

| Параметр | Что делает |
| --- | --- |
| `source` | Только `trajectory_csv`: позы читаются из отдельного CSV, не извлекаются автоматически из `/tf` или одометрии bag. |
| `path` | Путь к CSV относительно YAML. Демонстрационный `synthetic_run.csv` не является траекторией реального bag. |
| `reference_frame` | Система координат поз; должна совпадать с `track.reference_frame`. |
| `lidar_frame` | Имя фрейма LiDAR, для которого заданы позы CSV. Указывайте фактический `PointCloud2.header.frame_id`; текущий код не строит TF и не сверяет имя автоматически. |
| `timestamp_unit` | Только `nanoseconds`; столбец CSV называется `timestamp_ns`. |
| `max_interpolation_gap_ms` | Максимальный промежуток между соседними строками CSV, через который разрешена интерполяция, в миллисекундах; > 0. |

Формат CSV:

```csv
timestamp_ns,x_m,y_m,z_m,qx,qy,qz,qw
946692931600000000,0.0,0.0,0.0,0.0,0.0,0.0,1.0
946692931700000000,0.1,0.0,0.0,0.0,0.0,0.0,1.0
```

`timestamp_ns` — время в наносекундах, строго возрастающее.
`x_m,y_m,z_m` — положение LiDAR в `reference_frame`, метры.
`qx,qy,qz,qw` — кватернион ориентации LiDAR относительно этой системы;
он нормализуется при чтении. Нужны минимум две строки. Между строками
позиция интерполируется линейно, ориентация — SLERP. **Экстраполяция за
границы CSV запрещена.** Времена всех обрабатываемых кадров должны попадать
в диапазон CSV. Числа выше иллюстрируют формат, а не готовые позы для bag.

Для `track_relative` используется преобразование
`T_lidar_object = inverse(T_reference_lidar) @ T_reference_object`.
Ошибочный reference frame или несогласованные часы поставят объект не туда
или приведут к ошибке траектории.

## Как проверить результат

1. Проверьте топик и число кадров: `ros2 bag info /path/to/input_bag`.
2. Запустите с **новым** output-путём; существующий output не перезаписывается.
3. Смотрите итоговые `modified slots` и `<output>.annotations.jsonl`.
   Ноль означает, что лучи не дали записанных синтетических возвратов;
   маркер геометрии при этом всё равно может быть виден.
4. В RViz2 включите `PointCloud2` и `MarkerArray` отдельно; проверьте
   `Fixed Frame` и воспроизводите выбранные кадры.

Подробные инструкции: [работа с bag](BAG_WORKFLOW.md),
[RViz2](RVIZ_GUIDE.md), [диагностика](TROUBLESHOOTING.md).
