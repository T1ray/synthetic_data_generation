# Карта исходного кода и взаимодействие компонентов

Этот документ отвечает на вопросы «где реализована логика», «какой объект за
что отвечает» и «в каком месте безопасно расширять систему».

## `bag_roundtrip.py`

Ответственность: CLI, проверка путей и последовательное чтение/запись rosbag2.

Основные элементы:

- `RoundtripError` — ошибки безопасного roundtrip.
- `RoundtripResult` — число входных/выходных сообщений, topic counts и время.
- `validate_paths()` — разделяет input/output и запрещает опасное вложение.
- `prepare_output_path()` — требует несуществующий output.
- `detect_storage_id()` — читает backend из metadata.
- `load_message_types()` — динамически загружает классы ROS-сообщений.
- `open_reader()` / `open_writer()` — создают SequentialReader/Writer.
- `process_message()` — общий extension point.
- `roundtrip_bag()` — главный цикл, включая дополнительные сообщения.
- `build_processing_context()` — выбирает identity, legacy CLI или YAML-сценарий.
- `main()` — консольная точка входа.

`bag_roundtrip.py` не должен знать формулы сенсорной модели или детали Open3D.

## `scenario.py`

Ответственность: строгая типизированная конфигурация schema v1/v2/v3.

Ключевые структуры:

- `Scenario`, `ScenarioObject`;
- `PoseConfig`, `MaterialConfig`;
- `BoxGeometry`, `CylinderGeometry`, `HumanMeshGeometry`, `CableGeometry`;
- `VisualizationConfig`;
- `ZeroSlotRecoveryConfig`;
- `RangeNoiseConfig`, `DropoutConfig`, `IntensityConfig`;
- `SensorEffectsConfig`;
- `LidarPlacement`, `TrackPlacement`;
- `MotionConfig`, `TemporalConfig`;
- `TrackConfig`, `EgoMotionConfig`.

`load_scenario()` разрешает относительные пути относительно YAML.
`parse_scenario()` отклоняет неизвестные поля и неоднозначные значения.

Новое поле сценария сначала добавляется сюда, затем передаётся в потребляющий
модуль. Нельзя читать необработанный YAML непосредственно в processor.

## `processor.py`

Ответственность: orchestration одного выбранного PointCloud2-топика.

`ProcessingContext` хранит:

- выбранный топик и диапазон кадров;
- подготовленные или динамические геометрии;
- `TemporalScene`;
- JSONL writer;
- очередь дополнительных MarkerArray;
- накопленную статистику и тайминги.

Основные методы:

- `from_scenario()` — создаёт контекст;
- `validate_topics()` — проверяет PointCloud2 и marker topic;
- `prepare_outputs()` — готовит геометрии, trajectory и JSONL;
- `additional_topic_metadata()` — регистрирует MarkerArray в writer;
- `process_message()` — выбирает кадр и запускает конвейер;
- `pop_extra_messages()` — отдаёт MarkerArray bag writer;
- `finalize()` — проверяет полноту диапазона;
- `abort()` — закрывает JSONL при ошибке;
- `print_summary()` — выводит итоговые счётчики и тайминги.

Именно processor соединяет независимые модули, но не должен дублировать их
математику.

## `pointcloud_codec.py`

Ответственность: безопасная работа с бинарным `PointCloud2.data`.

- `PointCloudCodecError` — ошибка несовместимого layout.
- `DecodedPointCloud` — полный byte buffer и writable structured NumPy view.
- `_validate_layout()` — проверяет размеры, endian, XYZ и dtype.
- `decode_cloud()` — создаёт view со strides `(row_step, point_step)`.
- `encode_cloud()` — проверяет неизменность layout и возвращает buffer в message.

Этот модуль нельзя заменять преобразованием в простой `N×3`: оно потеряет
padding и дополнительные поля.

## `ray_model.py`

Legacy-модель валидных лучей для cube API. `RayBundle` хранит индексы,
направления, дальности и Open3D rays. Новая schema v3 использует расширенную
модель из `zero_slot_recovery.py`.

## `zero_slot_recovery.py`

Ответственность: категории слотов и восстановление направления.

- `DirectionRecoveryStats` — counts, причины отказов и angular-error metrics.
- `RecoveredRayBundle` — valid и recovered лучи, background ranges и маски.
- `build_rays_with_zero_recovery()` — median elevation по ring, unwrap и
  интерполяция azimuth по timestamp.

Модуль не изменяет PointCloud2 и не выполняет ray casting.

## `geometry.py`

Ответственность: единое построение поверхностей и transform.

- `BuiltGeometry` — vertices/triangles в LiDAR, bounds, marker transform и
  воспроизводимые mesh metadata.
- `pose_transform()` — RPY ZYX и translation.
- `transform_points()` — homogeneous transform точек.
- `build_geometry()` / `build_geometries()` — box, cylinder, mesh и cable.
- `transform_built_geometry()` — дешёвое per-frame преобразование заранее
  подготовленной локальной геометрии.

Human mesh загружается один раз. Для cable строятся цилиндрические сегменты и
сферические стыки, а не только RViz-линия.

## `temporal.py`

Ответственность: pose LiDAR и объектов на timestamp кадра.

- `PoseProvider` — интерфейс источника `T_reference_lidar`.
- `CsvTrajectoryPoseProvider` — CSV, linear translation и quaternion SLERP.
- `TrackFrame` — center/tangent/lateral/up.
- `PolylineTrack` — arc-length model.
- `FrameObjectState` — active flag и pose reference/lidar.
- `TemporalScene` — static/constant-velocity состояние каждого объекта.
- `quaternion_slerp()`, `quaternion_matrix()`, `matrix_quaternion()` — rotation.

TF provider пока отсутствует. Track-relative объект без явной trajectory
отклоняется.

## `object_injector.py`

Ответственность: единая Open3D-сцена и назначение луча объекту.

- `ObjectInjectionStats` — intersections, ideal hits, returns и dropout.
- `FrameInjectionResult` — общий результат для cloud/JSONL/RViz.
- `inject_objects()`:
  1. декодирует cloud;
  2. строит valid/recovered rays;
  3. добавляет active geometry в RaycastingScene;
  4. создаёт `geometry_id → object_id`;
  5. получает `t_hit`, `geometry_ids`, `primitive_normals`;
  6. проверяет фон;
  7. вызывает сенсорную модель;
  8. кодирует исходный layout.

## `sensor_model.py`

Ответственность: эффекты после идеального hit.

- `stable_object_key()` — SHA-256 вместо Python hash.
- `effect_rng()` — независимый RNG-поток.
- `SensorEffectStats`, `SensorModelResult` — фактический результат.
- `apply_sensor_model()` — incidence, noise, dropout, intensity и no-return.

Порядок noise → dropout → intensity менять нельзя без миграции формата.

## `visualization.py`

Ответственность: MarkerArray из готового результата.

- `build_marker_array()` — главный builder.
- geometry marker: CUBE/CYLINDER/MESH_RESOURCE/TRIANGLE_LIST/LINE_STRIP;
- POINTS — только `returned_after_dropout`;
- TEXT — ID/class/type/count;
- LINE_LIST — AABB.

Namespace и ID стабильны. Marker использует header облака и положительный
lifetime.

## `ground_truth.py`

- `annotation_path_for()` — `<output>.annotations.jsonl`.
- `AnnotationWriter` — exclusive UTF-8 writer с `allow_nan=False` и flush после
  каждой строки.

Содержимое record формируется processor, потому что там доступны scenario,
temporal state и frame result.

## `cube_injector.py`

Legacy API одного axis-aligned cube. Сохранён для schema v1 и регрессионной
совместимости. Новые типы объектов следует добавлять через `geometry.py` и
`object_injector.py`, а не расширять legacy-функцию.

## `smoke_test.py` и каталог `test`

`smoke_test.py` создаёт временный SQLite3 bag и проверяет identity roundtrip.
Каталог `test` содержит unit-, geometry-, ROS bag- и воспроизводимые
интеграционные тесты. Реальные bags в тестах не используются.

## Как добавить новую геометрию

1. Добавить dataclass и YAML validation в `scenario.py`.
2. Построить локальные vertices/triangles в `geometry.py`.
3. Добавить RViz-представление в `visualization.py`.
4. Не менять `object_injector.py`, если результат остаётся triangle mesh.
5. Добавить validation, ray-hit, miss, rotation, marker и JSONL-тесты.

## Как добавить новый сенсорный эффект

1. Добавить строгую конфигурацию.
2. Назначить фиксированный числовой effect code.
3. Применять эффект только к соответствующей semantic mask.
4. Расширить `SensorEffectStats` и JSONL.
5. Проверить независимость RNG от остальных эффектов.
6. Обеспечить disabled-режим, бинарно совместимый с предыдущим этапом.
