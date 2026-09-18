# Настройка и использование RViz2

## Что записывает генератор

При `visualization.enabled: true` в bag добавляется:

```text
/synthetic/markers
visualization_msgs/msg/MarkerArray
```

На каждый обработанный кадр создаётся один MarkerArray с теми же:

- bag timestamp;
- `header.stamp`;
- `header.frame_id`.

## Быстрый запуск

Терминал 1:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 bag play /path/to/generated_bag
```

Терминал 2:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch synthetic_data_generation preview.launch.py
```

Готовая конфигурация: `rviz/synthetic_lidar.rviz`.

## Fixed Frame

Установите RViz `Global Options → Fixed Frame` равным
`PointCloud2.header.frame_id`, например:

```text
synthetic_lidar
pandar
lidar_link
```

Система не создаёт TF. Если Fixed Frame отличается и TF-связи нет, RViz будет
показывать transform error.

## PointCloud2 display

Добавьте `PointCloud2`:

- Topic — ваш `source.pointcloud_topic`;
- Style — `Points`;
- Size — около `0.02–0.06 m`;
- Color Transformer — `FlatColor` либо фактическое поле облака;
- Queue Size — 2–10.

Если scenario использует `/lidar_points`, а bag публикует другой topic,
измените topic в RViz вручную.

## MarkerArray display

Добавьте `MarkerArray`:

```text
Topic: /synthetic/markers
```

Namespace объектов:

```text
synthetic/<object_id>/geometry
synthetic/<object_id>/points
synthetic/<object_id>/text
synthetic/<object_id>/bbox
```

ID стабильны:

```text
geometry 0
points   1
text     2
bbox     3
```

## Что означают маркеры

- Geometry — истинная pose объекта текущего кадра.
- Points — только synthetic returns, пережившие dropout.
- Text — object ID, class, geometry type и число точек.
- Bounding box — axis-aligned bounds в LiDAR frame.

Dropped hits не показываются как существующие точки. Возвраты из восстановленных
zero slots включаются в Points победившего объекта.

## Цвета

Цвет задаётся для каждого объекта:

```yaml
visualization:
  color_rgba: [1.0, 0.2, 0.1, 0.9]
```

Он не вычисляется через Python hash, поэтому стабилен между запусками.

## Lifetime

```yaml
marker_lifetime_sec: 0.25
```

Lifetime должен быть положительным. Это удаляет маркер, если объект стал
неактивным или воспроизведение закончилось. Подберите значение немного больше
периода LiDAR-кадра.

Примеры:

```text
10 Hz → 0.15–0.25 s
20 Hz → 0.08–0.15 s
```

## Human mesh

Предпочтительный URI:

```yaml
mesh_resource_uri: package://synthetic_data_generation/meshes/humans/person.obj
```

После изменения mesh выполните colcon build и source overlay. Если URI не
задан, генератор использует TRIANGLE_LIST, а для слишком большого mesh — AABB.

## Cable

Cable отображается как LINE_STRIP. Толщина равна диаметру `2 × radius_m`.
Ray casting при этом использует настоящую треугольную трубчатую поверхность.

## Визуализация отключена

```yaml
visualization:
  enabled: false
```

В этом режиме marker topic не регистрируется и дополнительные сообщения не
пишутся. PointCloud2 и JSONL остаются такими же.

## Диагностика

### MarkerArray отсутствует

- проверьте `visualization.enabled`;
- проверьте, что кадр входит в глобальный диапазон;
- выполните `ros2 bag info`;
- проверьте имя marker topic.

### Объект есть, цветных точек нет

Это допустимо: объект может не пересекать существующие лучи, находиться за
реальным фоном или потерять returns из-за dropout. Смотрите JSONL-счётчики.

### Marker зависает

Проверьте положительный lifetime и частоту кадров.

### Transform error

Fixed Frame не совпадает с frame облака и TF отсутствует. Выберите сам LiDAR
frame.

## Ограничение проверки

Наличие `.rviz` и корректность ROS-сообщений проверяется тестами, но финальную
цветовую композицию необходимо визуально проверить в вашей графической среде.
