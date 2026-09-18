# Диагностика и типичные ошибки

## Output уже существует

```text
Output path already exists
Annotation path already exists
```

Это защитное поведение. Выберите новый output. Генератор не имеет режима
автоматической перезаписи.

## Не найден PointCloud2-топик

Проверьте:

```bash
ros2 bag info /path/to/input_bag
```

Затем исправьте `source.pointcloud_topic`. Тип обязан быть
`sensor_msgs/msg/PointCloud2`.

## Неверный индекс кадра

Индекс считается только на выбранном топике и начинается с нуля. Если диапазон
частично отсутствует, output считается неуспешным.

## Open3D отсутствует

```text
Open3D is required for synthetic object ray casting
```

Активируйте правильный venv и установите Open3D. Проверка:

```bash
python -c "import open3d; print(open3d.__version__)"
```

## Trimesh отсутствует

Box/cylinder/cable работают без Trimesh. Human mesh требует:

```bash
python -m pip install trimesh
```

## Mesh не загружается

Проверьте:

- путь разрешается относительно YAML;
- файл существует и не является каталогом;
- расширение `.obj` или `.stl`;
- есть vertices и triangular faces;
- координаты конечные;
- scale положительный;
- units и origin поддерживаются.

## Marker topic конфликтует

Marker topic должен быть новым и отличаться от входных топиков. Измените:

```yaml
marker_topic: /synthetic/markers_run_2
```

## RViz ничего не показывает

1. Выполните `ros2 bag info` и убедитесь, что marker topic записан.
2. Установите Fixed Frame равным cloud `header.frame_id`.
3. Проверьте topic PointCloud2 и MarkerArray.
4. Увеличьте point size.
5. Посмотрите JSONL: возможно, visible point count равен нулю.

## Zero-slot recovery требует ring/timestamp

Укажите фактические имена:

```yaml
ring_field: ring
timestamp_field: timestamp
```

Если облако не сохраняет no-return slots или нужные поля отсутствуют, отключите
recovery. Поля автоматически не добавляются.

## `timestamp_unit: auto` отклоняется

Укажите единицы явно. Неверный масштаб может сделать temporal модель физически
ошибочной, поэтому критическое значение не угадывается молча.

## Высокая angular error

Возможные причины:

- ring содержит несколько elevation-каналов;
- timestamp не соответствует порядку азимута;
- единицы или поле выбраны неправильно;
- внутри ring слишком мало валидных points;
- scan имеет нелинейную временную развёртку.

Увеличивать threshold без анализа опасно. Сначала визуализируйте ошибку на
синтетически скрытых валидных лучах.

## Intensity field отсутствует

Выберите:

```yaml
on_missing: error
```

или осознанно:

```yaml
on_missing: skip_with_warning
```

Layout PointCloud2 не расширяется автоматически.

## Trajectory не покрывает timestamp

Экстраполяция запрещена. Проверьте:

- абсолютные `timestamp_ns` CSV;
- `header.stamp` облака;
- reference frame;
- диапазон trajectory;
- `max_interpolation_gap_ms`.

Bag timestamp используется только если header stamp равен нулю.

## Track-relative объект требует trajectory

Нужно задать одновременно `track` и `ego_motion`. Identity fallback намеренно
отсутствует. Если объект должен быть неподвижен относительно LiDAR, используйте
`placement.type: lidar_relative`.

## `s` вне track

Учитывайте движение:

```text
s(t) = s0 + velocity × dt
```

Весь active interval должен оставаться внутри длины полилинии.

## Неожиданный dropout

Проверьте:

- `base_return_probability`;
- distance reference/exponent;
- incidence exponent;
- material `return_probability_scale`;
- JSONL incidence/normal fallback и dropout counts.

## Synthetic hit оказался за фоном

Такой hit не должен изменять облако. Если кажется иначе, проверьте систему
координат, положение origin, RPY и исходный range конкретного слота.

## Частичный output после ошибки

Система его не удаляет. Убедитесь в точном пути и удалите вручную только
артефакт неуспешного запуска. Следующий запуск должен использовать новый путь.

## Сбор сведений для отчёта об ошибке

Приложите:

- команду запуска;
- scenario YAML без секретных путей;
- `ros2 bag info`;
- traceback;
- версии ROS 2/Python/NumPy/Open3D/Trimesh;
- PointField metadata;
- одну JSONL-строку проблемного кадра;
- минимальный synthetic test, если возможно.

Не прикладывайте большие реальные bags без необходимости.
