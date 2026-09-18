# Тестирование и проверка результата

## Быстрый прогон

```bash
source /opt/ros/humble/setup.bash
source .venv/bin/activate
python -m pytest -q
```

Тесты создают временные synthetic PointCloud2 и SQLite3 bags. Реальные bags не
требуются.

## Группы тестов

### Roundtrip и безопасность

- identity deserialize/serialize;
- исходные topic/order/timestamps;
- запрет существующего output;
- запрет существующего JSONL;
- сохранение других топиков.

### PointCloud2 codec

- byte equality;
- point и row padding;
- organized layout и strides;
- дополнительные поля;
- ошибки endian, размера и datatype XYZ.

### Геометрия и ray casting

- box/cylinder/cable hit и miss;
- повороты;
- взаимная окклюзия;
- реальный фон перед synthetic object;
- geometry ID → object ID;
- human mesh при доступном Trimesh.

### MarkerArray

- topic type;
- frame ID и stamp;
- bag timestamp;
- stable namespace/ID;
- положительный lifetime;
- POINTS содержит только возвращённые synthetic points;
- enabled/disabled эквивалентность cloud/JSONL.

### Sensor model

- disabled и sigma=0;
- воспроизводимый range noise;
- направление не меняется;
- hit не уходит за фон;
- dropout probability 0/1;
- zero XYZ encoding;
- ring/timestamp сохраняются;
- intensity только для returned hits;
- независимость dropout от intensity.

### Zero slots

- valid/zero/invalid классификация;
- azimuth unwrap через `−π/+π`;
- angular error;
- запрет экстраполяции;
- недостаточное число samples;
- отсутствующие поля;
- synthetic return в восстановленный slot.

### Temporal model

- `T_A_B` convention;
- trajectory translation и SLERP;
- запрет экстраполяции и больших gaps;
- arc-length polyline;
- ортонормальный track frame;
- static и constant velocity;
- active interval.

### Полный v3 integration

Проверяется временный bag с несколькими кадрами, zero slot, ring/timestamp,
trajectory, track-relative object, sensor effects, MarkerArray и JSONL. Два
запуска сравниваются на воспроизводимость.

## Запуск отдельных тестов

```bash
python -m pytest -q test/test_pointcloud_codec.py
python -m pytest -q test/test_object_injector_v2.py
python -m pytest -q test/test_sensor_model.py
python -m pytest -q test/test_zero_slot_recovery.py
python -m pytest -q test/test_temporal.py
python -m pytest -q test/test_v3_integration.py
```

## Подробный вывод и причины skip

```bash
python -m pytest -q -rs
```

Skip human-mesh тестов при отсутствующем Trimesh не означает, что остальные
геометрии не проверены. Open3D-тесты запускаются отдельно.

## Smoke test

```bash
python -m synthetic_data_generation.smoke_test
```

Он создаёт временный bag и проверяет семантическое равенство identity roundtrip.

## Проверка нового изменения

Минимальная последовательность:

1. Unit-тест изменяемого модуля.
2. Регрессионный тест disabled-режима.
3. Проверка JSONL-счётчиков.
4. Проверка PointCloud2 layout.
5. Проверка MarkerArray, если меняется semantic mask.
6. Полный `pytest -q`.

## Проверка реального bag перед использованием датасета

Рекомендуется начать с копии или read-only input:

1. `ros2 bag info`.
2. Identity roundtrip небольшого bag.
3. Сценарий на одном кадре и одном объекте.
4. Проверка JSONL.
5. Визуальная проверка RViz.
6. Небольшой диапазон с effects disabled.
7. Последовательное включение zero recovery, noise, dropout, intensity.
8. Только затем полный диапазон.

## Проверка воспроизводимости

Запустите один input/scenario дважды с разными output-путями и сравните:

- `PointCloud2.data` после десериализации;
- JSONL bytes;
- semantic содержимое MarkerArray;
- число сообщений и статистику.

Не сравнивайте SQLite-файлы bag побайтово: служебное представление базы не
является контрактом.

## Производительность

Processor выводит отдельно:

- pose/geometry update time;
- ray/recovery/sensor time;
- MarkerArray build time;
- полное elapsed roundtrip.

Первый Open3D-вызов включает прогрев и не должен использоваться как steady-state
benchmark. Для сравнения измеряйте несколько одинаковых запусков.

## Что тесты не заменяют

- визуальную проверку RViz;
- проверку фактической схемы лучей конкретного LiDAR;
- подтверждение единиц point timestamp;
- валидацию trajectory и extrinsic calibration;
- проверку лицензии production mesh;
- полевой тест perception pipeline.
