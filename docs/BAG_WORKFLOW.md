# Работа с исходными и выходными ROS 2 bags

## Перед запуском

Никогда не используйте исходный bag как output. Подготовьте:

1. Путь к существующему bag-каталогу.
2. Новый несуществующий output-путь.
3. Имя PointCloud2-топика.
4. `header.frame_id` облака.
5. Scenario YAML подходящей версии.

Просмотр metadata без воспроизведения:

```bash
ros2 bag info /path/to/input_bag
```

Просмотр доступных топиков при воспроизведении:

```bash
ros2 bag play /path/to/input_bag
ros2 topic list
ros2 topic echo /lidar_points --once
```

Для большого облака используйте `ros2 topic info` и собственный небольшой
инспектор вместо полного `echo`.

## Identity roundtrip

Первый безопасный тест нового bag:

```bash
python -m synthetic_data_generation.bag_roundtrip \
  --input /path/to/input_bag \
  --output /path/to/identity_output
```

При таком запуске `ProcessingContext` отсутствует, MarkerArray и JSONL не
создаются, а сообщения проходят deserialize/serialize без прикладной обработки.

## Генерация по сценарию

```bash
python -m synthetic_data_generation.bag_roundtrip \
  --input /path/to/input_bag \
  --output /path/to/generated_output \
  --scenario config/scenarios/mixed_objects_lidar.yaml \
  --progress-every 100
```

Для schema v3:

```bash
python -m synthetic_data_generation.bag_roundtrip \
  --input /path/to/input_bag \
  --output /path/to/generated_sensor_output \
  --scenario config/scenarios/track_sensor_model.yaml
```

## Что считается кадром

Индекс начинается с нуля и увеличивается только для сообщений выбранного
PointCloud2-топика. TF, камера, IMU и другие сообщения не влияют на индекс.

```text
/points → frame 0
/imu
/tf
/points → frame 1
```

`start_index` и `end_index` включительны. Если часть диапазона отсутствует,
программа завершится ошибкой.

## Выходные файлы

Для:

```text
--output /data/run_generated
```

создаются:

```text
/data/run_generated/                    ROS 2 bag
/data/run_generated.annotations.jsonl  ground truth
```

JSONL появляется только в scenario-режиме. MarkerArray хранится внутри bag.

## Сообщения выходного bag

При visualization disabled:

```text
output_message_count == input_message_count
```

При visualization enabled:

```text
output_message_count = input_message_count + processed_frame_count
```

MarkerArray записывается сразу после соответствующего PointCloud2 с тем же bag
timestamp.

## Проверка результата

```bash
ros2 bag info /path/to/generated_output
```

Проверьте:

- все исходные topics присутствуют;
- storage backend ожидаемый;
- количество PointCloud2 не изменилось;
- MarkerArray count равен числу обработанных кадров;
- duration и временной диапазон не потеряны.

Проверка JSONL:

```bash
python - <<'PY'
import json
from pathlib import Path

path = Path('/path/to/generated_output.annotations.jsonl')
for line_number, line in enumerate(path.open(encoding='utf-8'), 1):
    record = json.loads(line)
    print(line_number, record['frame_index'], record['modified_slot_count'])
PY
```

## Воспроизведение

```bash
ros2 bag play /path/to/generated_output
```

В другом терминале:

```bash
ros2 launch synthetic_data_generation preview.launch.py
```

## Безопасный повторный запуск

Нельзя повторно использовать существующий output. Выберите новое имя:

```text
run_generated_001
run_generated_002
```

Система намеренно не предоставляет `--overwrite`.

## Ошибка в середине обработки

Если ошибка произошла после создания writer, output может быть неполным.
Программа не удаляет его автоматически. Действия оператора:

1. Прочитать исходную ошибку.
2. Убедиться, что путь действительно относится к неудачному запуску.
3. Удалить или архивировать его вручную.
4. Запустить с новым output-именем.

## Проверка PointCloud2 layout

Входной codec ожидает:

- scalar `x`, `y`, `z` типа FLOAT32/FLOAT64;
- little-endian layout;
- корректные `point_step`, `row_step` и длину `data`.

Zero-slot recovery дополнительно требует scalar numeric ring/timestamp. Intensity
effect требует настроенное intensity-поле или `skip_with_warning`.

## Legacy CLI

Однокадровый cube CLI сохранён только для совместимости и диагностики. Его
нельзя смешивать с `--scenario`. Для новых наборов используйте YAML.
