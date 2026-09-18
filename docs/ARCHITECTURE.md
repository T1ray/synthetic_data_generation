# Архитектура генератора синтетических LiDAR-данных

## Назначение системы

Система читает исходный ROS 2 bag, сохраняет все его сообщения и добавляет
синтетические препятствия в выбранные кадры `sensor_msgs/msg/PointCloud2`.
Результатом одного запуска являются:

1. Новый ROS 2 bag с изменёнными облаками и, опционально, MarkerArray.
2. JSONL ground truth с фактическими масками, pose и статистикой объектов.
3. Консольная сводка количества сообщений, лучей, изменений и времени стадий.

Исходный bag никогда не изменяется. Существующие output-каталоги и JSONL-файлы
не перезаписываются и не удаляются автоматически.

## Архитектурные слои

```text
┌──────────────────────────────────────────────────────────────┐
│ CLI и безопасный ROS 2 bag roundtrip                         │
│ bag_roundtrip.py                                             │
└───────────────────────────────┬──────────────────────────────┘
                                │ ROS messages
┌───────────────────────────────▼──────────────────────────────┐
│ Сценарий и координация кадра                                 │
│ scenario.py + processor.py                                   │
└───────────┬───────────────────┬───────────────────┬──────────┘
            │                   │                   │
┌───────────▼─────────┐ ┌───────▼─────────┐ ┌──────▼──────────┐
│ Время и pose        │ │ PointCloud2     │ │ Геометрии       │
│ temporal.py         │ │ codec/rays      │ │ geometry.py     │
└───────────┬─────────┘ └───────┬─────────┘ └──────┬──────────┘
            └───────────────────┼───────────────────┘
                                ▼
                    ┌──────────────────────┐
                    │ Open3D ray casting   │
                    │ object_injector.py   │
                    └──────────┬───────────┘
                               ▼
                    ┌──────────────────────┐
                    │ Сенсорная модель     │
                    │ sensor_model.py      │
                    └───────┬───────┬──────┘
                            │       │
              ┌─────────────▼─┐   ┌─▼────────────────┐
              │ JSONL         │   │ RViz MarkerArray │
              │ ground_truth  │   │ visualization.py │
              └───────────────┘   └──────────────────┘
```

## Поток одного сообщения

`roundtrip_bag()` последовательно читает сообщения через
`rosbag2_py.SequentialReader`. Для каждого сообщения:

1. Определяется ROS-тип по metadata входного bag.
2. Выполняется `deserialize_message()`.
3. Сообщение передаётся в `ProcessingContext.process_message()`.
4. Результат сериализуется через `serialize_message()`.
5. Сообщение записывается с исходным topic и bag timestamp.
6. Если сформирован MarkerArray, он записывается сразу после облака с тем же
   bag timestamp.

Сообщения других топиков проходят через систему без прикладных изменений.

## Поток одного PointCloud2-кадра

Порядок стадий является частью контракта:

```text
PointCloud2 decode
→ valid/zero/invalid classification
→ zero-slot direction recovery
→ frame timestamp selection
→ object poses in LiDAR frame
→ geometry transforms
→ one Open3D RaycastingScene
→ nearest synthetic hit and object ownership
→ real-background occlusion test
→ ideal synthetic return
→ range noise
→ dropout/no-return
→ synthetic intensity
→ write XYZ/intensity into original slots
→ PointCloud2 encode
→ MarkerArray
→ JSONL
```

Повторно вычислять маски отдельно для cloud, JSONL или MarkerArray запрещено.
Все потребители используют единый `FrameInjectionResult`.

## Основные инварианты

### Безопасность файлов

- input и output должны различаться;
- output не может находиться внутри input;
- существующий output отклоняется;
- существующий JSONL отклоняется;
- автоматический `rmtree` и overwrite не используются;
- при аварии частичный новый output может остаться для диагностики.

### Сохранение PointCloud2

- количество слотов не меняется;
- `height`, `width`, `point_step`, `row_step` сохраняются;
- point padding и row padding сохраняются;
- поля, не участвующие в эффекте, сохраняются;
- ring и timestamp точки не меняются;
- при отключённой обработке бинарное поле `data` сохраняется.

### Окклюзия

Все активные объекты кадра добавляются в одну Open3D-сцену. Open3D выбирает
ближайший synthetic hit. Затем выполняется проверка реального фона:

```text
epsilon < synthetic_range < original_background_range - epsilon
```

Для восстановленного нулевого слота background range равен бесконечности.

### Детерминизм

RNG не является глобальным. Поток определяется сценарием, кадром, стабильным
SHA-256-ключом объекта и кодом эффекта. Добавление вызова intensity не меняет
dropout, а один объект не сдвигает последовательность другого.

## Версии сценария

- Schema v1: один неповёрнутый box, старый контракт центра объекта.
- Schema v2: несколько геометрий, повороты и MarkerArray.
- Schema v3: zero-slot recovery, сенсорная модель, trajectory и track-relative
  temporal placement.

Поддержка v1/v2 сохранена для регрессии. Новая функциональность не включается
неявно.

## Система координат

В LiDAR frame используется соглашение:

```text
X — вперёд
Y — влево
Z — вверх
```

Матрица `T_A_B` преобразует координаты из B в A:

```text
p_A = T_A_B @ p_B
```

Для track-relative объекта:

```text
T_lidar_object = inverse(T_reference_lidar) @ T_reference_object
```

Вращение RPY:

```text
R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
```

## Временная модель

Один кадр использует один reference timestamp:

1. `PointCloud2.header.stamp`, если он ненулевой;
2. иначе bag timestamp.

Timestamp отдельных точек применяется только для восстановления направления
внутри скана. Полная deskew и intra-scan motion compensation не реализованы.

## Выходные артефакты

### ROS 2 bag

Содержит все исходные сообщения и дополнительные MarkerArray при включённой
визуализации.

### JSONL

Одна строка на обработанный кадр. Содержит фактические счётчики, pose, bounds,
сенсорные эффекты, причины отказов восстановления и статистику объектов.

### MarkerArray

Использует тот же `FrameInjectionResult`. POINTS показывает только реальные
synthetic returns после dropout. Dropped hits не визуализируются как точки.

## Границы ответственности

Система не выполняет:

- автоматическое распознавание рельсов;
- обучение моделей;
- получение trajectory из отсутствующих данных;
- скрытую подстановку identity pose для track-relative режима;
- скачивание mesh из сети;
- проверку лицензий сторонних production mesh;
- TF-интеграцию и intra-scan deskew в текущей версии.
