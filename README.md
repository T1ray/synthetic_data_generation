# Генерация синтетических LiDAR-данных в ROS 2 bag

Пакет реализует первый, намеренно неизменяющий этап конвейера генерации
синтетических лидарных данных. Каждое сообщение из bag проходит полный путь:

```text
сериализованное сообщение из SequentialReader
  -> динамическое определение ROS-типа
  -> deserialize_message
  -> process_message (возвращает сообщение без изменений)
  -> serialize_message
  -> SequentialWriter с исходными топиком и timestamp
```

Обычный roundtrip по-прежнему ничего не изменяет. Дополнительно реализованы:

- layout-сохраняющий NumPy codec для `sensor_msgs/msg/PointCloud2`;
- опциональное внедрение одного Open3D-куба в один выбранный кадр;
- замена только тех исходных LiDAR-возвратов, которые куб физически закрывает.

## Требования к окружению

- Ubuntu 22.04;
- ROS 2 Humble;
- Python 3.10;
- `rosbag2_py` и storage plugin, соответствующий входному bag.
- NumPy;
- Open3D для режима `--inject-cube`.

Текущие bags хакатона используют SQLite3. Другие storage backend сохраняются,
если в окружении установлены соответствующие reader- и writer-плагины.

## Создание виртуального окружения

Перед созданием окружения необходимо подключить ROS. Параметр
`--system-site-packages` обязателен, поскольку Python-модули ROS установлены в
`/opt/ros/humble`.

Следующий вариант работает и в минимальной установке Ubuntu без
`ensurepip`/`python3.10-venv` и не требует скачивания зависимостей:

```bash
source /opt/ros/humble/setup.bash
cd /path/to/HackathonLoDT/synthetic_data_generation
python3 -m venv --without-pip --system-site-packages .venv
source .venv/bin/activate
python setup.py develop --no-deps
```

Если в окружении доступны `python3.10-venv` и `pip`, вместо последней команды
предпочтительно использовать современный способ установки:

```bash
python -m pip install --editable .
python -m pip install -r requirements.txt
```

## Запуск без сборки colcon

```bash
source /opt/ros/humble/setup.bash
source .venv/bin/activate

python -m synthetic_data_generation.bag_roundtrip \
  --input /path/to/input_bag \
  --output /path/to/output_bag
```

Выходной путь обязан быть новым и несуществующим. Существующие каталоги и файлы
никогда автоматически не удаляются и не перезаписываются. Выберите другое имя,
если указанный output уже существует.

## Внедрение одного куба

Куб задаётся в системе координат LiDAR. Индекс кадра начинается с нуля и
считается только среди сообщений выбранного PointCloud2-топика:

```bash
python -m synthetic_data_generation.bag_roundtrip \
  --input /path/to/input_bag \
  --output /path/to/output_bag \
  --pointcloud-topic /lidar_points \
  --target-frame-index 10 \
  --inject-cube \
  --cube-center-x 10.0 \
  --cube-center-y 0.0 \
  --cube-center-z 0.0 \
  --cube-size-x 2.0 \
  --cube-size-y 2.0 \
  --cube-size-z 2.0
```

Начало каждого луча принимается равным `(0, 0, 0)`. Обрабатываются только
конечные ненулевые точки. Направление луча нормализуется, а точка заменяется,
только если первая поверхность куба находится ближе исходного фона:

```text
epsilon < t_hit < original_range - epsilon
```

Меняются только `x`, `y`, `z`. Число точек, дополнительные поля, point padding,
row padding, `header.stamp` и timestamp записи bag сохраняются.

## NumPy codec PointCloud2

`pointcloud_codec.py` создаёт изменяемый structured NumPy view поверх полной
копии `PointCloud2.data`. Dtype строится штатной функцией
`sensor_msgs_py.point_cloud2.dtype_from_fields()`, а адресация учитывает
`row_step` и `point_step` через NumPy strides.

Identity-проверка сравнивает именно бинарное поле облака:

```python
assert bytes(encoded_message.data) == bytes(original_message.data)
```

Это отличается от побайтового сравнения всего CDR-представления bag, которое
не является контрактом roundtrip.

## Сборка и запуск как ROS 2-пакета

Для этого варианта дополнительно требуется `colcon`, обычно устанавливаемый
пакетом `python3-colcon-common-extensions`.

Из корня репозитория выполните:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --base-paths synthetic_data_generation \
  --packages-select synthetic_data_generation
source install/setup.bash

ros2 run synthetic_data_generation bag_roundtrip \
  --input /path/to/input_bag \
  --output /path/to/output_bag
```

Если путь к проекту содержит пробелы, рекомендуется запуск через Python-модуль:
некоторые shebang-строки генерируемых консольных скриптов не поддерживают такие
пути.

## Локальная smoke-проверка

Smoke-проверка не использует данные хакатона. Она программно создаёт временный
SQLite3 bag с чередующимися сообщениями `std_msgs/msg/String` и
`sensor_msgs/msg/PointCloud2`, выполняет roundtrip, читает оба bag и проверяет:

- имена топиков и типы сообщений;
- порядок сообщений и bag timestamps;
- количество сообщений;
- семантическое равенство после десериализации;
- все поля PointCloud2, включая header, fields, размеры, шаги, порядок байтов,
  признак плотности и бинарные данные.
- сохранение point padding и row padding NumPy codec;
- безопасность уже существующего output-каталога;
- геометрию куба, окклюзию, невалидные точки и выбор единственного кадра при
  наличии Open3D.

Запуск smoke-проверки:

```bash
source /opt/ros/humble/setup.bash
source .venv/bin/activate
python -m synthetic_data_generation.smoke_test
```

Запуск тестов:

```bash
python -m pytest -q
```

Временные bags создаются через `tempfile.TemporaryDirectory` и удаляются после
освобождения всех reader-объектов.

## Известные ограничения

- Поддерживаются только ROS-сообщения с CDR-сериализацией.
- Тип каждого сообщения должен быть установлен и доступен в подключённом
  ROS-окружении.
- Storage backend входного bag сохраняется только при наличии соответствующего
  writer-плагина.
- Критерием является семантическая эквивалентность. Побайтовое совпадение
  повторно сериализованного CDR не предполагается.
- Big-endian PointCloud2 пока намеренно отклоняется.
- Восстанавливаются только существующие валидные лучи; новые LiDAR-слоты не
  создаются.
- Поддерживается только один axis-aligned куб в одном кадре, без шума, dropout
  и изменения intensity.
- Реальные bags хакатона на этом этапе не использовались.
