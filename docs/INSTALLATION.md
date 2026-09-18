# Установка и подготовка окружения

## Поддерживаемая среда

- Ubuntu 22.04;
- ROS 2 Humble;
- Python 3.10;
- rosbag2 storage plugin входного bag;
- NumPy, PyYAML, Open3D и Trimesh;
- `sensor_msgs_py`, `visualization_msgs`, `geometry_msgs`, `std_msgs`;
- RViz2 и ROS 2 launch для визуального просмотра.

Windows используется как host, но ROS-команды проекта выполняются в WSL
Ubuntu. Не смешивайте Windows Python с `/opt/ros/humble`.

## Проверка ROS 2

```bash
source /opt/ros/humble/setup.bash
ros2 --help
python3 -c "import rclpy, rosbag2_py, sensor_msgs_py"
```

Если импорт не работает, сначала исправьте установку ROS 2. Виртуальное
окружение не заменяет ROS underlay.

## Вариант 1: editable virtual environment

```bash
source /opt/ros/humble/setup.bash
cd /path/to/HackathonLoDT/synthetic_data_generation

python3 -m venv --system-site-packages .venv
source .venv/bin/activate

python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python -m pip install --editable .
```

`--system-site-packages` нужен, чтобы venv видел Python-модули ROS из
`/opt/ros/humble`.

В минимальном окружении без pip:

```bash
python3 -m venv --without-pip --system-site-packages .venv
source .venv/bin/activate
python setup.py develop --no-deps
```

При этом Open3D и Trimesh должны быть установлены отдельно.

## Вариант 2: colcon

Из корня репозитория:

```bash
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --base-paths synthetic_data_generation \
  --packages-select synthetic_data_generation

source install/setup.bash
```

После сборки доступны:

```bash
ros2 run synthetic_data_generation bag_roundtrip --help
ros2 launch synthetic_data_generation preview.launch.py
```

## ROS-зависимости

Зависимости перечислены в `package.xml`. Если используется rosdep:

```bash
source /opt/ros/humble/setup.bash
rosdep update
rosdep install \
  --from-paths synthetic_data_generation \
  --ignore-src \
  -y
```

Open3D и Trimesh могут потребовать pip, если соответствующего apt/rosdep пакета
нет в вашей установке.

## Проверка Python-зависимостей

```bash
python - <<'PY'
import numpy
import yaml
import open3d
import trimesh
import rosbag2_py
from visualization_msgs.msg import MarkerArray
print("environment is ready")
PY
```

Если Trimesh отсутствует, box/cylinder/cable продолжат работать, но
`human_mesh` завершится понятной ошибкой.

## Проверка package assets

После colcon build:

```bash
ros2 pkg prefix synthetic_data_generation
```

В share-каталоге пакета должны присутствовать:

- `config/scenarios/*.yaml`;
- `launch/preview.launch.py`;
- `rviz/synthetic_lidar.rviz`;
- `meshes/humans/*`;
- `trajectories/*.csv`.

Это необходимо для переносимых `package://` mesh URI.

## Первый тест

```bash
source /opt/ros/humble/setup.bash
source .venv/bin/activate
python -m pytest -q
```

Тесты используют только синтетические временные bags.

## Типичные ошибки установки

### `ModuleNotFoundError: rclpy`

ROS underlay не подключён или venv создан без `--system-site-packages`.

### `Open3D is required`

Установите Open3D именно в Python-окружение, из которого запускается генератор.

### `Trimesh is required for human_mesh`

Установите `trimesh` либо временно исключите human mesh из сценария.

### `visualization_msgs is required`

Установите ROS-пакет visualization messages или отключите visualization.

### Пути с пробелами

При запуске всегда заключайте путь в кавычки. Python module launch часто
надёжнее сгенерированного console script для workspace на Windows-диске.
