# Инструкция по установке и запуску проекта

## Требования

- ROS (Melodic или Noetic)
- Gazebo
- Python 2.7 или 3.x (в зависимости от версии ROS)
- Установленные пакеты:
  - `cv_bridge`
  - `tf2_ros`
  - `gazebo_ros`
  - `gazebo_plugins`
  - Python библиотеки: `opencv-python`, `numpy`, `matplotlib`

## Установка зависимостей

### ROS пакеты

```bash
sudo apt-get install ros-<distro>-cv-bridge \
                     ros-<distro>-tf2-ros \
                     ros-<distro>-gazebo-ros \
                     ros-<distro>-gazebo-plugins
```

Где `<distro>` - ваша версия ROS (melodic/noetic).

### Python библиотеки

```bash
pip install opencv-python numpy matplotlib
```

Или для Python 3:

```bash
pip3 install opencv-python numpy matplotlib
```

## Сборка проекта

1. Перейдите в корень catkin workspace:

```bash
cd ~/catkin_ws  # или ваш путь к workspace
```

2. Скопируйте пакеты в `src/`:

```bash
cp -r /path/to/ysl/gazebo/sim_pkg src/
cp -r /path/to/ysl/python/tracker_pkg src/
```

3. Соберите проект:

```bash
catkin_make
# или
catkin build
```

4. Источте workspace:

```bash
source devel/setup.bash
```

## Запуск проекта

### Вариант 1: Полный запуск (симуляция + трекер + rviz)

```bash
roslaunch sim_pkg demo.launch
```

Этот launch файл запустит:

- Gazebo симуляцию с роботом и камерой
- Трекер ноду для обработки изображений
- RViz для визуализации

### Вариант 2: Пошаговый запуск

1. Запустите симуляцию:

```bash
roslaunch sim_pkg sim.launch
```

2. В другом терминале запустите трекер:

```bash
roslaunch tracker_pkg tracker.launch
```

3. (Опционально) Запустите RViz:

```bash
rosrun rviz rviz -d $(rospack find tracker_pkg)/rviz/tracker.rviz
```

## Проверка работы

### Проверка топиков

```bash
rostopic list
```

Должны появиться:

- `/camera/image_raw`
- `/camera/camera_info`
- `/robot_pose_external`
- `/robot_path_external`
- `/debug/image`

### Просмотр текущей позы робота

```bash
rostopic echo /robot_pose_external
```

### Просмотр изображения с камеры

```bash
rqt_image_view /camera/image_raw
```

### Просмотр debug изображения с отмеченными метками

```bash
rqt_image_view /debug/image
```

## Оффлайн визуализация траектории

После завершения работы трекера, траектория сохраняется в файл (по умолчанию `~/tracker_logs/trajectory.csv`).

Для визуализации:

```bash
rosrun tracker_pkg offline_visualizer.py ~/tracker_logs/trajectory.csv --interactive
```

Или напрямую:

```bash
python $(rospack find tracker_pkg)/scripts/offline_visualizer.py ~/tracker_logs/trajectory.csv --interactive
```

Опции:

- `--format {csv,json,auto}` - формат файла
- `--interactive` - включить интерактивный режим с ползунком времени

## Настройка параметров

Параметры трекера настраиваются в файле:

```
python/tracker_pkg/config/tracker.yaml
```

Основные параметры:

- HSV пороги для красной и синей меток
- Минимальная/максимальная площадь контуров
- Расстояние между метками
- Параметры сглаживания позы
- Настройки логирования траектории

## Устранение проблем

### Камера не публикует изображения

Проверьте, что Gazebo запущен и камера видна в симуляции:

```bash
rostopic hz /camera/image_raw
```

### Метки не детектируются

1. Проверьте debug изображение: `rqt_image_view /debug/image`
2. Настройте HSV пороги в `tracker.yaml`
3. Используйте `camera_debug.py` для просмотра сырого изображения:

```bash
rosrun tracker_pkg camera_debug.py
```

### TF transform не найден

Проверьте статический transform:

```bash
rosrun tf view_frames
```

Должен быть transform `world` -> `camera_link`.

### Траектория не сохраняется

Проверьте параметр `log_trajectory: true` в `tracker.yaml` и убедитесь, что путь для сохранения доступен для записи.
