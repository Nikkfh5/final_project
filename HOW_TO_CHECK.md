# Как проверить запуск кода

## Быстрая проверка без ROS

### 1. Автоматическая проверка (рекомендуется)

Запустите скрипт проверки:

```bash
./check_code.sh
```

Этот скрипт проверит:

- ✓ Синтаксис всех Python файлов
- ✓ Наличие необходимых библиотек (numpy, opencv-python, matplotlib)
- ✓ Наличие всех необходимых файлов конфигурации
- ✓ Права на выполнение скриптов

### 2. Ручная проверка синтаксиса

```bash
# Проверка tracker_node.py
python3 -m py_compile python/tracker_pkg/scripts/tracker_node.py

# Проверка offline_visualizer.py
python3 -m py_compile python/tracker_pkg/scripts/offline_visualizer.py

# Проверка camera_debug.py
python3 -m py_compile python/tracker_pkg/scripts/camera_debug.py
```

Если ошибок нет - команды завершатся без вывода.

### 3. Проверка импортов (без запуска ROS)

```bash
python3 -c "import numpy; import cv2; import matplotlib; print('Все библиотеки установлены')"
```

Если какие-то библиотеки отсутствуют, установите их:

```bash
pip3 install numpy opencv-python matplotlib
```

## Проверка с ROS (полный тест)

### Предварительные требования

1. **Убедитесь, что ROS установлен и настроен:**

   ```bash
   echo $ROS_DISTRO  # Должна быть указана версия ROS (melodic/noetic)
   ```

2. **Убедитесь, что проект собран в catkin workspace:**
   ```bash
   cd ~/catkin_ws  # или ваш путь к workspace
   catkin_make
   source devel/setup.bash
   ```

### Шаг 1: Проверка доступности пакетов

```bash
# Проверка, что пакеты найдены ROS
rospack find tracker_pkg
rospack find sim_pkg
```

### Шаг 2: Проверка структуры launch файлов

```bash
# Проверка синтаксиса launch файлов
roslaunch tracker_pkg tracker.launch --check
roslaunch sim_pkg demo.launch --check
```

### Шаг 3: Проверка конфигурации

```bash
# Проверка конфигурационного файла
rosparam load $(rospack find tracker_pkg)/config/tracker.yaml
rosparam list
```

### Шаг 4: Запуск симуляции (полный тест)

**В первом терминале:**

```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch sim_pkg demo.launch
```

**Во втором терминале - проверка работы:**

```bash
# 1. Проверка топиков (должны появиться через несколько секунд)
rostopic list
# Ожидаемые топики:
# /camera/image_raw
# /camera/camera_info
# /robot_pose_external
# /robot_path_external
# /debug/image

# 2. Проверка частоты публикации изображений
rostopic hz /camera/image_raw
# Должна быть частота ~30 Hz

# 3. Просмотр текущей позы робота
rostopic echo /robot_pose_external -n 1
# Должна вывестись структура PoseStamped

# 4. Просмотр debug изображения (должно открыться окно)
rqt_image_view /debug/image
# Вы должны увидеть изображение с отмеченными красными и синими точками (метками)

# 5. Проверка TF трансформаций
rosrun tf view_frames
# Создаст файл frames.pdf с графом трансформаций
# Должны быть: world -> camera_link
```

### Шаг 5: Проверка логирования траектории

1. Запустите симуляцию и дайте роботу немного поездить (или подвигайте его вручную в Gazebo)
2. Остановите симуляцию (Ctrl+C)
3. Проверьте сохраненную траекторию:

```bash
# Проверка наличия файла
ls -lh ~/tracker_logs/trajectory.csv

# Визуализация траектории
python3 $(rospack find tracker_pkg)/scripts/offline_visualizer.py ~/tracker_logs/trajectory.csv --interactive
```

## Минимальный тест без Gazebo

Если нужно проверить только трекер без симуляции (но с ROS master):

```bash
# Терминал 1: Запуск ROS master
roscore

# Терминал 2: Запуск трекера
source ~/catkin_ws/devel/setup.bash
roslaunch tracker_pkg tracker.launch
```

Трекер должен запуститься, но будет ожидать данные с камеры. Проверьте логи:

```bash
# В логах должно быть:
# "Waiting for camera_info..."
# "Camera matrix not yet initialized, waiting for camera_info..."
```

## Типичные проблемы и решения

### Проблема: "No module named 'rospy'"

**Решение:** Убедитесь, что ROS настроен и вы выполнили `source /opt/ros/<distro>/setup.bash`

### Проблема: "Package 'tracker_pkg' not found"

**Решение:**

1. Проверьте, что пакет скопирован в `~/catkin_ws/src/`
2. Выполните `catkin_make` в workspace
3. Выполните `source ~/catkin_ws/devel/setup.bash`

### Проблема: Скрипты не выполняются

**Решение:**

```bash
chmod +x python/tracker_pkg/scripts/*.py
```

### Проблема: Трекер не находит метки

**Решение:**

1. Проверьте debug изображение: `rqt_image_view /debug/image`
2. Настройте HSV пороги в `config/tracker.yaml`
3. Используйте `camera_debug.py` для просмотра сырого изображения

## Быстрый чек-лист перед запуском

- [ ] Python файлы компилируются без ошибок (`./check_code.sh`)
- [ ] Установлены все зависимости (numpy, opencv-python, matplotlib)
- [ ] ROS установлен и настроен
- [ ] Пакеты скопированы в catkin workspace
- [ ] Проект собран (`catkin_make`)
- [ ] Workspace активирован (`source devel/setup.bash`)
- [ ] Launch файлы проверены (`roslaunch ... --check`)

После прохождения всех пунктов можно запускать полную симуляцию!
