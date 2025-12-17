# Tracker Package (tracker_pkg)

Этот пакет содержит Python-алгоритм для трекинга робота по внешней камере с использованием цветных меток.

## Что делает Python team

1. **Подписывается на топики камеры**:

   - `/camera/image_raw` (sensor_msgs/Image)
   - `/camera/camera_info` (sensor_msgs/CameraInfo)

2. **Детектирует цветные метки** (красная и синяя) используя OpenCV:

   - Конвертация в HSV цветовое пространство
   - Пороговая фильтрация
   - Морфологические операции
   - Поиск центроидов контуров

3. **Вычисляет мировые координаты**:

   - Получает позу камеры через TF (world -> camera_link)
   - Использует camera intrinsics для unproject пикселей
   - Пересекает лучи с плоскостью пола (z = ground_z)

4. **Вычисляет позу робота**:

   - Центр = середина между метками
   - Yaw = atan2(tail - head)

5. **Публикует результаты**:
   - `/robot_pose_external` (geometry_msgs/PoseStamped) - текущая поза
   - `/robot_path_external` (nav_msgs/Path) - траектория
   - `/debug/image` (sensor_msgs/Image) - изображение с отрисованными метками
   - (опционально) сохраняет траекторию в CSV/JSON на диск при завершении ноды

## Как запускать

### С живой симуляцией

1. **Запустить симуляцию** (в отдельном терминале):

   ```bash
   roslaunch sim_pkg sim.launch
   ```

2. **Запустить трекер**:
   ```bash
   roslaunch tracker_pkg tracker.launch
   ```

3. **Проверить координаты в реальном времени**:
   ```bash
   rostopic echo /robot_pose_external -n5
   rostopic echo /robot_path_external -n3
   rqt_image_view /debug/image  # картинка с метками
   ```

4. **Запустить движение по окружности (если не используете demo.launch)**:
   ```bash
   rosrun sim_pkg cmd_vel_circle.py radius:=0.6 angular_speed:=0.35
   ```

Или использовать единый demo launch:

```bash
roslaunch sim_pkg demo.launch
```

### Быстрый запуск для любого пользователя (в корне репозитория)

```bash
./run_demo.sh
```

Скрипт сам:
- создаёт `tracker_logs` рядом с проектом;
- гасит старые `gzserver/gzclient/roscore`;
- загружает `catkin_ws/devel/setup.bash`;
- запускает `roslaunch sim_pkg demo.launch` с включённой записью.

Где искать результаты:
- JSON: `tracker_logs/trajectory.json`
- PNG:  `tracker_logs/trajectory.png`

Остановить запись вручную (если нужно раньше времени):
```bash
cd /path/to/your/clone/final_project/catkin_ws
source devel/setup.bash
rosservice call /tracker_node/stop_logging
```

Если используете пользовательский world (лежит в `sim_pkg/worlds/custom/`), передайте его в demo.launch — он попадёт и в симулятор, и в трекер:

```bash
roslaunch sim_pkg demo.launch \
  world_file:=$(find sim_pkg)/worlds/custom/my_world.world \
  publish_static_camera_tf:=false \
  spawn_robot:=false
```

### Проверка работы

1. **Проверить топики**:

   ```bash
   rostopic list
   # Должны появиться:
   # /robot_pose_external
   # /robot_path_external
   # /debug/image
   ```

2. **Посмотреть текущую позу**:

   ```bash
   rostopic echo /robot_pose_external
   ```

3. **Посмотреть траекторию**:

   ```bash
   rostopic echo /robot_path_external
   ```

4. **Визуализация в rviz**:

   ```bash
   rosrun rviz rviz -d $(rospack find tracker_pkg)/rviz/tracker.rviz
   ```

   Или вручную добавить:

   - Path display: `/robot_path_external`
   - Pose display: `/robot_pose_external` (или использовать TF)

5. **Посмотреть debug изображение**:

   ```bash
   rqt_image_view /debug/image
   ```

6. **Сохранить траекторию**:
   - Формат по умолчанию — JSON (`log_format: json`, путь задаётся в `log_file`).
   - Включить запись: `rosservice call /tracker_node/start_logging`
   - Остановить и сохранить (JSON + PNG): `rosservice call /tracker_node/stop_logging`
   - PNG путь задаётся `png_output` (если пусто, берётся имя `log_file` с суффиксом `.png`).

### Работа с rosbag (TODO)

Если нужно протестировать на записи:

1. **Записать данные**:

   ```bash
   rosbag record /camera/image_raw /camera/camera_info /tf /tf_static
   ```

2. **Воспроизвести**:
   ```bash
   rosbag play your_bag.bag --clock
   roslaunch tracker_pkg tracker.launch use_sim_time:=true
   ```

## Конфигурация

Параметры настраиваются в `config/tracker.yaml`:

- **HSV пороги** для красной и синей метки (нужно подобрать экспериментально)
- **Минимальная/максимальная площадь контура** для фильтрации шума
- **Имена топиков** и фреймов
- **Z координата плоскости пола** (по умолчанию 0.0)
- **min/max_marker_distance** — допустимая дистанция между метками для отбрасывания ложных срабатываний
- **enable_pose_smoothing / pose_smoothing_alpha** — сглаживание позы (экспоненциальное)
- **log_trajectory / log_format / log_file** — логирование траектории (csv/json) на shutdown
- **logging_initially_on** — начинать запись сразу при старте ноды
- **png_output / png_dpi** — автоматический PNG при остановке записи
- **Логирование траектории**: `log_trajectory`, `log_format`, `log_file`

## Оффлайн визуализация

Для просмотра сохраненной траектории:

```bash
python $(rospack find tracker_pkg)/scripts/offline_visualizer.py trajectory.csv
```

Формат CSV: `t, x, y, theta` (один ряд на одну точку)

Опции:

- `--format {csv,json,auto}` — выбрать формат или дать автодетект по расширению;
- `--interactive` — добавить слайдер по времени и клик по точкам.

## Проверка, что всё работает

- В `/debug/image` метки подписаны как FOUND, линия между ними есть.
- `rostopic echo /robot_pose_external` выдаёт координаты (x,y,z, quaternion).
- `rostopic echo /robot_path_external` выдаёт массив поз траектории.
- Файл траектории сохраняется при остановке трекера: по умолчанию `~/tracker_logs/trajectory.csv`.

## Алгоритм преобразования координат

1. **Unproject пикселя в луч камеры**:

   - Используем camera intrinsics (K матрица)
   - Получаем направление луча в системе координат камеры

2. **Преобразование луча в мировую систему**:

   - Получаем transform world -> camera_link через TF
   - Поворачиваем направление луча используя rotation из transform
   - Получаем позицию камеры в мире из translation

3. **Пересечение с плоскостью пола**:
   - Решаем уравнение: camera_z + t \* ray_dir_z = ground_z
   - Получаем точку пересечения в мировой системе координат

## TODO для Python team

- [ ] Подобрать оптимальные HSV пороги для меток
- [ ] Настроить фильтры морфологии для стабильной детекции
- [x] Добавить проверку на минимальное расстояние между метками
- [x] Реализовать интерполяцию при пропадании меток
- [x] Добавить экспорт траектории в CSV/JSON
- [x] Улучшить offline visualizer (интерактивный ползунок, клики)
- [x] Добавить обработку каждого N-го кадра (frame skipping)
- [x] Создать скрипт для обработки последовательности изображений

## Новые возможности

### Обработка каждого N-го кадра

Параметр `frame_skip` в `tracker.yaml` позволяет обрабатывать не все кадры, а только каждый N-й. Это полезно для снижения нагрузки на процессор или при работе с высокочастотными камерами.

### Интерполяция при пропадании меток

При временной потере меток (например, из-за заслонения) система автоматически использует последнюю известную позу для интерполяции. Максимальное количество кадров для интерполяции настраивается через `max_lost_frames`.

### Обработка последовательности изображений

Скрипт `process_image_sequence.py` позволяет обрабатывать папку с изображениями оффлайн:

```bash
rosrun tracker_pkg process_image_sequence.py \
    /path/to/images \
    output_trajectory.csv \
    --camera-matrix "500,0,320,0,500,240,0,0,1" \
    --camera-pose "0,0,2,0,0.524,0" \
    --format csv
```
