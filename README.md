# Final Project — External Camera Tracker (ROS + Offline)

Полный стек для трекинга робота по внешней камере: симуляция в Gazebo/ROS, Python‑нода трекера, офлайн обработка последовательностей и контейнер. Ниже — минимальные команды запуска, проверки и работы с данными.

## Быстрый старт (онлайн, ROS)
```bash
cd ~/Documents/final_project/catkin_ws
catkin_make
source devel/setup.bash
roslaunch sim_pkg demo.launch          # симуляция + трекер + RViz, логирование включено
```
Управление логированием в другом терминале (после `source devel/setup.bash`):
```bash
rosservice call /tracker_node/start_logging
rosservice call /tracker_node/stop_logging
```
Результаты: `tracker_logs/trajectory.json` и `tracker_logs/trajectory.png` (пути задаются в `log_file`/`png_output` или через launch).

## Быстрый старт (офлайн, без ROS)
1) Подготовьте кадры в `catkin_ws/samples/` (см. раздел «Нарезка кадров»).
2) Обработайте последовательность:
```bash
docker build -f python/tracker_pkg/Dockerfile -t offline-tracker .   # разово
docker run --rm -v "$PWD/catkin_ws/samples:/data" offline-tracker \
  python -m tracker_pkg.scripts.offline_processor /data /data/trajectory.json \
  --format json \
  --width 640 --height 480 --fov 1.047 \
  --camera-pose 0 0 1.0 0 -1.5708 0 \
  --max-distance 3.0
```
3) Сохраните PNG траектории (в контейнере без GUI):
```bash
docker run --rm \
  -v "$PWD/catkin_ws/samples:/data" \
  -v "$PWD/catkin_ws/src/tracker_pkg/scripts/offline_visualizer.py:/app/offline_visualizer.py" \
  offline-tracker \
  sh -c "pip install matplotlib && python /app/offline_visualizer.py /data/trajectory.json --format json --output /data/trajectory.png"

## Проверено «вручную» (шпаргалка)
Рабочий набор команд, которыми прогоняли end-to-end:
```bash
cd ~/Documents/final_project
# сборка образа
docker build -f python/tracker_pkg/Dockerfile -t offline-tracker .

# офлайн обработка снятых кадров (камера z=1.0, смотрит вниз)
docker run --rm -v "$PWD/catkin_ws/samples:/data" offline-tracker \
  python -m tracker_pkg.scripts.offline_processor /data /data/trajectory.json \
  --format json --width 640 --height 480 --fov 1.047 \
  --camera-pose 0 0 1.0 0 -1.5708 0 --max-distance 3.0

# сохранение PNG траектории (без GUI, в контейнере)
docker run --rm \
  -v "$PWD/catkin_ws/samples:/data" \
  -v "$PWD/catkin_ws/src/tracker_pkg/scripts/offline_visualizer.py:/app/offline_visualizer.py" \
  offline-tracker \
  sh -c "pip install matplotlib && python /app/offline_visualizer.py /data/trajectory.json --format json --output /data/trajectory.png"

# онлайн запуск симуляции+трекера+rviz
cd ~/Documents/final_project/catkin_ws
catkin_make && source devel/setup.bash
roslaunch sim_pkg demo.launch
```
```
PNG появится в `catkin_ws/samples/trajectory.png`.

## Нарезка кадров из симуляции
1) Запустите симуляцию:
```bash
cd ~/Documents/final_project/catkin_ws
source devel/setup.bash
roslaunch sim_pkg demo.launch
```
2) В другом терминале сохраните поток камеры:
```bash
cd ~/Documents/final_project/catkin_ws
source devel/setup.bash
mkdir -p samples
rosrun image_view image_saver image:=/camera/image_raw \
  _save_all_image:=true \
  _filename_format:=$PWD/samples/frame_%04d.png
```
Дайте роботу проехать, остановите `Ctrl+C`. Кадры будут в `catkin_ws/samples/`.

## Тесты и покрытие
Офлайн‑ядро покрыто pytest (coverage > 65%).
```bash
cd ~/Documents/final_project/python/tracker_pkg
python3 -m pip install .[dev]    # зависимости для тестов, если нужно
python3 -m pytest --cov=tracker_pkg --cov-report=term-missing
```

## Полезные детали
- Дефолтный мир `sim_pkg/worlds/arena.world`: камера на (0,0,1.0), смотрит вниз (`pitch ≈ -1.57`), FOV ≈ 60°, 640×480.
- Если используете другой мир, передайте путь в `demo.launch`/`tracker.launch` аргументом `world_file`.
- Offline visualizer: интерактивный режим `--interactive` (без `--output` откроется окно), для headless сохранения добавляйте `--output` (+ `--dpi`, по умолчанию 200).
- Docker доступен только при запущенном демоне и добавленном пользователе в группу `docker` (`sudo systemctl start docker`, `sudo usermod -aG docker $USER`, `newgrp docker`).

## Структура
- `catkin_ws/src/sim_pkg` — Gazebo/ROS симуляция, launch файлы, миры.
- `catkin_ws/src/tracker_pkg` — ROS-нода трекера, configs, визуализатор.
- `python/tracker_pkg` — офлайн ядро + CLI + tests + Dockerfile.
- `catkin_ws/samples` — примеры/ваши кадры и результаты офлайн обработки.
- `tracker_logs` — логи онлайн трекера (JSON/PNG).

Ядро `tracker_pkg` (и в ROS-пакете, и в offline-пакете) разложено по слоям:
- `domain/` — чистые модели и расчеты (интринсики/проекция, позы, траектория).
- `adapters/` — работа с внешними библиотеками (OpenCV детектор маркеров).
- `usecases/` — прикладные сценарии (обработка последовательности изображений).
- `plot_trajectory_interactive.py` — инфраструктура/аналитика для готовых логов.
