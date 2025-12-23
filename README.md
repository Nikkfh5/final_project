# Final Project — External Camera Tracker (ROS + Offline)

Полный стек: симуляция в Gazebo/ROS, нода трекера и офлайн пайплайн на одном коде `python/tracker_pkg` (ROS-пакет подключает его через symlink). Данные держим в `samples/` (это ссылка на `catkin_ws/samples/`), логи — в `tracker_logs/`.

## Быстрый старт (ROS)
```bash
cd ~/Documents/final_project/catkin_ws
catkin_make
source devel/setup.bash
roslaunch sim_pkg demo.launch logging_initially_on:=true log_format:=json
```
Логи появятся в `../tracker_logs/trajectory.{json,png}`. Управление логированием: `rosservice call /tracker_node/start_logging` и `.../stop_logging`.

## Быстрый старт (офлайн)
1) Сложите кадры PNG в `samples/` (общая папка для офлайна и ROS).  
2) Соберите образ: `docker build -f python/Dockerfile -t offline-tracker .`  
3) Запустите обработку (пример для камеры z=1.0, смотрит вниз):
```bash
docker run --rm -v "$PWD/samples:/data" offline-tracker \
  python -m tracker_pkg.scripts.offline_processor /data /data/trajectory.json \
  --format json --width 640 --height 480 --fov 1.047 \
  --camera-pose 0 0 1.0 0 -1.5708 0 --max-distance 3.0
```
4) Сохраните PNG траектории (matplotlib уже в образе):
```bash
docker run --rm -v "$PWD/samples:/data" offline-tracker \
  python -m tracker_pkg.scripts.offline_visualizer /data/trajectory.json \
  --format json --output /data/trajectory.png
```

## Нарезка кадров из симуляции
```bash
cd ~/Documents/final_project/catkin_ws
source devel/setup.bash
roslaunch sim_pkg demo.launch
rosrun image_view image_saver image:=/camera/image_raw _save_all_image:=true \
  _filename_format:=$PWD/samples/frame_%04d.png
```

## Тесты и зависимости
```bash
cd ~/Documents/final_project
python3 -m pip install -r requirements.txt   # ставит пакет + viz + dev
python3 -m pytest --cov=tracker_pkg --cov-report=term-missing python/tests
```

## Архитектура
- `catkin_ws/src/sim_pkg` — Gazebo миры и launch для камеры; статический TF world→camera.  
- `catkin_ws/src/tracker_pkg` — ROS-нода (`scripts/tracker_node.py`), конфиг `config/tracker.yaml`, viz/launch. Код пакета — ссылка на `python/tracker_pkg`.  
- `python/tracker_pkg` — офлайн ядро (слои `domain/`, `adapters/`, `usecases/` + CLI `scripts/offline_processor.py`, визуализация `plot_trajectory_interactive.py`).  
- `samples/` — пользовательские кадры/лог траекторий, шарятся с ROS через symlink.  
- `tracker_logs/` — выходы онлайн трекера (git-ignored).

Ключевые параметры: HSV-пороги, min/max расстояние между метками, frame_skip, сглаживание yaw (`config/tracker.yaml`); пути логов задаются аргументами `roslaunch` (`log_file`, `png_output`).

## Проверка вручную
```bash
cd ~/Documents/final_project/catkin_ws
catkin_make && source devel/setup.bash
roslaunch sim_pkg demo.launch           # online проверка
```
```bash
cd ~/Documents/final_project
docker build -f python/Dockerfile -t offline-tracker .
docker run --rm -v "$PWD/samples:/data" offline-tracker \
  python -m tracker_pkg.scripts.offline_processor /data /data/trajectory.json --format json \
  --width 640 --height 480 --fov 1.047 --camera-pose 0 0 1.0 0 -1.5708 0
```
