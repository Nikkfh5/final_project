# Final Project - External Camera Tracker (ROS + Offline)

Полный стек: симуляция в Gazebo/ROS, нода трекера и офлайн пайплайн на одном коде `python/tracker_pkg` (ROS-пакет подключает его через symlink). Общие папки:
- `samples/` (ссылка на `catkin_ws/samples/`) — кадры из симуляции и офлайн выходы `trajectory.{json,png}`.
- `tracker_logs/` — логи онлайн трекера и артефакты аналитики (`trajectory.json`, PNG/HTML/report).

## Онлайн: Gazebo + RViz
```bash
cd ~/Documents/final_project/catkin_ws
catkin_make
source devel/setup.bash
roslaunch sim_pkg demo.launch use_rviz:=true logging_initially_on:=true log_format:=json \
  log_file:=$PWD/../tracker_logs/trajectory.json png_output:=$PWD/../tracker_logs/trajectory.png
```
- RViz открывается автоматически с конфигом `tracker_pkg/rviz/tracker.rviz`.
- Управление логированием: `rosservice call /tracker_node/start_logging` и `.../stop_logging` (пишет в `tracker_logs/`).
- Можно подменять окружение/движение флагами: `scenario:=empty|obstacles|corridor`, `trajectory:=circle|zigzag|figure8`, `gui:=false` для headless.

  Пример: `roslaunch sim_pkg demo.launch scenario:=obstacles trajectory:=figure8 logging_initially_on:=true`.

## Съёмка кадров из онлайна для офлайна
1) Запусти симуляцию (см. выше).
2) Во втором терминале:
```bash
cd ~/Documents/final_project/catkin_ws
source devel/setup.bash
rosrun image_view image_saver image:=/camera/image_raw _save_all_image:=true \
  _filename_format:=$PWD/samples/frame_%04d.png
```
Кадры и `.ini` с интринзиками складываются в `catkin_ws/samples/` (тот же `samples/` в корне).

## Оффлайн через Docker (обработка кадров и отрисовка)
```bash
cd ~/Documents/final_project
docker build -f python/Dockerfile -t offline-tracker .
```
Обработка с реальными параметрами сим-камеры (матрица из `samples/frame_0002.ini`, камера на z=3, смотрит вниз):
```bash
cd ~/Documents/final_project/catkin_ws
# 1) offline_processor: читает кадры из /data, строит траекторию и сохраняет /data/trajectory.json
docker run --rm -v "$PWD/samples:/data" offline-tracker \
  python -m tracker_pkg.scripts.offline_processor /data /data/trajectory.json --format json \
  --camera-matrix "554.38271,0,320.5,0,554.38271,240.5,0,0,1" \
  --camera-pose 0 0 3.0 3.14159265 0 -1.570796
```
Визуализация офлайн траектории:
```bash
# 2) offline_visualizer: читает готовый /data/trajectory.json и рисует /data/trajectory.png
docker run --rm -v "$PWD/samples:/data" offline-tracker \
  python -m tracker_pkg.scripts.offline_visualizer /data/trajectory.json \
  --format json --output /data/trajectory.png
```
Итоговые файлы смотреть в `catkin_ws/samples/trajectory.{json,png}`.

## Аналитика / отчёты
Работает по умолчанию с онлайн-логом `tracker_logs/trajectory.json`:
```bash
cd ~/Documents/final_project
./run_analysis.sh --report      # HTML-дашборд + PNG/HTML артефакты
./run_analysis.sh --all         # то же самое, явно включает все графики и анимацию
```
Выходы кладутся в `tracker_logs/` (`trajectory_interactive.png`, `trajectory_heatmap.png`, `trajectory_animated.html`, `report.html` и т.д.). Чтобы анализировать офлайн результат, скопируй `samples/trajectory.json` в `tracker_logs/trajectory.json` или пробрось путь через аргументы скрипта.

## Тесты и разработка
```bash
cd ~/Documents/final_project
python3 -m pip install -r requirements.txt   # пакет + визуализация + dev
python3 -m pytest --cov=tracker_pkg --cov-report=term-missing python/tests
```

## Архитектура
- `catkin_ws/src/sim_pkg` — Gazebo миры и launch для камеры; статический TF world->camera.  
- `catkin_ws/src/tracker_pkg` — ROS-нода (`scripts/tracker_node.py`), конфиг `config/tracker.yaml`, RViz/launch. Код пакета — ссылка на `python/tracker_pkg`.  
- `python/tracker_pkg` — офлайн ядро (`domain/`, `adapters/`, `usecases/`, CLI `scripts/offline_processor.py`, интерактивный анализ `plot_trajectory_interactive.py`).  
- `samples/` — кадры + офлайн траектории (общая папка с ROS).  
- `tracker_logs/` — онлайн логи и артефакты аналитики.
