# Offline Tracker (Python)

Обработчик последовательностей изображений без ROS. По цветным меткам вычисляет позу робота и сохраняет траекторию в CSV/JSON.

## Установка

```bash
cd python/tracker_pkg
python -m pip install .[dev]
```

## Запуск

```bash
python -m tracker_pkg.scripts.offline_processor \
  ./data/frames \
  ./trajectory.csv \
  --camera-pose 0 0 1 0 3.14159 0 \
  --width 640 --height 480 --fov 1.047
```

Если известна матрица камеры:

```bash
python -m tracker_pkg.scripts.offline_processor ./data frames.csv \
  --camera-matrix 500,0,320,0,500,240,0,0,1
```

## Offline Trajectory Analysis

Для анализа траектории используется отдельный Python-скрипт:
`python/tracker_pkg/tracker_pkg/plot_trajectory_interactive.py`

### Запуск

Из корня проекта:

`./run_analysis.sh`


### Доступные режимы

- Без флагов  
  Интерактивный matplotlib-график (hover по точкам)

- `--save`  
  Сохранение PNG:
  `tracker_logs/trajectory_interactive.png`
  
  
- `--html`  
Интерактивный HTML-график с подсказками (открывается в браузере)

- `--html-anim`  
Анимированная HTML-визуализация с ориентацией робота (стрелка),
фиксированным масштабом и управлением воспроизведением



## Тесты и coverage

```bash
python -m pytest --cov=tracker_pkg --cov-report=term-missing
```

## Docker

```bash
docker build -t offline-tracker:latest python/tracker_pkg
docker run --rm -v $(pwd)/samples:/data offline-tracker:latest \
  python -m tracker_pkg.scripts.offline_processor /data /data/out.csv
```

Образ можно запушить в DockerHub: `docker tag offline-tracker <login>/offline-tracker:latest && docker push <login>/offline-tracker:latest`.
