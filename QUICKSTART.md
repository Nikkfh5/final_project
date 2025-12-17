# Быстрый старт

## Минимальные шаги для запуска

1. **Установите зависимости:**

   ```bash
   ./setup.sh
   ```

2. **Скопируйте пакеты в catkin workspace:**

   ```bash
   # Если у вас еще нет workspace:
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/src

   # Скопируйте пакеты (замените путь на актуальный)
   cp -r /path/to/ysl/gazebo/sim_pkg .
   cp -r /path/to/ysl/python/tracker_pkg .
   ```

3. **Соберите проект:**

   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

4. **Запустите:**
   ```bash
   roslaunch sim_pkg demo.launch
   ```

## Что вы увидите

- **Gazebo окно** - симуляция с роботом и камерой
- **RViz окно** - визуализация траектории и позы робота
- **rqt_image_view** - окно с `/debug/image`, где видны найденные метки
- В консоли будут логи от трекера

## Проверка работы

В новом терминале:

```bash
# Проверьте топики
rostopic list

# Посмотрите текущую позу
rostopic echo /robot_pose_external

# Посмотрите debug изображение с метками
rqt_image_view /debug/image
```

## Остановка

Нажмите `Ctrl+C` в терминале где запущен launch файл. Траектория автоматически сохранится в `~/tracker_logs/trajectory.csv` (если включено логирование).

## Визуализация сохраненной траектории

```bash
rosrun tracker_pkg offline_visualizer.py ~/tracker_logs/trajectory.csv --interactive
```

Используйте ползунок для перемещения по времени и кликайте на точки траектории для просмотра координат.

## Проблемы?

См. [INSTALL.md](INSTALL.md) для подробной информации и решения проблем.
