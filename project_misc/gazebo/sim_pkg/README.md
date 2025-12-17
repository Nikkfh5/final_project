# Gazebo Simulation Package (sim_pkg)

Этот пакет содержит конфигурацию Gazebo для симуляции робота с цветными метками и внешней камерой.

## Что делает Gazebo team

1. **Обеспечивает работу симуляции** с миром `arena.world`
2. **Публикует топики камеры**:
   - `/camera/image_raw` (sensor_msgs/Image) - изображение с внешней камеры
   - `/camera/camera_info` (sensor_msgs/CameraInfo) - параметры камеры (intrinsics)
3. **Публикует статический TF**: `world` → `camera_link` (поза камеры фиксирована и известна)
4. **Содержит модель робота** с двумя цветными метками (красная и синяя)

## Как проверить работу

После запуска `roslaunch sim_pkg sim.launch`:

1. **Проверить топики**:
   ```bash
   rostopic list
   # Должны появиться:
   # /camera/image_raw
   # /camera/camera_info
   ```

2. **Посмотреть изображение**:
   ```bash
   rqt_image_view /camera/image_raw
   ```

3. **Проверить TF**:
   ```bash
   rosrun tf view_frames
   # Должен появиться transform world -> camera_link
   ```

4. **Проверить camera_info**:
   ```bash
   rostopic echo /camera/camera_info
   ```

## Фиксированные параметры

- **Поза камеры**: определяется в `sim.launch` через static_transform_publisher
  - По умолчанию: x=0.0, y=0.0, z=2.0, roll=0.0, pitch=0.524 (~30°), yaw=0.0
  - TODO: согласовать точные значения с Python team

- **Параметры камеры**: в `arena.world`
  - Разрешение: 640x480
  - FOV: ~60 градусов
  - Частота: 30 Hz

- **Система координат**:
  - `world` - фиксированная система координат мира
  - `camera_link` - система координат камеры
  - Плоскость пола: z = 0.0

## Структура пакета

- `launch/sim.launch` - запуск симуляции
- `worlds/arena.world` - описание мира с камерой
- `models/robot_with_markers/model.sdf` - модель робота с метками

## TODO для Gazebo team

- [ ] Настроить оптимальную позу камеры (убедиться, что робот в кадре)
- [ ] Уточнить позиции меток на роботе относительно центра
- [ ] Проверить, что метки хорошо видны на изображении
- [ ] Документировать точные параметры камеры и TF для Python team

