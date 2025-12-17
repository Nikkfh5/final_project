#!/bin/bash
# Скрипт для проверки кода перед запуском

echo "=== Проверка синтаксиса Python файлов ==="
echo ""

# Проверка tracker_node.py
echo "Проверка tracker_node.py..."
python3 -m py_compile python/tracker_pkg/scripts/tracker_node.py
if [ $? -eq 0 ]; then
    echo "✓ tracker_node.py - OK"
else
    echo "✗ tracker_node.py - ОШИБКИ СИНТАКСИСА"
    exit 1
fi

# Проверка offline_visualizer.py
echo "Проверка offline_visualizer.py..."
python3 -m py_compile python/tracker_pkg/scripts/offline_visualizer.py
if [ $? -eq 0 ]; then
    echo "✓ offline_visualizer.py - OK"
else
    echo "✗ offline_visualizer.py - ОШИБКИ СИНТАКСИСА"
    exit 1
fi

# Проверка camera_debug.py
echo "Проверка camera_debug.py..."
python3 -m py_compile python/tracker_pkg/scripts/camera_debug.py
if [ $? -eq 0 ]; then
    echo "✓ camera_debug.py - OK"
else
    echo "✗ camera_debug.py - ОШИБКИ СИНТАКСИСА"
    exit 1
fi

echo ""
echo "=== Проверка импортов (без ROS) ==="
echo ""

# Проверка базовых импортов (только стандартные и numpy/cv2, без ROS)
python3 -c "
import sys
import os

# Добавляем путь к скриптам
sys.path.insert(0, 'python/tracker_pkg/scripts')

# Проверяем наличие необходимых библиотек
try:
    import numpy as np
    print('✓ numpy установлен')
except ImportError:
    print('✗ numpy НЕ установлен')
    sys.exit(1)

try:
    import cv2
    print('✓ opencv-python установлен')
except ImportError:
    print('✗ opencv-python НЕ установлен')
    sys.exit(1)

try:
    import matplotlib
    print('✓ matplotlib установлен')
except ImportError:
    print('✗ matplotlib НЕ установлен')
    sys.exit(1)

print('')
print('Все необходимые Python библиотеки установлены')
"

if [ $? -ne 0 ]; then
    echo ""
    echo "ВНИМАНИЕ: Некоторые библиотеки не установлены"
    echo "Установите их командой: pip3 install numpy opencv-python matplotlib"
    exit 1
fi

echo ""
echo "=== Проверка структуры файлов ==="
echo ""

# Проверка наличия важных файлов
files_to_check=(
    "python/tracker_pkg/scripts/tracker_node.py"
    "python/tracker_pkg/scripts/offline_visualizer.py"
    "python/tracker_pkg/config/tracker.yaml"
    "python/tracker_pkg/launch/tracker.launch"
    "gazebo/sim_pkg/launch/demo.launch"
    "gazebo/sim_pkg/launch/sim.launch"
)

for file in "${files_to_check[@]}"; do
    if [ -f "$file" ]; then
        echo "✓ $file существует"
    else
        echo "✗ $file НЕ найден"
    fi
done

echo ""
echo "=== Проверка прав на выполнение ==="
echo ""

# Проверка прав на выполнение для Python скриптов
scripts=(
    "python/tracker_pkg/scripts/tracker_node.py"
    "python/tracker_pkg/scripts/offline_visualizer.py"
)

for script in "${scripts[@]}"; do
    if [ -x "$script" ]; then
        echo "✓ $script имеет права на выполнение"
    else
        echo "⚠ $script не имеет прав на выполнение (можно добавить: chmod +x $script)"
    fi
done

echo ""
echo "=== Все проверки завершены ==="
echo ""
echo "Для полной проверки с ROS выполните:"
echo "  source ~/catkin_ws/devel/setup.bash"
echo "  rostopic list"
echo "  roslaunch sim_pkg demo.launch"

