#!/bin/bash

# Скрипт для установки зависимостей проекта

set -e

echo "=== Установка зависимостей для проекта трекинга робота ==="

# Определяем версию ROS
if [ -z "$ROS_DISTRO" ]; then
    echo "Ошибка: ROS не найден. Убедитесь, что вы запустили setup.bash для вашей версии ROS."
    exit 1
fi

echo "Обнаружена версия ROS: $ROS_DISTRO"

# Установка ROS пакетов
echo ""
echo "Установка ROS пакетов..."
sudo apt-get update
sudo apt-get install -y \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-gazebo-ros \
    ros-$ROS_DISTRO-gazebo-plugins \
    ros-$ROS_DISTRO-rviz \
    ros-$ROS_DISTRO-rqt-image-view

# Установка Python библиотек
echo ""
echo "Установка Python библиотек..."

# Определяем версию Python
if command -v python3 &> /dev/null; then
    PYTHON_CMD=python3
    PIP_CMD=pip3
else
    PYTHON_CMD=python
    PIP_CMD=pip
fi

echo "Используется: $PYTHON_CMD"

# Проверяем наличие pip
if ! command -v $PIP_CMD &> /dev/null; then
    echo "Установка pip..."
    sudo apt-get install -y python3-pip
fi

# Установка библиотек
$PIP_CMD install --user opencv-python numpy matplotlib

echo ""
echo "=== Установка завершена ==="
echo ""
echo "Следующие шаги:"
echo "1. Скопируйте пакеты в ваш catkin workspace:"
echo "   cp -r gazebo/sim_pkg ~/catkin_ws/src/"
echo "   cp -r python/tracker_pkg ~/catkin_ws/src/"
echo ""
echo "2. Соберите проект:"
echo "   cd ~/catkin_ws"
echo "   catkin_make"
echo "   source devel/setup.bash"
echo ""
echo "3. Запустите проект:"
echo "   roslaunch sim_pkg demo.launch"
