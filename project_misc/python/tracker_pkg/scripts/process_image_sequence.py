#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Скрипт для обработки последовательности изображений из файлов.

Обрабатывает папку с изображениями, детектирует метки, вычисляет траекторию
и сохраняет результаты в CSV/JSON формате.
"""

import os
import sys
import argparse
import json
import csv
import cv2
import numpy as np
from pathlib import Path
import glob


class ImageSequenceProcessor:
    def __init__(self, camera_matrix, camera_pose, ground_z=0.0, 
                 red_hsv_lower=None, red_hsv_upper=None,
                 blue_hsv_lower=None, blue_hsv_upper=None,
                 min_contour_area=50, max_contour_area=10000,
                 frame_skip=1):
        """
        Инициализация процессора изображений.
        
        Args:
            camera_matrix: 3x3 матрица камеры (intrinsics)
            camera_pose: (x, y, z, roll, pitch, yaw) поза камеры в мире
            ground_z: Z координата плоскости пола
            red_hsv_lower/upper: HSV пороги для красной метки
            blue_hsv_lower/upper: HSV пороги для синей метки
            min/max_contour_area: фильтры площади контуров
            frame_skip: обрабатывать каждый N-й кадр
        """
        self.camera_matrix = camera_matrix
        self.camera_pose = camera_pose
        self.ground_z = ground_z
        self.frame_skip = frame_skip
        
        # HSV пороги по умолчанию
        self.red_lower = np.array(red_hsv_lower if red_hsv_lower else [0, 100, 100])
        self.red_upper = np.array(red_hsv_upper if red_hsv_upper else [10, 255, 255])
        self.blue_lower = np.array(blue_hsv_lower if blue_hsv_lower else [100, 100, 100])
        self.blue_upper = np.array(blue_hsv_upper if blue_hsv_upper else [130, 255, 255])
        
        self.min_contour_area = min_contour_area
        self.max_contour_area = max_contour_area
        
        # Вычисляем матрицу поворота из углов Эйлера
        self.rotation_matrix = self._euler_to_rotation_matrix(
            camera_pose[3], camera_pose[4], camera_pose[5])
        self.camera_position = np.array(camera_pose[:3])
        
    def _euler_to_rotation_matrix(self, roll, pitch, yaw):
        """Преобразование углов Эйлера в матрицу поворота."""
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        
        R = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp, cp*sr, cp*cr]
        ])
        return R
    
    def detect_marker(self, cv_image, lower_hsv, upper_hsv):
        """Детекция цветной метки в изображении."""
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
        
        # Морфологические операции
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Поиск контуров
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None
        
        # Находим наибольший контур
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        
        if area < self.min_contour_area or area > self.max_contour_area:
            return None
        
        # Вычисляем центроид
        M = cv2.moments(largest_contour)
        if M["m00"] == 0:
            return None
        
        u = int(M["m10"] / M["m00"])
        v = int(M["m01"] / M["m00"])
        
        return (u, v)
    
    def pixel_to_world_point(self, u, v):
        """Преобразование пикселя в мировую точку через ray-plane intersection."""
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        
        # Нормализованные координаты
        x_norm = (u - cx) / fx
        y_norm = (v - cy) / fy
        
        # Направление луча в системе камеры
        ray_dir_camera = np.array([x_norm, y_norm, 1.0])
        ray_dir_camera = ray_dir_camera / np.linalg.norm(ray_dir_camera)
        
        # Преобразуем в мировую систему
        ray_dir_world = self.rotation_matrix.dot(ray_dir_camera)
        
        # Пересечение с плоскостью пола
        if abs(ray_dir_world[2]) < 1e-6:
            return None
        
        t = (self.ground_z - self.camera_position[2]) / ray_dir_world[2]
        if t < 0:
            return None
        
        point_world = self.camera_position + t * ray_dir_world
        return (point_world[0], point_world[1], point_world[2])
    
    def process_image(self, image_path):
        """Обработка одного изображения."""
        img = cv2.imread(str(image_path))
        if img is None:
            return None
        
        # Детекция меток
        red_center = self.detect_marker(img, self.red_lower, self.red_upper)
        blue_center = self.detect_marker(img, self.blue_lower, self.blue_upper)
        
        if red_center is None or blue_center is None:
            return None
        
        # Преобразование в мировые координаты
        red_world = self.pixel_to_world_point(red_center[0], red_center[1])
        blue_world = self.pixel_to_world_point(blue_center[0], blue_center[1])
        
        if red_world is None or blue_world is None:
            return None
        
        # Вычисление позы робота
        center_x = (red_world[0] + blue_world[0]) / 2.0
        center_y = (red_world[1] + blue_world[1]) / 2.0
        
        dx = blue_world[0] - red_world[0]
        dy = blue_world[1] - red_world[1]
        yaw = np.arctan2(dy, dx)
        
        return {
            'x': center_x,
            'y': center_y,
            'theta': yaw,
            'red_pixel': red_center,
            'blue_pixel': blue_center
        }
    
    def process_sequence(self, image_dir, output_file, output_format='csv'):
        """
        Обработка последовательности изображений.
        
        Args:
            image_dir: Путь к папке с изображениями
            output_file: Путь к выходному файлу
            output_format: 'csv' или 'json'
        """
        # Находим все изображения
        image_extensions = ['*.jpg', '*.jpeg', '*.png', '*.bmp']
        image_files = []
        for ext in image_extensions:
            image_files.extend(glob.glob(os.path.join(image_dir, ext)))
            image_files.extend(glob.glob(os.path.join(image_dir, ext.upper())))
        
        image_files.sort()
        
        if not image_files:
            print(f"Не найдено изображений в {image_dir}")
            return
        
        print(f"Найдено {len(image_files)} изображений")
        
        # Обработка
        trajectory = []
        frame_counter = 0
        
        for i, img_path in enumerate(image_files):
            if i % self.frame_skip != 0:
                continue
            
            result = self.process_image(img_path)
            if result:
                trajectory.append({
                    't': i * 0.033,  # Предполагаем 30 FPS
                    'x': result['x'],
                    'y': result['y'],
                    'theta': result['theta'],
                    'frame': frame_counter
                })
                frame_counter += 1
            else:
                print(f"Не удалось обработать {img_path}")
        
        print(f"Обработано {len(trajectory)} точек траектории")
        
        # Сохранение
        if output_format == 'json':
            with open(output_file, 'w') as f:
                json.dump(trajectory, f, indent=2)
        else:  # CSV
            with open(output_file, 'w') as f:
                writer = csv.writer(f)
                writer.writerow(['t', 'x', 'y', 'theta', 'frame'])
                for point in trajectory:
                    writer.writerow([
                        point['t'],
                        point['x'],
                        point['y'],
                        point['theta'],
                        point['frame']
                    ])
        
        print(f"Траектория сохранена в {output_file}")


def main():
    parser = argparse.ArgumentParser(
        description='Обработка последовательности изображений для трекинга робота')
    parser.add_argument('image_dir', type=str, help='Путь к папке с изображениями')
    parser.add_argument('output_file', type=str, help='Путь к выходному файлу')
    parser.add_argument('--format', choices=['csv', 'json'], default='csv',
                       help='Формат выходного файла')
    parser.add_argument('--camera-matrix', type=str, required=True,
                       help='Матрица камеры в формате "fx,0,cx,0,fy,cy,0,0,1"')
    parser.add_argument('--camera-pose', type=str, required=True,
                       help='Поза камеры в формате "x,y,z,roll,pitch,yaw"')
    parser.add_argument('--ground-z', type=float, default=0.0,
                       help='Z координата плоскости пола')
    parser.add_argument('--frame-skip', type=int, default=1,
                       help='Обрабатывать каждый N-й кадр')
    parser.add_argument('--red-hsv', type=str,
                       help='HSV пороги для красной метки: "lower_h,lower_s,lower_v,upper_h,upper_s,upper_v"')
    parser.add_argument('--blue-hsv', type=str,
                       help='HSV пороги для синей метки: "lower_h,lower_s,lower_v,upper_h,upper_s,upper_v"')
    
    args = parser.parse_args()
    
    # Парсинг матрицы камеры
    k_values = [float(x) for x in args.camera_matrix.split(',')]
    camera_matrix = np.array(k_values).reshape(3, 3)
    
    # Парсинг позы камеры
    pose_values = [float(x) for x in args.camera_pose.split(',')]
    camera_pose = tuple(pose_values)
    
    # Парсинг HSV порогов
    red_lower, red_upper = None, None
    blue_lower, blue_upper = None, None
    
    if args.red_hsv:
        hsv_vals = [int(x) for x in args.red_hsv.split(',')]
        red_lower = hsv_vals[:3]
        red_upper = hsv_vals[3:]
    
    if args.blue_hsv:
        hsv_vals = [int(x) for x in args.blue_hsv.split(',')]
        blue_lower = hsv_vals[:3]
        blue_upper = hsv_vals[3:]
    
    # Создание процессора
    processor = ImageSequenceProcessor(
        camera_matrix=camera_matrix,
        camera_pose=camera_pose,
        ground_z=args.ground_z,
        red_hsv_lower=red_lower,
        red_hsv_upper=red_upper,
        blue_hsv_lower=blue_lower,
        blue_hsv_upper=blue_upper,
        frame_skip=args.frame_skip
    )
    
    # Обработка
    processor.process_sequence(args.image_dir, args.output_file, args.format)


if __name__ == '__main__':
    main()
