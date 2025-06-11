from setuptools import find_packages, setup
import os 
from glob import glob 

package_name = 'chair_detector'

setup(
    name=package_name,
    version='0.0.1', 
    packages=find_packages(exclude=['test']), # Найдет 'chair_detector' как Python-пакет (директорию с __init__.py)
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]), # 'resource/chair_detector'
        ('share/' + package_name, ['package.xml']), # 'chair_detector/package.xml'
        # Пути для launch и rviz файлов, если они на верхнем уровне пакета
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')), # Ищет в 'launch/*.py' относительно setup.py
        (os.path.join('share', package_name, 'rviz'),   glob('rviz/*.rviz')),   # Ищет в 'rviz/*.rviz' относительно setup.py
        # Если у вас есть директория config, добавьте и ее:
        # (os.path.join('share', package_name, 'config'), glob('config/*')), 
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='mirand',
    maintainer_email='your_actual_email@example.com', # Замените
    description='Package for chair detection, including ESP32 lidar processing.', # Обновите
    license='Apache License 2.0', # Замените
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'chair_detector_node = chair_detector.chair_detector_node:main',
            'lidar_visual = chair_detector.lidar_visual_node:main',
            'lidar_bridge = chair_detector.lidar_bridge_node:main',
            'chair_detector_simple = chair_detector.chair_detector_simple:main',
        ],
    },
)