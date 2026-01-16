from setuptools import setup
import os
from glob import glob

package_name = 'domino_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # CARTELLE MODELLI
        (os.path.join('share', package_name, 'models/domino_rg'), glob('models/domino_rg/*')),
        (os.path.join('share', package_name, 'models/domino_gb'), glob('models/domino_gb/*')),
        (os.path.join('share', package_name, 'models/domino_br'), glob('models/domino_br/*')),
        (os.path.join('share', package_name, 'models/work_table'), glob('models/work_table/*')),
        (os.path.join('share', package_name, 'models/camera_sensor'), glob('models/camera_sensor/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),

        # --- QUESTA È LA RIGA NUOVA DA AGGIUNGERE ---
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tu',
    maintainer_email='tu@todo.todo',
    description='Progetto Domino Robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_processor = domino_project.vision_processor:main',
            'robot_mover = domino_project.robot_mover:main', 
        ],
    },
)