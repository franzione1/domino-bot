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

        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # I 3 Domino (Aggiungi le nuove righe)
        (os.path.join('share', package_name, 'models/domino_rg'), glob('models/domino_rg/*')),
        (os.path.join('share', package_name, 'models/domino_gb'), glob('models/domino_gb/*')),
        (os.path.join('share', package_name, 'models/domino_br'), glob('models/domino_br/*')),
	
	# Il tavolo
        (os.path.join('share', package_name, 'models/work_table'), glob('models/work_table/*')),
	
	# La videocamera
	(os.path.join('share', package_name, 'models/camera_sensor'), glob('models/camera_sensor/*')),

        # Il file World (NUOVO)
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
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
            # Nome comando = pacchetto.file:funzione_main
            'vision_processor = domino_project.vision_processor:main',
            'robot_mover = domino_project.robot_mover:main', 
        ],
    },
)
