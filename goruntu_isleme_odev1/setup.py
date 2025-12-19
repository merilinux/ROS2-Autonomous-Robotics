from setuptools import setup # find_packages'ı sildik
import os 
from glob import glob

package_name = 'goruntu_isleme'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name], # BURAYI DOĞRUDAN BÖYLE YAZ AŞKIM
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='meri',
    maintainer_email='meri@todo.todo',
    description='Görüntü İşleme Ödevi',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = goruntu_isleme.vision_node:main',
            'controller_node = goruntu_isleme.controller_node:main',
        ],
    },
)