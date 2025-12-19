from setuptools import find_packages, setup

package_name = 'goruntu_isleme_odev2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='meri',
    maintainer_email='meri@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lane_node = goruntu_isleme_odev2.lane_node:main',
            'stop_sign_node = goruntu_isleme_odev2.stop_sign_node:main',
            'lane_follow_controller = goruntu_isleme_odev2.lane_follow_controller:main',
        ],
    },
)
