from setuptools import find_packages, setup

package_name = 'strn_step'

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
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'height_grid = strn_step.height_grid:main',
            'wheel_step_detector = strn_step.wheel_step_detector:main',
            'ground_grid = strn_step.ground_grid:main',
        ],
    },
)
