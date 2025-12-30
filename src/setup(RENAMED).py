from setuptools import find_packages, setup

package_name = 'tsp_solver'

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
    maintainer='stefania',
    maintainer_email='stefania@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'tsp_solver_node = tsp_solver.tsp_solver_node:main',
            'tsp_route_calculator = tsp_solver.tsp_route_calculator:main',
            'deliver_robot_tsp_client = tsp_solver.deliver_robot_tsp_client:main'
        ],
    },
)
