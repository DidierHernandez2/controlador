from setuptools import find_packages, setup

package_name = 'controlador'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
     ('share/ament_index/resource_index/packages', ['resource/controlador']),
     ('share/controlador', ['package.xml']),
     ('share/controlador/launch', ['launch/square_launch.py']),
    ] ,
    install_requires=[ 'setuptools',
        'transforms3d',
        'numpy',],
    zip_safe=True,
    maintainer='darhf',
    maintainer_email='didier.hernandez1972@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'square_pid_control = controlador.square_pid_control:main'
        ],
    },
)
