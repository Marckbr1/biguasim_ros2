from setuptools import find_packages, setup

package_name = 'biguasim_sensores'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/params_biguasim.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='biguauser',
    maintainer_email='biguauser@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sensores = biguasim_sensores.sensores_biguasim:main',
        ],
    },
)
