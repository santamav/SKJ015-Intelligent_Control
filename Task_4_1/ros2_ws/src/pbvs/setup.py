from setuptools import find_packages, setup

package_name = 'pbvs'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/pbvs.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sjk015',
    maintainer_email='santamav@uji.es',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'feature_extractor = pbvs.feature_extractor:main',
            'controller = pbvs.controller:main',
            'actuator = pbvs.actuator:main'
        ],
    },
)
