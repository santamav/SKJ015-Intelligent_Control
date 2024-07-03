from setuptools import find_packages, setup

package_name = 'ibvs'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ibvs.launch.py'])
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
            'feature_extractor = ibvs.feature_extractor:main',
            'controller = ibvs.controller:main',
            'actuator = ibvs.actuator:main'
        ],
    },
)
