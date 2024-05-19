from setuptools import find_packages, setup

package_name = 'comm_package_test'

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
    maintainer='sjk015',
    maintainer_email='santamav@uji.es',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = comm_package_test.publisher:main',
            'subscriber = comm_package_test.subscriber:main'
        ],
    },
)
