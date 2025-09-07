from setuptools import find_packages, setup

package_name = 'demo_basecontrol'

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
    maintainer='dk',
    maintainer_email='30941028@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = demo_basecontrol.publisher:main',
            'listener = demo_basecontrol.subscriber:main',
            'base_control = demo_basecontrol.base_control:main',
        ],
    },
)
