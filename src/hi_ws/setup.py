from setuptools import find_packages, setup

package_name = 'hi_ws'

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
    maintainer='davinci',
    maintainer_email='you@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['pid_circle=hi_ws.PIDcircle:main',
                            'auto=hi_ws.Auto:main',
                            'c_server=hi_ws.circle_server:main',
                            'c_client=hi_ws.circle_client:main',
        ],
    },
)
