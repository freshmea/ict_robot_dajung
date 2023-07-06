from setuptools import setup

package_name = 'my_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aa',
    maintainer_email='freshmea@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mpub = my_pkg.simple_message_pub:main',
            'msub = my_pkg.simple_message_sub:main',
            'tpub = my_pkg.simple_time_pub:main',
            'tsub = my_pkg.simple_time_sub:main',
            'm2sub = my_pkg.simple_message_sub2:main',
            'mtsub = my_pkg.simple_message_time_sub:main',
            'myinterfacepub = my_pkg.simple_myinterface_pub:main',
            'myinterfacesub = my_pkg.simple_myinterface_sub:main',
            'msrvserver = my_pkg.simple_srv_server:main',
            'msrvclient = my_pkg.simple_srv_client:main',
            'mactionserver = my_pkg.simple_action_server:main',
            'mactionclient = my_pkg.simple_action_client:main'
        ],
    },
)
