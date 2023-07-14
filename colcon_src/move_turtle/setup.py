from setuptools import setup

package_name = 'move_turtle'

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
            'mturtle = move_turtle.random_move:main',
            'tb3m = move_turtle.tb3_my_move:main',
            'tb3tori = move_turtle.tb3_to_origin:main',
            'tb3rot = move_turtle.tb3_rotate:main',
            'tb3imagesub = move_turtle.tb3_image_sub:main',
            'tb3lasersub = move_turtle.tb3_laser_sub:main',
            'tb3imagehog = move_turtle.tb3_image_hog:main',
            'tb3yelline = move_turtle.tb3_yellow_line:main'
        ],
    },
)
