from setuptools import setup

package_name = 'arithmetic'

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
            'arg = arithmetic.argument:main',
            'argsub = arithmetic.argumentsub:main',
            'main = arithmetic.main:main',
            'operator = arithmetic.operator:main',
            'checker = arithmetic.checker:main'
        ],
    },
)
