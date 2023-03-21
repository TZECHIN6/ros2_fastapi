from setuptools import setup

package_name = 'ros2_fastapi'

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
    maintainer='Thomas LUK',
    maintainer_email='thomasluk@hkpc.org',
    description='charging robot web service node',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_service = ros2_fastapi.web_service:main'
        ],
    },
)
