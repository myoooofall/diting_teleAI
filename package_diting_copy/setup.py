from setuptools import setup

package_name = 'package_diting'

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
    maintainer='diting',
    maintainer_email='diting@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'detect=package_diting.detect_voice_ros:main',
        'recognize=package_diting.xfyun_recognize_ros:main',
        'voice=package_diting.tts_voice_ros:main',
        ],
    },
    
)
