from setuptools import setup

package_name = 'navi_diting'
#submodule1 = "navi_diting/Yolov5"
#submodule2 = "navi_diting/Yolov5/utils"
#submodule3 = "navi_diting/Yolov5/models"

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
        'navigate_start=navi_diting.navigation_ros:main',
        'test_navi=navi_diting.test_navi_ros:main',
        ],
    },
)
