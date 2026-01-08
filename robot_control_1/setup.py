from setuptools import setup, find_packages

package_name = 'robot_control_1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name]),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Takayuki Hashizume',
    maintainer_email='AE1212@nara.kosen-ac.jp',
    description='SCOUT MINIを制御するための自作コントローラーパッケージ',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'robot_control_1 = robot_control_1.robot_control_1:main',
        ],
    },
)
