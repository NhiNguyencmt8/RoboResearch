from setuptools import find_packages, setup

package_name = 'lab4'

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
    maintainer='rafael',
    maintainer_email='61450663+NhiNguyencmt8@users.noreply.github.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detect = lab4.object_detect:main',
            'object_avoidance = lab4.object_avoidance:main',
            'get_object_range = lab4.get_object_range:main',
            'go_to_goal = lab4.go_to_goal:main',
            'go_to_goal_v2 = lab4.go_to_goal_v2:main'
        ],
    },
)
