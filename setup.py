from setuptools import find_packages, setup

package_name = 'windsor_ekf_tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=['windsor_ekf_tracker'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='michaelalabi',
    maintainer_email='michaelalabi1601@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ekf_node = windsor_ekf_tracker.ekf_tracker_node:main',
            'trajectory_sim = windsor_ekf_tracker.trajectory_publisher:main',
        ],
    },
)
