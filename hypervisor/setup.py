from setuptools import setup

package_name = 'hypervisor'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Brett Csotty',
    maintainer_email='bcsotty@umich.edu',
    description='Project Phoenix hypervisor node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'hypervisor = hypervisor.hypervisor:main'
        ],
    },
)
