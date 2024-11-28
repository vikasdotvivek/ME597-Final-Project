from setuptools import find_packages, setup

package_name = 'try1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mapper.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='me597',
    maintainer_email='me597@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task1 = try1.task1:main',
        ],
    },
)
