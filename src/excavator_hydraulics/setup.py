from setuptools import setup

package_name = 'excavator_hydraulics'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    extras_require={'test': ['pytest']},
    maintainer='tadas',
    maintainer_email='tadasrasimavicius@outlook.com',
    description="Publishes TF for the excavator's hydraulic cylinder barrels and rods.",
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'cylinder_tf = excavator_hydraulics.cylinder_tf:main',
        ],
    },
)
