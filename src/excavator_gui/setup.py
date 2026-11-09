from setuptools import setup

package_name = 'excavator_gui'

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
    description='Qt slider GUI that publishes joint states for the excavator.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'slider_gui = excavator_gui.slider_gui:main',
        ],
    },
)
