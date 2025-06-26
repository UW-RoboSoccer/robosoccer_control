from setuptools import setup

package_name = 'robosoccer_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=[
        'robosoccer_control.tsid_controller_node',
        'robosoccer_control.tsid_to_trajectory_bridge',
        'robosoccer_control.simple_bridge_test',
        'robosoccer_control.tsid_test_commander',
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='UWRS',
    maintainer_email='uwrobosoccer@gmail.com',
    description='TSID-based control for robosoccer humanoid robot (Python version)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tsid_controller_node = robosoccer_control.tsid_controller_node:main',
            'tsid_to_trajectory_bridge = robosoccer_control.tsid_to_trajectory_bridge:main',
            'simple_bridge_test = robosoccer_control.simple_bridge_test:main',
            'tsid_test_commander = robosoccer_control.tsid_test_commander:main',
        ],
    },
) 