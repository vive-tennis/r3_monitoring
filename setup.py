from setuptools import setup

setup(
    name='r3_monitoring',
    version='0.1.0',
    packages=['r3_monitoring', 'r3_monitoring_rqt'],
    package_dir={'': 'src'},

    # scripts=['scripts/my_script.py'],
    install_requires=['rospy', 'paho-mqtt'],
    author='Javad Amirian',
    author_email='amiryan.j@gmail.com',
    description='Monitoring Robot Remotely in Realtime',
    license='MIT License',
    keywords='ROS',
    url='https://github.com/vive-tennis/r3-monitoring-client',
)
