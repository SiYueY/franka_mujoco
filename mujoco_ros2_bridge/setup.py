from setuptools import setup

package_name = "mujoco_ros2_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name, f"{package_name}.robocasa"],
    package_dir={"": "."},
    zip_safe=True,
    maintainer="TODO",
    maintainer_email="todo@example.com",
    description="Generic MuJoCo runtime bridge for ROS 2.",
    license="Apache-2.0",
)
