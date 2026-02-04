from setuptools import find_packages, setup

package_name = "unitree_g1_app"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/g1_app_launch.py"]),
        ("share/" + package_name + "/config", ["config/g1_app_params.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="User",
    maintainer_email="user@example.com",
    description="Unitree G1 ROS2 Jazzy app: movement, audio, video",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "movement_node = unitree_g1_app.movement_node:main",
            "audio_node = unitree_g1_app.audio_node:main",
            "video_node = unitree_g1_app.video_node:main",
            "g1_demo = unitree_g1_app.g1_demo:main",
        ],
    },
)
