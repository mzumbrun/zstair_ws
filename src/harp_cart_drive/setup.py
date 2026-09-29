from glob import glob
from setuptools import setup

package_name = "harp_cart_drive"

setup(
    name=package_name,
    version="0.2.2",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mike",
    description="HARP cart differential-drive node",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "drive_node = harp_cart_drive.drive_node:main",
            "enc_snapshot = harp_cart_drive.enc_snapshot:main",
            "speed_check = harp_cart_drive.speed_check:main",
        ],
    },
)
