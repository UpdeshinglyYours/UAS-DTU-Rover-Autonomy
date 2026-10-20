from glob import glob

from setuptools import setup

package_name = "fcb_orin_driver"

setup(
    name=package_name,
    version="0.2.0",
    packages=[package_name, "fcb_base_driver"],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="uasavionics",
    maintainer_email="uasavionics@gmail.com",
    description="ROS 2 driver and flight recorder for the Sony FCB-EV9520L",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "fcb_driver = fcb_orin_driver.node:main",
            "fcb_keys = fcb_orin_driver.keys:main",
            "fcb_describe_camera = fcb_orin_driver.describe:main",
        ],
    },
)
