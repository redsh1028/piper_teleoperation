from setuptools import find_packages, setup

package_name = "piper_sdk"
subpackages = find_packages(where=".")

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name] + [f"{package_name}.{pkg}" for pkg in subpackages],
    package_dir={package_name: "."},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="yumin",
    maintainer_email="yumin@example.com",
    description="Python Piper SDK used by the ROS Piper control node.",
    license="Apache-2.0",
)
