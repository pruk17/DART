"""Classic setuptools packaging for the pure-Python DART library.

A plain pip package (NOT a ROS/ament package): `pip install -e src/DART`.
Classic setup.py metadata is used because the ROS2 Humble container pins
setuptools 59.6, which predates PEP 621 (pyproject [project]) and PEP 660
(editable installs from pyproject). This keeps colcon/ament untouched.
"""

from setuptools import find_packages, setup

setup(
    name="dart",
    version="0.1.0",
    description="DART core logic (scoring + decider + decay). Pure Python, ROS-agnostic.",
    author="chaiyapruk",
    author_email="chaiyapruk6494@gmail.com",
    license="MIT",
    python_requires=">=3.8",
    packages=find_packages(exclude=["sim.tests", "tests"]),
    install_requires=["pyyaml"],
    include_package_data=True,
    package_data={"dart": ["config.yaml"]},
    entry_points={
        "console_scripts": [
            # Headless sim comparison (DART vs nearest-first), unchanged from upstream.
            "dart-sim = dart.sim.run_comparison:main",
        ],
    },
)
