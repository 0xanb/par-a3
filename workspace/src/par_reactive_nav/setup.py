from setuptools import setup

package_name = "par_reactive_nav"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="PAR Group",
    maintainer_email="student@rmit.edu.au",
    description="Project C — reactive navigation.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "perception_fusion = par_reactive_nav.perception_fusion:main",
            "vfh_planner = par_reactive_nav.vfh_planner:main",
            "recovery_controller = par_reactive_nav.recovery_controller:main",
            "nd_planner = par_reactive_nav.nd_planner:main",
        ],
    },
)
