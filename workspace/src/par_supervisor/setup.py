from setuptools import setup

package_name = "par_supervisor"

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
    description="2-mode supervisor for the cold-boot self-validate, 360 announce, button-driven mode switch flow.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "supervisor = par_supervisor.supervisor:main",
        ],
    },
)
