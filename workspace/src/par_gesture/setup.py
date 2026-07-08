from setuptools import setup

package_name = "par_gesture"

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
    description="Project D — hand-gesture control.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gesture_detector = par_gesture.gesture_detector:main",
            "gesture_interpreter = par_gesture.gesture_interpreter:main",
        ],
    },
)
