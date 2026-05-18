from setuptools import setup

package_name = "par_anomaly"

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
 description="IMU-based tilt + collision + wheel-stall anomaly detection.",
 license="MIT",
 tests_require=["pytest"],
 entry_points={
 "console_scripts": [
 "anomaly_detector = par_anomaly.anomaly_detector:main",
 ],
 },
)
