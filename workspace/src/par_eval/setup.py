from setuptools import setup

package_name = "par_eval"

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
 description="Evaluation + report generation.",
 license="MIT",
 tests_require=["pytest"],
 entry_points={
 "console_scripts": [
 "recorder = par_eval.recorder:main",
 "report = par_eval.report:main",
 "extract = par_eval.extract:main",
 "session_logger = par_eval.session_logger:main",
 "snapshotter = par_eval.snapshotter:main",
 ],
 },
)
