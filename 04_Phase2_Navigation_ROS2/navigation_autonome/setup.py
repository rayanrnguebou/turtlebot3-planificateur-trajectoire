from glob import glob

from setuptools import find_packages, setup

package_name = "navigation_autonome"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Rayan",
    maintainer_email="rayanrnguebou@users.noreply.github.com",
    description="Planificateur de trajectoire et suivi pour TurtleBot3 Burger",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "planificateur = navigation_autonome.noeud_planificateur:main",
            "suivi = navigation_autonome.noeud_suivi:main",
        ],
    },
)
