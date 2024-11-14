from setuptools import find_packages, setup

package_name = "foundation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="hiroto125takeuchi@gmail.com",
    description="Main low-level control node",
    license="My license",
    # tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "foundation = foundation:main",
        ],
    },
)
