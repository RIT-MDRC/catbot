from setuptools import find_packages, setup

package_name = "ui"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("lib/python3.10/site-packages/ui", ["ui/index.tcss"]),
    ],
    install_requires=["setuptools", "dotenv"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="hiroto125takeuchi@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    # tests_require=["pytest"],
    entry_points={
        "console_scripts": ["main = ui.main:main"],
    },
)
