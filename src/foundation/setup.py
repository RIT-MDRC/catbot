from setuptools import find_packages, setup

package_name = "foundation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("lib/python3.10/site-packages/foundation", ["foundation/pinconfig.json"]),
        (
            "lib/python3.12/site-packages/foundation/component/motor",
            ["foundation/component/motor/odrive-cansimple.dbc"],
        ),
    ],
    install_requires=["setuptools", "dotenv"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="hiroto125takeuchi@gmail.com",
    description="Main low-level control node",
    license="My license",
    # tests_require=["pytest"],
    prefix=[
        'sudo -E env "PYTHONPATH=$PYTHONPATH" "LD_LIBRARY_PATH=$LD_LIBRARY_PATH" "PATH=$PATH" "USER=$USER"  bash -c '
    ],
    entry_points={
        "console_scripts": ["main = foundation.main:main"],
    },
)
