from glob import glob
import os

from setuptools import find_packages, setup


package_name = "falcon_contact"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "docs"), glob("docs/*.md")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="falcon team",
    maintainer_email="user@example.com",
    description="Independent monocular soft-target contact controller.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "search_contact_node = falcon_contact.search_contact_node:main",
        ],
    },
)
