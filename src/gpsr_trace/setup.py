from setuptools import find_packages, setup


PACKAGE_NAME = "gpsr_trace"


setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Tinker Team",
    maintainer_email="cindy.w0135@gmail.com",
    description="Stdlib-only GPSR trace transport and declarative behavior-tree IR.",
    license="Apache-2.0",
    tests_require=["pytest"],
)
