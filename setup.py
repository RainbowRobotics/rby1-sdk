from skbuild_conan import setup
from setuptools import find_packages

setup(  # https://scikit-build.readthedocs.io/en/latest/usage.html#setup-options
    name="rby1_sdk",
    version="0.0.1",
    packages=find_packages("python"),  # Include all packages in `./src`.
    package_dir={"": "python"},  # The root for our python package is in `./src`.
    python_requires=">=3.10, <3.13",  # lowest python version supported.
    install_requires=[],  # Python Dependencies
    cmake_args=["-DBUILD_PYTHON_BINDINGS=ON"],
    conan_profile_settings={"compiler.cppstd": 17},
    # conan_requirements=["fmt/[>=10.0.0]"],  # C++ Dependencies
    cmake_minimum_required_version="3.28",
)