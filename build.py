"""
Build script.
Reference: https://github.com/tueda/test-poetry-scikit-build
"""

import shutil
import platform
import os
from distutils import log as distutils_log
from pathlib import Path
from typing import Any, Dict

import skbuild_conan
from setuptools import find_packages
import skbuild
import skbuild.constants

__all__ = ("build",)


def build(setup_kwargs: Dict[str, Any]) -> None:
    """Build Conan-based C++ project"""

    arch = platform.machine()
    system = platform.system().lower()
    conan_env = None

    if system == "linux" and arch in ("aarch64", "arm64"):
        conan_env = {
            "CC": os.environ.get("CC", "/usr/bin/aarch64-linux-gnu-gcc")
        }
        # cc_env = os.environ.get("CC", "/usr/bin/aarch64-linux-gnu-gcc")
        # cxx_env = "/usr/bin/aarch64-linux-gnu-g++"

    skbuild_conan.setup(
        **setup_kwargs,
        script_args=["build_ext"],
        conan_env=conan_env,
    )

    src_dir = Path(skbuild.constants.CMAKE_INSTALL_DIR()) / "python" / "rby1_sdk"
    dest_dir = Path("rby1_sdk")

    # Delete C-extensions copied in previous runs, just in case.
    remove_files(dest_dir, "**/*.py")
    remove_files(dest_dir, "**/*.pyd")
    remove_files(dest_dir, "**/*.so")
    remove_files(dest_dir, "**/*.pyi")

    # Copy built C-extensions back to the project.
    copy_files(src_dir, dest_dir, "**/*.py")
    copy_files(src_dir, dest_dir, "**/*.pyd")
    copy_files(src_dir, dest_dir, "**/*.so")
    copy_files(src_dir, dest_dir, "**/*.pyi")


def remove_files(target_dir: Path, pattern: str) -> None:
    """Delete files matched with a glob pattern in a directory tree."""
    for path in target_dir.glob(pattern):
        if path.is_dir():
            shutil.rmtree(path)
        else:
            path.unlink()
        distutils_log.info(f"removed {path}")


def copy_files(src_dir: Path, dest_dir: Path, pattern: str) -> None:
    """Copy files matched with a glob pattern in a directory tree to another."""
    for src in src_dir.glob(pattern):
        dest = dest_dir / src.relative_to(src_dir)
        if src.is_dir():
            # NOTE: inefficient if subdirectories also match to the pattern.
            copy_files(src, dest, "*")
        else:
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, dest)
            distutils_log.info(f"copied {src} to {dest}")


if __name__ == "__main__":
    build(
        {
            "packages": find_packages("python"),
            "package_dir": {"": "python"},
            "python_requires": ">=3.10",
            "install_requires": [],
            "cmake_args": ["-DBUILD_PYTHON_BINDINGS=ON"],
            "conan_profile_settings": {"compiler.cppstd": 17},
            "cmake_minimum_required_version": "3.28",
        }
    )
