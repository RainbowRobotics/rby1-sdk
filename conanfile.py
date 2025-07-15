from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMakeDeps, CMake, cmake_layout
from conan.tools.build import cross_building, valid_min_cppstd, check_min_cppstd
import re


def extract_version_from_pyproject_toml(path="pyproject.toml"):
    with open(path, "r", encoding="utf-8") as f:
        content = f.read()
    match = re.search(r'^version[\t\r\n ]*=[\t\r\n ]*"([^"]+)"', content, re.MULTILINE)
    if match:
        return match.group(1)
    raise RuntimeError("version not found")


class rby1_sdkRecipe(ConanFile):
    name = "rby1-sdk"
    version = extract_version_from_pyproject_toml()

    # Optional metadata
    license = "Rainbow Robotics"
    author = "Keunjun Choi <keunjun.choi@rainbow-robotics.com>"
    url = "https://www.rainbow-robotics.com/"
    description = "RB-Y1 SDK"
    topics = ("rainbow-robotics")

    generators = "CMakeDeps", "CMakeToolchain"

    # Binary configuration
    settings = "os", "compiler", "build_type", "arch"
    options = {"shared": [True, False], "fPIC": [True, False]}
    default_options = {"shared": False, "fPIC": True}

    # Sources are located in the same place as this recipe, copy them to the recipe
    exports_sources = ["CMakeLists.txt", "src/", "protos/", "include/", "generated/", "examples/"]

    @property
    def _cxxstd_required(self):
        return 17

    def validate(self):
        if self.settings.compiler.get_safe("cppstd"):
            check_min_cppstd(self, self._cxxstd_required)

    def requirements(self):
        self.requires("grpc/1.72.0")
        self.requires("eigen/3.4.0")
        self.requires("tinyxml2/10.0.0", visible=False)
        self.requires("nlohmann_json/3.11.3")

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def package_info(self):
        self.cpp_info.libs = ["rby1-sdk"]
        self.cpp_info.includedirs = ['include']
