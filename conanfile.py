from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMakeDeps, CMake, cmake_layout


class rby1_sdkRecipe(ConanFile):
    name = "rby1-sdk"
    version = "0.1"

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

    def requirements(self):
        self.requires("grpc/1.54.3")
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
