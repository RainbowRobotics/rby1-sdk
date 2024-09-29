# rby1-sdk
![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)

## Prerequisites

- **CMake**: Version 3.30 or higher  
  Install using snap:
  ```bash
  snap install cmake
  ```

- **Conan**: C++ package manager required for dependency management  
  Install using pip:
  ```bash
  pip install conan
  ```

- **Poetry**: Python dependency management and packaging tool  
  Install using curl:
  ```bash
  curl -sSL https://install.python-poetry.org | python3 -
  source ~/.profile
  ```

## Installation

Clone the repository with submodules:

```bash
git clone --recurse-submodules git@github.com:RainbowRobotics/rby1-sdk.git
```

### Configure

Configure Conan dependencies and build settings:

```bash
conan install . -s build_type=Release -b missing -of build
```

### Build

Build the project using CMake:

```bash
cmake --preset conan-release -D BUILD_EXAMPLES=ON
cmake --build --preset conan-release
```

### Usage

Run the examples:

- **C++ Example**:
  ```bash
  ./build/examples/example_demo_motion [RPC IP]:50051
  ```

- **Python Example**:
  ```bash
  poetry shell
  python examples_python/demo_motion.py 
  ```

## ARM Intellisense Issue

For issues with ARM Intellisense, refer to the following [GitHub Issue](https://github.com/microsoft/vscode-cpptools/issues/7413).

To fix the Intellisense problem, add the following code at the top of your source file:

```c++
// Add this at the top of your source file
#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif
```
## Resources and Support
- Documentation: [https://rainbowrobotics.github.io/rby1-dev/](https://rainbowrobotics.github.io/rby1-dev/)
- Official Website: [Rainbow Robotics](https://www.rainbowrobotics.com/rby1eng)
- Official Support Email: rby.support@rainbow-robotics.com
- GitHub Discussions: [Join the Discussion](https://github.com/RainbowRobotics/rby1-sdk/discussions)
