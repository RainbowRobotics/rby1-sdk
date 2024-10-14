# Data Collector

## Configuration
- RBY1
  - Model: A
  - Camera:
    - D405 x 2 (End-effector)
    - D435 (Head)
- Master Device

## Prerequisites

1. [HDF5](https://support.hdfgroup.org/downloads/hdf5/hdf5_1_14_5.html)
    - [HighFive](https://github.com/BlueBrain/HighFive)
2. OpenCV
3. ZeroMQ
    - [libzmq](https://github.com/zeromq/libzmq)
    - [cppzmq](https://github.com/zeromq/cppzmq)
4. JSON
    - [nlohmann_json](https://github.com/nlohmann/json)
5. (Optional) [Realsense](https://github.com/IntelRealSense/librealsense)


```bash
sudo apt install libhdf5-dev libopencv-dev libzmq3-dev nlohmann-json3-dev libboost-system-dev libboost-serialization-dev libboost-filesystem-dev

# cppzmq
wget https://github.com/zeromq/cppzmq/archive/refs/tags/v4.10.0.tar.gz
tar -xvf v4.10.0.tar.gz
cd cppzmq-4.10.0 && mkdir build && cd build && cmake -D CMAKE_BUILD_TYPE=Release .. && sudo make install -j 4 

# HighFive
wget https://github.com/BlueBrain/HighFive/archive/refs/tags/v2.10.0.tar.gz
tar -xvf v2.10.0.tar.gz
cd HighFive-2.10.0 && mkdir build && cd build && cmake -D CMAKE_BUILD_TYPE=Release .. && sudo make install -j 4 

# Realsense
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update
sudo apt-get install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg
```

## Step

1. Run ``data-collector`` program in UPC.

```bash
data-collector 192.168.30.1:50051 ./
# data-collector <rpc address> <save file directory>
```