# 🦾 my_online_ik — Real-Time OpenSim Pipeline

This repository contains the **real-time inverse kinematics pipeline** based on OpenSim and RTOSIM, customized by **Mathis D’Haene** for upper-limb motion tracking and biomechanical analysis.

---

## 📋 1. System Requirements

Tested on **Ubuntu 24.04 LTS** with:

- GCC 11 / G++ 11  
- CMake ≥ 3.20  
- Eigen 3  
- Boost ≥ 1.65  
- LAPACK / BLAS  
- OpenSceneGraph  
- Qt5 OpenGL

Install all dependencies:

```bash
sudo apt update
sudo apt install -y cmake build-essential git \
  libboost-all-dev libeigen3-dev libtbb-dev \
  liblapack-dev libblas-dev libxml2-dev \
  libopenscenegraph-dev libqt5opengl5-dev
````

---

## ⚙️ 2. Build Simbody 3.7 (required for OpenSim 4.3)

RTOSIM and OpenSim 4.3 require **Simbody 3.7**.
Later releases (≥ 3.8 / 3.9) are incompatible with OpenSim 4.3, so make sure to check out the correct tag.

```bash
# Clone and enter Simbody
git clone https://github.com/simbody/simbody.git ~/simbody
cd ~/simbody

# Checkout the version used by OpenSim 4.3
git checkout Simbody-3.7

# Create and enter build directory
mkdir build && cd build

# Configure and compile
CC=gcc-11 CXX=g++-11 cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$HOME/simbody-install

make -j$(nproc)
make install
```

After installation, you should see:

```
~/simbody-install/
├── include/SimTK/
├── lib/
│   └── cmake/simbody/SimbodyConfig.cmake
└── bin/
```

### 🔍 Verify Simbody version

Run:

```bash
grep SIMBODY_VERSION_MAJOR ~/simbody-install/include/SimTKcommon/internal/common.h | head -n 1
```

Expected output:

```
#define SIMBODY_VERSION_MAJOR 3
#define SIMBODY_VERSION_MINOR 7
```

Alternatively, check with:

```bash
strings ~/simbody-install/lib/libSimTKsimbody.so | grep "Simbody"
```

You should see something like `Simbody-3.7`.

---

## 🧠 3. Build my_online_ik + RTOSIM

Once Simbody is installed, simply run the provided build script.
This script automatically builds all dependencies (Concurrency, RTOSIM, my_online_ik) and creates a temporary *fake Filter* to bypass unfinished components.

```bash
cd ~/my_online_ik
bash scripts/build_all.sh | tee build_log.txt
```

This will:

1. Check that **OpenSim 4.3** and **Simbody 3.7** are installed.
2. Build and install **Concurrency**.
3. Create a temporary **fake Filter** package.
4. Build and install **RTOSIM** inside `my_online_ik/RTOSIM/install`.
5. Build your project (**my_online_ik**) and produce the executable `online_ik_test`.

Expected terminal output:

```
[info] Checking required dependencies...
[ok] Concurrency installed at /home/user/concurrency-install
[ok] Fake Filter ready at /home/user/filter-install
[ok] RTOSIM installed.
[success] my_online_ik built successfully 🎉
```

---

## 📂 4. Typical Project Structure

```
my_online_ik/
├── CMakeLists.txt
├── main.cpp
├── scripts/
│   └── build_all.sh
├── RTOSIM/
│   ├── lib/
│   ├── data/
│   └── install/
├── concurrency-install/
├── filter-install/
└── build/
    └── online_ik_test
```

---

## 🧪 5. Running a Test

You can now launch the executable:

```bash
cd ~/my_online_ik/build
./online_ik_test
```

You should see output similar to:

```
[info] Updating Model file from 30000 to latest format...
[error] Object::newInstanceOfType(): object type 'Schutte1993Muscle_Deprecated' is not a registered Object! It will be ignored.
```

✅ This means OpenSim and RTOSIM are running correctly and loading the model.

---

## 🧱 6. Troubleshooting

| Error                              | Cause                           | Fix                                                                                                         |
| ---------------------------------- | ------------------------------- | ----------------------------------------------------------------------------------------------------------- |
| `libopenblas.so.0 not found`       | LAPACK/BLAS not installed       | `sudo apt install libopenblas-dev`                                                                          |
| `spdlog missing from command line` | OpenSim’s spdlog not linked     | Ensure `find_library(SPDLOG_LIB spdlog PATHS $HOME/opensim-core-install/lib)` and link it in CMakeLists.txt |
| `gcc version mismatch`             | RTOSIM fails with GCC 13        | Force build with GCC 11: `CC=gcc-11 CXX=g++-11`                                                             |
| `Filter not found`                 | Filter library not yet compiled | The build script creates a temporary fake Filter; ignore this until later.                                  |



