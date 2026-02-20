# 🔧 Build & Installation

This project depends on **OpenSim 4.3**, **Simbody**, and **RTOSIM**.
Because OpenSim does **not** ship reliable system packages, it must be built from source.

The steps below were tested on **Ubuntu 22.04 / 24.04**.

---

## 0. System Requirements

* Ubuntu 22.04 or newer
* GCC ≥ 11
* CMake ≥ 3.22
* Python ≥ 3.8 (for OpenSim Python bindings, optional)

---

## 1. Install System Dependencies

```bash
sudo apt-get update
sudo apt-get install --yes \
  build-essential \
  libtool autoconf pkg-config gfortran \
  libopenblas-dev liblapack-dev \
  freeglut3-dev libxi-dev libxmu-dev \
  doxygen \
  python3 python3-dev python3-numpy python3-setuptools
```

---

## 2. Build and Install SWIG (required by OpenSim)

OpenSim requires a **recent SWIG**, newer than what Ubuntu provides.

```bash
mkdir -p ~/swig-release && cd ~/swig-release

wget https://sourceforge.net/projects/swig/files/swig/swig-4.1.1/swig-4.1.1.tar.gz/download -O swig-4.1.1.tar.gz
tar xzf swig-4.1.1.tar.gz
cd swig-4.1.1

./configure --prefix="$HOME/swig"
make -j"$(nproc)"
make install
```

---

## 3. Download OpenSim 4.3 Source Code

```bash
cd ~
git clone https://github.com/opensim-org/opensim-core.git
cd opensim-core
git checkout 4.3
```

---

## 4. Build OpenSim Dependencies (Superbuild)

This builds **Simbody**, **ezc3d**, and other required libraries.

```bash
mkdir -p ~/build_opensim_deps
cd ~/build_opensim_deps

cmake ../opensim-core/dependencies \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$HOME/opensim_dependencies_install" \
  -DSUPERBUILD_ezc3d=ON \
  -DOPENSIM_WITH_CASADI=ON

make -j$(nproc)
```

---

## 5. Build and Install OpenSim

```bash
mkdir -p ~/build_opensim
cd ~/build_opensim

cmake ../opensim-core \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$HOME/opensim-core-install" \
  -DOPENSIM_DEPENDENCIES_DIR="$HOME/opensim_dependencies_install" \
  -DOPENSIM_C3D_PARSER=ezc3d \
  -DBUILD_PYTHON_WRAPPING=ON \
  -DBUILD_JAVA_WRAPPING=OFF \
  -DSWIG_DIR="$HOME/swig/share/swig" \
  -DSWIG_EXECUTABLE="$HOME/swig/bin/swig" \
  -DOPENSIM_WITH_TROPTER=OFF \
  -DBUILD_TESTING=OFF

make -j$(nproc)
make install
```

### ⚠️ Python bindings fix (known OpenSim issue)

```bash
mkdir -p ~/opensim-core-install/sdk/Python
cp -r ~/build_opensim/Bindings/Python/Release/opensim \
      ~/opensim-core-install/sdk/Python/
cp ~/build_opensim/Bindings/Python/version.py \
   ~/opensim-core-install/sdk/Python/
```

---

## 6. Environment Setup

This project provides a **safe environment setup script**:

```bash
source scripts/env.sh
```

To verify:

```bash
source scripts/env.sh --print
```

Expected output includes:

* `OPENSIM_PREFIX`
* `SIMBODY_PREFIX`
* `LD_LIBRARY_PATH`
* `PATH`

⚠️ **Important**
Do **not** manually export OpenSim paths.
Always use `env.sh`.

---

## 7. Build my_online_ik + RTOSIM

From the project root:

```bash
./scripts/build_all.sh
```

This script builds, in order:

1. Concurrency (RTOSIM dependency)
2. Filter (RTOSIM dependency)
3. RTOSIM
4. `my_online_ik`

All build artifacts are excluded from git.

---

## 8. Run

```bash
source scripts/env.sh
./build/online_ik_test data/upperlimb-biorob.osim
```

To disable the Simbody visualizer:

```bash
./build/online_ik_test --no-viz data/upperlimb-biorob.osim
```

---

## 9. Notes on Reproducibility

* No binary artifacts are committed
* OpenSim is built from pinned version `4.3`
* RTOSIM is included under **Apache License 2.0**
* Build is deterministic given the same compiler toolchain

---

## 10. Troubleshooting

### ❌ `libSimTKcommon.so.X not found`

```bash
source scripts/env.sh
```

You forgot to load the environment.

---

### ❌ Terminal closes after sourcing env.sh

You are using an **old version** of `env.sh`.
Pull latest version and retry.

---

## License

* **my_online_ik**: Apache License 2.0
* **RTOSIM**: Apache License 2.0
* **OpenSim**: Apache License 2.0


