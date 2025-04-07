<div align="center">
  <h1>QLIO</h1>
  <h2>Quantized LiDAR-Inertial Odometry</h2>
  <p><strong>This work introduces a bandwidth-efficient, quantization-based LIO framework.</strong></p>
  <br>

  [![Code](https://img.shields.io/badge/Code-GitHub-black?logo=github)](https://github.com/luobodan/QLIO)
  [![arXiv](https://img.shields.io/badge/arXiv-2503.07949-b31b1b.svg)](https://arxiv.org/abs/2503.07949)
  [![YouTube](https://img.shields.io/badge/YouTube-FF0000?logo=youtube&logoColor=white)](https://youtu.be/t2pN2poO3hc)
</div>

<p align="center">
  <img src="https://github.com/snakehaihai/QLIO/blob/main/Figure/Motivation.png?raw=true" width="80%">
</p>

<p align="center">
  <img src="https://github.com/snakehaihai/QLIO/blob/main/Figure/ezgif-8e43eb40ffdc95.gif?raw=true" width="70%">
</p>

---

## 🚀 Key Features

- **Distributed Quantized Architecture**
  - Dual-processor pipeline with host-co-processor collaboration
  - 14.1× residual data compression through quantization

- **Adaptive Resampling**
  - rQ-vector-based feature selection (85% redundancy reduction)
  - Bandwidth-aware point cloud downsampling

- **QMAP**
  - Quantized-based MAP state estimation

---

## 📦 Installation

```bash
# Clone repository
git clone https://github.com/luobodan/QLIO.git

# Build with catkin
mkdir -p qlio_ws/src && cd qlio_ws/src
catkin_make
source devel/setup.bash
