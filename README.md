<div align="center">
  <h1>QLIO</h1>
  <h2>Quantized LiDAR-Inertial Odometry</h2>
  <p><strong>This work introduces a bandwidth-efficient, quantization-based LIO framework.</strong></p>
  <br>

  [![Code](https://img.shields.io/badge/Code-GitHub-black?logo=github)](https://github.com/luobodan/QLIO)
  <!-- 可添加 arXiv / YouTube / Bilibili 徽章 -->
</div>

<p align="center">
  <img src="docs/teaser.gif" width="80%">
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
