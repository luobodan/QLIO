# QLIO
QLIO: Quantized LiDAR-Inertial Odometry
# QLIO: Quantized LiDAR-Inertial Odometry 
<p align="center">
  <img src="docs/teaser.gif" width="80%">
</p>

## 🚀 Key Features
- **Distributed Quantized Architecture**
  - Dual-processor pipeline with host-co-processor collaboration
  - 14.1× residual data compression through quantization
- **Adaptive Resampling**
  - rQ-vector-based feature selection (85% redundancy reduction)
  - Bandwidth-aware point cloud downsampling
- **QMAP**
  -Quantized based MAP state estimation


## 📦 Installation
```bash
# Clone repository
git clone https://github.com/luobodan/QLIO.git

# Build with catkin
mkdir -p qlio_ws/src && cd qlio_ws/src
catkin_make
source devel/setup.bash
