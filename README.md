<!--
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2025-04-07 22:34:58
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2025-04-07 22:48:34
 * @FilePath: /QLIO_comit/QLIO/README.md
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
-->
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
Very thanks for Fastlio!
please download the MCD-ntu dataset and then:
```bash
mkdir catkin_ws/src
cd catkin_ws/src
# Clone repository
git clone https://github.com/luobodan/QLIO.git
# Build with catkin
cd ../
catkin_make
source devel/setup.bash
roslaunch fastlio mapping_ouster128_MCD_ntu.launch
rosrun your_dataset_path/*.bag