<div align="center">
  <h1>A bearing-angle approach for unknown target motion analysis based on visual measurements</h1>
<p align="center">
  <a href="https://arxiv.org/abs/2401.17117">
    <img src="https://img.shields.io/badge/arXiv-paper?style=socia&logo=arxiv&logoColor=white&labelColor=grey&color=blue"></a>
  <a href="https://arxiv.org/pdf/2401.17117.pdf">
    <img src="https://img.shields.io/badge/Paper-blue?logo=googledocs&logoColor=white&labelColor=grey&color=blue"></a>
  <a href="https://journals.sagepub.com/doi/10.1177/02783649241229172">
    <img src="https://img.shields.io/badge/sage-blue?logo=sage&logoColor=white&labelColor=grey&color=blue"></a>
  <a href="https://doi.org/10.1177/0278364924122917">
    <img src="https://img.shields.io/badge/DOI-blue?logo=doi&logoColor=white&labelColor=grey&color=blue"></a>
  <a href="https://www.youtube.com/watch?v=EMQXMJ3_M9Y">
    <img src="https://img.shields.io/badge/Video-blue?logo=youtube&logoColor=white&labelColor=grey&color=blue"></a>
  <a href="https://www.bilibili.com/video/BV1EC411z7Lz/?spm_id_from=333.999.0.0">
    <img src="https://img.shields.io/badge/Video-blue?logo=bilibili&logoColor=white&labelColor=grey&color=blue"></a>
  <a href="https://opensource.org/licenses/MIT">
    <img src="https://img.shields.io/badge/License-MIT-yellow.svg"></a>
</p>
  
  [![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/EMQXMJ3_M9Y/0.jpg)](https://www.youtube.com/watch?v=EMQXMJ3_M9Y)
  
</div>
  
This is the source code for our **IJRR** paper "A bearing-angle approach for unknown target motion analysis based on visual measurements". This file contains five numerical simulation experiments conresponding to Fig. 4 and Fig. 5 in the paper.


## Illustration of bearing and angle measurements
![image](https://github.com/ningzian/Bearing-angle/assets/19403501/aae88b28-0472-4be7-8e4e-35aa3d4683c0)
An observer MAV observes a target MAV with a monocular camera. The bearing g and angle θ can be obtained from the bounding box that surrounds the target in the image.

## Running Experiments
Simply run the 'main.m' script using Matlab 2021a.
- The source code has been tested with MATLAB 2021a.
- Each folder corresponds to one numerical simulation experiments.
- The folder's name indicates which experiment it corresponds to.

## Notice
- The simulation results may vary slightly between each run due to the random collection of measurement noises.
- The .mat files in "Fig5a_square_circular" and "Fig5b_square_straight" are the original data used in the paper.
- If you wish to reproduce the exact curves presented in the paper, please modify the variable of mat file name in the 'plot_my.m' script. For example:
![image](https://github.com/ningzian/Bearing-angle/assets/19403501/c5c8bdda-44e9-4a8d-aa74-80060ffc7b69)

## Citing
If you find our work useful, please consider citing:
```latex
@Article{Ning2024bearingAngle,
  author    = {Zian Ning and Yin Zhang and Jianan Li and Zhang Chen and Shiyu Zhao},
  journal   = {The International Journal of Robotics Research},
  title     = {A bearing-angle approach for unknown target motion analysis based on visual measurements},
  year      = {2024},
  month     = feb,
  pages     = {1-20},
  doi       = {10.1177/02783649241229172},
  publisher = {SAGE Publications},
}
```
