# A bearing-angle approach for unknown target motion analysis based on visual measurements

- arxiv: https://arxiv.org/abs/2401.17117
- Sage：https://journals.sagepub.com/doi/10.1177/02783649241229172
- DOI：https://doi.org/10.1177/0278364924122917
- Youtube: https://www.youtube.com/watch?v=EMQXMJ3_M9Y
- Bilibili: https://www.bilibili.com/video/BV1EC411z7Lz/?spm_id_from=333.999.0.0


This is the source code for our **IJRR** paper "A bearing-angle approach for unknown target motion analysis based on visual measurements". This file contains five numerical simulation experiments conresponding to Fig. 4 and Fig. 5 in the paper.


## Illustration of bearing and angle measurements
![image](https://github.com/ningzian/Bearing-angle/assets/19403501/aae88b28-0472-4be7-8e4e-35aa3d4683c0)
An observer MAV observes a target MAV with a monocular camera. The bearing g and angle θ can be obtained from the bounding box that surrounds the target in the image.

## Running Experiments
Simply run the 'main.m' script in Matlab 2021a.
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
