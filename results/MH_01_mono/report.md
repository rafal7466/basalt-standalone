# Basalt-Win VIO - EuRoC MH_01_mono

Generated 2026-11-11 13:11

Dataset: `dataset\MH_01_easy`  
Frames: 3682  
Keyframes created: 472  
Build: MSVC 1944, Release, OpenMP, Eigen 5.0.1-dev, OpenCV 4.12.0, Windows x64

## Accuracy

| metric | RMSE | mean | median | std | min | max | N |
|---|---|---|---|---|---|---|---|
| ATE SE3 [m] | 0.0936 | 0.0859 | 0.0903 | 0.0371 | 0.0050 | 0.1554 | 3638 |
| ATE Sim3 [m] | 0.0742 | 0.0663 | 0.0618 | 0.0334 | 0.0103 | 0.2106 | 3638 |
| RPE trans 1 s [m] | 0.0149 | 0.0116 | 0.0097 | 0.0094 | 0.0000 | 0.0614 | 3618 |
| RPE rot 1 s [deg] | 0.0943 | 0.0801 | 0.0718 | 0.0498 | 0.0033 | 0.3907 | 3618 |

Sim3 scale: 1.0134  
GT path length: 80.51 m, duration 181.8 s  
Reference: original Basalt VIO reports 0.07 m RMS ATE on MH_01 (Usenko et al., RA-L 2020)

## Runtime (ms per stereo frame)

| stage | mean | median | max |
|---|---|---|---|
| optical flow | 3.06 | 2.65 | 28.10 |
| VIO | 4.68 | 4.33 | 17.93 |
| total | 7.74 | 7.74 | 28.26 |

Wall time: 39.9 s
