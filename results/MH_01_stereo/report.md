# Basalt-Win VIO - EuRoC MH_01_stereo

Generated 2026-11-11 13:09

Dataset: `dataset\MH_01_easy`  
Frames: 3682  
Keyframes created: 461  
Build: MSVC 1944, Release, OpenMP, Eigen 5.0.1-dev, OpenCV 4.12.0, Windows x64

## Accuracy

| metric | RMSE | mean | median | std | min | max | N |
|---|---|---|---|---|---|---|---|
| ATE SE3 [m] | 0.0760 | 0.0722 | 0.0739 | 0.0234 | 0.0266 | 0.1188 | 3638 |
| ATE Sim3 [m] | 0.0276 | 0.0238 | 0.0207 | 0.0140 | 0.0012 | 0.0826 | 3638 |
| RPE trans 1 s [m] | 0.0124 | 0.0101 | 0.0092 | 0.0073 | 0.0000 | 0.0392 | 3618 |
| RPE rot 1 s [deg] | 0.1020 | 0.0814 | 0.0697 | 0.0615 | 0.0070 | 0.6577 | 3618 |

Sim3 scale: 1.0167  
GT path length: 80.51 m, duration 181.8 s  
Reference: original Basalt VIO reports 0.07 m RMS ATE on MH_01 (Usenko et al., RA-L 2020)

## Runtime (ms per stereo frame)

| stage | mean | median | max |
|---|---|---|---|
| optical flow | 4.07 | 3.72 | 124.53 |
| VIO | 5.98 | 5.66 | 21.56 |
| total | 10.04 | 10.15 | 132.69 |

Wall time: 48.4 s
